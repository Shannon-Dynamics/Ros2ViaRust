//! The occupancy grid: one log-odds value per cell.
//!
//! Reference: Thrun, Burgard & Fox, *Probabilistic Robotics*, Chapter 9, Table 9.1
//! (`occupancy_grid_mapping`) and Equation 9.7, the log-odds form of the recursive update.
//! *Probabilistic Robotics via Rust*, Chapter 13.
//!
//! The belief over the map is the product of one Bernoulli per cell,
//! `bel(m) ~ prod_i p(m_i | z_1:t, x_1:t)`. That factorization is an assumption (two cells of
//! one wall are not independent) and it is what makes the update one addition per cell. Each
//! cell stores `l = ln(p / (1 - p))`, so that a new observation adds a constant and the value
//! cannot overflow into probability 0 or 1. The value is clamped to `[l_min, l_max]` so that a
//! cell stays able to change its mind: a wall that is removed must eventually be forgotten,
//! and with no clamp a cell seen occupied a thousand times needs a thousand free
//! observations to flip.
//!
//! Layout is that of `nav_msgs/OccupancyGrid`: row-major with `j` (y) as the slow index, cell
//! `(i, j)` covering `[origin + (i, j) * resolution, origin + (i + 1, j + 1) * resolution)`,
//! which is also the layout of
//! [`robotics_models::sensor::likelihood_field::DistanceGrid`].
//!
//! The grid is a flat `Vec<f32>`. A quadtree or a hash of chunks pays off when the map is
//! large and mostly empty; this room is 6 by 6 m, a dense array of a few tens of thousands of
//! cells, and growing a grid is a different problem from representing one.

use crate::MappingError;
use robotics_core::nalgebra::Vector2;

/// Largest number of cells a grid may have, so that `width * height` cannot overflow and a
/// mistyped resolution fails instead of allocating gigabytes.
pub const MAX_CELLS: usize = 1 << 26;

/// Where the grid is and how fine it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridGeometry {
    /// Cells along x.
    pub width: usize,
    /// Cells along y.
    pub height: usize,
    /// Cell edge length (m).
    pub resolution: f64,
    /// World position of the lower-left corner of cell `(0, 0)` (m).
    pub origin: Vector2<f64>,
}

impl GridGeometry {
    /// Validate the geometry.
    pub fn validate(&self) -> Result<(), MappingError> {
        if self.width == 0 || self.height == 0 {
            return Err(MappingError::InvalidParameter {
                what: "grid size",
                value: 0.0,
            });
        }
        match self.width.checked_mul(self.height) {
            Some(n) if n <= MAX_CELLS => {}
            _ => {
                return Err(MappingError::InvalidParameter {
                    what: "grid cell count",
                    value: self.width as f64 * self.height as f64,
                })
            }
        }
        if !(self.resolution.is_finite() && self.resolution > 0.0) {
            return Err(MappingError::InvalidParameter {
                what: "resolution",
                value: self.resolution,
            });
        }
        if !(self.origin.x.is_finite() && self.origin.y.is_finite()) {
            return Err(MappingError::InvalidParameter {
                what: "origin",
                value: f64::NAN,
            });
        }
        Ok(())
    }

    /// Number of cells.
    pub fn cells(&self) -> usize {
        self.width * self.height
    }

    /// Row-major index of cell `(i, j)`, `None` outside the grid.
    #[inline]
    pub fn index(&self, i: usize, j: usize) -> Option<usize> {
        (i < self.width && j < self.height).then(|| j * self.width + i)
    }

    /// The cell containing a world point, `None` outside the grid or for a non-finite point.
    /// A point exactly on a cell edge belongs to the cell on its positive side.
    #[inline]
    pub fn world_to_cell(&self, p: &Vector2<f64>) -> Option<(usize, usize)> {
        let fx = ((p.x - self.origin.x) / self.resolution).floor();
        let fy = ((p.y - self.origin.y) / self.resolution).floor();
        if !(fx.is_finite() && fy.is_finite()) || fx < 0.0 || fy < 0.0 {
            return None;
        }
        let (i, j) = (fx as usize, fy as usize);
        (i < self.width && j < self.height).then_some((i, j))
    }

    /// World position of the centre of cell `(i, j)`.
    #[inline]
    pub fn cell_to_world(&self, i: usize, j: usize) -> Vector2<f64> {
        Vector2::new(
            self.origin.x + (i as f64 + 0.5) * self.resolution,
            self.origin.y + (j as f64 + 0.5) * self.resolution,
        )
    }
}

/// What a cell is called, given its log-odds and the two thresholds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CellState {
    /// Log-odds at or below the free threshold.
    Free,
    /// Log-odds at or above the occupied threshold.
    Occupied,
    /// Anything between: never observed, or observed too little or too contradictorily.
    Unknown,
}

/// The numbers of the inverse sensor model and of the read-out, all as log-odds.
///
/// These are the defaults the development run of `docs/phase-4-mapping/benchmark.md` left, and
/// they are not touched after it. All but the occupied threshold were chosen by reasoning before
/// any run; that one is the first parameter a run changed (its row says why):
///
/// | quantity | probability | log-odds | why |
/// |---|---|---|---|
/// | `l_occ` | 0.7 | `ln(7/3)` | the usual hit weight (OctoMap's 0.7): one hit is evidence, not proof |
/// | `l_free` | 0.4 | `ln(2/3)` | the usual miss weight (OctoMap's 0.4): a miss is weaker than a hit because the beam may have been blocked by something nearer |
/// | `l_min` | 0.12 | `ln(0.12/0.88)` | OctoMap's clamp: a saturated free cell flips after four hits |
/// | `l_max` | 0.97 | `ln(0.97/0.03)` | OctoMap's clamp: a saturated occupied cell flips after eleven misses |
/// | occupied threshold | 0.75 | `ln 3` | **two hits** make a cell occupied (one hit is 0.7, two are 0.845). Set on the first development run, `evidence/phase-4-mapping/2026-10-11-development-route-long`: with one hit it left 113 cells occupied one and a half cells beyond the walls, each one hit from a pose that was a few centimetres off, and nothing ever crosses a cell outside a wall to clear it |
/// | free threshold | 0.35 | `-ln(65/35)` | two misses are needed, one is not enough |
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OccupancyParams {
    /// Log-odds added to a cell a beam ended in.
    pub l_occ: f32,
    /// Log-odds added to a cell a beam crossed (negative).
    pub l_free: f32,
    /// Lower clamp (negative).
    pub l_min: f32,
    /// Upper clamp (positive).
    pub l_max: f32,
    /// Cells with log-odds at or above this are [`CellState::Occupied`] (positive).
    pub occupied_threshold: f32,
    /// Cells with log-odds at or below this are [`CellState::Free`] (negative).
    pub free_threshold: f32,
}

fn logit(p: f64) -> f32 {
    (p / (1.0 - p)).ln() as f32
}

impl OccupancyParams {
    /// Build from probabilities: the hit and miss weights, the clamps and the two thresholds.
    pub fn from_probabilities(
        p_hit: f64,
        p_miss: f64,
        p_min: f64,
        p_max: f64,
        p_occupied: f64,
        p_free: f64,
    ) -> Self {
        Self {
            l_occ: logit(p_hit),
            l_free: logit(p_miss),
            l_min: logit(p_min),
            l_max: logit(p_max),
            occupied_threshold: logit(p_occupied),
            free_threshold: logit(p_free),
        }
    }

    /// Validate the ordering the grid relies on.
    pub fn validate(&self) -> Result<(), MappingError> {
        let bad = |what, value: f32| {
            Err(MappingError::InvalidParameter {
                what,
                value: value as f64,
            })
        };
        let all = [
            self.l_occ,
            self.l_free,
            self.l_min,
            self.l_max,
            self.occupied_threshold,
            self.free_threshold,
        ];
        if all.iter().any(|v| !v.is_finite()) {
            return bad("log-odds parameter", f32::NAN);
        }
        if self.l_occ <= 0.0 {
            return bad("l_occ", self.l_occ);
        }
        if self.l_free >= 0.0 {
            return bad("l_free", self.l_free);
        }
        if self.free_threshold >= 0.0 {
            return bad("free_threshold", self.free_threshold);
        }
        if self.occupied_threshold <= 0.0 {
            return bad("occupied_threshold", self.occupied_threshold);
        }
        // A cell must be able to reach each state, or the clamp would hide it forever.
        if self.l_min > self.free_threshold {
            return bad("l_min", self.l_min);
        }
        if self.l_max < self.occupied_threshold {
            return bad("l_max", self.l_max);
        }
        Ok(())
    }
}

impl Default for OccupancyParams {
    fn default() -> Self {
        Self::from_probabilities(0.7, 0.4, 0.12, 0.97, 0.75, 0.35)
    }
}

/// A log-odds occupancy grid.
#[derive(Debug, Clone, PartialEq)]
pub struct OccupancyGrid {
    geometry: GridGeometry,
    params: OccupancyParams,
    log_odds: Vec<f32>,
}

impl OccupancyGrid {
    /// A grid of unknown cells (log-odds 0, probability one half).
    pub fn new(geometry: GridGeometry, params: OccupancyParams) -> Result<Self, MappingError> {
        geometry.validate()?;
        params.validate()?;
        Ok(Self {
            geometry,
            params,
            log_odds: vec![0.0; geometry.cells()],
        })
    }

    /// The geometry.
    pub fn geometry(&self) -> &GridGeometry {
        &self.geometry
    }

    /// The model and read-out parameters.
    pub fn params(&self) -> &OccupancyParams {
        &self.params
    }

    /// Cells along x.
    pub fn width(&self) -> usize {
        self.geometry.width
    }

    /// Cells along y.
    pub fn height(&self) -> usize {
        self.geometry.height
    }

    /// Cell edge length (m).
    pub fn resolution(&self) -> f64 {
        self.geometry.resolution
    }

    /// The raw log-odds, row-major with `j` as the slow index.
    pub fn log_odds_slice(&self) -> &[f32] {
        &self.log_odds
    }

    /// Log-odds of cell `(i, j)`; `None` outside the grid, which is not the same as unknown.
    pub fn log_odds(&self, i: usize, j: usize) -> Option<f32> {
        self.geometry.index(i, j).map(|k| self.log_odds[k])
    }

    /// Probability that cell `(i, j)` is occupied.
    pub fn probability(&self, i: usize, j: usize) -> Option<f64> {
        self.log_odds(i, j)
            .map(|l| 1.0 / (1.0 + (-(l as f64)).exp()))
    }

    /// State of a log-odds value under this grid's thresholds.
    #[inline]
    pub fn classify(&self, l: f32) -> CellState {
        if l >= self.params.occupied_threshold {
            CellState::Occupied
        } else if l <= self.params.free_threshold {
            CellState::Free
        } else {
            CellState::Unknown
        }
    }

    /// State of cell `(i, j)`; `None` outside the grid.
    pub fn state(&self, i: usize, j: usize) -> Option<CellState> {
        self.log_odds(i, j).map(|l| self.classify(l))
    }

    /// The cell containing a world point.
    pub fn world_to_cell(&self, p: &Vector2<f64>) -> Option<(usize, usize)> {
        self.geometry.world_to_cell(p)
    }

    /// World position of the centre of cell `(i, j)`.
    pub fn cell_to_world(&self, i: usize, j: usize) -> Vector2<f64> {
        self.geometry.cell_to_world(i, j)
    }

    /// Add `delta` to the cell at row-major `index`, clamped to `[l_min, l_max]`.
    #[inline]
    pub(crate) fn add_at(&mut self, index: usize, delta: f32) {
        let l = &mut self.log_odds[index];
        *l = (*l + delta).clamp(self.params.l_min, self.params.l_max);
    }

    /// Add `delta` to cell `(i, j)`, clamped; returns whether the cell exists.
    pub fn add_evidence(&mut self, i: usize, j: usize, delta: f32) -> bool {
        match self.geometry.index(i, j) {
            Some(k) => {
                self.add_at(k, delta);
                true
            }
            None => false,
        }
    }

    /// Forget everything: every cell back to unknown.
    pub fn reset(&mut self) {
        self.log_odds.iter_mut().for_each(|l| *l = 0.0);
    }

    /// Per-cell states, row-major.
    pub fn states(&self) -> Vec<CellState> {
        self.log_odds.iter().map(|&l| self.classify(l)).collect()
    }

    /// `true` for every occupied cell, row-major: the mask of the distance transform.
    pub fn occupied_mask(&self) -> Vec<bool> {
        self.log_odds
            .iter()
            .map(|&l| self.classify(l) == CellState::Occupied)
            .collect()
    }

    /// Number of `(free, occupied, unknown)` cells.
    pub fn count_states(&self) -> (usize, usize, usize) {
        let (mut free, mut occ, mut unk) = (0, 0, 0);
        for &l in &self.log_odds {
            match self.classify(l) {
                CellState::Free => free += 1,
                CellState::Occupied => occ += 1,
                CellState::Unknown => unk += 1,
            }
        }
        (free, occ, unk)
    }

    /// The values of `nav_msgs/OccupancyGrid.data`: 0 free, 100 occupied, -1 unknown. The three
    /// states only; the probability is deliberately not smeared across 0 to 100, because a
    /// consumer that treats 37 as "a bit occupied" would be reading a number nobody calibrated.
    pub fn to_ros_data(&self) -> Vec<i8> {
        self.log_odds
            .iter()
            .map(|&l| match self.classify(l) {
                CellState::Free => 0,
                CellState::Occupied => 100,
                CellState::Unknown => -1,
            })
            .collect()
    }

    /// A grid with the states of `nav_msgs/OccupancyGrid.data` (0 free, 100 occupied, -1
    /// unknown), as a consumer of a published map sees it. The probabilities are gone, so each
    /// state is given the value that reads back as itself: the lower clamp, the upper clamp, or
    /// zero. Any other value is an error, because this library publishes only those three.
    pub fn from_ros_data(
        geometry: GridGeometry,
        params: OccupancyParams,
        data: &[i8],
    ) -> Result<Self, MappingError> {
        let mut grid = Self::new(geometry, params)?;
        if data.len() != grid.log_odds.len() {
            return Err(MappingError::Parse(format!(
                "{} values for a {} by {} grid",
                data.len(),
                geometry.width,
                geometry.height
            )));
        }
        for (cell, &v) in grid.log_odds.iter_mut().zip(data) {
            *cell = match v {
                0 => params.l_min,
                100 => params.l_max,
                -1 => 0.0,
                other => {
                    return Err(MappingError::Parse(format!(
                        "occupancy value {other} is not 0, 100 or -1"
                    )))
                }
            };
        }
        Ok(grid)
    }

    /// Whether two grids have the same geometry and the same bits in every cell. This is the
    /// determinism check of claim M2: `==` on `f32` would call `0.0` and `-0.0` equal and a
    /// pair of NaNs different, and neither is what "bit-identical" means.
    pub fn bit_identical(&self, other: &OccupancyGrid) -> bool {
        self.geometry == other.geometry
            && self.log_odds.len() == other.log_odds.len()
            && self
                .log_odds
                .iter()
                .zip(&other.log_odds)
                .all(|(a, b)| a.to_bits() == b.to_bits())
    }

    /// A text form that round-trips bit for bit (Rust prints the shortest decimal that parses
    /// back to the same `f32`). One header line per field, then `height` rows of `width`
    /// values, row 0 first.
    pub fn to_text(&self) -> String {
        let g = &self.geometry;
        let p = &self.params;
        let mut s = String::with_capacity(self.log_odds.len() * 8 + 256);
        s.push_str("robotics_mapping grid v1\n");
        s.push_str(&format!("width {}\nheight {}\n", g.width, g.height));
        s.push_str(&format!("resolution {}\n", g.resolution));
        s.push_str(&format!(
            "origin_x {}\norigin_y {}\n",
            g.origin.x, g.origin.y
        ));
        s.push_str(&format!(
            "l_occ {}\nl_free {}\nl_min {}\nl_max {}\noccupied_threshold {}\nfree_threshold {}\n",
            p.l_occ, p.l_free, p.l_min, p.l_max, p.occupied_threshold, p.free_threshold
        ));
        s.push_str("data\n");
        for row in self.log_odds.chunks(g.width) {
            let line: Vec<String> = row.iter().map(|v| v.to_string()).collect();
            s.push_str(&line.join(" "));
            s.push('\n');
        }
        s
    }

    /// Parse the form written by [`OccupancyGrid::to_text`].
    pub fn from_text(text: &str) -> Result<Self, MappingError> {
        let err = |m: String| MappingError::Parse(m);
        let mut lines = text.lines();
        if lines.next() != Some("robotics_mapping grid v1") {
            return Err(err("missing the `robotics_mapping grid v1` header".into()));
        }
        let mut field = |name: &str| -> Result<String, MappingError> {
            let line = lines
                .next()
                .ok_or_else(|| err(format!("missing the {name} line")))?;
            line.strip_prefix(name)
                .and_then(|r| r.strip_prefix(' '))
                .map(|v| v.trim().to_string())
                .ok_or_else(|| err(format!("expected `{name} <value>`, found {line:?}")))
        };
        fn num<T: std::str::FromStr>(name: &str, v: &str) -> Result<T, MappingError> {
            v.parse::<T>()
                .map_err(|_| MappingError::Parse(format!("{name}: {v:?} is not a number")))
        }
        let width: usize = num("width", &field("width")?)?;
        let height: usize = num("height", &field("height")?)?;
        let resolution: f64 = num("resolution", &field("resolution")?)?;
        let ox: f64 = num("origin_x", &field("origin_x")?)?;
        let oy: f64 = num("origin_y", &field("origin_y")?)?;
        let params = OccupancyParams {
            l_occ: num("l_occ", &field("l_occ")?)?,
            l_free: num("l_free", &field("l_free")?)?,
            l_min: num("l_min", &field("l_min")?)?,
            l_max: num("l_max", &field("l_max")?)?,
            occupied_threshold: num("occupied_threshold", &field("occupied_threshold")?)?,
            free_threshold: num("free_threshold", &field("free_threshold")?)?,
        };
        if lines.next() != Some("data") {
            return Err(err("expected the `data` line".into()));
        }
        let geometry = GridGeometry {
            width,
            height,
            resolution,
            origin: Vector2::new(ox, oy),
        };
        let mut grid = OccupancyGrid::new(geometry, params)?;
        let mut count = 0usize;
        for line in lines {
            for tok in line.split_whitespace() {
                if count >= grid.log_odds.len() {
                    return Err(err("more values than width * height".into()));
                }
                grid.log_odds[count] = num::<f32>("cell", tok)?;
                count += 1;
            }
        }
        if count != grid.log_odds.len() {
            return Err(err(format!(
                "{count} values for a {width} by {height} grid"
            )));
        }
        Ok(grid)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn geom() -> GridGeometry {
        GridGeometry {
            width: 4,
            height: 3,
            resolution: 0.5,
            origin: Vector2::new(-1.0, -0.5),
        }
    }

    fn grid() -> OccupancyGrid {
        OccupancyGrid::new(geom(), OccupancyParams::default()).unwrap()
    }

    #[test]
    fn a_new_grid_is_entirely_unknown_at_probability_one_half() {
        let g = grid();
        assert_eq!(g.count_states(), (0, 0, 12));
        assert_eq!(g.probability(2, 1), Some(0.5));
        assert_eq!(g.log_odds(0, 0), Some(0.0));
    }

    /// Worked example, by hand. From the prior one half, a hit of weight 0.7 gives odds 7/3,
    /// so probability 0.7; a second hit gives odds (7/3)^2 = 49/9, so probability
    /// 49/58 = 0.844827586...; two misses of weight 0.4 give odds (2/3)^2 = 4/9, probability
    /// 4/13 = 0.307692307... The log-odds form must reproduce them by addition alone.
    #[test]
    fn worked_example_log_odds_updates_match_the_odds_products() {
        let p = OccupancyParams::default();
        let mut g = grid();
        g.add_evidence(1, 1, p.l_occ);
        assert_relative_eq!(g.probability(1, 1).unwrap(), 0.7, epsilon = 1e-6);
        g.add_evidence(1, 1, p.l_occ);
        assert_relative_eq!(g.probability(1, 1).unwrap(), 49.0 / 58.0, epsilon = 1e-6);

        g.add_evidence(2, 2, p.l_free);
        g.add_evidence(2, 2, p.l_free);
        assert_relative_eq!(g.probability(2, 2).unwrap(), 4.0 / 13.0, epsilon = 1e-6);
    }

    /// Two hits make a cell occupied (the second takes it from 0.7 to 0.845, past the 0.75
    /// threshold) and one is not enough; two misses make it free and one is not enough.
    #[test]
    fn two_hits_make_a_cell_occupied_and_two_misses_make_it_free() {
        let p = OccupancyParams::default();
        let mut g = grid();
        g.add_evidence(0, 0, p.l_occ);
        assert_eq!(g.state(0, 0), Some(CellState::Unknown));
        g.add_evidence(0, 0, p.l_occ);
        assert_eq!(g.state(0, 0), Some(CellState::Occupied));
        g.add_evidence(1, 0, p.l_free);
        assert_eq!(g.state(1, 0), Some(CellState::Unknown));
        g.add_evidence(1, 0, p.l_free);
        assert_eq!(g.state(1, 0), Some(CellState::Free));
    }

    /// The clamp is what lets a cell change its mind: from saturation the documented number
    /// of contradicting observations flips it, and not one fewer.
    #[test]
    fn clamps_bound_the_log_odds_and_set_how_fast_a_cell_flips() {
        let p = OccupancyParams::default();
        let mut g = grid();
        for _ in 0..1000 {
            g.add_evidence(0, 0, p.l_occ);
        }
        assert_eq!(g.log_odds(0, 0), Some(p.l_max));
        let mut misses = 0;
        while g.state(0, 0) != Some(CellState::Free) {
            g.add_evidence(0, 0, p.l_free);
            misses += 1;
            assert!(misses < 100);
        }
        assert_eq!(
            misses, 11,
            "a saturated occupied cell flips after eleven misses"
        );

        for _ in 0..1000 {
            g.add_evidence(1, 0, p.l_free);
        }
        assert_eq!(g.log_odds(1, 0), Some(p.l_min));
        let mut hits = 0;
        while g.state(1, 0) != Some(CellState::Occupied) {
            g.add_evidence(1, 0, p.l_occ);
            hits += 1;
            assert!(hits < 100);
        }
        assert_eq!(hits, 4, "a saturated free cell flips after four hits");
    }

    #[test]
    fn world_cell_round_trip_and_edges() {
        let g = grid();
        // Cell (0, 0) covers [-1, -0.5) x [-0.5, 0).
        assert_eq!(g.world_to_cell(&Vector2::new(-1.0, -0.5)), Some((0, 0)));
        assert_eq!(g.world_to_cell(&Vector2::new(-0.51, -0.01)), Some((0, 0)));
        // A point exactly on an edge belongs to the cell on its positive side.
        assert_eq!(g.world_to_cell(&Vector2::new(-0.5, -0.5)), Some((1, 0)));
        // The far edge of the grid is outside.
        assert_eq!(g.world_to_cell(&Vector2::new(1.0, 0.0)), None);
        assert_eq!(g.world_to_cell(&Vector2::new(0.0, 1.0)), None);
        assert_eq!(g.world_to_cell(&Vector2::new(-1.01, 0.0)), None);
        assert_eq!(g.world_to_cell(&Vector2::new(f64::NAN, 0.0)), None);
        for j in 0..3 {
            for i in 0..4 {
                let c = g.cell_to_world(i, j);
                assert_eq!(g.world_to_cell(&c), Some((i, j)));
            }
        }
        assert_eq!(g.cell_to_world(0, 0), Vector2::new(-0.75, -0.25));
    }

    #[test]
    fn outside_the_grid_is_none_not_unknown() {
        let mut g = grid();
        assert_eq!(g.log_odds(4, 0), None);
        assert_eq!(g.state(0, 3), None);
        assert!(!g.add_evidence(9, 9, 1.0));
    }

    #[test]
    fn ros_data_round_trips_the_states_and_refuses_other_values() {
        let p = OccupancyParams::default();
        let mut g = grid();
        g.add_evidence(0, 0, p.l_occ);
        g.add_evidence(1, 0, p.l_free);
        g.add_evidence(1, 0, p.l_free);
        let back = OccupancyGrid::from_ros_data(geom(), p, &g.to_ros_data()).unwrap();
        assert_eq!(back.states(), g.states());
        assert_eq!(back.to_ros_data(), g.to_ros_data());
        let mut bad = g.to_ros_data();
        bad[3] = 37;
        assert!(OccupancyGrid::from_ros_data(geom(), p, &bad).is_err());
        assert!(OccupancyGrid::from_ros_data(geom(), p, &bad[..5]).is_err());
    }

    #[test]
    fn ros_data_has_exactly_the_three_values() {
        let p = OccupancyParams::default();
        let mut g = grid();
        g.add_evidence(0, 0, p.l_occ);
        g.add_evidence(0, 0, p.l_occ);
        g.add_evidence(1, 0, p.l_free);
        g.add_evidence(1, 0, p.l_free);
        g.add_evidence(2, 0, p.l_free); // one miss: still unknown
        let d = g.to_ros_data();
        assert_eq!(d.len(), 12);
        assert_eq!(&d[0..3], &[100, 0, -1]);
        assert!(d.iter().all(|v| [100, 0, -1].contains(v)));
    }

    #[test]
    fn invalid_parameters_are_refused() {
        let ok = OccupancyParams::default();
        let mut p = ok;
        p.l_free = 0.1;
        assert!(OccupancyGrid::new(geom(), p).is_err());
        let mut p = ok;
        p.l_min = -0.1; // a clamp above the free threshold: no cell could ever be free
        assert!(OccupancyGrid::new(geom(), p).is_err());
        let mut p = ok;
        p.l_max = 0.1;
        assert!(OccupancyGrid::new(geom(), p).is_err());
        let mut bad = geom();
        bad.resolution = 0.0;
        assert!(OccupancyGrid::new(bad, ok).is_err());
        let mut bad = geom();
        bad.width = 0;
        assert!(OccupancyGrid::new(bad, ok).is_err());
        let mut bad = geom();
        bad.width = usize::MAX;
        assert!(OccupancyGrid::new(bad, ok).is_err());
    }

    #[test]
    fn text_round_trip_is_bit_identical() {
        let p = OccupancyParams::default();
        let mut g = grid();
        g.add_evidence(0, 0, p.l_occ);
        g.add_evidence(3, 2, p.l_free);
        g.add_evidence(2, 1, 0.1234567);
        let back = OccupancyGrid::from_text(&g.to_text()).unwrap();
        assert!(g.bit_identical(&back));
        assert_eq!(g, back);
    }

    #[test]
    fn malformed_text_is_an_error_not_a_panic() {
        assert!(OccupancyGrid::from_text("").is_err());
        assert!(OccupancyGrid::from_text("robotics_mapping grid v1\nwidth x\n").is_err());
        let mut t = grid().to_text();
        t.push_str("1.0\n");
        assert!(OccupancyGrid::from_text(&t).is_err());
        let short = grid().to_text().replace("0 0 0 0\n", "0 0 0\n");
        assert!(OccupancyGrid::from_text(&short).is_err());
    }

    #[test]
    fn bit_identical_tells_zero_from_negative_zero() {
        let a = grid();
        let mut b = grid();
        b.log_odds[0] = -0.0;
        assert_eq!(a.log_odds[0], b.log_odds[0]);
        assert!(!a.bit_identical(&b));
    }
}
