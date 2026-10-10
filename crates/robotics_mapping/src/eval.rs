//! The map-accuracy evaluator: a grid graded against the analytic geometry of the world.
//!
//! The world of Phase 3 and 4 is known exactly: an inner square whose faces are at
//! `+/- half_extent`, and cylinders of known radius. The lidar sees *surfaces*, so the ground
//! truth is the set `S` of surfaces a beam can hit, as exact geometry, and not a raster of
//! the wall volume: the wall is as thick as one proposed cell, so a raster would make every
//! metric depend on how the wall happens to straddle the grid.
//!
//! A map is not charged for what it could not have seen. `S_obs` is the part of `S` that an
//! exact visibility test shows to be seen, within range, from at least one pose of the run:
//! the sample is on the side of its surface that faces the pose, and no other surface lies on
//! the segment between them. The fraction set aside is reported ([`MapMetrics::observable_fraction`]),
//! so that nobody can quietly shrink the denominator. "Observable" means *line of sight within
//! range from some pose*, not *a beam hit it*: with 360 beams a surface patch can lie between
//! two beams, and the completeness metric is exactly what shows that.
//!
//! The metrics are those of `docs/phase-4-mapping/benchmark.md` section 2. The evaluator takes
//! a grid and a list of sensor positions and knows nothing about how the grid was made. The
//! tolerance `tau` is an argument with no default, because it is fixed in
//! `scripts/phase4_verdict.py` and nowhere else.

use crate::occupancy::{CellState, OccupancyGrid};
use crate::MappingError;
use robotics_core::nalgebra::Vector2;
use robotics_models::sensor::likelihood_field::{DistanceField, DistanceGrid};

type V2 = Vector2<f64>;

#[derive(Debug, Clone, Copy, PartialEq)]
struct Wall {
    a: V2,
    b: V2,
    /// Unit normal pointing into free space.
    normal: V2,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Pillar {
    c: V2,
    r: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Owner {
    Wall(usize),
    Pillar(usize),
}

/// One point of a ground-truth surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceSample {
    /// Position (m).
    pub p: Vector2<f64>,
    /// Unit normal, pointing into free space.
    pub normal: Vector2<f64>,
    owner: Owner,
}

/// The analytic surfaces of the world.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GroundTruth {
    walls: Vec<Wall>,
    pillars: Vec<Pillar>,
}

fn seg_distance(a: &V2, b: &V2, p: &V2) -> f64 {
    let e = b - a;
    let s = ((p - a).dot(&e) / e.norm_squared()).clamp(0.0, 1.0);
    (a + e * s - p).norm()
}

/// Whether the open segment `p -> q` properly crosses the segment `a -> b`.
fn crosses(p: &V2, q: &V2, a: &V2, b: &V2) -> bool {
    let cross = |u: &V2, v: &V2| u.x * v.y - u.y * v.x;
    let (d, e) = (q - p, b - a);
    let denom = cross(&d, &e);
    if denom.abs() < 1e-15 {
        return false;
    }
    let ap = a - p;
    let t = cross(&ap, &e) / denom;
    let s = cross(&ap, &d) / denom;
    const EPS: f64 = 1e-9;
    t > EPS && t < 1.0 - EPS && (-EPS..=1.0 + EPS).contains(&s)
}

impl GroundTruth {
    /// A rectangular room centred on the origin, `half_x` by `half_y` to its inner faces, and
    /// cylinders given as `(x, y, radius)`.
    pub fn room(
        half_x: f64,
        half_y: f64,
        pillars: &[(f64, f64, f64)],
    ) -> Result<Self, MappingError> {
        for (what, v) in [("half_x", half_x), ("half_y", half_y)] {
            if !(v.is_finite() && v > 0.0) {
                return Err(MappingError::InvalidParameter { what, value: v });
            }
        }
        let v = V2::new;
        let walls = vec![
            Wall {
                a: v(-half_x, -half_y),
                b: v(half_x, -half_y),
                normal: v(0.0, 1.0),
            },
            Wall {
                a: v(half_x, -half_y),
                b: v(half_x, half_y),
                normal: v(-1.0, 0.0),
            },
            Wall {
                a: v(half_x, half_y),
                b: v(-half_x, half_y),
                normal: v(0.0, -1.0),
            },
            Wall {
                a: v(-half_x, half_y),
                b: v(-half_x, -half_y),
                normal: v(1.0, 0.0),
            },
        ];
        let mut ps = Vec::new();
        for &(x, y, r) in pillars {
            if !(x.is_finite() && y.is_finite() && r.is_finite() && r > 0.0) {
                return Err(MappingError::InvalidParameter {
                    what: "pillar",
                    value: r,
                });
            }
            ps.push(Pillar { c: v(x, y), r });
        }
        Ok(Self { walls, pillars: ps })
    }

    /// Read the room and the pillars out of the text of `landmarks.yaml`: the `floor_size`
    /// line (the room is centred on the origin, inner faces at half the floor size) and the
    /// `landmarks:` entries `{id: .., x: .., y: .., radius: ..}`.
    pub fn from_landmarks_yaml(text: &str) -> Result<Self, MappingError> {
        let err = |m: String| MappingError::Parse(m);
        let num = |what: &str, s: &str| -> Result<f64, MappingError> {
            s.trim()
                .parse::<f64>()
                .map_err(|_| err(format!("{what}: {s:?} is not a number")))
        };
        let mut half: Option<(f64, f64)> = None;
        let mut pillars = Vec::new();
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            if let Some(rest) = line.strip_prefix("floor_size:") {
                let inner = rest.trim().trim_start_matches('[').trim_end_matches(']');
                let parts: Vec<&str> = inner.split(',').collect();
                if parts.len() != 2 {
                    return Err(err(format!("floor_size needs two numbers, found {rest:?}")));
                }
                half = Some((
                    num("floor_size", parts[0])? / 2.0,
                    num("floor_size", parts[1])? / 2.0,
                ));
            } else if line.starts_with("- {") && line.contains("radius") {
                let body = line.trim_start_matches("- {").trim_end_matches('}');
                let (mut x, mut y, mut r) = (None, None, None);
                for pair in body.split(',') {
                    if let Some((k, val)) = pair.split_once(':') {
                        match k.trim() {
                            "x" => x = Some(num("x", val)?),
                            "y" => y = Some(num("y", val)?),
                            "radius" => r = Some(num("radius", val)?),
                            _ => {}
                        }
                    }
                }
                match (x, y, r) {
                    (Some(x), Some(y), Some(r)) => pillars.push((x, y, r)),
                    _ => {
                        return Err(err(format!(
                            "landmark line without x, y and radius: {line:?}"
                        )))
                    }
                }
            }
        }
        let (hx, hy) = half.ok_or_else(|| err("no `floor_size:` line".into()))?;
        Self::room(hx, hy, &pillars)
    }

    /// Number of pillars.
    pub fn pillar_count(&self) -> usize {
        self.pillars.len()
    }

    /// Exact distance from `p` to the nearest surface (m).
    pub fn distance_to_surface(&self, p: &Vector2<f64>) -> f64 {
        let w = self
            .walls
            .iter()
            .map(|w| seg_distance(&w.a, &w.b, p))
            .fold(f64::INFINITY, f64::min);
        let c = self
            .pillars
            .iter()
            .map(|c| ((p - c.c).norm() - c.r).abs())
            .fold(f64::INFINITY, f64::min);
        w.min(c)
    }

    /// Points on every surface, `spacing` apart or finer: the midpoints of equal sub-intervals
    /// of each wall and equal arcs of each pillar.
    pub fn surface_samples(&self, spacing: f64) -> Result<Vec<SurfaceSample>, MappingError> {
        if !(spacing.is_finite() && spacing > 0.0) {
            return Err(MappingError::InvalidParameter {
                what: "sample spacing",
                value: spacing,
            });
        }
        let mut out = Vec::new();
        for (k, w) in self.walls.iter().enumerate() {
            let len = (w.b - w.a).norm();
            let n = (len / spacing).ceil().max(1.0) as usize;
            for s in 0..n {
                let t = (s as f64 + 0.5) / n as f64;
                out.push(SurfaceSample {
                    p: w.a + (w.b - w.a) * t,
                    normal: w.normal,
                    owner: Owner::Wall(k),
                });
            }
        }
        for (k, c) in self.pillars.iter().enumerate() {
            let n = (std::f64::consts::TAU * c.r / spacing).ceil().max(8.0) as usize;
            for s in 0..n {
                let a = std::f64::consts::TAU * (s as f64 + 0.5) / n as f64;
                let normal = V2::new(a.cos(), a.sin());
                out.push(SurfaceSample {
                    p: c.c + normal * c.r,
                    normal,
                    owner: Owner::Pillar(k),
                });
            }
        }
        Ok(out)
    }

    /// Whether another surface lies on the open segment `from -> to`, ignoring `skip`.
    fn blocked(&self, from: &V2, to: &V2, skip: Option<Owner>) -> bool {
        for (k, w) in self.walls.iter().enumerate() {
            if skip != Some(Owner::Wall(k)) && crosses(from, to, &w.a, &w.b) {
                return true;
            }
        }
        for (k, c) in self.pillars.iter().enumerate() {
            if skip != Some(Owner::Pillar(k)) && seg_distance(from, to, &c.c) < c.r - 1e-9 {
                return true;
            }
        }
        false
    }

    /// Whether `sample` is seen from `origin`: within `max_range`, on the lit side of its
    /// surface, with nothing in between.
    pub fn is_visible(
        &self,
        origin: &Vector2<f64>,
        sample: &SurfaceSample,
        max_range: f64,
    ) -> bool {
        let to_origin = origin - sample.p;
        let dist = to_origin.norm();
        dist <= max_range
            && sample.normal.dot(&to_origin) > 1e-9
            && !self.blocked(origin, &sample.p, Some(sample.owner))
    }

    /// Whether a cell centre can be reached by a beam from `origin`: within range, outside every
    /// pillar, and with no surface in between.
    fn cell_reachable(&self, origin: &V2, centre: &V2, max_range: f64) -> bool {
        (centre - origin).norm() <= max_range
            && self.pillars.iter().all(|c| (centre - c.c).norm() > c.r)
            && !self.blocked(origin, centre, None)
    }
}

/// Parameters of one evaluation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EvalParams {
    /// Tolerance (m): a map-occupied cell farther than this from every surface is a false
    /// occupied cell, and a surface sample farther than this from every occupied cell is
    /// missing. Fixed in `scripts/phase4_verdict.py`.
    pub tau: f64,
    /// Spacing of the ground-truth samples (m); finer than a cell.
    pub sample_spacing: f64,
    /// Sensor range limit (m).
    pub max_range: f64,
    /// Use every `stride`-th sensor position for the coverage test (observability always uses
    /// all of them). Coverage costs cells times positions, and neighbouring positions see
    /// almost the same cells.
    pub coverage_stride: usize,
}

/// Mean, 95th percentile and maximum of a distance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceErrorStats {
    /// Mean (m).
    pub mean_m: f64,
    /// 95th percentile, nearest rank (m).
    pub p95_m: f64,
    /// Maximum (m).
    pub max_m: f64,
}

/// The metrics of `docs/phase-4-mapping/benchmark.md` section 2 that are computed from a grid.
#[derive(Debug, Clone, PartialEq)]
pub struct MapMetrics {
    /// Cell edge (m), so every length can be read in cells.
    pub resolution: f64,
    /// The tolerance used (m).
    pub tau: f64,
    /// Cells called occupied.
    pub occupied_cells: usize,
    /// Cells called free.
    pub free_cells: usize,
    /// Cells called unknown.
    pub unknown_cells: usize,
    /// Distance from every occupied cell centre to the nearest surface; `None` if none.
    pub surface_error: Option<SurfaceErrorStats>,
    /// Occupied cells farther than `tau` from every surface.
    pub false_occupied_cells: usize,
    /// `false_occupied_cells / occupied_cells`; `None` if there is no occupied cell.
    pub false_occupied_fraction: Option<f64>,
    /// Ground-truth samples in total.
    pub samples_total: usize,
    /// Samples seen from at least one pose.
    pub samples_observable: usize,
    /// `samples_observable / samples_total`.
    pub observable_fraction: f64,
    /// Observable samples with an occupied cell centre within `tau`; `None` if none observable.
    pub completeness: Option<f64>,
    /// Observable samples that no occupied or unknown cell touches: every cell whose closed box
    /// contains the sample is called free.
    pub false_free_samples: usize,
    /// `false_free_samples / samples_observable`; `None` if none observable.
    pub false_free_fraction: Option<f64>,
    /// Observable samples whose position is outside the grid (counted in neither of the two
    /// metrics above as missing or free, and reported so that a grid too small is visible).
    pub samples_outside_grid: usize,
    /// Cells within range and line of sight of some (strided) pose.
    pub reachable_cells: usize,
    /// Fraction of the reachable cells the map has decided; `None` if none is reachable.
    pub coverage: Option<f64>,
}

/// The cells of the grid whose closed box contains `p`: one cell normally, two or four when
/// `p` lies on a lattice line or corner (within a ten-thousandth of a cell, 5 micrometres at 5 cm; a grid whose resolution went through a `float32` is off by about 2e-6 cells at the far edge). Empty outside the grid.
fn closed_cells(g: &crate::occupancy::GridGeometry, p: &V2) -> Vec<(usize, usize)> {
    let axis = |v: f64, origin: f64, n: usize| -> Vec<usize> {
        let f = (v - origin) / g.resolution;
        if !f.is_finite() {
            return Vec::new();
        }
        let r = f.round();
        let mut idx = vec![f.floor() as i64];
        if (f - r).abs() < 1e-4 {
            idx = vec![r as i64 - 1, r as i64];
        }
        idx.into_iter()
            .filter(|&k| k >= 0 && (k as usize) < n)
            .map(|k| k as usize)
            .collect()
    };
    let xs = axis(p.x, g.origin.x, g.width);
    let ys = axis(p.y, g.origin.y, g.height);
    xs.iter()
        .flat_map(|&i| ys.iter().map(move |&j| (i, j)))
        .collect()
}

fn percentile_nearest_rank(sorted: &[f64], q: f64) -> f64 {
    let n = sorted.len();
    let rank = ((q * n as f64).ceil() as usize).clamp(1, n);
    sorted[rank - 1]
}

fn frac(num: usize, den: usize) -> Option<f64> {
    (den > 0).then(|| num as f64 / den as f64)
}

/// Grade `grid` against `truth`, given the positions the sensor had during the run.
pub fn evaluate(
    grid: &OccupancyGrid,
    truth: &GroundTruth,
    sensor_positions: &[Vector2<f64>],
    params: &EvalParams,
) -> Result<MapMetrics, MappingError> {
    for (what, v) in [("tau", params.tau), ("max_range", params.max_range)] {
        if !(v.is_finite() && v > 0.0) {
            return Err(MappingError::InvalidParameter { what, value: v });
        }
    }
    if params.coverage_stride == 0 {
        return Err(MappingError::InvalidParameter {
            what: "coverage_stride",
            value: 0.0,
        });
    }
    let g = *grid.geometry();
    let (free, occupied, unknown) = grid.count_states();

    // Accuracy of what the map claims.
    let mut errs: Vec<f64> = Vec::with_capacity(occupied);
    for j in 0..g.height {
        for i in 0..g.width {
            if grid.state(i, j) == Some(CellState::Occupied) {
                errs.push(truth.distance_to_surface(&grid.cell_to_world(i, j)));
            }
        }
    }
    let false_occupied = errs.iter().filter(|&&d| d > params.tau).count();
    let surface_error = if errs.is_empty() {
        None
    } else {
        let mean = errs.iter().sum::<f64>() / errs.len() as f64;
        errs.sort_by(|a, b| a.partial_cmp(b).expect("finite distances"));
        Some(SurfaceErrorStats {
            mean_m: mean,
            p95_m: percentile_nearest_rank(&errs, 0.95),
            max_m: *errs.last().expect("non-empty"),
        })
    };

    // What the map should contain.
    let samples = truth.surface_samples(params.sample_spacing)?;
    let observable: Vec<&SurfaceSample> = samples
        .iter()
        .filter(|s| {
            sensor_positions
                .iter()
                .any(|o| truth.is_visible(o, s, params.max_range))
        })
        .collect();
    let occupied_mask = grid.occupied_mask();
    let dist =
        DistanceGrid::from_occupancy(&occupied_mask, g.width, g.height, g.resolution, g.origin)
            .map_err(|e| MappingError::Parse(e.to_string()))?;
    let (mut found, mut false_free, mut outside) = (0usize, 0usize, 0usize);
    for s in &observable {
        let cells = closed_cells(&g, &s.p);
        if cells.is_empty() {
            outside += 1;
            continue;
        }
        // False free only if every cell that touches the sample is free: a surface that lies
        // exactly on a lattice line between a free cell and an occupied one is correctly
        // mapped, and calling the free side "wrong" would be an artefact of tie-breaking.
        if cells
            .iter()
            .all(|&(i, j)| grid.state(i, j) == Some(CellState::Free))
        {
            false_free += 1;
        }
        if occupied > 0 && dist.distance(&s.p) <= params.tau {
            found += 1;
        }
    }
    let inside_grid = observable.len() - outside;

    // How much of what could be known is known.
    let origins: Vec<&V2> = sensor_positions
        .iter()
        .step_by(params.coverage_stride)
        .collect();
    let (mut reachable, mut decided) = (0usize, 0usize);
    for j in 0..g.height {
        for i in 0..g.width {
            let c = grid.cell_to_world(i, j);
            if origins
                .iter()
                .any(|o| truth.cell_reachable(o, &c, params.max_range))
            {
                reachable += 1;
                if grid.state(i, j) != Some(CellState::Unknown) {
                    decided += 1;
                }
            }
        }
    }

    Ok(MapMetrics {
        resolution: g.resolution,
        tau: params.tau,
        occupied_cells: occupied,
        free_cells: free,
        unknown_cells: unknown,
        surface_error,
        false_occupied_cells: false_occupied,
        false_occupied_fraction: frac(false_occupied, occupied),
        samples_total: samples.len(),
        samples_observable: observable.len(),
        observable_fraction: observable.len() as f64 / samples.len().max(1) as f64,
        completeness: frac(found, inside_grid),
        false_free_samples: false_free,
        false_free_fraction: frac(false_free, inside_grid),
        samples_outside_grid: outside,
        reachable_cells: reachable,
        coverage: frac(decided, reachable),
    })
}

impl MapMetrics {
    /// The metrics as `key: value` lines, one per line, in a fixed order; absent values are
    /// `null`. Every number is printed with Rust's shortest round-trip form.
    pub fn to_yaml(&self) -> String {
        fn opt(v: Option<f64>) -> String {
            v.map_or_else(|| "null".to_string(), |x| x.to_string())
        }
        let (mean, p95, max) = match self.surface_error {
            Some(s) => (Some(s.mean_m), Some(s.p95_m), Some(s.max_m)),
            None => (None, None, None),
        };
        let cells = |v: Option<f64>| opt(v.map(|x| x / self.resolution));
        let mut s = String::new();
        let mut line = |k: &str, v: String| s.push_str(&format!("{k}: {v}\n"));
        line("resolution_m", self.resolution.to_string());
        line("tau_m", self.tau.to_string());
        line("tau_cells", (self.tau / self.resolution).to_string());
        line("occupied_cells", self.occupied_cells.to_string());
        line("free_cells", self.free_cells.to_string());
        line("unknown_cells", self.unknown_cells.to_string());
        line("surface_error_mean_m", opt(mean));
        line("surface_error_p95_m", opt(p95));
        line("surface_error_max_m", opt(max));
        line("surface_error_mean_cells", cells(mean));
        line("surface_error_p95_cells", cells(p95));
        line("surface_error_max_cells", cells(max));
        line(
            "false_occupied_cells",
            self.false_occupied_cells.to_string(),
        );
        line("false_occupied_fraction", opt(self.false_occupied_fraction));
        line("samples_total", self.samples_total.to_string());
        line("samples_observable", self.samples_observable.to_string());
        line("observable_fraction", self.observable_fraction.to_string());
        line("completeness", opt(self.completeness));
        line("false_free_samples", self.false_free_samples.to_string());
        line("false_free_fraction", opt(self.false_free_fraction));
        line(
            "samples_outside_grid",
            self.samples_outside_grid.to_string(),
        );
        line("reachable_cells", self.reachable_cells.to_string());
        line("coverage", opt(self.coverage));
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::occupancy::{GridGeometry, OccupancyParams};
    use approx::assert_relative_eq;

    const REAL_LANDMARKS: &str = include_str!("../../../sim/webots/config/landmarks.yaml");

    fn v(x: f64, y: f64) -> V2 {
        V2::new(x, y)
    }

    fn find(samples: &[SurfaceSample], p: V2) -> &SurfaceSample {
        samples
            .iter()
            .min_by(|a, b| (a.p - p).norm().partial_cmp(&(b.p - p).norm()).unwrap())
            .unwrap()
    }

    #[test]
    fn the_real_landmarks_file_gives_the_phase_three_room() {
        let t = GroundTruth::from_landmarks_yaml(REAL_LANDMARKS).unwrap();
        assert_eq!(t.pillar_count(), 4);
        // From the origin the nearest surface is pillar A at (1.8, 1.2): centre distance
        // sqrt(1.8^2 + 1.2^2) = sqrt(4.68), less the radius 0.15.
        assert_relative_eq!(
            t.distance_to_surface(&v(0.0, 0.0)),
            4.68f64.sqrt() - 0.15,
            epsilon = 1e-12
        );
        // A point on the east wall face (inner face at x = 3) and one on pillar A's surface.
        assert_relative_eq!(t.distance_to_surface(&v(3.0, 0.5)), 0.0, epsilon = 1e-12);
        assert_relative_eq!(t.distance_to_surface(&v(1.95, 1.2)), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn landmark_text_errors_are_errors() {
        assert!(GroundTruth::from_landmarks_yaml("").is_err());
        assert!(GroundTruth::from_landmarks_yaml("floor_size: [6.0]\n").is_err());
        assert!(GroundTruth::from_landmarks_yaml(
            "floor_size: [6.0, 6.0]\nlandmarks:\n  - {id: a, x: 1.0, radius: 0.1}\n"
        )
        .is_err());
        assert!(GroundTruth::room(0.0, 3.0, &[]).is_err());
        assert!(GroundTruth::room(3.0, 3.0, &[(0.0, 0.0, -1.0)]).is_err());
    }

    /// Visibility by hand. A 6 m room with a pillar of radius 0.2 centred at (1, 0), seen from
    /// the origin: the pillar's near side is seen and its far side is not (it faces away); the
    /// east wall behind the pillar is hidden at y = 0.25 (the ray passes 0.083 m from the
    /// pillar's axis at x = 1) and seen at y = 0.75 (0.25 m from the axis, clear of the 0.2 m
    /// radius); a point beyond range is not seen; and from outside the room the inner face of a
    /// wall faces away.
    #[test]
    fn visibility_respects_facing_occlusion_range_and_side() {
        let t = GroundTruth::room(3.0, 3.0, &[(1.0, 0.0, 0.2)]).unwrap();
        let s = t.surface_samples(0.05).unwrap();
        let o = v(0.0, 0.0);
        assert!(
            t.is_visible(&o, find(&s, v(0.8, 0.0)), 3.5),
            "near side of the pillar"
        );
        assert!(
            !t.is_visible(&o, find(&s, v(1.2, 0.0)), 3.5),
            "far side of the pillar"
        );
        assert!(
            !t.is_visible(&o, find(&s, v(3.0, 0.25)), 3.5),
            "wall hidden behind the pillar"
        );
        assert!(
            t.is_visible(&o, find(&s, v(3.0, 0.75)), 3.5),
            "wall beside the shadow"
        );
        assert!(
            !t.is_visible(&o, find(&s, v(3.0, 0.75)), 3.0),
            "3.09 m is beyond a 3 m range"
        );
        assert!(
            !t.is_visible(&v(5.0, 0.0), find(&s, v(3.0, 0.75)), 9.0),
            "seen from behind"
        );
    }

    #[test]
    fn the_unobservable_fraction_is_the_pillars_far_sides_and_shadow() {
        let t = GroundTruth::room(3.0, 3.0, &[(1.0, 0.0, 0.2)]).unwrap();
        let s = t.surface_samples(0.01).unwrap();
        let o = [v(0.0, 0.0)];
        // A range that reaches the corners (4.24 m) so that only occlusion and facing are tested.
        let range = 10.0;
        let seen = s.iter().filter(|x| t.is_visible(&o[0], x, range)).count();
        let frac = seen as f64 / s.len() as f64;
        // Half the pillar's circumference faces away, and some of the east wall is in its
        // shadow, so strictly less than everything and more than most.
        assert!(frac < 1.0 && frac > 0.9, "observable fraction {frac}");
        // A second pose on the other side of the pillar sees what the first did not.
        let o2 = [v(0.0, 0.0), v(2.5, 0.0)];
        let seen2 = s
            .iter()
            .filter(|x| o2.iter().any(|p| t.is_visible(p, x, range)))
            .count();
        assert!(seen2 > seen);
    }

    /// A 1 m square room, 0.1 m cells, a 12 by 12 grid whose cell centres fall exactly on the
    /// wall faces at +/- 0.5: the ideal map has 40 occupied cells (the ring of the walls), 81
    /// free cells (the interior) and 23 unknown (the outer margin).
    fn ideal() -> (OccupancyGrid, GroundTruth) {
        let geometry = GridGeometry {
            width: 12,
            height: 12,
            resolution: 0.1,
            origin: v(-0.55, -0.55),
        };
        let mut g = OccupancyGrid::new(geometry, OccupancyParams::default()).unwrap();
        let (lo, lf) = (g.params().l_occ, g.params().l_free);
        for j in 0..=10 {
            for i in 0..=10 {
                if i == 0 || i == 10 || j == 0 || j == 10 {
                    g.add_evidence(i, j, lo);
                } else {
                    g.add_evidence(i, j, lf);
                    g.add_evidence(i, j, lf);
                }
            }
        }
        (g, GroundTruth::room(0.5, 0.5, &[]).unwrap())
    }

    fn params() -> EvalParams {
        EvalParams {
            tau: 0.05,
            sample_spacing: 0.05,
            max_range: 3.5,
            coverage_stride: 1,
        }
    }

    /// Worked example, by hand. 4 walls of 1 m at 0.05 m spacing are 80 samples, all seen from
    /// the centre of a convex room.
    #[test]
    fn worked_example_the_ideal_map_scores_perfectly() {
        let (g, t) = ideal();
        assert_eq!(g.count_states(), (81, 40, 23));
        let m = evaluate(&g, &t, &[v(0.0, 0.0)], &params()).unwrap();
        assert_eq!(
            (m.occupied_cells, m.free_cells, m.unknown_cells),
            (40, 81, 23)
        );
        let e = m.surface_error.unwrap();
        assert!(e.mean_m < 1e-9 && e.p95_m < 1e-9 && e.max_m < 1e-9);
        assert_eq!((m.false_occupied_cells, m.false_free_samples), (0, 0));
        assert_eq!((m.samples_total, m.samples_observable), (80, 80));
        assert_eq!(m.observable_fraction, 1.0);
        assert_eq!(m.completeness, Some(1.0));
        assert_eq!(m.false_free_fraction, Some(0.0));
        assert_eq!(m.samples_outside_grid, 0);
        // Reachable: the 121 cells on or inside the walls; the margin is behind them.
        assert_eq!(m.reachable_cells, 121);
        assert_eq!(m.coverage, Some(1.0));
    }

    /// A phantom obstacle in the middle of the room: one false occupied cell of 41, 0.5 m from
    /// every surface. The 95th percentile (nearest rank 39 of 41) is still zero, which is why
    /// the maximum and the false-occupied count are reported next to it.
    #[test]
    fn a_phantom_cell_is_a_false_occupied_cell_and_moves_the_maximum() {
        let (mut g, t) = ideal();
        let lo = g.params().l_occ;
        g.add_evidence(5, 5, lo);
        g.add_evidence(5, 5, lo);
        let m = evaluate(&g, &t, &[v(0.0, 0.0)], &params()).unwrap();
        assert_eq!(m.occupied_cells, 41);
        assert_eq!(m.false_occupied_cells, 1);
        assert_relative_eq!(
            m.false_occupied_fraction.unwrap(),
            1.0 / 41.0,
            epsilon = 1e-12
        );
        let e = m.surface_error.unwrap();
        assert_relative_eq!(e.max_m, 0.5 - 0.0, epsilon = 1e-9);
        assert_relative_eq!(e.mean_m, 0.5 / 41.0, epsilon = 1e-9);
        assert!(e.p95_m < 1e-9);
        assert_eq!(m.completeness, Some(1.0));
    }

    /// A wall cell the map calls free: the cell (10, 5) at (0.5, 0) holds the two samples at
    /// y = -0.025 and y = 0.025, so two false-free samples of 80, and with `tau` one half cell
    /// those two samples are also missing (0.075 m from the next occupied centre).
    #[test]
    fn a_hole_in_a_wall_is_false_free_and_lowers_completeness() {
        let (mut g, t) = ideal();
        let lf = g.params().l_free;
        for _ in 0..4 {
            g.add_evidence(10, 5, lf);
        }
        assert_eq!(g.state(10, 5), Some(CellState::Free));
        let m = evaluate(&g, &t, &[v(0.0, 0.0)], &params()).unwrap();
        assert_eq!(m.false_free_samples, 2);
        assert_relative_eq!(m.false_free_fraction.unwrap(), 2.0 / 80.0, epsilon = 1e-12);
        assert_relative_eq!(m.completeness.unwrap(), 78.0 / 80.0, epsilon = 1e-12);
        assert_eq!(m.occupied_cells, 39);
    }

    /// The same hole read with a one-cell tolerance is *not* missing, only false free: the
    /// tolerance decides how much a near miss costs, which is why it is fixed in the verdict
    /// script before any run.
    #[test]
    fn the_tolerance_changes_completeness_but_not_false_free() {
        let (mut g, t) = ideal();
        let lf = g.params().l_free;
        for _ in 0..4 {
            g.add_evidence(10, 5, lf);
        }
        let mut p = params();
        p.tau = 0.1;
        let m = evaluate(&g, &t, &[v(0.0, 0.0)], &p).unwrap();
        assert_eq!(m.completeness, Some(1.0));
        assert_eq!(m.false_free_samples, 2);
    }

    /// Only the 40 wall cells decided, the 81 interior cells never observed: coverage is
    /// 40 of the 121 reachable cells.
    #[test]
    fn coverage_is_decided_cells_over_reachable_cells() {
        let (ideal_grid, t) = ideal();
        let mut g = OccupancyGrid::new(*ideal_grid.geometry(), *ideal_grid.params()).unwrap();
        let lo = g.params().l_occ;
        for j in 0..=10 {
            for i in 0..=10 {
                if i == 0 || i == 10 || j == 0 || j == 10 {
                    g.add_evidence(i, j, lo);
                }
            }
        }
        let m = evaluate(&g, &t, &[v(0.0, 0.0)], &params()).unwrap();
        assert_eq!(m.reachable_cells, 121);
        assert_relative_eq!(m.coverage.unwrap(), 40.0 / 121.0, epsilon = 1e-12);
    }

    #[test]
    fn an_empty_map_reports_what_it_cannot_say_as_null() {
        let (g0, t) = ideal();
        let g = OccupancyGrid::new(*g0.geometry(), *g0.params()).unwrap();
        let m = evaluate(&g, &t, &[v(0.0, 0.0)], &params()).unwrap();
        assert_eq!(m.surface_error, None);
        assert_eq!(m.false_occupied_fraction, None);
        assert_eq!(m.completeness, Some(0.0));
        assert_eq!(m.coverage, Some(0.0));
        let y = m.to_yaml();
        assert!(y.contains("surface_error_mean_m: null\n"));
        assert!(y.contains("completeness: 0\n"));
        assert!(y.contains("tau_cells: 0.5\n"));
    }

    /// A surface exactly on a lattice line between two cells. The room's east wall at x = 0.5
    /// is the line between cell column 10 ([0.4, 0.5)) and column 11 ([0.5, 0.6)) of a grid
    /// whose origin is -0.6. If either side is occupied the wall is mapped; only if both are
    /// free is it a false free sample.
    #[test]
    fn a_surface_on_a_lattice_line_is_false_free_only_if_both_sides_are_free() {
        let geometry = GridGeometry {
            width: 12,
            height: 12,
            resolution: 0.1,
            origin: v(-0.6, -0.6),
        };
        let t = GroundTruth::room(0.5, 0.5, &[]).unwrap();
        let build = |occupied_column: Option<usize>| {
            let mut g = OccupancyGrid::new(geometry, OccupancyParams::default()).unwrap();
            let (lo, lf) = (g.params().l_occ, g.params().l_free);
            // Rows 1 to 10 only: the top and bottom walls lie on the lines above row 10 and
            // below row 1, and row 0 and row 11 stay unknown so that they are not counted.
            for j in 1..=10 {
                for i in 9..=11 {
                    g.add_evidence(i, j, lf);
                    g.add_evidence(i, j, lf);
                }
                if let Some(c) = occupied_column {
                    g.add_evidence(c, j, lo);
                    g.add_evidence(c, j, lo);
                    g.add_evidence(c, j, lo);
                }
            }
            g
        };
        let east = |m: &MapMetrics| m.false_free_samples;
        let p = EvalParams {
            tau: 0.05,
            sample_spacing: 0.1,
            max_range: 3.5,
            coverage_stride: 1,
        };
        let origin = [v(0.0, 0.0)];
        // Occupied on the outer side, occupied on the inner side, free on both.
        let outer = evaluate(&build(Some(11)), &t, &origin, &p).unwrap();
        let inner = evaluate(&build(Some(10)), &t, &origin, &p).unwrap();
        let none = evaluate(&build(None), &t, &origin, &p).unwrap();
        assert_eq!(east(&outer), 0);
        assert_eq!(east(&inner), 0);
        // Ten samples on the east wall, spaced 0.1 m, each touching cells of rows j.. that are
        // all free in the empty map: the east wall's ten samples are false free.
        assert!(
            east(&none) >= 10,
            "false free with both sides free: {}",
            east(&none)
        );
    }

    /// A grid whose resolution went through a `float32` (a `nav_msgs/OccupancyGrid` carries it
    /// so) has resolution 0.05000000074505806 for 0.05, which puts the lattice line x = 3.0 of a
    /// grid from -3.5 at cell coordinate 129.999998 and not 130. The wall must still be found on
    /// the line, between cells 129 and 130.
    #[test]
    fn a_float32_resolution_does_not_hide_a_lattice_line() {
        let g = GridGeometry {
            width: 140,
            height: 140,
            resolution: 0.05f32 as f64,
            origin: v(-3.5, -3.5),
        };
        assert_ne!(g.resolution, 0.05);
        assert_eq!(
            closed_cells(&g, &v(3.0, 0.0125)),
            vec![(129, 70), (130, 70)]
        );
        // An interior point is one cell.
        assert_eq!(closed_cells(&g, &v(2.99, 0.0125)), vec![(129, 70)]);
    }

    #[test]
    fn invalid_evaluation_parameters_are_refused() {
        let (g, t) = ideal();
        let mut p = params();
        p.tau = 0.0;
        assert!(evaluate(&g, &t, &[v(0.0, 0.0)], &p).is_err());
        let mut p = params();
        p.coverage_stride = 0;
        assert!(evaluate(&g, &t, &[v(0.0, 0.0)], &p).is_err());
        let mut p = params();
        p.sample_spacing = f64::NAN;
        assert!(evaluate(&g, &t, &[v(0.0, 0.0)], &p).is_err());
    }
}
