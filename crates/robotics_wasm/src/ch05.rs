//! Chapter 5 bindings: mapping from known poses (see web/design/Chapter-05.md).
//!
//! Three wrappers over `robotics_mapping`, each calling the crate that the robot's `mapping_node`
//! calls and flattening what comes back:
//!
//! * [`Ch05Replay`]: the recorded Webots scans of `evidence/phase-4-mapping/2026-10-11-evaluation-4-oblique`
//!   integrated one at a time into an `OccupancyGrid` by the `ScanIntegrator`, with the model frozen
//!   at Stage D (Bresenham, `l_max` 5.0, the Webots beam layout). The page fetches the recording
//!   (a copy of `integrated-scans.csv`, made by `npm run wasm`; `checks/ch05.ts` asserts the copy
//!   equals the evidence file) and hands its text to [`Ch05Replay::new`]. Nothing is simulated here:
//!   the scans are what the simulated lidar of Webots returned, the poses are the ground truth the
//!   mapper was given, and the grid is the one the node built (a test below and a check on the
//!   compiled module assert that the replay equals the node's saved `final-grid.txt` byte for byte).
//! * [`ch05_beam_wall`]: one wall at a chosen incidence, seen by 360 beams whose *true* bearings are
//!   the Webots ones (`BeamLayout::PixelCentres`), read back with the bearings the driver
//!   *publishes* and with the corrected ones. The range comes from `robotics_sim::World::raycast`.
//! * [`ch05_trace`]: the cells the two traversals of `robotics_mapping::raycast`, `Bresenham` and
//!   `Supercover`, visit for one beam, and the cells on which they disagree.
//!
//! What is *not* a crate algorithm, and says so where it lives: reading the recording's CSV (the
//! parser mirrors the one in `crates/robotics_mapping/examples/replay_scans.rs`, which is an example
//! and not library code), the endpoint trigonometry of `scan_points` and `ch05_beam_wall`
//! (as `ch03_scan_points` does), and the phantom-cell count, which applies the rule of
//! `robotics_mapping::eval::evaluate` (an occupied cell whose centre is farther than `tau` from every
//! true surface) with the same `GroundTruth::distance_to_surface`.
//!
//! No function here draws a random number: the recording is data, the wall experiment is exact
//! geometry, and the traversals are deterministic. So, unlike Chapters 2 and 3, there is no seed
//! in this module and nothing on this page depends on the native-versus-wasm32 `SmallRng` stream.
//!
//! Flat-array layouts are documented on each function.

use robotics_core::nalgebra::Vector2;
use robotics_core::{FrameId, LaserScan, Se2, Timestamp};
use robotics_mapping::raycast::{Bresenham, Supercover};
use robotics_mapping::{
    grid_entropy, BeamLayout, CellState, GridGeometry, GroundTruth, NoReturn, OccupancyGrid,
    OccupancyParams, RayTracer, ScanIntegrator, DEFAULT_CLEAR_MARGIN,
};
use robotics_sim::World;
use std::f64::consts::TAU;
use wasm_bindgen::prelude::*;

fn err<E: std::fmt::Display>(e: E) -> String {
    e.to_string()
}

// ---------------------------------------------------------------------------------------
// The frozen model, and the recording
// ---------------------------------------------------------------------------------------

/// The occupied threshold of the frozen model (probability), `implementation.md` 5.4 item 4 and
/// `OccupancyParams::default`: four hits make a cell occupied.
pub const FROZEN_P_OCCUPIED: f64 = 0.95;

/// The geometry `mapping_node` used for every run of Phase 4: the header of each `final-grid.txt`
/// (`width 140`, `height 140`, `resolution 0.05`, `origin -3.5 -3.5`). A check asserts this against
/// the evidence file's header.
fn frozen_geometry() -> GridGeometry {
    GridGeometry {
        width: 140,
        height: 140,
        resolution: 0.05,
        origin: Vector2::new(-3.5, -3.5),
    }
}

/// The inverse sensor model and read-out of the frozen runs: hit 0.7, miss 0.4, lower clamp 0.12,
/// upper clamp 0.99330715 (log-odds 5.0, `launch p_max`), occupied threshold `p_occupied`, free
/// threshold 0.35.
fn frozen_params(p_occupied: f64) -> OccupancyParams {
    OccupancyParams::from_probabilities(0.7, 0.4, 0.12, 0.99330715, p_occupied, 0.35)
}

/// The grid and integrator of the frozen runs: Bresenham traversal, no-return beams clear up to
/// `range_max` less the default margin (`launch tracer`, `no_return`, `clear_margin`).
fn frozen_pair(p_occupied: f64) -> Result<(OccupancyGrid, ScanIntegrator), String> {
    let grid = OccupancyGrid::new(frozen_geometry(), frozen_params(p_occupied)).map_err(err)?;
    let integrator = ScanIntegrator::new(NoReturn::ClearToMaxRange, RayTracer::Bresenham)
        .with_clear_margin(DEFAULT_CLEAR_MARGIN)
        .map_err(err)?;
    Ok((grid, integrator))
}

/// The room of `landmarks.yaml`: the four pillars and the inner faces at ±3.0 m, read by the
/// crate's own parser from a copy of the file the world is built from.
fn room_truth() -> Result<GroundTruth, String> {
    GroundTruth::from_landmarks_yaml(include_str!("../../../sim/webots/config/landmarks.yaml"))
        .map_err(err)
}

/// One line of `integrated-scans.csv`: the sensor pose at the scan's measurement instant, and
/// the scan with the bearings the driver published.
struct Recorded {
    sensor: Se2,
    scan: LaserScan,
}

/// Parse `integrated-scans.csv`, mirroring `examples/replay_scans.rs`. Columns:
/// `stamp_s, sensor_x, sensor_y, sensor_theta, angle_min, angle_increment, range_min, range_max,
/// n, ranges...` (`inf` for a beam with no return).
fn parse_recording(csv: &str) -> Result<Vec<Recorded>, String> {
    let mut out = Vec::new();
    for (n, line) in csv.lines().enumerate() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        let num = |i: usize| -> Result<f64, String> {
            f.get(i)
                .ok_or_else(|| format!("line {}: too few fields", n + 1))?
                .parse::<f64>()
                .map_err(|_| format!("line {}: field {i} is not a number", n + 1))
        };
        let beams = num(8)? as usize;
        if f.len() != 9 + beams {
            return Err(format!("line {}: {} fields for {beams} beams", n + 1, f.len()));
        }
        let mut ranges = Vec::with_capacity(beams);
        for tok in &f[9..] {
            ranges.push(
                tok.parse::<f32>()
                    .map_err(|_| format!("line {}: range {tok:?}", n + 1))?,
            );
        }
        out.push(Recorded {
            sensor: Se2::new(num(1)?, num(2)?, num(3)?),
            scan: LaserScan {
                stamp: Timestamp::from_secs_f64(num(0)?),
                frame: FrameId::new("replay"),
                angle_min: num(4)?,
                angle_increment: num(5)?,
                time_increment: 0.0,
                scan_time: 0.0,
                range_min: num(6)?,
                range_max: num(7)?,
                ranges,
                intensities: Vec::new(),
            },
        });
    }
    if out.is_empty() {
        return Err("the recording holds no scans".into());
    }
    Ok(out)
}

/// The end points of a scan's returns in the grid's frame, flat `[x0, y0, x1, y1, …]`, `NaN` for a
/// beam without a finite return inside its limits. Bookkeeping, as `ch03_scan_points` is: the
/// bearings are those of `scan` as given.
fn scan_points(sensor: &Se2, scan: &LaserScan) -> Vec<f64> {
    let mut out = Vec::with_capacity(2 * scan.ranges.len());
    for (i, &r) in scan.ranges.iter().enumerate() {
        let r = r as f64;
        if r.is_finite() && r >= scan.range_min && r < scan.range_max {
            let a = sensor.theta() + scan.bearing(i);
            out.push(sensor.x() + r * a.cos());
            out.push(sensor.y() + r * a.sin());
        } else {
            out.push(f64::NAN);
            out.push(f64::NAN);
        }
    }
    out
}

/// A recorded run replayed through the mapper, one scan at a time.
#[wasm_bindgen]
pub struct Ch05Replay {
    recording: Vec<Recorded>,
    truth: GroundTruth,
    p_occupied: f64,
    grid: OccupancyGrid,
    integrator: ScanIntegrator,
    next: usize,
}

impl Ch05Replay {
    fn integrate_into(
        grid: &mut OccupancyGrid,
        integrator: &mut ScanIntegrator,
        rec: &Recorded,
    ) -> Result<(), String> {
        let scan = BeamLayout::PixelCentres.apply(&rec.scan).map_err(err)?;
        integrator
            .integrate(grid, &rec.sensor, &scan)
            .map_err(err)?;
        Ok(())
    }

    /// Rebuild the grid from scratch with `p_occupied` and integrate the first `upto` scans.
    fn rebuild(&mut self, p_occupied: f64, upto: usize) -> Result<(), String> {
        let (mut grid, mut integrator) = frozen_pair(p_occupied)?;
        for rec in &self.recording[..upto] {
            Self::integrate_into(&mut grid, &mut integrator, rec)?;
        }
        self.grid = grid;
        self.integrator = integrator;
        self.p_occupied = p_occupied;
        self.next = upto;
        Ok(())
    }
}

#[wasm_bindgen]
impl Ch05Replay {
    /// Parse a recording (the text of `integrated-scans.csv`) and start with an empty grid, the
    /// frozen model and the frozen occupied threshold. Nothing is integrated yet.
    #[wasm_bindgen(constructor)]
    pub fn new(csv: &str) -> Result<Ch05Replay, String> {
        let recording = parse_recording(csv)?;
        let (grid, integrator) = frozen_pair(FROZEN_P_OCCUPIED)?;
        Ok(Ch05Replay {
            recording,
            truth: room_truth()?,
            p_occupied: FROZEN_P_OCCUPIED,
            grid,
            integrator,
            next: 0,
        })
    }

    /// Scans in the recording.
    pub fn total(&self) -> u32 {
        self.recording.len() as u32
    }

    /// Scans integrated so far.
    pub fn integrated(&self) -> u32 {
        self.next as u32
    }

    /// Integrate up to `n` more scans; returns the number integrated in all.
    pub fn step(&mut self, n: u32) -> Result<u32, String> {
        let upto = (self.next + n as usize).min(self.recording.len());
        for k in self.next..upto {
            Self::integrate_into(&mut self.grid, &mut self.integrator, &self.recording[k])?;
        }
        self.next = upto;
        Ok(self.next as u32)
    }

    /// Back to an empty grid, same occupied threshold.
    pub fn reset(&mut self) -> Result<(), String> {
        let p = self.p_occupied;
        self.rebuild(p, 0)
    }

    /// Change the occupied threshold (the probability above which a cell is *called* occupied)
    /// and integrate the scans seen so far again with it. The log-odds a cell accumulates do not
    /// depend on the threshold; the grid is rebuilt because the crate fixes the thresholds when
    /// the grid is made.
    pub fn set_occupied_probability(&mut self, p: f64) -> Result<(), String> {
        let upto = self.next;
        self.rebuild(p, upto)
    }

    /// The occupied threshold now in force (probability).
    pub fn occupied_probability(&self) -> f64 {
        self.p_occupied
    }

    /// How many hits, with nothing else seen, take an unknown cell to occupied under this
    /// threshold: the smallest `k` with `k · l_occ >= occupied_threshold`.
    pub fn hits_to_occupy(&self) -> u32 {
        let p = self.grid.params();
        (p.occupied_threshold / p.l_occ).ceil() as u32
    }

    /// `[l_occ, l_free, l_min, l_max, occupied_threshold, free_threshold]`, log-odds.
    pub fn model(&self) -> Vec<f64> {
        let p = self.grid.params();
        [
            p.l_occ,
            p.l_free,
            p.l_min,
            p.l_max,
            p.occupied_threshold,
            p.free_threshold,
        ]
        .iter()
        .map(|&v| v as f64)
        .collect()
    }

    /// `[width, height, resolution, origin_x, origin_y]`.
    pub fn geometry(&self) -> Vec<f64> {
        let g = self.grid.geometry();
        vec![
            g.width as f64,
            g.height as f64,
            g.resolution,
            g.origin.x,
            g.origin.y,
        ]
    }

    /// Log-odds of every cell, row-major with `j` (y) as the slow index.
    pub fn log_odds(&self) -> Vec<f32> {
        self.grid.log_odds_slice().to_vec()
    }

    /// State of every cell under the current thresholds, row-major: 0 unknown, 1 free, 2 occupied.
    pub fn states(&self) -> Vec<u8> {
        self.grid
            .states()
            .iter()
            .map(|s| match s {
                CellState::Unknown => 0,
                CellState::Free => 1,
                CellState::Occupied => 2,
            })
            .collect()
    }

    /// `[free, occupied, unknown]` cell counts (`OccupancyGrid::count_states`).
    pub fn counts(&self) -> Vec<u32> {
        let (f, o, u) = self.grid.count_states();
        vec![f as u32, o as u32, u as u32]
    }

    /// `[total_bits, decided_bits, cells, decided_cells]` (`grid_entropy`).
    pub fn entropy(&self) -> Vec<f64> {
        let e = grid_entropy(&self.grid);
        vec![
            e.total_bits,
            e.decided_bits,
            e.cells as f64,
            e.decided_cells as f64,
        ]
    }

    /// The occupied cells farther than `tau_cells` cells from every true surface (the
    /// false-occupied rule of `evaluate`), flat `[i0, j0, i1, j1, …]`.
    pub fn phantom_cells(&self, tau_cells: f64) -> Vec<u32> {
        let g = *self.grid.geometry();
        let tau = tau_cells * g.resolution;
        let mut out = Vec::new();
        for j in 0..g.height {
            for i in 0..g.width {
                if self.grid.state(i, j) == Some(CellState::Occupied)
                    && self.truth.distance_to_surface(&self.grid.cell_to_world(i, j)) > tau
                {
                    out.push(i as u32);
                    out.push(j as u32);
                }
            }
        }
        out
    }

    /// The sensor positions of the whole recording, flat `[x0, y0, x1, y1, …]`: the route.
    pub fn route(&self) -> Vec<f64> {
        self.recording
            .iter()
            .flat_map(|r| [r.sensor.x(), r.sensor.y()])
            .collect()
    }

    /// `[x, y, theta]` of the sensor at the last integrated scan; empty before the first.
    pub fn last_pose(&self) -> Vec<f64> {
        match self.next.checked_sub(1) {
            Some(k) => {
                let s = &self.recording[k].sensor;
                vec![s.x(), s.y(), s.theta()]
            }
            None => Vec::new(),
        }
    }

    /// End points of the last integrated scan's returns, flat `[x, y, …]`, `NaN` for a beam with
    /// no return; bearings are the corrected ones the mapper integrated. Empty before the first.
    pub fn last_points(&self) -> Result<Vec<f64>, String> {
        match self.next.checked_sub(1) {
            Some(k) => {
                let rec = &self.recording[k];
                let scan = BeamLayout::PixelCentres.apply(&rec.scan).map_err(err)?;
                Ok(scan_points(&rec.sensor, &scan))
            }
            None => Ok(Vec::new()),
        }
    }

    /// The log-odds of cell `(i, j)` before the first scan and after each integrated one
    /// (`integrated() + 1` values), by replaying the scans so far on a scratch grid with the
    /// threshold now in force.
    pub fn trace_cell(&self, i: u32, j: u32) -> Result<Vec<f64>, String> {
        let (mut grid, mut integrator) = frozen_pair(self.p_occupied)?;
        let at = |g: &OccupancyGrid| -> Result<f64, String> {
            g.log_odds(i as usize, j as usize)
                .map(|l| l as f64)
                .ok_or_else(|| format!("cell ({i}, {j}) is outside the grid"))
        };
        let mut out = vec![at(&grid)?];
        for rec in &self.recording[..self.next] {
            Self::integrate_into(&mut grid, &mut integrator, rec)?;
            out.push(at(&grid)?);
        }
        Ok(out)
    }

    /// The text form of the grid (`OccupancyGrid::to_text`), the file the node writes as
    /// `final-grid.txt`.
    pub fn grid_text(&self) -> String {
        self.grid.to_text()
    }
}

// ---------------------------------------------------------------------------------------
// The beam-angle experiment
// ---------------------------------------------------------------------------------------

/// Beams of the Webots LDS-01 as published, and its range limits (the recorded scans: `n 360`,
/// `range_min 0.12`, `range_max 3.5`).
const BEAMS: usize = 360;
const RANGE_MIN: f64 = 0.12;
const RANGE_MAX: f64 = 3.5;

/// A 360-beam scan with the bearings `webots_ros2_driver` publishes for the LDS-01:
/// `angle_min = fov/2` and `angle_increment = -fov/(N-1)` with `fov = 2π`
/// (`crates/robotics_mapping/src/layout.rs`, `implementation.md` 5.4 item 1).
fn published_scan(ranges: Vec<f32>) -> LaserScan {
    LaserScan {
        stamp: Timestamp::from_secs_f64(0.0),
        frame: FrameId::new("LDS-01"),
        angle_min: TAU / 2.0,
        angle_increment: -TAU / (BEAMS as f64 - 1.0),
        time_increment: 0.0,
        scan_time: 0.0,
        range_min: RANGE_MIN,
        range_max: RANGE_MAX,
        ranges,
        intensities: Vec::new(),
    }
}

/// The bearing error of every beam, `true − published` (rad), 360 values: what
/// `BeamLayout::PixelCentres` changes. `−8.7 mrad` at beam 0, about 0 at beam 179, `+8.7 mrad` at
/// beam 359 (`layout.rs`).
#[wasm_bindgen]
pub fn ch05_beam_error_table() -> Result<Vec<f64>, String> {
    let published = published_scan(vec![1.0; BEAMS]);
    let truth = BeamLayout::PixelCentres.apply(&published).map_err(err)?;
    Ok((0..BEAMS)
        .map(|i| truth.bearing(i) - published.bearing(i))
        .collect())
}

/// One wall seen by the Webots lidar: what each beam really measured, and where a mapper that
/// reads the bearings as published puts each hit, against where one that applies
/// `BeamLayout::PixelCentres` does.
#[wasm_bindgen]
pub struct Ch05BeamWall {
    wall: Vec<f64>,
    ranges: Vec<f64>,
    range_error: Vec<f64>,
    error_published: Vec<f64>,
    error_corrected: Vec<f64>,
    hits_published: Vec<f64>,
    hits_corrected: Vec<f64>,
}

#[wasm_bindgen]
impl Ch05BeamWall {
    /// The wall's two ends `[ax, ay, bx, by]` (m), clipped to a ±4 m window along the wall for
    /// drawing; the world holds a wall 80 m long.
    #[wasm_bindgen(getter)]
    pub fn wall(&self) -> Vec<f64> {
        self.wall.clone()
    }
    /// The range each of the 360 beams measured (m); `NaN` where the beam saw no wall within 3.5 m.
    #[wasm_bindgen(getter)]
    pub fn ranges(&self) -> Vec<f64> {
        self.ranges.clone()
    }
    /// The range residual of each beam (m): the range it measured minus the range the wall gives
    /// along the *published* bearing, `robotics_sim::World::raycast` at that bearing. This is the
    /// quantity `NOTE-root-cause-analysis.md` fits as `range · tan(incidence) · ε`. `NaN` where
    /// either ray misses the wall within 3.5 m.
    #[wasm_bindgen(getter)]
    pub fn range_error(&self) -> Vec<f64> {
        self.range_error.clone()
    }
    /// Signed distance of each beam's hit from the wall (m) when its bearing is read as
    /// *published*, about `range · sin(incidence) · ε`; positive is beyond the wall's face, where the beam did not go. `NaN` where
    /// there is no hit.
    #[wasm_bindgen(getter)]
    pub fn error_published(&self) -> Vec<f64> {
        self.error_published.clone()
    }
    /// The same with the `pixel_centres` bearings; zero up to the float32 rounding of the range.
    #[wasm_bindgen(getter)]
    pub fn error_corrected(&self) -> Vec<f64> {
        self.error_corrected.clone()
    }
    /// The hits with the published bearings, flat `[x, y, …]` (m), `NaN` where there is none.
    #[wasm_bindgen(getter)]
    pub fn hits_published(&self) -> Vec<f64> {
        self.hits_published.clone()
    }
    /// The hits with the corrected bearings, flat `[x, y, …]` (m).
    #[wasm_bindgen(getter)]
    pub fn hits_corrected(&self) -> Vec<f64> {
        self.hits_corrected.clone()
    }
}

/// The largest incidence (rad) at which the beam pointing straight behind the robot still reaches
/// a wall `distance` metres away: `acos(distance / 3.5)`, since its range is `distance / cos`.
#[wasm_bindgen]
pub fn ch05_beam_wall_max_incidence(distance: f64) -> f64 {
    (distance / RANGE_MAX).clamp(-1.0, 1.0).acos()
}

/// A wall whose nearest point is `distance` metres from the sensor, tilted so that the beam
/// pointing straight behind the robot meets it at `incidence` radians off its normal.
///
/// The sensor sits at the origin facing +x. The 360 beams have the Webots *true* bearings; the
/// wall is 80 m long, and `robotics_sim::World::raycast` gives each beam's range up to 3.5 m.
/// Those ranges are then read twice: with the bearings the driver publishes, as `mapping_node`
/// does by default, and with `BeamLayout::PixelCentres`, as the frozen runs did. Fails if the
/// straight-behind beam would not reach the wall (`ch05_beam_wall_max_incidence`).
#[wasm_bindgen]
pub fn ch05_beam_wall(distance: f64, incidence: f64) -> Result<Ch05BeamWall, String> {
    if !(distance.is_finite() && (RANGE_MIN..RANGE_MAX).contains(&distance)) {
        return Err(format!("distance {distance} is outside [{RANGE_MIN}, {RANGE_MAX})"));
    }
    if !(incidence.is_finite() && incidence >= 0.0 && incidence < ch05_beam_wall_max_incidence(distance))
    {
        return Err(format!(
            "incidence {incidence} rad: the beam straight behind does not reach a wall {distance} m away at that angle (limit {:.3} rad)",
            ch05_beam_wall_max_incidence(distance)
        ));
    }
    // Unit vector from the sensor to the wall's nearest point; pi - incidence puts the wall
    // behind the robot, where the beams with the largest bearing error point.
    let phi = TAU / 2.0 - incidence;
    let n = Vector2::new(phi.cos(), phi.sin());
    let t = Vector2::new(-n.y, n.x);
    let p = n * distance;
    let mut world = World::empty();
    world.add_segment(p - t * 40.0, p + t * 40.0).map_err(err)?;

    let blank = published_scan(vec![0.0; BEAMS]);
    let truth = BeamLayout::PixelCentres.apply(&blank).map_err(err)?;
    let origin = Vector2::zeros();
    let ranges: Vec<f32> = (0..BEAMS)
        .map(|i| {
            world
                .raycast(&origin, truth.bearing(i), RANGE_MAX)
                .map_or(f32::INFINITY, |r| r as f32)
        })
        .collect();
    let scan = published_scan(ranges);
    let corrected = BeamLayout::PixelCentres.apply(&scan).map_err(err)?;

    let (a, b) = (p - t * 4.0, p + t * 4.0);
    let mut out = Ch05BeamWall {
        wall: vec![a.x, a.y, b.x, b.y],
        ranges: Vec::with_capacity(BEAMS),
        range_error: Vec::with_capacity(BEAMS),
        error_published: Vec::with_capacity(BEAMS),
        error_corrected: Vec::with_capacity(BEAMS),
        hits_published: Vec::with_capacity(2 * BEAMS),
        hits_corrected: Vec::with_capacity(2 * BEAMS),
    };
    for (i, &r) in scan.ranges.iter().enumerate() {
        if !r.is_finite() {
            out.ranges.push(f64::NAN);
            out.range_error.push(f64::NAN);
            out.error_published.push(f64::NAN);
            out.error_corrected.push(f64::NAN);
            out.hits_published.extend([f64::NAN, f64::NAN]);
            out.hits_corrected.extend([f64::NAN, f64::NAN]);
            continue;
        }
        let r = r as f64;
        let hit = |a: f64| Vector2::new(r * a.cos(), r * a.sin());
        let (hp, hc) = (hit(scan.bearing(i)), hit(corrected.bearing(i)));
        out.ranges.push(r);
        out.range_error.push(
            world
                .raycast(&origin, scan.bearing(i), RANGE_MAX)
                .map_or(f64::NAN, |expected| r - expected),
        );
        out.error_published.push(hp.dot(&n) - distance);
        out.error_corrected.push(hc.dot(&n) - distance);
        out.hits_published.extend([hp.x, hp.y]);
        out.hits_corrected.extend([hc.x, hc.y]);
    }
    Ok(out)
}

// ---------------------------------------------------------------------------------------
// The two traversals
// ---------------------------------------------------------------------------------------

/// The cells two ray traversals visit for one beam.
#[wasm_bindgen]
pub struct Ch05Trace {
    bresenham: Vec<i32>,
    supercover: Vec<i32>,
    only_bresenham: Vec<i32>,
    only_supercover: Vec<i32>,
}

#[wasm_bindgen]
impl Ch05Trace {
    /// Cells of `Bresenham`, in order, flat `[i0, j0, i1, j1, …]`.
    #[wasm_bindgen(getter)]
    pub fn bresenham(&self) -> Vec<i32> {
        self.bresenham.clone()
    }
    /// Cells of `Supercover`, in order, flat.
    #[wasm_bindgen(getter)]
    pub fn supercover(&self) -> Vec<i32> {
        self.supercover.clone()
    }
    /// Cells the supercover visits and Bresenham skips, flat.
    #[wasm_bindgen(getter)]
    pub fn only_supercover(&self) -> Vec<i32> {
        self.only_supercover.clone()
    }
    /// Cells Bresenham visits and the supercover does not, flat.
    #[wasm_bindgen(getter)]
    pub fn only_bresenham(&self) -> Vec<i32> {
        self.only_bresenham.clone()
    }
}

fn flat(cells: &[(i32, i32)]) -> Vec<i32> {
    cells.iter().flat_map(|&(i, j)| [i, j]).collect()
}

/// The cells a beam from `(ax, ay)` to `(bx, by)` crosses on a grid with its corner at the origin
/// and cells `resolution` wide, by each of the two traversals the mapper can use
/// (`RayTracer::Bresenham`, the frozen default, and `RayTracer::Supercover`). The end cells are
/// taken as `ScanIntegrator` takes them, by flooring.
#[wasm_bindgen]
pub fn ch05_trace(ax: f64, ay: f64, bx: f64, by: f64, resolution: f64) -> Result<Ch05Trace, String> {
    if ![ax, ay, bx, by].iter().all(|v| v.is_finite())
        || !(resolution.is_finite() && resolution > 0.0)
    {
        return Err("the beam and the resolution must be finite, the resolution positive".into());
    }
    let cell = |x: f64, y: f64| ((x / resolution).floor() as i32, (y / resolution).floor() as i32);
    let b: Vec<(i32, i32)> = Bresenham::new(cell(ax, ay), cell(bx, by)).collect();
    let s: Vec<(i32, i32)> = Supercover::new(
        &Vector2::new(ax, ay),
        &Vector2::new(bx, by),
        &Vector2::zeros(),
        resolution,
    )
    .collect();
    let only_b: Vec<(i32, i32)> = b.iter().copied().filter(|c| !s.contains(c)).collect();
    let only_s: Vec<(i32, i32)> = s.iter().copied().filter(|c| !b.contains(c)).collect();
    Ok(Ch05Trace {
        bresenham: flat(&b),
        supercover: flat(&s),
        only_bresenham: flat(&only_b),
        only_supercover: flat(&only_s),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const RUN: &str = "2026-10-11-evaluation-4-oblique";

    fn evidence(file: &str) -> String {
        let path = format!(
            "{}/../../evidence/phase-4-mapping/{RUN}/{file}",
            env!("CARGO_MANIFEST_DIR")
        );
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
    }

    /// The replay of the recorded scans, with the frozen model, is the grid the node saved: byte
    /// for byte (claim M2 of `docs/phase-4-mapping/problem.md`, here on the page's code path).
    #[test]
    fn replay_of_the_oblique_run_is_its_saved_grid_byte_for_byte() {
        let mut r = Ch05Replay::new(&evidence("integrated-scans.csv")).unwrap();
        assert_eq!(r.total(), 504);
        assert_eq!(r.step(10_000).unwrap(), 504);
        assert_eq!(r.grid_text(), evidence("final-grid.txt"));
    }

    /// Stepping in pieces gives the same grid as stepping at once, and a threshold change after
    /// the fact gives the grid a run with that threshold would have had.
    #[test]
    fn stepping_and_re_thresholding_do_not_change_the_log_odds() {
        let csv = evidence("integrated-scans.csv");
        let mut a = Ch05Replay::new(&csv).unwrap();
        let mut b = Ch05Replay::new(&csv).unwrap();
        for _ in 0..126 {
            a.step(4).unwrap();
        }
        b.step(504).unwrap();
        assert_eq!(a.log_odds(), b.log_odds());
        b.set_occupied_probability(0.65).unwrap();
        assert_eq!(a.log_odds(), b.log_odds());
        assert_eq!(b.hits_to_occupy(), 1);
        assert_eq!(a.hits_to_occupy(), 4);
        assert_eq!(a.integrated(), 504);
    }

    /// The frozen header is the header of the saved grid.
    #[test]
    fn frozen_header_is_the_header_of_the_saved_grid() {
        let r = Ch05Replay::new(&evidence("integrated-scans.csv")).unwrap();
        let ours = r.grid_text();
        let theirs = evidence("final-grid.txt");
        let head = |s: &str| s.lines().take(12).map(String::from).collect::<Vec<_>>();
        assert_eq!(head(&ours), head(&theirs));
    }

    /// Native numbers the chapter quotes about the finished map (and which a check asserts on the
    /// compiled module).
    #[test]
    fn finished_oblique_map_numbers() {
        let mut r = Ch05Replay::new(&evidence("integrated-scans.csv")).unwrap();
        r.step(10_000).unwrap();
        assert_eq!(r.counts(), vec![14196, 902, 4502]); // map-metrics.yaml: free, occupied, unknown
        assert_eq!(r.phantom_cells(1.0).len() / 2, 1); // false_occupied_cells: 1
        r.set_occupied_probability(0.65).unwrap();
        let one_hit = r.phantom_cells(1.0).len() / 2;
        assert!(one_hit > 1, "one-hit occupancy should leave phantom cells, got {one_hit}");
    }

    #[test]
    fn beam_error_table_is_the_documented_one() {
        let eps = ch05_beam_error_table().unwrap();
        let half = TAU / 360.0 / 2.0; // fov / (2 N)
        assert!((eps[0] + half).abs() < 1e-12, "{}", eps[0]);
        assert!((eps[359] - half).abs() < 1e-12, "{}", eps[359]);
        assert!(eps[179].abs() < 3e-5, "{}", eps[179]);
        // strictly increasing: the published grid spreads 360 beams over the span of 359
        assert!(eps.windows(2).all(|w| w[1] > w[0]));
    }

    fn wrap(a: f64) -> f64 {
        (a + TAU / 2.0).rem_euclid(TAU) - TAU / 2.0
    }

    /// Two quantities, two formulas. The *range residual* of a beam, measured minus the range the
    /// wall gives at the published bearing, is `range * tan(incidence) * eps` (the quantity
    /// `NOTE-root-cause-analysis.md` fits); the distance of the mapped hit from the wall face is
    /// that times `cos(incidence)`, `range * sin(incidence) * eps`. Both are 0 at normal incidence.
    /// Beams 0 and 359 point straight behind, where the published grid is wrong by -8.7 and
    /// +8.7 mrad.
    #[test]
    fn the_wall_error_follows_the_formulas_and_the_correction_removes_it() {
        let eps = ch05_beam_error_table().unwrap();
        for inc_deg in [0.0f64, 20.0, 45.0, 70.0] {
            let alpha = inc_deg.to_radians();
            let d = 0.9;
            let w = ch05_beam_wall(d, alpha).unwrap();
            let (e, c, r, dr) = (w.error_published(), w.error_corrected(), w.ranges(), w.range_error());
            let phi = TAU / 2.0 - alpha;
            for j in [0usize, 359] {
                assert!(r[j].is_finite(), "beam {j} must reach the wall at {inc_deg} deg");
                let published = TAU / 2.0 - j as f64 * TAU / 359.0;
                let inc = wrap(published + eps[j] - phi).abs(); // incidence of the true beam
                let (tan_pred, sin_pred) = (r[j] * inc.tan() * eps[j].abs(), r[j] * inc.sin() * eps[j].abs());
                assert!((dr[j].abs() - tan_pred).abs() <= 0.03 * tan_pred + 1e-4,
                    "beam {j} at {inc_deg} deg: range residual {} against range*tan*eps {tan_pred}", dr[j]);
                assert!((e[j].abs() - sin_pred).abs() <= 0.03 * sin_pred + 1e-4,
                    "beam {j} at {inc_deg} deg: displacement {} against range*sin*eps {sin_pred}", e[j]);
            }
            assert!(c.iter().filter(|v| v.is_finite()).all(|v| v.abs() < 1e-5), "corrected at {inc_deg} deg");
        }
        // invisible at normal incidence; 4.4 cm of range at 45 degrees and the largest range
        let normal = ch05_beam_wall(1.0, 0.0).unwrap();
        assert!(normal.error_published()[0].abs() < 1e-3);
        assert!(normal.range_error()[0].abs() < 1e-3);
    }

    /// Opposite signs at the two ends of the index range (the asymmetry that put false cells
    /// outside the wall on some routes and not others, `NOTE-root-cause-analysis.md`).
    #[test]
    fn error_has_opposite_signs_at_the_two_ends_of_the_scan() {
        let w = ch05_beam_wall(1.5, 45f64.to_radians()).unwrap();
        let e = w.error_published();
        assert!(e[0].is_finite() && e[359].is_finite());
        assert!(e[0].signum() != e[359].signum(), "{} {}", e[0], e[359]);
    }

    #[test]
    fn a_wall_the_straight_behind_beam_cannot_reach_is_refused() {
        assert!(ch05_beam_wall(3.0, 45f64.to_radians()).is_err());
        let limit = ch05_beam_wall_max_incidence(3.0);
        assert!(ch05_beam_wall(3.0, limit - 1e-6).is_ok());
        assert!(ch05_beam_wall(3.0, limit + 1e-6).is_err());
    }

    #[test]
    fn bresenham_and_supercover_agree_on_an_axis_and_differ_on_a_diagonal() {
        let t = ch05_trace(0.5, 0.5, 5.5, 0.5, 1.0).unwrap();
        assert_eq!(t.bresenham(), t.supercover());
        assert!(t.only_bresenham().is_empty() && t.only_supercover().is_empty());
        let d = ch05_trace(0.1, 0.1, 9.9, 7.6, 1.0).unwrap();
        assert!(!d.only_supercover().is_empty());
        let (b, s) = (d.bresenham().len() / 2, d.supercover().len() / 2);
        assert_eq!(b, 10, "Bresenham has max(|dx|, |dy|) + 1 cells");
        assert!(s > b, "supercover cells {s}");
    }
}
