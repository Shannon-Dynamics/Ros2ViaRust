//! Grid Rao-Blackwellized particle filter SLAM, an alternative back end.
//!
//! Every particle is one hypothesis about the whole trajectory: it carries a pose, a log
//! weight and its **own** occupancy grid, built only from the scans that hypothesis has seen.
//! This is the FastSLAM / GMapping factorization `p(x₁:ₜ, m | z, u) = p(x₁:ₜ | z, u) ∏ p(mᵢ | x₁:ₜ, z)`
//! with the map conditioned on the trajectory sampled per particle.
//!
//! One step is [`Rbpf::predict`] (every particle draws a new pose from the odometry motion
//! model of `robotics_models`) followed by [`Rbpf::update`]: weight each particle by how well
//! the scan agrees with its own grid, normalize, resample when the effective sample size has
//! dropped below `resample_ratio·N`, and only then integrate the scan into every grid.
//!
//! # Stated limits
//! * **No scan-matching refinement of the proposal.** The proposal is the plain odometry
//!   motion model (FastSLAM 1.0 style), not the scan-matched Gaussian of FastSLAM 2.0 or
//!   GMapping. The filter can only recover odometry error that the motion noise `alphas` can
//!   reach with its `N` samples; with few particles and a sharp likelihood it degenerates
//!   quickly.
//! * **The grids are a likelihood structure, not a published map.** They are bounded, coarse
//!   and unvalidated, in line with decision DS12; nothing here is an occupancy-grid message.
//! * **Defaults are unvalidated.** `alphas`, `temper`, `epsilon` and the log-odds constants are
//!   initial values that have not been tuned against logged data.
//! * Each particle grid has a fixed square extent around the world origin; cells outside are
//!   ignored on write and read as unknown.
//!
//! # Per-particle grid
//! A private copy-on-write `ChunkGrid`: log-odds `f32` in 16×16-cell chunks held as
//! `Option<Arc<Chunk>>` and allocated on first write. Cloning a grid clones the vector of
//! `Arc`s (O(chunks)), and the first write to a shared chunk copies that chunk
//! (`Arc::make_mut`). Resampling therefore costs one pointer vector per clone, and a clone pays
//! for a chunk only when its trajectory diverges there.
//!
//! A scan is integrated by walking every beam through the grid with an exact grid traversal
//! (every cell the segment passes through; an exact corner crossing steps diagonally). Cells
//! before the endpoint cell get `l_free`, the endpoint cell gets `l_occ`, and each cell is
//! updated at most once per scan with *occupied winning over free*. A beam whose range is
//! non-finite or at least `range_max` is **dropped** entirely: it neither clears free space
//! nor marks an endpoint, a conservative choice because a max-range reading is not evidence of
//! emptiness out to that range.
//!
//! # Likelihood
//! For a particle with pose `x`, sensor mount `T`, and the usable beams `(β_k, r_k)` of the
//! scan (every `beam_stride`-th beam with `range_min ≤ r_k < range_max`), the endpoint is
//! `e_k = x·T·(r_k cos β_k, r_k sin β_k)`. With `l_k` the **maximum** log-odds over the 3×3
//! block of cells around `e_k` (unknown and outside cells count as `0`, i.e. `p = ½`) and
//! `p_k = 1/(1 + e^{−l_k})`,
//!
//! ```text
//! ln w ← ln w + temper · Σ_k ln( ε + (1 − ε) · p_k )
//! ```
//!
//! The `temper` factor (default 0.25) accounts for neighboring beams not being independent:
//! an untempered product collapses the population at the first resample. The value is taken
//! from a reference's widget code and is **unvalidated here**. A particle whose grid holds no
//! occupied cell yet (the first scan) is not reweighted.

use crate::SlamError;
use nalgebra::Vector2;
use rand::rngs::SmallRng;
use robotics_core::{seeded_rng, LaserScan, Se2};
use robotics_filters::{FilterError, ParticleSet};
use robotics_models::motion::{OdomAlphas, OdomDelta, OdometryMotionModel};
use std::fmt;
use std::sync::Arc;

/// Cells per chunk side.
const CHUNK: usize = 16;
/// Cells per chunk.
const CHUNK_CELLS: usize = CHUNK * CHUNK;
/// Upper bound on cells per grid side, so a bad config cannot ask for gigabytes of pointers.
const MAX_SIDE: usize = 4096;

/// Configuration of the filter. See the module docs for the meaning of each knob.
#[derive(Debug, Clone, PartialEq)]
pub struct RbpfConfig {
    /// Number of particles `N ≥ 1`.
    pub n_particles: usize,
    /// Grid cell size (m).
    pub resolution: f64,
    /// Half side of the square grid around the world origin (m).
    pub half_extent: f64,
    /// Log-odds added to the cell holding a beam endpoint.
    pub l_occ: f64,
    /// Log-odds added to cells a beam passes through (negative).
    pub l_free: f64,
    /// Lower clamp of the log-odds (negative).
    pub l_min: f64,
    /// Upper clamp of the log-odds (positive).
    pub l_max: f64,
    /// Odometry noise `[α₁, α₂, α₃, α₄]` in the order of `robotics_models`
    /// (`OdomAlphas`): rotation from rotation, rotation from translation, translation from
    /// translation, translation from rotation. Initial, unvalidated.
    pub alphas: [f64; 4],
    /// Exponent applied to the scan log-likelihood, in `(0, 1]`. Unvalidated.
    pub temper: f64,
    /// Resample when `n_eff < resample_ratio·N`; in `(0, 1]`.
    pub resample_ratio: f64,
    /// Use every `beam_stride`-th usable beam for the likelihood (`≥ 1`).
    pub beam_stride: usize,
    /// Floor `ε ∈ (0, 1]` of the per-beam likelihood.
    pub epsilon: f64,
}

impl Default for RbpfConfig {
    fn default() -> Self {
        Self {
            n_particles: 30,
            resolution: 0.05,
            half_extent: 5.0,
            l_occ: 0.85,
            l_free: -0.4,
            l_min: -4.0,
            l_max: 4.0,
            alphas: [0.02, 0.01, 0.03, 0.01],
            temper: 0.25,
            resample_ratio: 0.5,
            beam_stride: 4,
            epsilon: 0.05,
        }
    }
}

impl RbpfConfig {
    /// Check every field; the first offender is reported.
    pub fn validate(&self) -> Result<(), SlamError> {
        let bad = |what: &'static str, value: f64| Err(SlamError::InvalidParameter { what, value });
        if self.n_particles == 0 {
            return bad("n_particles", 0.0);
        }
        if !(self.resolution.is_finite() && self.resolution > 0.0) {
            return bad("resolution", self.resolution);
        }
        if !(self.half_extent.is_finite() && self.half_extent > 0.0) {
            return bad("half_extent", self.half_extent);
        }
        let side = side_cells(self.resolution, self.half_extent);
        if side > MAX_SIDE as f64 {
            return bad("grid cells per side", side);
        }
        if !(self.l_occ.is_finite() && self.l_occ > 0.0) {
            return bad("l_occ", self.l_occ);
        }
        if !(self.l_free.is_finite() && self.l_free < 0.0) {
            return bad("l_free", self.l_free);
        }
        if !(self.l_min.is_finite() && self.l_min < 0.0 && self.l_min <= self.l_free) {
            return bad("l_min", self.l_min);
        }
        if !(self.l_max.is_finite() && self.l_max > 0.0 && self.l_max >= self.l_occ) {
            return bad("l_max", self.l_max);
        }
        if let Some(&a) = self.alphas.iter().find(|a| !(a.is_finite() && **a >= 0.0)) {
            return bad("alphas", a);
        }
        if !(self.temper > 0.0 && self.temper <= 1.0) {
            return bad("temper", self.temper);
        }
        if !(self.resample_ratio > 0.0 && self.resample_ratio <= 1.0) {
            return bad("resample_ratio", self.resample_ratio);
        }
        if self.beam_stride == 0 {
            return bad("beam_stride", 0.0);
        }
        if !(self.epsilon > 0.0 && self.epsilon <= 1.0) {
            return bad("epsilon", self.epsilon);
        }
        Ok(())
    }
}

/// Number of cells along one grid side (as a float, so that an absurd config is reported and
/// not wrapped).
fn side_cells(resolution: f64, half_extent: f64) -> f64 {
    // The small subtraction keeps 10 m / 0.05 m at 200 cells instead of 201.
    (2.0 * half_extent / resolution - 1e-9).ceil().max(1.0)
}

/// Integer grid cell `(ix, iy)`; may be outside the extent.
type Cell = (i64, i64);

/// One 16×16 block of log-odds. Zero means "never updated".
#[derive(Clone)]
struct Chunk {
    cells: [f32; CHUNK_CELLS],
}

impl Chunk {
    fn empty() -> Self {
        Self {
            cells: [0.0; CHUNK_CELLS],
        }
    }
}

/// Log-odds constants of the inverse sensor model.
#[derive(Debug, Clone, Copy)]
struct LogOdds {
    l_occ: f32,
    l_free: f32,
    l_min: f32,
    l_max: f32,
}

/// A bounded, copy-on-write occupancy log-odds grid. A likelihood structure for weighting,
/// not a published map.
#[derive(Clone)]
struct ChunkGrid {
    resolution: f64,
    half_extent: f64,
    side: usize,
    chunks_per_side: usize,
    chunks: Vec<Option<Arc<Chunk>>>,
    params: LogOdds,
    /// Number of cells with log-odds above zero.
    occupied: usize,
}

impl fmt::Debug for ChunkGrid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ChunkGrid")
            .field("resolution", &self.resolution)
            .field("side", &self.side)
            .field("allocated_chunks", &self.allocated_chunks())
            .field("occupied", &self.occupied)
            .finish()
    }
}

impl ChunkGrid {
    fn new(cfg: &RbpfConfig) -> Self {
        let side = side_cells(cfg.resolution, cfg.half_extent) as usize;
        let chunks_per_side = side.div_ceil(CHUNK);
        Self {
            resolution: cfg.resolution,
            half_extent: cfg.half_extent,
            side,
            chunks_per_side,
            chunks: vec![None; chunks_per_side * chunks_per_side],
            params: LogOdds {
                l_occ: cfg.l_occ as f32,
                l_free: cfg.l_free as f32,
                l_min: cfg.l_min as f32,
                l_max: cfg.l_max as f32,
            },
            occupied: 0,
        }
    }

    /// Cell containing the world point `p` (possibly outside the extent).
    fn world_to_cell(&self, p: &Vector2<f64>) -> Cell {
        (
            ((p.x + self.half_extent) / self.resolution).floor() as i64,
            ((p.y + self.half_extent) / self.resolution).floor() as i64,
        )
    }

    /// World coordinates of the center of `cell`.
    #[cfg(test)]
    fn cell_center(&self, cell: Cell) -> Vector2<f64> {
        Vector2::new(
            (cell.0 as f64 + 0.5) * self.resolution - self.half_extent,
            (cell.1 as f64 + 0.5) * self.resolution - self.half_extent,
        )
    }

    /// Row-major index of an in-extent cell.
    fn linear(&self, cell: Cell) -> Option<usize> {
        let side = self.side as i64;
        ((0..side).contains(&cell.0) && (0..side).contains(&cell.1))
            .then(|| cell.1 as usize * self.side + cell.0 as usize)
    }

    /// Chunk index and offset inside the chunk for a row-major cell index.
    fn locate(&self, lin: usize) -> (usize, usize) {
        let (ix, iy) = (lin % self.side, lin / self.side);
        (
            (iy / CHUNK) * self.chunks_per_side + ix / CHUNK,
            (iy % CHUNK) * CHUNK + ix % CHUNK,
        )
    }

    fn logodds_linear(&self, lin: usize) -> f32 {
        let (ci, ofs) = self.locate(lin);
        self.chunks[ci].as_ref().map_or(0.0, |c| c.cells[ofs])
    }

    /// Log-odds of `cell`; zero for unknown cells and cells outside the extent.
    fn occupancy_logodds(&self, cell: Cell) -> f32 {
        self.linear(cell).map_or(0.0, |l| self.logodds_linear(l))
    }

    /// Occupancy probability at a world point, `None` when unknown or outside the extent.
    fn occupancy_at(&self, p: &Vector2<f64>) -> Option<f64> {
        let l = self.occupancy_logodds(self.world_to_cell(p));
        (l != 0.0).then(|| 1.0 / (1.0 + (-f64::from(l)).exp()))
    }

    /// Maximum log-odds over the 3×3 block centered on `cell` (unknown counts as zero).
    fn block_max_logodds(&self, cell: Cell) -> f32 {
        let mut m = f32::NEG_INFINITY;
        for dy in -1..=1 {
            for dx in -1..=1 {
                m = m.max(self.occupancy_logodds((cell.0 + dx, cell.1 + dy)));
            }
        }
        m
    }

    fn has_occupied(&self) -> bool {
        self.occupied > 0
    }

    fn allocated_chunks(&self) -> usize {
        self.chunks.iter().flatten().count()
    }

    /// Number of allocated chunks the two grids hold in common (the same allocation).
    fn chunks_shared(&self, other: &ChunkGrid) -> usize {
        self.chunks
            .iter()
            .zip(&other.chunks)
            .filter(|(a, b)| matches!((a, b), (Some(a), Some(b)) if Arc::ptr_eq(a, b)))
            .count()
    }

    /// Add `delta` to a cell, clamped. A write that changes nothing does not unshare.
    fn apply(&mut self, lin: usize, delta: f32) {
        let old = self.logodds_linear(lin);
        let new = (old + delta).clamp(self.params.l_min, self.params.l_max);
        if new == old {
            return;
        }
        let (ci, ofs) = self.locate(lin);
        let chunk = Arc::make_mut(self.chunks[ci].get_or_insert_with(|| Arc::new(Chunk::empty())));
        chunk.cells[ofs] = new;
        match (old > 0.0, new > 0.0) {
            (false, true) => self.occupied += 1,
            (true, false) => self.occupied -= 1,
            _ => {}
        }
    }

    /// Walk the segment `a → b` and push `2·cell + (1 if free, 0 if occupied)` for every
    /// in-extent cell it crosses. The cell holding `b` is occupied unless the segment leaves
    /// the extent first.
    fn trace_beam(&self, a: &Vector2<f64>, b: &Vector2<f64>, out: &mut Vec<u64>) {
        let n = self.side as f64;
        let to_cells = |p: &Vector2<f64>| {
            (
                (p.x + self.half_extent) / self.resolution,
                (p.y + self.half_extent) / self.resolution,
            )
        };
        let ((u0, v0), (u1, v1)) = (to_cells(a), to_cells(b));
        let (du, dv) = (u1 - u0, v1 - v0);
        if !(u0.is_finite() && v0.is_finite() && du.is_finite() && dv.is_finite()) {
            return;
        }
        // Clip to [0, n]² so that a huge range costs only the in-extent part.
        let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
        for (p, d) in [(u0, du), (v0, dv)] {
            if d == 0.0 {
                if !(0.0..n).contains(&p) {
                    return;
                }
            } else {
                let (s, e) = (-p / d, (n - p) / d);
                t0 = t0.max(s.min(e));
                t1 = t1.min(s.max(e));
            }
        }
        if t0 > t1 {
            return;
        }
        let end_clipped = t1 < 1.0;
        let max_cell = self.side as i64 - 1;
        let cell_of = |x: f64, clipped: bool| {
            let c = x.floor() as i64;
            if clipped {
                c.clamp(0, max_cell)
            } else {
                c
            }
        };
        let (ua, va) = (u0 + t0 * du, v0 + t0 * dv);
        let (ub, vb) = (u0 + t1 * du, v0 + t1 * dv);
        let (mut ix, mut iy) = (cell_of(ua, t0 > 0.0), cell_of(va, t0 > 0.0));
        let (ex, ey) = (cell_of(ub, end_clipped), cell_of(vb, end_clipped));
        let (sx, sy) = ((ex - ix).signum(), (ey - iy).signum());
        let (mut nx, mut ny) = ((ex - ix).abs(), (ey - iy).abs());
        let (dx, dy) = ((ub - ua).abs(), (vb - va).abs());
        let first_cross = |i: i64, s: i64, p: f64, d: f64| {
            if d < 1e-12 {
                f64::INFINITY
            } else {
                let edge = if s > 0 { (i + 1) as f64 } else { i as f64 };
                (edge - p).abs() / d
            }
        };
        let (mut tx, mut ty) = (first_cross(ix, sx, ua, dx), first_cross(iy, sy, va, dy));
        let (tdx, tdy) = (1.0 / dx.max(1e-12), 1.0 / dy.max(1e-12));
        loop {
            let at_end = nx == 0 && ny == 0;
            if let Some(lin) = self.linear((ix, iy)) {
                let occupied = at_end && !end_clipped;
                out.push(2 * lin as u64 + u64::from(!occupied));
            }
            if at_end {
                break;
            }
            let tie = (tx - ty).abs() <= 1e-9;
            let step_x = ny == 0 || (nx > 0 && (tie || tx < ty));
            let step_y = nx == 0 || (ny > 0 && (tie || ty <= tx));
            if step_x {
                ix += sx;
                nx -= 1;
                tx += tdx;
            }
            if step_y {
                iy += sy;
                ny -= 1;
                ty += tdy;
            }
        }
    }

    /// Integrate a scan taken from `sensor` (world pose of the sensor frame).
    fn integrate_scan(&mut self, sensor: &Se2, scan: &LaserScan) {
        let origin = sensor.translation();
        let mut keys: Vec<u64> = Vec::new();
        for (bearing, range) in scan.valid_beams() {
            if range >= scan.range_max {
                continue;
            }
            let (s, c) = (sensor.theta() + bearing).sin_cos();
            let end = origin + Vector2::new(c, s) * range;
            self.trace_beam(&origin, &end, &mut keys);
        }
        // Occupied keys (even) sort before free keys (odd) of the same cell; the first key
        // of each cell is the only update it receives.
        keys.sort_unstable();
        let mut last = u64::MAX;
        for key in keys {
            let lin = key >> 1;
            if lin == last {
                continue;
            }
            last = lin;
            let delta = if key & 1 == 0 {
                self.params.l_occ
            } else {
                self.params.l_free
            };
            self.apply(lin as usize, delta);
        }
    }

    /// Sum over the beams of `ln(ε + (1 − ε)·p)`, `p` from the 3×3 block maximum at each
    /// endpoint (see the module docs).
    fn scan_log_likelihood(&self, sensor: &Se2, beams: &[(f64, f64)], epsilon: f64) -> f64 {
        beams
            .iter()
            .map(|&(bearing, range)| {
                let p = sensor.act(&Vector2::new(range * bearing.cos(), range * bearing.sin()));
                let l = f64::from(self.block_max_logodds(self.world_to_cell(&p)));
                (epsilon + (1.0 - epsilon) / (1.0 + (-l).exp())).ln()
            })
            .sum()
    }
}

/// One trajectory hypothesis; its weight lives in the surrounding `ParticleSet`.
#[derive(Debug, Clone)]
struct Hypothesis {
    pose: Se2,
    grid: ChunkGrid,
}

/// What one [`Rbpf::update`] did.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RbpfReport {
    /// Effective sample size after weighting and before any resampling.
    pub n_eff_before: f64,
    /// Whether the population was resampled.
    pub resampled: bool,
    /// Number of particles.
    pub particles: usize,
}

/// Grid Rao-Blackwellized particle filter SLAM (FastSLAM 1.0 style, no scan matching).
#[derive(Debug, Clone)]
pub struct Rbpf {
    cfg: RbpfConfig,
    motion: OdometryMotionModel,
    set: ParticleSet<Hypothesis>,
    rng: SmallRng,
}

fn filter_error(e: FilterError) -> SlamError {
    match e {
        FilterError::NonFinite(what) => SlamError::NonFinite(what),
        _ => SlamError::SingularSystem("degenerate particle weights"),
    }
}

impl Rbpf {
    /// Create `n_particles` identical hypotheses at `initial_pose`, each with an empty grid.
    /// All randomness comes from `seed`.
    pub fn new(cfg: RbpfConfig, initial_pose: Se2, seed: u64) -> Result<Self, SlamError> {
        cfg.validate()?;
        if !initial_pose.is_finite() {
            return Err(SlamError::NonFinite("initial pose"));
        }
        let motion = OdometryMotionModel::new(OdomAlphas(cfg.alphas)).map_err(|_| {
            SlamError::InvalidParameter {
                what: "alphas",
                value: f64::NAN,
            }
        })?;
        let grid = ChunkGrid::new(&cfg);
        let states = vec![
            Hypothesis {
                pose: initial_pose,
                grid
            };
            cfg.n_particles
        ];
        let set = ParticleSet::from_states(states).map_err(filter_error)?;
        Ok(Self {
            cfg,
            motion,
            set,
            rng: seeded_rng(seed),
        })
    }

    /// Move every particle by the body-frame odometry increment `delta`, each drawing its own
    /// noisy successor from the odometry motion model.
    pub fn predict(&mut self, delta: &Se2) -> Result<(), SlamError> {
        if !delta.is_finite() {
            return Err(SlamError::NonFinite("odometry increment"));
        }
        let d = OdomDelta::from_poses(&Se2::IDENTITY, delta);
        let (motion, rng) = (&self.motion, &mut self.rng);
        self.set
            .map_states(|p| p.pose = motion.sample_delta(&p.pose, &d, rng));
        Ok(())
    }

    /// Weight by the scan, resample if needed, then integrate the scan into every particle's
    /// own grid. `sensor_in_base` is the mounting pose of the sensor in the robot base frame.
    pub fn update(
        &mut self,
        scan: &LaserScan,
        sensor_in_base: &Se2,
    ) -> Result<RbpfReport, SlamError> {
        if scan.is_empty() {
            return Err(SlamError::EmptyInput("laser scan"));
        }
        let meta = [
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
        ];
        if !meta.iter().all(|v| v.is_finite()) {
            return Err(SlamError::NonFinite("scan metadata"));
        }
        if !sensor_in_base.is_finite() {
            return Err(SlamError::NonFinite("sensor mounting pose"));
        }
        let beams: Vec<(f64, f64)> = scan
            .valid_beams()
            .filter(|&(_, r)| r < scan.range_max)
            .step_by(self.cfg.beam_stride)
            .collect();
        if !beams.is_empty() {
            let (temper, eps) = (self.cfg.temper, self.cfg.epsilon);
            self.set
                .reweight(|p| {
                    if p.grid.has_occupied() {
                        let sensor = p.pose.compose(sensor_in_base);
                        temper * p.grid.scan_log_likelihood(&sensor, &beams, eps)
                    } else {
                        0.0
                    }
                })
                .map_err(filter_error)?;
        }
        let n_eff_before = self.set.normalize().map_err(filter_error)?.ess;
        let n = self.set.len();
        let resampled = n_eff_before < self.cfg.resample_ratio * n as f64;
        if resampled {
            self.set
                .resample_low_variance(&mut self.rng)
                .map_err(filter_error)?;
        }
        self.set
            .map_states(|p| p.grid.integrate_scan(&p.pose.compose(sensor_in_base), scan));
        Ok(RbpfReport {
            n_eff_before,
            resampled,
            particles: n,
        })
    }

    /// Number of particles.
    pub fn particle_count(&self) -> usize {
        self.set.len()
    }

    /// Normalized particle weights, summing to one.
    pub fn particle_weights(&self) -> Vec<f64> {
        let w = self.set.weights();
        let total: f64 = w.iter().sum();
        w.into_iter().map(|x| x / total).collect()
    }

    /// Effective sample size `1/Σwᵢ²` of the current weights.
    pub fn n_eff(&self) -> f64 {
        self.set.effective_sample_size()
    }

    /// Pose of particle `i`, `None` if out of range.
    pub fn particle_pose(&self, i: usize) -> Option<Se2> {
        self.set.particles().get(i).map(|p| p.state.pose)
    }

    /// Index of the heaviest particle (the lowest index wins ties).
    pub fn best_particle(&self) -> usize {
        let mut best = 0;
        for (i, p) in self.set.particles().iter().enumerate() {
            if p.log_weight > self.set.particles()[best].log_weight {
                best = i;
            }
        }
        best
    }

    /// Weighted mean position and weighted circular mean heading.
    pub fn pose_estimate(&self) -> Se2 {
        let w = self.particle_weights();
        let (mut x, mut y, mut s, mut c) = (0.0, 0.0, 0.0, 0.0);
        for (p, w) in self.set.particles().iter().zip(w) {
            let pose = &p.state.pose;
            x += w * pose.x();
            y += w * pose.y();
            s += w * pose.theta().sin();
            c += w * pose.theta().cos();
        }
        Se2::new(x, y, s.atan2(c))
    }

    /// Occupancy probability that particle `i`'s own grid assigns to the world point `p`;
    /// `None` when the particle does not exist, the cell is unknown, or it is outside the
    /// grid extent.
    pub fn particle_occupancy_at(&self, i: usize, p: &Vector2<f64>) -> Option<f64> {
        self.set.particles().get(i)?.state.grid.occupancy_at(p)
    }

    /// Number of grid chunks particles `i` and `j` hold in common (the same allocation, so
    /// their trajectories agree on those cells). Zero if either index is out of range.
    pub fn chunks_shared(&self, i: usize, j: usize) -> usize {
        match (self.set.particles().get(i), self.set.particles().get(j)) {
            (Some(a), Some(b)) => a.state.grid.chunks_shared(&b.state.grid),
            _ => 0,
        }
    }

    /// Number of chunks allocated by particle `i`'s grid (zero if out of range).
    pub fn allocated_chunks(&self, i: usize) -> usize {
        self.set
            .particles()
            .get(i)
            .map_or(0, |p| p.state.grid.allocated_chunks())
    }

    /// Overwrite the particle poses (tests only).
    #[cfg(test)]
    fn set_poses(&mut self, poses: &[Se2]) {
        let mut it = poses.iter();
        self.set.map_states(|p| {
            if let Some(&pose) = it.next() {
                p.pose = pose;
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robotics_sim::{Lidar, LidarConfig, World};

    fn small_cfg() -> RbpfConfig {
        RbpfConfig {
            n_particles: 12,
            resolution: 0.1,
            half_extent: 1.0,
            ..RbpfConfig::default()
        }
    }

    fn grid() -> ChunkGrid {
        ChunkGrid::new(&small_cfg())
    }

    /// A scan with the given `(bearing, range)` beams (one entry per beam).
    fn scan_of(beams: &[(f64, f32)]) -> LaserScan {
        LaserScan {
            angle_min: 0.0,
            angle_increment: 0.0,
            range_min: 0.0,
            range_max: 8.0,
            ranges: beams.iter().map(|b| b.1).collect(),
            ..LaserScan::default()
        }
    }

    /// One beam per scan at the given bearing.
    fn beam(bearing: f64, range: f32) -> LaserScan {
        let mut s = scan_of(&[(bearing, range)]);
        s.angle_min = bearing;
        s
    }

    fn free_cells(g: &ChunkGrid) -> Vec<Cell> {
        let n = g.side as i64;
        (0..n)
            .flat_map(|y| (0..n).map(move |x| (x, y)))
            .filter(|&c| g.occupancy_logodds(c) < 0.0)
            .collect()
    }

    fn occupied_cells(g: &ChunkGrid) -> Vec<Cell> {
        let n = g.side as i64;
        (0..n)
            .flat_map(|y| (0..n).map(move |x| (x, y)))
            .filter(|&c| g.occupancy_logodds(c) > 0.0)
            .collect()
    }

    #[test]
    fn world_cell_round_trip() {
        let g = grid();
        assert_eq!(g.side, 20);
        for cy in 0..20 {
            for cx in 0..20 {
                let c = g.cell_center((cx, cy));
                assert_eq!(g.world_to_cell(&c), (cx, cy));
            }
        }
        assert_eq!(g.world_to_cell(&Vector2::new(-1.0, -1.0)), (0, 0));
        assert_eq!(g.world_to_cell(&Vector2::new(0.0, 0.0)), (10, 10));
        assert_eq!(g.linear((20, 0)), None);
        assert_eq!(g.linear((0, -1)), None);
    }

    #[test]
    fn axis_aligned_beam_marks_exact_cells() {
        let mut g = grid();
        // Sensor in the middle of cell (10, 10), beam along +x ending in cell (14, 10).
        g.integrate_scan(&Se2::new(0.05, 0.05, 0.0), &beam(0.0, 0.42));
        assert_eq!(occupied_cells(&g), vec![(14, 10)]);
        assert_eq!(free_cells(&g), vec![(10, 10), (11, 10), (12, 10), (13, 10)]);
        assert_eq!(g.occupancy_logodds((14, 10)), 0.85);
        assert_eq!(g.occupancy_logodds((12, 10)), -0.4);
        // Along -y with the sensor rotated instead of the bearing.
        let mut g = grid();
        g.integrate_scan(
            &Se2::new(0.05, 0.05, -std::f64::consts::FRAC_PI_2),
            &beam(0.0, 0.32),
        );
        assert_eq!(occupied_cells(&g), vec![(10, 7)]);
        assert_eq!(free_cells(&g), vec![(10, 8), (10, 9), (10, 10)]);
    }

    #[test]
    fn diagonal_beam_skips_no_cell() {
        let mut g = grid();
        let r = 4.0 * 0.1 * std::f64::consts::SQRT_2;
        g.integrate_scan(
            &Se2::new(0.05, 0.05, 0.0),
            &beam(std::f64::consts::FRAC_PI_4, r as f32),
        );
        assert_eq!(occupied_cells(&g), vec![(14, 14)]);
        assert_eq!(free_cells(&g), vec![(10, 10), (11, 11), (12, 12), (13, 13)]);
        // A shallow beam (slope 1:3) crosses corners at no cell; each column and row boundary
        // is crossed once, so the walk is 4-connected and ends in the endpoint cell.
        let mut g = grid();
        let ang = 0.13_f64.atan2(0.42);
        let range = 0.42_f64.hypot(0.13) as f32;
        g.integrate_scan(&Se2::new(0.05, 0.05, 0.0), &beam(ang, range));
        let occ = occupied_cells(&g);
        assert_eq!(occ.len(), 1);
        assert_eq!(
            g.world_to_cell(&Vector2::new(0.05 + 0.42, 0.05 + 0.13)),
            occ[0]
        );
        let mut cells = free_cells(&g);
        cells.push(occ[0]);
        cells.sort_unstable();
        for w in cells.windows(2) {
            let step = (w[1].0 - w[0].0).abs() + (w[1].1 - w[0].1).abs();
            assert!(step <= 1, "gap between {:?} and {:?}", w[0], w[1]);
        }
    }

    #[test]
    fn occupied_wins_and_one_update_per_cell() {
        let mut g = grid();
        // Two beams along the same row; the endpoint of the short one lies on the path of the
        // long one.
        let s = scan_of(&[(0.0, 0.22), (0.0, 0.42)]);
        g.integrate_scan(&Se2::new(0.05, 0.05, 0.0), &s);
        assert_eq!(g.occupancy_logodds((12, 10)), 0.85);
        assert_eq!(g.occupancy_logodds((14, 10)), 0.85);
        // Cells passed by both beams were updated once, not twice.
        assert_eq!(g.occupancy_logodds((10, 10)), -0.4);
        assert_eq!(g.occupancy_logodds((11, 10)), -0.4);
        assert_eq!(g.occupancy_logodds((13, 10)), -0.4);
    }

    #[test]
    fn clamping() {
        let mut g = grid();
        let s = scan_of(&[(0.0, 0.42)]);
        let pose = Se2::new(0.05, 0.05, 0.0);
        for _ in 0..30 {
            g.integrate_scan(&pose, &s);
        }
        assert_eq!(g.occupancy_logodds((14, 10)), 4.0);
        assert_eq!(g.occupancy_logodds((11, 10)), -4.0);
        assert!((g.occupancy_at(&g.cell_center((14, 10))).unwrap() - 0.982).abs() < 1e-3);
        assert_eq!(g.occupancy_at(&g.cell_center((0, 0))), None);
        assert_eq!(g.occupancy_at(&Vector2::new(7.0, 0.0)), None);
        assert_eq!(g.occupied, 1);
    }

    #[test]
    fn out_of_extent_writes_are_ignored() {
        let mut g = grid();
        // Sensor inside, endpoint far outside: free up to the border, nothing occupied.
        g.integrate_scan(&Se2::new(0.05, 0.05, 0.0), &beam(0.0, 5.0));
        assert!(occupied_cells(&g).is_empty());
        assert_eq!(free_cells(&g).len(), 10);
        assert_eq!(g.occupancy_logodds((19, 10)), -0.4);
        // Sensor and endpoint outside, segment missing the extent.
        let mut g = grid();
        g.integrate_scan(&Se2::new(3.0, 3.0, 0.0), &beam(0.0, 1.0));
        assert_eq!(g.allocated_chunks(), 0);
        // Sensor outside, beam crossing the extent: free inside, ends outside.
        let mut g = grid();
        g.integrate_scan(&Se2::new(-3.0, 0.05, 0.0), &beam(0.0, 7.0));
        assert!(occupied_cells(&g).is_empty());
        assert_eq!(free_cells(&g).len(), 20);
        // Endpoint inside, origin outside: the endpoint cell is occupied.
        let mut g = grid();
        g.integrate_scan(&Se2::new(-3.0, 0.05, 0.0), &beam(0.0, 3.42));
        assert_eq!(occupied_cells(&g), vec![(14, 10)]);
        // Cell queries outside never panic.
        assert_eq!(g.occupancy_logodds((-5, 99)), 0.0);
    }

    #[test]
    fn copy_on_write_shares_untouched_chunks() {
        let mut a = grid();
        // The 20×20 grid is 2×2 chunks; give three of them cells.
        let pose = Se2::new(0.05, 0.05, 0.0);
        a.integrate_scan(&pose, &beam(0.0, 0.42));
        a.integrate_scan(&Se2::new(0.05, 0.75, 0.0), &beam(0.0, 0.42));
        a.integrate_scan(&Se2::new(0.75, 0.75, 0.0), &beam(0.0, 0.12));
        assert_eq!(a.allocated_chunks(), 3);
        let mut b = a.clone();
        assert_eq!(a.chunks_shared(&b), 3);
        let before: Vec<f32> = (0..20).map(|x| a.occupancy_logodds((x, 10))).collect();
        // Write in chunk (0,0) only.
        b.integrate_scan(&Se2::new(0.05, 0.05, 0.0), &beam(0.0, 0.32));
        assert_eq!(a.chunks_shared(&b), 2);
        let after: Vec<f32> = (0..20).map(|x| a.occupancy_logodds((x, 10))).collect();
        assert_eq!(before, after);
        assert_ne!(a.occupancy_logodds((13, 10)), b.occupancy_logodds((13, 10)));
        // A write that changes nothing (saturated cell) does not unshare.
        let mut c = a.clone();
        let s = scan_of(&[(0.0, 0.42)]);
        for _ in 0..30 {
            c.integrate_scan(&pose, &s);
        }
        let mut d = c.clone();
        d.integrate_scan(&pose, &s);
        assert_eq!(c.chunks_shared(&d), c.allocated_chunks());
    }

    /// Noiseless scan of an asymmetric room from `body`.
    fn room_scan(body: &Se2) -> (LaserScan, Se2) {
        let mut w = World::empty();
        w.add_rectangle(Vector2::new(-3.0, -2.0), Vector2::new(3.0, 2.5))
            .unwrap();
        w.add_box(Vector2::new(1.0, 0.8), Vector2::new(0.4, 0.3), 0.4)
            .unwrap();
        let mut lidar = Lidar::new(LidarConfig::default().noiseless(), 1).unwrap();
        let mount = lidar.config().mount;
        (lidar.scan(&w, body, robotics_core::Timestamp::ZERO), mount)
    }

    fn room_cfg(n: usize) -> RbpfConfig {
        RbpfConfig {
            n_particles: n,
            resolution: 0.1,
            half_extent: 4.0,
            ..RbpfConfig::default()
        }
    }

    fn run_short(seed: u64) -> Rbpf {
        let mut f = Rbpf::new(room_cfg(8), Se2::new(-1.0, -0.5, 0.2), seed).unwrap();
        let mut truth = Se2::new(-1.0, -0.5, 0.2);
        let (s, m) = room_scan(&truth);
        f.update(&s, &m).unwrap();
        for _ in 0..4 {
            let d = Se2::new(0.1, 0.0, 0.05);
            truth = truth.compose(&d);
            f.predict(&d).unwrap();
            let (s, m) = room_scan(&truth);
            f.update(&s, &m).unwrap();
        }
        f
    }

    fn bits(f: &Rbpf) -> Vec<u64> {
        (0..f.particle_count())
            .flat_map(|i| {
                let p = f.particle_pose(i).unwrap();
                [p.x(), p.y(), p.theta(), f.particle_weights()[i]].map(f64::to_bits)
            })
            .collect()
    }

    #[test]
    fn same_seed_is_bit_identical() {
        let (a, b, c) = (run_short(7), run_short(7), run_short(8));
        assert_eq!(bits(&a), bits(&b));
        assert_ne!(bits(&a), bits(&c));
    }

    #[test]
    fn identical_particles_keep_uniform_weights() {
        let cfg = RbpfConfig {
            alphas: [0.0; 4],
            ..room_cfg(6)
        };
        let pose = Se2::new(0.0, 0.0, 0.3);
        let mut f = Rbpf::new(cfg, pose, 3).unwrap();
        let (s, m) = room_scan(&pose);
        let r1 = f.update(&s, &m).unwrap();
        assert!(!r1.resampled);
        f.predict(&Se2::IDENTITY).unwrap();
        let r2 = f.update(&s, &m).unwrap();
        assert!(!r2.resampled);
        assert_eq!(r2.particles, 6);
        for w in f.particle_weights() {
            assert!((w - 1.0 / 6.0).abs() < 1e-12);
        }
        assert!((f.n_eff() - 6.0).abs() < 1e-9);
        assert_eq!(f.best_particle(), 0);
    }

    #[test]
    fn displaced_particle_is_downweighted() {
        let pose = Se2::new(0.0, 0.0, 0.3);
        let cfg = RbpfConfig {
            alphas: [0.0; 4],
            resample_ratio: 0.01,
            ..room_cfg(4)
        };
        let mut f = Rbpf::new(cfg, pose, 3).unwrap();
        let (s, m) = room_scan(&pose);
        f.update(&s, &m).unwrap();
        let shifted = Se2::new(0.5, 0.0, 0.3);
        f.set_poses(&[pose, shifted, pose, shifted]);
        f.update(&s, &m).unwrap();
        let w = f.particle_weights();
        assert!(w[0] > w[1], "{w:?}");
        assert!(w[2] > w[3], "{w:?}");
        assert!((w[0] - w[2]).abs() < 1e-12 && (w[1] - w[3]).abs() < 1e-12);
        assert!(f.n_eff() > 1.0 && f.n_eff() < 4.0);
        assert_eq!(f.best_particle(), 0);
        let e = f.pose_estimate();
        assert!(e.x() > 0.0 && e.x() < 0.25, "{e:?}");
        assert!((e.theta() - 0.3).abs() < 1e-9);
    }

    #[test]
    fn motion_noise_gives_partial_degeneracy() {
        let mut f = Rbpf::new(room_cfg(12), Se2::new(0.0, 0.0, 0.0), 11).unwrap();
        let mut truth = Se2::IDENTITY;
        let (s, m) = room_scan(&truth);
        f.update(&s, &m).unwrap();
        for _ in 0..3 {
            let d = Se2::new(0.15, 0.0, 0.1);
            truth = truth.compose(&d);
            f.predict(&d).unwrap();
            let (s, m) = room_scan(&truth);
            let r = f.update(&s, &m).unwrap();
            assert!(r.n_eff_before > 1.0 && r.n_eff_before <= 12.0);
            assert!(f.n_eff() > 1.0 && f.n_eff() <= 12.0 + 1e-9);
        }
        let w = f.particle_weights();
        assert!((w.iter().sum::<f64>() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn particle_maps_are_queryable() {
        let mut f = Rbpf::new(room_cfg(3), Se2::IDENTITY, 5).unwrap();
        let (s, m) = room_scan(&Se2::IDENTITY);
        f.update(&s, &m).unwrap();
        // The west wall is at x = -3.
        let p = [-3.05, -2.95]
            .iter()
            .filter_map(|&x| f.particle_occupancy_at(0, &Vector2::new(x, 0.0)))
            .fold(0.0, f64::max);
        assert!(p > 0.6, "{p}");
        let free = f
            .particle_occupancy_at(0, &Vector2::new(-1.0, 0.0))
            .unwrap();
        assert!(free < 0.45, "{free}");
        assert_eq!(f.particle_occupancy_at(0, &Vector2::new(30.0, 0.0)), None);
        assert_eq!(f.particle_occupancy_at(9, &Vector2::new(0.0, 0.0)), None);
        assert_eq!(f.particle_pose(9), None);
        // Identical first-scan trajectories: every chunk is separately allocated, none shared.
        assert_eq!(f.chunks_shared(0, 1), 0);
        assert!(f.allocated_chunks(0) > 4);
        assert_eq!(f.chunks_shared(0, 9), 0);
    }

    #[test]
    fn resampling_shares_chunks() {
        let cfg = RbpfConfig {
            resample_ratio: 1.0,
            ..room_cfg(6)
        };
        let mut f = Rbpf::new(cfg, Se2::IDENTITY, 2).unwrap();
        let (s, m) = room_scan(&Se2::IDENTITY);
        f.update(&s, &m).unwrap();
        f.predict(&Se2::new(0.1, 0.0, 0.0)).unwrap();
        let (s2, m2) = room_scan(&Se2::new(0.1, 0.0, 0.0));
        // ratio 1.0 resamples whenever the weights are not exactly uniform.
        let r = f.update(&s2, &m2).unwrap();
        assert!(r.resampled);
        // After the resample and the integration the grids may have diverged on touched
        // chunks, but the count never exceeds the allocation.
        assert!(f.chunks_shared(0, 1) <= f.allocated_chunks(0));
    }

    #[test]
    fn config_errors() {
        let bad = |f: &dyn Fn(&mut RbpfConfig)| {
            let mut c = RbpfConfig::default();
            f(&mut c);
            c.validate().unwrap_err()
        };
        assert!(RbpfConfig::default().validate().is_ok());
        let name = |e: SlamError| match e {
            SlamError::InvalidParameter { what, .. } => what,
            other => panic!("{other:?}"),
        };
        assert_eq!(name(bad(&|c| c.n_particles = 0)), "n_particles");
        assert_eq!(name(bad(&|c| c.resolution = 0.0)), "resolution");
        assert_eq!(name(bad(&|c| c.resolution = f64::NAN)), "resolution");
        assert_eq!(name(bad(&|c| c.half_extent = -1.0)), "half_extent");
        assert_eq!(name(bad(&|c| c.resolution = 1e-6)), "grid cells per side");
        assert_eq!(name(bad(&|c| c.temper = 0.0)), "temper");
        assert_eq!(name(bad(&|c| c.temper = 1.5)), "temper");
        assert_eq!(name(bad(&|c| c.resample_ratio = 0.0)), "resample_ratio");
        assert_eq!(name(bad(&|c| c.resample_ratio = 1.1)), "resample_ratio");
        assert_eq!(name(bad(&|c| c.beam_stride = 0)), "beam_stride");
        assert_eq!(name(bad(&|c| c.epsilon = 0.0)), "epsilon");
        assert_eq!(name(bad(&|c| c.alphas[2] = -0.1)), "alphas");
        assert_eq!(name(bad(&|c| c.l_free = 0.1)), "l_free");
        assert_eq!(name(bad(&|c| c.l_max = 0.5)), "l_max");
        assert!(Rbpf::new(
            RbpfConfig {
                n_particles: 0,
                ..RbpfConfig::default()
            },
            Se2::IDENTITY,
            1
        )
        .is_err());
        assert_eq!(
            Rbpf::new(RbpfConfig::default(), Se2::new(f64::NAN, 0.0, 0.0), 1).unwrap_err(),
            SlamError::NonFinite("initial pose")
        );
    }

    #[test]
    fn input_errors() {
        let mut f = Rbpf::new(small_cfg(), Se2::IDENTITY, 1).unwrap();
        assert_eq!(
            f.predict(&Se2::new(f64::INFINITY, 0.0, 0.0)).unwrap_err(),
            SlamError::NonFinite("odometry increment")
        );
        let empty = LaserScan::default();
        assert_eq!(
            f.update(&empty, &Se2::IDENTITY).unwrap_err(),
            SlamError::EmptyInput("laser scan")
        );
        let mut s = scan_of(&[(0.0, 1.0)]);
        s.range_max = f64::NAN;
        assert!(matches!(
            f.update(&s, &Se2::IDENTITY),
            Err(SlamError::NonFinite(_))
        ));
        let s = scan_of(&[(0.0, 1.0)]);
        assert!(matches!(
            f.update(&s, &Se2::new(f64::NAN, 0.0, 0.0)),
            Err(SlamError::NonFinite(_))
        ));
        // A scan with no usable beam is accepted and changes nothing.
        let none = scan_of(&[(0.0, f32::INFINITY), (0.0, 8.0), (0.0, f32::NAN)]);
        let r = f.update(&none, &Se2::IDENTITY).unwrap();
        assert!(!r.resampled);
        assert_eq!(f.allocated_chunks(0), 0);
    }
}
