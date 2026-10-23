//! Chapter 03 bindings: sensor models and the simulator (see web/design/Chapter-03.md).
//!
//! Two halves, both thin wrappers over crate code:
//!
//! * the simulator `robotics_sim` (`World::rusty_arena`, `World::raycast`, `Lidar`,
//!   `RustySim`), which produces scans a reader can replay from a seed;
//! * the sensor models of `robotics_models::sensor` (beam mixture with EM, likelihood field
//!   over the exact EDT, range-bearing landmark model, tempering) that say how probable a
//!   reading is.
//!
//! Three pieces of code in this file are **not** crate algorithms, and each says so where
//! it lives: (1) `rasterize`, the 15-line wiring that turns the arena's segments into the
//! occupancy mask `DistanceGrid::from_occupancy` expects (the repository has no rasterizer
//! yet; `docs/SOURCE_MAPPING.md` Ch. 10 assigns it to Phase 3), (2) the *data generator*
//! `generate_beam_data`, which draws readings from the beam mixture exactly the way the
//! crate's own EM test does (the crate has a density and a fit, but no sampler), and (3)
//! the *strawman* naive bearing residual in the landmark functions, which exists only so
//! the chapter can measure the wrapped residual against it. None of the three is code the
//! robot runs.
//!
//! Flat-array layouts are documented on each function. Every stochastic function takes a
//! `seed: u64`.

use robotics_core::nalgebra::Vector2;
use robotics_core::{seeded_rng, LaserScan, Se2, Timestamp};
use robotics_filters::particle::effective_sample_size;
use robotics_models::sensor::beam::{BeamModel, BeamModelParams};
use robotics_models::sensor::landmark::{LandmarkModel, RangeBearing};
use robotics_models::sensor::likelihood_field::{
    DistanceField, DistanceGrid, LikelihoodFieldModel, LikelihoodFieldParams,
};
use robotics_models::sensor::normal_pdf;
use robotics_models::sensor::tempering::Tempering;
use robotics_sim::{Lidar, LidarConfig, NoReturnPolicy, RustyConfig, RustySim, Trajectory, World};
use std::f64::consts::TAU;
use std::sync::OnceLock;
use wasm_bindgen::prelude::*;

// ---------------------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------------------

fn err<E: std::fmt::Display>(e: E) -> String {
    e.to_string()
}

/// FNV-1a over bytes: a 64-bit fingerprint, used only to show that two runs are
/// bit-identical (equal fingerprints) or not.
fn fnv1a(bytes: impl IntoIterator<Item = u8>) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

fn hash_ranges(ranges: &[f32]) -> u64 {
    fnv1a(ranges.iter().flat_map(|r| r.to_bits().to_le_bytes()))
}

fn lidar_config(sigma: f64, dropout: f64, book_encoding: bool) -> LidarConfig {
    LidarConfig {
        range_sigma: sigma,
        dropout_probability: dropout,
        no_return: if book_encoding {
            NoReturnPolicy::MaxRange
        } else {
            NoReturnPolicy::Infinity
        },
        ..LidarConfig::default()
    }
}

/// A `LaserScan` with the default LiDAR's 360-beam metadata around measured `ranges`
/// (`NaN` and `±inf` allowed).
fn scan_from_ranges(ranges: &[f64]) -> Result<LaserScan, String> {
    let cfg = LidarConfig::default();
    if ranges.len() != cfg.beams {
        return Err(format!("expected {} ranges, got {}", cfg.beams, ranges.len()));
    }
    Ok(LaserScan {
        angle_min: cfg.angle_min,
        angle_increment: cfg.angle_increment(),
        range_min: cfg.range_min,
        range_max: cfg.range_max,
        ranges: ranges.iter().map(|r| *r as f32).collect(),
        ..Default::default()
    })
}

fn mount() -> Se2 {
    LidarConfig::default().mount
}

// ---------------------------------------------------------------------------------------
// The simulator half: world, scan, replay
// ---------------------------------------------------------------------------------------

/// The segments of `World::rusty_arena()` as `[ax, ay, bx, by, …]` (21 segments).
#[wasm_bindgen]
pub fn ch03_arena_segments() -> Vec<f64> {
    World::rusty_arena()
        .segments()
        .iter()
        .flat_map(|s| [s.a.x, s.a.y, s.b.x, s.b.y])
        .collect()
}

/// One LiDAR sweep in the arena.
#[wasm_bindgen]
pub struct Ch03Scan {
    ranges: Vec<f64>,
    ideal: Vec<f64>,
    valid: u32,
    lost: u32,
    hash: u64,
}

#[wasm_bindgen]
impl Ch03Scan {
    /// The 360 measured ranges (m), beam 0 at −π, 1° steps. `+inf` = no return,
    /// `−inf` = too close (REP 117), or `range_max` for no return in the book encoding.
    #[wasm_bindgen(getter)]
    pub fn ranges(&self) -> Vec<f64> {
        self.ranges.clone()
    }
    /// The noise-free ray-cast distances `z*` for the same pose (`+inf` = nothing within
    /// 8 m).
    #[wasm_bindgen(getter)]
    pub fn ideal(&self) -> Vec<f64> {
        self.ideal.clone()
    }
    /// Beams with a usable range (`LaserScan::is_valid`).
    #[wasm_bindgen(getter)]
    pub fn valid(&self) -> u32 {
        self.valid
    }
    /// Beams that had a wall within range (`z*` finite) but returned nothing usable
    /// (dropouts, or noise pushing the range out of `[range_min, range_max]`).
    #[wasm_bindgen(getter)]
    pub fn lost(&self) -> u32 {
        self.lost
    }
    /// FNV-1a fingerprint of the `f32` bit patterns of the ranges: equal iff bit-identical.
    #[wasm_bindgen(getter)]
    pub fn hash(&self) -> u64 {
        self.hash
    }
}

/// Simulate one scan of `World::rusty_arena()` from the robot pose `(x, y, theta)` with the
/// default Rusty LiDAR (360 beams, 0.12–8 m, mounted 0.05 m ahead of the base), range noise
/// `sigma` (m), dropout probability `dropout`, and the LiDAR noise stream seeded by `seed`.
/// `book_encoding` selects `NoReturnPolicy::MaxRange` instead of REP 117 `+inf`.
#[wasm_bindgen]
pub fn ch03_lidar_scan(
    x: f64,
    y: f64,
    theta: f64,
    sigma: f64,
    dropout: f64,
    seed: u64,
    book_encoding: bool,
) -> Result<Ch03Scan, String> {
    let world = World::rusty_arena();
    let body = Se2::new(x, y, theta);
    let mut lidar = Lidar::new(lidar_config(sigma, dropout, book_encoding), seed).map_err(err)?;
    let ideal: Vec<f64> = lidar
        .ideal_ranges(&world, &body)
        .into_iter()
        .map(|z| z.unwrap_or(f64::INFINITY))
        .collect();
    let scan = lidar.scan(&world, &body, Timestamp::ZERO);
    let valid = scan.valid_beams().count() as u32;
    let lost = (0..scan.len())
        .filter(|&i| ideal[i].is_finite() && !scan.is_valid(i))
        .count() as u32;
    Ok(Ch03Scan {
        ranges: scan.ranges.iter().map(|r| *r as f64).collect(),
        ideal,
        valid,
        lost,
        hash: hash_ranges(&scan.ranges),
    })
}

/// The sensor pose `(x, y, θ)` in the world for a robot at `(x, y, theta)`:
/// `x ∘ T_base_laser`.
#[wasm_bindgen]
pub fn ch03_sensor_pose(x: f64, y: f64, theta: f64) -> Vec<f64> {
    let s = Se2::new(x, y, theta).compose(&mount());
    vec![s.x(), s.y(), s.theta()]
}

/// World-frame endpoints of the usable beams of a 360-beam scan taken by a robot at
/// `(x, y, theta)`: `[x₀, y₀, x₁, y₁, …]` with `NaN, NaN` for beams that are not valid.
/// Passing a pose other than the one the scan was taken from shows where the scan would
/// land if the robot were there.
#[wasm_bindgen]
pub fn ch03_scan_points(ranges: &[f64], x: f64, y: f64, theta: f64) -> Result<Vec<f64>, String> {
    let scan = scan_from_ranges(ranges)?;
    let sensor = Se2::new(x, y, theta).compose(&mount());
    let mut out = Vec::with_capacity(2 * scan.len());
    for i in 0..scan.len() {
        if scan.is_valid(i) {
            let (r, b) = (scan.ranges[i] as f64, scan.bearing(i));
            let p = sensor.act(&Vector2::new(r * b.cos(), r * b.sin()));
            out.extend_from_slice(&[p.x, p.y]);
        } else {
            out.extend_from_slice(&[f64::NAN, f64::NAN]);
        }
    }
    Ok(out)
}

/// Fingerprints of a whole `RustySim` run (`secs` seconds, `Trajectory::mixed`).
#[wasm_bindgen]
pub struct Ch03Replay {
    scans: u64,
    wheel: u64,
    imu: u64,
    truth: u64,
    n_scans: u32,
}

#[wasm_bindgen]
impl Ch03Replay {
    /// Fingerprint of every scan taken.
    #[wasm_bindgen(getter)]
    pub fn scans(&self) -> u64 {
        self.scans
    }
    /// Fingerprint of every encoder sample.
    #[wasm_bindgen(getter)]
    pub fn wheel(&self) -> u64 {
        self.wheel
    }
    /// Fingerprint of every IMU sample.
    #[wasm_bindgen(getter)]
    pub fn imu(&self) -> u64 {
        self.imu
    }
    /// Fingerprint of the ground-truth sequence.
    #[wasm_bindgen(getter)]
    pub fn truth(&self) -> u64 {
        self.truth
    }
    /// Number of scans taken.
    #[wasm_bindgen(getter)]
    pub fn n_scans(&self) -> u32 {
        self.n_scans
    }
}

/// Run `RustySim` (default noise, `Trajectory::mixed(false)`) for `secs` seconds with master
/// seed `seed`, with the LiDAR present (`lidar = true`, dropout `dropout`) or removed
/// (`lidar = false`), and fingerprint each output stream. The fingerprint hashes the `Debug`
/// text of every sample, which prints floats with their shortest round-trip form, so equal
/// fingerprints mean bit-identical streams.
#[wasm_bindgen]
pub fn ch03_sim_replay(seed: u64, secs: f64, lidar: bool, dropout: f64) -> Result<Ch03Replay, String> {
    let cfg = RustyConfig {
        seed,
        lidar: lidar.then(|| lidar_config(0.02, dropout, false)),
        ..RustyConfig::default()
    };
    let mut sim = RustySim::new(cfg, Trajectory::mixed(false)).map_err(err)?;
    let steps = (secs / sim.config().physics_dt).round() as usize;
    let (mut scans, mut wheel, mut imu, mut truth) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut n_scans = 0u32;
    for _ in 0..steps {
        let s = sim.step();
        truth.extend(format!("{:?}", s.truth).into_bytes());
        if let Some(w) = &s.wheel {
            wheel.extend(format!("{w:?}").into_bytes());
        }
        if let Some(m) = &s.imu {
            imu.extend(format!("{m:?}").into_bytes());
        }
        if let Some(sc) = &s.scan {
            n_scans += 1;
            scans.extend(sc.ranges.iter().flat_map(|r| r.to_bits().to_le_bytes()));
        }
    }
    Ok(Ch03Replay {
        scans: fnv1a(scans),
        wheel: fnv1a(wheel),
        imu: fnv1a(imu),
        truth: fnv1a(truth),
        n_scans,
    })
}

// ---------------------------------------------------------------------------------------
// The beam model
// ---------------------------------------------------------------------------------------

/// The default parameters the chapter talks about, so the page never repeats a constant that
/// lives in the crate: `[beam z_hit, z_short, z_max, z_rand, σ_hit, λ_short, max_range,
/// field z_hit, field z_rand, field σ_hit]` (`BeamModelParams::default()`,
/// `LikelihoodFieldParams::default()`).
#[wasm_bindgen]
pub fn ch03_model_defaults() -> Vec<f64> {
    let (b, f) = (BeamModelParams::default(), LikelihoodFieldParams::default());
    vec![
        b.z_hit,
        b.z_short,
        b.z_max,
        b.z_rand,
        b.sigma_hit,
        b.lambda_short,
        b.max_range,
        f.z_hit,
        f.z_rand,
        f.sigma_hit,
    ]
}

/// The beam model's density at one `z*` on a grid of readings.
#[wasm_bindgen]
pub struct Ch03BeamDensity {
    curve: Vec<f64>,
    atom: f64,
    mass: f64,
    weights: Vec<f64>,
}

#[wasm_bindgen]
impl Ch03BeamDensity {
    /// Rows `[z, hit, short, rand, total]` for `n` mid-points `z = (i + ½)·z_max/n`; each
    /// column is the component density times its mixture weight, `total` their sum
    /// (`BeamModel::likelihood`). The `max` component is a point mass, so it is `atom`.
    #[wasm_bindgen(getter)]
    pub fn curve(&self) -> Vec<f64> {
        self.curve.clone()
    }
    /// The point mass at `z_max` (the weight `z_max`).
    #[wasm_bindgen(getter)]
    pub fn atom(&self) -> f64 {
        self.atom
    }
    /// Riemann sum of `total` over `[0, z_max)` plus the atom: the mixture's total mass.
    #[wasm_bindgen(getter)]
    pub fn mass(&self) -> f64 {
        self.mass
    }
    /// The four weights after normalization, `[z_hit, z_short, z_max, z_rand]`.
    #[wasm_bindgen(getter)]
    pub fn weights(&self) -> Vec<f64> {
        self.weights.clone()
    }
}

/// `BeamModel` at expected range `z_star`. The four weights are normalized to sum to one
/// (the crate requires it); `n` is the number of grid points.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn ch03_beam_density(
    z_star: f64,
    w_hit: f64,
    w_short: f64,
    w_max: f64,
    w_rand: f64,
    sigma_hit: f64,
    lambda_short: f64,
    max_range: f64,
    n: usize,
) -> Result<Ch03BeamDensity, String> {
    let sum = w_hit + w_short + w_max + w_rand;
    if sum.is_nan() || sum <= 0.0 || n == 0 {
        return Err("weights must have a positive sum and n must be positive".into());
    }
    let w = [w_hit / sum, w_short / sum, w_max / sum, w_rand / sum];
    let model = BeamModel::new(BeamModelParams {
        z_hit: w[0],
        z_short: w[1],
        z_max: w[2],
        z_rand: w[3],
        sigma_hit,
        lambda_short,
        max_range,
    })
    .map_err(err)?;
    let h = max_range / n as f64;
    let mut curve = Vec::with_capacity(5 * n);
    let mut mass = 0.0;
    for i in 0..n {
        let z = (i as f64 + 0.5) * h;
        let c = model.components(z, z_star);
        let total = model.likelihood(z, z_star);
        mass += total * h;
        curve.extend_from_slice(&[z, w[0] * c.hit, w[1] * c.short, w[3] * c.rand, total]);
    }
    let atom = w[2] * model.components(max_range, z_star).max;
    Ok(Ch03BeamDensity {
        curve,
        atom,
        mass: mass + atom,
        weights: w.to_vec(),
    })
}

/// `Σ ln p(z_k | z*_k)` of a measured 360-beam scan under the book-default `BeamModel`, with
/// `z*_k` ray cast in the arena from the hypothesised robot pose `(x, y, theta)` (no wall
/// within range counts as `z* = z_max`; non-finite measured ranges count as max-range
/// readings, as the crate does).
#[wasm_bindgen]
pub fn ch03_beam_scan_loglik(ranges: &[f64], x: f64, y: f64, theta: f64) -> Result<f64, String> {
    let scan = scan_from_ranges(ranges)?;
    let model = BeamModel::new(BeamModelParams::default()).map_err(err)?;
    Ok(beam_loglik(&model, &World::rusty_arena(), &scan, &Se2::new(x, y, theta), 1))
}

fn beam_loglik(model: &BeamModel, world: &World, scan: &LaserScan, pose: &Se2, stride: usize) -> f64 {
    let sensor = pose.compose(&mount());
    let origin = Vector2::new(sensor.x(), sensor.y());
    let max = scan.range_max;
    model.log_likelihood((0..scan.len()).step_by(stride.max(1)).map(|i| {
        let z_star = world
            .raycast(&origin, sensor.theta() + scan.bearing(i), max)
            .unwrap_or(max);
        (scan.ranges[i] as f64, z_star)
    }))
}

/// Readings drawn from the beam mixture at random expected ranges, the way the crate's own
/// EM test does it (`beam::tests::em_recovers_generating_parameters_and_is_monotone`): the
/// same draw sequence, so on a 64-bit target seed 10 and `n = 20 000` reproduce that test's
/// data set. On WebAssembly the same seed is a different stream (`SmallRng` is a different
/// generator on 32-bit targets, see `ch02_seeded_rng_probe`), so there it is a different
/// data set from the same distribution. Not crate code: the crate has the density and the
/// fit but no sampler.
fn generate_beam_data(truth: &BeamModelParams, seed: u64, n: usize) -> (Vec<f64>, Vec<f64>) {
    use rand::RngExt;
    use rand_distr::{Distribution, Exp, Normal};
    let mut rng = seeded_rng(seed);
    let (mut z, mut zs) = (Vec::with_capacity(n), Vec::with_capacity(n));
    let hit = Normal::new(0.0, truth.sigma_hit).expect("positive sigma");
    let short = Exp::new(truth.lambda_short).expect("positive rate");
    while z.len() < n {
        let z_star = 1.0 + 6.0 * rng.random::<f64>();
        let u = rng.random::<f64>();
        let r = if u < truth.z_hit {
            (z_star + hit.sample(&mut rng)).clamp(0.0, truth.max_range)
        } else if u < truth.z_hit + truth.z_short {
            let s: f64 = short.sample(&mut rng);
            if s > z_star {
                continue; // truncated exponential: redraw
            }
            s
        } else if u < truth.z_hit + truth.z_short + truth.z_max {
            truth.max_range
        } else {
            truth.max_range * rng.random::<f64>()
        };
        z.push(r);
        zs.push(z_star);
    }
    (z, zs)
}

fn params_vec(p: &BeamModelParams) -> Vec<f64> {
    vec![p.z_hit, p.z_short, p.z_max, p.z_rand, p.sigma_hit, p.lambda_short]
}

/// The generating parameters of [`ch03_em_fit`]'s data (the crate test's).
fn em_truth() -> BeamModelParams {
    BeamModelParams {
        z_hit: 0.8,
        z_short: 0.1,
        z_max: 0.05,
        z_rand: 0.05,
        sigma_hit: 0.1,
        lambda_short: 1.0,
        max_range: 8.0,
    }
}

/// The starting parameters of [`ch03_em_fit`] (the crate test's).
fn em_start() -> BeamModelParams {
    BeamModelParams {
        z_hit: 0.25,
        z_short: 0.25,
        z_max: 0.25,
        z_rand: 0.25,
        sigma_hit: 0.5,
        lambda_short: 0.5,
        max_range: 8.0,
    }
}

/// EM on simulated readings.
#[wasm_bindgen]
pub struct Ch03Em {
    data: Vec<f64>,
    truth: Vec<f64>,
    start: Vec<f64>,
    history: Vec<f64>,
    trace: Vec<f64>,
    worst_dip: f64,
}

#[wasm_bindgen]
impl Ch03Em {
    /// `[z*₀, z₀, z*₁, z₁, …]`: expected range and reading of each of the `n` samples.
    #[wasm_bindgen(getter)]
    pub fn data(&self) -> Vec<f64> {
        self.data.clone()
    }
    /// The generating parameters `[z_hit, z_short, z_max, z_rand, σ_hit, λ_short]`.
    #[wasm_bindgen(getter)]
    pub fn truth(&self) -> Vec<f64> {
        self.truth.clone()
    }
    /// The starting parameters, same order.
    #[wasm_bindgen(getter)]
    pub fn start(&self) -> Vec<f64> {
        self.start.clone()
    }
    /// Row `k` (6 numbers, same order) = the parameters after `k` EM steps; `iters + 1` rows.
    #[wasm_bindgen(getter)]
    pub fn history(&self) -> Vec<f64> {
        self.history.clone()
    }
    /// `trace[k]` = mean log-likelihood per beam under the parameters of `history` row `k`
    /// (`EmStep::mean_log_likelihood`); `iters + 1` values.
    #[wasm_bindgen(getter)]
    pub fn trace(&self) -> Vec<f64> {
        self.trace.clone()
    }
    /// `max_k max(0, trace[k] − trace[k+1])`: the largest drop of the trace between two
    /// iterations (nats per beam); 0 for a monotone trace.
    #[wasm_bindgen(getter)]
    pub fn worst_dip(&self) -> f64 {
        self.worst_dip
    }
}

/// Simulate `n` readings from the generating mixture with `seed`, then run `iters` steps of
/// `BeamModel::em_step` (Thrun Table 6.2) from the fixed start. The truth and start
/// parameters are those of the crate's EM test.
#[wasm_bindgen]
pub fn ch03_em_fit(seed: u64, n: usize, iters: usize) -> Result<Ch03Em, String> {
    let truth = em_truth();
    let (z, zs) = generate_beam_data(&truth, seed, n);
    let mut model = BeamModel::new(em_start()).map_err(err)?;
    let mut history = params_vec(model.params());
    let mut trace = Vec::with_capacity(iters + 1);
    for _ in 0..iters {
        let step = model.em_step(&z, &zs).map_err(err)?;
        trace.push(step.mean_log_likelihood);
        model = BeamModel::new(step.params).map_err(err)?;
        history.extend(params_vec(model.params()));
    }
    trace.push(model.em_step(&z, &zs).map_err(err)?.mean_log_likelihood);
    let worst_dip = trace.windows(2).map(|w| (w[0] - w[1]).max(0.0)).fold(0.0, f64::max);
    Ok(Ch03Em {
        data: z.iter().zip(&zs).flat_map(|(a, b)| [*b, *a]).collect(),
        truth: params_vec(&truth),
        start: params_vec(&em_start()),
        history,
        trace,
        worst_dip,
    })
}

/// The crate's own `BeamModel::fit_em` on the same data as [`ch03_em_fit`], run for exactly
/// `iters` iterations (tolerance 0): returns the final parameters `[z_hit, z_short, z_max,
/// z_rand, σ_hit, λ_short]`. Lets a check confirm the per-iteration history above is the
/// crate's fit, step for step.
#[wasm_bindgen]
pub fn ch03_em_fit_final(seed: u64, n: usize, iters: usize) -> Result<Vec<f64>, String> {
    let (z, zs) = generate_beam_data(&em_truth(), seed, n);
    let start = BeamModel::new(em_start()).map_err(err)?;
    let (fit, _) = start.fit_em(&z, &zs, iters, 0.0).map_err(err)?;
    Ok(params_vec(fit.params()))
}

/// The book's worked EM iteration (Ch. 10): readings `Z = {4.5, 5, 3, 7, 10}` against a wall
/// at `z* = 5` with `z_max = 10`, uniform weights `¼`, `σ_hit = 1`, `λ = ½`. Returns
/// `[z_hit, z_short, z_max, z_rand, σ_hit, λ_short, mean_log_likelihood, used]` after one
/// `BeamModel::em_step`.
#[wasm_bindgen]
pub fn ch03_em_worked_example() -> Result<Vec<f64>, String> {
    let m = BeamModel::new(BeamModelParams {
        z_hit: 0.25,
        z_short: 0.25,
        z_max: 0.25,
        z_rand: 0.25,
        sigma_hit: 1.0,
        lambda_short: 0.5,
        max_range: 10.0,
    })
    .map_err(err)?;
    let s = m.em_step(&[4.5, 5.0, 3.0, 7.0, 10.0], &[5.0; 5]).map_err(err)?;
    let mut out = params_vec(&s.params);
    out.push(s.mean_log_likelihood);
    out.push(s.used as f64);
    Ok(out)
}

// ---------------------------------------------------------------------------------------
// The likelihood field
// ---------------------------------------------------------------------------------------

const FIELD_RES: f64 = 0.05;
const FIELD_ORIGIN: (f64, f64) = (-6.0, -4.5);
const FIELD_W: usize = 280; // x ∈ [−6, 8)
const FIELD_H: usize = 280; // y ∈ [−4.5, 9.5)

/// Mark every cell a segment passes through. Not crate code: the repository has no
/// rasterizer yet (`docs/SOURCE_MAPPING.md`, Ch. 10: "rasterizing a `World` into the EDT →
/// Phase 3"). It samples each segment every quarter cell, so the mask is a one-cell-thick
/// outline and the map is only as exact as half a cell diagonal.
fn rasterize(world: &World, res: f64, origin: (f64, f64), w: usize, h: usize) -> Vec<bool> {
    let mut occ = vec![false; w * h];
    for s in world.segments() {
        let len = (s.b - s.a).norm();
        let n = (len / (res / 4.0)).ceil().max(1.0) as usize;
        for k in 0..=n {
            let p = s.a + (s.b - s.a) * (k as f64 / n as f64);
            let i = ((p.x - origin.0) / res).floor();
            let j = ((p.y - origin.1) / res).floor();
            if i >= 0.0 && j >= 0.0 && (i as usize) < w && (j as usize) < h {
                occ[j as usize * w + i as usize] = true;
            }
        }
    }
    occ
}

fn arena_grid() -> &'static DistanceGrid {
    static GRID: OnceLock<DistanceGrid> = OnceLock::new();
    GRID.get_or_init(|| {
        let occ = rasterize(&World::rusty_arena(), FIELD_RES, FIELD_ORIGIN, FIELD_W, FIELD_H);
        DistanceGrid::from_occupancy(
            &occ,
            FIELD_W,
            FIELD_H,
            FIELD_RES,
            Vector2::new(FIELD_ORIGIN.0, FIELD_ORIGIN.1),
        )
        .expect("static grid")
    })
}

/// The arena rasterized and run through the exact Euclidean distance transform.
#[wasm_bindgen]
pub struct Ch03Field {
    distance: Vec<f64>,
    value: Vec<f64>,
}

#[wasm_bindgen]
impl Ch03Field {
    /// Grid width in cells.
    #[wasm_bindgen(getter)]
    pub fn width(&self) -> u32 {
        FIELD_W as u32
    }
    /// Grid height in cells.
    #[wasm_bindgen(getter)]
    pub fn height(&self) -> u32 {
        FIELD_H as u32
    }
    /// Cell size (m).
    #[wasm_bindgen(getter)]
    pub fn resolution(&self) -> f64 {
        FIELD_RES
    }
    /// World x of the grid's lower-left corner (m).
    #[wasm_bindgen(getter)]
    pub fn origin_x(&self) -> f64 {
        FIELD_ORIGIN.0
    }
    /// World y of the grid's lower-left corner (m).
    #[wasm_bindgen(getter)]
    pub fn origin_y(&self) -> f64 {
        FIELD_ORIGIN.1
    }
    /// Distance (m) from each cell centre to the nearest wall cell, row-major with `y` as
    /// the slow index (`DistanceGrid::cell`).
    #[wasm_bindgen(getter)]
    pub fn distance(&self) -> Vec<f64> {
        self.distance.clone()
    }
    /// The per-beam term `z_hit·N(d; 0, σ_hit²) + z_rand/z_max` of the likelihood-field model
    /// (default parameters) evaluated at each cell's distance, same layout. It is a score
    /// density per endpoint, not a probability of anything.
    #[wasm_bindgen(getter)]
    pub fn value(&self) -> Vec<f64> {
        self.value.clone()
    }
}

/// The likelihood field of the arena.
#[wasm_bindgen]
pub fn ch03_likelihood_field() -> Ch03Field {
    let grid = arena_grid();
    let p = LikelihoodFieldParams::default();
    let mut distance = Vec::with_capacity(FIELD_W * FIELD_H);
    for j in 0..FIELD_H {
        for i in 0..FIELD_W {
            distance.push(grid.cell(i, j).expect("in range"));
        }
    }
    let value = distance
        .iter()
        .map(|d| p.z_hit * normal_pdf(*d, p.sigma_hit) + p.z_rand / p.max_range)
        .collect();
    Ch03Field { distance, value }
}

/// Exact Euclidean distance transform of an occupancy mask (`1` = obstacle) of `width ×
/// height` cells of size `resolution`, row-major with `y` slow: `DistanceGrid::from_occupancy`
/// read back with `cell`. `+inf` where the mask has no obstacle at all.
#[wasm_bindgen]
pub fn ch03_edt(mask: &[u8], width: usize, height: usize, resolution: f64) -> Result<Vec<f64>, String> {
    let occ: Vec<bool> = mask.iter().map(|m| *m != 0).collect();
    let g = DistanceGrid::from_occupancy(&occ, width, height, resolution, Vector2::zeros()).map_err(err)?;
    let mut out = Vec::with_capacity(width * height);
    for j in 0..height {
        for i in 0..width {
            out.push(g.cell(i, j).expect("in range"));
        }
    }
    Ok(out)
}

/// Scores of one measured scan along a line of hypothesised poses.
#[wasm_bindgen]
pub struct Ch03Slice {
    offsets: Vec<f64>,
    field: Vec<f64>,
    beam: Vec<f64>,
    k_field: u32,
    k_beam: u32,
}

#[wasm_bindgen]
impl Ch03Slice {
    /// The `n` offsets from the reference pose (m for `axis` 0 and 1, rad for 2).
    #[wasm_bindgen(getter)]
    pub fn offsets(&self) -> Vec<f64> {
        self.offsets.clone()
    }
    /// `LikelihoodFieldModel::log_likelihood` at each offset (nats).
    #[wasm_bindgen(getter)]
    pub fn field(&self) -> Vec<f64> {
        self.field.clone()
    }
    /// `BeamModel::log_likelihood` (book-default parameters, `z*` ray cast from each
    /// hypothesised pose) at each offset (nats).
    #[wasm_bindgen(getter)]
    pub fn beam(&self) -> Vec<f64> {
        self.beam.clone()
    }
    /// Beams the field model scored (valid, below max range, every `stride`-th).
    #[wasm_bindgen(getter)]
    pub fn k_field(&self) -> u32 {
        self.k_field
    }
    /// Beams the beam model scored (every `stride`-th of the 360).
    #[wasm_bindgen(getter)]
    pub fn k_beam(&self) -> u32 {
        self.k_beam
    }
    /// `[argmax offset (field), argmax offset (beam), width (field), width (beam)]`: where
    /// each score peaks, and the extent of the offsets whose score is within 10 nats of the
    /// peak (the offset spacing is the resolution of both).
    #[wasm_bindgen(getter)]
    pub fn summary(&self) -> Vec<f64> {
        let peak = |v: &[f64]| {
            let i = v.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).map_or(0, |(i, _)| i);
            self.offsets[i]
        };
        let width = |v: &[f64]| {
            let max = v.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let near: Vec<f64> = self.offsets.iter().zip(v).filter(|(_, s)| max - **s < 10.0).map(|(o, _)| *o).collect();
            near.iter().copied().fold(f64::NEG_INFINITY, f64::max) - near.iter().copied().fold(f64::INFINITY, f64::min)
        };
        vec![peak(&self.field), peak(&self.beam), width(&self.field), width(&self.beam)]
    }
}

/// Score a measured 360-beam `ranges` at `n` poses spread evenly over `±half_width` around
/// the reference pose `(x, y, theta)` along `axis` (0 = world x, 1 = world y, 2 = heading),
/// using every `stride`-th beam, under both the likelihood-field model (over the rasterized
/// arena, [`ch03_likelihood_field`]) and the beam model.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn ch03_slice_scores(
    ranges: &[f64],
    x: f64,
    y: f64,
    theta: f64,
    axis: u32,
    half_width: f64,
    n: usize,
    stride: usize,
) -> Result<Ch03Slice, String> {
    if n < 2 || axis > 2 || half_width.is_nan() || half_width <= 0.0 {
        return Err("need n ≥ 2, axis in 0..=2 and half_width > 0".into());
    }
    let stride = stride.max(1);
    let scan = scan_from_ranges(ranges)?;
    let world = World::rusty_arena();
    let grid = arena_grid();
    let field = LikelihoodFieldModel::new(LikelihoodFieldParams::default()).map_err(err)?;
    let beam = BeamModel::new(BeamModelParams::default()).map_err(err)?;
    let (mut offsets, mut f, mut b) = (Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n));
    for i in 0..n {
        let o = -half_width + 2.0 * half_width * i as f64 / (n - 1) as f64;
        let pose = match axis {
            0 => Se2::new(x + o, y, theta),
            1 => Se2::new(x, y + o, theta),
            _ => Se2::new(x, y, theta + o),
        };
        offsets.push(o);
        f.push(field.log_likelihood(&scan, &pose, &mount(), grid, stride));
        b.push(beam_loglik(&beam, &world, &scan, &pose, stride));
    }
    let max = LikelihoodFieldParams::default().max_range.min(scan.range_max);
    let k_field = scan
        .valid_beams_strided(stride)
        .filter(|(_, r)| *r < max - 1e-9)
        .count() as u32;
    let k_beam = (0..scan.len()).step_by(stride).count() as u32;
    Ok(Ch03Slice {
        offsets,
        field: f,
        beam: b,
        k_field,
        k_beam,
    })
}

/// `LikelihoodFieldModel` score of a measured scan at one hypothesised pose (every beam).
#[wasm_bindgen]
pub fn ch03_field_scan_loglik(ranges: &[f64], x: f64, y: f64, theta: f64) -> Result<f64, String> {
    let scan = scan_from_ranges(ranges)?;
    let field = LikelihoodFieldModel::new(LikelihoodFieldParams::default()).map_err(err)?;
    Ok(field.log_likelihood(&scan, &Se2::new(x, y, theta), &mount(), arena_grid(), 1))
}

/// Distance from a world point to the nearest rasterized wall (`DistanceField::distance`).
#[wasm_bindgen]
pub fn ch03_field_distance(x: f64, y: f64) -> f64 {
    arena_grid().distance(&Vector2::new(x, y))
}

// ---------------------------------------------------------------------------------------
// Tempering
// ---------------------------------------------------------------------------------------

/// The normalized posterior over a set of hypotheses after a tempered update.
#[wasm_bindgen]
pub struct Ch03Posterior {
    posterior: Vec<f64>,
    ess_ratio: f64,
    mean: f64,
    std: f64,
    beta_target: f64,
}

#[wasm_bindgen]
impl Ch03Posterior {
    /// `w_i ∝ exp(β ℓ_i)`, normalized, one per hypothesis.
    #[wasm_bindgen(getter)]
    pub fn posterior(&self) -> Vec<f64> {
        self.posterior.clone()
    }
    /// Effective sample size over the number of hypotheses, `(Σw)²/(N Σw²)`.
    #[wasm_bindgen(getter)]
    pub fn ess_ratio(&self) -> f64 {
        self.ess_ratio
    }
    /// Posterior mean of the offsets.
    #[wasm_bindgen(getter)]
    pub fn mean(&self) -> f64 {
        self.mean
    }
    /// Posterior standard deviation of the offsets.
    #[wasm_bindgen(getter)]
    pub fn std(&self) -> f64 {
        self.std
    }
    /// `Tempering::for_target_ess(…).beta()` for the requested target ratio.
    #[wasm_bindgen(getter)]
    pub fn beta_target(&self) -> f64 {
        self.beta_target
    }
}

/// Reweight a uniform prior over hypotheses at `offsets` by `p(z|x)^β`, where `log_lik[i]`
/// is `ln p(z | x_i)` (for instance [`Ch03Slice::field`]); report the posterior, its ESS
/// ratio, mean and standard deviation, and the largest `β` that would keep the ESS ratio at
/// or above `target_ratio` (`Tempering::for_target_ess`, `β_min = 1e-6`).
#[wasm_bindgen]
pub fn ch03_tempered_posterior(
    offsets: &[f64],
    log_lik: &[f64],
    beta: f64,
    target_ratio: f64,
) -> Result<Ch03Posterior, String> {
    let (posterior, ess_ratio, mean, std) = temper(offsets, log_lik, beta)?;
    let n = offsets.len() as f64;
    let log_prior = vec![-n.ln(); offsets.len()];
    let beta_target = Tempering::for_target_ess(&log_prior, log_lik, target_ratio, 1e-6)
        .map_err(err)?
        .beta();
    Ok(Ch03Posterior {
        posterior,
        ess_ratio,
        mean,
        std,
        beta_target,
    })
}

/// `(posterior, ESS ratio, mean, std)` of `w_i ∝ exp(β ℓ_i)` over `offsets`.
fn temper(offsets: &[f64], log_lik: &[f64], beta: f64) -> Result<(Vec<f64>, f64, f64, f64), String> {
    if offsets.len() != log_lik.len() || offsets.is_empty() {
        return Err("offsets and log_lik must match and be non-empty".into());
    }
    let n = offsets.len() as f64;
    let t = Tempering::new(beta).map_err(err)?;
    let lw: Vec<f64> = log_lik.iter().map(|l| t.apply_log(*l)).collect();
    let max = lw.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let s: f64 = lw.iter().map(|l| (l - max).exp()).sum();
    let w: Vec<f64> = lw.iter().map(|l| (l - max).exp() / s).collect();
    let mean: f64 = w.iter().zip(offsets).map(|(w, o)| w * o).sum();
    let var: f64 = w.iter().zip(offsets).map(|(w, o)| w * (o - mean).powi(2)).sum();
    let ess = effective_sample_size(&w) / n;
    Ok((w, ess, mean, var.sqrt()))
}

/// Many scans, scored once, answering "what does tempering do?" for any `β`.
///
/// For each of eight poses and `n_seeds` noise seeds, one scan is simulated at the pose, scored
/// along the world x axis over `±0.5 m` (101 offsets) under the likelihood-field model
/// (`model = 0`) or the beam model (`model = 1`), and kept. The true pose is offset 0, so the
/// mean of a tempered posterior is its error.
#[wasm_bindgen]
pub struct Ch03Study {
    offsets: Vec<f64>,
    scores: Vec<Vec<f64>>,
}

#[wasm_bindgen]
impl Ch03Study {
    /// The 101 offsets (m).
    #[wasm_bindgen(getter)]
    pub fn offsets(&self) -> Vec<f64> {
        self.offsets.clone()
    }
    /// Number of scans.
    #[wasm_bindgen(getter)]
    pub fn scans(&self) -> u32 {
        self.scores.len() as u32
    }
    /// `[rms error (m), median σ (m), rms of error/σ, median ESS ratio]` over the scans for
    /// tempering exponent `beta`. "error" is the posterior mean (the truth is offset 0) and σ
    /// the posterior standard deviation; a posterior is calibrated when the rms ratio is 1, over-confident when above.
    pub fn at(&self, beta: f64) -> Result<Vec<f64>, String> {
        let (mut err2, mut z2, mut stds, mut esss) = (0.0, 0.0, Vec::new(), Vec::new());
        for l in &self.scores {
            let (_, ess, mean, std) = temper(&self.offsets, l, beta)?;
            err2 += mean * mean;
            z2 += (mean / std.max(1e-12)).powi(2);
            stds.push(std);
            esss.push(ess);
        }
        let n = self.scores.len() as f64;
        let median = |v: &mut Vec<f64>| {
            v.sort_by(f64::total_cmp);
            v[v.len() / 2]
        };
        Ok(vec![(err2 / n).sqrt(), median(&mut stds), (z2 / n).sqrt(), median(&mut esss)])
    }
    /// The tempered posterior over the offsets for the first scan.
    pub fn example(&self, beta: f64) -> Result<Vec<f64>, String> {
        Ok(temper(&self.offsets, &self.scores[0], beta)?.0)
    }
}

/// The poses [`ch03_study`] visits (the chapter's widgets use the same list).
const STUDY_POSES: [(f64, f64, f64); 8] = [
    (1.0, 2.0, 0.4),
    (-2.0, 0.0, 1.2),
    (3.0, 5.0, -2.0),
    (4.0, -1.0, 0.7),
    (-3.0, 6.0, 3.0),
    (2.0, 3.0, -0.5),
    (5.0, 4.0, 1.9),
    (0.0, 0.0, 0.0),
];

/// Simulate and score the scans of a [`Ch03Study`]: `n_seeds` per pose, seeds
/// `seed, seed + 1, …`, range noise 0.02 m, dropout 1 %, every `stride`-th beam.
#[wasm_bindgen]
pub fn ch03_study(model: u32, seed: u64, n_seeds: usize, stride: usize) -> Result<Ch03Study, String> {
    if model > 1 || n_seeds == 0 {
        return Err("model is 0 (likelihood field) or 1 (beam), and n_seeds must be positive".into());
    }
    let n = 101;
    let (field, beam) = (
        LikelihoodFieldModel::new(LikelihoodFieldParams::default()).map_err(err)?,
        BeamModel::new(BeamModelParams::default()).map_err(err)?,
    );
    let (world, grid) = (World::rusty_arena(), arena_grid());
    let offsets: Vec<f64> = (0..n).map(|i| -0.5 + i as f64 / (n - 1) as f64).collect();
    let mut scores = Vec::new();
    for (x, y, theta) in STUDY_POSES {
        for k in 0..n_seeds {
            let mut lidar = Lidar::new(lidar_config(0.02, 0.01, false), seed.wrapping_add(k as u64)).map_err(err)?;
            let scan = lidar.scan(&world, &Se2::new(x, y, theta), Timestamp::ZERO);
            scores.push(
                offsets
                    .iter()
                    .map(|o| {
                        let pose = Se2::new(x + o, y, theta);
                        if model == 0 {
                            field.log_likelihood(&scan, &pose, &mount(), grid, stride.max(1))
                        } else {
                            beam_loglik(&beam, &world, &scan, &pose, stride.max(1))
                        }
                    })
                    .collect(),
            );
        }
    }
    Ok(Ch03Study { offsets, scores })
}

// ---------------------------------------------------------------------------------------
// The landmark model
// ---------------------------------------------------------------------------------------

/// Range and bearing of landmark `(lx, ly)` seen from the robot pose `(x, y, theta)` with the
/// sensor at the robot origin: `[r̂, φ̂]`.
#[wasm_bindgen]
pub fn ch03_landmark_predict(x: f64, y: f64, theta: f64, lx: f64, ly: f64) -> Result<Vec<f64>, String> {
    let m = LandmarkModel::new(0.1, 0.05).map_err(err)?;
    let p = m
        .predict(&Se2::new(x, y, theta), &Vector2::new(lx, ly))
        .map_err(err)?;
    Ok(vec![p.range, p.bearing])
}

/// `LandmarkModel::jacobian`: the 2 × 3 matrix `∂ĥ/∂δ` for a right-perturbation
/// `δ = (v_x, v_y, ω)` of the pose, row-major `[∂r/∂v_x, ∂r/∂v_y, ∂r/∂ω, ∂φ/∂v_x, …]`.
#[wasm_bindgen]
pub fn ch03_landmark_jacobian(x: f64, y: f64, theta: f64, lx: f64, ly: f64) -> Result<Vec<f64>, String> {
    let m = LandmarkModel::new(0.1, 0.05).map_err(err)?;
    let h = m
        .jacobian(&Se2::new(x, y, theta), &Vector2::new(lx, ly))
        .map_err(err)?;
    Ok((0..2).flat_map(|r| (0..3).map(move |c| (r, c))).map(|(r, c)| h[(r, c)]).collect())
}

/// `ln p(z | x, m_j)` of a range-bearing reading `(z_range, z_bearing)`.
///
/// With `naive = false` this is `LandmarkModel::log_likelihood`, which wraps the bearing
/// residual to `(−π, π]`. With `naive = true` it is the **strawman** the chapter measures it
/// against, the same Gaussian with the bearing residual left as `φ − φ̂`; it is not code the
/// robot runs.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn ch03_landmark_loglik(
    x: f64,
    y: f64,
    theta: f64,
    lx: f64,
    ly: f64,
    z_range: f64,
    z_bearing: f64,
    sigma_range: f64,
    sigma_bearing: f64,
    naive: bool,
) -> Result<f64, String> {
    let m = LandmarkModel::new(sigma_range, sigma_bearing).map_err(err)?;
    let pose = Se2::new(x, y, theta);
    let l = Vector2::new(lx, ly);
    let z = RangeBearing {
        range: z_range,
        bearing: z_bearing,
    };
    if !naive {
        return m.log_likelihood(&pose, &l, &z).map_err(err);
    }
    let pred = m.predict(&pose, &l).map_err(err)?;
    let (er, eb) = (z.range - pred.range, z.bearing - pred.bearing);
    Ok(-0.5 * ((er / sigma_range).powi(2) + (eb / sigma_bearing).powi(2))
        - (sigma_range * sigma_bearing).ln()
        - TAU.ln())
}

/// `LandmarkModel::residual(z, ẑ)` for a reading `(z_range, z_bearing)` and a prediction
/// `(p_range, p_bearing)`: `[r − r̂, wrap(φ − φ̂)]`.
#[wasm_bindgen]
pub fn ch03_landmark_residual(z_range: f64, z_bearing: f64, p_range: f64, p_bearing: f64) -> Vec<f64> {
    let e = LandmarkModel::residual(
        &RangeBearing {
            range: z_range,
            bearing: z_bearing,
        },
        &RangeBearing {
            range: p_range,
            bearing: p_bearing,
        },
    );
    vec![e.x, e.y]
}

#[cfg(test)]
mod tests {
    use super::*;
    use robotics_core::Manifold;
    use std::f64::consts::PI;

    const SEED: u64 = 0x5eed_2026;

    #[test]
    fn scan_replays_and_depends_on_the_seed() {
        let a = ch03_lidar_scan(0.0, 0.0, 0.0, 0.02, 0.01, SEED, false).unwrap();
        let b = ch03_lidar_scan(0.0, 0.0, 0.0, 0.02, 0.01, SEED, false).unwrap();
        let c = ch03_lidar_scan(0.0, 0.0, 0.0, 0.02, 0.01, SEED + 1, false).unwrap();
        assert_eq!(a.hash, b.hash);
        assert_ne!(a.hash, c.hash);
        assert_eq!(a.ranges, b.ranges);
    }

    /// A noisy scan's fingerprint depends on the random stream, so it is pinned per pointer
    /// width: this is the 64-bit value (`checks/ch03.ts` pins the WebAssembly one).
    #[cfg(target_pointer_width = "64")]
    #[test]
    fn noisy_scan_fingerprint_on_64_bit() {
        let s = ch03_lidar_scan(1.0, 2.0, 0.4, 0.02, 0.01, 7, false).unwrap();
        assert_eq!(format!("{:016x}", s.hash), "8a526fd3ad0062b1");
    }

    #[test]
    fn ideal_scan_from_the_origin_matches_the_regression_test() {
        let s = ch03_lidar_scan(0.0, 0.0, 0.0, 0.0, 0.0, 0, false).unwrap();
        assert!((s.ranges[180] - 7.45).abs() < 1e-5);
        assert!((s.ranges[0] - 5.55).abs() < 1e-5);
        assert!((s.ranges[90] - 4.0).abs() < 1e-5);
        assert!(s.ranges[270].is_infinite());
        assert!(s.ideal[270].is_infinite());
    }

    #[test]
    fn model_defaults_are_the_documented_ones() {
        assert_eq!(ch03_model_defaults(), vec![0.74, 0.1, 0.06, 0.1, 0.12, 1.0, 8.0, 0.9, 0.1, 0.2]);
    }

    #[test]
    fn beam_density_has_unit_mass() {
        let d = ch03_beam_density(3.0, 0.74, 0.1, 0.06, 0.1, 0.12, 1.0, 8.0, 4000).unwrap();
        assert!((d.mass - 1.0).abs() < 3e-3, "{}", d.mass);
        assert!((d.atom - 0.06).abs() < 1e-12);
    }

    #[test]
    fn em_worked_example() {
        let v = ch03_em_worked_example().unwrap();
        for (got, want) in v.iter().zip([0.394282, 0.127210, 0.2, 0.278508, 1.093905, 0.283737, -2.275038]) {
            assert!((got - want).abs() < 5e-6, "{got} vs {want}");
        }
        assert_eq!(v[7], 5.0);
    }

    #[test]
    fn em_history_is_the_crates_fit_and_recovers_the_truth() {
        let e = ch03_em_fit(10, 20_000, 80).unwrap();
        let fit = ch03_em_fit_final(10, 20_000, 80).unwrap();
        for (a, b) in e.history[6 * 80..].iter().zip(&fit) {
            assert!((a - b).abs() < 1e-12);
        }
        // The crate's own assertions for this seed and size.
        assert!(e.trace.windows(2).all(|w| w[1] >= w[0] - 1e-3));
        assert!(e.trace.last().unwrap() > &(e.trace[0] + 0.5));
        assert!((fit[0] - 0.8).abs() < 0.03 && (fit[4] - 0.1).abs() < 0.01);
    }

    #[test]
    fn field_scores_peak_at_the_true_pose_along_x() {
        let s = ch03_lidar_scan(1.0, 2.0, 0.4, 0.02, 0.01, SEED, false).unwrap();
        let sl = ch03_slice_scores(&s.ranges, 1.0, 2.0, 0.4, 0, 0.5, 101, 1).unwrap();
        let mid = sl.field.len() / 2;
        let argmax = |v: &[f64]| v.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
        assert!((argmax(&sl.field) as i64 - mid as i64).abs() <= 3);
        assert!((argmax(&sl.beam) as i64 - mid as i64).abs() <= 1);
    }

    #[test]
    fn tempering_keeps_the_peak_and_widens() {
        let s = ch03_lidar_scan(1.0, 2.0, 0.4, 0.02, 0.01, SEED, false).unwrap();
        let sl = ch03_slice_scores(&s.ranges, 1.0, 2.0, 0.4, 0, 0.5, 101, 1).unwrap();
        let hi = ch03_tempered_posterior(&sl.offsets, &sl.field, 1.0, 0.5).unwrap();
        let lo = ch03_tempered_posterior(&sl.offsets, &sl.field, 0.01, 0.5).unwrap();
        assert!(lo.std > hi.std);
        let at = ch03_tempered_posterior(&sl.offsets, &sl.field, hi.beta_target, 0.5).unwrap();
        assert!((at.ess_ratio - 0.5).abs() < 2e-3, "{}", at.ess_ratio);
    }

    #[test]
    fn the_study_matches_the_slice_functions_and_tempering_calibrates_the_field() {
        let st = ch03_study(0, 1, 2, 1).unwrap();
        assert_eq!(st.scans(), 16);
        // Scan 0 is the pose (1, 2, 0.4) with seed 1: its scores are the slice's.
        let sc = ch03_lidar_scan(1.0, 2.0, 0.4, 0.02, 0.01, 1, false).unwrap();
        let sl = ch03_slice_scores(&sc.ranges, 1.0, 2.0, 0.4, 0, 0.5, 101, 1).unwrap();
        let p = ch03_tempered_posterior(&sl.offsets, &sl.field, 0.3, 0.5).unwrap();
        for (a, b) in st.example(0.3).unwrap().iter().zip(&p.posterior) {
            assert!((a - b).abs() < 1e-12);
        }
        // The field is over-confident untempered and the rms ratio falls as β does.
        let (hi, lo) = (st.at(1.0).unwrap(), st.at(0.1).unwrap());
        assert!(hi[2] > 1.0 && lo[2] < hi[2], "{hi:?} {lo:?}");
        let s = ch03_slice_scores(&sc.ranges, 1.0, 2.0, 0.4, 0, 0.5, 101, 1).unwrap();
        let sum = s.summary();
        assert_eq!(sum.len(), 4);
        assert!(sum[1].abs() < 0.011 && sum[3] < sum[2], "{sum:?}");
    }

    #[test]
    fn landmark_residual_wraps_and_the_jacobian_is_the_derivative() {
        // 179° predicted, −179° read: 2° apart, not 358°.
        let e = ch03_landmark_residual(1.0, -3.12, 1.0, 3.12);
        assert!((e[1] - (2.0 * PI - 6.24)).abs() < 1e-12, "{e:?}");
        // The wrapped log-likelihood is continuous across ±π; the strawman is not.
        let (x, y, th, lx, ly) = (0.3, -0.2, 0.9, -2.0, 1.5);
        let pred = ch03_landmark_predict(x, y, th, lx, ly).unwrap();
        let near = |bearing: f64, naive: bool| {
            ch03_landmark_loglik(x, y, th, lx, ly, pred[0], bearing, 0.1, 0.05, naive).unwrap()
        };
        let k = pred[1] + 1e-6;
        assert!((near(k, false) - near(pred[1], false)).abs() < 1e-3);
        let flipped = pred[1] - 2.0 * PI + 1e-6;
        assert!((near(flipped, false) - near(pred[1], false)).abs() < 1e-3);
        assert!(near(flipped, true) < near(pred[1], false) - 100.0);
        // Jacobian against a central difference of `exp`-perturbed poses.
        let j = ch03_landmark_jacobian(x, y, th, lx, ly).unwrap();
        let h = 1e-6;
        for c in 0..3 {
            let mut d = [0.0; 3];
            d[c] = h;
            let at = |sign: f64| {
                let p = Se2::new(x, y, th).boxplus(&robotics_core::Tangent2::new(sign * d[0], sign * d[1], sign * d[2]));
                ch03_landmark_predict(p.x(), p.y(), p.theta(), lx, ly).unwrap()
            };
            let (a, b) = (at(1.0), at(-1.0));
            let dr = (a[0] - b[0]) / (2.0 * h);
            let db = ch03_landmark_residual(a[0], a[1], b[0], b[1])[1] / (2.0 * h);
            assert!((dr - j[c]).abs() < 1e-6 && (db - j[3 + c]).abs() < 1e-6, "column {c}: {dr} {db} vs {} {}", j[c], j[3 + c]);
        }
    }

    #[test]
    fn lidar_does_not_change_the_wheel_or_imu_streams() {
        let with = ch03_sim_replay(7, 2.0, true, 0.01).unwrap();
        let without = ch03_sim_replay(7, 2.0, false, 0.01).unwrap();
        let again = ch03_sim_replay(7, 2.0, true, 0.01).unwrap();
        let other = ch03_sim_replay(8, 2.0, true, 0.01).unwrap();
        assert_eq!((with.wheel, with.imu, with.truth), (without.wheel, without.imu, without.truth));
        assert_eq!(with.scans, again.scans);
        assert_ne!(with.scans, other.scans);
        assert_eq!(with.n_scans, 20);
        assert_eq!(without.n_scans, 0);
    }

    #[test]
    fn edt_matches_a_brute_force_oracle() {
        let (w, h) = (23, 17);
        let mask: Vec<u8> = (0..w * h).map(|k| u8::from((k * 7919 + k / 5) % 23 == 0)).collect();
        let d = ch03_edt(&mask, w, h, 0.1).unwrap();
        for j in 0..h {
            for i in 0..w {
                let mut best = f64::INFINITY;
                for jj in 0..h {
                    for ii in 0..w {
                        if mask[jj * w + ii] != 0 {
                            best = best.min((((i as f64 - ii as f64).powi(2) + (j as f64 - jj as f64).powi(2)).sqrt()) * 0.1);
                        }
                    }
                }
                assert!((d[j * w + i] - best).abs() < 1e-9);
            }
        }
    }
}
