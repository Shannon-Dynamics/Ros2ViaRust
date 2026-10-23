//! Chapter 02 bindings (owned by the chapter; see web/design/Chapter-02.md).
//!
//! Everything here is a thin layer over the Phase 2 crates. The one substantial function,
//! [`run_traced`], is the pipeline of `crates/robotics_sim/tests/common/pipeline.rs`
//! (`run_pipeline`, the code behind `docs/STATE_ESTIMATION.md` §4.1) written so that it
//! also *records* what it computes: per wheel sample the truth, raw and fused poses, the
//! covariances, the NEES/NIS series and the gyro-bias estimate. It calls the same crate
//! functions in the same order and accumulates the metrics with the same expressions; the
//! unit tests below include the original file with `#[path]` and assert that every metric
//! is bit-identical, for the documented seeds, both `bias_update_max_speed` variants and
//! the noise-free configuration. Nothing is copied from it and then changed.
//!
//! Two deliberate differences from `run_pipeline`, both shown not to matter by those tests:
//! the LiDAR is switched off (the pipeline never reads a scan, and the simulator draws the
//! LiDAR from its own RNG stream, so the wheel and gyro data of a seed are unchanged), and
//! the optional `wheel_delay` queue, which at `0` delivers each wheel sample in the step
//! that produced it, exactly as the original does.
//!
//! The other wrappers expose the "rest of the toolbox" worked examples (information filter,
//! RTS smoother, χ² bounds) so the chapter's checks and widgets call the crates.

use nalgebra::{Matrix1, Matrix2, Vector1};
use robotics_core::angle::angle_diff;
use robotics_core::prob::chi2_mean_interval;
use robotics_core::{Gaussian, Manifold, Se2};
use robotics_filters::consistency::{nees, ConsistencyStats};
use robotics_filters::{BayesFilter, FusionConfig, InformationFilter, KalmanFilter, OdomImuFusion, RtsSmoother};
use robotics_models::{OdometryNoise, WheelOdometry};
use robotics_sim::{RustyConfig, RustySim, Trajectory};
use std::collections::VecDeque;
use wasm_bindgen::prelude::*;

/// Names of the columns of one trace row, in order (see [`Ch02Trace::rows`]).
pub const COLUMNS: [&str; 34] = [
    "t",
    "truth_x",
    "truth_y",
    "truth_th",
    "raw_x",
    "raw_y",
    "raw_th",
    "fus_x",
    "fus_y",
    "fus_th",
    "raw_pos_err",
    "raw_yaw_err",
    "fus_pos_err",
    "fus_yaw_err",
    "gyro_yaw_err",
    "raw_sigma_th",
    "fus_sigma_th",
    "nees_raw",
    "nees_fused",
    "nees_raw_mean",
    "nees_fused_mean",
    "nis",
    "nis_dof",
    "nis2_mean",
    "nis1_mean",
    "bias_true",
    "bias_est",
    "bias_sigma",
    "fus_cov_xx",
    "fus_cov_xy",
    "fus_cov_yy",
    "raw_cov_xx",
    "raw_cov_xy",
    "raw_cov_yy",
];

/// Names of the entries of the summary vector, in order (see [`Metrics::to_vec`]).
pub const SUMMARY: [&str; 25] = [
    "samples",
    "raw_pos_rmse",
    "raw_yaw_rmse",
    "raw_final_pos",
    "raw_final_yaw",
    "fused_pos_rmse",
    "fused_yaw_rmse",
    "fused_final_pos",
    "fused_final_yaw",
    "true_bias",
    "est_bias",
    "est_bias_sigma",
    "yaw_within_3sigma",
    "rejected",
    "raw_nees",
    "fused_nees",
    "fused_nis2",
    "nis2_samples",
    "fused_nis1",
    "nis1_samples",
    "raw_yaw_within_3sigma",
    "gyro_only_yaw_rmse",
    "wheel_accepted",
    "applied_late",
    "rejected_out_of_order",
];

/// The error statistics of `run_pipeline` (same names, same meaning), plus a few more that
/// only the trace can compute: the raw odometry's own ±3σ yaw coverage, the RMS yaw error of
/// the *strawman* that integrates the raw gyro with no bias estimate, and the fusion
/// front-end's counters.
#[derive(Debug, Clone, Copy, Default)]
pub struct Metrics {
    pub samples: usize,
    pub raw_pos_rmse: f64,
    pub raw_yaw_rmse: f64,
    pub raw_final_pos: f64,
    pub raw_final_yaw: f64,
    pub fused_pos_rmse: f64,
    pub fused_yaw_rmse: f64,
    pub fused_final_pos: f64,
    pub fused_final_yaw: f64,
    pub true_bias: f64,
    pub est_bias: f64,
    pub est_bias_sigma: f64,
    pub yaw_within_3sigma: f64,
    pub rejected: u64,
    pub raw_nees: f64,
    pub fused_nees: f64,
    pub fused_nis2: f64,
    pub nis2_samples: usize,
    pub fused_nis1: f64,
    pub nis1_samples: usize,
    pub raw_yaw_within_3sigma: f64,
    pub gyro_only_yaw_rmse: f64,
    pub wheel_accepted: u64,
    pub applied_late: u64,
    pub rejected_out_of_order: u64,
}

impl Metrics {
    /// The summary vector, in the order of [`SUMMARY`].
    pub fn to_vec(&self) -> Vec<f64> {
        vec![
            self.samples as f64,
            self.raw_pos_rmse,
            self.raw_yaw_rmse,
            self.raw_final_pos,
            self.raw_final_yaw,
            self.fused_pos_rmse,
            self.fused_yaw_rmse,
            self.fused_final_pos,
            self.fused_final_yaw,
            self.true_bias,
            self.est_bias,
            self.est_bias_sigma,
            self.yaw_within_3sigma,
            self.rejected as f64,
            self.raw_nees,
            self.fused_nees,
            self.fused_nis2,
            self.nis2_samples as f64,
            self.fused_nis1,
            self.nis1_samples as f64,
            self.raw_yaw_within_3sigma,
            self.gyro_only_yaw_rmse,
            self.wheel_accepted as f64,
            self.applied_late as f64,
            self.rejected_out_of_order as f64,
        ]
    }
}

/// A finished run: the per-sample rows and the summary.
#[derive(Debug, Clone)]
pub struct Trace {
    pub rows: Vec<f64>,
    pub metrics: Metrics,
}

/// The mixed Phase 2 drive of `docs/STATE_ESTIMATION.md` §4.1 on Rusty, recording a trace.
///
/// Same calls as `run_pipeline`: `RustySim` → `WheelOdometry` (raw) and `OdomImuFusion`
/// (fused), errors against the simulator's truth at every wheel sample. `wheel_delay`
/// (seconds, rounded to physics steps) delays the *delivery* of each wheel sample to the
/// estimator, which keeps its own stamp, so the filter sees the wheel stream late relative
/// to the gyro (`0` reproduces `run_pipeline`).
pub fn run_traced(
    cfg: RustyConfig,
    fusion_cfg: FusionConfig,
    seconds: f64,
    wheel_delay: f64,
) -> Result<Trace, String> {
    let mut sim = RustySim::new(cfg.clone(), Trajectory::mixed(true)).map_err(|e| e.to_string())?;
    let mut odom = WheelOdometry::new(cfg.params, OdometryNoise::default()).map_err(|e| e.to_string())?;
    let mut fusion = OdomImuFusion::new(fusion_cfg).map_err(|e| e.to_string())?;
    let w0 = sim.initial_wheel_sample();
    odom.update_from_wheel_angles(w0.left_angle, w0.right_angle, w0.stamp);

    let (mut rp, mut ry, mut fp, mut fy) = (0.0, 0.0, 0.0, 0.0);
    let mut n = 0usize;
    let mut within = 0usize;
    let mut raw_within = 0usize;
    let mut gyro_sq = 0.0;
    let mut m = Metrics::default();
    let mut raw_nees = ConsistencyStats::new(3).map_err(|e| e.to_string())?;
    let mut fused_nees = ConsistencyStats::new(3).map_err(|e| e.to_string())?;
    let mut nis1 = ConsistencyStats::new(1).map_err(|e| e.to_string())?;
    let mut nis2 = ConsistencyStats::new(2).map_err(|e| e.to_string())?;
    let steps = (seconds / cfg.physics_dt).round() as usize;
    let delay_steps = (wheel_delay / cfg.physics_dt).round().max(0.0) as usize;

    // Truth by physics step, so a delayed delivery can be graded against the truth at the
    // time the estimate is for. Entry k is the truth after step k (entry 0: the start).
    let mut truth_by_step: Vec<Se2> = Vec::with_capacity(steps + 1);
    truth_by_step.push(Se2::IDENTITY);
    let mut queue: VecDeque<(usize, robotics_sim::WheelSample)> = VecDeque::new();
    // Strawman: the gyro integrated as it is, with no bias estimate. Not code the robot runs.
    let mut gyro_theta = 0.0f64;
    let imu_dt = 1.0 / cfg.imu_rate_hz;
    let start = cfg.start;
    let mut rows: Vec<f64> = Vec::with_capacity(COLUMNS.len() * (steps / 4 + 1));

    for step in 1..=steps {
        let s = sim.step();
        truth_by_step.push(s.truth.pose);
        if let Some(imu) = &s.imu {
            gyro_theta += imu.yaw_rate * imu_dt;
            if fusion.on_imu(imu).is_err() {
                m.rejected += 1;
            }
        }
        if let Some(w) = s.wheel {
            queue.push_back((step + delay_steps, w));
        }
        while queue.front().is_some_and(|(due, _)| *due <= step) {
            let (_, w) = queue.pop_front().expect("front checked");
            let Some(wstep) = odom.update_from_wheel_angles(w.left_angle, w.right_angle, w.stamp)
            else {
                continue;
            };
            let mut nis_now = (f64::NAN, f64::NAN);
            if fusion.on_wheel(w.stamp, &wstep.twist).is_err() {
                m.rejected += 1;
            } else if let Some((v, dof)) = fusion.last_nis() {
                let _ = if dof == 2 { nis2.push(v) } else { nis1.push(v) };
                nis_now = (v, dof as f64);
            }
            let Some(est) = fusion.estimate() else {
                continue;
            };
            let truth_raw = truth_by_step[((w.stamp - start) / cfg.physics_dt).round() as usize];
            let truth_fus = truth_by_step
                [(((est.stamp - start) / cfg.physics_dt).round() as usize).min(truth_by_step.len() - 1)];
            let raw = odom.pose();
            let e_rp = raw.distance_to(&truth_raw);
            let e_ry = angle_diff(raw.theta(), truth_raw.theta());
            let e_fp = est.pose.distance_to(&truth_fus);
            let e_fy = angle_diff(est.pose.theta(), truth_fus.theta());
            rp += e_rp * e_rp;
            ry += e_ry * e_ry;
            fp += e_fp * e_fp;
            fy += e_fy * e_fy;
            let e_gy = angle_diff(gyro_theta, truth_raw.theta());
            gyro_sq += e_gy * e_gy;
            if e_fy.abs() <= 3.0 * est.pose_cov[(2, 2)].sqrt() {
                within += 1;
            }
            let raw_cov = odom.covariance();
            if e_ry.abs() <= 3.0 * raw_cov[(2, 2)].sqrt() {
                raw_within += 1;
            }
            // NEES in the tangent space at each estimate (right perturbation, body frame),
            // the space both covariances live in. The raw covariance starts at zero and
            // is skipped until positive definite.
            let mut nees_raw_now = f64::NAN;
            if let Ok(v) = nees(&truth_raw.boxminus(&raw), &raw_cov) {
                let _ = raw_nees.push(v);
                nees_raw_now = v;
            }
            let mut nees_fus_now = f64::NAN;
            if let Some(f) = fusion.filter() {
                if let Ok(v) = nees(&truth_fus.boxminus(&est.pose), &f.pose_covariance_body()) {
                    let _ = fused_nees.push(v);
                    nees_fus_now = v;
                }
            }
            n += 1;
            m.raw_final_pos = e_rp;
            m.raw_final_yaw = e_ry;
            m.fused_final_pos = e_fp;
            m.fused_final_yaw = e_fy;
            m.true_bias = s.truth.gyro_bias;
            m.est_bias = est.gyro_bias;
            m.est_bias_sigma = est.gyro_bias_sigma;

            // The raw covariance lives in the body frame; rotate its position block into
            // the world frame to draw it next to the fused one (ROS convention).
            let r = raw.rotation();
            let rb = r * raw_cov.fixed_view::<2, 2>(0, 0).into_owned() * r.matrix().transpose();
            rows.extend_from_slice(&[
                w.stamp - start,
                truth_raw.x(),
                truth_raw.y(),
                truth_raw.theta(),
                raw.x(),
                raw.y(),
                raw.theta(),
                est.pose.x(),
                est.pose.y(),
                est.pose.theta(),
                e_rp,
                e_ry,
                e_fp,
                e_fy,
                e_gy,
                raw_cov[(2, 2)].sqrt(),
                est.pose_cov[(2, 2)].sqrt(),
                nees_raw_now,
                nees_fus_now,
                raw_nees.mean(),
                fused_nees.mean(),
                nis_now.0,
                nis_now.1,
                nis2.mean(),
                nis1.mean(),
                m.true_bias,
                m.est_bias,
                m.est_bias_sigma,
                est.pose_cov[(0, 0)],
                est.pose_cov[(0, 1)],
                est.pose_cov[(1, 1)],
                rb[(0, 0)],
                rb[(0, 1)],
                rb[(1, 1)],
            ]);
        }
    }
    let nf = n.max(1) as f64;
    m.samples = n;
    m.raw_pos_rmse = (rp / nf).sqrt();
    m.raw_yaw_rmse = (ry / nf).sqrt();
    m.fused_pos_rmse = (fp / nf).sqrt();
    m.fused_yaw_rmse = (fy / nf).sqrt();
    m.yaw_within_3sigma = within as f64 / nf;
    m.raw_yaw_within_3sigma = raw_within as f64 / nf;
    m.gyro_only_yaw_rmse = (gyro_sq / nf).sqrt();
    m.raw_nees = raw_nees.mean();
    m.fused_nees = fused_nees.mean();
    m.fused_nis2 = nis2.mean();
    m.nis2_samples = nis2.samples();
    m.fused_nis1 = nis1.mean();
    m.nis1_samples = nis1.samples();
    let stats = fusion.stats();
    m.wheel_accepted = stats.wheel_accepted;
    m.applied_late = stats.applied_late;
    m.rejected_out_of_order = stats.rejected_out_of_order;
    debug_assert_eq!(rows.len(), n * COLUMNS.len());
    Ok(Trace { rows, metrics: m })
}

/// Rusty's default configuration with the knobs the widgets expose, LiDAR off (see the
/// module docs). With `slip_sigma = 0.02`, `right_radius_error = 0.002`,
/// `gyro_initial_bias = 0.005` this is `RustyConfig::default()` apart from the LiDAR.
pub fn rusty_config(seed: u64, slip_sigma: f64, right_radius_error: f64, gyro_initial_bias: f64) -> RustyConfig {
    RustyConfig {
        seed,
        slip_sigma,
        right_radius_error,
        gyro_initial_bias,
        lidar: None,
        ..RustyConfig::default()
    }
}

/// The fusion configuration with `bias_update_max_speed` set (`+∞` allowed).
pub fn fusion_config(bias_update_max_speed: f64) -> FusionConfig {
    FusionConfig {
        bias_update_max_speed,
        ..FusionConfig::default()
    }
}

/// A recorded run, for the widgets. `rows` is row-major with [`ch02_trace_columns`] columns,
/// one row per wheel sample (50 Hz); `summary` is [`ch02_summary_names`] long.
#[wasm_bindgen]
pub struct Ch02Trace {
    rows: Vec<f64>,
    summary: Vec<f64>,
}

#[wasm_bindgen]
impl Ch02Trace {
    /// Row-major table, `samples × 34` (see `ch02_trace_columns`). Missing values are NaN
    /// (for example the NIS column at a sample with no correction).
    #[wasm_bindgen(getter)]
    pub fn rows(&self) -> Vec<f64> {
        self.rows.clone()
    }
    /// The summary statistics (see `ch02_summary_names`).
    #[wasm_bindgen(getter)]
    pub fn summary(&self) -> Vec<f64> {
        self.summary.clone()
    }
}

/// Column names of [`Ch02Trace::rows`], in order.
#[wasm_bindgen]
pub fn ch02_trace_columns() -> Vec<String> {
    COLUMNS.iter().map(|s| s.to_string()).collect()
}

/// Names of the entries of [`Ch02Trace::summary`], in order.
#[wasm_bindgen]
pub fn ch02_summary_names() -> Vec<String> {
    SUMMARY.iter().map(|s| s.to_string()).collect()
}

/// Run the mixed Phase 2 drive for `seconds` and record it.
///
/// `slip_sigma` (per wheel per 5 ms step, dimensionless), `right_radius_error`
/// (dimensionless) and `gyro_initial_bias` (rad/s) are the simulator's error sources; the
/// documented configuration is `0.02, 0.002, 0.005`. `bias_update_max_speed` (m/s, `Infinity`
/// allowed) is the fusion setting of `docs/STATE_ESTIMATION.md` §4.1; its default is `0.05`.
/// `wheel_delay` (s) delays the delivery of each wheel sample to the estimator (`0` is the
/// documented pipeline).
#[wasm_bindgen]
pub fn ch02_run(
    seed: u64,
    seconds: f64,
    slip_sigma: f64,
    right_radius_error: f64,
    gyro_initial_bias: f64,
    bias_update_max_speed: f64,
    wheel_delay: f64,
) -> Result<Ch02Trace, String> {
    let t = run_traced(
        rusty_config(seed, slip_sigma, right_radius_error, gyro_initial_bias),
        fusion_config(bias_update_max_speed),
        seconds,
        wheel_delay,
    )?;
    Ok(Ch02Trace {
        rows: t.rows,
        summary: t.metrics.to_vec(),
    })
}

/// Only the summary of [`ch02_run`] with the documented configuration (the table of
/// `docs/STATE_ESTIMATION.md` §4.1): `0.02, 0.002, 0.005`, no delay.
#[wasm_bindgen]
pub fn ch02_table_row(seed: u64, seconds: f64, bias_update_max_speed: f64) -> Result<Vec<f64>, String> {
    Ok(run_traced(rusty_config(seed, 0.02, 0.002, 0.005), fusion_config(bias_update_max_speed), seconds, 0.0)?
        .metrics
        .to_vec())
}

/// Summary of the noise-free run (`RustyConfig::noiseless()`, LiDAR off) with the default
/// fusion configuration.
#[wasm_bindgen]
pub fn ch02_noiseless_summary(seconds: f64) -> Result<Vec<f64>, String> {
    let cfg = RustyConfig {
        lidar: None,
        ..RustyConfig::noiseless()
    };
    Ok(run_traced(cfg, FusionConfig::default(), seconds, 0.0)?.metrics.to_vec())
}

/// Acceptance band for the time-averaged NEES/NIS: for each `n` in `counts`, the
/// `[lower, upper]` of `robotics_core::prob::chi2_mean_interval(dof, n, alpha)`, flattened as
/// `[lo₀, hi₀, lo₁, hi₁, …]`.
#[wasm_bindgen]
pub fn ch02_chi2_mean_band(dof: usize, counts: &[f64], alpha: f64) -> Result<Vec<f64>, String> {
    let mut out = Vec::with_capacity(2 * counts.len());
    for &n in counts {
        let (lo, hi) = chi2_mean_interval(dof, n.max(1.0) as usize, alpha).map_err(|e| e.to_string())?;
        out.extend_from_slice(&[lo, hi]);
    }
    Ok(out)
}

/// The `k`-sigma ellipse of a 2×2 covariance `[[sxx, sxy], [sxy, syy]]`, from its
/// symmetric eigendecomposition: `[semi-major, semi-minor, angle of the major axis (rad)]`.
#[wasm_bindgen]
pub fn ch02_ellipse(sxx: f64, sxy: f64, syy: f64, k: f64) -> Vec<f64> {
    let e = Matrix2::new(sxx, sxy, sxy, syy).symmetric_eigen();
    let (big, small) = if e.eigenvalues[0] >= e.eigenvalues[1] { (0, 1) } else { (1, 0) };
    let v = e.eigenvectors.column(big);
    vec![
        k * e.eigenvalues[big].max(0.0).sqrt(),
        k * e.eigenvalues[small].max(0.0).sqrt(),
        v[1].atan2(v[0]),
    ]
}

/// The fusion front-end's defaults the chapter talks about, so the page never repeats a
/// constant that lives in the crate: `[bias_update_max_speed (m/s), max_measurement_delay (s),
/// max_dt (s)]` of `FusionConfig::default()`.
#[wasm_bindgen]
pub fn ch02_fusion_defaults() -> Vec<f64> {
    let c = FusionConfig::default();
    vec![c.bias_update_max_speed, c.max_measurement_delay, c.max_dt]
}

/// The first 64-bit word of `robotics_core::seeded_rng(seed)`.
///
/// `seeded_rng` returns `rand`'s `SmallRng`, which is Xoshiro256++ where `usize` is 64 bits
/// and Xoshiro128++ where it is 32 bits (`rand` 0.10, `rngs/small.rs`). WebAssembly is
/// 32-bit, so a seed names a different noise stream here than on the robot. The chapter
/// states this, and `checks/ch02.ts` pins this word so that a change on either side
/// (a new `rand`, or a fixed-algorithm generator in `robotics_core`) fails the build until
/// the text is revisited.
#[wasm_bindgen]
pub fn ch02_seeded_rng_probe(seed: u64) -> u64 {
    use rand::Rng;
    robotics_core::seeded_rng(seed).next_u64()
}

/// The book's 1-D cart (`A = B = C = 1`, `R = 0.25`, `Q = 0.5`, prior `N(0, 1)`) driven by
/// controls `u1, u2` and measurements `z1, z2`, run three ways. Layout of the result:
///
/// | index | value |
/// |---|---|
/// | 0, 1 | information filter after predict 1: `Ω̄₁`, `ξ̄₁` |
/// | 2 | evidence `N(z1; μ̄₁ , S)` returned by the information filter's `correct` at step 1 |
/// | 3, 4 | Kalman filter at t = 1: `μ₁`, `Σ₁` |
/// | 5, 6 | Kalman filter at t = 2: `μ₂`, `Σ₂` |
/// | 7, 8 | information filter at t = 2, converted back: `μ₂`, `Σ₂` |
/// | 9, 10 | RTS smoothed `μ₀|₂`, `Σ₀|₂` |
/// | 11, 12 | RTS smoothed `μ₁|₂`, `Σ₁|₂` |
/// | 13, 14 | RTS smoothed `μ₂|₂`, `Σ₂|₂` (equal to the filtered values) |
#[wasm_bindgen]
pub fn ch02_cart(u1: f64, z1: f64, u2: f64, z2: f64) -> Result<Vec<f64>, String> {
    let one = Matrix1::new(1.0);
    let prior = Gaussian::new(Vector1::new(0.0), one).map_err(|e| e.to_string())?;
    let (r, q) = (Matrix1::new(0.25), Matrix1::new(0.5));
    let mut kf = KalmanFilter::<1, 1, 1>::new(one, one, one, r, q, prior.clone()).map_err(|e| e.to_string())?;
    let mut inf = InformationFilter::<1, 1, 1>::from_gaussian(one, one, one, r, q, &prior).map_err(|e| e.to_string())?;
    let mut rts = RtsSmoother::new(prior);
    let mut out = vec![0.0; 15];
    for (k, (u, z)) in [(u1, z1), (u2, z2)].into_iter().enumerate() {
        let (u, z) = (Vector1::new(u), Vector1::new(z));
        kf.predict(&u).map_err(|e| e.to_string())?;
        let predicted = kf.belief().clone();
        kf.correct(&z).map_err(|e| e.to_string())?;
        rts.push(one, predicted, kf.belief().clone()).map_err(|e| e.to_string())?;
        inf.predict(&u).map_err(|e| e.to_string())?;
        if k == 0 {
            out[0] = inf.omega()[(0, 0)];
            out[1] = inf.xi()[0];
        }
        let evidence = inf.correct(&z).map_err(|e| e.to_string())?;
        if k == 0 {
            out[2] = evidence;
            out[3] = kf.belief().mean()[0];
            out[4] = kf.belief().cov()[(0, 0)];
        }
    }
    out[5] = kf.belief().mean()[0];
    out[6] = kf.belief().cov()[(0, 0)];
    out[7] = inf.mean().map_err(|e| e.to_string())?[0];
    out[8] = inf.covariance().map_err(|e| e.to_string())?[(0, 0)];
    let s = rts.smooth().map_err(|e| e.to_string())?;
    for (i, g) in s.iter().enumerate() {
        out[9 + 2 * i] = g.mean()[0];
        out[10 + 2 * i] = g.cov()[(0, 0)];
    }
    Ok(out)
}

/// The original pipeline, included from where `examples/estimator_accuracy.rs` and
/// `tests/estimation_pipeline.rs` of `robotics_sim` include it. Not copied. (It sits next to
/// the test module rather than inside it: a `#[path]` inside an inline module of a
/// non-`mod.rs` file is resolved against `src/ch02/tests/`, which does not exist.)
#[cfg(test)]
#[path = "../../robotics_sim/tests/common/pipeline.rs"]
mod pipeline;

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;


    fn assert_same(seed: u64, bias_update_max_speed: f64, seconds: f64) {
        let cfg = RustyConfig {
            seed,
            ..RustyConfig::default()
        };
        let fcfg = fusion_config(bias_update_max_speed);
        let want = pipeline::run_pipeline(cfg, fcfg, seconds);
        let got = run_traced(rusty_config(seed, 0.02, 0.002, 0.005), fcfg, seconds, 0.0)
            .expect("trace")
            .metrics;
        let same = |name: &str, a: f64, b: f64| {
            assert_eq!(a.to_bits(), b.to_bits(), "{name} (seed {seed}, bias_update_max_speed {bias_update_max_speed}): trace {a} vs run_pipeline {b}");
        };
        assert_eq!(got.samples, want.samples);
        assert_eq!(got.rejected, want.rejected);
        assert_eq!(got.nis2_samples, want.nis2_samples);
        assert_eq!(got.nis1_samples, want.nis1_samples);
        same("raw_pos_rmse", got.raw_pos_rmse, want.raw_pos_rmse);
        same("raw_yaw_rmse", got.raw_yaw_rmse, want.raw_yaw_rmse);
        same("raw_final_pos", got.raw_final_pos, want.raw_final_pos);
        same("raw_final_yaw", got.raw_final_yaw, want.raw_final_yaw);
        same("fused_pos_rmse", got.fused_pos_rmse, want.fused_pos_rmse);
        same("fused_yaw_rmse", got.fused_yaw_rmse, want.fused_yaw_rmse);
        same("fused_final_pos", got.fused_final_pos, want.fused_final_pos);
        same("fused_final_yaw", got.fused_final_yaw, want.fused_final_yaw);
        same("true_bias", got.true_bias, want.true_bias);
        same("est_bias", got.est_bias, want.est_bias);
        same("est_bias_sigma", got.est_bias_sigma, want.est_bias_sigma);
        same("yaw_within_3sigma", got.yaw_within_3sigma, want.yaw_within_3sigma);
        same("raw_nees", got.raw_nees, want.raw_nees);
        same("fused_nees", got.fused_nees, want.fused_nees);
        same("fused_nis2", got.fused_nis2, want.fused_nis2);
        same("fused_nis1", got.fused_nis1, want.fused_nis1);
    }

    /// The documented seeds, both variants of §4.1, the full 180 s: every metric of the
    /// trace pipeline is bit-identical to `run_pipeline`.
    #[test]
    fn trace_equals_run_pipeline_for_the_documented_runs() {
        for seed in 1..=5u64 {
            assert_same(seed, FusionConfig::default().bias_update_max_speed, 180.0);
            assert_same(seed, f64::INFINITY, 180.0);
        }
    }

    #[test]
    fn trace_equals_run_pipeline_noise_free() {
        let want = pipeline::run_pipeline(RustyConfig::noiseless(), FusionConfig::default(), 60.0);
        let cfg = RustyConfig {
            lidar: None,
            ..RustyConfig::noiseless()
        };
        let got = run_traced(cfg, FusionConfig::default(), 60.0, 0.0).unwrap().metrics;
        assert_eq!(got.raw_pos_rmse.to_bits(), want.raw_pos_rmse.to_bits());
        assert_eq!(got.fused_yaw_rmse.to_bits(), want.fused_yaw_rmse.to_bits());
        assert_eq!(got.fused_pos_rmse.to_bits(), want.fused_pos_rmse.to_bits());
    }

    /// The rows are self-consistent: the last running means are the summary's, and the
    /// error columns equal the pose columns' differences.
    #[test]
    fn rows_agree_with_the_summary() {
        let t = run_traced(rusty_config(1, 0.02, 0.002, 0.005), fusion_config(0.05), 20.0, 0.0).unwrap();
        let c = COLUMNS.len();
        let n = t.metrics.samples;
        assert_eq!(t.rows.len(), n * c);
        let last = &t.rows[(n - 1) * c..];
        assert_eq!(last[19].to_bits(), t.metrics.raw_nees.to_bits());
        assert_eq!(last[20].to_bits(), t.metrics.fused_nees.to_bits());
        assert_eq!(last[23].to_bits(), t.metrics.fused_nis2.to_bits());
        assert_eq!(last[12].to_bits(), t.metrics.fused_final_pos.to_bits());
        let dx = last[7] - last[1];
        let dy = last[8] - last[2];
        assert_relative_eq!((dx * dx + dy * dy).sqrt(), last[12], epsilon = 1e-12);
    }

    /// A wheel stream delayed within `max_measurement_delay` (50 ms) is applied late and
    /// loses little; delayed beyond it every wheel sample is rejected (documented in
    /// `docs/STATE_ESTIMATION.md` §6) and the estimate no longer moves.
    #[test]
    fn delayed_wheel_samples_follow_the_documented_policy() {
        let run = |d: f64| run_traced(rusty_config(1, 0.02, 0.002, 0.005), fusion_config(0.05), 30.0, d).unwrap().metrics;
        let on_time = run(0.0);
        let late = run(0.03);
        let too_late = run(0.1);
        assert_eq!(on_time.applied_late, 0);
        assert!(late.applied_late > 0 && late.rejected_out_of_order == 0, "{late:?}");
        assert_eq!(too_late.wheel_accepted, 0, "{too_late:?}");
        assert!(too_late.rejected_out_of_order > 0);
        assert!(too_late.fused_pos_rmse > 3.0 * on_time.fused_pos_rmse, "{too_late:?} {on_time:?}");
    }

    /// The table of `docs/STATE_ESTIMATION.md` §4.1, reproduced through the trace pipeline
    /// to the precision the document prints. These are the repository's recorded numbers; they
    /// hold where `SmallRng` is Xoshiro256++ (64-bit), so the module is skipped on 32-bit
    /// targets, and the chapter quotes them as native results.
    #[cfg(target_pointer_width = "64")]
    #[test]
    fn the_documented_table_is_reproduced_natively() {
        // seed: raw pos, raw yaw, fused pos, fused yaw, true bias, est bias, yaw in 3σ %, NEES raw, NEES fused, NIS (2 DoF)
        let want = [
            (1u64, "0.710", "0.2564", "0.152", "0.0476", "0.00584", "0.00561", "100", "185.24", "0.45", "0.90"),
            (2, "0.700", "0.2433", "0.317", "0.1475", "0.00565", "0.00628", "100", "176.28", "2.29", "0.95"),
            (3, "0.923", "0.3123", "0.175", "0.0516", "0.00392", "0.00457", "100", "274.14", "0.50", "0.97"),
            (4, "0.687", "0.2264", "0.255", "0.0659", "0.00472", "0.00520", "100", "145.37", "0.95", "0.96"),
            (5, "0.887", "0.3085", "0.108", "0.0347", "0.00479", "0.00563", "100", "291.30", "0.38", "0.98"),
        ];
        for (seed, rp, ry, fp, fy, tb, eb, cov, nr, nf, nis) in want {
            let m = run_traced(rusty_config(seed, 0.02, 0.002, 0.005), fusion_config(0.05), 180.0, 0.0).unwrap().metrics;
            let got = [
                format!("{:.3}", m.raw_pos_rmse),
                format!("{:.4}", m.raw_yaw_rmse),
                format!("{:.3}", m.fused_pos_rmse),
                format!("{:.4}", m.fused_yaw_rmse),
                format!("{:.5}", m.true_bias),
                format!("{:.5}", m.est_bias),
                format!("{:.0}", 100.0 * m.yaw_within_3sigma),
                format!("{:.2}", m.raw_nees),
                format!("{:.2}", m.fused_nees),
                format!("{:.2}", m.fused_nis2),
            ];
            let got: Vec<&str> = got.iter().map(String::as_str).collect();
            assert_eq!(got, [rp, ry, fp, fy, tb, eb, cov, nr, nf, nis], "seed {seed}");
        }
        // "With the bias updated at every speed the fused errors grow to 0.62-0.79 m /
        // 0.19-0.27 rad and the yaw ±3σ coverage drops to 42-89 %."
        let all: Vec<Metrics> = (1..=5u64)
            .map(|seed| run_traced(rusty_config(seed, 0.02, 0.002, 0.005), fusion_config(f64::INFINITY), 180.0, 0.0).unwrap().metrics)
            .collect();
        let range = |f: fn(&Metrics) -> f64, d: usize| {
            let lo = all.iter().map(f).fold(f64::INFINITY, f64::min);
            let hi = all.iter().map(f).fold(f64::NEG_INFINITY, f64::max);
            (format!("{lo:.d$}"), format!("{hi:.d$}"))
        };
        assert_eq!(range(|m| m.fused_pos_rmse, 2), ("0.62".into(), "0.79".into()));
        assert_eq!(range(|m| m.fused_yaw_rmse, 2), ("0.19".into(), "0.27".into()));
        assert_eq!(range(|m| 100.0 * m.yaw_within_3sigma, 0), ("42".into(), "89".into()));
    }

    /// The noise-free 60 s run of `docs/STATE_ESTIMATION.md` §4.1 does not depend on any
    /// random stream, so it holds on every target; `checks/ch02.ts` asserts the same numbers
    /// on the compiled module.
    #[test]
    fn the_noise_free_numbers_are_reproduced() {
        let v = ch02_noiseless_summary(60.0).unwrap();
        assert!(v[1] < 2.0e-9, "raw position RMSE {}", v[1]);
        assert!(v[6] < 3.0e-8, "fused heading RMSE {}", v[6]);
        assert_eq!(format!("{:.1}", 1000.0 * v[5]), "5.7");
    }

    /// Pinned native value of the probe; see [`ch02_seeded_rng_probe`].
    #[cfg(target_pointer_width = "64")]
    #[test]
    fn seeded_rng_first_word_on_64_bit() {
        assert_eq!(ch02_seeded_rng_probe(1), 14_971_601_782_005_023_387);
    }

    #[test]
    fn fusion_defaults_are_the_documented_ones() {
        assert_eq!(ch02_fusion_defaults(), vec![0.05, 0.05, 0.5]);
    }

    #[test]
    fn toolbox_wrappers_reproduce_the_book_examples() {
        let c = ch02_cart(1.0, 1.7, 1.0, 2.1).unwrap();
        assert_relative_eq!(c[0], 0.8, epsilon = 1e-12);
        assert_relative_eq!(c[1], 0.8, epsilon = 1e-12);
        assert_relative_eq!(c[5], 2.280645161290323, epsilon = 1e-12);
        assert_relative_eq!(c[6], 0.2741935483870968, epsilon = 1e-12);
        assert_relative_eq!(c[7], c[5], epsilon = 1e-12);
        assert_relative_eq!(c[11], 85.0 / 62.0, epsilon = 1e-12);
        assert_relative_eq!(c[12], 15.0 / 62.0, epsilon = 1e-12);
        assert_relative_eq!(c[9], 18.4 / 62.0, epsilon = 1e-12);
        assert_relative_eq!(c[10], 22.0 / 62.0, epsilon = 1e-12);
        let e = ch02_ellipse(4.0, 0.0, 1.0, 3.0);
        assert_relative_eq!(e[0], 6.0, epsilon = 1e-12);
        assert_relative_eq!(e[1], 3.0, epsilon = 1e-12);
    }
}
