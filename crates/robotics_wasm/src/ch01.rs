//! Chapter 1 bindings (owned by the chapter; see web/design/Chapter-01.md).
//!
//! Thin wrappers over `robotics_core`, `robotics_models` and `robotics_filters`. The only
//! logic in this file is plumbing (flattening results, drawing a *simulated* door sensor
//! reading from a seeded generator) and one deliberate strawman, `ch01_integrate_twist`
//! with `exact = false`, which is not code the robot runs.

use rand::RngExt;
use rand_distr::{Distribution, Normal};
use robotics_core::{seeded_rng, Gaussian, Manifold, Se2, Tangent2};
use robotics_filters::{BayesFilter, HistogramFilter1D, KalmanFilter};
use robotics_models::diff_drive::DiffDriveParams;
use robotics_models::motion::{MotionModel, VelocityAlphas, VelocityCmd, VelocityMotionModel};
use wasm_bindgen::prelude::*;

/// Dead-reckon a constant body twist `(v, ω)` for `steps` steps of `dt` seconds, starting at
/// the origin, and return the poses as `[x₀, y₀, θ₀, x₁, y₁, θ₁, …]` (`steps + 1` poses).
///
/// With `exact = true` each step is `x ⊞ exp((v·dt, 0, ω·dt))`, the constant-curvature arc
/// the robot's odometry uses (`Se2::boxplus`). With `exact = false` it is the textbook
/// Euler step `x += v·cos θ·dt; y += v·sin θ·dt; θ += ω·dt`, which is *not* code the robot
/// runs: it is the strawman the chapter measures the exact step against.
#[wasm_bindgen]
pub fn ch01_integrate_twist(v: f64, omega: f64, dt: f64, steps: usize, exact: bool) -> Vec<f64> {
    let mut out = Vec::with_capacity(3 * (steps + 1));
    let mut pose = Se2::IDENTITY;
    out.extend_from_slice(&[pose.x(), pose.y(), pose.theta()]);
    for _ in 0..steps {
        pose = if exact {
            pose.boxplus(&Tangent2::new(v * dt, 0.0, omega * dt))
        } else {
            let th = pose.theta();
            Se2::new(pose.x() + v * th.cos() * dt, pose.y() + v * th.sin() * dt, th + omega * dt)
        };
        out.extend_from_slice(&[pose.x(), pose.y(), pose.theta()]);
    }
    out
}

/// `Se2::exp((vx, vy, ω))` as `[x, y, θ]`.
#[wasm_bindgen]
pub fn ch01_se2_exp(vx: f64, vy: f64, omega: f64) -> Vec<f64> {
    let p = Se2::exp(&Tangent2::new(vx, vy, omega));
    vec![p.x(), p.y(), p.theta()]
}

/// `Se2::new(x, y, θ).log()` as `[v_x, v_y, ω]`.
#[wasm_bindgen]
pub fn ch01_se2_log(x: f64, y: f64, theta: f64) -> Vec<f64> {
    let t = Se2::new(x, y, theta).log();
    vec![t[0], t[1], t[2]]
}

fn pose_of(p: &[f64], what: &str) -> Result<Se2, String> {
    match p {
        [x, y, th] => Ok(Se2::new(*x, *y, *th)),
        _ => Err(format!("{what}: expected [x, y, theta], got {} numbers", p.len())),
    }
}

/// `a · b` (apply `b` in the body frame of `a`) as `[x, y, θ]`; both inputs `[x, y, θ]`.
#[wasm_bindgen]
pub fn ch01_se2_compose(a: &[f64], b: &[f64]) -> Result<Vec<f64>, String> {
    let p = pose_of(a, "a")?.compose(&pose_of(b, "b")?);
    Ok(vec![p.x(), p.y(), p.theta()])
}

/// `a⁻¹` as `[x, y, θ]`; input `[x, y, θ]`.
#[wasm_bindgen]
pub fn ch01_se2_inverse(a: &[f64]) -> Result<Vec<f64>, String> {
    let p = pose_of(a, "a")?.inverse();
    Ok(vec![p.x(), p.y(), p.theta()])
}

/// One odometry update of the book's rover Rusty (`DiffDriveParams::RUSTY`: `r = 0.033 m`,
/// `ℓ = 0.16 m`) from the arc lengths the two wheels rolled (m).
///
/// Returns `[Δs, Δθ, x, y, θ]`: the body increment `(Δs, 0, Δθ)` and the pose `exp` of it.
#[wasm_bindgen]
pub fn ch01_wheel_arcs(ds_left: f64, ds_right: f64) -> Vec<f64> {
    let tau = DiffDriveParams::RUSTY.wheel_arcs_to_tangent(ds_left, ds_right);
    let p = Se2::exp(&tau);
    vec![tau[0], tau[2], p.x(), p.y(), p.theta()]
}

/// Sample the velocity motion model (`VelocityMotionModel::sample`, Thrun Table 5.3).
///
/// `n` robots start at the origin and each applies the command `(v, ω)` for `dt` seconds,
/// `steps` times in a row, with independent noise per step. The noise parameters are the
/// book's `VelocityAlphas::DEFAULT` multiplied by `alpha_scale`.
///
/// Layout: sample-major, `n × (steps + 1) × 3` numbers, `[x, y, θ]` for step 0..=steps of
/// sample 0, then sample 1, and so on. Step 0 is the origin.
#[wasm_bindgen]
pub fn ch01_velocity_samples(
    seed: u64,
    n: usize,
    v: f64,
    omega: f64,
    dt: f64,
    steps: usize,
    alpha_scale: f64,
) -> Result<Vec<f64>, String> {
    let mut alphas = VelocityAlphas::DEFAULT.0;
    alphas.iter_mut().for_each(|a| *a *= alpha_scale);
    let model = VelocityMotionModel::new(VelocityAlphas(alphas)).map_err(|e| e.to_string())?;
    let cmd = VelocityCmd::new(v, omega, dt).map_err(|e| e.to_string())?;
    let mut rng = seeded_rng(seed);
    let mut out = Vec::with_capacity(n * (steps + 1) * 3);
    for _ in 0..n {
        let mut pose = Se2::IDENTITY;
        out.extend_from_slice(&[pose.x(), pose.y(), pose.theta()]);
        for _ in 0..steps {
            pose = model.sample(&pose, &cmd, &mut rng);
            out.extend_from_slice(&[pose.x(), pose.y(), pose.theta()]);
        }
    }
    Ok(out)
}

/// Corridor of the hallway example: length (m) and number of cells.
const HALL_LENGTH: f64 = 10.0;
const HALL_CELLS: usize = 100;
/// Doors, as in `histogram::tests::sensing_sharpens_moving_smears`.
const DOORS: [f64; 3] = [2.0, 4.5, 7.5];
const DOOR_HALF_WIDTH: f64 = 0.3;
/// `P(sensor says "door" | at a door)`; `P(says "door" | not at a door) = 1 − P_DOOR`.
const P_DOOR: f64 = 0.9;
/// Where the simulated robot starts (m), and how close to a wall it can get (m).
const HALL_START: f64 = 0.25;
const WALL_MARGIN: f64 = 0.01;

fn near_door(x: f64) -> bool {
    DOORS.iter().any(|d| (x - d).abs() < DOOR_HALF_WIDTH)
}

/// The door positions (m) and the half-width of a door (m), `[d₀, d₁, d₂, half_width]`.
#[wasm_bindgen]
pub fn ch01_hallway_doors() -> Vec<f64> {
    vec![DOORS[0], DOORS[1], DOORS[2], DOOR_HALF_WIDTH]
}

/// Number of numbers per time step in [`ch01_hallway`]'s output.
#[wasm_bindgen]
pub fn ch01_hallway_block() -> usize {
    7 + 2 * HALL_CELLS
}

/// A robot walks a 10 m corridor with three doors, moving `step_m` per step, with true
/// motion noise `N(0, sigma²)` per step, and reads a noisy door detector after each move.
/// A `HistogramFilter1D` (100 cells) tracks it, using `sigma` as its motion kernel.
///
/// Output: `steps + 1` blocks of [`ch01_hallway_block`] numbers. Block `t` is
/// `[truth, z, mean, mode, entropy_bits, evidence, entropy_pred_bits, pred × 100, post × 100]`
/// where `z` is 1 if the detector said "door", 0 if not; `pred` is the belief before the
/// correction (`bel̄`) and `post` after it (`bel`); `mode` is the centre of the most probable
/// cell (m); among equal cells it is the lowest one. Block 0 has no motion: `pred` is the
/// uniform prior. The simulated robot stops at the end wall.
#[wasm_bindgen]
pub fn ch01_hallway(seed: u64, steps: usize, step_m: f64, sigma: f64) -> Result<Vec<f64>, String> {
    let mut filter = HistogramFilter1D::uniform(HALL_CELLS, HALL_LENGTH, false).map_err(|e| e.to_string())?;
    filter.set_gaussian_kernel(sigma);
    let motion = Normal::new(0.0, sigma.max(1e-12)).map_err(|e| e.to_string())?;
    let mut rng = seeded_rng(seed);
    let mut truth = HALL_START;
    let mut out = Vec::with_capacity((steps + 1) * (7 + 2 * HALL_CELLS));
    for t in 0..=steps {
        if t > 0 {
            truth += step_m + if sigma > 0.0 { motion.sample(&mut rng) } else { 0.0 };
            // The robot stops at the end wall; the filter is not told, which is what a robot
            // that bumps a wall does to its own odometry.
            truth = truth.clamp(0.0, HALL_LENGTH - WALL_MARGIN);
            filter.predict(&step_m).map_err(|e| e.to_string())?;
        }
        let pred = filter.belief().clone();
        let entropy_pred = filter.entropy_bits();
        let p_says_door = if near_door(truth) { P_DOOR } else { 1.0 - P_DOOR };
        let z = rng.random::<f64>() < p_says_door;
        let likelihood: Box<dyn Fn(f64) -> f64> = Box::new(move |x| {
            if z == near_door(x) {
                P_DOOR
            } else {
                1.0 - P_DOOR
            }
        });
        let evidence = filter.correct(&likelihood).map_err(|e| e.to_string())?;
        let mode = filter.center(filter.mode());
        out.extend_from_slice(&[
            truth,
            if z { 1.0 } else { 0.0 },
            filter.mean(),
            mode,
            filter.entropy_bits(),
            evidence,
            entropy_pred,
        ]);
        out.extend_from_slice(&pred);
        out.extend_from_slice(filter.belief());
    }
    Ok(out)
}

/// One sensor update of a uniform histogram belief over `likelihood.len()` cells of 1 m
/// each. Returns `[evidence, entropy_bits, mode_cell, belief…]`.
#[wasm_bindgen]
pub fn ch01_histogram_sense_uniform(likelihood: &[f64]) -> Result<Vec<f64>, String> {
    let n = likelihood.len();
    let mut f = HistogramFilter1D::uniform(n, n as f64, false).map_err(|e| e.to_string())?;
    let table = likelihood.to_vec();
    let lik: Box<dyn Fn(f64) -> f64> = Box::new(move |x| table[(x.floor() as usize).min(n - 1)]);
    let evidence = f.correct(&lik).map_err(|e| e.to_string())?;
    let mut out = vec![evidence, f.entropy_bits(), f.mode() as f64];
    out.extend_from_slice(f.belief());
    Ok(out)
}

/// One predict-then-correct step of the scalar Kalman filter (`KalmanFilter<1, 1, 1>` with
/// `A = B = C = 1`), prior `N(mu, var)`, control `u`, measurement `z`, motion noise `r` and
/// measurement noise `q` (Thrun naming).
///
/// Returns `[μ̄, Σ̄, μ, Σ, K, evidence, NIS, S]`: predicted mean and variance, posterior mean
/// and variance, Kalman gain, evidence `p(z | z₁:ₜ₋₁)`, the normalized innovation squared
/// and the innovation variance.
#[wasm_bindgen]
pub fn ch01_kf_step(mu: f64, var: f64, u: f64, z: f64, r: f64, q: f64) -> Result<Vec<f64>, String> {
    use nalgebra::{Matrix1, Vector1};
    let one = Matrix1::new(1.0);
    let prior = Gaussian::new(Vector1::new(mu), Matrix1::new(var)).map_err(|e| e.to_string())?;
    let mut kf = KalmanFilter::<1, 1, 1>::new(one, one, one, Matrix1::new(r), Matrix1::new(q), prior)
        .map_err(|e| e.to_string())?;
    kf.predict(&Vector1::new(u)).map_err(|e| e.to_string())?;
    let (mu_bar, var_bar) = (kf.mean()[0], kf.cov()[(0, 0)]);
    let evidence = kf.correct(&Vector1::new(z)).map_err(|e| e.to_string())?;
    let up = kf.last_update().ok_or("no update recorded")?;
    Ok(vec![
        mu_bar,
        var_bar,
        kf.mean()[0],
        kf.cov()[(0, 0)],
        up.gain[(0, 0)],
        evidence,
        up.nis,
        up.s[(0, 0)],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// With ω ≠ 0 the exact integrator stays on the circle of radius v/ω at any step size.
    #[test]
    fn exact_steps_stay_on_the_circle() {
        let (v, w) = (1.0, 0.5);
        let r = v / w;
        let p = ch01_integrate_twist(v, w, 2.0, 20, true);
        for k in 0..=20 {
            let (x, y) = (p[3 * k], p[3 * k + 1]);
            assert_relative_eq!((x * x + (y - r) * (y - r)).sqrt(), r, epsilon = 1e-9);
        }
    }

    /// Euler's step leaves the exact circle.
    #[test]
    fn euler_steps_leave_the_circle() {
        let (v, w) = (1.0, 0.5);
        let r = v / w;
        let p = ch01_integrate_twist(v, w, 0.5, 40, false);
        let (x, y) = (p[120], p[121]);
        assert!(((x * x + (y - r) * (y - r)).sqrt() - r).abs() > 0.05);
    }

    /// The Euler vertices all lie on one circle (a regular polygon), of radius
    /// `v·dt / (2 sin(ω·dt / 2))` centred at `(v·dt/2, v·dt/2 · cot(ω·dt/2))`: a bounded,
    /// systematic error, not a spiral.
    #[test]
    fn euler_vertices_lie_on_a_larger_off_centre_circle() {
        let (v, w, dt) = (1.0, 0.5, 1.5);
        let phi: f64 = w * dt;
        let radius = v * dt / (2.0 * (phi / 2.0).sin());
        let (cx, cy) = (v * dt / 2.0, v * dt / 2.0 / (phi / 2.0).tan());
        let p = ch01_integrate_twist(v, w, dt, 30, false);
        for k in 0..=30 {
            let d = ((p[3 * k] - cx).powi(2) + (p[3 * k + 1] - cy).powi(2)).sqrt();
            assert_relative_eq!(d, radius, epsilon = 1e-9);
        }
        assert!(radius > v / w);
    }

    #[test]
    fn worked_examples() {
        let q = ch01_se2_exp(1.0, 0.0, std::f64::consts::FRAC_PI_2);
        assert_relative_eq!(q[0], 2.0 / std::f64::consts::PI, epsilon = 1e-12);
        let w = ch01_wheel_arcs(0.72, 0.88);
        assert_relative_eq!(w[0], 0.80, epsilon = 1e-12);
        assert_relative_eq!(w[1], 1.0, epsilon = 1e-12);
        assert_relative_eq!(w[2], 0.673176788, epsilon = 1e-9);
        assert_relative_eq!(w[3], 0.367758155, epsilon = 1e-9);
    }

    #[test]
    fn velocity_samples_are_seeded() {
        let a = ch01_velocity_samples(7, 20, 1.0, 0.5, 1.0, 3, 1.0).unwrap();
        let b = ch01_velocity_samples(7, 20, 1.0, 0.5, 1.0, 3, 1.0).unwrap();
        let c = ch01_velocity_samples(8, 20, 1.0, 0.5, 1.0, 3, 1.0).unwrap();
        assert_eq!(a, b);
        assert_ne!(a, c);
        assert_eq!(a.len(), 20 * 4 * 3);
    }

    #[test]
    fn hallway_beliefs_are_distributions() {
        let out = ch01_hallway(42, 18, 0.5, 0.3).unwrap();
        let block = ch01_hallway_block();
        assert_eq!(out.len(), 19 * block);
        for t in 0..=18 {
            let b = &out[t * block..(t + 1) * block];
            assert_relative_eq!(b[7..107].iter().sum::<f64>(), 1.0, epsilon = 1e-9);
            assert_relative_eq!(b[107..207].iter().sum::<f64>(), 1.0, epsilon = 1e-9);
        }
    }

    #[test]
    fn hallway_never_errors_whatever_the_seed() {
        for seed in 0..200 {
            let out = ch01_hallway(seed, 20, 0.5, 0.5).unwrap();
            assert!(out.iter().all(|v| v.is_finite()));
        }
    }

    #[test]
    fn kf_step_reproduces_the_cart_example() {
        let s1 = ch01_kf_step(0.0, 1.0, 1.0, 1.7, 0.25, 0.5).unwrap();
        assert_relative_eq!(s1[2], 1.5, epsilon = 1e-12);
        assert_relative_eq!(s1[3], 2.5 / 7.0, epsilon = 1e-12);
        let s2 = ch01_kf_step(s1[2], s1[3], 1.0, 2.1, 0.25, 0.5).unwrap();
        assert_relative_eq!(s2[2], 2.280645161290323, epsilon = 1e-12);
        assert_relative_eq!(s2[3], 0.2741935483870968, epsilon = 1e-12);
    }

    #[test]
    fn histogram_four_cells() {
        let out = ch01_histogram_sense_uniform(&[0.2, 0.8, 0.2, 0.8]).unwrap();
        assert_relative_eq!(out[0], 0.5, epsilon = 1e-12);
        for (g, w) in out[3..].iter().zip([0.1, 0.4, 0.1, 0.4]) {
            assert_relative_eq!(*g, w, epsilon = 1e-12);
        }
    }
}
