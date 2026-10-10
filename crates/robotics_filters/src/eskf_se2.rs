//! Error-state Kalman filter for a planar ground robot on SE(2).
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 7 (error-state EKF on `SE2`,
//! `inject_and_reset`); Solà, "Quaternion kinematics for the error-state Kalman filter"
//! (2017) for the nominal/error split; Solà, Deray & Atchuthan (2018) for SE(2) calculus.
//!
//! # State
//!
//! Nominal state `x̂ = (T̂, v̂, b̂)`:
//!
//! | symbol | meaning | unit |
//! |---|---|---|
//! | `T ∈ SE(2)` | pose of `base_link` in `odom` | m, rad |
//! | `v` | forward body speed | m/s |
//! | `b` | gyro yaw-rate bias | rad/s |
//!
//! Error state `δx = (δρ_x, δρ_y, δθ, δv, δb) ∈ ℝ⁵`, covariance `P` (5×5) lives here.
//!
//! # Perturbation convention: **right (body-frame, local)**
//!
//! `T = T̂ · Exp(δξ)`, `δξ = (δρ_x, δρ_y, δθ)`, i.e. `x = x̂ ⊞ δx` with the existing
//! [`Se2`]/[`Manifold`] retraction. Position errors are therefore expressed in the body
//! frame of the estimate. Heading is never integrated as an unconstrained scalar: it lives
//! inside `Se2` (normalized to `(−π, π]`) and its error `δθ` is a small tangent component.
//!
//! # Process model (between measurements, zero-order hold on the gyro)
//!
//! `ω = ω_m − b̂`, `τ = (v̂ Δt, 0, ω Δt)`, `T̂ ← T̂ · Exp(τ)`, `v̂ ← v̂`, `b̂ ← b̂`.
//!
//! Error propagation (first order, `J_r(τ) ≈ I` for the short steps between samples):
//!
//! ```text
//!        ⎡ Ad(Exp(τ))⁻¹   Δt·e_x   −Δt·e_θ ⎤
//!   F =  ⎢      0            1        0    ⎥ ,   Q = diag(σ_lat²Δt, σ_lat²Δt, σ_g²Δt, σ_a²Δt, σ_bw²Δt)
//!        ⎣      0            0        1    ⎦
//! ```
//!
//! with gyro noise density `σ_g` (rad/s/√Hz), longitudinal-acceleration density `σ_a`
//! (m/s²/√Hz, speed random walk), bias random walk `σ_bw` (rad/s/√s) and a small planar
//! slip diffusion `σ_lat` (m/√s) that keeps unmodelled wheel slip from making the filter
//! over-confident.
//!
//! # Reset
//!
//! After injecting `δ̂` the error is re-expressed at the new estimate:
//! `δ⁺ ≈ G(δ − δ̂)` with `G = diag(I₃ − ½ ad(δ̂ξ), 1, 1)` and `P ← G P Gᵀ`, where
//! `ad(v_x, v_y, ω) = [[0, −ω, v_y], [ω, 0, −v_x], [0, 0, 0]]`.

use crate::update::{
    joseph_correction, symmetrize, validate_covariance, GaussianCorrection, Tolerances,
};
use crate::FilterError;
use nalgebra::{Matrix3, SMatrix, SVector, Vector2};
use robotics_core::angle::angle_diff;
use robotics_core::{Manifold, Se2, Tangent2};

/// Error-state dimension.
pub const ESKF_DIM: usize = 5;
/// 5×5 error covariance.
pub type EskfCov = SMatrix<f64, ESKF_DIM, ESKF_DIM>;
/// Error-state vector.
pub type EskfError = SVector<f64, ESKF_DIM>;

/// Index of each error-state component.
pub mod idx {
    /// Body-frame x position error.
    pub const PX: usize = 0;
    /// Body-frame y position error.
    pub const PY: usize = 1;
    /// Heading error.
    pub const TH: usize = 2;
    /// Forward-speed error.
    pub const V: usize = 3;
    /// Gyro-bias error.
    pub const B: usize = 4;
}

/// Nominal planar state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarState {
    /// Pose of the body in the odometry frame.
    pub pose: Se2,
    /// Forward body speed (m/s).
    pub speed: f64,
    /// Gyro yaw-rate bias (rad/s).
    pub gyro_bias: f64,
}

impl Default for PlanarState {
    fn default() -> Self {
        Self {
            pose: Se2::IDENTITY,
            speed: 0.0,
            gyro_bias: 0.0,
        }
    }
}

impl PlanarState {
    /// `true` when every component is finite.
    pub fn is_finite(&self) -> bool {
        self.pose.is_finite() && self.speed.is_finite() && self.gyro_bias.is_finite()
    }
}

impl Manifold<ESKF_DIM> for PlanarState {
    fn boxplus(&self, d: &EskfError) -> Self {
        Self {
            pose: self.pose.boxplus(&Tangent2::new(d[0], d[1], d[2])),
            speed: self.speed + d[3],
            gyro_bias: self.gyro_bias + d[4],
        }
    }
    fn boxminus(&self, base: &Self) -> EskfError {
        let t = self.pose.boxminus(&base.pose);
        EskfError::new(
            t[0],
            t[1],
            t[2],
            self.speed - base.speed,
            self.gyro_bias - base.gyro_bias,
        )
    }
}

/// Continuous-time process-noise densities (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProcessNoise {
    /// Gyro white-noise density `σ_g` (rad/s/√Hz).
    pub gyro_noise_density: f64,
    /// Longitudinal acceleration density `σ_a` driving the speed random walk (m/s²/√Hz).
    pub accel_noise_density: f64,
    /// Gyro-bias random-walk density `σ_bw` (rad/s/√s).
    pub gyro_bias_random_walk: f64,
    /// Planar slip diffusion `σ_lat` (m/√s).
    pub slip_diffusion: f64,
}

impl Default for ProcessNoise {
    fn default() -> Self {
        Self {
            gyro_noise_density: 0.005,
            accel_noise_density: 0.5,
            gyro_bias_random_walk: 1e-4,
            slip_diffusion: 0.01,
        }
    }
}

impl ProcessNoise {
    /// Validate that all densities are finite and non-negative.
    pub fn validated(self) -> Result<Self, FilterError> {
        FilterError::check_non_negative("gyro_noise_density", self.gyro_noise_density)?;
        FilterError::check_non_negative("accel_noise_density", self.accel_noise_density)?;
        FilterError::check_non_negative("gyro_bias_random_walk", self.gyro_bias_random_walk)?;
        FilterError::check_non_negative("slip_diffusion", self.slip_diffusion)?;
        Ok(self)
    }

    /// Discrete process noise for a step of `dt` seconds. `yaw_var_per_s` replaces the
    /// gyro density when the step is driven by a non-gyro yaw rate (wheel-only mode).
    fn discrete(&self, dt: f64, yaw_var_per_s: f64) -> EskfCov {
        let s = self.slip_diffusion * self.slip_diffusion * dt;
        EskfCov::from_diagonal(&EskfError::new(
            s,
            s,
            yaw_var_per_s * dt,
            self.accel_noise_density * self.accel_noise_density * dt,
            self.gyro_bias_random_walk * self.gyro_bias_random_walk * dt,
        ))
    }
}

/// `ad(ξ)` for the SE(2) tangent `ξ = (v_x, v_y, ω)`.
fn se2_ad(xi: &Tangent2) -> Matrix3<f64> {
    Matrix3::new(0.0, -xi[2], xi[1], xi[2], 0.0, -xi[0], 0.0, 0.0, 0.0)
}

/// The SE(2) error-state Kalman filter.
#[derive(Debug, Clone)]
pub struct EskfSe2 {
    x: PlanarState,
    p: EskfCov,
    noise: ProcessNoise,
    tol: Tolerances,
    reset_jacobian: bool,
}

impl EskfSe2 {
    /// Create a filter; the noise and covariance are validated.
    pub fn new(x: PlanarState, p: EskfCov, noise: ProcessNoise) -> Result<Self, FilterError> {
        if !x.is_finite() {
            return Err(FilterError::NonFinite("initial state"));
        }
        let tol = Tolerances::default();
        Ok(Self {
            x,
            p: validate_covariance(&p, &tol)?,
            noise: noise.validated()?,
            tol,
            reset_jacobian: true,
        })
    }

    /// Enable/disable the reset Jacobian `G` (on by default; off gives the common
    /// `P` unchanged approximation, used in tests to show the difference).
    pub fn set_reset_jacobian(&mut self, on: bool) {
        self.reset_jacobian = on;
    }

    /// Nominal state.
    #[inline]
    pub fn state(&self) -> &PlanarState {
        &self.x
    }
    /// Error covariance.
    #[inline]
    pub fn covariance(&self) -> &EskfCov {
        &self.p
    }
    /// Process-noise densities in use.
    #[inline]
    pub fn noise(&self) -> &ProcessNoise {
        &self.noise
    }

    /// Covariance of the pose part `(x, y, θ)` expressed in the **body** frame (right
    /// perturbation).
    pub fn pose_covariance_body(&self) -> Matrix3<f64> {
        self.p.fixed_view::<3, 3>(0, 0).into_owned()
    }

    /// Pose covariance with the position block rotated into the parent (`odom`) frame, as
    /// `nav_msgs/Odometry` expects: `Σ_world = J Σ_body Jᵀ`, `J = diag(R(θ), 1)`.
    pub fn pose_covariance_world(&self) -> Matrix3<f64> {
        let (s, c) = self.x.pose.theta().sin_cos();
        let j = Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
        symmetrize(&(j * self.pose_covariance_body() * j.transpose()))
    }

    /// Overwrite the state and covariance (e.g. after a relocalization).
    pub fn reset(&mut self, x: PlanarState, p: EskfCov) -> Result<(), FilterError> {
        if !x.is_finite() {
            return Err(FilterError::NonFinite("reset state"));
        }
        self.p = validate_covariance(&p, &self.tol)?;
        self.x = x;
        Ok(())
    }

    /// Propagate over `dt` seconds with a gyro reading `yaw_rate_meas` (bias is removed).
    pub fn predict_gyro(&mut self, yaw_rate_meas: f64, dt: f64) -> Result<(), FilterError> {
        let g = self.noise.gyro_noise_density;
        self.predict_inner(yaw_rate_meas, true, g * g, dt)
    }

    /// Propagate with an externally supplied yaw rate that is *not* affected by the gyro
    /// bias (wheel-only mode, used when the IMU is unavailable). `yaw_rate_var_per_s` is
    /// the variance density of that rate (rad²/s).
    pub fn predict_yaw_rate(
        &mut self,
        yaw_rate: f64,
        yaw_rate_var_per_s: f64,
        dt: f64,
    ) -> Result<(), FilterError> {
        FilterError::check_non_negative("yaw_rate_var_per_s", yaw_rate_var_per_s)?;
        self.predict_inner(yaw_rate, false, yaw_rate_var_per_s, dt)
    }

    fn predict_inner(
        &mut self,
        rate: f64,
        biased: bool,
        yaw_var_per_s: f64,
        dt: f64,
    ) -> Result<(), FilterError> {
        if !rate.is_finite() {
            return Err(FilterError::NonFinite("yaw rate"));
        }
        if !dt.is_finite() || dt < 0.0 {
            return Err(FilterError::InvalidTimeDelta { dt });
        }
        if dt == 0.0 {
            return Ok(());
        }
        let omega = if biased {
            rate - self.x.gyro_bias
        } else {
            rate
        };
        let tau = Tangent2::new(self.x.speed * dt, 0.0, omega * dt);
        let mut f = EskfCov::identity();
        f.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&Se2::exp(&tau).inverse().adjoint());
        f[(idx::PX, idx::V)] = dt;
        if biased {
            f[(idx::TH, idx::B)] = -dt;
        }
        let p = symmetrize(&(f * self.p * f.transpose() + self.noise.discrete(dt, yaw_var_per_s)));
        let p = validate_covariance(&p, &self.tol)?;
        let pose = self.x.pose.boxplus(&tau);
        if !pose.is_finite() {
            return Err(FilterError::NonFinite("propagated pose"));
        }
        self.x.pose = pose;
        self.p = p;
        Ok(())
    }

    /// Inflate the covariance by `dt` seconds of process noise without moving the state
    /// (used when a time gap is too large to integrate).
    pub fn inflate(&mut self, dt: f64) -> Result<(), FilterError> {
        if !dt.is_finite() || dt < 0.0 {
            return Err(FilterError::InvalidTimeDelta { dt });
        }
        let g = self.noise.gyro_noise_density;
        let p = symmetrize(&(self.p + self.noise.discrete(dt, g * g)));
        self.p = validate_covariance(&p, &self.tol)?;
        Ok(())
    }

    /// Generic linear(ized) error-state update, followed by injection and reset.
    pub fn update<const M: usize>(
        &mut self,
        innovation: &SVector<f64, M>,
        h: &SMatrix<f64, M, ESKF_DIM>,
        r: &SMatrix<f64, M, M>,
    ) -> Result<GaussianCorrection<ESKF_DIM, M>, FilterError> {
        let c = joseph_correction(&self.p, h, r, innovation, &self.tol)?;
        let x_new = self.x.boxplus(&c.delta);
        if !x_new.is_finite() {
            return Err(FilterError::NonFinite("injected state"));
        }
        let mut p = c.covariance;
        if self.reset_jacobian {
            let mut g = EskfCov::identity();
            let dxi = Tangent2::new(c.delta[0], c.delta[1], c.delta[2]);
            g.fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&(Matrix3::identity() - se2_ad(&dxi) * 0.5));
            p = validate_covariance(&symmetrize(&(g * p * g.transpose())), &self.tol)?;
        }
        self.x = x_new;
        self.p = p;
        Ok(c)
    }

    /// Wheel-odometry update when a gyro reading is available.
    ///
    /// Measurement `z = (v_wheel, ω_wheel − ω_gyro)`, model `h(x) = (v, −b)`:
    /// the wheel speed observes `v` directly, and the difference between the wheel and gyro
    /// yaw rates observes the gyro bias (`ω_wheel ≈ ω_true`, `ω_gyro ≈ ω_true + b`).
    /// `R = diag(σ_v², σ_ωw² + σ_ωg²)`.
    pub fn update_wheel_with_gyro(
        &mut self,
        wheel_speed: f64,
        wheel_yaw_rate: f64,
        gyro_yaw_rate: f64,
        var_speed: f64,
        var_wheel_yaw_rate: f64,
        var_gyro_yaw_rate: f64,
    ) -> Result<GaussianCorrection<ESKF_DIM, 2>, FilterError> {
        for v in [wheel_speed, wheel_yaw_rate, gyro_yaw_rate] {
            if !v.is_finite() {
                return Err(FilterError::NonFinite("wheel/gyro measurement"));
            }
        }
        let z = Vector2::new(wheel_speed, wheel_yaw_rate - gyro_yaw_rate);
        let z_hat = Vector2::new(self.x.speed, -self.x.gyro_bias);
        let mut h = SMatrix::<f64, 2, ESKF_DIM>::zeros();
        h[(0, idx::V)] = 1.0;
        h[(1, idx::B)] = -1.0;
        let r = nalgebra::Matrix2::new(var_speed, 0.0, 0.0, var_wheel_yaw_rate + var_gyro_yaw_rate);
        self.update(&(z - z_hat), &h, &r)
    }

    /// Wheel speed update alone (IMU unavailable).
    pub fn update_speed(
        &mut self,
        wheel_speed: f64,
        var_speed: f64,
    ) -> Result<GaussianCorrection<ESKF_DIM, 1>, FilterError> {
        if !wheel_speed.is_finite() {
            return Err(FilterError::NonFinite("wheel speed"));
        }
        let mut h = SMatrix::<f64, 1, ESKF_DIM>::zeros();
        h[(0, idx::V)] = 1.0;
        self.update(
            &SVector::<f64, 1>::new(wheel_speed - self.x.speed),
            &h,
            &nalgebra::Matrix1::new(var_speed),
        )
    }

    /// Absolute heading update (compass, localization). The residual is wrapped, so
    /// measurements across the ±π branch cut are handled correctly.
    pub fn update_heading(
        &mut self,
        yaw: f64,
        var_yaw: f64,
    ) -> Result<GaussianCorrection<ESKF_DIM, 1>, FilterError> {
        if !yaw.is_finite() {
            return Err(FilterError::NonFinite("heading"));
        }
        let mut h = SMatrix::<f64, 1, ESKF_DIM>::zeros();
        h[(0, idx::TH)] = 1.0;
        self.update(
            &SVector::<f64, 1>::new(angle_diff(yaw, self.x.pose.theta())),
            &h,
            &nalgebra::Matrix1::new(var_yaw),
        )
    }

    /// Absolute position update in the parent frame (e.g. from localization).
    /// `p = p̂ + R(θ̂) δρ` ⇒ `H = [R(θ̂) 0 0 0]`.
    pub fn update_position(
        &mut self,
        xy: &Vector2<f64>,
        cov: &nalgebra::Matrix2<f64>,
    ) -> Result<GaussianCorrection<ESKF_DIM, 2>, FilterError> {
        if xy.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("position"));
        }
        let (s, c) = self.x.pose.theta().sin_cos();
        let mut h = SMatrix::<f64, 2, ESKF_DIM>::zeros();
        h[(0, 0)] = c;
        h[(0, 1)] = -s;
        h[(1, 0)] = s;
        h[(1, 1)] = c;
        self.update(&(xy - self.x.pose.translation()), &h, cov)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rand_distr::{Distribution, Normal};
    use robotics_core::seeded_rng;
    use std::f64::consts::PI;

    fn filter(x: PlanarState) -> EskfSe2 {
        EskfSe2::new(
            x,
            EskfCov::from_diagonal(&EskfError::new(0.01, 0.01, 0.01, 0.1, 1e-4)),
            ProcessNoise::default(),
        )
        .unwrap()
    }

    #[test]
    fn planar_state_boxplus_boxminus_round_trip() {
        let a = PlanarState {
            pose: Se2::new(1.0, -2.0, 3.0),
            speed: 0.4,
            gyro_bias: 0.01,
        };
        let d = EskfError::new(0.1, -0.2, 0.5, 0.05, -0.002);
        let b = a.boxplus(&d);
        assert!((b.boxminus(&a) - d).norm() < 1e-12);
        // Heading wrapped: 3.0 + 0.5 lands on the other side of the branch cut.
        assert!(b.pose.theta() < 0.0 && b.pose.theta() > -PI);
        assert!(a.boxplus(&EskfError::zeros()).boxminus(&a).norm() < 1e-12);
    }

    #[test]
    fn predict_matches_exact_arc_and_grows_covariance() {
        let mut f = filter(PlanarState {
            pose: Se2::IDENTITY,
            speed: 1.0,
            gyro_bias: 0.1,
        });
        let p0 = f.covariance().trace();
        f.predict_gyro(0.6, 1.0).unwrap(); // ω = 0.6 − 0.1 = 0.5
        let expect = Se2::exp(&Tangent2::new(1.0, 0.0, 0.5));
        assert!(f.state().pose.boxminus(&expect).norm() < 1e-12);
        assert_relative_eq!(f.state().pose.x(), 0.958851077208, epsilon = 1e-12);
        assert!(f.covariance().trace() > p0);
        let p = f.covariance();
        assert_eq!((p - p.transpose()).abs().max(), 0.0);
        // dt = 0 is a no-op, negative dt is an error.
        let before = *f.state();
        f.predict_gyro(0.6, 0.0).unwrap();
        assert_eq!(*f.state(), before);
        assert_eq!(
            f.predict_gyro(0.6, -0.1).unwrap_err(),
            FilterError::InvalidTimeDelta { dt: -0.1 }
        );
        assert!(f.predict_gyro(f64::NAN, 0.1).is_err());
    }

    #[test]
    fn heading_update_across_the_branch_cut() {
        let mut f = filter(PlanarState {
            pose: Se2::new(0.0, 0.0, PI - 0.01),
            ..Default::default()
        });
        // Measurement on the other side of ±π: true difference is +0.02 rad, not −2π+0.02.
        let c = f.update_heading(-PI + 0.01, 1e-4).unwrap();
        assert_relative_eq!(c.innovation[0], 0.02, epsilon = 1e-12);
        let th = f.state().pose.theta();
        assert!(!(-PI + 0.02..=PI - 0.01).contains(&th), "theta {th}");
        assert!(angle_diff(th, -PI + 0.01).abs() < 0.002);
    }

    #[test]
    fn small_error_injection_and_reset() {
        let mut f = filter(PlanarState::default());
        f.set_reset_jacobian(false);
        let mut g = filter(PlanarState::default());
        // A zero innovation injects nothing and the reset Jacobian is the identity.
        let c = g.update_heading(0.0, 1e-2).unwrap();
        assert_eq!(c.delta, EskfError::zeros());
        f.update_heading(0.0, 1e-2).unwrap();
        assert!((f.covariance() - g.covariance()).norm() < 1e-15);
        // A heading correction with correlated position error: reset changes P slightly.
        let mut a = filter(PlanarState::default());
        let mut b = filter(PlanarState::default());
        b.set_reset_jacobian(false);
        for e in [&mut a, &mut b] {
            e.predict_gyro(0.3, 1.0).unwrap();
            e.x.speed = 1.0;
            e.predict_gyro(0.3, 1.0).unwrap();
            e.update_heading(0.9, 1e-3).unwrap();
        }
        assert!((a.state().pose.boxminus(&b.state().pose)).norm() < 1e-15);
        // The reset is a first-order correction: it must change P, but only by a fraction
        // of its size (|δθ| ≈ 0.3 rad here, so G deviates from I by ≈ 0.15).
        let rel = (a.covariance() - b.covariance()).norm() / b.covariance().norm();
        assert!(rel > 0.0 && rel < 0.3, "relative reset effect {rel}");
        assert!(validate_covariance(a.covariance(), &Tolerances::default()).is_ok());
    }

    #[test]
    fn wheel_update_estimates_speed_and_bias() {
        let mut f = filter(PlanarState::default());
        let true_bias = 0.02;
        let mut rng = seeded_rng(11);
        let n_g = Normal::new(0.0, 0.002).unwrap();
        let n_w = Normal::new(0.0, 0.01).unwrap();
        for _ in 0..2000 {
            let omega = 0.3;
            let gyro = omega + true_bias + n_g.sample(&mut rng);
            f.predict_gyro(gyro, 0.02).unwrap();
            f.update_wheel_with_gyro(
                0.5 + n_w.sample(&mut rng),
                omega + n_w.sample(&mut rng),
                gyro,
                1e-4,
                1e-4,
                4e-6,
            )
            .unwrap();
        }
        assert!(
            (f.state().gyro_bias - true_bias).abs() < 2e-3,
            "bias {}",
            f.state().gyro_bias
        );
        assert!((f.state().speed - 0.5).abs() < 0.01);
        let sigma_b = f.covariance()[(idx::B, idx::B)].sqrt();
        assert!(sigma_b < 5e-3);
    }

    #[test]
    fn converges_from_perturbed_initial_state() {
        // Truth drives a circle; the filter starts 0.5 rad and 1 m off and receives
        // position + heading fixes (as localization will provide in Phase 3).
        let mut rng = seeded_rng(3);
        let noise = Normal::new(0.0, 0.05).unwrap();
        let mut truth = Se2::IDENTITY;
        let mut f = EskfSe2::new(
            PlanarState {
                pose: Se2::new(1.0, -0.5, 0.5),
                speed: 0.0,
                gyro_bias: 0.0,
            },
            EskfCov::from_diagonal(&EskfError::new(2.0, 2.0, 1.0, 1.0, 1e-3)),
            ProcessNoise::default(),
        )
        .unwrap();
        let (v, w, dt) = (0.5, 0.2, 0.05);
        for k in 0..600 {
            truth = truth.boxplus(&Tangent2::new(v * dt, 0.0, w * dt));
            f.predict_gyro(w, dt).unwrap();
            f.update_wheel_with_gyro(v, w, w, 1e-4, 1e-4, 1e-6).unwrap();
            if k % 10 == 0 {
                let z = truth.translation()
                    + Vector2::new(noise.sample(&mut rng), noise.sample(&mut rng));
                f.update_position(&z, &(nalgebra::Matrix2::identity() * 0.0025))
                    .unwrap();
                f.update_heading(truth.theta() + 0.2 * noise.sample(&mut rng), 1e-4)
                    .unwrap();
            }
        }
        let err = f.state().pose.boxminus(&truth);
        assert!(err.fixed_rows::<2>(0).norm() < 0.05, "pos err {err}");
        assert!(err[2].abs() < 0.02, "yaw err {}", err[2]);
    }

    #[test]
    fn repeated_turns_crossing_pi() {
        let mut f = filter(PlanarState {
            pose: Se2::IDENTITY,
            speed: 0.3,
            gyro_bias: 0.0,
        });
        let mut truth = Se2::IDENTITY;
        let (w, dt) = (1.0, 0.01);
        let mut crossings = 0;
        let mut prev = 0.0f64;
        for k in 0..6000 {
            truth = truth.boxplus(&Tangent2::new(0.3 * dt, 0.0, w * dt));
            f.predict_gyro(w, dt).unwrap();
            f.update_wheel_with_gyro(0.3, w, w, 1e-4, 1e-4, 1e-6)
                .unwrap();
            if k % 50 == 0 {
                f.update_heading(truth.theta(), 1e-4).unwrap();
            }
            let th = f.state().pose.theta();
            assert!(th > -PI && th <= PI);
            if prev > 2.5 && th < -2.5 {
                crossings += 1;
            }
            prev = th;
            assert!(angle_diff(th, truth.theta()).abs() < 1e-3);
        }
        assert!(crossings >= 9, "crossed ±π {crossings} times");
    }

    #[test]
    fn world_covariance_rotates_position_block() {
        let mut f = filter(PlanarState {
            pose: Se2::new(0.0, 0.0, std::f64::consts::FRAC_PI_2),
            ..Default::default()
        });
        let mut p = EskfCov::zeros();
        p[(0, 0)] = 4.0; // body-x variance, body x = world y at θ = 90°
        p[(1, 1)] = 1.0;
        p[(2, 2)] = 0.1;
        f.reset(*f.state(), p).unwrap();
        let w = f.pose_covariance_world();
        assert_relative_eq!(w[(0, 0)], 1.0, epsilon = 1e-12);
        assert_relative_eq!(w[(1, 1)], 4.0, epsilon = 1e-12);
    }

    #[test]
    fn rejects_invalid_noise() {
        let bad = ProcessNoise {
            gyro_noise_density: -1.0,
            ..Default::default()
        };
        assert!(matches!(
            EskfSe2::new(PlanarState::default(), EskfCov::identity(), bad),
            Err(FilterError::InvalidNoiseParameter { .. })
        ));
    }
}
