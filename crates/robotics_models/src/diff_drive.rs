//! Differential-drive kinematics and wheel-encoder odometry.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 4 — Rusty, Sensors, and the
//! Simulator (`RobotParams`, `EncoderTicks`, `odometry_delta`).
//!
//! # Model
//! With wheel radius `r`, wheel separation `ℓ` and wheel angular rates `ω_L, ω_R`:
//!
//! * inverse kinematics: `ω_L = (v − ω ℓ/2)/r`, `ω_R = (v + ω ℓ/2)/r`
//! * forward kinematics: `v = r(ω_R + ω_L)/2`, `ω = r(ω_R − ω_L)/ℓ`
//!
//! Per update, from wheel arc lengths `Δs_L, Δs_R`:
//! `Δs = (Δs_R + Δs_L)/2`, `Δθ = (Δs_R − Δs_L)/ℓ`, and the pose advances by the **exact
//! constant-curvature arc** `x ← x ⊞ (Δs, 0, Δθ)`. No small-angle approximation is made,
//! so with noiseless encoders dead reckoning reproduces the true motion (the chapter's
//! `odometry_zero_noise_is_exact` invariant).

use crate::ModelError;
use robotics_core::{Cov3, Manifold, Odometry2, Se2, Tangent2, Timestamp, Twist2};
use std::f64::consts::TAU;

/// Physical parameters of a differential-drive base.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiffDriveParams {
    /// Wheel radius (m).
    pub wheel_radius: f64,
    /// Distance between the wheel contact points (m).
    pub wheel_separation: f64,
}

impl DiffDriveParams {
    /// The book's rover *Rusty*: `r = 0.033 m`, `ℓ = 0.16 m`.
    pub const RUSTY: DiffDriveParams = DiffDriveParams {
        wheel_radius: 0.033,
        wheel_separation: 0.16,
    };

    /// Validate the parameters (both must be positive and finite).
    pub fn validated(self) -> Result<Self, ModelError> {
        if !(self.wheel_radius > 0.0 && self.wheel_radius.is_finite()) {
            return Err(ModelError::NonPositive {
                what: "wheel_radius",
                value: self.wheel_radius,
            });
        }
        if !(self.wheel_separation > 0.0 && self.wheel_separation.is_finite()) {
            return Err(ModelError::NonPositive {
                what: "wheel_separation",
                value: self.wheel_separation,
            });
        }
        Ok(self)
    }

    /// Wheel angular rates that realize a body twist (lateral velocity is ignored).
    #[inline]
    pub fn inverse_kinematics(&self, twist: Twist2) -> WheelSpeeds {
        let half = 0.5 * self.wheel_separation * twist.omega;
        WheelSpeeds {
            left: (twist.vx - half) / self.wheel_radius,
            right: (twist.vx + half) / self.wheel_radius,
        }
    }

    /// Body twist produced by wheel angular rates.
    #[inline]
    pub fn forward_kinematics(&self, wheels: WheelSpeeds) -> Twist2 {
        let r = self.wheel_radius;
        Twist2::unicycle(
            r * (wheels.right + wheels.left) * 0.5,
            r * (wheels.right - wheels.left) / self.wheel_separation,
        )
    }

    /// Tangent increment `(Δs, 0, Δθ)` from per-wheel arc lengths (m).
    #[inline]
    pub fn wheel_arcs_to_tangent(&self, ds_left: f64, ds_right: f64) -> Tangent2 {
        Tangent2::new(
            0.5 * (ds_right + ds_left),
            0.0,
            (ds_right - ds_left) / self.wheel_separation,
        )
    }

    /// Distance travelled by a wheel per encoder tick for `ticks_per_rev` ticks/revolution.
    #[inline]
    pub fn tick_length(&self, ticks_per_rev: u32) -> f64 {
        TAU * self.wheel_radius / ticks_per_rev as f64
    }
}

/// Wheel angular rates (rad/s).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelSpeeds {
    /// Left wheel (rad/s).
    pub left: f64,
    /// Right wheel (rad/s).
    pub right: f64,
}

/// Cumulative encoder counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EncoderTicks {
    /// Left wheel count.
    pub left: i64,
    /// Right wheel count.
    pub right: i64,
}

/// A simple stochastic model of wheel-odometry error, used to grow the published
/// covariance. Standard deviations scale with the motion of the step:
///
/// `σ_s = trans_per_m·|Δs| + trans_floor`, `σ_θ = rot_per_rad·|Δθ| + rot_per_m·|Δs| + rot_floor`,
/// plus a lateral term `σ_y = lateral_per_m·|Δs|` so the covariance stays non-degenerate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryNoise {
    /// Longitudinal error per metre travelled (dimensionless).
    pub trans_per_m: f64,
    /// Lateral error per metre travelled (dimensionless).
    pub lateral_per_m: f64,
    /// Heading error per radian turned (dimensionless).
    pub rot_per_rad: f64,
    /// Heading error per metre travelled (rad/m).
    pub rot_per_m: f64,
    /// Floors (m, rad) added to every step so a stationary robot still has a PD covariance.
    pub trans_floor: f64,
    /// Heading floor (rad).
    pub rot_floor: f64,
}

impl Default for OdometryNoise {
    fn default() -> Self {
        Self {
            trans_per_m: 0.02,
            lateral_per_m: 0.01,
            rot_per_rad: 0.02,
            rot_per_m: 0.01,
            trans_floor: 1e-4,
            rot_floor: 1e-4,
        }
    }
}

impl OdometryNoise {
    /// Validate (all entries non-negative and finite).
    pub fn validated(self) -> Result<Self, ModelError> {
        for (what, value) in [
            ("trans_per_m", self.trans_per_m),
            ("lateral_per_m", self.lateral_per_m),
            ("rot_per_rad", self.rot_per_rad),
            ("rot_per_m", self.rot_per_m),
            ("trans_floor", self.trans_floor),
            ("rot_floor", self.rot_floor),
        ] {
            if !(value >= 0.0 && value.is_finite()) {
                return Err(ModelError::Negative { what, value });
            }
        }
        Ok(self)
    }

    /// Step noise covariance `R` in the body tangent frame for a step `(Δs, 0, Δθ)`.
    pub fn step_covariance(&self, delta: &Tangent2) -> Cov3 {
        let ds = delta[0].abs();
        let dth = delta[2].abs();
        let sig_s = self.trans_per_m * ds + self.trans_floor;
        let sig_y = self.lateral_per_m * ds + self.trans_floor;
        let sig_th = self.rot_per_rad * dth + self.rot_per_m * ds + self.rot_floor;
        Cov3::from_diagonal(&nalgebra::Vector3::new(
            sig_s * sig_s,
            sig_y * sig_y,
            sig_th * sig_th,
        ))
    }
}

/// Result of one odometry update.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryStep {
    /// Body-frame increment `(Δs, 0, Δθ)` applied in this step.
    pub delta: Tangent2,
    /// Elapsed time (s) since the previous update.
    pub dt: f64,
    /// Body twist `(Δs/dt, 0, Δθ/dt)`, or zero if `dt` is not positive.
    pub twist: Twist2,
}

/// Dead-reckoning integrator: turns cumulative wheel positions into an [`Odometry2`].
///
/// Feed it wheel *positions* (rad) or encoder *ticks*; it differences them itself, so
/// the first sample only initializes the reference and produces no step. The
/// covariance is compounded in the body frame:
/// `Σ ← Ad(Δ⁻¹) Σ Ad(Δ⁻¹)ᵀ + R(Δ)` (Chapter 9, tangent-space noise compounding).
#[derive(Debug, Clone)]
pub struct WheelOdometry {
    params: DiffDriveParams,
    noise: OdometryNoise,
    pose: Se2,
    cov: Cov3,
    last: Option<(f64, f64, Timestamp)>,
    last_twist: Twist2,
}

impl WheelOdometry {
    /// Create an integrator at the origin with zero covariance.
    pub fn new(params: DiffDriveParams, noise: OdometryNoise) -> Result<Self, ModelError> {
        Ok(Self {
            params: params.validated()?,
            noise: noise.validated()?,
            pose: Se2::IDENTITY,
            cov: Cov3::zeros(),
            last: None,
            last_twist: Twist2::ZERO,
        })
    }

    /// Current integrated pose.
    #[inline]
    pub fn pose(&self) -> Se2 {
        self.pose
    }
    /// Current pose covariance.
    #[inline]
    pub fn covariance(&self) -> Cov3 {
        self.cov
    }
    /// Twist from the most recent step.
    #[inline]
    pub fn twist(&self) -> Twist2 {
        self.last_twist
    }
    /// The kinematic parameters in use.
    #[inline]
    pub fn params(&self) -> &DiffDriveParams {
        &self.params
    }

    /// Reset the pose (and covariance) — e.g. after a localization correction.
    pub fn reset(&mut self, pose: Se2, cov: Cov3) {
        self.pose = pose;
        self.cov = cov;
        self.last = None;
        self.last_twist = Twist2::ZERO;
    }

    /// Update from cumulative wheel angles (rad). Returns `None` on the first call.
    pub fn update_from_wheel_angles(
        &mut self,
        left: f64,
        right: f64,
        stamp: Timestamp,
    ) -> Option<OdometryStep> {
        if !(left.is_finite() && right.is_finite()) {
            return None;
        }
        let Some((l0, r0, t0)) = self.last else {
            self.last = Some((left, right, stamp));
            return None;
        };
        let r = self.params.wheel_radius;
        let delta = self
            .params
            .wheel_arcs_to_tangent((left - l0) * r, (right - r0) * r);
        let dt = stamp - t0;
        self.last = Some((left, right, stamp));
        Some(self.apply(delta, dt))
    }

    /// Update from cumulative encoder ticks. Returns `None` on the first call.
    pub fn update_from_ticks(
        &mut self,
        ticks: EncoderTicks,
        ticks_per_rev: u32,
        stamp: Timestamp,
    ) -> Option<OdometryStep> {
        let per_tick = TAU / ticks_per_rev.max(1) as f64;
        self.update_from_wheel_angles(
            ticks.left as f64 * per_tick,
            ticks.right as f64 * per_tick,
            stamp,
        )
    }

    fn apply(&mut self, delta: Tangent2, dt: f64) -> OdometryStep {
        let step_pose = Se2::exp(&delta);
        // Compound the covariance: the old uncertainty is carried through the new step
        // (Ad of the inverse increment) and the fresh step noise is added.
        let ad_inv = step_pose.inverse().adjoint();
        self.cov = ad_inv * self.cov * ad_inv.transpose() + self.noise.step_covariance(&delta);
        self.pose = self.pose.boxplus(&delta);
        let twist = if dt > 0.0 && dt.is_finite() {
            Twist2::unicycle(delta[0] / dt, delta[2] / dt)
        } else {
            Twist2::ZERO
        };
        self.last_twist = twist;
        OdometryStep { delta, dt, twist }
    }

    /// Snapshot as an [`Odometry2`] message-ready struct.
    pub fn odometry(
        &self,
        stamp: Timestamp,
        frame: impl Into<robotics_core::FrameId>,
        child_frame: impl Into<robotics_core::FrameId>,
    ) -> Odometry2 {
        let twist_sigma = 0.05;
        Odometry2 {
            stamp,
            frame: frame.into(),
            child_frame: child_frame.into(),
            pose: self.pose,
            pose_cov: self.cov,
            twist: self.last_twist,
            twist_cov: Cov3::identity() * (twist_sigma * twist_sigma),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rand::RngExt;
    use robotics_core::seeded_rng;

    /// Chapter 4 worked example: ℓ = 0.16, Δs_L = 0.72, Δs_R = 0.88 → Δs = 0.80, Δθ = 1.0,
    /// pose = exp((0.8, 0, 1)) = (0.673176788, 0.367758155, 1.0).
    #[test]
    fn worked_example_wheel_arcs() {
        let p = DiffDriveParams::RUSTY;
        let tau = p.wheel_arcs_to_tangent(0.72, 0.88);
        assert_relative_eq!(tau[0], 0.80, epsilon = 1e-12);
        assert_relative_eq!(tau[1], 0.0, epsilon = 1e-12);
        assert_relative_eq!(tau[2], 1.0, epsilon = 1e-12);
        let pose = Se2::exp(&tau);
        assert_relative_eq!(pose.x(), 0.673176788, epsilon = 1e-9);
        assert_relative_eq!(pose.y(), 0.367758155, epsilon = 1e-9);
        assert_relative_eq!(pose.theta(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn kinematics_round_trip() {
        let p = DiffDriveParams::RUSTY;
        let t = Twist2::unicycle(0.6, -0.8);
        let w = p.inverse_kinematics(t);
        let back = p.forward_kinematics(w);
        assert_relative_eq!(back.vx, t.vx, epsilon = 1e-12);
        assert_relative_eq!(back.omega, t.omega, epsilon = 1e-12);
        assert_relative_eq!(p.tick_length(4096), TAU * 0.033 / 4096.0, epsilon = 1e-15);
    }

    /// Noiseless encoders ⇒ dead reckoning is exact, even over a long wiggly path.
    #[test]
    fn zero_noise_dead_reckoning_is_exact() {
        let p = DiffDriveParams::RUSTY;
        let mut odom = WheelOdometry::new(p, OdometryNoise::default()).unwrap();
        let mut truth = Se2::new(1.0, -2.0, 0.3);
        let mut angles = (0.0f64, 0.0f64);
        let dt = 0.02;
        let mut rng = seeded_rng(7);
        let start = Timestamp::from_secs_f64(100.0);
        odom.reset(truth, Cov3::zeros());
        odom.update_from_wheel_angles(angles.0, angles.1, start);
        for k in 1..=3000 {
            let cmd = Twist2::unicycle(
                0.4 + 0.2 * (k as f64 * 0.01).sin(),
                rng.random_range(-1.5..1.5),
            );
            let w = p.inverse_kinematics(cmd);
            angles.0 += w.left * dt;
            angles.1 += w.right * dt;
            truth = truth.boxplus(&cmd.tangent(dt));
            odom.update_from_wheel_angles(angles.0, angles.1, start.plus_secs_f64(k as f64 * dt));
        }
        assert!(odom.pose().boxminus(&truth).norm() < 1e-7);
        assert!(odom.covariance().iter().all(|v| v.is_finite()));
        assert!(robotics_core::types::is_valid_covariance(
            &odom.covariance(),
            1e-12
        ));
    }

    #[test]
    fn ticks_path_matches_angles_path() {
        let p = DiffDriveParams::RUSTY;
        let n = 4096;
        let mut a = WheelOdometry::new(p, OdometryNoise::default()).unwrap();
        let mut b = WheelOdometry::new(p, OdometryNoise::default()).unwrap();
        let t0 = Timestamp::from_secs_f64(0.0);
        let t1 = Timestamp::from_secs_f64(0.1);
        assert!(a
            .update_from_ticks(EncoderTicks { left: 0, right: 0 }, n, t0)
            .is_none());
        assert!(b.update_from_wheel_angles(0.0, 0.0, t0).is_none());
        let sa = a
            .update_from_ticks(
                EncoderTicks {
                    left: 100,
                    right: 140,
                },
                n,
                t1,
            )
            .unwrap();
        let sb = b
            .update_from_wheel_angles(100.0 * TAU / n as f64, 140.0 * TAU / n as f64, t1)
            .unwrap();
        assert_relative_eq!(sa.delta[0], sb.delta[0], epsilon = 1e-12);
        assert_relative_eq!(sa.delta[2], sb.delta[2], epsilon = 1e-12);
        assert_relative_eq!(sa.twist.vx, sa.delta[0] / 0.1, epsilon = 1e-12);
        assert_eq!(sa.dt, 0.1);
    }

    #[test]
    fn covariance_grows_with_motion() {
        let mut odom =
            WheelOdometry::new(DiffDriveParams::RUSTY, OdometryNoise::default()).unwrap();
        odom.update_from_wheel_angles(0.0, 0.0, Timestamp::ZERO);
        let mut prev_trace = 0.0;
        for k in 1..=50 {
            let a = k as f64 * 2.0;
            odom.update_from_wheel_angles(a, a, Timestamp::from_secs_f64(k as f64 * 0.1));
            let tr = odom.covariance().trace();
            assert!(tr > prev_trace, "trace must grow: {tr} <= {prev_trace}");
            prev_trace = tr;
        }
    }

    #[test]
    fn rejects_bad_parameters() {
        let bad = DiffDriveParams {
            wheel_radius: 0.0,
            wheel_separation: 0.1,
        };
        assert!(matches!(
            bad.validated(),
            Err(ModelError::NonPositive {
                what: "wheel_radius",
                ..
            })
        ));
        let noise = OdometryNoise {
            trans_per_m: -1.0,
            ..Default::default()
        };
        assert!(WheelOdometry::new(DiffDriveParams::RUSTY, noise).is_err());
    }
}
