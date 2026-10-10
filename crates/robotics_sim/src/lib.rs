//! `robotics_sim` — the deterministic *Rusty* simulator, independent of ROS.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 4 — Rusty, Sensors, and the
//! Simulator (`Robot::step`, `EncoderTicks`, split RNG streams, fixed timestep,
//! bit-exact replay, `raycast_scan`). This is an independent implementation of that
//! design: wheel encoders, a gyro and a ray-cast 2-D LiDAR in a static segment
//! [`World`] (see `docs/SIMULATION.md`).
//!
//! # Physics (one fixed step `dt`)
//!
//! 1. The scripted twist `(v, ω)` is turned into wheel rates by inverse kinematics; the
//!    wheels (and therefore the encoders) turn exactly as commanded.
//! 2. Ground travel differs from wheel rotation: per-wheel multiplicative slip
//!    `Δs_i = r_i ω_i dt (1 + ε_i)`, `ε_i ~ N(0, σ_slip²)`, and a systematic right-wheel
//!    radius error `r_R = r (1 + δ_R)`.
//! 3. The pose advances by the exact arc `x ← x ⊞ ((Δs_R+Δs_L)/2, 0, (Δs_R−Δs_L)/ℓ)`.
//! 4. The gyro bias performs a random walk; each IMU sample reports the *average* true yaw
//!    rate over its interval plus bias plus white noise.
//!
//! 5. At the LiDAR rate, every beam is ray cast from the sensor pose `x ∘ T_base_laser`
//!    against the world and corrupted by Gaussian range noise and dropouts.
//!
//! Encoders are quantized to `ticks_per_rev`. Every noise source has its own RNG stream
//! derived from the seed, so enabling one source never changes another's draws.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod lidar;
pub mod rusty;
pub mod trajectory;
pub mod world;

pub use lidar::{Lidar, LidarConfig, NoReturnPolicy};
pub use rusty::{GroundTruth, RustyConfig, RustySim, SimStep, WheelSample};
pub use trajectory::{Segment, Trajectory};
pub use world::{Segment2, World};

/// Simulator configuration errors.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SimError {
    /// A parameter is out of range.
    #[error("invalid simulator parameter {what} = {value}")]
    InvalidParameter {
        /// Parameter name.
        what: &'static str,
        /// Offending value.
        value: f64,
    },
    /// A sensor rate does not divide the physics rate.
    #[error(
        "sensor rate {rate_hz} Hz is not an integer divisor of the physics rate {physics_hz} Hz"
    )]
    RateMismatch {
        /// Requested sensor rate.
        rate_hz: f64,
        /// Physics rate.
        physics_hz: f64,
    },
    /// A trajectory needs at least one segment of positive duration.
    #[error("empty trajectory")]
    EmptyTrajectory,
    /// Invalid kinematic model parameters.
    #[error(transparent)]
    Model(#[from] robotics_models::ModelError),
}
