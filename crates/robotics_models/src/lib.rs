//! `robotics_models` — robot and motion models, independent of ROS.
//!
//! * [`diff_drive`]: differential-drive kinematics and wheel-encoder odometry
//!   (*Probabilistic Robotics via Rust*, Chapter 4 — Rusty, Sensors, and the Simulator).
//! * [`motion`]: Thrun's probabilistic velocity and odometry motion models, sampler and
//!   closed-form density (Chapter 9 — Probabilistic Motion Models).
//! * [`sensor`]: gyro noise model, beam mixture model with EM-learned intrinsics,
//!   likelihood field over an exact distance transform, range–bearing landmark model and
//!   likelihood tempering (Chapter 10 — Probabilistic Sensor Models).
//!
//! Everything here is pure Rust over [`robotics_core`]; ROS nodes convert messages to the
//! core types and call in.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod diff_drive;
pub mod motion;
pub mod sensor;

pub use diff_drive::{
    DiffDriveParams, EncoderTicks, OdometryNoise, OdometryStep, WheelOdometry, WheelSpeeds,
};
pub use motion::{
    MotionModel, OdomAlphas, OdomDelta, OdometryMotionModel, VelocityAlphas, VelocityCmd,
    VelocityMotionModel,
};

/// Errors from model construction.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ModelError {
    /// A physical parameter must be strictly positive.
    #[error("{what} must be positive, got {value}")]
    NonPositive {
        /// Parameter name.
        what: &'static str,
        /// Offending value.
        value: f64,
    },
    /// A noise parameter must be non-negative.
    #[error("{what} must be non-negative, got {value}")]
    Negative {
        /// Parameter name.
        what: &'static str,
        /// Offending value.
        value: f64,
    },
    /// Mixture weights must sum to one.
    #[error("mixture weights sum to {sum}, expected 1")]
    InvalidMixture {
        /// The actual sum.
        sum: f64,
    },
    /// A time step must be positive and finite.
    #[error("invalid time step {0}")]
    InvalidTimeStep(f64),
    /// Input data was unusable (empty, mismatched lengths, non-finite).
    #[error("invalid data: {0}")]
    InvalidData(&'static str),
    /// The geometry makes the model undefined (e.g. a landmark at the sensor origin has
    /// no bearing).
    #[error("degenerate geometry: {0}")]
    DegenerateGeometry(&'static str),
}
