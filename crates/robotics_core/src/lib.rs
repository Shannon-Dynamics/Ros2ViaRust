//! `robotics_core` — the ROS-independent foundation of ros2-rust-robotics.
//!
//! This crate owns the *internal data model* every algorithm crate and every ROS adapter
//! agrees on. Nothing in here knows about ROS messages, DDS or threads; it is plain Rust
//! over `nalgebra`, builds for native targets and `wasm32`, and is deterministic when
//! seeded.
//!
//! Reference: *Probabilistic Robotics via Rust* (Shannon Dynamics), Chapters 2–4:
//! probability (Gaussians in moments and canonical form, normal and χ² quantiles), the
//! geometry of motion (SE(2), ⊞/⊟ retractions) and the sensor data model of the simulator
//! chapter. [`tf`] holds the planar transform buffer behind the ROS TF listener.
//!
//! # Conventions (framework-wide, do not change silently)
//! * Poses are [`Se2`] elements; tangent vectors are ordered translation-first
//!   `τ = (v_x, v_y, ω)`.
//! * Perturbations are **right / body-frame**: `x ⊞ δ = x · exp(δ)`, `y ⊟ x = log(x⁻¹ · y)`.
//! * Angles are normalized to `(−π, π]`.
//! * Covariances of a pose live in the tangent space at that pose, in the same
//!   `(x, y, θ)` order (this is also the order `geometry_msgs` uses for the planar block).
//! * Noise covariance naming follows Thrun: `R` = motion noise, `Q` = measurement noise.
//! * All timestamps are nanoseconds since the epoch of the source clock ([`Timestamp`]).

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod angle;
pub mod error;
pub mod frame;
pub mod prob;
pub mod se2;
pub mod tf;
pub mod time;
pub mod types;

pub use error::CoreError;
pub use frame::FrameId;
pub use prob::{seeded_rng, Canonical, Gaussian, BOOK_SEED};
pub use se2::{Manifold, Se2, Tangent2};
pub use tf::{LookupTime, TfError, TransformBuffer, TransformBufferConfig, TransformStamped2};
pub use time::Timestamp;
pub use types::{
    Cov3, ImuMeasurement, ImuSample, LaserScan, Odometry2, Point2, PointCloud2, Stamped, Twist2,
};

/// Re-export of the linear-algebra crate the whole framework is built on, so downstream
/// crates use exactly one `nalgebra` version.
pub use nalgebra;
