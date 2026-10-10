//! `robotics_localization` — pose estimation against a known map, independent of ROS.
//!
//! Phase 3 of the framework (see `docs/phase-3-localization/`). The problem is the one
//! every dead reckoning system has: odometry drifts without bound, so the robot needs
//! absolute information to stay located, and a way to find itself when it has no idea
//! where it is.
//!
//! This crate is the first half of the answer, position tracking with a Gaussian belief:
//!
//! * [`map`]: the landmark map, read from the simulation's own ground truth file so that
//!   the map and the world cannot drift apart.
//! * [`pillars`]: range and bearing observations of the round pillars, extracted from a
//!   laser scan.
//! * [`ekf`]: EKF localization on SE(2), with maximum likelihood association and a
//!   chi-square gate.
//!
//! The second half, a belief that can be multimodal and can therefore solve global
//! localization and recover from a kidnapping, is Monte Carlo localization and is not in
//! this crate yet. The pieces it needs (`ParticleSet`, KLD sampling, beam and likelihood
//! field models) are already in `robotics_filters` and `robotics_models`.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod ekf;
pub mod map;
pub mod pillars;

pub use ekf::{
    EkfLocalizer, LandmarkMeasurement, LocalizationError, LocalizerConfig, LocalizerStats,
    OdometryMotion, UpdateReport,
};
pub use map::{LandmarkEntry, MapError, MapFile};
pub use pillars::{detect, Detection, PillarDetector};
