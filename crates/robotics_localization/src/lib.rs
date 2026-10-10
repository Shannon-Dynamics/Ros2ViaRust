//! `robotics_localization` — pose estimation against a known map, independent of ROS.
//!
//! Phase 3 of the framework (see `docs/phase-3-localization/`). The problem is the one
//! every dead reckoning system has: odometry drifts without bound, so the robot needs
//! absolute information to stay located, and a way to find itself when it has no idea
//! where it is.
//!
//! This crate answers the question two ways:
//!
//! * [`map`]: the landmark map, read from the simulation's own ground truth file so that
//!   the map and the world cannot drift apart.
//! * [`pillars`]: range and bearing observations of the round pillars, extracted from a
//!   laser scan.
//! * [`ekf`]: EKF localization on SE(2), with maximum likelihood association and a
//!   chi-square gate. One Gaussian belief: tracks a known pose well, and structurally
//!   cannot recover from a kidnapping, which is its own test's point.
//! * [`mcl`]: particle filter localization with the same landmark model and a documented
//!   random-injection recovery strategy. A population of hypotheses: tracks a known pose
//!   about as well, and can recover from a kidnapping because losing a wrong hypothesis
//!   costs it nothing.
//!
//! Both read the same map and the same sensor model; only the representation of the
//! belief, and what each can do with it, differs. That comparison is Phase 3's answer to
//! the kidnapped robot problem (Thrun and Fox), and it is measured, not asserted, in
//! `evidence/phase-3-correctness/`.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod ekf;
pub mod map;
pub mod mcl;
pub mod pillars;

pub use ekf::{
    EkfLocalizer, LandmarkMeasurement, LocalizationError, LocalizerConfig, LocalizerStats,
    OdometryMotion, UpdateReport,
};
pub use map::{LandmarkEntry, MapError, MapFile};
pub use mcl::{McLocalizer, McUpdateReport, MclConfig, MclStats};
pub use pillars::{detect, Detection, PillarDetector};
