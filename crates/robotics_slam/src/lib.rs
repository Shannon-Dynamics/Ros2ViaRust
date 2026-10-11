//! `robotics_slam` — simultaneous localization and mapping, independent of ROS.
//!
//! Phase 5 of the framework (see `docs/phase-5-slam/`). The poses are now unknowns: the library
//! turns scans into relative-pose constraints, holds them in a factor graph, solves for the
//! trajectory, and believes a loop closure only after it has been verified.
//!
//! * [`geom`]: the SE(2) calculus (left and right Jacobians, the relative-pose residual with
//!   exact Jacobians) every other module uses.
//! * [`voxel`]: a voxel-hash point map with a fixed hasher and a point cap.
//! * [`icp`]: point-to-point and point-to-plane ICP, and the ICP information matrix.
//! * [`adaptive`], [`odometry`]: the KISS-style adaptive correspondence threshold and the
//!   LiDAR odometry front end that uses it.
//! * [`ndt`]: the normal distributions transform.
//! * [`kernel`], [`factor_graph`]: robust kernels and the factor-graph optimizer (Gauss-Newton
//!   and Levenberg-Marquardt, dense and sparse solvers, marginal covariances).
//! * [`pose_graph`]: keyframes, loop-candidate proposal and the four-check loop verifier.
//! * [`ekf_slam`], [`rbpf`]: the two alternative back ends.
//!
//! **Nothing here reads a map or publishes one.** The occupancy grid of `robotics_mapping` is not
//! a dependency of this crate (decision DS12), because the grid a SLAM node would render into is
//! the thing Phase 4's claim M1 has not validated. Nothing reads a clock and nothing uses OS
//! entropy: the same inputs give the same outputs bit for bit (DS15).

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod adaptive;
pub mod ekf_slam;
pub mod factor_graph;
pub mod geom;
pub mod icp;
pub mod kernel;
pub mod ndt;
pub mod odometry;
pub mod pose_graph;
pub mod rbpf;
pub mod voxel;

/// Errors of the SLAM library.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SlamError {
    /// A parameter is outside its valid range.
    #[error("invalid parameter {what}: {value}")]
    InvalidParameter {
        /// Which parameter.
        what: &'static str,
        /// The offending value.
        value: f64,
    },
    /// An input that must not be empty was.
    #[error("empty input: {0}")]
    EmptyInput(&'static str),
    /// The normal equations could not be factorized. With a prior factor on one pose this means
    /// the graph is under-constrained somewhere else; it is never papered over with a nudge.
    #[error("the normal equations are singular: {0}")]
    SingularSystem(&'static str),
    /// A variable key is not in the graph or values.
    #[error("unknown variable {0}")]
    UnknownVariable(String),
    /// A non-finite number reached a place that cannot tolerate one.
    #[error("non-finite value in {0}")]
    NonFinite(&'static str),
}
