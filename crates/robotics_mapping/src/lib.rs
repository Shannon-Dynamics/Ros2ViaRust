//! `robotics_mapping` — mapping with known poses, independent of ROS.
//!
//! Phase 4 of the framework (see `docs/phase-4-mapping/`). The problem: the robot knows where
//! it is (the poses are inputs, not unknowns) and has to write down what the room looks like,
//! as three things the rest of the stack reads: which cells are free, occupied or unknown;
//! how much is still unknown; and how far every cell is from the nearest obstacle.
//!
//! * [`occupancy`]: the log-odds grid, its three-state read-out and the conversion to the
//!   `nav_msgs/OccupancyGrid` values (Probabilistic Robotics, Chapter 9, Table 9.1).
//! * [`raycast`]: the two ways of walking a beam through the grid, Bresenham between cell
//!   centres and the exact supercover of the continuous segment.
//! * [`gate`]: the refusal of scans whose timestamp cannot be trusted while the pose changes,
//!   a measured property of the Webots lidar driver and not a general nicety.
//! * [`integrate`]: the hand-crafted inverse sensor model and the integration of one laser
//!   scan, one update per cell per scan, with a documented policy for beams that returned
//!   nothing.
//! * [`entropy`]: the entropy of the grid, total and over decided cells.
//! * [`esdf`]: the signed Euclidean distance field over the occupied cells, built on the
//!   exact distance transform of `robotics_models` rather than duplicating it.
//! * [`eval`]: the map-accuracy evaluator, which grades a grid against the analytic geometry
//!   of the world and never sees how the grid was made.
//!
//! No algorithm here reads a pose estimate. The poses come in as arguments, and where they
//! came from is the business of whoever calls: that separation is the point of the phase.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod entropy;
pub mod esdf;
pub mod eval;
pub mod gate;
pub mod integrate;
pub mod occupancy;
pub mod raycast;

pub use entropy::{cell_entropy_bits, grid_entropy, GridEntropy};
pub use esdf::{signed_esdf, EsdfGrid};
pub use eval::{evaluate, EvalParams, GroundTruth, MapMetrics, SurfaceErrorStats, SurfaceSample};
pub use gate::MotionGate;
pub use integrate::{NoReturn, RayTracer, ScanIntegrator, ScanReport, DEFAULT_CLEAR_MARGIN};
pub use occupancy::{CellState, GridGeometry, OccupancyGrid, OccupancyParams};

/// Errors of the mapping library.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum MappingError {
    /// A parameter is outside its valid range.
    #[error("invalid parameter {what}: {value}")]
    InvalidParameter {
        /// Which parameter.
        what: &'static str,
        /// The offending value.
        value: f64,
    },
    /// A serialized grid or a landmark file could not be parsed.
    #[error("parse error: {0}")]
    Parse(String),
    /// A scan has no usable structure (for example, a non-finite bearing step).
    #[error("malformed scan: {0}")]
    MalformedScan(&'static str),
    /// The sensor lies outside the grid, so nothing can be traced from it.
    #[error("the sensor at ({x}, {y}) is outside the grid")]
    SensorOutsideGrid {
        /// Sensor x (m).
        x: f64,
        /// Sensor y (m).
        y: f64,
    },
}
