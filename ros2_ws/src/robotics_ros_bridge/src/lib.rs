//! `robotics_ros_bridge` — the only place where ROS 2 message types and
//! [`robotics_core`] types meet.
//!
//! ```text
//!   sensor_msgs / nav_msgs / geometry_msgs   ⇄   robotics_core types
//!                        (this crate: convert.rs)
//! ```
//!
//! * [`convert`]: lossless converters in both directions, returning errors for malformed
//!   messages instead of panicking.
//! * [`qos`]: the framework's Quality-of-Service presets, one per data semantic.
//! * [`tf`]: TF broadcaster and listener (`/tf`, `/tf_static`) for planar transforms; the
//!   listener feeds `robotics_core::TransformBuffer` and detects TF authority conflicts.
//! * [`diagnostics`]: a `diagnostic_msgs/DiagnosticArray` publisher with a small builder.
//! * [`params`]: `ParamStore`, validated read-only parameters whose handles stay alive.
//! * [`shared`]: `Shared<T>` node state for node-level callbacks (no rclrs `Worker`s).
//! * [`shutdown`]: SIGINT/SIGTERM → stop the executor (rclrs 0.8 has no signal handler).
//!
//! This crate needs a sourced ROS 2 Jazzy environment to compile (it links `rclrs`).

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod convert;
pub mod diagnostics;
pub mod params;
pub mod qos;
pub mod shared;
pub mod shutdown;
pub mod tf;

pub use convert::ConvertError;
pub use diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
pub use params::ParamStore;
pub use tf::{TfBroadcaster, TfListener, TfListenerStats};

/// Re-export of the generated interface crates so downstream packages import one thing.
pub use ros_env;
