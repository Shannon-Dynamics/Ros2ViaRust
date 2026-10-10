//! Planar odometry: an integrated pose plus the instantaneous body velocity.

use crate::frame::FrameId;
use crate::se2::Se2;
use crate::time::Timestamp;
use crate::types::{Cov3, Twist2};

/// A planar odometry estimate, the internal twin of `nav_msgs/Odometry`.
#[derive(Debug, Clone, PartialEq)]
pub struct Odometry2 {
    /// Time of the estimate.
    pub stamp: Timestamp,
    /// Frame the pose is expressed in (typically `odom`).
    pub frame: FrameId,
    /// Frame the pose describes (typically `base_link`); the twist is in this frame.
    pub child_frame: FrameId,
    /// Pose of `child_frame` in `frame`.
    pub pose: Se2,
    /// Pose covariance over `(x, y, θ)`.
    pub pose_cov: Cov3,
    /// Body velocity.
    pub twist: Twist2,
    /// Twist covariance over `(v_x, v_y, ω)`.
    pub twist_cov: Cov3,
}

impl Default for Odometry2 {
    fn default() -> Self {
        Self {
            stamp: Timestamp::ZERO,
            frame: FrameId::default(),
            child_frame: FrameId::default(),
            pose: Se2::IDENTITY,
            pose_cov: Cov3::zeros(),
            twist: Twist2::ZERO,
            twist_cov: Cov3::zeros(),
        }
    }
}
