//! The internal data model: what sensors produce and estimators consume, independent of
//! any middleware message definition.
//!
//! ROS adapters convert `sensor_msgs`/`nav_msgs`/`geometry_msgs` to and from these types
//! at the node boundary (see the `robotics_ros_bridge` package); algorithm crates never
//! see a ROS message.

mod covariance;
mod imu;
mod odometry;
mod point;
mod scan;
mod stamped;
mod twist;

pub use covariance::{is_valid_covariance, symmetrize3, Cov3};
pub use imu::{ImuMeasurement, ImuSample};
pub use odometry::Odometry2;
pub use point::{Point2, PointCloud2};
pub use scan::LaserScan;
pub use stamped::Stamped;
pub use twist::Twist2;
