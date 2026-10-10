//! Quality-of-Service presets, chosen per data semantic. Every non-default decision is
//! documented in `docs/ROS2_INTERFACES.md`; the summary:
//!
//! | Preset            | Reliability | Durability      | History      | Used for                       |
//! |-------------------|-------------|-----------------|--------------|--------------------------------|
//! | `sensor_data`     | best effort | volatile        | keep last 5  | `/scan`, `/imu/data`, joints   |
//! | `odometry`        | reliable    | volatile        | keep last 10 | `/odom`, `/state_estimate`     |
//! | `map_latched`     | reliable    | transient local | keep last 1  | `/map`, static costmaps        |
//! | `command`         | reliable    | volatile        | keep last 10 | `/cmd_vel`, `/goal_pose`       |
//! | `tf`              | reliable    | volatile        | keep last 100| `/tf`                          |
//! | `tf_static`       | reliable    | transient local | keep last 100| `/tf_static`                   |
//! | `diagnostics`     | reliable    | volatile        | keep last 10 | `/diagnostics`                 |
//!
//! Rationale: high-rate sensor streams prefer a fresh sample over a retransmitted stale
//! one (best effort); maps and static transforms must reach late-joining subscribers such
//! as RViz or a planner started after the mapper (transient local); commands and
//! estimates must not be silently dropped (reliable).

use rclrs::QoSProfile;

/// High-rate sensor streams: best effort, volatile, shallow queue.
///
/// Matches `rmw_qos_profile_sensor_data`, which is what `ros2 topic echo` and most
/// drivers use for LiDAR and IMU, so a Rust node interoperates without QoS mismatches.
pub fn sensor_data() -> QoSProfile {
    QoSProfile::sensor_data_default().keep_last(5)
}

/// State estimates and odometry: reliable, volatile.
pub fn odometry() -> QoSProfile {
    QoSProfile::default().reliable().volatile().keep_last(10)
}

/// Maps: reliable and *transient local* so late subscribers receive the latest map.
pub fn map_latched() -> QoSProfile {
    QoSProfile::default()
        .reliable()
        .transient_local()
        .keep_last(1)
}

/// Commands and goals: reliable, volatile.
pub fn command() -> QoSProfile {
    QoSProfile::default().reliable().volatile().keep_last(10)
}

/// Dynamic transforms on `/tf` (matches `tf2_ros` defaults).
pub fn tf() -> QoSProfile {
    QoSProfile::default().reliable().volatile().keep_last(100)
}

/// Static transforms on `/tf_static` (matches `tf2_ros::StaticTransformBroadcaster`).
pub fn tf_static() -> QoSProfile {
    QoSProfile::default()
        .reliable()
        .transient_local()
        .keep_last(100)
}

/// `/diagnostics`.
pub fn diagnostics() -> QoSProfile {
    QoSProfile::default().reliable().volatile().keep_last(10)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rclrs::{QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy};

    #[test]
    fn presets_have_the_documented_policies() {
        let s = sensor_data();
        assert_eq!(s.reliability, QoSReliabilityPolicy::BestEffort);
        assert_eq!(s.durability, QoSDurabilityPolicy::Volatile);
        assert_eq!(s.history, QoSHistoryPolicy::KeepLast { depth: 5 });

        let m = map_latched();
        assert_eq!(m.reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(m.durability, QoSDurabilityPolicy::TransientLocal);
        assert_eq!(m.history, QoSHistoryPolicy::KeepLast { depth: 1 });

        assert_eq!(command().reliability, QoSReliabilityPolicy::Reliable);
        assert_eq!(tf_static().durability, QoSDurabilityPolicy::TransientLocal);
        assert_eq!(tf().durability, QoSDurabilityPolicy::Volatile);
    }
}
