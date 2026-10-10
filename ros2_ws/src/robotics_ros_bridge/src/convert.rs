//! Lossless converters between ROS 2 messages and [`robotics_core`] types.
//!
//! Direction naming: `*_to_ros` builds a message from a core type, `*_from_ros` parses a
//! message into a core type and returns [`ConvertError`] for malformed input.
//!
//! Planar pose ↔ 3-D message: yaw `θ` maps to the quaternion `(0, 0, sin θ/2, cos θ/2)`;
//! the `(x, y, θ)` covariance occupies rows/columns `(0, 1, 5)` of the 6×6 message
//! covariance (the `x, y, yaw` slots of `geometry_msgs/PoseWithCovariance`).

use robotics_core::nalgebra::{Matrix2, Vector2};
use robotics_core::nalgebra::{Matrix3, UnitQuaternion, Vector3};
use robotics_core::{
    Cov3, FrameId, ImuMeasurement, ImuSample, LaserScan, Odometry2, Se2, Timestamp,
    TransformStamped2, Twist2,
};
use ros_env::{builtin_interfaces, geometry_msgs, nav_msgs, sensor_msgs, std_msgs};

/// Why a message could not be converted.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ConvertError {
    /// A numeric field was NaN or infinite.
    #[error("non-finite value in {0}")]
    NonFinite(&'static str),
    /// The orientation quaternion had (near) zero norm.
    #[error("invalid quaternion (norm {0})")]
    InvalidQuaternion(f64),
    /// A scan violated its own structural invariants.
    #[error("malformed LaserScan: {0}")]
    MalformedScan(&'static str),
    /// A covariance field was malformed (asymmetric, negative variance, non-finite).
    #[error("invalid covariance in {0}")]
    InvalidCovariance(&'static str),
    /// The header stamp was zero.
    #[error("zero timestamp")]
    InvalidTimestamp,
    /// A 3-D rotation had roll or pitch beyond [`PLANAR_TOLERANCE`]; projecting it onto
    /// the plane would silently misplace the child frame.
    #[error("non-planar rotation (|roll| or |pitch| = {0} rad)")]
    NonPlanar(f64),
    /// A frame id was empty.
    #[error("empty frame id")]
    EmptyFrameId,
}

/// Largest roll or pitch (rad) a transform may carry and still be treated as planar
/// (≈ 0.57°, e.g. a LiDAR mounted a little crooked). Translations along z are dropped.
pub const PLANAR_TOLERANCE: f64 = 0.01;

/// Indices of `(x, y, yaw)` inside the 6×6 row-major covariance of ROS pose/twist messages.
const PLANAR_SLOTS: [usize; 3] = [0, 1, 5];

// ---------------------------------------------------------------- time & header

/// [`Timestamp`] → `builtin_interfaces/Time`.
pub fn time_to_ros(t: Timestamp) -> builtin_interfaces::msg::Time {
    let (sec, nanosec) = t.sec_nanosec();
    builtin_interfaces::msg::Time { sec, nanosec }
}

/// `builtin_interfaces/Time` → [`Timestamp`].
pub fn time_from_ros(t: &builtin_interfaces::msg::Time) -> Timestamp {
    Timestamp::from_sec_nanosec(t.sec, t.nanosec)
}

/// `rclrs::Time` (a clock reading) → [`Timestamp`].
pub fn time_from_rclrs(t: &rclrs::Time) -> Result<Timestamp, ConvertError> {
    let m = t
        .to_ros_msg()
        .map_err(|_| ConvertError::NonFinite("clock time"))?;
    Ok(time_from_ros(&m))
}

/// Current time of a node's clock as a [`Timestamp`].
pub fn now(node: &rclrs::Node) -> Result<Timestamp, ConvertError> {
    time_from_rclrs(&node.get_clock().now())
}

/// Build a `std_msgs/Header`.
pub fn header(stamp: Timestamp, frame: &FrameId) -> std_msgs::msg::Header {
    std_msgs::msg::Header {
        stamp: time_to_ros(stamp),
        frame_id: frame.as_str().to_owned(),
    }
}

// ---------------------------------------------------------------- pose / transform

/// Yaw → unit quaternion about `z`.
pub fn yaw_to_quaternion(theta: f64) -> geometry_msgs::msg::Quaternion {
    let (s, c) = (0.5 * theta).sin_cos();
    geometry_msgs::msg::Quaternion {
        x: 0.0,
        y: 0.0,
        z: s,
        w: c,
    }
}

/// Unit quaternion → yaw (rotation about `z`), tolerant of non-normalized input.
pub fn quaternion_to_yaw(q: &geometry_msgs::msg::Quaternion) -> Result<f64, ConvertError> {
    if ![q.x, q.y, q.z, q.w].iter().all(|v| v.is_finite()) {
        return Err(ConvertError::NonFinite("quaternion"));
    }
    let norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if norm2 < 1e-12 {
        return Err(ConvertError::InvalidQuaternion(norm2.sqrt()));
    }
    Ok((2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
}

/// [`Se2`] → `geometry_msgs/Pose` (z = 0).
pub fn pose_to_ros(p: &Se2) -> geometry_msgs::msg::Pose {
    geometry_msgs::msg::Pose {
        position: geometry_msgs::msg::Point {
            x: p.x(),
            y: p.y(),
            z: 0.0,
        },
        orientation: yaw_to_quaternion(p.theta()),
    }
}

/// `geometry_msgs/Pose` → [`Se2`] (z, roll and pitch are dropped).
pub fn pose_from_ros(p: &geometry_msgs::msg::Pose) -> Result<Se2, ConvertError> {
    if !(p.position.x.is_finite() && p.position.y.is_finite()) {
        return Err(ConvertError::NonFinite("position"));
    }
    Ok(Se2::new(
        p.position.x,
        p.position.y,
        quaternion_to_yaw(&p.orientation)?,
    ))
}

/// [`Se2`] → `geometry_msgs/Transform`.
pub fn transform_to_ros(p: &Se2) -> geometry_msgs::msg::Transform {
    geometry_msgs::msg::Transform {
        translation: geometry_msgs::msg::Vector3 {
            x: p.x(),
            y: p.y(),
            z: 0.0,
        },
        rotation: yaw_to_quaternion(p.theta()),
    }
}

/// `geometry_msgs/Transform` → [`Se2`].
pub fn transform_from_ros(t: &geometry_msgs::msg::Transform) -> Result<Se2, ConvertError> {
    if !(t.translation.x.is_finite() && t.translation.y.is_finite()) {
        return Err(ConvertError::NonFinite("translation"));
    }
    Ok(Se2::new(
        t.translation.x,
        t.translation.y,
        quaternion_to_yaw(&t.rotation)?,
    ))
}

/// A stamped transform `parent → child`.
pub fn transform_stamped(
    stamp: Timestamp,
    parent: &FrameId,
    child: &FrameId,
    pose: &Se2,
) -> geometry_msgs::msg::TransformStamped {
    geometry_msgs::msg::TransformStamped {
        header: header(stamp, parent),
        child_frame_id: child.as_str().to_owned(),
        transform: transform_to_ros(pose),
    }
}

/// tf2 accepts frame ids with a leading `/` for backwards compatibility and strips it.
fn tf_frame(name: &str) -> Result<FrameId, ConvertError> {
    let name = name.strip_prefix('/').unwrap_or(name);
    if name.is_empty() {
        return Err(ConvertError::EmptyFrameId);
    }
    Ok(FrameId::new(name))
}

/// `geometry_msgs/TransformStamped` → planar [`TransformStamped2`].
///
/// Validation: non-empty frame ids (a leading `/` is stripped, as tf2 does), finite
/// translation and quaternion, quaternion norm not ≈ 0, `|roll|, |pitch| ≤`
/// [`PLANAR_TOLERANCE`]. The z translation is dropped (sensor heights do not affect planar
/// geometry); yaw is the z-y-x Euler yaw of the rotation.
pub fn transform_stamped_from_ros(
    m: &geometry_msgs::msg::TransformStamped,
) -> Result<TransformStamped2, ConvertError> {
    let parent = tf_frame(&m.header.frame_id)?;
    let child = tf_frame(&m.child_frame_id)?;
    let t = &m.transform.translation;
    if !(t.x.is_finite() && t.y.is_finite() && t.z.is_finite()) {
        return Err(ConvertError::NonFinite("translation"));
    }
    let q = &m.transform.rotation;
    let yaw = quaternion_to_yaw(q)?;
    let n = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
    let (x, y, z, w) = (q.x / n, q.y / n, q.z / n, q.w / n);
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let tilt = roll.abs().max(pitch.abs());
    if tilt > PLANAR_TOLERANCE {
        return Err(ConvertError::NonPlanar(tilt));
    }
    Ok(TransformStamped2 {
        stamp: time_from_ros(&m.header.stamp),
        parent,
        child,
        transform: Se2::new(t.x, t.y, yaw),
    })
}

/// [`TransformStamped2`] → `geometry_msgs/TransformStamped`.
pub fn transform_stamped_to_ros(t: &TransformStamped2) -> geometry_msgs::msg::TransformStamped {
    transform_stamped(t.stamp, &t.parent, &t.child, &t.transform)
}

/// A `geometry_msgs/PoseWithCovarianceStamped` (e.g. `/state_estimate`).
pub fn pose_with_covariance_stamped_to_ros(
    stamp: Timestamp,
    frame: &FrameId,
    pose: &Se2,
    cov: &Cov3,
) -> geometry_msgs::msg::PoseWithCovarianceStamped {
    geometry_msgs::msg::PoseWithCovarianceStamped {
        header: header(stamp, frame),
        pose: geometry_msgs::msg::PoseWithCovariance {
            pose: pose_to_ros(pose),
            covariance: cov3_to_ros(cov),
        },
    }
}

// ---------------------------------------------------------------- covariance

/// `(x, y, θ)` covariance → 6×6 row-major message covariance. Unused 3-D slots are zero.
pub fn cov3_to_ros(c: &Cov3) -> [f64; 36] {
    let mut out = [0.0; 36];
    for (i, &ri) in PLANAR_SLOTS.iter().enumerate() {
        for (j, &cj) in PLANAR_SLOTS.iter().enumerate() {
            out[ri * 6 + cj] = c[(i, j)];
        }
    }
    out
}

/// 6×6 message covariance → `(x, y, θ)` covariance.
pub fn cov3_from_ros(c: &[f64; 36]) -> Result<Cov3, ConvertError> {
    let mut out = Cov3::zeros();
    for (i, &ri) in PLANAR_SLOTS.iter().enumerate() {
        for (j, &cj) in PLANAR_SLOTS.iter().enumerate() {
            let v = c[ri * 6 + cj];
            if !v.is_finite() {
                return Err(ConvertError::NonFinite("covariance"));
            }
            out[(i, j)] = v;
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------- twist

/// [`Twist2`] → `geometry_msgs/Twist`.
pub fn twist_to_ros(t: &Twist2) -> geometry_msgs::msg::Twist {
    geometry_msgs::msg::Twist {
        linear: geometry_msgs::msg::Vector3 {
            x: t.vx,
            y: t.vy,
            z: 0.0,
        },
        angular: geometry_msgs::msg::Vector3 {
            x: 0.0,
            y: 0.0,
            z: t.omega,
        },
    }
}

/// `geometry_msgs/Twist` → [`Twist2`].
pub fn twist_from_ros(t: &geometry_msgs::msg::Twist) -> Result<Twist2, ConvertError> {
    let out = Twist2 {
        vx: t.linear.x,
        vy: t.linear.y,
        omega: t.angular.z,
    };
    if !out.is_finite() {
        return Err(ConvertError::NonFinite("twist"));
    }
    Ok(out)
}

// ---------------------------------------------------------------- odometry

/// [`Odometry2`] → `nav_msgs/Odometry`.
pub fn odometry_to_ros(o: &Odometry2) -> nav_msgs::msg::Odometry {
    nav_msgs::msg::Odometry {
        header: header(o.stamp, &o.frame),
        child_frame_id: o.child_frame.as_str().to_owned(),
        pose: geometry_msgs::msg::PoseWithCovariance {
            pose: pose_to_ros(&o.pose),
            covariance: cov3_to_ros(&o.pose_cov),
        },
        twist: geometry_msgs::msg::TwistWithCovariance {
            twist: twist_to_ros(&o.twist),
            covariance: cov3_to_ros(&o.twist_cov),
        },
    }
}

/// `nav_msgs/Odometry` → [`Odometry2`].
pub fn odometry_from_ros(m: &nav_msgs::msg::Odometry) -> Result<Odometry2, ConvertError> {
    Ok(Odometry2 {
        stamp: time_from_ros(&m.header.stamp),
        frame: FrameId::new(&m.header.frame_id),
        child_frame: FrameId::new(&m.child_frame_id),
        pose: pose_from_ros(&m.pose.pose)?,
        pose_cov: cov3_from_ros(&m.pose.covariance)?,
        twist: twist_from_ros(&m.twist.twist)?,
        twist_cov: cov3_from_ros(&m.twist.covariance)?,
    })
}

// ---------------------------------------------------------------- laser scan

/// `sensor_msgs/LaserScan` → [`LaserScan`]. Individual NaN/±inf ranges are *kept* (REP 117:
/// `+inf` no return, `−inf` too close, NaN invalid; `LaserScan::valid_beams` skips them);
/// only structural problems error: non-finite metadata, zero increment, `range_min < 0`,
/// `range_max ≤ range_min`, mismatched intensities, no beams, and an `angle_max` that
/// disagrees with `angle_min + (n − 1)·angle_increment` by more than one increment (drivers
/// differ on whether `angle_max` is inclusive; more than that is a corrupt message).
pub fn laser_scan_from_ros(m: &sensor_msgs::msg::LaserScan) -> Result<LaserScan, ConvertError> {
    if !m.angle_max.is_finite() {
        return Err(ConvertError::MalformedScan("angle_max not finite"));
    }
    let scan = LaserScan {
        stamp: time_from_ros(&m.header.stamp),
        frame: FrameId::new(&m.header.frame_id),
        angle_min: m.angle_min as f64,
        angle_increment: m.angle_increment as f64,
        time_increment: m.time_increment as f64,
        scan_time: m.scan_time as f64,
        range_min: m.range_min as f64,
        range_max: m.range_max as f64,
        ranges: m.ranges.clone(),
        intensities: m.intensities.clone(),
    };
    if !scan.is_well_formed() {
        return Err(ConvertError::MalformedScan(
            "angle/range metadata or intensity length",
        ));
    }
    if scan.ranges.is_empty() {
        return Err(ConvertError::MalformedScan("no beams"));
    }
    if scan.ranges.len() > 1 {
        let mismatch = (m.angle_max as f64 - scan.angle_max()).abs();
        if mismatch > scan.angle_increment.abs() * 1.01 + 1e-5 {
            return Err(ConvertError::MalformedScan(
                "angle_max inconsistent with angle_min, angle_increment and the number of ranges",
            ));
        }
    }
    Ok(scan)
}

/// [`LaserScan`] → `sensor_msgs/LaserScan`.
pub fn laser_scan_to_ros(s: &LaserScan) -> sensor_msgs::msg::LaserScan {
    let n = s.ranges.len();
    let angle_max = if n > 0 { s.bearing(n - 1) } else { s.angle_min };
    sensor_msgs::msg::LaserScan {
        header: header(s.stamp, &s.frame),
        angle_min: s.angle_min as f32,
        angle_max: angle_max as f32,
        angle_increment: s.angle_increment as f32,
        time_increment: s.time_increment as f32,
        scan_time: s.scan_time as f32,
        range_min: s.range_min as f32,
        range_max: s.range_max as f32,
        ranges: s.ranges.clone(),
        intensities: s.intensities.clone(),
    }
}

// ---------------------------------------------------------------- imu

fn cov9(c: &[f64; 9]) -> Option<Matrix3<f64>> {
    // REP 145 / sensor_msgs convention: a leading -1 means "unknown".
    if c[0] < 0.0 {
        return None;
    }
    Some(Matrix3::from_row_slice(c))
}

/// `sensor_msgs/Imu` → [`ImuSample`]. A covariance whose first element is `-1` is reported
/// as `None`, and an all-zero orientation quaternion (the "no orientation" convention) as
/// `None`.
pub fn imu_from_ros(m: &sensor_msgs::msg::Imu) -> Result<ImuSample, ConvertError> {
    let av = Vector3::new(
        m.angular_velocity.x,
        m.angular_velocity.y,
        m.angular_velocity.z,
    );
    let la = Vector3::new(
        m.linear_acceleration.x,
        m.linear_acceleration.y,
        m.linear_acceleration.z,
    );
    if !(av.iter().all(|v| v.is_finite()) && la.iter().all(|v| v.is_finite())) {
        return Err(ConvertError::NonFinite("imu"));
    }
    let q = &m.orientation;
    let orientation = if m.orientation_covariance[0] < 0.0
        || (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0)
    {
        None
    } else {
        Some(UnitQuaternion::from_quaternion(
            robotics_core::nalgebra::Quaternion::new(q.w, q.x, q.y, q.z),
        ))
    };
    Ok(ImuSample {
        stamp: time_from_ros(&m.header.stamp),
        frame: FrameId::new(&m.header.frame_id),
        angular_velocity: av,
        angular_velocity_cov: cov9(&m.angular_velocity_covariance),
        linear_acceleration: la,
        linear_acceleration_cov: cov9(&m.linear_acceleration_covariance),
        orientation,
        orientation_cov: if orientation.is_some() {
            cov9(&m.orientation_covariance)
        } else {
            None
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn pose_round_trip_including_branch_cut() {
        for th in [0.0, 1.0, -2.5, PI, -PI + 1e-6, 3.0] {
            let p = Se2::new(1.5, -0.25, th);
            let back = pose_from_ros(&pose_to_ros(&p)).unwrap();
            assert_relative_eq!(back.x(), p.x(), epsilon = 1e-12);
            assert_relative_eq!(back.y(), p.y(), epsilon = 1e-12);
            assert!(robotics_core::angle::angle_diff(back.theta(), p.theta()).abs() < 1e-12);
            let t = transform_from_ros(&transform_to_ros(&p)).unwrap();
            assert!(t.boxminus_norm(&p) < 1e-12);
        }
    }

    trait BoxminusNorm {
        fn boxminus_norm(&self, other: &Se2) -> f64;
    }
    impl BoxminusNorm for Se2 {
        fn boxminus_norm(&self, other: &Se2) -> f64 {
            use robotics_core::Manifold;
            self.boxminus(other).norm()
        }
    }

    #[test]
    fn covariance_slots_are_x_y_yaw() {
        let c = Cov3::new(1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0);
        let m = cov3_to_ros(&c);
        assert_eq!(m[0], 1.0);
        assert_eq!(m[7], 2.0);
        assert_eq!(m[35], 3.0);
        assert_eq!(m[5], 0.2);
        assert_eq!(m[30], 0.2);
        assert_eq!(m[2], 0.0);
        let back = cov3_from_ros(&m).unwrap();
        assert_eq!(back, c);
    }

    #[test]
    fn odometry_round_trip() {
        let o = Odometry2 {
            stamp: Timestamp::from_sec_nanosec(12, 500),
            frame: "odom".into(),
            child_frame: "base_link".into(),
            pose: Se2::new(0.3, -0.7, 0.9),
            pose_cov: Cov3::identity() * 0.01,
            twist: Twist2::unicycle(0.4, -0.2),
            twist_cov: Cov3::identity() * 0.02,
        };
        let m = odometry_to_ros(&o);
        assert_eq!(m.header.frame_id, "odom");
        assert_eq!(m.child_frame_id, "base_link");
        let back = odometry_from_ros(&m).unwrap();
        assert_eq!(back.stamp, o.stamp);
        assert!(back.pose.boxminus_norm(&o.pose) < 1e-12);
        assert_eq!(back.pose_cov, o.pose_cov);
        assert_eq!(back.twist, o.twist);
    }

    #[test]
    fn scan_round_trip_and_validation() {
        let s = LaserScan {
            stamp: Timestamp::from_secs_f64(1.0),
            frame: "laser".into(),
            angle_min: -1.0,
            angle_increment: 0.5,
            range_min: 0.1,
            range_max: 10.0,
            ranges: vec![1.0, f32::NAN, 3.0],
            ..Default::default()
        };
        let m = laser_scan_to_ros(&s);
        assert_relative_eq!(m.angle_max as f64, 0.0, epsilon = 1e-6);
        let back = laser_scan_from_ros(&m).unwrap();
        assert_eq!(back.ranges.len(), 3);
        assert_eq!(back.valid_beams().count(), 2);

        let mut bad = m.clone();
        bad.range_max = 0.0;
        assert!(matches!(
            laser_scan_from_ros(&bad),
            Err(ConvertError::MalformedScan(_))
        ));
        let mut empty = m.clone();
        empty.ranges.clear();
        assert!(laser_scan_from_ros(&empty).is_err());

        // angle_max off by one increment (exclusive convention) is tolerated, more is not.
        let mut off_by_one = m.clone();
        off_by_one.angle_max += off_by_one.angle_increment;
        assert!(laser_scan_from_ros(&off_by_one).is_ok());
        let mut corrupt = m.clone();
        corrupt.angle_max += 3.0 * corrupt.angle_increment;
        assert!(laser_scan_from_ros(&corrupt).is_err());
        let corruptions: [fn(&mut sensor_msgs::msg::LaserScan); 5] = [
            |s| s.angle_increment = 0.0,
            |s| s.angle_min = f32::NAN,
            |s| s.range_min = -1.0,
            |s| s.intensities = vec![1.0],
            |s| s.angle_max = f32::INFINITY,
        ];
        for f in corruptions {
            let mut bad = m.clone();
            f(&mut bad);
            assert!(laser_scan_from_ros(&bad).is_err(), "{bad:?}");
        }
        // REP 117 ranges survive the round trip untouched.
        let mut special = s.clone();
        special.ranges = vec![f32::INFINITY, f32::NEG_INFINITY, f32::NAN];
        let back = laser_scan_from_ros(&laser_scan_to_ros(&special)).unwrap();
        assert_eq!(back.ranges[0], f32::INFINITY);
        assert_eq!(back.ranges[1], f32::NEG_INFINITY);
        assert!(back.ranges[2].is_nan());
        assert_eq!(back.valid_beams().count(), 0);
    }

    #[test]
    fn planar_transform_round_trip_and_validation() {
        let t = TransformStamped2::new(
            Timestamp::from_sec_nanosec(3, 7),
            "base_link",
            "laser",
            Se2::new(0.05, -0.02, 0.3),
        );
        let m = transform_stamped_to_ros(&t);
        assert_eq!(m.header.frame_id, "base_link");
        assert_eq!(m.child_frame_id, "laser");
        let back = transform_stamped_from_ros(&m).unwrap();
        assert_eq!(back.stamp, t.stamp);
        assert_eq!(back.parent, t.parent);
        assert!(back.transform.boxminus_norm(&t.transform) < 1e-12);

        // Leading slashes are stripped, z is dropped.
        let mut slashed = m.clone();
        slashed.header.frame_id = "/base_link".into();
        slashed.transform.translation.z = 0.17;
        assert_eq!(
            transform_stamped_from_ros(&slashed)
                .unwrap()
                .parent
                .as_str(),
            "base_link"
        );

        // A 5° roll is not planar; a 0.1° roll is.
        let tilted = |roll: f64| {
            let mut q = m.clone();
            let (sr, cr) = (0.5 * roll).sin_cos();
            // Rotation = yaw(0.3) · roll(roll): q = q_yaw ⊗ q_roll.
            let (sy, cy) = (0.15f64).sin_cos();
            q.transform.rotation = geometry_msgs::msg::Quaternion {
                w: cy * cr,
                x: cy * sr,
                y: sy * sr,
                z: sy * cr,
            };
            q
        };
        assert!(matches!(
            transform_stamped_from_ros(&tilted(5f64.to_radians())),
            Err(ConvertError::NonPlanar(_))
        ));
        let slight = transform_stamped_from_ros(&tilted(0.1f64.to_radians())).unwrap();
        assert!((slight.transform.theta() - 0.3).abs() < 1e-5);

        let mut empty = m.clone();
        empty.child_frame_id = "/".into();
        assert_eq!(
            transform_stamped_from_ros(&empty),
            Err(ConvertError::EmptyFrameId)
        );
        let mut nan = m.clone();
        nan.transform.translation.x = f64::NAN;
        assert!(transform_stamped_from_ros(&nan).is_err());
    }

    #[test]
    fn state_estimate_message() {
        let c = Cov3::new(1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0);
        let m = pose_with_covariance_stamped_to_ros(
            Timestamp::from_sec_nanosec(9, 0),
            &FrameId::new("odom"),
            &Se2::new(1.0, 2.0, -0.5),
            &c,
        );
        assert_eq!(m.header.frame_id, "odom");
        assert_eq!(m.header.stamp.sec, 9);
        assert_eq!(cov3_from_ros(&m.pose.covariance).unwrap(), c);
        assert!(
            pose_from_ros(&m.pose.pose)
                .unwrap()
                .boxminus_norm(&Se2::new(1.0, 2.0, -0.5))
                < 1e-12
        );
    }

    #[test]
    fn imu_unknown_covariance_is_none() {
        let mut m = sensor_msgs::msg::Imu::default();
        m.orientation_covariance[0] = -1.0;
        m.angular_velocity.z = 0.25;
        let s = imu_from_ros(&m).unwrap();
        assert!(s.orientation.is_none());
        assert_relative_eq!(s.yaw_rate(), 0.25);
        assert!(s.angular_velocity_cov.is_some());
    }

    #[test]
    fn bad_quaternion_is_rejected() {
        let mut p = geometry_msgs::msg::Pose::default();
        p.orientation.w = 0.0;
        assert!(matches!(
            pose_from_ros(&p),
            Err(ConvertError::InvalidQuaternion(_))
        ));
    }
}

// ---------------------------------------------------------------- planar IMU (strict)

/// Classification of a 3×3 row-major ROS covariance field (REP 145).
#[derive(Debug, Clone, Copy, PartialEq)]
enum Cov9 {
    /// Element 0 is −1: "this quantity is not provided".
    NotProvided,
    /// All zeros: "covariance unknown".
    Unknown,
    /// A usable covariance.
    Known([f64; 9]),
}

fn classify_cov9(c: &[f64; 9], what: &'static str) -> Result<Cov9, ConvertError> {
    if c[0] == -1.0 {
        return Ok(Cov9::NotProvided);
    }
    if c.iter().any(|v| !v.is_finite()) {
        return Err(ConvertError::InvalidCovariance(what));
    }
    if c.iter().all(|v| *v == 0.0) {
        return Ok(Cov9::Unknown);
    }
    let scale = 1.0 + c.iter().fold(0.0f64, |a, v| a.max(v.abs()));
    for (i, j) in [(0, 1), (0, 2), (1, 2)] {
        if (c[i * 3 + j] - c[j * 3 + i]).abs() > 1e-9 * scale {
            return Err(ConvertError::InvalidCovariance(what));
        }
    }
    if c[0] < 0.0 || c[4] < 0.0 || c[8] < 0.0 {
        return Err(ConvertError::InvalidCovariance(what));
    }
    Ok(Cov9::Known(*c))
}

/// `sensor_msgs/Imu` → planar [`ImuMeasurement`] with strict validation.
///
/// Rejected (never silently accepted): zero stamp, NaN/Inf rates or accelerations,
/// malformed covariances, a provided orientation whose quaternion norm is not ≈ 1.
/// `angular_velocity_covariance[8]` (the z-z entry) becomes `yaw_rate_var`; a covariance
/// marked "not provided" (−1) or "unknown" (all zeros) becomes `None`. Linear acceleration
/// is included unless marked "not provided".
pub fn imu_measurement_from_ros(m: &sensor_msgs::msg::Imu) -> Result<ImuMeasurement, ConvertError> {
    let stamp = time_from_ros(&m.header.stamp);
    if stamp == Timestamp::ZERO {
        return Err(ConvertError::InvalidTimestamp);
    }
    let av = &m.angular_velocity;
    if ![av.x, av.y, av.z].iter().all(|v| v.is_finite()) {
        return Err(ConvertError::NonFinite("angular_velocity"));
    }
    let yaw_rate_var = match classify_cov9(
        &m.angular_velocity_covariance,
        "angular_velocity_covariance",
    )? {
        Cov9::Known(c) => Some(c[8]),
        Cov9::NotProvided | Cov9::Unknown => None,
    };
    let (accel, accel_cov) = match classify_cov9(
        &m.linear_acceleration_covariance,
        "linear_acceleration_covariance",
    )? {
        Cov9::NotProvided => (None, None),
        c => {
            let la = &m.linear_acceleration;
            if ![la.x, la.y, la.z].iter().all(|v| v.is_finite()) {
                return Err(ConvertError::NonFinite("linear_acceleration"));
            }
            let cov = match c {
                Cov9::Known(k) => Some(Matrix2::new(k[0], k[1], k[3], k[4])),
                _ => None,
            };
            (Some(Vector2::new(la.x, la.y)), cov)
        }
    };
    if !matches!(
        classify_cov9(&m.orientation_covariance, "orientation_covariance")?,
        Cov9::NotProvided
    ) {
        let q = &m.orientation;
        let all_zero = q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0;
        if !all_zero {
            if ![q.x, q.y, q.z, q.w].iter().all(|v| v.is_finite()) {
                return Err(ConvertError::NonFinite("orientation"));
            }
            let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
            if (norm - 1.0).abs() > 1e-3 {
                return Err(ConvertError::InvalidQuaternion(norm));
            }
        }
    }
    let out = ImuMeasurement {
        stamp,
        frame: FrameId::new(&m.header.frame_id),
        yaw_rate: av.z,
        yaw_rate_var,
        accel,
        accel_cov,
    };
    out.validate()
        .map_err(|_| ConvertError::InvalidCovariance("imu"))?;
    Ok(out)
}

/// Planar [`ImuMeasurement`] → `sensor_msgs/Imu`. Orientation is marked "not provided";
/// acceleration too unless present. Roll/pitch rates are reported as zero with the same
/// variance as the yaw rate.
pub fn imu_measurement_to_ros(m: &ImuMeasurement) -> sensor_msgs::msg::Imu {
    let mut msg = sensor_msgs::msg::Imu {
        header: header(m.stamp, &m.frame),
        ..Default::default()
    };
    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity.z = m.yaw_rate;
    if let Some(v) = m.yaw_rate_var {
        msg.angular_velocity_covariance = [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v];
    }
    match m.accel {
        Some(a) => {
            msg.linear_acceleration.x = a.x;
            msg.linear_acceleration.y = a.y;
            if let Some(c) = m.accel_cov {
                msg.linear_acceleration_covariance = [
                    c[(0, 0)],
                    c[(0, 1)],
                    0.0,
                    c[(1, 0)],
                    c[(1, 1)],
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ];
            }
        }
        None => msg.linear_acceleration_covariance[0] = -1.0,
    }
    msg
}

#[cfg(test)]
mod imu_tests {
    use super::*;

    fn valid() -> sensor_msgs::msg::Imu {
        imu_measurement_to_ros(
            &ImuMeasurement::yaw_rate(
                Timestamp::from_sec_nanosec(5, 0),
                "imu_link",
                0.25,
                Some(4e-4),
            )
            .unwrap(),
        )
    }

    #[test]
    fn round_trip() {
        let m = imu_measurement_from_ros(&valid()).unwrap();
        assert_eq!(m.yaw_rate, 0.25);
        assert_eq!(m.yaw_rate_var, Some(4e-4));
        assert_eq!(m.frame.as_str(), "imu_link");
        assert!(m.accel.is_none());
    }

    #[test]
    fn rejects_corrupt_messages() {
        let mut m = valid();
        m.angular_velocity.z = f64::NAN;
        assert!(matches!(
            imu_measurement_from_ros(&m),
            Err(ConvertError::NonFinite(_))
        ));

        let mut m = valid();
        m.header.stamp = builtin_interfaces::msg::Time { sec: 0, nanosec: 0 };
        assert_eq!(
            imu_measurement_from_ros(&m),
            Err(ConvertError::InvalidTimestamp)
        );

        let mut m = valid();
        m.angular_velocity_covariance[1] = 1.0; // asymmetric
        assert!(matches!(
            imu_measurement_from_ros(&m),
            Err(ConvertError::InvalidCovariance(_))
        ));

        let mut m = valid();
        m.angular_velocity_covariance[8] = -1.0; // negative variance (element 0 is not -1)
        assert!(matches!(
            imu_measurement_from_ros(&m),
            Err(ConvertError::InvalidCovariance(_))
        ));

        let mut m = valid();
        m.orientation_covariance[0] = 0.01;
        m.orientation.w = 0.5; // norm 0.5
        assert!(matches!(
            imu_measurement_from_ros(&m),
            Err(ConvertError::InvalidQuaternion(_))
        ));

        let mut m = valid();
        m.linear_acceleration_covariance[0] = 0.1;
        m.linear_acceleration.x = f64::INFINITY;
        assert!(matches!(
            imu_measurement_from_ros(&m),
            Err(ConvertError::NonFinite(_))
        ));
    }

    #[test]
    fn unknown_covariance_is_none() {
        let mut m = valid();
        m.angular_velocity_covariance = [0.0; 9];
        assert_eq!(imu_measurement_from_ros(&m).unwrap().yaw_rate_var, None);
        m.angular_velocity_covariance[0] = -1.0;
        assert_eq!(imu_measurement_from_ros(&m).unwrap().yaw_rate_var, None);
    }
}
