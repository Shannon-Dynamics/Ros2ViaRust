//! Inertial measurements.

use crate::error::CoreError;
use crate::frame::FrameId;
use crate::time::Timestamp;
use nalgebra::{Matrix2, Matrix3, UnitQuaternion, Vector2, Vector3};

/// One IMU sample, the internal twin of `sensor_msgs/Imu`.
///
/// Accelerations are *specific force* (gravity included), as an IMU reports them.
/// Fields the device does not provide are `None` rather than zero so estimators can tell
/// "unknown" from "measured zero".
#[derive(Debug, Clone, PartialEq)]
pub struct ImuSample {
    /// Sample time.
    pub stamp: Timestamp,
    /// Sensor frame.
    pub frame: FrameId,
    /// Angular velocity (rad/s), body frame.
    pub angular_velocity: Vector3<f64>,
    /// Covariance of `angular_velocity`, if reported.
    pub angular_velocity_cov: Option<Matrix3<f64>>,
    /// Specific force (m/s²), body frame.
    pub linear_acceleration: Vector3<f64>,
    /// Covariance of `linear_acceleration`, if reported.
    pub linear_acceleration_cov: Option<Matrix3<f64>>,
    /// Orientation estimate from the device's own fusion, if it provides one.
    pub orientation: Option<UnitQuaternion<f64>>,
    /// Covariance of the orientation (roll, pitch, yaw), if reported.
    pub orientation_cov: Option<Matrix3<f64>>,
}

impl ImuSample {
    /// Yaw rate (rad/s), the component a planar estimator uses.
    #[inline]
    pub fn yaw_rate(&self) -> f64 {
        self.angular_velocity.z
    }
}

/// The planar subset of an IMU sample that the Phase 2 estimators consume.
///
/// Conventions (REP 103 / REP 145):
/// * **Frame**: `frame` is the IMU's frame. The estimator assumes it is aligned with
///   `base_link` (z up, x forward); a mounting rotation must be removed upstream until a
///   TF buffer exists (rclrs 0.8 has none).
/// * **Axes / sign**: `yaw_rate` is the angular velocity about body `+z`, counter-clockwise
///   positive, in rad/s. `accel` is the specific force along body `x`/`y` in m/s²
///   (gravity-free in the plane only if the IMU is level).
/// * **Time**: `stamp` is the sample (measurement) time, not the arrival time; a zero
///   stamp is invalid.
/// * **Covariance**: `yaw_rate_var` is the per-sample variance in (rad/s)². `None` means
///   "not reported" (ROS convention: covariance element 0 = −1 or all zeros); consumers then
///   use their configured noise.
#[derive(Debug, Clone, PartialEq)]
pub struct ImuMeasurement {
    /// Sample time.
    pub stamp: Timestamp,
    /// IMU frame.
    pub frame: FrameId,
    /// Yaw rate about body +z (rad/s).
    pub yaw_rate: f64,
    /// Variance of `yaw_rate`, if reported ((rad/s)²).
    pub yaw_rate_var: Option<f64>,
    /// Planar specific force (m/s²), if provided.
    pub accel: Option<Vector2<f64>>,
    /// Covariance of `accel`, if reported ((m/s²)²).
    pub accel_cov: Option<Matrix2<f64>>,
}

impl ImuMeasurement {
    /// A yaw-rate-only measurement, validated.
    pub fn yaw_rate(
        stamp: Timestamp,
        frame: impl Into<FrameId>,
        yaw_rate: f64,
        yaw_rate_var: Option<f64>,
    ) -> Result<Self, CoreError> {
        let m = Self {
            stamp,
            frame: frame.into(),
            yaw_rate,
            yaw_rate_var,
            accel: None,
            accel_cov: None,
        };
        m.validate()?;
        Ok(m)
    }

    /// Reject non-finite values, negative variances, non-symmetric/negative accel
    /// covariance, and a zero timestamp.
    pub fn validate(&self) -> Result<(), CoreError> {
        if self.stamp == Timestamp::ZERO {
            return Err(CoreError::InvalidValue {
                what: "imu stamp",
                value: 0.0,
            });
        }
        if !self.yaw_rate.is_finite() {
            return Err(CoreError::NonFinite("imu yaw rate"));
        }
        if let Some(v) = self.yaw_rate_var {
            if !v.is_finite() || v < 0.0 {
                return Err(CoreError::InvalidValue {
                    what: "imu yaw rate variance",
                    value: v,
                });
            }
        }
        if let Some(a) = self.accel {
            if a.iter().any(|v| !v.is_finite()) {
                return Err(CoreError::NonFinite("imu acceleration"));
            }
        }
        if let Some(c) = self.accel_cov {
            if c.iter().any(|v| !v.is_finite()) {
                return Err(CoreError::NonFinite("imu acceleration covariance"));
            }
            if (c[(0, 1)] - c[(1, 0)]).abs() > 1e-9 * (1.0 + c.abs().max())
                || c[(0, 0)] < 0.0
                || c[(1, 1)] < 0.0
                || c[(0, 0)] * c[(1, 1)] < c[(0, 1)] * c[(1, 0)]
            {
                return Err(CoreError::NotPositiveDefinite);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validation_rejects_corrupt_samples() {
        let t = Timestamp::from_secs_f64(1.0);
        assert!(ImuMeasurement::yaw_rate(t, "imu", 0.1, Some(1e-4)).is_ok());
        assert!(ImuMeasurement::yaw_rate(t, "imu", f64::NAN, None).is_err());
        assert!(ImuMeasurement::yaw_rate(t, "imu", f64::INFINITY, None).is_err());
        assert!(ImuMeasurement::yaw_rate(t, "imu", 0.1, Some(-1.0)).is_err());
        assert!(ImuMeasurement::yaw_rate(Timestamp::ZERO, "imu", 0.1, None).is_err());
        let mut m = ImuMeasurement::yaw_rate(t, "imu", 0.1, None).unwrap();
        m.accel_cov = Some(Matrix2::new(1.0, 2.0, 2.0, 1.0));
        assert!(m.validate().is_err());
        m.accel_cov = Some(Matrix2::new(1.0, 0.1, 0.1, 1.0));
        m.accel = Some(Vector2::new(0.0, f64::NAN));
        assert!(m.validate().is_err());
    }
}
