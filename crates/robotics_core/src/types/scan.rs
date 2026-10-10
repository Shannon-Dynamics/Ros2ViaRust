//! Planar range-finder scans.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 4 (`Scan{ranges, bearings,
//! max_range}`) and Chapter 10 (which beams a sensor model may use).

use crate::error::CoreError;
use crate::frame::FrameId;
use crate::time::Timestamp;
use crate::types::{Point2, PointCloud2};

/// One sweep of a 2-D LiDAR: `ranges[i]` was measured at bearing
/// `angle_min + i·angle_increment` in the sensor frame.
///
/// The layout mirrors `sensor_msgs/LaserScan` on purpose so the ROS conversion is
/// lossless, but the type is plain data and carries no ROS dependency. A beam is *valid*
/// when its range is finite and inside `[range_min, range_max]`; anything else (NaN,
/// `inf`, `0`) is a dropout or a max-range return and must be skipped by consumers.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LaserScan {
    /// Acquisition time of the first beam.
    pub stamp: Timestamp,
    /// Sensor frame the bearings are expressed in.
    pub frame: FrameId,
    /// Bearing of beam 0 (rad).
    pub angle_min: f64,
    /// Bearing step between consecutive beams (rad).
    pub angle_increment: f64,
    /// Time between beams (s), `0` if unknown.
    pub time_increment: f64,
    /// Time of one full sweep (s), `0` if unknown.
    pub scan_time: f64,
    /// Minimum valid range (m).
    pub range_min: f64,
    /// Maximum valid range (m); returns at or beyond it are max-range readings.
    pub range_max: f64,
    /// Measured ranges (m), one per beam.
    pub ranges: Vec<f32>,
    /// Optional intensities, empty or the same length as `ranges`.
    pub intensities: Vec<f32>,
}

impl LaserScan {
    /// Number of beams.
    #[inline]
    pub fn len(&self) -> usize {
        self.ranges.len()
    }
    /// `true` when the scan has no beams.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.ranges.is_empty()
    }

    /// Bearing of beam `i` (rad, sensor frame).
    #[inline]
    pub fn bearing(&self, i: usize) -> f64 {
        self.angle_min + i as f64 * self.angle_increment
    }

    /// Whether beam `i` carries a usable range.
    #[inline]
    pub fn is_valid(&self, i: usize) -> bool {
        match self.ranges.get(i) {
            Some(&r) => {
                let r = r as f64;
                r.is_finite() && r >= self.range_min && r <= self.range_max
            }
            None => false,
        }
    }

    /// Iterate over `(bearing, range)` for the valid beams only.
    pub fn valid_beams(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        self.ranges
            .iter()
            .enumerate()
            .filter(|(i, _)| self.is_valid(*i))
            .map(move |(i, &r)| (self.bearing(i), r as f64))
    }

    /// Every `stride`-th valid beam, the cheap subsampling sensor models use.
    pub fn valid_beams_strided(&self, stride: usize) -> impl Iterator<Item = (f64, f64)> + '_ {
        let stride = stride.max(1);
        self.valid_beams().step_by(stride)
    }

    /// Valid beam endpoints as points in the sensor frame.
    pub fn to_points(&self) -> PointCloud2 {
        let points = self
            .valid_beams()
            .map(|(b, r)| Point2::new(r * b.cos(), r * b.sin()))
            .collect();
        PointCloud2 {
            stamp: self.stamp,
            frame: self.frame.clone(),
            points,
        }
    }

    /// Bearing of the last beam, `angle_min + (n − 1)·angle_increment` (`angle_min` when
    /// empty). This is what `sensor_msgs/LaserScan.angle_max` carries.
    #[inline]
    pub fn angle_max(&self) -> f64 {
        match self.ranges.len() {
            0 => self.angle_min,
            n => self.bearing(n - 1),
        }
    }

    /// Structural validation used by ROS adapters and the simulator before a scan reaches
    /// an algorithm. Individual ranges may be NaN or ±∞ (REP 117: `+∞` no return, `−∞`
    /// too close, NaN invalid); the *metadata* may not:
    ///
    /// * `angle_min`, `angle_increment`, `range_min`, `range_max`, `time_increment`,
    ///   `scan_time` finite;
    /// * `angle_increment ≠ 0` (negative = clockwise scanner) and the sweep
    ///   `|(n − 1)·angle_increment|` at most one turn plus one increment;
    /// * `0 ≤ range_min < range_max`; `time_increment, scan_time ≥ 0`;
    /// * intensities empty or one per range.
    pub fn validate(&self) -> Result<(), CoreError> {
        for (what, v) in [
            ("angle_min", self.angle_min),
            ("angle_increment", self.angle_increment),
            ("range_min", self.range_min),
            ("range_max", self.range_max),
            ("time_increment", self.time_increment),
            ("scan_time", self.scan_time),
        ] {
            if !v.is_finite() {
                return Err(CoreError::NonFinite(what));
            }
        }
        if self.angle_increment == 0.0 && self.ranges.len() > 1 {
            return Err(CoreError::InvalidValue {
                what: "angle_increment",
                value: 0.0,
            });
        }
        let sweep = (self.ranges.len().saturating_sub(1) as f64 * self.angle_increment).abs();
        if sweep > std::f64::consts::TAU + self.angle_increment.abs() + 1e-9 {
            return Err(CoreError::InvalidValue {
                what: "angular sweep",
                value: sweep,
            });
        }
        if self.range_min < 0.0 {
            return Err(CoreError::InvalidValue {
                what: "range_min",
                value: self.range_min,
            });
        }
        if self.range_max <= self.range_min {
            return Err(CoreError::InvalidValue {
                what: "range_max",
                value: self.range_max,
            });
        }
        if self.time_increment < 0.0 || self.scan_time < 0.0 {
            return Err(CoreError::InvalidValue {
                what: "scan timing",
                value: self.time_increment.min(self.scan_time),
            });
        }
        if !(self.intensities.is_empty() || self.intensities.len() == self.ranges.len()) {
            return Err(CoreError::DimensionMismatch {
                expected: self.ranges.len(),
                got: self.intensities.len(),
            });
        }
        Ok(())
    }

    /// `true` when [`LaserScan::validate`] passes.
    pub fn is_well_formed(&self) -> bool {
        self.validate().is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn scan() -> LaserScan {
        LaserScan {
            angle_min: 0.0,
            angle_increment: FRAC_PI_2,
            range_min: 0.1,
            range_max: 5.0,
            ranges: vec![1.0, f32::NAN, 2.0, 6.0, f32::INFINITY],
            ..Default::default()
        }
    }

    #[test]
    fn filters_invalid_beams() {
        let s = scan();
        assert!(s.is_well_formed());
        let beams: Vec<_> = s.valid_beams().collect();
        assert_eq!(beams.len(), 2);
        assert!((beams[0].0 - 0.0).abs() < 1e-12 && (beams[0].1 - 1.0).abs() < 1e-12);
        assert!((beams[1].0 - 2.0 * FRAC_PI_2).abs() < 1e-12 && (beams[1].1 - 2.0).abs() < 1e-12);
        assert_eq!(s.valid_beams_strided(2).count(), 1);
    }

    #[test]
    fn validation_rejects_malformed_metadata() {
        let good = scan();
        assert!(good.validate().is_ok());
        assert!((good.angle_max() - 4.0 * FRAC_PI_2).abs() < 1e-12);

        let mut s = scan();
        s.angle_increment = 0.0;
        assert!(s.validate().is_err());
        let mut s = scan();
        s.angle_min = f64::NAN;
        assert_eq!(s.validate(), Err(CoreError::NonFinite("angle_min")));
        let mut s = scan();
        s.range_min = -0.1;
        assert!(s.validate().is_err());
        let mut s = scan();
        s.range_max = s.range_min;
        assert!(s.validate().is_err());
        let mut s = scan();
        s.intensities = vec![1.0];
        assert!(matches!(
            s.validate(),
            Err(CoreError::DimensionMismatch {
                expected: 5,
                got: 1
            })
        ));
        let mut s = scan();
        s.angle_increment = 3.0; // 4 steps × 3 rad = 12 rad: more than one turn
        assert!(s.validate().is_err());
        let mut s = scan();
        s.angle_increment = -FRAC_PI_2; // clockwise scanners are fine
        assert!(s.validate().is_ok());
        let mut s = scan();
        s.scan_time = -1.0;
        assert!(s.validate().is_err());
    }

    #[test]
    fn endpoints_in_sensor_frame() {
        let pc = scan().to_points();
        assert_eq!(pc.len(), 2);
        assert!((pc.points[0].x - 1.0).abs() < 1e-12 && pc.points[0].y.abs() < 1e-12);
        assert!((pc.points[1].x + 2.0).abs() < 1e-12 && pc.points[1].y.abs() < 1e-12);
    }
}
