//! Refusing scans whose timestamp cannot be trusted at the pose's current rate of change.
//!
//! A scan is integrated at the pose TF gives for its header stamp. That is only right if the
//! beams were measured at that stamp, and the Webots LDS-01 driver does not stamp them so: the
//! stamp is the time the scan was published, and the beams were measured up to a full scan
//! period earlier (`docs/phase-4-mapping/implementation.md` section 5.2, item 2). The mapper
//! cannot recover the exact instant, but it can bound it: the measurement lies in the window
//! `[stamp - window, stamp]`. If the pose barely changes across that window, the exact instant
//! does not matter, and the scan is safe at any pose in it. If the pose changes a lot, the
//! scan would be written into the map at a pose that may be wrong by the whole change.
//!
//! The error of integrating at the stamp is therefore at most the change of pose across the
//! window, and [`MotionGate`] caps that change: no more than `max_translation` metres and no
//! more than `max_yaw` radians. For a beam at range `r` the yaw bound moves the hit by at most
//! `r * max_yaw`, so the default 0.015 rad is 5 cm (one cell) at the lidar's 3.5 m limit.
//!
//! The price is scans: none are integrated while the robot turns in place at route speed. The
//! count of scans skipped for motion is reported, never hidden.

use crate::MappingError;
use robotics_core::Se2;
use std::f64::consts::{PI, TAU};

/// Limits on how far the sensor may have moved across a scan's timestamp window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionGate {
    max_translation: f64,
    max_yaw: f64,
}

impl Default for MotionGate {
    /// 5 cm and 0.015 rad. A robot at its 0.2 m/s limit moves 4 cm in a 0.2 s window, so
    /// straight driving always passes; a turn at 0.075 rad/s or more does not.
    fn default() -> Self {
        Self {
            max_translation: 0.05,
            max_yaw: 0.015,
        }
    }
}

impl MotionGate {
    /// A gate with the given limits (metres, radians); both must be finite and positive.
    pub fn new(max_translation: f64, max_yaw: f64) -> Result<Self, MappingError> {
        for (what, v) in [("max_translation", max_translation), ("max_yaw", max_yaw)] {
            if !(v.is_finite() && v > 0.0) {
                return Err(MappingError::InvalidParameter { what, value: v });
            }
        }
        Ok(Self {
            max_translation,
            max_yaw,
        })
    }

    /// The change between two poses: translation (m) and absolute yaw change wrapped to
    /// `[0, pi]` (rad).
    pub fn change(earlier: &Se2, later: &Se2) -> (f64, f64) {
        let dt = ((later.x() - earlier.x()).powi(2) + (later.y() - earlier.y()).powi(2)).sqrt();
        let mut dy = (later.theta() - earlier.theta()) % TAU;
        if dy > PI {
            dy -= TAU;
        } else if dy < -PI {
            dy += TAU;
        }
        (dt, dy.abs())
    }

    /// Whether a scan whose sensor pose was `earlier` at the start of its window and `later`
    /// at its stamp may be integrated. Non-finite poses are not admitted.
    pub fn admits(&self, earlier: &Se2, later: &Se2) -> bool {
        if !(earlier.is_finite() && later.is_finite()) {
            return false;
        }
        let (dt, dy) = Self::change(earlier, later);
        dt <= self.max_translation && dy <= self.max_yaw
    }
}

/// Refusing scans that repeat the last integrated one because the sensor has not moved.
///
/// The log-odds update treats every scan as an independent measurement. Sixty scans from one
/// standing pose are one measurement repeated sixty times: they drive every cell they touch to
/// saturation, including a cell that holds a sensor artefact (the first Webots runs saturated a
/// "flying pixel" at a pillar's silhouette that way, 0.28 m from the pillar), and a saturated
/// cell takes eleven contradicting scans to undo. A motion filter integrates a scan only if the
/// sensor has moved at least `min_translation` metres or turned at least `min_yaw` radians since
/// the last scan that was integrated; it is the usual remedy (Cartographer calls it a motion
/// filter). A robot at its cruise speed of 0.18 m/s moves 3.6 cm per scan, so a threshold of
/// 2 cm never skips a moving scan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionFilter {
    min_translation: f64,
    min_yaw: f64,
}

impl Default for MotionFilter {
    fn default() -> Self {
        Self {
            min_translation: 0.02,
            min_yaw: 0.02,
        }
    }
}

impl MotionFilter {
    /// A filter with the given minimum motion (metres, radians); both must be finite and
    /// positive.
    pub fn new(min_translation: f64, min_yaw: f64) -> Result<Self, MappingError> {
        for (what, v) in [("min_translation", min_translation), ("min_yaw", min_yaw)] {
            if !(v.is_finite() && v > 0.0) {
                return Err(MappingError::InvalidParameter { what, value: v });
            }
        }
        Ok(Self {
            min_translation,
            min_yaw,
        })
    }

    /// Whether a scan taken from `now` is worth integrating, given the pose of the last scan
    /// that was integrated (`None` if there is none yet: the first scan always is).
    pub fn admits(&self, last_integrated: Option<&Se2>, now: &Se2) -> bool {
        match last_integrated {
            None => true,
            Some(last) => {
                let (dt, dy) = MotionGate::change(last, now);
                dt >= self.min_translation || dy >= self.min_yaw
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Worked example, by hand, with the default gate over a 0.2 s window. Driving straight at
    /// the route driver's 0.18 m/s moves 0.036 m: admitted. Turning in place at its 0.8 rad/s
    /// turns 0.16 rad: refused, and so is the 2.0 rad/s stress turn (0.4 rad). A slow drift of
    /// 0.05 rad/s turns 0.01 rad: admitted.
    #[test]
    fn worked_example_the_default_gate_over_a_scan_period() {
        let g = MotionGate::default();
        let a = Se2::new(1.0, 2.0, 0.3);
        assert!(g.admits(
            &a,
            &Se2::new(1.0 + 0.036 * 0.3f64.cos(), 2.0 + 0.036 * 0.3f64.sin(), 0.3)
        ));
        assert!(!g.admits(&a, &Se2::new(1.0, 2.0, 0.3 + 0.8 * 0.2)));
        assert!(!g.admits(&a, &Se2::new(1.0, 2.0, 0.3 + 2.0 * 0.2)));
        assert!(g.admits(&a, &Se2::new(1.0, 2.0, 0.3 + 0.05 * 0.2)));
        assert!(g.admits(&a, &a));
    }

    #[test]
    fn the_limits_are_inclusive_and_checked_separately() {
        let g = MotionGate::new(0.05, 0.015).unwrap();
        let a = Se2::new(0.0, 0.0, 0.0);
        assert!(g.admits(&a, &Se2::new(0.05, 0.0, 0.0)));
        assert!(!g.admits(&a, &Se2::new(0.0501, 0.0, 0.0)));
        assert!(g.admits(&a, &Se2::new(0.0, 0.0, 0.015)));
        assert!(!g.admits(&a, &Se2::new(0.0, 0.0, 0.0151)));
        // Either limit alone refuses the scan.
        assert!(!g.admits(&a, &Se2::new(0.06, 0.0, 0.0)));
        assert!(!g.admits(&a, &Se2::new(0.0, 0.0, 0.02)));
    }

    #[test]
    fn the_yaw_change_wraps_across_pi() {
        let g = MotionGate::default();
        // pi - 0.0015 to -(pi - 0.0015) is a change of 0.003 rad, not 2 pi - 0.003.
        let near_pi = PI - 0.0015;
        assert!(g.admits(&Se2::new(0.0, 0.0, near_pi), &Se2::new(0.0, 0.0, -near_pi)));
        // 3.1 to -3.1 is 0.083 rad.
        assert!(!g.admits(&Se2::new(0.0, 0.0, 3.1), &Se2::new(0.0, 0.0, -3.1)));
        let (_, dy) = MotionGate::change(&Se2::new(0.0, 0.0, -3.0), &Se2::new(0.0, 0.0, 3.0));
        assert!((dy - (TAU - 6.0)).abs() < 1e-12);
    }

    #[test]
    fn a_non_finite_pose_is_never_admitted_and_bad_limits_are_refused() {
        let g = MotionGate::default();
        assert!(!g.admits(&Se2::new(f64::NAN, 0.0, 0.0), &Se2::new(0.0, 0.0, 0.0)));
        assert!(!g.admits(&Se2::new(0.0, 0.0, 0.0), &Se2::new(0.0, f64::INFINITY, 0.0)));
        assert!(MotionGate::new(0.0, 0.1).is_err());
        assert!(MotionGate::new(0.1, -1.0).is_err());
        assert!(MotionGate::new(f64::NAN, 0.1).is_err());
    }

    #[test]
    fn the_motion_filter_skips_a_repeat_and_keeps_every_moving_scan() {
        let f = MotionFilter::default();
        let here = Se2::new(1.0, 1.0, 0.5);
        assert!(f.admits(None, &here), "the first scan is always integrated");
        assert!(!f.admits(Some(&here), &here), "a repeat from the same pose");
        assert!(
            !f.admits(Some(&here), &Se2::new(1.005, 1.0, 0.505)),
            "millimetres of jitter"
        );
        // A robot at 0.18 m/s moves 3.6 cm between scans: always kept.
        assert!(f.admits(Some(&here), &Se2::new(1.036, 1.0, 0.5)));
        // A turn of 0.02 rad is kept, and the wrap across pi is handled.
        assert!(f.admits(Some(&here), &Se2::new(1.0, 1.0, 0.52)));
        assert!(f.admits(
            Some(&Se2::new(0.0, 0.0, PI - 0.005)),
            &Se2::new(0.0, 0.0, -(PI - 0.025))
        ));
        assert!(!f.admits(
            Some(&Se2::new(0.0, 0.0, PI - 0.005)),
            &Se2::new(0.0, 0.0, -(PI - 0.01))
        ));
        assert!(MotionFilter::new(0.0, 0.02).is_err());
        assert!(MotionFilter::new(0.02, f64::NAN).is_err());
    }
}
