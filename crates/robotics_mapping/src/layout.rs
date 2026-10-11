//! The bearings a scan's beams are taken to point at.
//!
//! A [`LaserScan`] says that beam `i` points at `angle_min + i * angle_increment`. For the
//! Webots LDS-01 that is not what the simulator measured, and the two differ by up to half a
//! degree. Webots renders a 360 degree lidar as a cylindrical projection and gives output column
//! `j` of `N` the bearing `(0.5 - (j + 0.5) / N) * fov` (`resources/wren/shaders/merge_spherical.frag`,
//! line 158, tag R2025a): the centres of `N` equal bins, `fov / N` apart, the first at `fov/2 - fov/(2N)`.
//! The official `webots_ros2_driver` (2025.0.0, `Ros2Lidar.cpp`) publishes `angle_min = fov/2` and
//! `angle_increment = -fov / (N - 1)`, the grid of `N` points that include both ends of the field
//! of view. The error is `j * (fov/(N-1) - fov/N) - fov/(2N)`: -8.7 mrad at beam 0, zero near beam
//! 179, +8.7 mrad at beam 359 for the 360-beam, 2 pi sensor. A beam's range error at a wall is
//! `range * tan(incidence) * error`, so it vanishes at normal incidence and reaches 5 cm at 3 m and
//! 45 degrees, which is how it showed up as cells outside the wall on some routes and not on others
//! (`evidence/phase-4-mapping/2026-10-11-evaluation-group/NOTE-root-cause-analysis.md`).
//!
//! [`BeamLayout::Published`] takes the scan as it is (the default: a real driver is not touched);
//! [`BeamLayout::PixelCentres`] replaces the bearings by the pixel-centre grid. It is applied to the
//! scan before integration and changes nothing else about it.
//!
//! This is not the whole beam error: the fitted error at the two ends of the Webots scan is about
//! twice this correction, and the rest has no explanation yet (`NOTE-stage-B-gate.md`).

use crate::MappingError;
use robotics_core::LaserScan;
use std::borrow::Cow;

/// How far from symmetry (rad) a scan may be for [`BeamLayout::PixelCentres`]: the middle of the
/// published bearings must be on the sensor's x axis. The Webots scan is symmetric to 1e-7 (its
/// bearings are single precision on the wire).
pub const SYMMETRY_TOLERANCE: f64 = 1e-4;

/// Which bearings the beams of a scan are taken to have.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BeamLayout {
    /// `angle_min + i * angle_increment`, as published.
    #[default]
    Published,
    /// The centres of `N` equal bins over the field of view the scan publishes, as Webots'
    /// cylindrical projection places them: `increment' = increment * (N - 1) / N` and
    /// `angle_min' = angle_min + increment' / 2`.
    PixelCentres,
}

impl BeamLayout {
    /// The name used in parameters and records: `published` or `pixel_centres`.
    pub fn name(self) -> &'static str {
        match self {
            BeamLayout::Published => "published",
            BeamLayout::PixelCentres => "pixel_centres",
        }
    }

    /// Parse a name written by [`BeamLayout::name`].
    pub fn parse(name: &str) -> Result<Self, MappingError> {
        match name {
            "published" => Ok(BeamLayout::Published),
            "pixel_centres" => Ok(BeamLayout::PixelCentres),
            other => Err(MappingError::Parse(format!(
                "unknown beam layout {other:?}, expected published or pixel_centres"
            ))),
        }
    }

    /// The scan with this layout's bearings. [`BeamLayout::Published`] borrows the scan unchanged;
    /// [`BeamLayout::PixelCentres`] copies it with `angle_min` and `angle_increment` replaced.
    ///
    /// Fails for [`BeamLayout::PixelCentres`] if the scan has fewer than two beams, a non-finite or
    /// zero increment, or bearings that are not symmetric about the sensor's x axis (the
    /// correction is derived for a full cylindrical field of view centred on the axis).
    pub fn apply(self, scan: &LaserScan) -> Result<Cow<'_, LaserScan>, MappingError> {
        match self {
            BeamLayout::Published => Ok(Cow::Borrowed(scan)),
            BeamLayout::PixelCentres => {
                let n = scan.ranges.len();
                if n < 2 {
                    return Err(MappingError::MalformedScan(
                        "pixel_centres layout needs at least two beams",
                    ));
                }
                if !(scan.angle_min.is_finite()
                    && scan.angle_increment.is_finite()
                    && scan.angle_increment != 0.0)
                {
                    return Err(MappingError::MalformedScan(
                        "pixel_centres layout needs a finite, non-zero bearing step",
                    ));
                }
                let n = n as f64;
                // First bearing to last, signed: minus the field of view for the Webots scan.
                let span = (n - 1.0) * scan.angle_increment;
                if (scan.angle_min + 0.5 * span).abs() > SYMMETRY_TOLERANCE {
                    return Err(MappingError::MalformedScan(
                        "pixel_centres layout needs bearings symmetric about the sensor's x axis",
                    ));
                }
                let mut out = scan.clone();
                out.angle_increment = span / n;
                out.angle_min = scan.angle_min + 0.5 * out.angle_increment;
                Ok(Cow::Owned(out))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use robotics_core::{FrameId, Timestamp};

    /// `fieldOfView` of `RobotisLds01.proto`: the value Webots declares, 2 pi to five decimals.
    #[allow(clippy::approx_constant)]
    const WEBOTS_FOV: f64 = 6.28318;

    /// The LDS-01 as `webots_ros2_driver` publishes it: 360 beams.
    fn webots_scan() -> LaserScan {
        let (n, fov) = (360usize, WEBOTS_FOV);
        LaserScan {
            stamp: Timestamp::from_secs_f64(12.5),
            frame: FrameId::new("base_scan"),
            angle_min: fov / 2.0,
            angle_increment: -fov / (n as f64 - 1.0),
            time_increment: 0.0,
            scan_time: 0.0,
            range_min: 0.12,
            range_max: 3.5,
            ranges: (0..n).map(|i| 1.0 + i as f32 * 0.001).collect(),
            intensities: Vec::new(),
        }
    }

    #[test]
    fn published_borrows_the_scan_untouched() {
        let s = webots_scan();
        assert!(matches!(
            BeamLayout::Published.apply(&s).unwrap(),
            Cow::Borrowed(_)
        ));
        assert_eq!(BeamLayout::default(), BeamLayout::Published);
    }

    /// Worked example: the Webots scan. Beam 0 moves from +180.000 to +179.500 degrees, beam 359
    /// from -180.000 to -179.500, and the step becomes exactly `fov / 360`.
    #[test]
    fn pixel_centres_puts_the_webots_beams_at_the_centres_of_360_bins() {
        let s = webots_scan();
        let fov = WEBOTS_FOV;
        let c = BeamLayout::PixelCentres.apply(&s).unwrap();
        assert_eq!(c.ranges, s.ranges, "ranges are not touched");
        assert_eq!((c.stamp, c.range_min, c.range_max), (s.stamp, 0.12, 3.5));
        assert!((c.angle_increment + fov / 360.0).abs() < 1e-12);
        // `fieldOfView` is 6.28318, which is 2 pi to 3e-4 degrees, hence the tolerance.
        assert!((c.bearing(0).to_degrees() - (180.0 - 0.5)).abs() < 5e-4);
        assert!((c.bearing(359).to_degrees() + (180.0 - 0.5)).abs() < 5e-4);
        assert!((c.bearing(0) + c.bearing(359)).abs() < 1e-9, "symmetric");
    }

    /// The correction equals the closed form `j (fov/(N-1) - fov/N) - fov/(2N)` beam by beam.
    #[test]
    fn the_correction_is_the_closed_form_of_the_mismatch() {
        let s = webots_scan();
        let (n, fov) = (360.0_f64, WEBOTS_FOV);
        let c = BeamLayout::PixelCentres.apply(&s).unwrap();
        for j in 0..360usize {
            let want = j as f64 * (fov / (n - 1.0) - fov / n) - 0.5 * fov / n;
            let got = c.bearing(j) - s.bearing(j);
            assert!((got - want).abs() < 1e-9, "beam {j}: {got} against {want}");
        }
        let (b0, b359) = (c.bearing(0) - s.bearing(0), c.bearing(359) - s.bearing(359));
        assert!((b0 + 8.7266e-3).abs() < 1e-6 && (b359 - 8.7266e-3).abs() < 1e-6);
    }

    #[test]
    fn single_precision_bearings_on_the_wire_are_still_symmetric() {
        let mut s = webots_scan();
        s.angle_min = f64::from(s.angle_min as f32);
        s.angle_increment = f64::from(s.angle_increment as f32);
        assert!(BeamLayout::PixelCentres.apply(&s).is_ok());
    }

    #[test]
    fn scans_the_correction_is_not_derived_for_are_refused() {
        let mut s = webots_scan();
        s.angle_min = -0.1; // a sector that is not centred on the axis
        s.angle_increment = 0.005;
        assert!(BeamLayout::PixelCentres.apply(&s).is_err());
        let mut one = webots_scan();
        one.ranges = vec![1.0];
        assert!(BeamLayout::PixelCentres.apply(&one).is_err());
        let mut flat = webots_scan();
        flat.angle_increment = 0.0;
        assert!(BeamLayout::PixelCentres.apply(&flat).is_err());
        let mut nan = webots_scan();
        nan.angle_min = f64::NAN;
        assert!(BeamLayout::PixelCentres.apply(&nan).is_err());
        // refusing does not touch the published layout
        assert!(BeamLayout::Published.apply(&s).is_ok());
    }

    #[test]
    fn names_round_trip_and_unknown_names_are_errors() {
        for l in [BeamLayout::Published, BeamLayout::PixelCentres] {
            assert_eq!(BeamLayout::parse(l.name()).unwrap(), l);
        }
        assert!(BeamLayout::parse("pixel-centres").is_err());
        assert!(BeamLayout::parse("").is_err());
    }

    proptest! {
        /// For any symmetric published grid the corrected bearings are symmetric and evenly spaced
        /// by `fov / N`, and the corrected span is `fov (N - 1) / N`.
        #[test]
        fn corrected_bearings_are_symmetric_and_evenly_spaced(n in 2usize..800, fov in 0.1f64..6.3) {
            let s = LaserScan {
                angle_min: fov / 2.0,
                angle_increment: -fov / (n as f64 - 1.0),
                ranges: vec![1.0; n],
                range_min: 0.1,
                range_max: 3.5,
                ..LaserScan::default()
            };
            let c = BeamLayout::PixelCentres.apply(&s).unwrap();
            prop_assert!((c.angle_increment + fov / n as f64).abs() < 1e-12);
            for j in 0..n {
                prop_assert!((c.bearing(j) + c.bearing(n - 1 - j)).abs() < 1e-9);
            }
            prop_assert!((c.bearing(0) - c.bearing(n - 1) - fov * (n as f64 - 1.0) / n as f64).abs() < 1e-9);
        }
    }
}
