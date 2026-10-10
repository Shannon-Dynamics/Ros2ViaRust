//! Turning a laser scan into range and bearing observations of the pillars.
//!
//! The map of the Phase 3 room is four cylindrical pillars of known radius standing in an
//! otherwise empty rectangular room. In a scan they look like short runs of neighbouring
//! beams at a similar range, separated from everything else by a jump in range or by
//! beams with no return. The walls look like long runs. That difference is the whole
//! detector:
//!
//! 1. split the scan into clusters of consecutive beams whose range does not jump by more
//!    than `cluster_gap`, treating the scan as circular;
//! 2. drop clusters whose chord (the distance between the first and last point) is longer
//!    than `max_chord`, which is what removes the walls;
//! 3. report the remaining clusters as a range and a bearing to the pillar *centre*, by
//!    taking the centroid of the points and pushing it `radius` further away along its own
//!    bearing, because the laser sees the near surface and the map holds the centre.
//!
//! The output is in the sensor frame. Turning it into the body frame is the job of
//! [`crate::ekf`], through `LandmarkModel::with_sensor_pose`.
//!
//! What this is not: a general feature extractor. It assumes free standing round pillars
//! and straight walls, which is exactly what the Phase 3 world has, and it is replaced by
//! a map based likelihood field once Phase 4 provides an occupancy grid.

use robotics_core::types::LaserScan;
use robotics_models::sensor::landmark::RangeBearing;

/// Tuning of [`detect`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PillarDetector {
    /// Ignore returns beyond this range (m); far clusters are a few beams wide and carry
    /// little information while being easy to confuse with each other.
    pub max_range: f64,
    /// Range jump between neighbouring beams that starts a new cluster (m).
    pub cluster_gap: f64,
    /// Smallest number of beams a cluster must have.
    pub min_points: usize,
    /// Longest chord a cluster may have to still be a pillar (m).
    pub max_chord: f64,
    /// Radius of the pillars (m), added to the measured surface distance.
    pub radius: f64,
}

impl Default for PillarDetector {
    fn default() -> Self {
        Self {
            max_range: 3.2,
            cluster_gap: 0.12,
            min_points: 2,
            max_chord: 0.60,
            radius: 0.15,
        }
    }
}

/// One accepted cluster, kept for diagnostics as well as for the observation itself.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detection {
    /// Range and bearing of the pillar centre, in the sensor frame.
    pub z: RangeBearing,
    /// Number of beams in the cluster.
    pub beams: usize,
    /// Chord length of the cluster (m).
    pub chord: f64,
}

/// Extract pillar observations from one scan.
pub fn detect(scan: &LaserScan, cfg: &PillarDetector) -> Vec<Detection> {
    let n = scan.ranges.len();
    if n == 0 || !cfg.max_range.is_finite() || cfg.max_range <= 0.0 {
        return Vec::new();
    }
    let bearing = |i: usize| scan.angle_min + (i as f64) * scan.angle_increment;
    let range = |i: usize| -> Option<f64> {
        let r = scan.ranges[i] as f64;
        (r.is_finite() && r >= scan.range_min.max(1e-3) && r <= scan.range_max.min(cfg.max_range))
            .then_some(r)
    };

    // Clusters of consecutive valid beams without a range jump, as index runs.
    let mut runs: Vec<Vec<usize>> = Vec::new();
    let mut current: Vec<usize> = Vec::new();
    let mut previous: Option<(usize, f64)> = None;
    for i in 0..n {
        match range(i) {
            Some(r) => {
                let continues = previous
                    .map(|(j, pr)| j + 1 == i && (r - pr).abs() <= cfg.cluster_gap)
                    .unwrap_or(false);
                if !continues && !current.is_empty() {
                    runs.push(std::mem::take(&mut current));
                }
                current.push(i);
                previous = Some((i, r));
            }
            None => {
                if !current.is_empty() {
                    runs.push(std::mem::take(&mut current));
                }
                previous = None;
            }
        }
    }
    if !current.is_empty() {
        runs.push(current);
    }

    // The scan is a full circle, so a cluster may straddle the seam between the last and
    // the first beam. Merge the two ends when they are themselves continuous.
    if runs.len() > 1 {
        let first_is_start = runs.first().map(|r| r[0] == 0).unwrap_or(false);
        let last_is_end = runs
            .last()
            .map(|r| *r.last().unwrap() == n - 1)
            .unwrap_or(false);
        let spans_full_circle = (scan.angle_increment.abs() * n as f64 - std::f64::consts::TAU)
            .abs()
            < scan.angle_increment.abs();
        if first_is_start && last_is_end && spans_full_circle {
            let (a, b) = (
                range(0).unwrap_or(f64::INFINITY),
                range(n - 1).unwrap_or(f64::INFINITY),
            );
            if (a - b).abs() <= cfg.cluster_gap {
                let mut tail = runs.pop().unwrap();
                tail.extend(runs.remove(0));
                runs.insert(0, tail);
            }
        }
    }

    let mut out = Vec::new();
    for run in runs {
        if run.len() < cfg.min_points {
            continue;
        }
        let point = |i: usize| {
            let r = scan.ranges[i] as f64;
            let b = bearing(i);
            (r * b.cos(), r * b.sin())
        };
        let (fx, fy) = point(run[0]);
        let (lx, ly) = point(*run.last().unwrap());
        let chord = ((lx - fx).powi(2) + (ly - fy).powi(2)).sqrt();
        if chord > cfg.max_chord {
            continue; // a wall, or two things the detector cannot separate
        }
        let (mut cx, mut cy) = (0.0, 0.0);
        for &i in &run {
            let (x, y) = point(i);
            cx += x;
            cy += y;
        }
        cx /= run.len() as f64;
        cy /= run.len() as f64;
        let surface = (cx * cx + cy * cy).sqrt();
        if surface <= 0.0 {
            continue;
        }
        out.push(Detection {
            z: RangeBearing {
                range: surface + cfg.radius,
                bearing: cy.atan2(cx),
            },
            beams: run.len(),
            chord,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use robotics_core::{FrameId, Timestamp};
    use std::f64::consts::{PI, TAU};

    /// A 360 beam scan with `inf` everywhere, as the Webots LDS-01 reports no return.
    fn empty_scan(n: usize) -> LaserScan {
        LaserScan {
            stamp: Timestamp::from_secs_f64(0.0),
            frame: FrameId::new("laser"),
            angle_min: -PI,
            angle_increment: TAU / n as f64,
            time_increment: 0.0,
            scan_time: 0.2,
            range_min: 0.12,
            range_max: 3.5,
            ranges: vec![f32::INFINITY; n],
            intensities: Vec::new(),
        }
    }

    fn put(scan: &mut LaserScan, from: usize, ranges: &[f32]) {
        let n = scan.ranges.len();
        for (k, r) in ranges.iter().enumerate() {
            scan.ranges[(from + k) % n] = *r;
        }
    }

    #[test]
    fn finds_a_single_pillar_and_reports_its_centre() {
        let mut scan = empty_scan(360);
        // Five beams around index 180 (bearing 0) hitting a surface 2.00 m away.
        put(&mut scan, 178, &[2.01, 2.00, 2.00, 2.00, 2.01]);
        let d = detect(&scan, &PillarDetector::default());
        assert_eq!(d.len(), 1, "exactly one pillar");
        assert!(
            (d[0].z.bearing).abs() < 0.02,
            "bearing {} should be near zero",
            d[0].z.bearing
        );
        // Surface at 2.00 m plus the 0.15 m radius gives the centre.
        assert!(
            (d[0].z.range - 2.15).abs() < 0.02,
            "range {} should be near 2.15",
            d[0].z.range
        );
        assert_eq!(d[0].beams, 5);
    }

    #[test]
    fn rejects_a_wall() {
        let mut scan = empty_scan(360);
        // A long, smooth run of returns: a wall, not a pillar.
        let wall: Vec<f32> = (0..80).map(|k| 3.0 + 0.001 * k as f32).collect();
        put(&mut scan, 100, &wall);
        assert!(detect(&scan, &PillarDetector::default()).is_empty());
    }

    #[test]
    fn separates_two_pillars() {
        let mut scan = empty_scan(360);
        put(&mut scan, 100, &[2.0, 2.0, 2.0]);
        put(&mut scan, 200, &[1.5, 1.5, 1.5]);
        let d = detect(&scan, &PillarDetector::default());
        assert_eq!(d.len(), 2);
    }

    #[test]
    fn a_cluster_across_the_seam_is_one_detection() {
        let mut scan = empty_scan(360);
        put(&mut scan, 358, &[2.0, 2.0, 2.0, 2.0]); // 358, 359, 0, 1
        let d = detect(&scan, &PillarDetector::default());
        assert_eq!(d.len(), 1, "the seam must not split a pillar in two");
        assert_eq!(d[0].beams, 4);
    }

    #[test]
    fn ignores_returns_beyond_max_range() {
        let mut scan = empty_scan(360);
        put(&mut scan, 10, &[3.4, 3.4, 3.4]);
        let cfg = PillarDetector {
            max_range: 3.2,
            ..Default::default()
        };
        assert!(detect(&scan, &cfg).is_empty());
    }
}
