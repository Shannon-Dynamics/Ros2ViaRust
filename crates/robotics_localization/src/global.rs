//! Global localization from a uniform prior: the pieces that are pure logic.
//!
//! This is the P2 problem of `docs/phase-3-localization/problem.md` in its starting form,
//! the robot has **no** estimate of where it is at all, as opposed to the kidnapped robot,
//! where a confident belief is invalidated and the filter has to notice. The two share the
//! weighting and resampling machinery of [`crate::mcl`] and nothing else, and they are kept
//! apart on purpose:
//!
//! | | kidnap recovery | global localization |
//! |---|---|---|
//! | belief at the start | tight Gaussian at the known pose | uniform over the free space |
//! | what triggers the work | a sustained low-fit streak, judged by the filter | nothing: the cloud starts out as wide as it can be |
//! | what the cloud is replaced with | a uniform reset, at the steady-state size | not replaced, it narrows by itself |
//! | how it ends | the streak breaks | [`ConvergenceDetector`] declares it |
//! | code | `McLocalizer::inject_random_particles` | everything in this file and `McLocalizer::new_uniform_prior` |
//!
//! While a filter is in [`LocalizationPhase::GlobalSearch`] its kidnap trigger is inert. It
//! has to be: the trigger asks "does anything in the cloud explain this scan", and at the
//! start of a search the honest answer is "very little, yet", so leaving it armed would
//! reset a cloud that is doing exactly what it should. Once the search has converged the
//! filter becomes an ordinary tracking filter and the trigger is armed again with the
//! unchanged kidnap-recovery tuning.
//!
//! # What "free space" means here
//!
//! There is no occupancy grid yet (Phase 4), so there are no free-space cells to enumerate.
//! The free space of this map is the room, a square of half side
//! [`crate::mcl::MclConfig::room_half_extent`], minus a disc of radius
//! [`GlobalLocalizationConfig::landmark_keepout_m`] around every pillar, and a prior over
//! it is a uniform draw of `(x, y)` over that region and of the heading over `(-pi, pi]`.
//! It is the continuous limit of "uniform over the free cells", and it is no coarser than
//! the grid would be.
//!
//! # The convergence criterion
//!
//! Nothing outside the filter says when a search has succeeded, so the filter has to say
//! it, from its own cloud, and the statement is a claim that can be wrong (a cloud can
//! collapse onto a plausible wrong hypothesis). Whether it was right is what the
//! experiments grade against ground truth; this file only decides when to claim it.
//!
//! The claim is made on the first scan at which all of the following have held on
//! [`GlobalLocalizationConfig::convergence_sustained_scans`] consecutive scored scans:
//!
//! 1. `sigma_xy < convergence_sigma_xy_m`, where `sigma_xy = sqrt(var(x) + var(y))` is the
//!    root-mean-square distance of the weighted cloud from its weighted mean (the square
//!    root of the trace of the position covariance, so a cloud stretched along a ring is
//!    not mistaken for a tight one by looking along one axis only);
//! 2. `sigma_theta < convergence_sigma_theta_rad`, where `sigma_theta = sqrt(-2 ln R)` and
//!    `R` is the mean resultant length of the weighted headings, the circular standard
//!    deviation. A linear deviation about a circular mean is wrong for a cloud whose
//!    headings straddle `+-pi`, or that holds two opposite headings; this one is not;
//! 3. the weighted-mean pose explains the scan: at least
//!    [`GlobalLocalizationConfig::convergence_min_explained_fraction`] of the scan's
//!    detections (rounded up) have a landmark that fits them at the weighted-mean pose, a
//!    fit being a per-observation log likelihood of at least
//!    [`crate::mcl::McLocalizer::fit_log_threshold`], the same threshold the kidnap trigger
//!    uses. A cloud that is tight but whose mean does not fit what the laser sees has
//!    converged on something the sensor disagrees with. The default of `0.5` is the
//!    upper-median rule the kidnap trigger uses (one of two detections is enough); it is
//!    lenient on purpose and the first experiments showed what that costs, see
//!    `docs/phase-3-localization/benchmark.md`.
//!
//! The two spread limits are `0.05` in the units of each, chosen as *one third of the
//! tolerance the experiments grade success against* (`0.15 m`, `0.15 rad`, the thresholds of
//! `scripts/phase3_kidnap_verdict.py`), so that a cloud that has met them has, to three
//! standard deviations, a mean inside the tolerance. They are not tuned against the global
//! localization results. For scale, the steady-state tracking spread of the 6000-particle
//! filter on the 51 m route is far below `0.05` between turns: read the `est_sigma_xy_m`
//! column of
//! `evidence/phase-3-correctness/2026-10-10T144153-final-global-reset/normal-run-mcl.csv`.
//!
//! That same spread is not steady, though. It rises by an order of magnitude for a few
//! seconds around each in-place turn while the pose error stays within a few centimetres (the
//! same file, `est_sigma_xy_m` against `position_error_m`). A converged filter therefore
//! falls out of the criterion at corners and a declaration can be delayed to the next
//! straight leg. That delay is a property of this criterion and is measured in the global
//! localization results, not corrected for here.

use rand::{Rng, RngExt};
use robotics_core::Se2;
use robotics_models::sensor::landmark::LandmarkMap;
use std::f64::consts::PI;

/// Which regime a filter is in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LocalizationPhase {
    /// A belief the filter already trusts: the start was known, or a global search has
    /// converged. The kidnap trigger of [`crate::mcl`] is armed.
    #[default]
    Tracking,
    /// A global search from a uniform prior that has not converged yet. The kidnap trigger
    /// is inert.
    GlobalSearch,
}

/// Configuration of a global search.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalLocalizationConfig {
    /// Particles drawn for the search. Independent of
    /// [`crate::mcl::MclConfig::num_particles`], the steady-state count the filter shrinks
    /// to on convergence; making this larger than that is how a search is given more
    /// coverage without changing what tracking and kidnap recovery cost. If it is not
    /// larger, the filter keeps this many.
    pub num_particles: usize,
    /// Radius around every landmark inside which no particle is drawn (m): the pillar radius
    /// plus the robot's footprint radius, since a pose inside it is a pose the robot cannot
    /// be in. The pillars here are `0.15 m` and the Burger's footprint about `0.11 m`.
    pub landmark_keepout_m: f64,
    /// Convergence limit on [`Spread::sigma_xy`] (m); see the module documentation.
    pub convergence_sigma_xy_m: f64,
    /// Convergence limit on [`Spread::sigma_theta`] (rad); see the module documentation.
    pub convergence_sigma_theta_rad: f64,
    /// Consecutive scored scans for which the criterion has to hold. Ten is two seconds at
    /// the LDS-01's 5 Hz, long enough that a single lucky scan cannot declare convergence
    /// and short enough that one straight leg of a route can.
    pub convergence_sustained_scans: u32,
    /// Fraction of a scan's detections the weighted-mean pose has to explain, in `(0, 1]`;
    /// the count needed is `ceil(fraction * detections)`. `0.5` is the upper-median rule
    /// (one of two, two of three, two of four); anything above `0.5` and up to `1.0`
    /// tightens it towards "every detection". See the module documentation.
    pub convergence_min_explained_fraction: f64,
}

impl Default for GlobalLocalizationConfig {
    fn default() -> Self {
        Self {
            num_particles: 6000,
            landmark_keepout_m: 0.26,
            convergence_sigma_xy_m: 0.05,
            convergence_sigma_theta_rad: 0.05,
            convergence_sustained_scans: 10,
            convergence_min_explained_fraction: 0.5,
        }
    }
}

impl GlobalLocalizationConfig {
    /// Detections out of `observed` that must be explained for a scan to count.
    pub fn explained_needed(&self, observed: usize) -> usize {
        // The small epsilon keeps 0.5 * 4 from rounding up to 3 through float noise.
        (self.convergence_min_explained_fraction * observed as f64 - 1e-9)
            .ceil()
            .max(1.0) as usize
    }

    /// Reject a configuration that cannot work, with a message naming the field.
    pub fn validate(&self) -> Result<(), String> {
        if self.num_particles == 0 {
            return Err("global num_particles must be > 0".into());
        }
        if !(self.landmark_keepout_m >= 0.0 && self.landmark_keepout_m.is_finite()) {
            return Err("landmark_keepout_m must be finite and >= 0".into());
        }
        if !(self.convergence_sigma_xy_m > 0.0 && self.convergence_sigma_xy_m.is_finite()) {
            return Err("convergence_sigma_xy_m must be finite and > 0".into());
        }
        if !(self.convergence_sigma_theta_rad > 0.0 && self.convergence_sigma_theta_rad.is_finite())
        {
            return Err("convergence_sigma_theta_rad must be finite and > 0".into());
        }
        if self.convergence_sustained_scans == 0 {
            return Err("convergence_sustained_scans must be >= 1".into());
        }
        if !(self.convergence_min_explained_fraction > 0.0
            && self.convergence_min_explained_fraction <= 1.0)
        {
            return Err("convergence_min_explained_fraction must be in (0, 1]".into());
        }
        Ok(())
    }
}

/// How spread out a weighted cloud is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spread {
    /// `sqrt(var(x) + var(y))` of the weighted cloud (m).
    pub sigma_xy: f64,
    /// Circular standard deviation of the weighted headings, `sqrt(-2 ln R)` (rad).
    pub sigma_theta: f64,
}

/// [`Spread`] of a cloud. `weights` must be non-negative and sum to one, in the order of
/// `states`; an empty cloud has infinite spread.
pub fn cloud_spread<'a>(states: impl Iterator<Item = &'a Se2>, weights: &[f64]) -> Spread {
    let (mut sx, mut sy, mut ssin, mut scos, mut total) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let pairs: Vec<(f64, &Se2)> = weights.iter().copied().zip(states).collect();
    for (w, s) in &pairs {
        sx += w * s.x();
        sy += w * s.y();
        ssin += w * s.theta().sin();
        scos += w * s.theta().cos();
        total += w;
    }
    if pairs.is_empty() || total <= 0.0 {
        return Spread {
            sigma_xy: f64::INFINITY,
            sigma_theta: f64::INFINITY,
        };
    }
    let (mx, my) = (sx / total, sy / total);
    let mut var = 0.0;
    for (w, s) in &pairs {
        var += w * ((s.x() - mx).powi(2) + (s.y() - my).powi(2));
    }
    let r = (ssin.hypot(scos) / total).clamp(1e-12, 1.0);
    Spread {
        sigma_xy: (var / total).sqrt(),
        sigma_theta: (-2.0 * r.ln()).max(0.0).sqrt(),
    }
}

/// A pose drawn uniformly over the free space of the map: `(x, y)` uniform over the square
/// of half side `half_extent` outside every landmark's keep-out disc, heading uniform over
/// `(-pi, pi]`. `None` if no free pose was found in a thousand draws, which means the
/// keep-out discs cover the square.
pub fn sample_free_pose<R: Rng + ?Sized>(
    map: &LandmarkMap,
    half_extent: f64,
    keepout: f64,
    rng: &mut R,
) -> Option<Se2> {
    for _ in 0..1000 {
        let x = rng.random_range(-half_extent..half_extent);
        let y = rng.random_range(-half_extent..half_extent);
        let blocked = map
            .landmarks()
            .iter()
            .any(|l| (l.position.x - x).hypot(l.position.y - y) < keepout);
        if !blocked {
            return Some(Se2::new(x, y, rng.random_range(-PI..PI)));
        }
    }
    None
}

/// Decides when a global search has converged, from the spread of the cloud and whether its
/// mean explains the scan. See the module documentation for the criterion and the reasons
/// for each part.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConvergenceDetector {
    cfg: GlobalLocalizationConfig,
    streak: u32,
    converged: bool,
}

impl ConvergenceDetector {
    /// A detector that has not seen a scan yet.
    pub fn new(cfg: GlobalLocalizationConfig) -> Self {
        Self {
            cfg,
            streak: 0,
            converged: false,
        }
    }

    /// Feed one scored scan: the spread of the cloud after the update and whether its mean
    /// explains the scan. Returns `true` on the one update at which convergence is
    /// declared, and `false` before and after it.
    pub fn observe(&mut self, spread: Spread, mean_explains_scan: bool) -> bool {
        if self.converged {
            return false;
        }
        let tight = spread.sigma_xy < self.cfg.convergence_sigma_xy_m
            && spread.sigma_theta < self.cfg.convergence_sigma_theta_rad
            && mean_explains_scan;
        self.streak = if tight { self.streak + 1 } else { 0 };
        if self.streak >= self.cfg.convergence_sustained_scans {
            self.converged = true;
            return true;
        }
        false
    }

    /// The configuration the detector judges by.
    pub fn config(&self) -> &GlobalLocalizationConfig {
        &self.cfg
    }

    /// Consecutive scans so far for which the criterion held.
    pub fn streak(&self) -> u32 {
        self.streak
    }

    /// Whether convergence has been declared.
    pub fn converged(&self) -> bool {
        self.converged
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;
    use rand::rngs::SmallRng;
    use rand::SeedableRng;
    use robotics_models::sensor::landmark::Landmark;

    fn map() -> LandmarkMap {
        LandmarkMap::new(vec![
            Landmark {
                id: 0,
                position: Vector2::new(1.0, 1.0),
            },
            Landmark {
                id: 1,
                position: Vector2::new(-1.0, -1.0),
            },
        ])
        .unwrap()
    }

    #[test]
    fn free_poses_stay_in_the_room_and_out_of_the_keepout_discs() {
        let m = map();
        let mut rng = SmallRng::seed_from_u64(3);
        for _ in 0..5000 {
            let p = sample_free_pose(&m, 2.8, 0.26, &mut rng).unwrap();
            assert!(p.x().abs() <= 2.8 && p.y().abs() <= 2.8);
            assert!(p.theta() > -PI && p.theta() <= PI);
            for l in m.landmarks() {
                assert!((l.position.x - p.x()).hypot(l.position.y - p.y()) >= 0.26);
            }
        }
    }

    #[test]
    fn a_keepout_that_covers_the_room_yields_no_pose() {
        let mut rng = SmallRng::seed_from_u64(3);
        assert!(sample_free_pose(&map(), 0.1, 50.0, &mut rng).is_none());
    }

    #[test]
    fn spread_of_a_point_is_zero_and_of_two_clusters_is_not() {
        let tight = vec![Se2::new(1.0, 2.0, 0.3); 10];
        let w = vec![0.1; 10];
        let s = cloud_spread(tight.iter(), &w);
        assert!(s.sigma_xy < 1e-12 && s.sigma_theta < 1e-6);

        let mut two = vec![Se2::new(0.0, 0.0, 0.0); 5];
        two.extend(vec![Se2::new(3.0, 0.0, 0.0); 5]);
        let s = cloud_spread(two.iter(), &w);
        assert!((s.sigma_xy - 1.5).abs() < 1e-9, "got {}", s.sigma_xy);
    }

    #[test]
    fn heading_spread_is_circular() {
        // Headings either side of +-pi are 0.02 rad apart, not 6.26.
        let across = [
            Se2::new(0.0, 0.0, PI - 0.01),
            Se2::new(0.0, 0.0, -PI + 0.01),
        ];
        let s = cloud_spread(across.iter(), &[0.5, 0.5]);
        assert!(s.sigma_theta < 0.02, "got {}", s.sigma_theta);
        // Opposite headings are as spread out as headings can be.
        let opposite = [Se2::new(0.0, 0.0, 0.0), Se2::new(0.0, 0.0, PI)];
        let s = cloud_spread(opposite.iter(), &[0.5, 0.5]);
        assert!(s.sigma_theta > 5.0, "got {}", s.sigma_theta);
    }

    #[test]
    fn an_empty_cloud_has_infinite_spread() {
        let s = cloud_spread(std::iter::empty(), &[]);
        assert!(s.sigma_xy.is_infinite() && s.sigma_theta.is_infinite());
    }

    fn tight() -> Spread {
        Spread {
            sigma_xy: 0.01,
            sigma_theta: 0.01,
        }
    }

    #[test]
    fn convergence_needs_a_sustained_run_and_declares_once() {
        let cfg = GlobalLocalizationConfig {
            convergence_sustained_scans: 3,
            ..Default::default()
        };
        let mut d = ConvergenceDetector::new(cfg);
        assert!(!d.observe(tight(), true));
        assert!(!d.observe(tight(), true));
        assert!(d.observe(tight(), true), "third tight scan declares it");
        assert!(d.converged());
        assert!(!d.observe(tight(), true), "and it is declared only once");
    }

    #[test]
    fn one_bad_scan_restarts_the_count() {
        let cfg = GlobalLocalizationConfig {
            convergence_sustained_scans: 3,
            ..Default::default()
        };
        let mut d = ConvergenceDetector::new(cfg);
        d.observe(tight(), true);
        d.observe(tight(), true);
        // Wide in position, then wide in heading, then a tight cloud whose mean does not fit.
        for bad in [
            (
                Spread {
                    sigma_xy: 0.5,
                    ..tight()
                },
                true,
            ),
            (
                Spread {
                    sigma_theta: 0.5,
                    ..tight()
                },
                true,
            ),
            (tight(), false),
        ] {
            d.observe(bad.0, bad.1);
            assert_eq!(d.streak(), 0);
            d.observe(tight(), true);
            d.observe(tight(), true);
            assert!(!d.converged());
            d.observe(
                Spread {
                    sigma_xy: 0.5,
                    ..tight()
                },
                true,
            );
        }
    }

    #[test]
    fn explained_needed_matches_the_upper_median_at_the_default() {
        let c = GlobalLocalizationConfig::default();
        let needed: Vec<usize> = (1..=6).map(|n| c.explained_needed(n)).collect();
        assert_eq!(needed, vec![1, 1, 2, 2, 3, 3]);
        let strict = GlobalLocalizationConfig {
            convergence_min_explained_fraction: 1.0,
            ..c
        };
        assert_eq!(
            (1..=4)
                .map(|n| strict.explained_needed(n))
                .collect::<Vec<_>>(),
            vec![1, 2, 3, 4]
        );
        let majority = GlobalLocalizationConfig {
            convergence_min_explained_fraction: 0.51,
            ..c
        };
        assert_eq!(
            (1..=4)
                .map(|n| majority.explained_needed(n))
                .collect::<Vec<_>>(),
            vec![1, 2, 2, 3]
        );
    }

    #[test]
    fn validation_names_the_bad_field() {
        let bad = GlobalLocalizationConfig {
            convergence_sustained_scans: 0,
            ..Default::default()
        };
        assert!(bad.validate().unwrap_err().contains("sustained"));
        let bad = GlobalLocalizationConfig {
            convergence_min_explained_fraction: 0.0,
            ..Default::default()
        };
        assert!(bad.validate().unwrap_err().contains("explained_fraction"));
        assert!(GlobalLocalizationConfig::default().validate().is_ok());
    }
}
