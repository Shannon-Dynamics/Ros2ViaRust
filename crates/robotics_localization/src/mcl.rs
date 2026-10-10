//! MCL localization against a known landmark map (Thrun, Burgard and Fox, Table 8.2:
//! particle filter localization with a known map and unknown correspondence).
//!
//! This reuses, rather than reimplements, three things [`crate::ekf`] and the rest of the
//! framework already provide:
//!
//! * [`robotics_models::sensor::landmark::LandmarkModel`], the same range and bearing
//!   model the EKF path corrects with; only the way its likelihood is used differs
//!   (reweighting a population instead of correcting one Gaussian).
//! * [`robotics_models::motion::OdometryMotionModel`], Thrun's sampling odometry motion
//!   model (Table 5.6, the book's `OdomDelta`/`OdometryModel`), which the EKF path does
//!   not need because it propagates a covariance in closed form instead of sampling.
//! * [`robotics_filters::particle::ParticleSet`], the weighted particle machinery (log
//!   weights, effective sample size, low-variance resampling) built in Phase 2 and left
//!   unused until this module.
//!
//! # Why this filter exists next to the EKF
//!
//! [`crate::ekf::EkfLocalizer`] represents the belief as one Gaussian. That is enough for
//! position tracking (P1) and its own test
//! (`a_teleported_robot_is_not_recovered_by_a_single_gaussian`) demonstrates the one thing
//! it structurally cannot do: after a kidnapping the gate rejects every observation that no
//! longer fits the belief it has, and the belief never moves. A particle set has no such
//! single point of failure. Each particle is an independent hypothesis, and
//! [`McLocalizer::observe`] can discard the ones the scan disagrees with and still keep
//! others around, or, when none of them fit, replace some with fresh guesses. Section 4 of
//! this file is that recovery mechanism.
//!
//! # Resampling: low-variance, triggered by effective sample size
//!
//! [`ParticleSet::resample_if_needed`] is used as-is: low-variance (systematic) resampling,
//! the comb algorithm of Thrun Table 4.4, triggered when the effective sample size drops
//! below `resample_threshold_ratio · M` (the book's own default is 0.5, kept here). Low
//! variance is the well-established choice over multinomial resampling for the same reason
//! the book gives it: for the same `M` it has strictly lower resampling variance, because
//! the `M` draws are one correlated comb instead of `M` independent ones, and it is `O(M)`
//! rather than needing a sort or a binary search per draw. Multinomial resampling
//! ([`ParticleSet::resample_multinomial`]) exists in the same crate for cases that want the
//! textbook's reference scheme instead; this filter does not use it.
//!
//! Adaptive sizing (KLD sampling, [`robotics_filters::kld`]) and the dominant-cluster pose
//! extraction that a full Augmented MCL implementation would use are not in this module. A
//! fixed particle count and a single weighted mean are enough to answer the question this
//! phase asks (does MCL recover from a kidnapping where the EKF does not), and both are
//! deferred to whichever later phase needs the adaptive cost or the multimodal output; see
//! `docs/phase-3-localization/implementation.md`.

use nalgebra::{Matrix3, Vector3};
use rand::{Rng, RngExt};
use robotics_core::angle::{angle_diff, normalize_angle};
use robotics_core::prob::stats::chi2_quantile;
use robotics_core::types::Cov3;
use robotics_core::{Gaussian, Se2};
use robotics_filters::particle::{multinomial_indices, ParticleSet};
use robotics_models::motion::{OdomAlphas, OdomDelta, OdometryMotionModel};
use robotics_models::sensor::landmark::{LandmarkMap, LandmarkModel, RangeBearing};
use std::f64::consts::PI;

pub use crate::ekf::LocalizationError;
use crate::global::{cloud_spread, sample_free_pose};
pub use crate::global::{ConvergenceDetector, GlobalLocalizationConfig, LocalizationPhase, Spread};

/// Configuration of [`McLocalizer`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MclConfig {
    /// Particle count, fixed for the lifetime of the filter.
    ///
    /// 6000 is the measured floor used for Phase 3 correctness. With 300 particles the
    /// Monte Carlo spread produced 0.051534 m RMSE on the 51.5 m normal route; 3000 reduced
    /// it to 0.020937 m but remained 10.9 percent worse than odometry. 6000 produced
    /// 0.014769 m versus odometry's 0.015697 m while publishing 3265 estimates over about
    /// 371 seconds. The complete runs are listed in
    /// `docs/phase-3-localization/benchmark.md`. This fixed count is intentionally a
    /// correctness choice; adaptive KLD sizing remains future work.
    pub num_particles: usize,
    /// Range noise of one landmark observation (m); shared with the EKF's measurement
    /// model so the two filters are graded on the same sensor, not on two different ones.
    pub sigma_range: f64,
    /// Bearing noise of one landmark observation (rad).
    pub sigma_bearing: f64,
    /// Initial position standard deviation (m), used by [`McLocalizer::new_at`] to spread
    /// the particles around the known starting pose. Not used by [`McLocalizer::new_global`],
    /// which spreads uniformly over the whole room instead. Same name and default as
    /// [`crate::ekf::LocalizerConfig::initial_sigma_xy`], for the same reason the measurement
    /// sigmas are shared: this is a property of how well the start is known, not of which
    /// filter is asked to track it.
    ///
    /// Starting every particle at *exactly* the same point, with no spread at all, was tried
    /// first and is why this field exists. With zero spread every particle is identical, so
    /// the first several scans reweight them all equally (no information to resample on),
    /// and by the time motion noise alone has spread them apart, a small motion-model bias
    /// has already been baked into the whole set with nothing to correct it against. On the
    /// 51 m route of `evidence/phase-3-correctness/`, that produced a filter whose estimate
    /// diverged to metre-scale error and then triggered the recovery injection on nearly
    /// every scan, each injection further corrupting the mean; see this crate's test
    /// `converges_from_a_tight_prior` for the fixed behaviour.
    pub initial_sigma_xy: f64,
    /// Initial heading standard deviation (rad), same reasoning as `initial_sigma_xy`.
    pub initial_sigma_theta: f64,
    /// Odometry motion model noise coefficients, Thrun Table 5.6's `alpha1..alpha4`. Kept
    /// separate from [`crate::ekf::OdometryMotion`]'s coefficients because the two models
    /// are different functional forms (this one needs `[f64; 4]` for
    /// [`OdomAlphas`], the EKF's needs a lateral-slip fraction and noise floors that this
    /// model has no slot for), not because the physical motion is believed to differ.
    pub motion_alphas: [f64; 4],
    /// Resample when the effective sample size drops below this fraction of
    /// `num_particles`. 0.5 is Thrun's own default and is used unchanged.
    pub resample_threshold_ratio: f64,
    /// Pose of the laser in the body frame, exactly as the EKF's `sensor_pose` is used.
    pub sensor_pose: Se2,
    /// Half the side length of the square free-space region particles are injected into
    /// (m). The room is 6 by 6 m with 0.05 m walls, so 2.8 m keeps every injected particle
    /// off the walls with margin for the robot's own footprint.
    pub room_half_extent: f64,
    /// The gate probability a per-scan fit is judged against, in the same sense
    /// [`crate::ekf::LocalizerConfig::gate_probability`] is: the probability mass of a
    /// chi-square(2) distribution the EKF's own association gate keeps inside. Expressed
    /// here as a likelihood ratio rather than a Mahalanobis distance (see
    /// [`McLocalizer::fit_log_threshold`]), it becomes the threshold a scan's best-fitting
    /// particle has to clear to count as "explained". Reusing the EKF's own gate number
    /// means the trigger is "this scan fits the whole particle cloud no better than the
    /// EKF would already have refused to accept it", not a new constant invented for this
    /// filter and tuned until a demo looked right.
    pub recovery_gate_probability: f64,
    /// Consecutive low-fit scans required before particles are injected. With 6000
    /// particles, 5 still fired seven times on a normal route. Twenty fired zero times on
    /// the measured normal run. Recovery remains fast because the injection is a full
    /// global reset: the measured kidnapped run recovered in 6.92 seconds.
    pub recovery_sustained_updates: u32,
    /// Probability, independently per particle, that a low-fit streak replaces it with a
    /// uniform random pose rather than leaving it where it was. 1.0 makes a sustained
    /// mismatch a full global reset. The earlier 0.2 mixture failed recovery after the
    /// persistence was raised to protect normal tracking; a full 6000-particle reset
    /// recovered in the measured kidnapped run.
    pub recovery_injection_fraction: f64,
}

impl Default for MclConfig {
    fn default() -> Self {
        Self {
            num_particles: 6000,
            sigma_range: 0.05,
            sigma_bearing: 0.05,
            initial_sigma_xy: 0.05,
            initial_sigma_theta: 0.05,
            motion_alphas: OdomAlphas::DEFAULT.0,
            resample_threshold_ratio: 0.5,
            sensor_pose: Se2::IDENTITY,
            room_half_extent: 2.8,
            recovery_gate_probability: 0.99,
            recovery_sustained_updates: 20,
            recovery_injection_fraction: 1.0,
        }
    }
}

/// What one measurement update did.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct McUpdateReport {
    /// Observations handed in.
    pub observed: usize,
    /// Effective sample size after reweighting (and after injection, if it happened).
    pub ess: f64,
    /// Best particle's upper-median raw per-observation log likelihood, the robust
    /// statistic [`McLocalizer::fit_log_threshold`] is compared against. `None` when there
    /// were no observations to score. The values entering the median are not gated or
    /// clamped, so a real kidnapping remains visible to the recovery trigger.
    pub best_fit_median_log_likelihood: Option<f64>,
    /// Consecutive low-fit scans counted so far (reset to 0 by a good scan or by an
    /// injection).
    pub low_fit_streak: u32,
    /// Whether this update injected random particles.
    pub injected: bool,
    /// Whether this update resampled (never true in the same update as an injection: the
    /// injected set is already fresh and uniform, so resampling it immediately would throw
    /// away the particles just added for no reason).
    pub resampled: bool,
    /// The phase this update *started* in. A search that converges during this update still
    /// reports [`LocalizationPhase::GlobalSearch`] here and sets `converged_now`.
    pub phase: LocalizationPhase,
    /// Spread of the cloud after this update, while a global search is running, else
    /// `None`. See [`crate::global`] for the definition.
    pub global_spread: Option<Spread>,
    /// Whether this update is the one at which a global search was declared converged.
    pub converged_now: bool,
}

/// What a global search had achieved at the moment it was declared converged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalConvergence {
    /// Scored scans (scans with at least one observation) since the search began.
    pub scored_updates: u64,
    /// Odometric distance travelled since the search began (m): the sum of the translation
    /// of each odometry step handed to [`McLocalizer::predict`].
    pub travel_m: f64,
    /// Spread of the cloud on the converging scan.
    pub spread: Spread,
    /// Particle count during the search.
    pub particles_before: usize,
    /// Particle count after shrinking to the steady-state size.
    pub particles_after: usize,
}

/// Progress of a global search, running or finished.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalStatus {
    /// Spread of the cloud after the latest scored scan.
    pub spread: Spread,
    /// Consecutive scans the convergence criterion has held for.
    pub streak: u32,
    /// Scored scans since the search began.
    pub scored_updates: u64,
    /// Odometric distance travelled since the search began (m), frozen at convergence.
    pub travel_m: f64,
    /// `Some` once the search has been declared converged.
    pub converged: Option<GlobalConvergence>,
}

/// Search state carried by a filter that was started from a uniform prior.
struct GlobalState {
    detector: ConvergenceDetector,
    status: GlobalStatus,
}

/// Counters over the lifetime of the filter.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MclStats {
    /// Prediction steps applied.
    pub predictions: u64,
    /// Scans handed to [`McLocalizer::observe`].
    pub updates: u64,
    /// Updates whose scan carried at least one observation.
    pub scored_updates: u64,
    /// Resample events.
    pub resamples: u64,
    /// Injection events.
    pub injections: u64,
}

/// MCL localization on SE(2) against a known landmark map.
pub struct McLocalizer {
    particles: ParticleSet<Se2>,
    map: LandmarkMap,
    model: LandmarkModel,
    motion: OdometryMotionModel,
    cfg: MclConfig,
    /// `ln(peak likelihood of one observation) - 0.5 * chi2_quantile(2, gate_probability)`;
    /// see [`McLocalizer::fit_log_threshold`].
    fit_log_threshold: f64,
    low_fit_streak: u32,
    stats: MclStats,
    phase: LocalizationPhase,
    global: Option<GlobalState>,
}

impl McLocalizer {
    /// Build a filter with particles drawn uniformly over the whole configured square, with
    /// no keep-out around the pillars and no convergence logic: a bare starting belief, kept
    /// for tests that need one. It is **not** global localization; that is
    /// [`McLocalizer::new_uniform_prior`], which draws only free poses and runs a search that
    /// ends in a declared convergence.
    pub fn new_global<R: Rng + ?Sized>(
        cfg: &MclConfig,
        map: LandmarkMap,
        rng: &mut R,
    ) -> Result<Self, LocalizationError> {
        let half = cfg.room_half_extent;
        let states: Vec<Se2> = (0..cfg.num_particles)
            .map(|_| Self::uniform_pose(half, rng))
            .collect();
        Self::new_from_states(cfg, map, states)
    }

    /// Build a filter for **global localization**: the robot has no estimate of its pose, so
    /// `global.num_particles` particles are drawn uniformly over the free space of the map
    /// (see [`crate::global`]) and the filter starts in [`LocalizationPhase::GlobalSearch`].
    ///
    /// This is a different code path from the kidnap recovery of this file. There is no
    /// trigger here: the cloud starts as wide as it can be and narrows through the ordinary
    /// reweight and resample steps of [`McLocalizer::observe`], and the filter itself says
    /// when it has narrowed enough ([`crate::global::ConvergenceDetector`]). On the scan that
    /// declares it, the cloud is thinned to `cfg.num_particles` (when the search used more)
    /// and the filter becomes an ordinary tracking filter, kidnap trigger armed with its
    /// unchanged tuning.
    pub fn new_uniform_prior<R: Rng + ?Sized>(
        cfg: &MclConfig,
        global: &GlobalLocalizationConfig,
        map: LandmarkMap,
        rng: &mut R,
    ) -> Result<Self, LocalizationError> {
        global.validate().map_err(LocalizationError::Config)?;
        let mut states = Vec::with_capacity(global.num_particles);
        for _ in 0..global.num_particles {
            let pose = sample_free_pose(&map, cfg.room_half_extent, global.landmark_keepout_m, rng)
                .ok_or_else(|| {
                    LocalizationError::Config(
                        "no free pose: the landmark keep-out covers the room".into(),
                    )
                })?;
            states.push(pose);
        }
        let mut filter = Self::new_from_states(cfg, map, states)?;
        filter.phase = LocalizationPhase::GlobalSearch;
        filter.global = Some(GlobalState {
            detector: ConvergenceDetector::new(*global),
            status: GlobalStatus {
                spread: Spread {
                    sigma_xy: f64::INFINITY,
                    sigma_theta: f64::INFINITY,
                },
                streak: 0,
                scored_updates: 0,
                travel_m: 0.0,
                converged: None,
            },
        });
        Ok(filter)
    }

    /// The regime the filter is in.
    pub fn phase(&self) -> LocalizationPhase {
        self.phase
    }

    /// Progress of the global search, or `None` for a filter that was not started from a
    /// uniform prior. Stays available after convergence, with `converged` filled in.
    pub fn global_status(&self) -> Option<GlobalStatus> {
        self.global.as_ref().map(|g| g.status)
    }

    /// Build a filter with particles spread as `N(initial_pose, diag(initial_sigma_xy²,
    /// initial_sigma_xy², initial_sigma_theta²))`, for position tracking (P1) where the
    /// start is known. Not spread uniformly, and not placed at `initial_pose` with no
    /// spread at all either; see [`MclConfig::initial_sigma_xy`] for why both of those
    /// were tried and rejected.
    pub fn new_at<R: Rng + ?Sized>(
        cfg: &MclConfig,
        map: LandmarkMap,
        initial_pose: Se2,
        rng: &mut R,
    ) -> Result<Self, LocalizationError> {
        let gaussian = Gaussian::<3>::new(
            Vector3::new(initial_pose.x(), initial_pose.y(), initial_pose.theta()),
            Matrix3::from_diagonal(&Vector3::new(
                cfg.initial_sigma_xy.powi(2),
                cfg.initial_sigma_xy.powi(2),
                cfg.initial_sigma_theta.powi(2),
            )),
        )
        .map_err(|e| LocalizationError::Config(e.to_string()))?;
        let states: Vec<Se2> = (0..cfg.num_particles)
            .map(|_| {
                let v = gaussian.sample(rng);
                Se2::new(v.x, v.y, normalize_angle(v.z))
            })
            .collect();
        Self::new_from_states(cfg, map, states)
    }

    fn new_from_states(
        cfg: &MclConfig,
        map: LandmarkMap,
        states: Vec<Se2>,
    ) -> Result<Self, LocalizationError> {
        if map.is_empty() {
            return Err(LocalizationError::EmptyMap);
        }
        if cfg.num_particles == 0 {
            return Err(LocalizationError::Config(
                "num_particles must be > 0".into(),
            ));
        }
        if !(cfg.recovery_gate_probability > 0.0 && cfg.recovery_gate_probability < 1.0) {
            return Err(LocalizationError::Config(
                "recovery_gate_probability must be in (0, 1)".into(),
            ));
        }
        let model = LandmarkModel::new(cfg.sigma_range, cfg.sigma_bearing)
            .and_then(|m| m.with_sensor_pose(cfg.sensor_pose))
            .map_err(|e| LocalizationError::Config(e.to_string()))?;
        let motion = OdometryMotionModel::new(OdomAlphas(cfg.motion_alphas))
            .map_err(|e| LocalizationError::Config(e.to_string()))?;
        let particles = ParticleSet::from_states(states).map_err(LocalizationError::Filter)?;

        // ln(peak) = -ln(sigma_r * sigma_phi) - ln(2*pi), the value LandmarkModel's own
        // log_likelihood returns at zero residual; see its doc comment. The chi-square
        // term converts the EKF's gate_probability into "how many natural-log units below
        // that peak a scan may fall and still count as explained".
        let log_peak = -(cfg.sigma_range * cfg.sigma_bearing).ln() - (2.0 * PI).ln();
        let chi2 = chi2_quantile(2, cfg.recovery_gate_probability)
            .map_err(|e| LocalizationError::Config(e.to_string()))?;
        let fit_log_threshold = log_peak - 0.5 * chi2;

        Ok(Self {
            particles,
            map,
            model,
            motion,
            cfg: *cfg,
            fit_log_threshold,
            low_fit_streak: 0,
            stats: MclStats::default(),
            phase: LocalizationPhase::Tracking,
            global: None,
        })
    }

    fn uniform_pose<R: Rng + ?Sized>(half_extent: f64, rng: &mut R) -> Se2 {
        Se2::new(
            rng.random_range(-half_extent..half_extent),
            rng.random_range(-half_extent..half_extent),
            rng.random_range(-PI..PI),
        )
    }

    /// The threshold [`McUpdateReport::best_fit_median_log_likelihood`] is
    /// compared against: the natural log of the per-observation likelihood ratio, relative
    /// to a perfect (zero-residual) observation, below which a scan counts as unexplained.
    /// Equal to `ln(peak likelihood) - 0.5 * chi2_quantile(2, recovery_gate_probability)`.
    pub fn fit_log_threshold(&self) -> f64 {
        self.fit_log_threshold
    }

    /// Weighted mean pose. A plain weighted mean with a circular mean for heading, not a
    /// dominant-cluster estimate: while the belief is genuinely multimodal (freshly
    /// kidnapped, or before global localization has converged), this can sit between
    /// clusters and mean nothing on its own. Once resampling has collapsed the particles
    /// onto one cluster, which is what P1 measures and what a recovered kidnap looks like,
    /// it is an ordinary, correct mean.
    pub fn pose(&self) -> Se2 {
        let weights = self.particles.weights();
        let (mut sx, mut sy, mut ssin, mut scos) = (0.0, 0.0, 0.0, 0.0);
        for (w, s) in weights.iter().zip(self.particles.states()) {
            sx += w * s.x();
            sy += w * s.y();
            ssin += w * s.theta().sin();
            scos += w * s.theta().cos();
        }
        Se2::new(sx, sy, ssin.atan2(scos))
    }

    /// Weighted sample covariance around [`McLocalizer::pose`], diagonal in heading (no
    /// position/heading cross terms, matching the simplicity level of the diagnostics the
    /// EKF publishes). Same multimodality caveat as `pose()`.
    pub fn covariance(&self) -> Cov3 {
        let weights = self.particles.weights();
        let mean = self.pose();
        let (mut vxx, mut vyy, mut vxy, mut vtt) = (0.0, 0.0, 0.0, 0.0);
        for (w, s) in weights.iter().zip(self.particles.states()) {
            let dx = s.x() - mean.x();
            let dy = s.y() - mean.y();
            let dt = angle_diff(s.theta(), mean.theta());
            vxx += w * dx * dx;
            vyy += w * dy * dy;
            vxy += w * dx * dy;
            vtt += w * dt * dt;
        }
        Cov3::new(vxx, vxy, 0.0, vxy, vyy, 0.0, 0.0, 0.0, vtt)
    }

    /// Effective sample size of the current (normalized) weights.
    pub fn effective_sample_size(&self) -> f64 {
        self.particles.effective_sample_size()
    }

    /// Particle count. Constant for the filter's lifetime except at the one moment a global
    /// search converges with more particles than the steady-state count, when it drops to
    /// `cfg.num_particles`.
    pub fn len(&self) -> usize {
        self.particles.len()
    }

    /// Always `false`: the constructors reject a zero particle count.
    pub fn is_empty(&self) -> bool {
        self.particles.len() == 0
    }

    /// Lifetime counters.
    pub fn stats(&self) -> MclStats {
        self.stats
    }

    /// Prediction: sample each particle's motion from the odometry delta between two
    /// consecutive raw odometry poses, exactly as [`crate::ekf::EkfLocalizer::predict`]
    /// takes its control, except that this filter needs the two absolute odometry poses
    /// rather than their composed relative transform. [`OdomDelta::from_poses`] decomposes
    /// a motion into rotate-translate-rotate using the *direction of travel*
    /// (`atan2(dy, dx)`) to split the heading change between the two rotations, which
    /// needs the actual positions; a pure body-frame relative transform throws that away.
    pub fn predict<R: Rng + ?Sized>(
        &mut self,
        previous_odom: &Se2,
        current_odom: &Se2,
        rng: &mut R,
    ) {
        let delta = OdomDelta::from_poses(previous_odom, current_odom);
        if delta.trans.abs() < 1e-9 && delta.rot1.abs() < 1e-9 && delta.rot2.abs() < 1e-9 {
            return; // standing still: nothing to predict, and no noise to add
        }
        if self.phase == LocalizationPhase::GlobalSearch {
            if let Some(g) = self.global.as_mut() {
                g.status.travel_m += delta.trans.abs();
            }
        }
        let motion = self.motion;
        self.particles.map_states(|state| {
            *state = motion.sample_delta(state, &delta, rng);
        });
        self.stats.predictions += 1;
    }

    /// `ln max_i p(z | x, map landmark i)`: the maximum-likelihood association Thrun
    /// Table 8.2 uses for a known map with unknown correspondence, applied per particle
    /// rather than to one belief (which is [`crate::ekf::EkfLocalizer::observe`]'s job).
    /// Each detection independently picks whichever landmark explains it best *for this
    /// particle's pose*; a wrong particle can and does pick a different landmark than a
    /// nearby correct one would, which is exactly the information that lets the sensor
    /// model tell the two apart.
    fn best_landmark_log_likelihood(&self, pose: &Se2, z: &RangeBearing) -> f64 {
        self.map
            .landmarks()
            .iter()
            .filter_map(|l| self.model.log_likelihood(pose, &l.position, z).ok())
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Upper median of the raw per-observation scores for one particle.
    ///
    /// A scan is considered explained when at least half of its observations clear the
    /// same likelihood-ratio threshold used by the EKF. Unlike
    /// [`McLocalizer::gated_log_likelihood`], this function neither drops nor clamps an
    /// outlier before summarising the scan. Selecting the upper median only prevents a
    /// minority of arbitrarily bad detections from overwhelming the observations that do
    /// fit. After a kidnapping, when the existing cloud explains fewer than half of the
    /// genuine observations, the raw median remains below the threshold and recovery still
    /// fires.
    fn median_log_likelihood(&self, pose: &Se2, observations: &[RangeBearing]) -> f64 {
        let mut scores: Vec<f64> = observations
            .iter()
            .map(|z| self.best_landmark_log_likelihood(pose, z))
            .collect();
        scores.sort_unstable_by(f64::total_cmp);
        scores[scores.len() / 2]
    }

    /// The score reweighting actually uses: the same per-observation best-landmark
    /// likelihood as [`McLocalizer::median_log_likelihood`], except that an observation no
    /// landmark explains within the same chi-square gate the EKF applies per observation
    /// (`self.fit_log_threshold`) contributes nothing, rather than its arbitrarily negative
    /// raw value.
    ///
    /// This is the fix for a real divergence, not a defensive guess. Without it, every
    /// particle's weight included the full, unbounded penalty of every observation,
    /// including ones no particle could have explained (an occasional spurious cluster from
    /// `pillars::detect`, a reflection, a scan that briefly saw a wall edge as a pillar).
    /// That penalty is the *same* for every particle regardless of how good its pose is, so
    /// it does not change which particles resampling favours, but it does make the scan
    /// look like nothing fits at all: exactly the statistic the recovery trigger watches
    /// for. On `sim/webots/config/route_long.yaml`, with no kidnap anywhere in the run, the
    /// unclamped version fired the recovery injection on about one scan in five, each firing
    /// replacing a fifth of the particles with uniform random guesses and resetting every
    /// weight to uniform; the result was a filter whose own corrective mechanism was its
    /// biggest source of error (`evidence/phase-3-correctness/2026-10-10-mcl-comparison`,
    /// `rmse_position_m` over 1 m against the EKF's 0.009 m on the same simulated run).
    /// EKF's `observe()` already does the equivalent thing for the single belief it
    /// corrects: an observation the gate rejects is skipped, not punished (see its
    /// `rejected` counter); this is that same gate, applied per particle instead of per
    /// belief, which is the only form a gate can take once the belief is a population
    /// instead of one Gaussian.
    fn gated_log_likelihood(&self, pose: &Se2, observations: &[RangeBearing]) -> f64 {
        observations
            .iter()
            .map(|z| {
                let best = self.best_landmark_log_likelihood(pose, z);
                if best >= self.fit_log_threshold {
                    best
                } else {
                    0.0
                }
            })
            .sum()
    }

    /// Correction step, reweighting by the scan's observations, then recovering from a
    /// kidnapping if the scan does not fit any particle, then resampling.
    ///
    /// Order matters and is deliberate. The low-fit streak is judged from this scan's own
    /// evidence before it is folded into the weights, because the weights the filter would
    /// otherwise compare against still carry history from before the kidnapping and would
    /// make a bad scan look less bad than it is. Resampling is skipped in the same update
    /// an injection happens: the injected set is already uniform and fresh, and
    /// low-variance resampling it immediately would be as likely to throw an injected
    /// particle away as to keep it, for no benefit.
    pub fn observe<R: Rng + ?Sized>(
        &mut self,
        observations: &[RangeBearing],
        rng: &mut R,
    ) -> McUpdateReport {
        self.stats.updates += 1;
        let mut report = McUpdateReport {
            observed: observations.len(),
            ess: self.particles.effective_sample_size(),
            phase: self.phase,
            ..Default::default()
        };
        if observations.is_empty() {
            report.low_fit_streak = self.low_fit_streak;
            return report;
        }
        self.stats.scored_updates += 1;

        // Raw and unclamped, for the recovery trigger only. Use the upper median rather
        // than the arithmetic mean so a minority of spurious detections cannot dominate
        // an otherwise well-explained scan.
        //
        // During a global search from a uniform prior the trigger is inert (see
        // crate::global), so neither the statistic nor the streak is computed: the first is
        // an O(particles) pass that a search with tens of thousands of particles should not
        // pay for a number nothing reads, and `best_fit_median_log_likelihood` is `None`
        // for those updates. The Tracking branch is unchanged.
        if self.phase == LocalizationPhase::Tracking {
            let raw: Vec<f64> = self
                .particles
                .states()
                .map(|pose| self.median_log_likelihood(pose, observations))
                .collect();
            let best = raw.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            report.best_fit_median_log_likelihood = Some(best);

            if best < self.fit_log_threshold {
                self.low_fit_streak += 1;
            } else {
                self.low_fit_streak = 0;
            }
        } else {
            self.low_fit_streak = 0;
        }

        // Gated, for reweighting: see gated_log_likelihood for why this has to be a
        // different number from the one just used above.
        let gated: Vec<f64> = self
            .particles
            .states()
            .map(|pose| self.gated_log_likelihood(pose, observations))
            .collect();
        let mut idx = 0usize;
        let reweighted = self.particles.reweight(|_| {
            let l = gated[idx];
            idx += 1;
            l
        });
        if reweighted.is_err() || self.particles.normalize().is_err() {
            // Every particle scored -inf or NaN on this scan (only possible if a particle
            // sits exactly on a landmark, see median_log_likelihood): treat it as a scan
            // with no usable evidence rather than propagating a degenerate weight set.
            self.particles.reset_uniform();
        }

        if self.low_fit_streak >= self.cfg.recovery_sustained_updates {
            self.inject_random_particles(rng);
            self.low_fit_streak = 0;
            self.stats.injections += 1;
            report.injected = true;
        } else if self
            .particles
            .resample_if_needed(self.cfg.resample_threshold_ratio, rng)
            .unwrap_or(false)
        {
            self.stats.resamples += 1;
            report.resampled = true;
        }

        if self.phase == LocalizationPhase::GlobalSearch {
            self.advance_global_search(observations, rng, &mut report);
        }

        report.low_fit_streak = self.low_fit_streak;
        report.ess = self.particles.effective_sample_size();
        report
    }

    /// Global search bookkeeping after one scored update: measure the cloud, ask the
    /// detector whether the search has converged, and if it has, thin the cloud to the
    /// steady-state size and hand over to tracking. Reached only in
    /// [`LocalizationPhase::GlobalSearch`], so the tracking path never executes any of it.
    fn advance_global_search<R: Rng + ?Sized>(
        &mut self,
        observations: &[RangeBearing],
        rng: &mut R,
        report: &mut McUpdateReport,
    ) {
        let weights = self.particles.weights();
        let spread = cloud_spread(self.particles.states(), &weights);
        // Does the cloud's own mean explain this scan? One evaluation per detection at one
        // pose, not one per particle.
        let mean = self.pose();
        let explained = observations
            .iter()
            .filter(|z| self.best_landmark_log_likelihood(&mean, z) >= self.fit_log_threshold)
            .count();
        report.global_spread = Some(spread);

        let particles_before = self.particles.len();
        let steady = self.cfg.num_particles;
        let Some(g) = self.global.as_mut() else {
            return;
        };
        let explains = explained >= g.detector.config().explained_needed(observations.len());
        g.status.spread = spread;
        g.status.scored_updates += 1;
        let converged_now = g.detector.observe(spread, explains);
        g.status.streak = g.detector.streak();
        if !converged_now {
            return;
        }
        let travel_m = g.status.travel_m;
        let scored_updates = g.status.scored_updates;

        if particles_before > steady {
            if let Ok(idx) = multinomial_indices(&weights, steady, rng) {
                let states: Vec<Se2> = idx
                    .into_iter()
                    .map(|i| self.particles.particles()[i].state)
                    .collect();
                // Cannot error: `states` is non-empty.
                let _ = self.particles.replace_uniform(states);
            }
        }
        self.phase = LocalizationPhase::Tracking;
        self.low_fit_streak = 0;
        report.converged_now = true;
        if let Some(g) = self.global.as_mut() {
            g.status.converged = Some(GlobalConvergence {
                scored_updates,
                travel_m,
                spread,
                particles_before,
                particles_after: self.particles.len(),
            });
        }
    }

    /// The simplified kidnap recovery strategy this filter uses: each particle
    /// independently has probability `recovery_injection_fraction` of being replaced by a
    /// pose drawn uniformly over the whole configured room. This is deliberately not full
    /// Augmented MCL (Thrun section 8.3.5's `w_slow`/`w_fast` exponential averages, whose
    /// injection fraction is `max(0, 1 - w_fast/w_slow)` and therefore adapts on its own).
    /// That scheme needs two more decay-rate constants tuned against data this phase does
    /// not have a reason to collect yet; a fixed fraction, triggered by a threshold already
    /// justified by the EKF's own gate, answers the question this phase asks (can the
    /// filter recover at all) without inventing tuning this phase cannot defend.
    ///
    /// Every weight is reset to uniform afterwards, surviving particles included: by the
    /// time an injection fires the surviving particles' relative weighting is one scan old,
    /// and there is no strong reason to preserve it over a clean slate the next scan will
    /// grade fairly.
    fn inject_random_particles<R: Rng + ?Sized>(&mut self, rng: &mut R) {
        let half = self.cfg.room_half_extent;
        let fraction = self.cfg.recovery_injection_fraction;
        let states: Vec<Se2> = self
            .particles
            .particles()
            .iter()
            .map(|p| {
                if rng.random::<f64>() < fraction {
                    Self::uniform_pose(half, rng)
                } else {
                    p.state
                }
            })
            .collect();
        // Cannot error: `states` has the same non-zero length as the current set.
        let _ = self.particles.replace_uniform(states);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;
    use rand::rngs::SmallRng;
    use rand::SeedableRng;
    use robotics_models::sensor::landmark::Landmark;

    fn square_map() -> LandmarkMap {
        LandmarkMap::new(vec![
            Landmark {
                id: 0,
                position: Vector2::new(2.0, 0.0),
            },
            Landmark {
                id: 1,
                position: Vector2::new(0.0, 2.0),
            },
            Landmark {
                id: 2,
                position: Vector2::new(-2.0, -1.0),
            },
            Landmark {
                id: 3,
                position: Vector2::new(1.5, -1.8),
            },
        ])
        .unwrap()
    }

    fn perfect_observations(cfg: &MclConfig, map: &LandmarkMap, pose: &Se2) -> Vec<RangeBearing> {
        let model = LandmarkModel::new(cfg.sigma_range, cfg.sigma_bearing)
            .unwrap()
            .with_sensor_pose(cfg.sensor_pose)
            .unwrap();
        map.landmarks()
            .iter()
            .map(|l| model.predict(pose, &l.position).unwrap())
            .collect()
    }

    #[test]
    fn converges_from_a_tight_prior() {
        let cfg = MclConfig::default();
        let map = square_map();
        let truth = Se2::new(0.3, -0.2, 0.2);
        let mut rng = SmallRng::seed_from_u64(1);
        let mut f =
            McLocalizer::new_at(&cfg, map.clone(), Se2::new(0.25, -0.15, 0.15), &mut rng).unwrap();
        let z = perfect_observations(&cfg, &map, &truth);
        for _ in 0..30 {
            f.observe(&z, &mut rng);
        }
        assert!(
            f.pose().distance_to(&truth) < 0.1,
            "estimate {:?} should have converged near {:?}",
            f.pose(),
            truth
        );
    }

    #[test]
    fn recovers_from_a_kidnapping_the_ekf_cannot() {
        let cfg = MclConfig {
            num_particles: 500,
            recovery_sustained_updates: 5,
            ..Default::default()
        };
        let map = square_map();
        let mut rng = SmallRng::seed_from_u64(7);
        let start = Se2::IDENTITY;
        let mut f = McLocalizer::new_at(&cfg, map.clone(), start, &mut rng).unwrap();
        let z0 = perfect_observations(&cfg, &map, &start);
        for _ in 0..20 {
            f.observe(&z0, &mut rng);
        }
        assert!(
            f.pose().distance_to(&start) < 0.15,
            "should converge before the kidnap"
        );

        // This unit test isolates the injection and reweighting mechanism. A small proposal
        // motion supplies the same particle jitter as the live node while the target scan
        // stays fixed; the simulator test covers physically consistent moving recovery.
        let kidnapped_to = Se2::new(-1.5, 1.5, 2.0);
        let mut z1 = perfect_observations(&cfg, &map, &kidnapped_to);
        // One arbitrarily bad detector output must not hide the fact that the old cloud
        // explains none of the four genuine observations, or prevent a newly injected
        // hypothesis from demonstrating that it explains most of the scan.
        z1.push(RangeBearing {
            range: 100.0,
            bearing: 2.9,
        });
        let mut injected = false;
        let mut odom = Se2::IDENTITY;
        for i in 0..150 {
            let next_odom = Se2::new(0.02 * (i + 1) as f64, 0.0, 0.0);
            f.predict(&odom, &next_odom, &mut rng);
            odom = next_odom;
            let r = f.observe(&z1, &mut rng);
            injected |= r.injected;
            if f.pose().distance_to(&kidnapped_to) < 0.2 {
                break;
            }
        }
        assert!(
            injected,
            "a sustained bad fit should have triggered an injection"
        );
        assert!(
            f.pose().distance_to(&kidnapped_to) < 0.2,
            "estimate {:?} should have recovered near {:?}",
            f.pose(),
            kidnapped_to
        );
    }

    #[test]
    fn one_persistent_outlier_does_not_trigger_recovery_during_normal_tracking() {
        let cfg = MclConfig::default();
        let map = square_map();
        let truth = Se2::new(0.3, -0.2, 0.2);
        let mut rng = SmallRng::seed_from_u64(11);
        let mut f = McLocalizer::new_at(&cfg, map.clone(), truth, &mut rng).unwrap();
        let mut observations = perfect_observations(&cfg, &map, &truth);
        observations.push(RangeBearing {
            range: 100.0,
            bearing: -2.7,
        });

        for _ in 0..(cfg.recovery_sustained_updates * 4) {
            let report = f.observe(&observations, &mut rng);
            assert!(!report.injected, "a minority outlier triggered recovery");
            assert_eq!(report.low_fit_streak, 0);
        }
        assert_eq!(f.stats().injections, 0);
        assert!(f.pose().distance_to(&truth) < 0.1);
    }

    #[test]
    fn empty_scans_do_not_advance_recovery_and_one_bad_observation_does() {
        let cfg = MclConfig {
            num_particles: 100,
            recovery_sustained_updates: 3,
            ..Default::default()
        };
        let mut rng = SmallRng::seed_from_u64(13);
        let mut f = McLocalizer::new_at(&cfg, square_map(), Se2::IDENTITY, &mut rng).unwrap();

        for _ in 0..5 {
            let report = f.observe(&[], &mut rng);
            assert_eq!(report.best_fit_median_log_likelihood, None);
            assert_eq!(report.low_fit_streak, 0);
            assert!(!report.injected);
        }

        let bad = [RangeBearing {
            range: 100.0,
            bearing: 2.9,
        }];
        for expected_streak in 1..cfg.recovery_sustained_updates {
            let report = f.observe(&bad, &mut rng);
            assert_eq!(report.low_fit_streak, expected_streak);
            assert!(!report.injected);
        }
        let report = f.observe(&bad, &mut rng);
        assert!(report.injected);
        assert_eq!(report.low_fit_streak, 0);
        assert_eq!(f.stats().injections, 1);
    }

    fn unexplainable_scan() -> Vec<RangeBearing> {
        // Three detections at ranges and bearings no pose in the map can all produce.
        vec![
            RangeBearing {
                range: 0.4,
                bearing: 0.1,
            },
            RangeBearing {
                range: 0.4,
                bearing: 2.0,
            },
            RangeBearing {
                range: 0.4,
                bearing: -2.0,
            },
        ]
    }

    fn global_cfg(particles: usize) -> GlobalLocalizationConfig {
        GlobalLocalizationConfig {
            num_particles: particles,
            ..Default::default()
        }
    }

    #[test]
    fn a_uniform_prior_is_free_space_and_starts_a_search() {
        let cfg = MclConfig::default();
        let map = square_map();
        let mut rng = SmallRng::seed_from_u64(11);
        let f =
            McLocalizer::new_uniform_prior(&cfg, &global_cfg(3000), map.clone(), &mut rng).unwrap();
        assert_eq!(f.phase(), LocalizationPhase::GlobalSearch);
        assert_eq!(f.len(), 3000, "the search count, not the steady-state one");
        assert_eq!(f.global_status().unwrap().converged, None);
        let keepout = GlobalLocalizationConfig::default().landmark_keepout_m;
        for p in f.particles.states() {
            for l in map.landmarks() {
                assert!((l.position.x - p.x()).hypot(l.position.y - p.y()) >= keepout);
            }
        }
        // And it is a prior, not a point: the cloud covers the room.
        let spread = f.global_status().unwrap().spread;
        assert!(spread.sigma_xy.is_infinite(), "no scan has measured it yet");
        let w = f.particles.weights();
        let s = cloud_spread(f.particles.states(), &w);
        assert!(s.sigma_xy > 1.5 && s.sigma_theta > 3.0, "{s:?}");
    }

    #[test]
    fn an_invalid_global_configuration_is_an_error_not_a_panic() {
        let mut rng = SmallRng::seed_from_u64(1);
        let bad = GlobalLocalizationConfig {
            num_particles: 0,
            ..Default::default()
        };
        assert!(McLocalizer::new_uniform_prior(
            &MclConfig::default(),
            &bad,
            square_map(),
            &mut rng
        )
        .is_err());
    }

    #[test]
    fn the_kidnap_trigger_is_inert_while_searching_and_no_false_convergence_on_nonsense() {
        let cfg = MclConfig {
            recovery_sustained_updates: 3, // would fire quickly if it were armed
            ..Default::default()
        };
        let mut rng = SmallRng::seed_from_u64(5);
        let mut f = McLocalizer::new_uniform_prior(&cfg, &global_cfg(2000), square_map(), &mut rng)
            .unwrap();
        for _ in 0..60 {
            let r = f.observe(&unexplainable_scan(), &mut rng);
            assert!(
                !r.injected,
                "a search must never be reset by the kidnap trigger"
            );
            assert!(!r.converged_now);
            assert_eq!(r.phase, LocalizationPhase::GlobalSearch);
            assert_eq!(r.best_fit_median_log_likelihood, None);
        }
        assert_eq!(f.stats().injections, 0);
        assert_eq!(f.phase(), LocalizationPhase::GlobalSearch);
    }

    #[test]
    fn empty_scans_do_not_advance_a_search() {
        let mut rng = SmallRng::seed_from_u64(5);
        let mut f = McLocalizer::new_uniform_prior(
            &MclConfig::default(),
            &global_cfg(1000),
            square_map(),
            &mut rng,
        )
        .unwrap();
        for _ in 0..50 {
            f.observe(&[], &mut rng);
        }
        assert_eq!(f.global_status().unwrap().scored_updates, 0);
        assert_eq!(f.phase(), LocalizationPhase::GlobalSearch);
    }

    /// A robot driving a circle through the middle of the four pillars, observed without
    /// noise by every pillar at every scan. This is the easiest search there is, so what it
    /// pins down is the plumbing (declaration, shrink, handover, the armed trigger), not how
    /// reliable a search is; the reliability is measured by `examples/global_trials.rs`.
    #[test]
    fn a_search_converges_hands_over_and_re_arms_the_kidnap_trigger() {
        let cfg = MclConfig {
            num_particles: 1000,
            recovery_sustained_updates: 5,
            ..Default::default()
        };
        let map = square_map();
        let mut rng = SmallRng::seed_from_u64(21);
        let mut f =
            McLocalizer::new_uniform_prior(&cfg, &global_cfg(6000), map.clone(), &mut rng).unwrap();

        let (cx, cy, radius, step) = (0.0_f64, 0.0_f64, 0.6_f64, 0.05_f64);
        let pose_at = |k: usize| {
            let a = k as f64 * step / radius;
            Se2::new(
                cx + radius * a.cos(),
                cy + radius * a.sin(),
                a + std::f64::consts::FRAC_PI_2,
            )
        };
        let mut previous = pose_at(0);
        let mut converged_at = None;
        for k in 1..400 {
            let truth = pose_at(k);
            f.predict(&previous, &truth, &mut rng);
            previous = truth;
            let z = perfect_observations(&cfg, &map, &truth);
            let r = f.observe(&z, &mut rng);
            if r.converged_now {
                converged_at = Some((k, truth));
                break;
            }
        }
        let (k, truth) = converged_at.expect("the easy search should have converged");
        assert_eq!(f.phase(), LocalizationPhase::Tracking);
        assert_eq!(
            f.len(),
            cfg.num_particles,
            "thinned to the steady-state count"
        );
        let conv = f.global_status().unwrap().converged.unwrap();
        assert_eq!(conv.particles_before, 6000);
        assert_eq!(conv.particles_after, 1000);
        assert!(conv.travel_m > 0.0 && conv.scored_updates as usize <= k);
        assert!(
            f.pose().distance_to(&truth) < 0.15,
            "converged to {:?}, truth {:?} after {k} scans",
            f.pose(),
            truth
        );

        // Travel is the distance of the search, so it stops counting when the search ends.
        let frozen = f.global_status().unwrap().travel_m;
        f.predict(&Se2::IDENTITY, &Se2::new(1.0, 0.0, 0.0), &mut rng);
        assert_eq!(f.global_status().unwrap().travel_m, frozen);

        // The kidnap trigger is armed again, with the configured persistence.
        let mut injected = false;
        for _ in 0..(cfg.recovery_sustained_updates + 2) {
            injected |= f.observe(&unexplainable_scan(), &mut rng).injected;
        }
        assert!(
            injected,
            "after convergence the kidnap trigger must be live"
        );
    }

    #[test]
    fn standing_still_predicts_to_a_fixed_point() {
        let cfg = MclConfig::default();
        let mut rng = SmallRng::seed_from_u64(3);
        let mut f = McLocalizer::new_at(&cfg, square_map(), Se2::IDENTITY, &mut rng).unwrap();
        let before = f.pose();
        f.predict(&Se2::IDENTITY, &Se2::IDENTITY, &mut rng);
        assert_eq!(f.pose(), before);
    }
}
