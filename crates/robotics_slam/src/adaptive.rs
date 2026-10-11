//! The KISS-style adaptive correspondence threshold.
//!
//! A fixed ICP threshold is wrong for most of a run: too small when the robot turns fast and
//! the prediction was poor, too large when it is nearly still and every wall point has a
//! plausible wrong neighbour. The adaptive rule tracks how wrong the *prediction* has been.
//! After each registration, the prediction error `pred⁻¹ · estimate` has the model deviation
//!
//! ```text
//! δ = ‖Δt‖ + 2 r_max sin(|Δθ| / 2)
//! ```
//!
//! the worst displacement the missed motion could cause at the far end of the sweep. The
//! running `σ² = mean(δ²)` gives `τ = 3σ`. Samples under `min_motion` are noise and are not
//! counted (including them collapses `τ` to zero when the robot is still). Before any sample
//! the threshold is `initial_threshold`; it never goes below `min_threshold`, which is set
//! above the point spacing so a correspondence can still be found.
//!
//! The prediction the caller compares with is the last per-scan increment, never a velocity
//! scaled by the time between stamps (decision DS6: the Webots lidar measures on a regular
//! lattice and stamps irregularly).

use crate::SlamError;
use robotics_core::Se2;

/// Parameters of the adaptive threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdaptiveConfig {
    /// `τ` before any sample (m). Initial value 0.5, chosen.
    pub initial_threshold: f64,
    /// Samples with `δ` below this are ignored (m). Initial 0.01, the order of the lidar noise.
    pub min_motion: f64,
    /// Lower bound on `τ` (m). Initial 0.05, the point spacing at 3 m.
    pub min_threshold: f64,
    /// Upper bound on `τ` (m).
    pub max_threshold: f64,
}

impl Default for AdaptiveConfig {
    fn default() -> Self {
        Self {
            initial_threshold: 0.5,
            min_motion: 0.01,
            min_threshold: 0.05,
            max_threshold: 2.0,
        }
    }
}

/// The running estimate of the prediction error and the threshold it implies.
#[derive(Debug, Clone)]
pub struct AdaptiveThreshold {
    cfg: AdaptiveConfig,
    sum_sq: f64,
    count: usize,
}

impl AdaptiveThreshold {
    /// Start with no history.
    pub fn new(cfg: AdaptiveConfig) -> Result<Self, SlamError> {
        let ok = |v: f64| v.is_finite() && v > 0.0;
        for (what, v) in [
            ("initial_threshold", cfg.initial_threshold),
            ("min_motion", cfg.min_motion),
            ("min_threshold", cfg.min_threshold),
            ("max_threshold", cfg.max_threshold),
        ] {
            if !ok(v) {
                return Err(SlamError::InvalidParameter { what, value: v });
            }
        }
        if cfg.min_threshold > cfg.max_threshold {
            return Err(SlamError::InvalidParameter {
                what: "min_threshold above max_threshold",
                value: cfg.min_threshold,
            });
        }
        Ok(Self {
            cfg,
            sum_sq: 0.0,
            count: 0,
        })
    }

    /// The model deviation of a prediction error for a sensor of maximum range `r_max`.
    pub fn model_deviation(prediction_error: &Se2, r_max: f64) -> f64 {
        let t = prediction_error.translation().norm();
        t + 2.0 * r_max * (0.5 * prediction_error.theta().abs()).sin()
    }

    /// The current threshold `τ` (m).
    pub fn threshold(&self) -> f64 {
        if self.count == 0 {
            return self
                .cfg
                .initial_threshold
                .clamp(self.cfg.min_threshold, self.cfg.max_threshold);
        }
        let sigma = (self.sum_sq / self.count as f64).sqrt();
        (3.0 * sigma).clamp(self.cfg.min_threshold, self.cfg.max_threshold)
    }

    /// Record how far the last prediction was from the registration result
    /// (`prediction⁻¹ · estimate`), for a sensor of maximum range `r_max`.
    pub fn update(&mut self, prediction_error: &Se2, r_max: f64) {
        let d = Self::model_deviation(prediction_error, r_max);
        if d.is_finite() && d >= self.cfg.min_motion {
            self.sum_sq += d * d;
            self.count += 1;
        }
    }

    /// Number of samples that have entered the estimate.
    pub fn samples(&self) -> usize {
        self.count
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn model_deviation_is_translation_plus_the_far_end_of_the_rotation() {
        let e = Se2::new(0.03, 0.04, 0.2);
        let d = AdaptiveThreshold::model_deviation(&e, 3.5);
        assert_relative_eq!(d, 0.05 + 7.0 * (0.1f64).sin(), epsilon = 1e-12);
        assert_relative_eq!(
            AdaptiveThreshold::model_deviation(&Se2::IDENTITY, 3.5),
            0.0,
            epsilon = 1e-15
        );
    }

    #[test]
    fn threshold_starts_at_the_initial_value_and_becomes_three_sigma() {
        let mut a = AdaptiveThreshold::new(AdaptiveConfig::default()).unwrap();
        assert_relative_eq!(a.threshold(), 0.5);
        a.update(&Se2::new(0.1, 0.0, 0.0), 3.5);
        a.update(&Se2::new(0.2, 0.0, 0.0), 3.5);
        let sigma = ((0.01f64 + 0.04) / 2.0).sqrt();
        assert_relative_eq!(a.threshold(), 3.0 * sigma, epsilon = 1e-12);
        assert_eq!(a.samples(), 2);
    }

    #[test]
    fn tiny_deviations_are_ignored_so_a_still_robot_does_not_collapse_the_threshold() {
        let mut a = AdaptiveThreshold::new(AdaptiveConfig::default()).unwrap();
        a.update(&Se2::new(0.2, 0.0, 0.0), 3.5);
        let before = a.threshold();
        for _ in 0..1000 {
            a.update(&Se2::new(0.001, 0.0, 0.0), 3.5);
        }
        assert_eq!(a.samples(), 1);
        assert_relative_eq!(a.threshold(), before);
    }

    #[test]
    fn threshold_respects_its_bounds() {
        let mut a = AdaptiveThreshold::new(AdaptiveConfig::default()).unwrap();
        a.update(&Se2::new(0.011, 0.0, 0.0), 3.5);
        assert_relative_eq!(a.threshold(), 0.05);
        a.update(&Se2::new(50.0, 0.0, 0.0), 3.5);
        assert_relative_eq!(a.threshold(), 2.0);
    }

    #[test]
    fn invalid_configuration_is_refused() {
        let bad = AdaptiveConfig {
            min_threshold: 3.0,
            ..Default::default()
        };
        assert!(AdaptiveThreshold::new(bad).is_err());
        let bad = AdaptiveConfig {
            min_motion: f64::NAN,
            ..Default::default()
        };
        assert!(AdaptiveThreshold::new(bad).is_err());
    }
}
