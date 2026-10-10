//! EKF localization against a known landmark map (Chapter 11, Table 7.2 in Thrun,
//! Burgard and Fox: EKF localization with unknown correspondence).
//!
//! The filter is the generic [`Ekf`] of `robotics_filters` instantiated on `Se2`, so the
//! recursion, the Joseph form correction and the numerical checks are the ones Phase 2
//! already tests. This module only adds the two models the localization problem needs and
//! the data association between them:
//!
//! * [`OdometryMotion`]: prediction from a relative motion `u` measured by the base, with
//!   the noise model of chapter 5 expressed in the body frame of the predicted pose.
//! * [`LandmarkMeasurement`]: the range and bearing model of
//!   `robotics_models::sensor::landmark`, which already provides `h`, its Jacobian with
//!   respect to a right (body frame) perturbation, and the wrapped residual.
//! * [`EkfLocalizer::observe`]: maximum likelihood association with a chi-square gate.
//!
//! What the filter cannot do is as important as what it can. The belief is a single
//! Gaussian over one pose, so it can track a pose it already knows (position tracking) and
//! it cannot represent "I am in one of four places". After a kidnapping the gate rejects
//! every observation that no longer fits, the filter keeps the belief it had, and it stays
//! wrong. That is a property of the representation, not a tuning failure, and it is the
//! reason MCL exists.

use nalgebra::{Matrix2, Matrix3, SMatrix, Vector2};
use robotics_core::prob::stats::chi2_quantile;
use robotics_core::Se2;
use robotics_filters::ekf::{
    Ekf, MeasurementJacobian, MeasurementModel, ProcessJacobian, ProcessModel,
};
use robotics_filters::FilterError;
use robotics_models::sensor::landmark::{LandmarkMap, LandmarkModel, RangeBearing};

/// Errors of the localizer.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum LocalizationError {
    /// A configuration value is out of range.
    #[error("invalid configuration: {0}")]
    Config(String),
    /// The underlying filter refused a step.
    #[error("filter: {0}")]
    Filter(#[from] FilterError),
    /// The map has no landmarks.
    #[error("the map is empty")]
    EmptyMap,
}

/// Prediction from a relative motion of the base.
///
/// The control `u` is the motion in the body frame of the previous pose, so that
/// `x_k = x_{k-1} · u`. Because the composition is on the right, the error state Jacobian
/// is the adjoint of `u⁻¹` and the process noise is naturally expressed in the body frame
/// of the new pose, which is where this model builds it.
///
/// Noise follows chapter 5: the variance of the rotation grows with the rotation and with
/// the distance driven, and the variance of the translation grows with the distance driven
/// and with the rotation. `alphas` are the four coefficients of that model, in the order
/// `[rot from rot, rot from trans, trans from trans, trans from rot]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryMotion {
    /// Noise coefficients of the odometry motion model.
    pub alphas: [f64; 4],
    /// Lateral (sideways) variance as a fraction of the along-track variance. A
    /// differential drive cannot move sideways, so this is small but never zero: it is the
    /// slip the model admits to.
    pub lateral_fraction: f64,
    /// Floor on the translation standard deviation per step (m), so that a standing robot
    /// still has a non-degenerate covariance.
    pub min_translation_sigma: f64,
    /// Floor on the rotation standard deviation per step (rad).
    pub min_rotation_sigma: f64,
}

impl Default for OdometryMotion {
    fn default() -> Self {
        Self {
            alphas: [0.05, 0.02, 0.05, 0.02],
            lateral_fraction: 0.1,
            min_translation_sigma: 1e-3,
            min_rotation_sigma: 1e-3,
        }
    }
}

impl ProcessModel<Se2, Se2, 3> for OdometryMotion {
    fn propagate(&self, x: &Se2, u: &Se2) -> Se2 {
        x.compose(u)
    }

    fn process_noise(&self, _x: &Se2, u: &Se2) -> Matrix3<f64> {
        let trans = u.translation().norm();
        let rot = u.theta().abs();
        let [a1, a2, a3, a4] = self.alphas;
        let var_along = a3 * trans * trans + a4 * rot * rot + self.min_translation_sigma.powi(2);
        let var_lateral = self.lateral_fraction * var_along + self.min_translation_sigma.powi(2);
        let var_rot = a1 * rot * rot + a2 * trans * trans + self.min_rotation_sigma.powi(2);
        Matrix3::from_diagonal(&nalgebra::Vector3::new(var_along, var_lateral, var_rot))
    }
}

impl ProcessJacobian<Se2, Se2, 3> for OdometryMotion {
    fn jacobian(&self, _x: &Se2, u: &Se2) -> Matrix3<f64> {
        u.inverse().adjoint()
    }
}

/// The range and bearing measurement of one known landmark.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandmarkMeasurement {
    model: LandmarkModel,
    position: Vector2<f64>,
}

impl MeasurementModel<Se2, 3, 2> for LandmarkMeasurement {
    fn predict(&self, x: &Se2) -> Vector2<f64> {
        // The caller validates the geometry before correcting, so the error branch is only
        // reached for a landmark sitting exactly under the sensor.
        match self.model.predict(x, &self.position) {
            Ok(z) => Vector2::new(z.range, z.bearing),
            Err(_) => Vector2::zeros(),
        }
    }

    fn measurement_noise(&self, _x: &Se2) -> Matrix2<f64> {
        self.model.covariance()
    }

    fn residual(&self, z: &Vector2<f64>, z_hat: &Vector2<f64>) -> Vector2<f64> {
        LandmarkModel::residual(
            &RangeBearing {
                range: z[0],
                bearing: z[1],
            },
            &RangeBearing {
                range: z_hat[0],
                bearing: z_hat[1],
            },
        )
    }
}

impl MeasurementJacobian<Se2, 3, 2> for LandmarkMeasurement {
    fn jacobian(&self, x: &Se2) -> SMatrix<f64, 2, 3> {
        self.model
            .jacobian(x, &self.position)
            .unwrap_or_else(|_| SMatrix::zeros())
    }
}

/// Configuration of [`EkfLocalizer`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalizerConfig {
    /// Range noise of one landmark observation (m).
    pub sigma_range: f64,
    /// Bearing noise of one landmark observation (rad).
    pub sigma_bearing: f64,
    /// Motion model.
    pub motion: OdometryMotion,
    /// Probability mass inside the association gate, for example 0.99.
    pub gate_probability: f64,
    /// Initial position standard deviation (m).
    pub initial_sigma_xy: f64,
    /// Initial heading standard deviation (rad).
    pub initial_sigma_theta: f64,
    /// Pose of the laser in the body frame.
    pub sensor_pose: Se2,
}

impl Default for LocalizerConfig {
    fn default() -> Self {
        Self {
            sigma_range: 0.05,
            sigma_bearing: 0.05,
            motion: OdometryMotion::default(),
            gate_probability: 0.99,
            initial_sigma_xy: 0.05,
            initial_sigma_theta: 0.05,
            sensor_pose: Se2::IDENTITY,
        }
    }
}

/// What one measurement update did, for diagnostics and for the evidence files.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct UpdateReport {
    /// Observations handed in.
    pub observed: usize,
    /// Observations that passed the gate and corrected the filter.
    pub accepted: usize,
    /// Observations rejected by the gate, or whose correction the filter refused.
    pub rejected: usize,
    /// Mean normalized innovation squared over the accepted observations (expected 2).
    pub mean_nis: Option<f64>,
    /// `(landmark id, NIS)` of every accepted observation, in order.
    pub associations: Vec<(u32, f64)>,
}

/// Counters over the lifetime of the filter.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LocalizerStats {
    /// Prediction steps applied.
    pub predictions: u64,
    /// Scans handed to [`EkfLocalizer::observe`].
    pub updates: u64,
    /// Observations accepted in total.
    pub accepted: u64,
    /// Observations rejected in total.
    pub rejected: u64,
    /// Sum of the NIS of accepted observations, for the running mean.
    pub nis_sum: f64,
}

/// EKF localization on SE(2) against a known landmark map.
#[derive(Debug, Clone)]
pub struct EkfLocalizer {
    ekf: Ekf<Se2, 3>,
    map: LandmarkMap,
    model: LandmarkModel,
    gate: f64,
    stats: LocalizerStats,
}

impl EkfLocalizer {
    /// Build a localizer around an initial pose guess.
    pub fn new(
        cfg: &LocalizerConfig,
        map: LandmarkMap,
        initial_pose: Se2,
    ) -> Result<Self, LocalizationError> {
        if map.is_empty() {
            return Err(LocalizationError::EmptyMap);
        }
        if !(cfg.gate_probability > 0.0 && cfg.gate_probability < 1.0) {
            return Err(LocalizationError::Config(
                "gate_probability must be in (0, 1)".into(),
            ));
        }
        let model = LandmarkModel::new(cfg.sigma_range, cfg.sigma_bearing)
            .and_then(|m| m.with_sensor_pose(cfg.sensor_pose))
            .map_err(|e| LocalizationError::Config(e.to_string()))?;
        let p = Matrix3::from_diagonal(&nalgebra::Vector3::new(
            cfg.initial_sigma_xy.powi(2),
            cfg.initial_sigma_xy.powi(2),
            cfg.initial_sigma_theta.powi(2),
        ));
        let gate = chi2_quantile(2, cfg.gate_probability)
            .map_err(|e| LocalizationError::Config(e.to_string()))?;
        Ok(Self {
            ekf: Ekf::new(initial_pose, p)?,
            map,
            model,
            gate,
            stats: LocalizerStats::default(),
        })
    }

    /// The relative motion between two consecutive odometry poses, in the body frame of
    /// the earlier one, which is the control [`EkfLocalizer::predict`] expects.
    pub fn relative_motion(previous: &Se2, current: &Se2) -> Se2 {
        previous.between(current)
    }

    /// Current estimate.
    pub fn pose(&self) -> Se2 {
        *self.ekf.state()
    }

    /// Current covariance, in the body frame of the estimate.
    pub fn covariance(&self) -> Matrix3<f64> {
        *self.ekf.covariance()
    }

    /// Lifetime counters.
    pub fn stats(&self) -> LocalizerStats {
        self.stats
    }

    /// Mean NIS over every accepted observation so far (expected 2 for a consistent
    /// filter).
    pub fn mean_nis(&self) -> Option<f64> {
        (self.stats.accepted > 0).then(|| self.stats.nis_sum / self.stats.accepted as f64)
    }

    /// Chi-square gate in use.
    pub fn gate(&self) -> f64 {
        self.gate
    }

    /// Prediction step with the relative motion `u` of the base.
    pub fn predict(&mut self, motion: &OdometryMotion, u: &Se2) -> Result<(), LocalizationError> {
        self.ekf.predict(motion, u)?;
        self.stats.predictions += 1;
        Ok(())
    }

    /// Correction step with the landmark observations of one scan, associating each
    /// observation with the landmark that explains it best and rejecting those that no
    /// landmark explains.
    pub fn observe(&mut self, observations: &[RangeBearing]) -> UpdateReport {
        let mut report = UpdateReport {
            observed: observations.len(),
            ..Default::default()
        };
        self.stats.updates += 1;
        let mut nis_sum = 0.0;

        for z in observations {
            if !(z.range.is_finite() && z.bearing.is_finite()) {
                report.rejected += 1;
                continue;
            }
            let zv = Vector2::new(z.range, z.bearing);
            let pose = self.pose();
            let p = self.covariance();

            // Maximum likelihood association: the landmark with the smallest NIS.
            let mut best: Option<(u32, Vector2<f64>, f64)> = None;
            for landmark in self.map.landmarks() {
                let (Ok(predicted), Ok(h)) = (
                    self.model.predict(&pose, &landmark.position),
                    self.model.jacobian(&pose, &landmark.position),
                ) else {
                    continue;
                };
                let nu = LandmarkModel::residual(z, &predicted);
                let s = h * p * h.transpose() + self.model.covariance();
                let Some(s_inv) = s.try_inverse() else {
                    continue;
                };
                let nis = (nu.transpose() * s_inv * nu)[(0, 0)];
                if !nis.is_finite() {
                    continue;
                }
                if best.map(|(_, _, b)| nis < b).unwrap_or(true) {
                    best = Some((landmark.id, landmark.position, nis));
                }
            }

            let Some((id, position, nis)) = best else {
                report.rejected += 1;
                continue;
            };
            if nis > self.gate {
                report.rejected += 1;
                continue;
            }
            let measurement = LandmarkMeasurement {
                model: self.model,
                position,
            };
            match self.ekf.correct(&measurement, &zv) {
                Ok(c) => {
                    report.accepted += 1;
                    report.associations.push((id, c.nis));
                    nis_sum += c.nis;
                    self.stats.nis_sum += c.nis;
                }
                Err(_) => report.rejected += 1,
            }
        }

        self.stats.accepted += report.accepted as u64;
        self.stats.rejected += report.rejected as u64;
        if report.accepted > 0 {
            report.mean_nis = Some(nis_sum / report.accepted as f64);
        }
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn square_map() -> LandmarkMap {
        use robotics_models::sensor::landmark::Landmark;
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
        ])
        .unwrap()
    }

    /// Observations a perfect sensor would produce at `pose`.
    fn perfect_observations(
        cfg: &LocalizerConfig,
        map: &LandmarkMap,
        pose: &Se2,
    ) -> Vec<RangeBearing> {
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
    fn prediction_alone_keeps_the_mean_and_grows_the_covariance() {
        let cfg = LocalizerConfig::default();
        let mut f = EkfLocalizer::new(&cfg, square_map(), Se2::IDENTITY).unwrap();
        let before = f.covariance()[(0, 0)];
        let u = Se2::new(0.5, 0.0, 0.0);
        f.predict(&cfg.motion, &u).unwrap();
        assert_relative_eq!(f.pose().x(), 0.5, epsilon = 1e-12);
        assert!(
            f.covariance()[(0, 0)] > before,
            "dead reckoning must become less certain"
        );
    }

    #[test]
    fn observations_pull_a_wrong_estimate_back_to_the_truth() {
        let cfg = LocalizerConfig {
            initial_sigma_xy: 0.3,
            initial_sigma_theta: 0.2,
            ..Default::default()
        };
        let map = square_map();
        let truth = Se2::new(0.4, -0.2, 0.3);
        // The filter starts 25 cm and 10 degrees away from the truth.
        let mut f = EkfLocalizer::new(&cfg, map.clone(), Se2::new(0.2, -0.05, 0.13)).unwrap();
        let z = perfect_observations(&cfg, &map, &truth);
        for _ in 0..5 {
            let r = f.observe(&z);
            assert_eq!(r.accepted, 3, "every landmark should be associated");
        }
        assert!(
            f.pose().distance_to(&truth) < 0.02,
            "estimate {:?} should have converged to {:?}",
            f.pose(),
            truth
        );
    }

    #[test]
    fn association_picks_the_right_landmark() {
        let cfg = LocalizerConfig::default();
        let map = square_map();
        let truth = Se2::new(0.1, 0.1, 0.0);
        let mut f = EkfLocalizer::new(&cfg, map.clone(), truth).unwrap();
        let z = perfect_observations(&cfg, &map, &truth);
        let r = f.observe(&z);
        let ids: Vec<u32> = r.associations.iter().map(|(id, _)| *id).collect();
        assert_eq!(ids, vec![0, 1, 2]);
        assert!(
            r.mean_nis.unwrap() < 1e-6,
            "a perfect measurement has no innovation"
        );
    }

    #[test]
    fn the_gate_rejects_an_observation_no_landmark_explains() {
        let cfg = LocalizerConfig::default();
        let mut f = EkfLocalizer::new(&cfg, square_map(), Se2::IDENTITY).unwrap();
        let r = f.observe(&[RangeBearing {
            range: 9.0,
            bearing: 2.5,
        }]);
        assert_eq!((r.accepted, r.rejected), (0, 1));
    }

    #[test]
    fn a_teleported_robot_is_not_recovered_by_a_single_gaussian() {
        // The honest limit of this filter: after a jump the observations no longer fit the
        // belief, the gate throws them away, and the estimate stays where it was.
        let cfg = LocalizerConfig::default();
        let map = square_map();
        let mut f = EkfLocalizer::new(&cfg, map.clone(), Se2::IDENTITY).unwrap();
        let z = perfect_observations(&cfg, &map, &Se2::IDENTITY);
        f.observe(&z);
        let before = f.pose();

        let kidnapped_to = Se2::new(-1.5, 1.5, 2.0);
        let z = perfect_observations(&cfg, &map, &kidnapped_to);
        let r = f.observe(&z);
        assert_eq!(r.accepted, 0, "nothing should fit after the jump");
        assert!(
            f.pose().distance_to(&before) < 1e-9,
            "the belief must stay where it was"
        );
        assert!(
            f.pose().distance_to(&kidnapped_to) > 1.0,
            "and therefore stay wrong"
        );
    }
}
