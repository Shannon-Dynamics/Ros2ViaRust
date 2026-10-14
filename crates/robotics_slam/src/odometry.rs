//! LiDAR odometry: scan-to-local-map registration with an adaptive threshold.
//!
//! The front end of the graph. Each scan is downsampled, registered by point-to-plane ICP
//! against a [`VoxelMap`] of the recent past, starting from a constant-velocity prediction,
//! and then added to that map. The correspondence threshold comes from the prediction error
//! of the previous scans ([`AdaptiveThreshold`]).
//!
//! * The pose tracked here is the **sensor** pose in the odometry frame, starting at the
//!   identity. The caller owns the static `base_link → sensor` transform.
//! * The prediction is the last per-scan increment, applied once. It is never scaled by the
//!   time between stamps (decision DS6).
//! * A scan is one instant: no deskewing (decision D9 of Phase 4, a property of the Webots
//!   driver).
//! * A registration that does not settle at the adaptive threshold is tried once more at the
//!   no-knowledge threshold (`adaptive.initial_threshold`) from the same guess, if the adaptive
//!   one is smaller. Without this the front end deadlocks: after a long straight the threshold is
//!   a few centimetres, the first turning scan needs several times that, ICP crawls and hits its
//!   iteration limit, and a failed step used to leave the threshold where it was, so no later
//!   step could do better (found by the first evidence run on `route_eval_cross.yaml`).
//! * A registration that settles but has **no information** (the matrix is not positive definite:
//!   no correspondence had a usable line normal, so nothing constrains it) is believed for its pose
//!   and reports the configured fallback information instead, with
//!   [`StepResult::information_fallback`] set. Without this its zero matrix cannot be inverted into a
//!   covariance, and a consumer that needs one (the pose graph's edge, a message) silently loses the
//!   increment (found by replaying a recorded Webots stream through the nodes).
//! * A registration that still does not settle is not believed: the pose advances by the
//!   prediction, the scan is **not** added to the map, the threshold is not updated, and the
//!   step reports [`TrackingStatus::Predicted`] with a fallback information matrix.

use crate::adaptive::{AdaptiveConfig, AdaptiveThreshold};
use crate::icp::{register, scan_points, IcpConfig, IcpMetric, IcpResult};
use crate::voxel::{downsample, VoxelMap};
use crate::SlamError;
use nalgebra::{Matrix3, Vector3};
use robotics_core::{LaserScan, Se2};

/// Front-end parameters. Initial values, frozen after the development run (DS18).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryConfig {
    /// Voxel size of the scan downsampling (m). Derived: the beam spacing at 3 m.
    pub downsample: f64,
    /// Cell of the local map (m).
    pub map_cell: f64,
    /// Points kept per local-map cell.
    pub map_max_per_cell: usize,
    /// Points farther than this from the current sensor are dropped from the map (m).
    pub map_radius: f64,
    /// Fewest downsampled points for a scan to be registered.
    pub min_points: usize,
    /// The adaptive threshold.
    pub adaptive: AdaptiveConfig,
    /// ICP settings; `threshold` is overwritten per scan.
    pub icp: IcpConfig,
    /// Translation standard deviation reported for a predicted (unregistered) step (m).
    pub fallback_sigma_trans: f64,
    /// Rotation standard deviation reported for a predicted step (rad).
    pub fallback_sigma_rot: f64,
}

impl Default for OdometryConfig {
    fn default() -> Self {
        Self {
            downsample: 0.05,
            map_cell: 0.10,
            map_max_per_cell: 8,
            map_radius: 6.0,
            min_points: 30,
            adaptive: AdaptiveConfig::default(),
            icp: IcpConfig {
                metric: IcpMetric::PointToPlane,
                ..IcpConfig::default()
            },
            fallback_sigma_trans: 0.1,
            fallback_sigma_rot: 0.05,
        }
    }
}

/// How a step was resolved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingStatus {
    /// The first scan: the map was started, the pose is the identity.
    Initialized,
    /// ICP converged and its result was used.
    Tracked,
    /// ICP did not converge; the prediction was used.
    Predicted,
}

/// What one scan did.
#[derive(Debug, Clone, Copy)]
pub struct StepResult {
    /// The sensor pose in the odometry frame after this scan.
    pub pose: Se2,
    /// The increment from the previous pose (previous⁻¹ · pose).
    pub delta: Se2,
    /// Information of `delta` in its own tangent: the registration's, or the fallback's.
    pub information: Matrix3<f64>,
    /// How the step was resolved.
    pub status: TrackingStatus,
    /// `information` is the configured fallback, not the registration's: true for the first scan, for
    /// a predicted step, and for a settled registration whose own information was not positive
    /// definite.
    pub information_fallback: bool,
    /// The correspondence threshold of the registration that was used (m).
    pub threshold: f64,
    /// The registration at the adaptive threshold did not settle and the retry at the
    /// no-knowledge threshold was the one used (or tried).
    pub retried: bool,
    /// The registration, when one was run.
    pub icp: Option<IcpResult>,
}

/// The front end.
#[derive(Debug, Clone)]
pub struct LidarOdometry {
    cfg: OdometryConfig,
    map: VoxelMap,
    adaptive: AdaptiveThreshold,
    pose: Se2,
    last_delta: Se2,
    started: bool,
}

impl LidarOdometry {
    /// A front end with no history.
    pub fn new(cfg: OdometryConfig) -> Result<Self, SlamError> {
        cfg.icp.validate()?;
        for (what, v) in [
            ("downsample", cfg.downsample),
            ("map_radius", cfg.map_radius),
            ("fallback_sigma_trans", cfg.fallback_sigma_trans),
            ("fallback_sigma_rot", cfg.fallback_sigma_rot),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return Err(SlamError::InvalidParameter { what, value: v });
            }
        }
        Ok(Self {
            map: VoxelMap::new(cfg.map_cell, cfg.map_max_per_cell)?,
            adaptive: AdaptiveThreshold::new(cfg.adaptive)?,
            cfg,
            pose: Se2::IDENTITY,
            last_delta: Se2::IDENTITY,
            started: false,
        })
    }

    /// The current sensor pose in the odometry frame.
    pub fn pose(&self) -> Se2 {
        self.pose
    }

    /// Points in the local map.
    pub fn map_len(&self) -> usize {
        self.map.len()
    }

    /// The current correspondence threshold (m).
    pub fn threshold(&self) -> f64 {
        self.adaptive.threshold()
    }

    fn fallback_information(&self) -> Matrix3<f64> {
        let (t, r) = (self.cfg.fallback_sigma_trans, self.cfg.fallback_sigma_rot);
        Matrix3::from_diagonal(&Vector3::new(1.0 / (t * t), 1.0 / (t * t), 1.0 / (r * r)))
    }

    /// Process one scan.
    pub fn step(&mut self, scan: &LaserScan) -> Result<StepResult, SlamError> {
        let pts = downsample(&scan_points(scan), self.cfg.downsample)?;
        if pts.len() < self.cfg.min_points {
            return Err(SlamError::EmptyInput("too few valid points in the scan"));
        }
        if !self.started {
            self.started = true;
            self.map.insert_all(&pts);
            return Ok(StepResult {
                pose: self.pose,
                delta: Se2::IDENTITY,
                information: self.fallback_information(),
                information_fallback: true,
                status: TrackingStatus::Initialized,
                threshold: self.adaptive.threshold(),
                retried: false,
                icp: None,
            });
        }
        let threshold = self.adaptive.threshold();
        let guess = self.pose.compose(&self.last_delta);
        let icp_cfg = IcpConfig {
            threshold,
            ..self.cfg.icp
        };
        let mut r = register(&pts, &self.map, guess, &icp_cfg)?;
        let mut used_threshold = threshold;
        let mut retried = false;
        if !r.settled && threshold < self.cfg.adaptive.initial_threshold {
            retried = true;
            let wide = IcpConfig {
                threshold: self.cfg.adaptive.initial_threshold,
                ..self.cfg.icp
            };
            let again = register(&pts, &self.map, guess, &wide)?;
            if again.settled {
                r = again;
                used_threshold = wide.threshold;
            }
        }
        let own_information = r.settled && r.information.cholesky().is_some();
        let (new_pose, status, information) = if r.settled {
            let information = if own_information {
                r.information
            } else {
                self.fallback_information()
            };
            (r.pose, TrackingStatus::Tracked, information)
        } else {
            (
                guess,
                TrackingStatus::Predicted,
                self.fallback_information(),
            )
        };
        let delta = self.pose.between(&new_pose);
        if status == TrackingStatus::Tracked {
            self.adaptive
                .update(&self.last_delta.between(&delta), scan.range_max);
            for p in &pts {
                self.map.insert(new_pose.act(p));
            }
            self.map
                .retain_within(&new_pose.translation(), self.cfg.map_radius);
        }
        self.pose = new_pose;
        self.last_delta = delta;
        Ok(StepResult {
            pose: new_pose,
            delta,
            information,
            status,
            information_fallback: !own_information,
            threshold: used_threshold,
            retried,
            icp: Some(r),
        })
    }
}

/// Compounds the increments between two keyframes into one relative pose and its information.
///
/// For `T = T₁·T₂` with right perturbations the covariance compounds as
/// `Σ = Ad(T₂⁻¹) Σ₁ Ad(T₂⁻¹)ᵀ + Σ₂`. Each increment's covariance is taken as the inverse of
/// the registration's information, an approximation (the registration is against a map, not
/// against the previous scan) that is as overconfident as that information is.
#[derive(Debug, Clone)]
pub struct EdgeAccumulator {
    delta: Se2,
    cov: Matrix3<f64>,
}

impl Default for EdgeAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl EdgeAccumulator {
    /// An empty accumulator (identity, zero covariance).
    pub fn new() -> Self {
        Self {
            delta: Se2::IDENTITY,
            cov: Matrix3::zeros(),
        }
    }

    /// Append one increment with its information. Errors if the information is singular.
    pub fn push(&mut self, delta: &Se2, information: &Matrix3<f64>) -> Result<(), SlamError> {
        let cov = information
            .try_inverse()
            .ok_or(SlamError::SingularSystem("increment information"))?;
        let ad = delta.inverse().adjoint();
        self.cov = ad * self.cov * ad.transpose() + cov;
        self.delta = self.delta.compose(delta);
        Ok(())
    }

    /// The compounded relative pose and its information, and reset.
    pub fn take(&mut self) -> Result<(Se2, Matrix3<f64>), SlamError> {
        let info = self
            .cov
            .try_inverse()
            .ok_or(SlamError::SingularSystem("accumulated covariance"))?;
        let out = (self.delta, info);
        *self = Self::new();
        Ok(out)
    }

    /// The relative pose accumulated so far.
    pub fn delta(&self) -> Se2 {
        self.delta
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn compounding_two_increments_matches_the_closed_form() {
        let (d1, d2) = (Se2::new(0.5, 0.0, 0.2), Se2::new(0.4, 0.1, -0.1));
        let (i1, i2) = (
            Matrix3::from_diagonal(&Vector3::new(100.0, 100.0, 400.0)),
            Matrix3::from_diagonal(&Vector3::new(50.0, 80.0, 200.0)),
        );
        let mut acc = EdgeAccumulator::new();
        acc.push(&d1, &i1).unwrap();
        acc.push(&d2, &i2).unwrap();
        let (d, info) = acc.take().unwrap();
        let ad = d2.inverse().adjoint();
        let expect = ad * i1.try_inverse().unwrap() * ad.transpose() + i2.try_inverse().unwrap();
        assert!((d.between(&d1.compose(&d2)).log()).norm() < 1e-12);
        assert!((info.try_inverse().unwrap() - expect).abs().max() < 1e-12);
        // Taking resets.
        assert_relative_eq!(acc.delta().x(), 0.0);
        assert!(acc.take().is_err());
    }

    #[test]
    fn bad_configuration_is_refused() {
        let cfg = OdometryConfig {
            downsample: 0.0,
            ..Default::default()
        };
        assert!(LidarOdometry::new(cfg).is_err());
    }
}
