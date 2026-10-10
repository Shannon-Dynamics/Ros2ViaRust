//! Range–bearing landmark model with known correspondence.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 10 (`LandmarkModel`,
//! `landmark_model_known_correspondence`, `sample_landmark_model_known_correspondence`);
//! Thrun, Burgard & Fox, Tables 6.4 and 6.5.
//!
//! A feature detector reports, for landmark `j`, a range `r` and a bearing `φ` in the
//! **sensor** frame. With the sensor mounted at `T_base_sensor` on a robot at pose `x`,
//! the landmark position in the sensor frame is `p = (x · T_base_sensor)⁻¹ m_j` and
//!
//! ```text
//! r̂ = ‖p‖,   φ̂ = atan2(p_y, p_x),
//! p(r, φ | x, m_j) = N(r − r̂; 0, σ_r²) · N(wrap(φ − φ̂); 0, σ_φ²).
//! ```
//!
//! The bearing residual is **always wrapped** to `(−π, π]` before it is scored: a
//! prediction of 179° and a reading of −179° are 2° apart, not 358°.
//!
//! # Correspondence
//! [`LandmarkObservation::id`] carries the landmark identity when the front end knows it
//! (a fiducial id, a signature class) — the "known correspondence" case of Chapter 10. The
//! book's continuous signature `s` is represented by the discrete id because every
//! detector this framework targets (AprilTags, reflectors, beacons) reports identities;
//! unknown correspondence (`id = None`) is the data-association problem of Chapter 11 and
//! is rejected here with an error rather than guessed.
//!
//! The model is only defined for landmarks away from the sensor origin: at `r̂ → 0` the
//! bearing is undefined and [`ModelError::DegenerateGeometry`] is returned.

use crate::ModelError;
use nalgebra::{Matrix2, SMatrix, Vector2};
use rand::{Rng, RngExt};
use rand_distr::{Distribution, Normal};
use robotics_core::angle::{angle_diff, normalize_angle};
use robotics_core::Se2;
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

/// Below this sensor-to-landmark distance (m) the bearing is considered undefined.
pub const MIN_RANGE: f64 = 1e-9;

/// A point landmark in the map frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Landmark {
    /// Identity (signature) of the landmark.
    pub id: u32,
    /// Position in the map frame (m).
    pub position: Vector2<f64>,
}

/// A set of landmarks indexed by id.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LandmarkMap {
    landmarks: Vec<Landmark>,
    index: HashMap<u32, usize>,
}

impl LandmarkMap {
    /// Build from landmarks; ids must be unique and positions finite.
    pub fn new(landmarks: Vec<Landmark>) -> Result<Self, ModelError> {
        let mut index = HashMap::with_capacity(landmarks.len());
        for (i, l) in landmarks.iter().enumerate() {
            if !(l.position.x.is_finite() && l.position.y.is_finite()) {
                return Err(ModelError::InvalidData("landmark position must be finite"));
            }
            if index.insert(l.id, i).is_some() {
                return Err(ModelError::InvalidData("duplicate landmark id"));
            }
        }
        Ok(Self { landmarks, index })
    }
    /// Look a landmark up by id.
    pub fn get(&self, id: u32) -> Option<&Landmark> {
        self.index.get(&id).map(|&i| &self.landmarks[i])
    }
    /// All landmarks.
    pub fn landmarks(&self) -> &[Landmark] {
        &self.landmarks
    }
    /// Number of landmarks.
    pub fn len(&self) -> usize {
        self.landmarks.len()
    }
    /// `true` without landmarks.
    pub fn is_empty(&self) -> bool {
        self.landmarks.is_empty()
    }
}

/// A range–bearing pair in the sensor frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RangeBearing {
    /// Range (m).
    pub range: f64,
    /// Bearing (rad), counter-clockwise from the sensor's x axis, in `(−π, π]`.
    pub bearing: f64,
}

/// One detected feature.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandmarkObservation {
    /// Known correspondence, if the detector reports one.
    pub id: Option<u32>,
    /// The measurement.
    pub z: RangeBearing,
}

/// The measurement model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandmarkModel {
    sigma_range: f64,
    sigma_bearing: f64,
    sensor: Se2,
}

impl LandmarkModel {
    /// Noise standard deviations (range in m, bearing in rad); both positive and finite.
    /// The sensor sits at the robot origin until [`LandmarkModel::with_sensor_pose`].
    pub fn new(sigma_range: f64, sigma_bearing: f64) -> Result<Self, ModelError> {
        for (what, value) in [
            ("sigma_range", sigma_range),
            ("sigma_bearing", sigma_bearing),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return Err(ModelError::NonPositive { what, value });
            }
        }
        if sigma_bearing >= PI {
            return Err(ModelError::InvalidData(
                "sigma_bearing must be well below pi for a wrapped Gaussian",
            ));
        }
        Ok(Self {
            sigma_range,
            sigma_bearing,
            sensor: Se2::IDENTITY,
        })
    }

    /// Mount the sensor at `T_base_sensor`.
    pub fn with_sensor_pose(mut self, t_base_sensor: Se2) -> Result<Self, ModelError> {
        if !t_base_sensor.is_finite() {
            return Err(ModelError::InvalidData("sensor pose must be finite"));
        }
        self.sensor = t_base_sensor;
        Ok(self)
    }

    /// Measurement noise covariance `Q = diag(σ_r², σ_φ²)`.
    pub fn covariance(&self) -> Matrix2<f64> {
        Matrix2::new(
            self.sigma_range.powi(2),
            0.0,
            0.0,
            self.sigma_bearing.powi(2),
        )
    }

    fn in_sensor(&self, pose: &Se2, landmark: &Vector2<f64>) -> Result<Vector2<f64>, ModelError> {
        if !pose.is_finite() || !(landmark.x.is_finite() && landmark.y.is_finite()) {
            return Err(ModelError::InvalidData("pose and landmark must be finite"));
        }
        let p = pose.compose(&self.sensor).act_inverse(landmark);
        if p.norm() < MIN_RANGE {
            return Err(ModelError::DegenerateGeometry(
                "landmark at the sensor origin has no bearing",
            ));
        }
        Ok(p)
    }

    /// Predicted measurement `ĥ(x, m_j)`.
    pub fn predict(&self, pose: &Se2, landmark: &Vector2<f64>) -> Result<RangeBearing, ModelError> {
        let p = self.in_sensor(pose, landmark)?;
        Ok(RangeBearing {
            range: p.norm(),
            bearing: p.y.atan2(p.x),
        })
    }

    /// Residual `z ⊟ ẑ = (r − r̂, wrap(φ − φ̂))`.
    pub fn residual(z: &RangeBearing, predicted: &RangeBearing) -> Vector2<f64> {
        Vector2::new(
            z.range - predicted.range,
            angle_diff(z.bearing, predicted.bearing),
        )
    }

    /// Jacobian `∂ĥ/∂δ` of the prediction with respect to a right (body-frame)
    /// perturbation of the pose, `x ⊞ δ = x·exp(δ)`, `δ = (v_x, v_y, ω)` — the `H` an EKF
    /// on SE(2) needs.
    pub fn jacobian(
        &self,
        pose: &Se2,
        landmark: &Vector2<f64>,
    ) -> Result<SMatrix<f64, 2, 3>, ModelError> {
        let p = self.in_sensor(pose, landmark)?;
        let pb = pose.act_inverse(landmark);
        // exp(−δ) p_b ≈ p_b − (v + ω × p_b)  ⇒  ∂p_b/∂δ = [[−1, 0, p_by], [0, −1, −p_bx]]
        let dpb = SMatrix::<f64, 2, 3>::new(-1.0, 0.0, pb.y, 0.0, -1.0, -pb.x);
        let rt = self.sensor.rotation().inverse().into_inner();
        let dp = rt * dpb;
        let r2 = p.norm_squared();
        let r = r2.sqrt();
        let dh = Matrix2::new(p.x / r, p.y / r, -p.y / r2, p.x / r2);
        Ok(dh * dp)
    }

    /// `ln p(z | x, m_j)` (Table 6.4 in log space).
    pub fn log_likelihood(
        &self,
        pose: &Se2,
        landmark: &Vector2<f64>,
        z: &RangeBearing,
    ) -> Result<f64, ModelError> {
        if !(z.range.is_finite() && z.bearing.is_finite()) {
            return Err(ModelError::InvalidData("measurement must be finite"));
        }
        let e = Self::residual(z, &self.predict(pose, landmark)?);
        let (sr, sb) = (self.sigma_range, self.sigma_bearing);
        Ok(-0.5 * ((e.x / sr).powi(2) + (e.y / sb).powi(2)) - (sr * sb).ln() - (TAU).ln())
    }

    /// `p(z | x, m_j)`.
    pub fn likelihood(
        &self,
        pose: &Se2,
        landmark: &Vector2<f64>,
        z: &RangeBearing,
    ) -> Result<f64, ModelError> {
        Ok(self.log_likelihood(pose, landmark, z)?.exp())
    }

    /// `Σ ln p(f_i | c_i, x, m)` over observations with known correspondence.
    pub fn observations_log_likelihood(
        &self,
        pose: &Se2,
        map: &LandmarkMap,
        observations: &[LandmarkObservation],
    ) -> Result<f64, ModelError> {
        observations.iter().try_fold(0.0, |acc, o| {
            let id = o.id.ok_or(ModelError::InvalidData(
                "observation without correspondence",
            ))?;
            let l = map.get(id).ok_or(ModelError::InvalidData(
                "observation of a landmark not in the map",
            ))?;
            Ok(acc + self.log_likelihood(pose, &l.position, &o.z)?)
        })
    }

    /// Draw a noisy measurement of `landmark` from `pose` (simulation).
    pub fn sample<R: Rng + ?Sized>(
        &self,
        pose: &Se2,
        landmark: &Vector2<f64>,
        rng: &mut R,
    ) -> Result<RangeBearing, ModelError> {
        let z = self.predict(pose, landmark)?;
        let nr =
            Normal::new(0.0, self.sigma_range).map_err(|_| ModelError::InvalidData("sigma"))?;
        let nb =
            Normal::new(0.0, self.sigma_bearing).map_err(|_| ModelError::InvalidData("sigma"))?;
        Ok(RangeBearing {
            range: (z.range + nr.sample(rng)).max(0.0),
            bearing: normalize_angle(z.bearing + nb.sample(rng)),
        })
    }

    /// Table 6.5: draw a robot pose from `p(x | z, m_j)` under a uniform pose prior. One
    /// observation constrains two of three degrees of freedom; the angle around the
    /// landmark is drawn uniformly. The result is a *proposal* to be reweighted.
    pub fn sample_pose<R: Rng + ?Sized>(
        &self,
        z: &RangeBearing,
        landmark: &Vector2<f64>,
        rng: &mut R,
    ) -> Result<Se2, ModelError> {
        if !(z.range.is_finite() && z.bearing.is_finite() && z.range >= 0.0) {
            return Err(ModelError::InvalidData("measurement must be finite"));
        }
        let nr =
            Normal::new(0.0, self.sigma_range).map_err(|_| ModelError::InvalidData("sigma"))?;
        let nb =
            Normal::new(0.0, self.sigma_bearing).map_err(|_| ModelError::InvalidData("sigma"))?;
        let gamma = TAU * rng.random::<f64>();
        let r = (z.range + nr.sample(rng)).max(0.0);
        let phi = z.bearing + nb.sample(rng);
        // Sensor at angle γ around the landmark, looking so that the landmark bears φ.
        let sensor = Se2::new(
            landmark.x + r * gamma.cos(),
            landmark.y + r * gamma.sin(),
            gamma + PI - phi,
        );
        Ok(sensor.compose(&self.sensor.inverse()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use robotics_core::{seeded_rng, Manifold};
    use std::f64::consts::FRAC_PI_2;

    fn model() -> LandmarkModel {
        LandmarkModel::new(0.1, 0.05).unwrap()
    }

    #[test]
    fn known_geometry() {
        let m = model();
        let x = Se2::new(1.0, 1.0, FRAC_PI_2);
        let ahead = m.predict(&x, &Vector2::new(1.0, 3.0)).unwrap();
        assert_relative_eq!(ahead.range, 2.0, epsilon = 1e-12);
        assert_relative_eq!(ahead.bearing, 0.0, epsilon = 1e-12);
        // Facing +y, a landmark at −x is on the robot's left.
        let left = m.predict(&x, &Vector2::new(0.0, 1.0)).unwrap();
        assert_relative_eq!(left.range, 1.0, epsilon = 1e-12);
        assert_relative_eq!(left.bearing, FRAC_PI_2, epsilon = 1e-12);
        // A sensor mounted 0.1 m ahead sees the landmark 0.1 m closer.
        let fwd = model().with_sensor_pose(Se2::new(0.1, 0.0, 0.0)).unwrap();
        let z = fwd.predict(&x, &Vector2::new(1.0, 3.0)).unwrap();
        assert_relative_eq!(z.range, 1.9, epsilon = 1e-12);
    }

    #[test]
    fn bearing_residual_wraps_across_pi() {
        let m = model();
        let x = Se2::IDENTITY;
        let behind = Vector2::new(-2.0, -1e-6); // predicted bearing ≈ −π
        let pred = m.predict(&x, &behind).unwrap();
        assert!(pred.bearing < -PI + 0.01);
        let z = RangeBearing {
            range: 2.0,
            bearing: PI - 0.01,
        };
        let e = LandmarkModel::residual(&z, &pred);
        assert!(e.y.abs() < 0.011, "wrapped residual {}", e.y);
        // The wrapped reading is almost as likely as a perfect one.
        let best = m.log_likelihood(&x, &behind, &pred).unwrap();
        let wrapped = m.log_likelihood(&x, &behind, &z).unwrap();
        assert!(best - wrapped < 0.03, "{best} vs {wrapped}");
    }

    #[test]
    fn zero_range_is_degenerate() {
        let m = model();
        let x = Se2::new(2.0, -1.0, 0.3);
        assert!(matches!(
            m.predict(&x, &Vector2::new(2.0, -1.0)),
            Err(ModelError::DegenerateGeometry(_))
        ));
        assert!(m.jacobian(&x, &Vector2::new(2.0, -1.0)).is_err());
        // Just above the threshold the model is still defined.
        assert!(m.predict(&x, &Vector2::new(2.0 + 1e-6, -1.0)).is_ok());
    }

    #[test]
    fn likelihood_integrates_to_one() {
        let m = model();
        let x = Se2::new(0.5, 0.2, 0.7);
        let l = Vector2::new(3.0, 2.0);
        let zhat = m.predict(&x, &l).unwrap();
        let (hr, hb) = (0.004, 0.002);
        let mut mass = 0.0;
        let mut r = zhat.range - 0.8;
        while r < zhat.range + 0.8 {
            let mut b = -PI + hb / 2.0;
            while b < PI {
                mass += m
                    .likelihood(
                        &x,
                        &l,
                        &RangeBearing {
                            range: r,
                            bearing: b,
                        },
                    )
                    .unwrap()
                    * hr
                    * hb;
                b += hb;
            }
            r += hr;
        }
        assert!((mass - 1.0).abs() < 2e-3, "mass {mass}");
    }

    #[test]
    fn jacobian_matches_finite_differences() {
        let m = model().with_sensor_pose(Se2::new(0.1, -0.05, 0.3)).unwrap();
        let x = Se2::new(0.4, -0.3, 2.9);
        let l = Vector2::new(-2.0, 1.5);
        let h = m.jacobian(&x, &l).unwrap();
        let z0 = m.predict(&x, &l).unwrap();
        let eps = 1e-6;
        for k in 0..3 {
            let mut d = nalgebra::Vector3::zeros();
            d[k] = eps;
            let z1 = m.predict(&x.boxplus(&d), &l).unwrap();
            let num = LandmarkModel::residual(&z1, &z0) / eps;
            assert!(
                (num - h.column(k)).norm() < 1e-5,
                "column {k}: {num} vs {}",
                h.column(k)
            );
        }
    }

    #[test]
    fn noise_and_data_validation() {
        for (sr, sb) in [
            (0.0, 0.1),
            (0.1, 0.0),
            (-1.0, 0.1),
            (f64::NAN, 0.1),
            (0.1, 4.0),
        ] {
            assert!(LandmarkModel::new(sr, sb).is_err(), "{sr} {sb}");
        }
        let m = model();
        assert!(m
            .log_likelihood(
                &Se2::IDENTITY,
                &Vector2::new(1.0, 0.0),
                &RangeBearing {
                    range: f64::NAN,
                    bearing: 0.0
                }
            )
            .is_err());
        assert!(LandmarkMap::new(vec![
            Landmark {
                id: 1,
                position: Vector2::new(0.0, 0.0)
            },
            Landmark {
                id: 1,
                position: Vector2::new(1.0, 0.0)
            },
        ])
        .is_err());
        let map = LandmarkMap::new(vec![Landmark {
            id: 7,
            position: Vector2::new(2.0, 0.0),
        }])
        .unwrap();
        let obs = |id| LandmarkObservation {
            id,
            z: RangeBearing {
                range: 2.0,
                bearing: 0.0,
            },
        };
        assert!(m
            .observations_log_likelihood(&Se2::IDENTITY, &map, &[obs(Some(7))])
            .unwrap()
            .is_finite());
        assert!(m
            .observations_log_likelihood(&Se2::IDENTITY, &map, &[obs(None)])
            .is_err());
        assert!(m
            .observations_log_likelihood(&Se2::IDENTITY, &map, &[obs(Some(8))])
            .is_err());
    }

    #[test]
    fn sampled_poses_explain_the_observation() {
        let m = LandmarkModel::new(1e-9, 1e-9)
            .unwrap()
            .with_sensor_pose(Se2::new(0.1, 0.0, 0.0))
            .unwrap();
        let l = Vector2::new(2.0, 1.0);
        let z = RangeBearing {
            range: 1.5,
            bearing: -0.4,
        };
        let mut rng = seeded_rng(3);
        for _ in 0..50 {
            let x = m.sample_pose(&z, &l, &mut rng).unwrap();
            let back = m.predict(&x, &l).unwrap();
            assert_relative_eq!(back.range, z.range, epsilon = 1e-6);
            assert!(angle_diff(back.bearing, z.bearing).abs() < 1e-6);
        }
        // Simulated measurements are centred on the prediction.
        let mut s = 0.0;
        let noisy = model();
        for _ in 0..4000 {
            s += noisy
                .sample(&Se2::IDENTITY, &Vector2::new(3.0, 0.0), &mut rng)
                .unwrap()
                .range;
        }
        assert!((s / 4000.0 - 3.0).abs() < 0.01);
    }
}
