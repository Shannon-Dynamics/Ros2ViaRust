//! Odometry motion model (Thrun Tables 5.5 & 5.6).
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 9 (`OdomDelta`, `OdometryModel`).
//!
//! The control is the pair of consecutive *odometry* poses `(x̄_{t−1}, x̄_t)`, decomposed
//! into `(δ_rot1, δ_trans, δ_rot2)`. Noise parameters `α₁..α₄` are indexed `alphas[0..4]`;
//! variances use the squared form of the 2005 edition.

use super::{prob_normal, sample_normal, MotionModel};
use crate::ModelError;
use rand::Rng;
use robotics_core::angle::normalize_angle;
use robotics_core::Se2;

/// The four odometry-model noise parameters `α₁..α₄`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdomAlphas(pub [f64; 4]);

impl OdomAlphas {
    /// The book's default `[0.01, 0.01, 0.02, 0.01]`.
    pub const DEFAULT: OdomAlphas = OdomAlphas([0.01, 0.01, 0.02, 0.01]);

    /// Validate (all non-negative and finite).
    pub fn validated(self) -> Result<Self, ModelError> {
        if let Some(&v) = self.0.iter().find(|v| !(**v >= 0.0 && v.is_finite())) {
            return Err(ModelError::Negative {
                what: "alpha",
                value: v,
            });
        }
        Ok(self)
    }
}

impl Default for OdomAlphas {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// A relative motion decomposed as rotate–translate–rotate.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct OdomDelta {
    /// Initial rotation (rad).
    pub rot1: f64,
    /// Translation (m).
    pub trans: f64,
    /// Final rotation (rad).
    pub rot2: f64,
}

impl OdomDelta {
    /// Decompose the motion from `prev` to `cur`. A pure spin (`trans < 1e-9`) is
    /// reported entirely in `rot2` so the decomposition stays well defined.
    pub fn from_poses(prev: &Se2, cur: &Se2) -> Self {
        let dx = cur.x() - prev.x();
        let dy = cur.y() - prev.y();
        let trans = (dx * dx + dy * dy).sqrt();
        if trans < 1e-9 {
            return Self {
                rot1: 0.0,
                trans: 0.0,
                rot2: normalize_angle(cur.theta() - prev.theta()),
            };
        }
        let rot1 = normalize_angle(dy.atan2(dx) - prev.theta());
        let rot2 = normalize_angle(cur.theta() - prev.theta() - rot1);
        Self { rot1, trans, rot2 }
    }

    /// Apply the decomposition to a pose.
    pub fn apply(&self, x: &Se2) -> Se2 {
        let heading = x.theta() + self.rot1;
        Se2::new(
            x.x() + self.trans * heading.cos(),
            x.y() + self.trans * heading.sin(),
            heading + self.rot2,
        )
    }
}

/// Thrun's odometry motion model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryMotionModel {
    alphas: OdomAlphas,
}

impl OdometryMotionModel {
    /// Create the model with validated noise parameters.
    pub fn new(alphas: OdomAlphas) -> Result<Self, ModelError> {
        Ok(Self {
            alphas: alphas.validated()?,
        })
    }

    /// The noise parameters.
    #[inline]
    pub fn alphas(&self) -> &OdomAlphas {
        &self.alphas
    }

    /// Sample a successor given an already-decomposed delta (Table 5.6, lines 5–7, noise
    /// subtracted as in the book).
    pub fn sample_delta<R: Rng + ?Sized>(&self, x_prev: &Se2, d: &OdomDelta, rng: &mut R) -> Se2 {
        let a = &self.alphas.0;
        let (r1, t, r2) = (d.rot1 * d.rot1, d.trans * d.trans, d.rot2 * d.rot2);
        let rot1 = d.rot1 - sample_normal(a[0] * r1 + a[1] * t, rng);
        let trans = d.trans - sample_normal(a[2] * t + a[3] * (r1 + r2), rng);
        let rot2 = d.rot2 - sample_normal(a[0] * r2 + a[1] * t, rng);
        OdomDelta { rot1, trans, rot2 }.apply(x_prev)
    }

    /// Density of a successor given an already-decomposed commanded delta (Table 5.5).
    pub fn prob_delta(&self, x_next: &Se2, d: &OdomDelta, x_prev: &Se2) -> f64 {
        let a = &self.alphas.0;
        let h = OdomDelta::from_poses(x_prev, x_next);
        let (r1, t, r2) = (h.rot1 * h.rot1, h.trans * h.trans, h.rot2 * h.rot2);
        prob_normal(normalize_angle(d.rot1 - h.rot1), a[0] * r1 + a[1] * t)
            * prob_normal(d.trans - h.trans, a[2] * t + a[3] * (r1 + r2))
            * prob_normal(normalize_angle(d.rot2 - h.rot2), a[0] * r2 + a[1] * t)
    }
}

impl MotionModel for OdometryMotionModel {
    /// The control is `(x̄_{t−1}, x̄_t)` in the odometry frame.
    type Control = (Se2, Se2);

    fn sample<R: Rng + ?Sized>(&self, x_prev: &Se2, u: &(Se2, Se2), rng: &mut R) -> Se2 {
        self.sample_delta(x_prev, &OdomDelta::from_poses(&u.0, &u.1), rng)
    }

    fn prob(&self, x_next: &Se2, u: &(Se2, Se2), x_prev: &Se2) -> f64 {
        self.prob_delta(x_next, &OdomDelta::from_poses(&u.0, &u.1), x_prev)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use robotics_core::{seeded_rng, Manifold};

    /// Book check #15: apply(decompose(a, b)) == b, and a pure spin lands in rot2.
    #[test]
    fn decomposition_round_trip() {
        let a = Se2::new(1.0, -2.0, 0.4);
        let b = Se2::new(2.5, -1.2, -0.9);
        let d = OdomDelta::from_poses(&a, &b);
        let back = d.apply(&a);
        assert!(back.boxminus(&b).norm() < 1e-12);

        let spin = OdomDelta::from_poses(&a, &Se2::new(1.0, -2.0, 1.4));
        assert_relative_eq!(spin.trans, 0.0, epsilon = 1e-12);
        assert_relative_eq!(spin.rot1, 0.0, epsilon = 1e-12);
        assert_relative_eq!(spin.rot2, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn noiseless_sample_is_deterministic_and_density_peaks_there() {
        let model = OdometryMotionModel::new(OdomAlphas([0.0; 4])).unwrap();
        let x = Se2::new(0.0, 0.0, 0.1);
        let u = (Se2::new(5.0, 5.0, 0.0), Se2::new(5.8, 5.3, 0.4));
        let mut rng = seeded_rng(1);
        let s1 = model.sample(&x, &u, &mut rng);
        let s2 = model.sample(&x, &u, &mut rng);
        assert!(s1.boxminus(&s2).norm() < 1e-15);

        let noisy = OdometryMotionModel::new(OdomAlphas::DEFAULT).unwrap();
        let p0 = noisy.prob(&s1, &u, &x);
        let off = Se2::new(s1.x() + 0.05, s1.y(), s1.theta());
        assert!(noisy.prob(&off, &u, &x) < p0);
    }

    #[test]
    fn sample_cloud_centres_on_exact_motion() {
        let model = OdometryMotionModel::new(OdomAlphas::DEFAULT).unwrap();
        let x = Se2::new(0.0, 0.0, 0.0);
        let u = (Se2::IDENTITY, Se2::new(1.0, 0.0, 0.0));
        let mut rng = seeded_rng(42);
        let n = 5000;
        let mut sx = 0.0;
        for _ in 0..n {
            sx += model.sample(&x, &u, &mut rng).x();
        }
        assert!((sx / n as f64 - 1.0).abs() < 0.01);
    }
}
