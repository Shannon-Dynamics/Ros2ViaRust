//! Velocity motion model (Thrun Tables 5.1 & 5.3).
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 9 (`VelocityModel`,
//! `sample_motion_model_velocity`, `invert_arc`).
//!
//! Noise parameters follow the 2005 edition: variances are `α₁v² + α₂ω²` etc. (squared
//! form), indexed `alphas[0..6]` for `α₁..α₆`.

use super::{prob_normal, sample_normal, MotionModel};
use crate::ModelError;
use rand::Rng;
use robotics_core::angle::normalize_angle;
use robotics_core::{Manifold, Se2, Tangent2};

/// Threshold on `|ω|` below which the arc degenerates to a straight line.
const STRAIGHT: f64 = 1e-6;

/// The six velocity-model noise parameters `α₁..α₆`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityAlphas(pub [f64; 6]);

impl VelocityAlphas {
    /// The book's default `[0.02, 0.02, 0.02, 0.02, 0.005, 0.005]`.
    pub const DEFAULT: VelocityAlphas = VelocityAlphas([0.02, 0.02, 0.02, 0.02, 0.005, 0.005]);

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

impl Default for VelocityAlphas {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// A velocity command held for `dt` seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityCmd {
    /// Forward speed (m/s).
    pub v: f64,
    /// Yaw rate (rad/s).
    pub omega: f64,
    /// Duration (s).
    pub dt: f64,
}

impl VelocityCmd {
    /// Build a command, rejecting non-positive or non-finite durations.
    pub fn new(v: f64, omega: f64, dt: f64) -> Result<Self, ModelError> {
        if !(dt > 0.0 && dt.is_finite()) {
            return Err(ModelError::InvalidTimeStep(dt));
        }
        Ok(Self { v, omega, dt })
    }

    /// The exact (noise-free) successor pose: the constant-curvature arc, i.e.
    /// `x ⊞ (v·dt, 0, ω·dt)`.
    #[inline]
    pub fn predict_exact(&self, x: &Se2) -> Se2 {
        x.boxplus(&Tangent2::new(self.v * self.dt, 0.0, self.omega * self.dt))
    }
}

/// Thrun's velocity motion model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityMotionModel {
    alphas: VelocityAlphas,
}

impl VelocityMotionModel {
    /// Create the model with validated noise parameters.
    pub fn new(alphas: VelocityAlphas) -> Result<Self, ModelError> {
        Ok(Self {
            alphas: alphas.validated()?,
        })
    }

    /// The noise parameters.
    #[inline]
    pub fn alphas(&self) -> &VelocityAlphas {
        &self.alphas
    }

    /// Advance a pose by a (possibly perturbed) velocity `(v̂, ω̂)` over `dt` and add the
    /// final-heading perturbation `γ̂·dt` (Table 5.3, lines 6–8).
    fn integrate(x: &Se2, v: f64, w: f64, gamma: f64, dt: f64) -> Se2 {
        let th = x.theta();
        if w.abs() < STRAIGHT {
            Se2::new(
                x.x() + v * th.cos() * dt,
                x.y() + v * th.sin() * dt,
                th + w * dt + gamma * dt,
            )
        } else {
            let r = v / w;
            Se2::new(
                x.x() - r * th.sin() + r * (th + w * dt).sin(),
                x.y() + r * th.cos() - r * (th + w * dt).cos(),
                th + w * dt + gamma * dt,
            )
        }
    }
}

impl MotionModel for VelocityMotionModel {
    type Control = VelocityCmd;

    /// Table 5.3: `v̂ = v + N(0, α₁v² + α₂ω²)`, `ω̂ = ω + N(0, α₃v² + α₄ω²)`,
    /// `γ̂ = N(0, α₅v² + α₆ω²)`.
    fn sample<R: Rng + ?Sized>(&self, x_prev: &Se2, u: &VelocityCmd, rng: &mut R) -> Se2 {
        let a = &self.alphas.0;
        let (v2, w2) = (u.v * u.v, u.omega * u.omega);
        let v_hat = u.v + sample_normal(a[0] * v2 + a[1] * w2, rng);
        let w_hat = u.omega + sample_normal(a[2] * v2 + a[3] * w2, rng);
        let g_hat = sample_normal(a[4] * v2 + a[5] * w2, rng);
        Self::integrate(x_prev, v_hat, w_hat, g_hat, u.dt)
    }

    /// Table 5.1: recover the arc `(v̂, ω̂, γ̂)` that would move `x_prev` to `x_next`,
    /// then score it against the commanded `(v, ω)`.
    ///
    /// Deviation from Thrun's table, documented: Thrun sets `v̂ = Δθ·r*/Δt`, whose sign
    /// is wrong for right-hand turns. We use `v̂ = |Δθ|·r*/Δt · sign(forward progress)`,
    /// which agrees with the sampler for all turn directions. As in the book, this is
    /// Thrun's unnormalized form (no Jacobian to `(x, y, θ)` space); it is a valid
    /// relative weight for localization, not a normalized density.
    fn prob(&self, x_next: &Se2, u: &VelocityCmd, x_prev: &Se2) -> f64 {
        let a = &self.alphas.0;
        let (x, y, th) = (x_prev.x(), x_prev.y(), x_prev.theta());
        let (xn, yn, thn) = (x_next.x(), x_next.y(), x_next.theta());
        let dt = u.dt;
        let (s, c) = th.sin_cos();
        let forward = (xn - x) * c + (yn - y) * s;
        let numer = 0.5 * ((x - xn) * c + (y - yn) * s);
        let denom = (y - yn) * c - (x - xn) * s;

        let (v_hat, w_hat) = if denom.abs() < 1e-9 {
            (forward / dt, 0.0)
        } else {
            let mu = numer / denom;
            let xs = 0.5 * (x + xn) + mu * (y - yn);
            let ys = 0.5 * (y + yn) + mu * (xn - x);
            let r_star = ((x - xs).powi(2) + (y - ys).powi(2)).sqrt();
            let dth = normalize_angle((yn - ys).atan2(xn - xs) - (y - ys).atan2(x - xs));
            let w_hat = dth / dt;
            let v_hat = dth.abs() * r_star / dt * forward.signum();
            (v_hat, w_hat)
        };
        let g_hat = normalize_angle(thn - th) / dt - w_hat;

        let (v2, w2) = (u.v * u.v, u.omega * u.omega);
        prob_normal(u.v - v_hat, a[0] * v2 + a[1] * w2)
            * prob_normal(u.omega - w_hat, a[2] * v2 + a[3] * w2)
            * prob_normal(g_hat, a[4] * v2 + a[5] * w2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use robotics_core::seeded_rng;

    /// Chapter 9 worked example: from the origin, v = 1, ω = 0.5, Δt = 1 lands at
    /// (2 sin 0.5, 2(1 − cos 0.5), 0.5) = (0.958851077208, 0.244834876219, 0.5), which is
    /// exactly exp((1, 0, 0.5)).
    #[test]
    fn worked_example_exact_arc() {
        let u = VelocityCmd::new(1.0, 0.5, 1.0).unwrap();
        let x1 = u.predict_exact(&Se2::IDENTITY);
        assert_relative_eq!(x1.x(), 0.958851077208, epsilon = 1e-12);
        assert_relative_eq!(x1.y(), 0.244834876219, epsilon = 1e-12);
        assert_relative_eq!(x1.theta(), 0.5, epsilon = 1e-12);
        let via_integrate = VelocityMotionModel::integrate(&Se2::IDENTITY, 1.0, 0.5, 0.0, 1.0);
        assert!(via_integrate.boxminus(&x1).norm() < 1e-12);
    }

    /// Book check #13: with tiny α the sample mean sits on the exact arc.
    #[test]
    fn sampler_mean_matches_exact_arc() {
        let model = VelocityMotionModel::new(VelocityAlphas([0.001; 6])).unwrap();
        let x0 = Se2::new(1.0, 2.0, 0.3);
        let u = VelocityCmd::new(1.0, 0.5, 1.0).unwrap();
        let exact = u.predict_exact(&x0);
        let mut rng = seeded_rng(7);
        let n = 4000;
        let (mut sx, mut sy) = (0.0, 0.0);
        for _ in 0..n {
            let s = model.sample(&x0, &u, &mut rng);
            sx += s.x();
            sy += s.y();
        }
        assert!((sx / n as f64 - exact.x()).abs() < 0.02);
        assert!((sy / n as f64 - exact.y()).abs() < 0.02);
    }

    /// Book check #14: the density peaks at the exact endpoint, for both turn directions.
    #[test]
    fn density_peaks_at_exact_endpoint() {
        let model =
            VelocityMotionModel::new(VelocityAlphas([0.02, 0.02, 0.02, 0.02, 0.01, 0.01])).unwrap();
        let x0 = Se2::new(0.0, 0.0, 0.2);
        for omega in [0.6, -0.6, 0.0] {
            let u = VelocityCmd::new(1.0, omega, 1.0).unwrap();
            let exact = u.predict_exact(&x0);
            let p0 = model.prob(&exact, &u, &x0);
            assert!(p0.is_finite() && p0 > 0.0);
            let shifted = [
                Se2::new(exact.x() + 0.1, exact.y(), exact.theta()),
                Se2::new(exact.x(), exact.y() - 0.1, exact.theta()),
                Se2::new(exact.x(), exact.y(), exact.theta() + 0.2),
            ];
            for s in shifted {
                assert!(
                    model.prob(&s, &u, &x0) < p0,
                    "omega={omega}: shifted pose scored higher"
                );
            }
        }
    }

    #[test]
    fn rejects_invalid_inputs() {
        assert!(VelocityCmd::new(1.0, 0.0, 0.0).is_err());
        assert!(VelocityMotionModel::new(VelocityAlphas([-0.1, 0.0, 0.0, 0.0, 0.0, 0.0])).is_err());
    }
}
