//! Probabilistic motion models `p(x_t | u_t, x_{t−1})`.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 9 — Probabilistic Motion Models
//! (Thrun, Burgard & Fox, Chapter 5: Tables 5.1, 5.3, 5.5, 5.6).
//!
//! Both models expose a **sampler** (for particle filters) and a **closed-form density**
//! (for grid localization and diagnostics) through the [`MotionModel`] trait.

mod odometry;
mod velocity;

pub use odometry::{OdomAlphas, OdomDelta, OdometryMotionModel};
pub use velocity::{VelocityAlphas, VelocityCmd, VelocityMotionModel};

use rand::Rng;
use rand_distr::{Distribution, Normal};
use robotics_core::Se2;
use std::f64::consts::TAU;

/// A motion model with a sampler and a (possibly unnormalized) density.
pub trait MotionModel {
    /// The control input type.
    type Control;

    /// Draw a successor pose `x_t ~ p(· | u, x_{t−1})`.
    fn sample<R: Rng + ?Sized>(&self, x_prev: &Se2, u: &Self::Control, rng: &mut R) -> Se2;

    /// Evaluate `p(x_next | u, x_prev)` (Thrun's closed forms; see each model for caveats).
    fn prob(&self, x_next: &Se2, u: &Self::Control, x_prev: &Se2) -> f64;

    /// `ln p(x_next | u, x_prev)`; `−∞` where `prob` is zero.
    fn log_prob(&self, x_next: &Se2, u: &Self::Control, x_prev: &Se2) -> f64 {
        self.prob(x_next, u, x_prev).ln()
    }
}

/// Zero-mean normal density `N(a; 0, var)` (Thrun's `prob(a, b²)`), with a variance
/// floor so a zero-variance component degenerates gracefully to a sharp spike.
#[inline]
pub(crate) fn prob_normal(a: f64, var: f64) -> f64 {
    let var = var.max(1e-12);
    (-0.5 * a * a / var).exp() / (TAU * var).sqrt()
}

/// Draw `N(0, var)`; a non-positive variance yields exactly `0`.
#[inline]
pub(crate) fn sample_normal<R: Rng + ?Sized>(var: f64, rng: &mut R) -> f64 {
    if var > 0.0 {
        Normal::new(0.0, var.sqrt())
            .map(|n| n.sample(rng))
            .unwrap_or(0.0)
    } else {
        0.0
    }
}
