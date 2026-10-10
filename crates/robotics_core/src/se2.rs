//! SE(2): planar rigid-body poses as Lie-group elements, with the `⊞`/`⊟` retractions
//! that every on-manifold estimator in the framework builds on.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 3 — The Geometry of Motion
//! (hand-rolled `SE2` with `exp`/`log`/`adjoint`, the `Manifold` trait), following the
//! micro-Lie conventions of Solà, Deray & Atchuthan (2018).
//!
//! # Conventions
//! * Tangent order is **translation-first**: `τ = (v_x, v_y, ω)`.
//! * Right (body-frame) perturbation: `x ⊞ δ = x · exp(δ)` and `y ⊟ x = log(x⁻¹ · y)`.
//! * `exp` is the exact constant-curvature arc, so integrating a twist for one step
//!   with `exp((v·Δt, 0, ω·Δt))` is Thrun's velocity motion model without approximation.
//! * `log` is valid for `|θ| < π`; at exactly `θ = π` the map is still finite but the
//!   inverse is not unique.

use crate::angle::normalize_angle;
use nalgebra::{Matrix3, Rotation2, SVector, Vector2, Vector3};
use std::ops::Mul;

/// Tangent vector of SE(2): `(v_x, v_y, ω)`.
pub type Tangent2 = Vector3<f64>;

/// Threshold below which `ω` is treated with Taylor expansions to avoid `sin ω / ω`
/// cancellation.
const SMALL_ANGLE: f64 = 1e-8;

/// A planar pose: translation `(x, y)` in metres and heading `θ` in radians, `θ ∈ (−π, π]`.
///
/// The struct is `Copy`; composing poses does not allocate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Se2 {
    x: f64,
    y: f64,
    theta: f64,
}

impl Default for Se2 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Se2 {
    /// The identity pose (origin, zero heading).
    pub const IDENTITY: Se2 = Se2 {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
    };

    /// Build a pose from `x`, `y` and a heading; the heading is normalized to `(−π, π]`.
    #[inline]
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self {
            x,
            y,
            theta: normalize_angle(theta),
        }
    }

    /// Build a pose from a translation vector and a heading.
    #[inline]
    pub fn from_parts(t: Vector2<f64>, theta: f64) -> Self {
        Self::new(t.x, t.y, theta)
    }

    /// `x` coordinate (m).
    #[inline]
    pub fn x(&self) -> f64 {
        self.x
    }
    /// `y` coordinate (m).
    #[inline]
    pub fn y(&self) -> f64 {
        self.y
    }
    /// Heading (rad), always in `(−π, π]`.
    #[inline]
    pub fn theta(&self) -> f64 {
        self.theta
    }
    /// Translation part.
    #[inline]
    pub fn translation(&self) -> Vector2<f64> {
        Vector2::new(self.x, self.y)
    }
    /// Rotation part as a 2×2 rotation.
    #[inline]
    pub fn rotation(&self) -> Rotation2<f64> {
        Rotation2::new(self.theta)
    }
    /// The pose as the vector `(x, y, θ)`.
    ///
    /// This is *not* a tangent vector; it is the chart most ROS messages use.
    #[inline]
    pub fn xytheta(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.theta)
    }

    /// Exponential map `se(2) → SE(2)`.
    ///
    /// For `τ = (v_x, v_y, ω)` the translation is
    /// `t = (a·v_x − b·v_y, b·v_x + a·v_y)` with `a = sin ω / ω` and `b = (1 − cos ω) / ω`
    /// (computed as `2 sin²(ω/2) / ω`), and the rotation is `Rot(ω)`. Near `ω = 0` the
    /// Taylor forms `a ≈ 1 − ω²/6`, `b ≈ ω/2 − ω³/24` are used.
    pub fn exp(tau: &Tangent2) -> Self {
        let (vx, vy, w) = (tau[0], tau[1], tau[2]);
        let (a, b) = if w.abs() < SMALL_ANGLE {
            (1.0 - w * w / 6.0, w / 2.0 - w * w * w / 24.0)
        } else {
            let s = (0.5 * w).sin();
            (w.sin() / w, 2.0 * s * s / w)
        };
        Self::new(a * vx - b * vy, b * vx + a * vy, w)
    }

    /// Logarithmic map `SE(2) → se(2)`, the inverse of [`Se2::exp`] for `|θ| < π`.
    ///
    /// With `h = θ/2` and `c = h·cot h` (Taylor `1 − θ²/12` near zero):
    /// `v = (c·x + h·y, −h·x + c·y)`.
    pub fn log(&self) -> Tangent2 {
        let th = self.theta;
        let h = 0.5 * th;
        let c = if th.abs() < SMALL_ANGLE {
            1.0 - th * th / 12.0
        } else {
            h * h.cos() / h.sin()
        };
        Tangent2::new(c * self.x + h * self.y, -h * self.x + c * self.y, th)
    }

    /// Group inverse: `(Rᵀ, −Rᵀ t)`.
    #[inline]
    pub fn inverse(&self) -> Self {
        let (s, c) = self.theta.sin_cos();
        Self::new(
            -(c * self.x + s * self.y),
            -(-s * self.x + c * self.y),
            -self.theta,
        )
    }

    /// Group composition `self · other` (apply `other` in the body frame of `self`).
    #[inline]
    pub fn compose(&self, other: &Se2) -> Se2 {
        let (s, c) = self.theta.sin_cos();
        Se2::new(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            self.theta + other.theta,
        )
    }

    /// Relative pose `self⁻¹ · other`: where `other` sits as seen from `self`.
    #[inline]
    pub fn between(&self, other: &Se2) -> Se2 {
        self.inverse().compose(other)
    }

    /// Adjoint matrix `Ad_T = [[R, (t_y, −t_x)ᵀ], [0, 1]]`, satisfying
    /// `T · exp(τ) · T⁻¹ = exp(Ad_T τ)`.
    pub fn adjoint(&self) -> Matrix3<f64> {
        let (s, c) = self.theta.sin_cos();
        Matrix3::new(c, -s, self.y, s, c, -self.x, 0.0, 0.0, 1.0)
    }

    /// Transform a point expressed in this pose's body frame into the parent frame.
    #[inline]
    pub fn act(&self, p: &Vector2<f64>) -> Vector2<f64> {
        let (s, c) = self.theta.sin_cos();
        Vector2::new(self.x + c * p.x - s * p.y, self.y + s * p.x + c * p.y)
    }

    /// Transform a point expressed in the parent frame into this pose's body frame.
    #[inline]
    pub fn act_inverse(&self, p: &Vector2<f64>) -> Vector2<f64> {
        let (s, c) = self.theta.sin_cos();
        let dx = p.x - self.x;
        let dy = p.y - self.y;
        Vector2::new(c * dx + s * dy, -s * dx + c * dy)
    }

    /// Euclidean distance between the two translations (m).
    #[inline]
    pub fn distance_to(&self, other: &Se2) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    /// `true` when all three components are finite.
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.theta.is_finite()
    }
}

impl Mul for Se2 {
    type Output = Se2;
    #[inline]
    fn mul(self, rhs: Se2) -> Se2 {
        self.compose(&rhs)
    }
}

impl Mul<&Se2> for &Se2 {
    type Output = Se2;
    #[inline]
    fn mul(self, rhs: &Se2) -> Se2 {
        self.compose(rhs)
    }
}

/// A smooth manifold with a local retraction, the abstraction every on-manifold filter
/// (EKF/UKF/ESKF/particle filter) is generic over.
///
/// Reference: *Probabilistic Robotics via Rust*, Chapter 7 (`Manifold` trait,
/// `boxplus`/`boxminus`).
pub trait Manifold<const D: usize>: Clone {
    /// Retract a tangent increment onto the manifold: `self ⊞ δ`.
    fn boxplus(&self, delta: &SVector<f64, D>) -> Self;
    /// Local difference `self ⊟ base`, i.e. the tangent vector at `base` that reaches
    /// `self`: `log(base⁻¹ · self)` for a Lie group, plain subtraction for a vector space.
    fn boxminus(&self, base: &Self) -> SVector<f64, D>;
}

impl Manifold<3> for Se2 {
    #[inline]
    fn boxplus(&self, delta: &SVector<f64, 3>) -> Self {
        self.compose(&Se2::exp(delta))
    }
    #[inline]
    fn boxminus(&self, base: &Self) -> SVector<f64, 3> {
        base.between(self).log()
    }
}

impl<const D: usize> Manifold<D> for SVector<f64, D> {
    #[inline]
    fn boxplus(&self, delta: &SVector<f64, D>) -> Self {
        self + delta
    }
    #[inline]
    fn boxminus(&self, base: &Self) -> SVector<f64, D> {
        self - base
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use proptest::prelude::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn assert_pose_eq(a: &Se2, b: &Se2, eps: f64) {
        assert_relative_eq!(a.x(), b.x(), epsilon = eps);
        assert_relative_eq!(a.y(), b.y(), epsilon = eps);
        assert_relative_eq!(normalize_angle(a.theta() - b.theta()), 0.0, epsilon = eps);
    }

    /// Chapter 3 worked example: a quarter-circle arc.
    #[test]
    fn worked_example_quarter_arc() {
        let t = Se2::exp(&Tangent2::new(1.0, 0.0, FRAC_PI_2));
        assert_relative_eq!(t.x(), 2.0 / PI, epsilon = 1e-12);
        assert_relative_eq!(t.y(), 2.0 / PI, epsilon = 1e-12);
        assert_relative_eq!(t.theta(), FRAC_PI_2, epsilon = 1e-12);
        let back = t.log();
        assert_relative_eq!(back[0], 1.0, epsilon = 1e-12);
        assert_relative_eq!(back[1], 0.0, epsilon = 1e-12);
        assert_relative_eq!(back[2], FRAC_PI_2, epsilon = 1e-12);
    }

    /// The book's pinned round-trip set (web/lib/__checks__.ts #1).
    #[test]
    fn log_exp_round_trip_pinned_set() {
        let taus = [
            [1.0, 0.0, 0.0],
            [0.4, -0.9, 1.7],
            [2.0, 3.0, -3.0],
            [1.0, 1.0, 1e-9],
            [-0.3, 0.2, 1e-4],
        ];
        for t in taus {
            let tau = Tangent2::new(t[0], t[1], t[2]);
            let rt = Se2::exp(&tau).log();
            assert!(
                (rt - tau).norm() < 1e-9,
                "round trip failed for {tau:?}: {rt:?}"
            );
        }
    }

    #[test]
    fn inverse_and_boxplus_boxminus() {
        let a = Se2::new(1.5, -0.4, 2.1);
        let b = Se2::new(-2.2, 3.3, -1.4);
        assert_pose_eq(&(a * a.inverse()), &Se2::IDENTITY, 1e-12);
        assert_pose_eq(&b.boxplus(&a.boxminus(&b)), &a, 1e-12);
        assert_relative_eq!(normalize_angle(3.0 * PI), PI, epsilon = 1e-12);
    }

    #[test]
    fn adjoint_identity() {
        let t = Se2::new(0.7, -1.3, 0.9);
        let tau = Tangent2::new(0.2, -0.15, 0.35);
        let lhs = t * Se2::exp(&tau) * t.inverse();
        let rhs = Se2::exp(&(t.adjoint() * tau));
        assert_pose_eq(&lhs, &rhs, 1e-12);
    }

    #[test]
    fn square_dance_closes() {
        let step =
            Se2::exp(&Tangent2::new(1.0, 0.0, 0.0)) * Se2::exp(&Tangent2::new(0.0, 0.0, FRAC_PI_2));
        let mut p = Se2::IDENTITY;
        for _ in 0..4 {
            p = p * step;
        }
        assert!(p.boxminus(&Se2::IDENTITY).norm() < 1e-14);
    }

    #[test]
    fn act_and_act_inverse_are_inverse() {
        let t = Se2::new(2.0, -1.0, 0.7);
        let p = Vector2::new(0.3, 0.8);
        let w = t.act(&p);
        let back = t.act_inverse(&w);
        assert!((back - p).norm() < 1e-12);
        // Sanity: the origin of the body frame maps to the translation.
        assert!((t.act(&Vector2::zeros()) - t.translation()).norm() < 1e-15);
    }

    #[test]
    fn log_is_stable_near_pi() {
        for k in 1..=6 {
            let th = PI - 10f64.powi(-k);
            let t = Se2::new(0.3, -0.2, th);
            let back = Se2::exp(&t.log());
            assert_pose_eq(&back, &t, 1e-9);
        }
    }

    proptest! {
        #[test]
        fn exp_log_round_trip(vx in -5.0f64..5.0, vy in -5.0f64..5.0, w in -3.1f64..3.1) {
            let tau = Tangent2::new(vx, vy, w);
            let rt = Se2::exp(&tau).log();
            prop_assert!((rt - tau).norm() < 1e-8, "{tau:?} -> {rt:?}");
        }

        #[test]
        fn compose_is_associative(
            a in (-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0),
            b in (-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0),
            c in (-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0),
        ) {
            let (a, b, c) = (Se2::new(a.0, a.1, a.2), Se2::new(b.0, b.1, b.2), Se2::new(c.0, c.1, c.2));
            let lhs = (a * b) * c;
            let rhs = a * (b * c);
            prop_assert!(lhs.boxminus(&rhs).norm() < 1e-10);
        }
    }
}
