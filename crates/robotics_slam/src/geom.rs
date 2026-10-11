//! SE(2) calculus the optimizer and the scan matchers share: the left and right Jacobians of
//! the exponential map and the residual of a relative-pose measurement with its Jacobians.
//!
//! Conventions are those of `robotics_core::se2`: tangent order `(v_x, v_y, ω)`, right
//! (body-frame) perturbations `X ⊞ δ = X · exp(δ)`. A relative-pose measurement `Z` of the
//! pose `X_j` seen from `X_i` has the residual `e = log(Z⁻¹ · X_i⁻¹ · X_j)`, the tangent
//! vector at `Z` that reaches the predicted relative pose. It is zero when the graph agrees
//! with the measurement.
//!
//! The Jacobians are exact, not the small-residual `J_r⁻¹ ≈ I` shortcut: a loop closure
//! enters an optimization with a residual of metres, where that shortcut is wrong by a
//! factor, and the unit tests compare every Jacobian here with central differences.

use nalgebra::{Matrix3, Vector3};
use robotics_core::{Se2, Tangent2};

/// Below this `|ω|` the closed forms use Taylor expansions.
const SMALL_ANGLE: f64 = 1e-6;

/// Left Jacobian of SE(2), `exp(τ + δ) ≈ exp(J_l(τ) δ) · exp(τ)`, equal to
/// `Σ ad_τⁿ / (n+1)!` with `ad_τ = [[0, -ω, v_y], [ω, 0, -v_x], [0, 0, 0]]`.
pub fn left_jacobian(tau: &Tangent2) -> Matrix3<f64> {
    let (vx, vy, w) = (tau[0], tau[1], tau[2]);
    // a = sin ω / ω, b = (1 - cos ω) / ω, c = (1 - cos ω) / ω², d = (ω - sin ω) / ω².
    let (a, b, c, d) = if w.abs() < SMALL_ANGLE {
        let w2 = w * w;
        (
            1.0 - w2 / 6.0,
            w / 2.0 - w * w2 / 24.0,
            0.5 - w2 / 24.0,
            w / 6.0 - w * w2 / 120.0,
        )
    } else {
        let (s, co) = w.sin_cos();
        (
            s / w,
            (1.0 - co) / w,
            (1.0 - co) / (w * w),
            (w - s) / (w * w),
        )
    };
    Matrix3::new(
        a,
        -b,
        c * vy + d * vx,
        b,
        a,
        -c * vx + d * vy,
        0.0,
        0.0,
        1.0,
    )
}

/// Right Jacobian of SE(2), `exp(τ + δ) ≈ exp(τ) · exp(J_r(τ) δ)`; `J_r(τ) = J_l(−τ)`.
pub fn right_jacobian(tau: &Tangent2) -> Matrix3<f64> {
    left_jacobian(&(-tau))
}

/// Inverse of the right Jacobian. Singular only at `|ω| = 2π`, outside the range of `log`.
pub fn right_jacobian_inverse(tau: &Tangent2) -> Matrix3<f64> {
    right_jacobian(tau)
        .try_inverse()
        .unwrap_or_else(Matrix3::identity)
}

/// Residual and Jacobians of one relative-pose measurement.
#[derive(Debug, Clone, Copy)]
pub struct RelativePoseResidual {
    /// `e = log(Z⁻¹ · X_i⁻¹ · X_j)`.
    pub error: Vector3<f64>,
    /// `∂e/∂δ_i` for the right perturbation `X_i ⊞ δ_i`.
    pub jac_i: Matrix3<f64>,
    /// `∂e/∂δ_j` for the right perturbation `X_j ⊞ δ_j`.
    pub jac_j: Matrix3<f64>,
}

/// Evaluate the relative-pose residual of `z` between `xi` and `xj` with exact Jacobians.
///
/// With `T = X_i⁻¹ X_j` and `E = Z⁻¹ T = exp(e)`: `∂e/∂δ_j = J_r⁻¹(e)` and
/// `∂e/∂δ_i = −J_r⁻¹(e) · Ad_{T⁻¹}`.
pub fn relative_pose_residual(xi: &Se2, xj: &Se2, z: &Se2) -> RelativePoseResidual {
    let t = xi.between(xj);
    let e = z.between(&t).log();
    let jr_inv = right_jacobian_inverse(&e);
    RelativePoseResidual {
        error: e,
        jac_i: -jr_inv * t.inverse().adjoint(),
        jac_j: jr_inv,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use robotics_core::Manifold;

    fn series_left_jacobian(tau: &Tangent2) -> Matrix3<f64> {
        let ad = Matrix3::new(0.0, -tau[2], tau[1], tau[2], 0.0, -tau[0], 0.0, 0.0, 0.0);
        let mut term = Matrix3::identity();
        let mut sum = Matrix3::identity();
        for n in 1..40 {
            term = term * ad / (n as f64 + 1.0);
            sum += term;
        }
        sum
    }

    #[test]
    fn left_jacobian_matches_its_series() {
        for tau in [
            Tangent2::new(0.3, -0.2, 0.5),
            Tangent2::new(1.0, 2.0, -1.2),
            Tangent2::new(-0.4, 0.1, 2.9),
            Tangent2::new(0.7, 0.2, 1e-9),
        ] {
            let a = left_jacobian(&tau);
            let b = series_left_jacobian(&tau);
            assert!((a - b).abs().max() < 1e-9, "{tau:?}\n{a}\n{b}");
        }
    }

    #[test]
    fn right_jacobian_is_the_first_order_effect_of_a_tangent_change() {
        let tau = Tangent2::new(0.8, -0.5, 0.9);
        let d = Tangent2::new(1e-6, -2e-6, 1.5e-6);
        let lhs = Se2::exp(&(tau + d));
        let rhs = Se2::exp(&tau).boxplus(&(right_jacobian(&tau) * d));
        assert!(lhs.between(&rhs).log().norm() < 1e-11);
    }

    #[test]
    fn jacobian_inverse_inverts() {
        let tau = Tangent2::new(2.0, -3.0, 2.2);
        let p = right_jacobian(&tau) * right_jacobian_inverse(&tau);
        assert_relative_eq!(p, Matrix3::identity(), epsilon = 1e-12);
    }

    fn numeric_jacobians(xi: &Se2, xj: &Se2, z: &Se2) -> (Matrix3<f64>, Matrix3<f64>) {
        let h = 1e-6;
        let f = |a: &Se2, b: &Se2| z.between(&a.between(b)).log();
        let mut ji = Matrix3::zeros();
        let mut jj = Matrix3::zeros();
        for k in 0..3 {
            let mut d = Vector3::zeros();
            d[k] = h;
            ji.set_column(
                k,
                &((f(&xi.boxplus(&d), xj) - f(&xi.boxplus(&-d), xj)) / (2.0 * h)),
            );
            jj.set_column(
                k,
                &((f(xi, &xj.boxplus(&d)) - f(xi, &xj.boxplus(&-d))) / (2.0 * h)),
            );
        }
        (ji, jj)
    }

    #[test]
    fn relative_pose_jacobians_match_central_differences_even_for_large_residuals() {
        // The last case has a residual of several metres and about a radian: the regime of a
        // loop closure, in which `J_r⁻¹ ≈ I` is wrong.
        let cases = [
            (
                Se2::new(0.0, 0.0, 0.0),
                Se2::new(1.0, 0.2, 0.3),
                Se2::new(1.0, 0.2, 0.3),
            ),
            (
                Se2::new(1.0, -2.0, 0.7),
                Se2::new(3.0, 0.5, -1.0),
                Se2::new(1.5, 2.0, 0.2),
            ),
            (
                Se2::new(-2.0, 1.0, 2.5),
                Se2::new(4.0, -3.0, 0.4),
                Se2::new(0.5, 0.1, -0.3),
            ),
        ];
        for (xi, xj, z) in cases {
            let r = relative_pose_residual(&xi, &xj, &z);
            let (ji, jj) = numeric_jacobians(&xi, &xj, &z);
            assert!((r.jac_i - ji).abs().max() < 1e-6, "i: {}\n{}", r.jac_i, ji);
            assert!((r.jac_j - jj).abs().max() < 1e-6, "j: {}\n{}", r.jac_j, jj);
        }
    }

    #[test]
    fn residual_is_zero_for_a_consistent_measurement() {
        let xi = Se2::new(1.0, 2.0, 0.4);
        let xj = Se2::new(2.0, 3.0, 1.0);
        let r = relative_pose_residual(&xi, &xj, &xi.between(&xj));
        assert!(r.error.norm() < 1e-12);
    }
}
