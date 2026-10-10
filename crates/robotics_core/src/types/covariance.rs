//! Planar pose/twist covariance helpers.

use nalgebra::{linalg::Cholesky, Matrix3};

/// 3×3 covariance over `(x, y, θ)` (or `(v_x, v_y, ω)`), in the tangent space of a pose.
pub type Cov3 = Matrix3<f64>;

/// `½(A + Aᵀ)` for 3×3 matrices.
#[inline]
pub fn symmetrize3(m: &Cov3) -> Cov3 {
    (m + m.transpose()) * 0.5
}

/// A covariance is valid when it is finite, symmetric (to `tol`) and positive
/// semi-definite. Semi-definiteness is checked by attempting a Cholesky factorization of
/// `Σ + tol·I`, which admits exactly-singular matrices such as the zero covariance.
pub fn is_valid_covariance(m: &Cov3, tol: f64) -> bool {
    if m.iter().any(|v| !v.is_finite()) {
        return false;
    }
    if (m - m.transpose()).abs().max() > tol {
        return false;
    }
    Cholesky::new(m + Cov3::identity() * tol).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validity_checks() {
        assert!(is_valid_covariance(&Cov3::zeros(), 1e-9));
        assert!(is_valid_covariance(
            &Cov3::from_diagonal(&nalgebra::Vector3::new(1.0, 2.0, 0.1)),
            1e-9
        ));
        assert!(!is_valid_covariance(
            &Cov3::from_diagonal(&nalgebra::Vector3::new(-1.0, 2.0, 0.1)),
            1e-9
        ));
        let mut asym = Cov3::identity();
        asym[(0, 1)] = 0.5;
        assert!(!is_valid_covariance(&asym, 1e-9));
        assert!(is_valid_covariance(&symmetrize3(&asym), 1e-9));
    }
}
