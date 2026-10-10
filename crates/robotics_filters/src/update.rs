//! Shared Gaussian-correction machinery used by the EKF and the ESKF.
//!
//! One implementation of "innovation → gain → Joseph covariance" keeps every estimator's
//! numerical safety rules identical: finite-value checks, Cholesky-based solves (never an
//! explicit `S⁻¹` in the gain), a relative pivot test for near-singular innovations, and
//! symmetrization of every covariance that leaves the function.

use crate::FilterError;
use nalgebra::{linalg::Cholesky, Const, SMatrix, SVector};
use std::f64::consts::PI;

/// Numerical tolerances shared by the Gaussian filters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerances {
    /// Maximum allowed asymmetry `max|P − Pᵀ|` relative to `1 + max|P|` before a covariance
    /// is declared invalid (it is symmetrized otherwise).
    pub symmetry: f64,
    /// A Cholesky pivot ratio `min(L_ii)² / max(L_ii)²` below this marks the innovation
    /// covariance as near-singular (≈ inverse condition number).
    pub min_pivot_ratio: f64,
    /// Relative diagonal loading used when checking positive semi-definiteness.
    pub psd_slack: f64,
}

impl Default for Tolerances {
    fn default() -> Self {
        Self {
            symmetry: 1e-6,
            min_pivot_ratio: 1e-14,
            psd_slack: 1e-12,
        }
    }
}

/// Result of one Gaussian correction.
#[derive(Debug, Clone, PartialEq)]
pub struct GaussianCorrection<const N: usize, const M: usize> {
    /// Tangent-space correction `δ = K ν` to be retracted onto the state.
    pub delta: SVector<f64, N>,
    /// Posterior covariance (Joseph form, symmetrized).
    pub covariance: SMatrix<f64, N, N>,
    /// Innovation `ν`.
    pub innovation: SVector<f64, M>,
    /// Innovation covariance `S = H P Hᵀ + R`.
    pub innovation_cov: SMatrix<f64, M, M>,
    /// Kalman gain `K = P Hᵀ S⁻¹`.
    pub gain: SMatrix<f64, N, M>,
    /// Normalized innovation squared `νᵀ S⁻¹ ν`.
    pub nis: f64,
    /// `ln N(ν; 0, S)`.
    pub log_likelihood: f64,
}

/// `½(A + Aᵀ)`.
#[inline]
pub fn symmetrize<const N: usize>(m: &SMatrix<f64, N, N>) -> SMatrix<f64, N, N> {
    (m + m.transpose()) * 0.5
}

/// Check that `p` is finite, (nearly) symmetric and positive semi-definite, and return
/// its symmetrized version.
pub fn validate_covariance<const N: usize>(
    p: &SMatrix<f64, N, N>,
    tol: &Tolerances,
) -> Result<SMatrix<f64, N, N>, FilterError> {
    if p.iter().any(|v| !v.is_finite()) {
        return Err(FilterError::InvalidCovariance("non-finite entry"));
    }
    let scale = 1.0 + p.abs().max();
    if (p - p.transpose()).abs().max() > tol.symmetry * scale {
        return Err(FilterError::InvalidCovariance("not symmetric"));
    }
    let s = symmetrize(p);
    let loaded = s + SMatrix::<f64, N, N>::identity() * (tol.psd_slack * scale);
    if Cholesky::new(loaded).is_none() {
        return Err(FilterError::InvalidCovariance("not positive semi-definite"));
    }
    Ok(s)
}

/// Cholesky factor of an innovation covariance, rejecting singular and near-singular
/// matrices.
pub fn factor_innovation<const M: usize>(
    s: &SMatrix<f64, M, M>,
    tol: &Tolerances,
) -> Result<Cholesky<f64, Const<M>>, FilterError> {
    if s.iter().any(|v| !v.is_finite()) {
        return Err(FilterError::NonFinite("innovation covariance"));
    }
    let chol = Cholesky::new(symmetrize(s)).ok_or(FilterError::SingularInnovation)?;
    let l = chol.l();
    let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
    for i in 0..M {
        let d = l[(i, i)] * l[(i, i)];
        lo = lo.min(d);
        hi = hi.max(d);
    }
    if hi <= 0.0 || lo / hi < tol.min_pivot_ratio {
        return Err(FilterError::SingularInnovation);
    }
    Ok(chol)
}

/// Linear(ized) Gaussian correction in Joseph form.
///
/// * `p` — prior covariance (tangent space of the state)
/// * `h` — measurement Jacobian w.r.t. the tangent error
/// * `r` — measurement noise covariance
/// * `innovation` — `ν = z ⊟ h(x̂)` (already wrapped for angular components)
pub fn joseph_correction<const N: usize, const M: usize>(
    p: &SMatrix<f64, N, N>,
    h: &SMatrix<f64, M, N>,
    r: &SMatrix<f64, M, M>,
    innovation: &SVector<f64, M>,
    tol: &Tolerances,
) -> Result<GaussianCorrection<N, M>, FilterError> {
    if innovation.iter().any(|v| !v.is_finite()) {
        return Err(FilterError::NonFinite("innovation"));
    }
    if h.iter().any(|v| !v.is_finite()) {
        return Err(FilterError::NonFinite("measurement Jacobian"));
    }
    let r = validate_covariance(r, tol)?;
    let s = symmetrize(&(h * p * h.transpose() + r));
    let chol = factor_innovation(&s, tol)?;
    // K = P Hᵀ S⁻¹  ⇔  Kᵀ = S⁻¹ H P  (S symmetric): one Cholesky solve, no explicit inverse.
    let gain = chol.solve(&(h * p)).transpose();
    let delta = gain * innovation;
    let ikh = SMatrix::<f64, N, N>::identity() - gain * h;
    let covariance = symmetrize(&(ikh * p * ikh.transpose() + gain * r * gain.transpose()));
    let covariance = validate_covariance(&covariance, tol)?;
    let nis = innovation.dot(&chol.solve(innovation));
    let ln_det = 2.0 * (0..M).map(|i| chol.l()[(i, i)].ln()).sum::<f64>();
    let log_likelihood = -0.5 * (nis + ln_det + M as f64 * (2.0 * PI).ln());
    Ok(GaussianCorrection {
        delta,
        covariance,
        innovation: *innovation,
        innovation_cov: s,
        gain,
        nis,
        log_likelihood,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix1, Matrix2, Vector1, Vector2};

    #[test]
    fn rejects_singular_and_near_singular_innovation() {
        let h = SMatrix::<f64, 1, 2>::new(1.0, 0.0);
        let e = joseph_correction(
            &Matrix2::zeros(),
            &h,
            &Matrix1::new(0.0),
            &Vector1::new(1.0),
            &Tolerances::default(),
        );
        assert_eq!(e.unwrap_err(), FilterError::SingularInnovation);

        let h2 = Matrix2::new(1.0, 1.0, 1.0, 1.0 + 1e-9);
        let e2 = joseph_correction(
            &Matrix2::identity(),
            &h2,
            &Matrix2::zeros(),
            &Vector2::new(0.1, 0.1),
            &Tolerances::default(),
        );
        assert_eq!(e2.unwrap_err(), FilterError::SingularInnovation);
    }

    #[test]
    fn covariance_validation() {
        let tol = Tolerances::default();
        assert!(validate_covariance(&Matrix2::identity(), &tol).is_ok());
        assert!(validate_covariance(&Matrix2::zeros(), &tol).is_ok());
        let bad = Matrix2::new(1.0, 0.0, 0.0, -1.0);
        assert!(matches!(
            validate_covariance(&bad, &tol),
            Err(FilterError::InvalidCovariance(_))
        ));
        let asym = Matrix2::new(1.0, 0.5, 0.0, 1.0);
        assert!(matches!(
            validate_covariance(&asym, &tol),
            Err(FilterError::InvalidCovariance(_))
        ));
        let nan = Matrix2::new(f64::NAN, 0.0, 0.0, 1.0);
        assert!(validate_covariance(&nan, &tol).is_err());
    }

    #[test]
    fn zero_measurement_noise_pins_the_observed_state() {
        let p = Matrix2::new(1.0, 0.3, 0.3, 2.0);
        let h = SMatrix::<f64, 1, 2>::new(1.0, 0.0);
        let c = joseph_correction(
            &p,
            &h,
            &Matrix1::new(0.0),
            &Vector1::new(0.5),
            &Tolerances::default(),
        )
        .unwrap();
        assert!((c.delta[0] - 0.5).abs() < 1e-12);
        assert!(c.covariance[(0, 0)].abs() < 1e-12);
        assert!((c.covariance[(1, 1)] - (2.0 - 0.09)).abs() < 1e-12);
    }
}
