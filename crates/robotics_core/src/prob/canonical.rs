//! Gaussian in canonical (information) form `(ξ, Ω)` with `Ω = Σ⁻¹`, `ξ = Σ⁻¹ μ`.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 2 (`Canonical<N>`) and Chapter 6
//! (the information filter). Multiplying Gaussians is addition in this form.

use crate::error::CoreError;
use crate::prob::gaussian::symmetrize;
use crate::prob::Gaussian;
use nalgebra::{linalg::Cholesky, SMatrix, SVector};

/// Information-form Gaussian.
#[derive(Debug, Clone, PartialEq)]
pub struct Canonical<const N: usize> {
    xi: SVector<f64, N>,
    omega: SMatrix<f64, N, N>,
}

impl<const N: usize> Canonical<N> {
    /// Build from an information vector and matrix without checking definiteness.
    ///
    /// A zero `Ω` is a legitimate "no information" prior in this form, which is why the
    /// constructor does not insist on positive definiteness; [`Canonical::to_moments`]
    /// checks it when the inverse is actually needed.
    pub fn new_unchecked(xi: SVector<f64, N>, omega: SMatrix<f64, N, N>) -> Self {
        Self {
            xi,
            omega: symmetrize(&omega),
        }
    }

    /// The uninformative prior `ξ = 0`, `Ω = 0`.
    pub fn uninformative() -> Self {
        Self {
            xi: SVector::zeros(),
            omega: SMatrix::zeros(),
        }
    }

    /// Information vector `ξ`.
    #[inline]
    pub fn xi(&self) -> &SVector<f64, N> {
        &self.xi
    }
    /// Information matrix `Ω`.
    #[inline]
    pub fn omega(&self) -> &SMatrix<f64, N, N> {
        &self.omega
    }

    /// Product of two Gaussians: `(ξ₁ + ξ₂, Ω₁ + Ω₂)`.
    #[inline]
    pub fn product(&self, other: &Self) -> Self {
        Self {
            xi: self.xi + other.xi,
            omega: self.omega + other.omega,
        }
    }

    /// Convert back to moments form; fails if `Ω` is not positive definite.
    pub fn to_moments(&self) -> Result<Gaussian<N>, CoreError> {
        let chol = Cholesky::new(self.omega).ok_or(CoreError::NotPositiveDefinite)?;
        let cov = symmetrize(&chol.inverse());
        let mean = chol.solve(&self.xi);
        Gaussian::new(mean, cov)
    }
}
