//! Multivariate Gaussian in moments form `N(μ, Σ)` with a cached Cholesky factor.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 2 (`Gaussian<N>`): never form
//! `Σ⁻¹` explicitly, solve with the Cholesky factor instead; symmetrize before factoring.

use crate::error::CoreError;
use crate::prob::Canonical;
use nalgebra::{linalg::Cholesky, Const, SMatrix, SVector};
use rand::{Rng, RngExt};
use rand_distr::StandardNormal;
use std::f64::consts::PI;

/// `N(μ, Σ)` over `ℝ^N`.
#[derive(Debug, Clone)]
pub struct Gaussian<const N: usize> {
    mean: SVector<f64, N>,
    cov: SMatrix<f64, N, N>,
    chol: Cholesky<f64, Const<N>>,
}

impl<const N: usize> PartialEq for Gaussian<N> {
    fn eq(&self, other: &Self) -> bool {
        self.mean == other.mean && self.cov == other.cov
    }
}

impl<const N: usize> Gaussian<N> {
    /// Construct from a mean and a covariance. The covariance is symmetrized first and
    /// must be positive definite, otherwise [`CoreError::NotPositiveDefinite`] is returned.
    pub fn new(mean: SVector<f64, N>, cov: SMatrix<f64, N, N>) -> Result<Self, CoreError> {
        if mean.iter().any(|v| !v.is_finite()) {
            return Err(CoreError::NonFinite("mean"));
        }
        if cov.iter().any(|v| !v.is_finite()) {
            return Err(CoreError::NonFinite("covariance"));
        }
        let cov = symmetrize(&cov);
        let chol = Cholesky::new(cov).ok_or(CoreError::NotPositiveDefinite)?;
        Ok(Self { mean, cov, chol })
    }

    /// An isotropic Gaussian `N(μ, σ²I)`.
    pub fn isotropic(mean: SVector<f64, N>, sigma: f64) -> Result<Self, CoreError> {
        if sigma.is_nan() || sigma <= 0.0 {
            return Err(CoreError::InvalidValue {
                what: "sigma",
                value: sigma,
            });
        }
        Self::new(mean, SMatrix::<f64, N, N>::identity() * (sigma * sigma))
    }

    /// Mean vector.
    #[inline]
    pub fn mean(&self) -> &SVector<f64, N> {
        &self.mean
    }
    /// Covariance matrix (symmetric positive definite).
    #[inline]
    pub fn cov(&self) -> &SMatrix<f64, N, N> {
        &self.cov
    }
    /// Lower Cholesky factor `L` with `Σ = L Lᵀ`.
    #[inline]
    pub fn cholesky_l(&self) -> SMatrix<f64, N, N> {
        self.chol.l()
    }

    /// `ln det Σ`, computed from the Cholesky diagonal (`2 Σ ln L_ii`).
    pub fn ln_det(&self) -> f64 {
        let l = self.chol.l();
        2.0 * (0..N).map(|i| l[(i, i)].ln()).sum::<f64>()
    }

    /// Squared Mahalanobis distance `(x−μ)ᵀ Σ⁻¹ (x−μ)` via a triangular solve.
    pub fn mahalanobis2(&self, x: &SVector<f64, N>) -> f64 {
        let d = x - self.mean;
        let y = self.chol.solve(&d);
        d.dot(&y)
    }

    /// Log density `ln N(x; μ, Σ)`.
    pub fn ln_pdf(&self, x: &SVector<f64, N>) -> f64 {
        -0.5 * (self.mahalanobis2(x) + self.ln_det() + (N as f64) * (2.0 * PI).ln())
    }

    /// Density `N(x; μ, Σ)`.
    #[inline]
    pub fn pdf(&self, x: &SVector<f64, N>) -> f64 {
        self.ln_pdf(x).exp()
    }

    /// Differential entropy in **nats**: `½ ln det(2πe Σ)`.
    pub fn entropy_nats(&self) -> f64 {
        0.5 * (self.ln_det() + (N as f64) * (1.0 + (2.0 * PI).ln()))
    }

    /// Draw one sample `μ + L z`, `z ~ N(0, I)`.
    pub fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> SVector<f64, N> {
        let z = SVector::<f64, N>::from_fn(|_, _| rng.sample::<f64, _>(StandardNormal));
        self.mean + self.chol.l() * z
    }

    /// Push the Gaussian through the affine map `y = A x + b`.
    ///
    /// Returns an error if the image covariance `A Σ Aᵀ` is singular (e.g. `A` drops rank).
    pub fn transform<const M: usize>(
        &self,
        a: &SMatrix<f64, M, N>,
        b: &SVector<f64, M>,
    ) -> Result<Gaussian<M>, CoreError> {
        Gaussian::new(a * self.mean + b, a * self.cov * a.transpose())
    }

    /// Convert to canonical (information) form `Ω = Σ⁻¹`, `ξ = Σ⁻¹ μ`.
    pub fn to_canonical(&self) -> Canonical<N> {
        let omega = self.chol.inverse();
        let omega = symmetrize(&omega);
        Canonical::new_unchecked(omega * self.mean, omega)
    }

    /// Bayes product of two Gaussians (unnormalized posterior), computed in canonical form.
    pub fn product(&self, other: &Gaussian<N>) -> Result<Gaussian<N>, CoreError> {
        self.to_canonical()
            .product(&other.to_canonical())
            .to_moments()
    }
}

/// `½(A + Aᵀ)`.
#[inline]
pub(crate) fn symmetrize<const N: usize>(m: &SMatrix<f64, N, N>) -> SMatrix<f64, N, N> {
    (m + m.transpose()) * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prob::seeded_rng;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix2, Vector1, Vector2};

    #[test]
    fn rejects_non_pd_covariance() {
        let bad = Matrix2::new(1.0, 2.0, 2.0, 1.0);
        assert_eq!(
            Gaussian::new(Vector2::zeros(), bad),
            Err(CoreError::NotPositiveDefinite)
        );
    }

    /// Chapter 2 worked example: N(5,4) × N(6.5,1) = N(6.2, 0.8).
    #[test]
    fn worked_example_product_of_two_scalar_gaussians() {
        let a = Gaussian::new(Vector1::new(5.0), nalgebra::Matrix1::new(4.0)).unwrap();
        let b = Gaussian::new(Vector1::new(6.5), nalgebra::Matrix1::new(1.0)).unwrap();
        let c = a.to_canonical();
        assert_relative_eq!(c.omega()[(0, 0)], 0.25, epsilon = 1e-12);
        assert_relative_eq!(c.xi()[0], 1.25, epsilon = 1e-12);
        let p = a.product(&b).unwrap();
        assert_relative_eq!(p.mean()[0], 6.2, epsilon = 1e-12);
        assert_relative_eq!(p.cov()[(0, 0)], 0.8, epsilon = 1e-12);
    }

    /// Chapter 2 worked example: the correlated 2-D Gaussian with det Σ = 0.39 and the
    /// entropy gained over its uncorrelated twin.
    #[test]
    fn worked_example_2d_determinant_and_entropy() {
        let corr = Gaussian::new(Vector2::zeros(), Matrix2::new(4.0, 1.9, 1.9, 1.0)).unwrap();
        let diag = Gaussian::new(Vector2::zeros(), Matrix2::new(4.0, 0.0, 0.0, 1.0)).unwrap();
        assert_relative_eq!(corr.ln_det().exp(), 0.39, epsilon = 1e-12);
        assert_relative_eq!(
            diag.entropy_nats() - corr.entropy_nats(),
            1.16395145,
            epsilon = 1e-8
        );
    }

    #[test]
    fn pdf_integrates_to_one_in_2d() {
        let g = Gaussian::new(Vector2::new(0.1, -0.2), Matrix2::new(0.5, 0.2, 0.2, 0.3)).unwrap();
        let h = 0.04;
        let mut mass = 0.0;
        let mut x = -3.5;
        while x <= 3.5 {
            let mut y = -3.5;
            while y <= 3.5 {
                mass += g.pdf(&Vector2::new(x, y)) * h * h;
                y += h;
            }
            x += h;
        }
        assert!((mass - 1.0).abs() < 2e-3, "mass {mass}");
    }

    #[test]
    fn canonical_round_trip_is_identity() {
        let g = Gaussian::new(Vector2::new(1.0, 2.0), Matrix2::new(2.0, 0.3, 0.3, 1.5)).unwrap();
        let back = g.to_canonical().to_moments().unwrap();
        assert!((back.mean() - g.mean()).norm() < 1e-10);
        assert!((back.cov() - g.cov()).norm() < 1e-10);
    }

    #[test]
    fn samples_have_the_right_moments() {
        let g = Gaussian::new(Vector2::new(1.0, -1.0), Matrix2::new(2.0, 0.8, 0.8, 1.0)).unwrap();
        let mut rng = seeded_rng(BOOK_SEED_FOR_TESTS);
        let n = 20_000;
        let mut sum = Vector2::zeros();
        let mut sum_sq = Matrix2::zeros();
        for _ in 0..n {
            let s = g.sample(&mut rng);
            sum += s;
            sum_sq += s * s.transpose();
        }
        let mean = sum / n as f64;
        let cov = sum_sq / n as f64 - mean * mean.transpose();
        assert!((mean - g.mean()).norm() < 0.05, "mean {mean}");
        assert!((cov - g.cov()).norm() < 0.1, "cov {cov}");
    }

    const BOOK_SEED_FOR_TESTS: u64 = crate::prob::BOOK_SEED;
}
