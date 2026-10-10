//! Filter consistency statistics: NEES and NIS with χ² acceptance bounds.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 6 (`nees`, `nis`: "NEES needs
//! ground truth, so it exists only in simulation; NIS runs on the real robot");
//! Bar-Shalom, Li & Kirubarajan, *Estimation with Applications to Tracking and
//! Navigation*, §5.4.
//!
//! * **NEES** `ε = eᵀ P⁻¹ e`, `e` = estimation error in the filter's own coordinates
//!   (for a manifold state: `e = x_true ⊟ x̂`, the tangent error at the estimate). For a
//!   consistent filter `ε ~ χ²_N`, `E[ε] = N`.
//! * **NIS** `ν = rᵀ S⁻¹ r`, `r` = innovation, `S` = innovation covariance;
//!   `ν ~ χ²_M`, `E[ν] = M`.
//!
//! [`ConsistencyStats`] accumulates either statistic and tests the time average against
//! the two-sided χ² interval of `n` samples. Both quadratic forms are evaluated with a
//! Cholesky solve (never an explicit inverse).

use crate::bayes::FilterError;
use crate::update::{factor_innovation, Tolerances};
use nalgebra::{SMatrix, SVector};
use robotics_core::prob::chi2_mean_interval;

/// Normalized estimation error squared `eᵀ P⁻¹ e`.
pub fn nees<const N: usize>(
    error: &SVector<f64, N>,
    covariance: &SMatrix<f64, N, N>,
) -> Result<f64, FilterError> {
    quadratic_form(error, covariance)
}

/// Normalized innovation squared `rᵀ S⁻¹ r`.
pub fn nis<const M: usize>(
    innovation: &SVector<f64, M>,
    innovation_cov: &SMatrix<f64, M, M>,
) -> Result<f64, FilterError> {
    quadratic_form(innovation, innovation_cov)
}

fn quadratic_form<const N: usize>(
    v: &SVector<f64, N>,
    m: &SMatrix<f64, N, N>,
) -> Result<f64, FilterError> {
    if v.iter().any(|x| !x.is_finite()) {
        return Err(FilterError::NonFinite("error vector"));
    }
    let chol = factor_innovation(m, &Tolerances::default())?;
    Ok(v.dot(&chol.solve(v)))
}

/// Running average of a χ²-distributed consistency statistic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConsistencyStats {
    dof: usize,
    n: usize,
    sum: f64,
    max: f64,
}

/// Verdict of a consistency test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConsistencyReport {
    /// Degrees of freedom of one sample.
    pub dof: usize,
    /// Number of samples.
    pub samples: usize,
    /// Time-averaged statistic (expected value `dof` when consistent).
    pub mean: f64,
    /// Largest single sample.
    pub max: f64,
    /// Lower bound of the `1 − α` acceptance interval for the mean.
    pub lower: f64,
    /// Upper bound of the `1 − α` acceptance interval for the mean.
    pub upper: f64,
}

impl ConsistencyReport {
    /// `mean` inside `[lower, upper]`.
    pub fn is_consistent(&self) -> bool {
        self.mean >= self.lower && self.mean <= self.upper
    }
    /// `mean > upper`: the filter is over-confident (covariance too small).
    pub fn is_overconfident(&self) -> bool {
        self.mean > self.upper
    }
}

impl ConsistencyStats {
    /// Accumulator for samples with `dof` degrees of freedom.
    pub fn new(dof: usize) -> Result<Self, FilterError> {
        if dof == 0 {
            return Err(FilterError::InvalidNoiseParameter {
                what: "degrees of freedom",
                value: 0.0,
            });
        }
        Ok(Self {
            dof,
            n: 0,
            sum: 0.0,
            max: 0.0,
        })
    }

    /// Add one sample (NEES or NIS value). Non-finite or negative values are rejected.
    pub fn push(&mut self, value: f64) -> Result<(), FilterError> {
        FilterError::check_non_negative("consistency sample", value)?;
        self.n += 1;
        self.sum += value;
        self.max = self.max.max(value);
        Ok(())
    }

    /// Number of samples.
    pub fn samples(&self) -> usize {
        self.n
    }

    /// Mean of the samples (`NaN` when empty).
    pub fn mean(&self) -> f64 {
        if self.n == 0 {
            f64::NAN
        } else {
            self.sum / self.n as f64
        }
    }

    /// Two-sided test at significance `alpha` (e.g. 0.05). Assumes independent samples;
    /// successive filter outputs are correlated, so treat the interval as indicative.
    pub fn report(&self, alpha: f64) -> Result<ConsistencyReport, FilterError> {
        let (lower, upper) = chi2_mean_interval(self.dof, self.n.max(1), alpha)?;
        Ok(ConsistencyReport {
            dof: self.dof,
            samples: self.n,
            mean: self.mean(),
            max: self.max,
            lower,
            upper,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix2, Vector2};
    use robotics_core::{seeded_rng, Gaussian};

    #[test]
    fn quadratic_forms() {
        let p = Matrix2::new(4.0, 0.0, 0.0, 0.25);
        assert_relative_eq!(
            nees(&Vector2::new(2.0, 0.5), &p).unwrap(),
            1.0 + 1.0,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            nis(&Vector2::new(1.0, 0.0), &Matrix2::identity()).unwrap(),
            1.0
        );
        assert!(nees(&Vector2::new(f64::NAN, 0.0), &p).is_err());
        assert!(nis(&Vector2::new(1.0, 0.0), &Matrix2::zeros()).is_err());
    }

    #[test]
    fn samples_from_the_model_are_consistent() {
        // Errors drawn from N(0, P) must give mean NEES ≈ 2 and pass the test; errors
        // drawn from N(0, 4P) (an over-confident filter) must fail it.
        let p = Matrix2::new(0.5, 0.2, 0.2, 0.3);
        let g = Gaussian::new(Vector2::zeros(), p).unwrap();
        let wide = Gaussian::new(Vector2::zeros(), p * 4.0).unwrap();
        let mut rng = seeded_rng(9);
        let (mut ok, mut bad) = (
            ConsistencyStats::new(2).unwrap(),
            ConsistencyStats::new(2).unwrap(),
        );
        for _ in 0..2000 {
            ok.push(nees(&g.sample(&mut rng), &p).unwrap()).unwrap();
            bad.push(nees(&wide.sample(&mut rng), &p).unwrap()).unwrap();
        }
        let r = ok.report(0.01).unwrap();
        assert!(r.is_consistent(), "{r:?}");
        assert!((r.mean - 2.0).abs() < 0.2);
        let r = bad.report(0.01).unwrap();
        assert!(r.is_overconfident(), "{r:?}");
        assert!(ConsistencyStats::new(0).is_err());
        assert!(ok.push(-1.0).is_err());
        assert!(ok.push(f64::NAN).is_err());
    }
}
