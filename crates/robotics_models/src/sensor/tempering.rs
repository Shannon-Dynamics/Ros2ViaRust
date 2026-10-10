//! Likelihood tempering: `p(z | x)^β`, `0 < β ≤ 1`, applied in log space.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 10 ("The lie of independence":
//! `log p(z | x)^{1/κ} = (1/κ) Σ_k log p(z_k | x, m)`, `Tempered<S: SensorModel>`);
//! Bissiri, Holmes & Walker (2016) for the generalized-Bayes reading.
//!
//! # Why
//! Scan models multiply hundreds of beam likelihoods as if the beams were independent.
//! They are not (a person blocks twenty adjacent beams; a 1 % map error biases all of
//! them), so the product overstates every log-odds by roughly `K / K_eff` and the
//! posterior collapses around a possibly wrong answer. Tempering with `β = 1/κ ≈ K_eff/K`
//! divides the log-likelihood — flattening the surface **without moving its peak** and
//! without discarding data (subsampling beams moves the peak; inflating `σ_hit` changes
//! which beams count as outliers). It is also the standard remedy when a particle set
//! would otherwise collapse onto a single particle after one very informative update:
//! [`Tempering::for_target_ess`] picks the largest `β` that keeps the effective sample
//! size above a target.
//!
//! # When not to
//! `β = 1` is exact Bayes for truly independent measurements. `β` is a calibration
//! parameter: choose the smallest value whose credible sets cover the truth at their
//! nominal rate on logged data with ground truth (the book finds `κ ≈ 10` for a 1 % map
//! error and `κ = 1` for a perfect map).
//!
//! The mechanism is model-agnostic: it operates on log-likelihood values or wraps any
//! closure `Fn(&X) -> f64`, so it composes with the beam model, the likelihood field, the
//! landmark model and `ParticleSet::reweight` alike.

use crate::ModelError;

/// A validated tempering exponent `β ∈ (0, 1]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tempering {
    beta: f64,
}

impl Default for Tempering {
    fn default() -> Self {
        Self::NONE
    }
}

impl Tempering {
    /// `β = 1`: no tempering.
    pub const NONE: Tempering = Tempering { beta: 1.0 };

    /// `β ∈ (0, 1]`.
    pub fn new(beta: f64) -> Result<Self, ModelError> {
        if beta.is_finite() && beta > 0.0 && beta <= 1.0 {
            Ok(Self { beta })
        } else {
            Err(ModelError::InvalidData("tempering beta must be in (0, 1]"))
        }
    }

    /// The book's parametrization `p^{1/κ}`, `κ ≥ 1`.
    pub fn from_kappa(kappa: f64) -> Result<Self, ModelError> {
        if kappa.is_finite() && kappa >= 1.0 {
            Self::new(1.0 / kappa)
        } else {
            Err(ModelError::InvalidData("tempering kappa must be >= 1"))
        }
    }

    /// `β = K_eff / K` for `K` measurements carrying `K_eff` independent measurements'
    /// worth of information (`1 ≤ K_eff ≤ K`).
    pub fn from_effective_count(total: usize, effective: usize) -> Result<Self, ModelError> {
        if total == 0 || effective == 0 || effective > total {
            return Err(ModelError::InvalidData(
                "effective count must satisfy 1 <= effective <= total",
            ));
        }
        Self::new(effective as f64 / total as f64)
    }

    /// The exponent `β`.
    #[inline]
    pub fn beta(&self) -> f64 {
        self.beta
    }

    /// `β · ln p`. `−∞` (zero likelihood) stays `−∞`; any finite value stays finite.
    #[inline]
    pub fn apply_log(&self, log_likelihood: f64) -> f64 {
        self.beta * log_likelihood
    }

    /// `p^β` for a likelihood `p ≥ 0`, computed as `exp(β ln p)` so subnormal inputs do not
    /// flush to zero. Returns NaN for negative or NaN input.
    #[inline]
    pub fn apply(&self, likelihood: f64) -> f64 {
        if likelihood.is_nan() || likelihood < 0.0 {
            f64::NAN
        } else if likelihood == 0.0 {
            0.0
        } else {
            (self.beta * likelihood.ln()).exp()
        }
    }

    /// Wrap a log-likelihood function `x ↦ ln p(z | x)` into its tempered version.
    pub fn wrap<X, F: Fn(&X) -> f64>(self, log_likelihood: F) -> impl Fn(&X) -> f64 {
        move |x| self.apply_log(log_likelihood(x))
    }

    /// The largest `β ∈ [β_min, 1]` whose tempered weights `w_i ∝ exp(log_prior_i + β ℓ_i)`
    /// keep `ESS ≥ target_ratio · N` (bisection, 50 iterations). Returns `β = 1` when the
    /// untempered update already satisfies the target and `β_min` when even that is not
    /// enough. Non-finite `ℓ_i = −∞` entries are allowed (zero likelihood); NaN / `+∞`
    /// are rejected.
    pub fn for_target_ess(
        log_prior: &[f64],
        log_likelihoods: &[f64],
        target_ratio: f64,
        beta_min: f64,
    ) -> Result<Self, ModelError> {
        if log_prior.len() != log_likelihoods.len() || log_prior.is_empty() {
            return Err(ModelError::InvalidData(
                "prior and likelihoods must match and be non-empty",
            ));
        }
        if !(target_ratio > 0.0 && target_ratio <= 1.0) {
            return Err(ModelError::InvalidData(
                "target ESS ratio must be in (0, 1]",
            ));
        }
        let min = Self::new(beta_min)?;
        if log_prior
            .iter()
            .chain(log_likelihoods)
            .any(|v| v.is_nan() || *v == f64::INFINITY)
        {
            return Err(ModelError::InvalidData(
                "log values must not be NaN or +inf",
            ));
        }
        let n = log_prior.len() as f64;
        let ess_ratio = |beta: f64| -> f64 {
            let lw: Vec<f64> = log_prior
                .iter()
                .zip(log_likelihoods)
                .map(|(p, l)| p + beta * l)
                .collect();
            let max = lw.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            if max == f64::NEG_INFINITY {
                return 0.0;
            }
            let (s, s2) = lw.iter().fold((0.0, 0.0), |(s, s2), l| {
                let w = (l - max).exp();
                (s + w, s2 + w * w)
            });
            s * s / s2 / n
        };
        if ess_ratio(1.0) >= target_ratio {
            return Ok(Self::NONE);
        }
        if ess_ratio(min.beta) < target_ratio {
            return Ok(min);
        }
        let (mut lo, mut hi) = (min.beta, 1.0);
        for _ in 0..50 {
            let mid = 0.5 * (lo + hi);
            if ess_ratio(mid) >= target_ratio {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Self::new(lo)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn normalized(lw: &[f64]) -> Vec<f64> {
        let max = lw.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let s: f64 = lw.iter().map(|l| (l - max).exp()).sum();
        lw.iter().map(|l| (l - max).exp() / s).collect()
    }

    #[test]
    fn beta_one_preserves_the_likelihood() {
        let t = Tempering::NONE;
        for ll in [-1234.5, -0.3, 0.0, 2.7] {
            assert_eq!(t.apply_log(ll), ll);
        }
        assert_relative_eq!(t.apply(0.37), 0.37, epsilon = 1e-15);
        let f = Tempering::default().wrap(|x: &f64| -x * x);
        assert_eq!(f(&3.0), -9.0);
    }

    #[test]
    fn smaller_beta_flattens_without_moving_the_peak() {
        // Log-likelihood of a Gaussian N(0, σ²) over a grid.
        let grid: Vec<f64> = (-100..=100).map(|i| i as f64 * 0.02).collect();
        let ll: Vec<f64> = grid.iter().map(|x| -0.5 * (x / 0.2).powi(2)).collect();
        let entropy = |w: &[f64]| {
            -w.iter()
                .filter(|p| **p > 0.0)
                .map(|p| p * p.ln())
                .sum::<f64>()
        };
        let mut last_entropy = 0.0;
        for beta in [1.0, 0.5, 0.1, 0.01] {
            let t = Tempering::new(beta).unwrap();
            let w = normalized(&ll.iter().map(|l| t.apply_log(*l)).collect::<Vec<_>>());
            let argmax = w
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(b.1))
                .unwrap()
                .0;
            assert_eq!(grid[argmax], 0.0, "peak must not move (beta {beta})");
            let h = entropy(&w);
            assert!(
                h > last_entropy,
                "beta {beta}: entropy {h} <= {last_entropy}"
            );
            last_entropy = h;
        }
        // N(x; 0, σ²)^β ∝ N(x; 0, σ²/β): log-ratios between two points scale by β.
        let t = Tempering::new(0.25).unwrap();
        let (a, b) = (-0.5 * (0.1f64 / 0.2).powi(2), -0.5 * (0.3f64 / 0.2).powi(2));
        let widened = |x: f64| -0.5 * x * x / (0.04 / 0.25);
        assert_relative_eq!(
            t.apply_log(a) - t.apply_log(b),
            widened(0.1) - widened(0.3),
            epsilon = 1e-12
        );
    }

    #[test]
    fn extreme_values_stay_finite_and_ordered() {
        let t = Tempering::new(0.1).unwrap();
        assert!(t.apply_log(-1e308).is_finite());
        assert_eq!(t.apply_log(f64::NEG_INFINITY), f64::NEG_INFINITY);
        // A subnormal likelihood stays positive and becomes representable.
        let p = t.apply(1e-320);
        assert!(p > 0.0 && p.is_finite());
        assert_relative_eq!(p.ln(), 0.1 * (1e-320f64).ln(), max_relative = 1e-12);
        assert_eq!(t.apply(0.0), 0.0);
        assert!(t.apply(-1.0).is_nan());
        // A 360-beam scan whose product underflows in linear space still orders
        // hypotheses correctly in log space after tempering.
        let good = t.apply_log(360.0 * (1e-3f64).ln());
        let bad = t.apply_log(360.0 * (1e-4f64).ln());
        assert!(good.is_finite() && bad.is_finite() && good > bad);
    }

    #[test]
    fn invalid_parameters_are_rejected() {
        for b in [0.0, -0.5, 1.5, f64::NAN, f64::INFINITY] {
            assert!(Tempering::new(b).is_err(), "{b}");
        }
        assert!(Tempering::from_kappa(0.5).is_err());
        assert_relative_eq!(Tempering::from_kappa(10.0).unwrap().beta(), 0.1);
        assert_relative_eq!(
            Tempering::from_effective_count(180, 18).unwrap().beta(),
            0.1
        );
        assert!(Tempering::from_effective_count(10, 20).is_err());
        assert!(Tempering::from_effective_count(0, 0).is_err());
    }

    #[test]
    fn adaptive_beta_meets_the_ess_target() {
        // 100 equally weighted particles, one very informative update.
        let prior = vec![-(100f64).ln(); 100];
        let ll: Vec<f64> = (0..100)
            .map(|i| -0.5 * ((i as f64 - 50.0) / 2.0).powi(2))
            .collect();
        let t = Tempering::for_target_ess(&prior, &ll, 0.5, 1e-6).unwrap();
        assert!(t.beta() < 1.0);
        let w = normalized(
            &prior
                .iter()
                .zip(&ll)
                .map(|(p, l)| p + t.apply_log(*l))
                .collect::<Vec<_>>(),
        );
        let ess = 1.0 / w.iter().map(|x| x * x).sum::<f64>();
        assert!((ess / 100.0 - 0.5).abs() < 1e-3, "ess {ess}");
        // A flat update needs no tempering.
        let flat = vec![-3.0; 100];
        assert_eq!(
            Tempering::for_target_ess(&prior, &flat, 0.5, 1e-6).unwrap(),
            Tempering::NONE
        );
        // Zero-likelihood particles are allowed; NaN is not.
        let mut some_dead = ll.clone();
        some_dead[0] = f64::NEG_INFINITY;
        assert!(Tempering::for_target_ess(&prior, &some_dead, 0.5, 1e-6).is_ok());
        some_dead[1] = f64::NAN;
        assert!(Tempering::for_target_ess(&prior, &some_dead, 0.5, 1e-6).is_err());
        assert!(Tempering::for_target_ess(&prior, &ll[1..], 0.5, 1e-6).is_err());
    }
}
