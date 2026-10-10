//! Reusable particle-filter machinery: weighted particle sets in log space, effective
//! sample size, low-variance (systematic) and multinomial resampling. Adaptive set sizing
//! lives in [`crate::kld`].
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 8 — Nonparametric Filters
//! (`ParticleSet<S>` with log weights, `low_variance_resample`, ESS-triggered resampling);
//! Thrun, Burgard & Fox, Tables 4.3 and 4.4.
//!
//! Weights are stored as **log** weights so products of many small likelihoods never
//! underflow; normalization uses the log-sum-exp trick. This module deliberately stops at
//! the machinery: Monte Carlo localization (motion + sensor models on a map) is Phase 3.

use crate::FilterError;
use rand::{Rng, RngExt};

/// A hypothesis and its (log) weight.
#[derive(Debug, Clone, PartialEq)]
pub struct Particle<S> {
    /// The hypothesized state.
    pub state: S,
    /// Natural-log importance weight (unnormalized until [`ParticleSet::normalize`]).
    pub log_weight: f64,
}

/// A weighted particle set.
#[derive(Debug, Clone, PartialEq)]
pub struct ParticleSet<S> {
    particles: Vec<Particle<S>>,
}

/// Outcome of a normalization.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Normalization {
    /// `ln Σ exp(log_w)` before normalization, i.e. the log evidence of the last
    /// reweighting (up to the previous normalization constant).
    pub log_normalizer: f64,
    /// Effective sample size after normalization.
    pub ess: f64,
}

impl<S: Clone> ParticleSet<S> {
    /// Particles with equal weight `1/M`.
    pub fn from_states(states: Vec<S>) -> Result<Self, FilterError> {
        if states.is_empty() {
            return Err(FilterError::DegenerateWeights);
        }
        let lw = -(states.len() as f64).ln();
        Ok(Self {
            particles: states
                .into_iter()
                .map(|state| Particle {
                    state,
                    log_weight: lw,
                })
                .collect(),
        })
    }

    /// Number of particles.
    #[inline]
    pub fn len(&self) -> usize {
        self.particles.len()
    }
    /// Always `false` for a constructed set.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.particles.is_empty()
    }
    /// The particles.
    #[inline]
    pub fn particles(&self) -> &[Particle<S>] {
        &self.particles
    }
    /// Iterator over the states.
    pub fn states(&self) -> impl Iterator<Item = &S> {
        self.particles.iter().map(|p| &p.state)
    }

    /// Apply `f` to every state (the proposal / motion step).
    pub fn map_states(&mut self, mut f: impl FnMut(&mut S)) {
        for p in &mut self.particles {
            f(&mut p.state);
        }
    }

    /// Add `log_likelihood(state)` to each log weight (the correction step). Returns an
    /// error, leaving the weights untouched, if any likelihood is NaN or `+∞`.
    pub fn reweight(
        &mut self,
        mut log_likelihood: impl FnMut(&S) -> f64,
    ) -> Result<(), FilterError> {
        let lls: Vec<f64> = self
            .particles
            .iter()
            .map(|p| log_likelihood(&p.state))
            .collect();
        if lls.iter().any(|l| l.is_nan() || *l == f64::INFINITY) {
            return Err(FilterError::NonFinite("log likelihood"));
        }
        for (p, l) in self.particles.iter_mut().zip(lls) {
            p.log_weight += l;
        }
        Ok(())
    }

    /// Normalize so the weights sum to one (log-sum-exp). If every weight is zero (all
    /// log weights `−∞`) the set is **degenerate**: the weights are left untouched and
    /// [`FilterError::DegenerateWeights`] is returned so the caller can decide (reset to
    /// uniform, reinitialize, report).
    pub fn normalize(&mut self) -> Result<Normalization, FilterError> {
        if self
            .particles
            .iter()
            .any(|p| p.log_weight.is_nan() || p.log_weight == f64::INFINITY)
        {
            return Err(FilterError::NonFinite("log weight"));
        }
        let max = self
            .particles
            .iter()
            .map(|p| p.log_weight)
            .fold(f64::NEG_INFINITY, f64::max);
        if max == f64::NEG_INFINITY {
            return Err(FilterError::DegenerateWeights);
        }
        let sum: f64 = self
            .particles
            .iter()
            .map(|p| (p.log_weight - max).exp())
            .sum();
        let log_normalizer = max + sum.ln();
        for p in &mut self.particles {
            p.log_weight -= log_normalizer;
        }
        Ok(Normalization {
            log_normalizer,
            ess: self.effective_sample_size(),
        })
    }

    /// Reset every weight to `1/M`.
    pub fn reset_uniform(&mut self) {
        let lw = -(self.particles.len() as f64).ln();
        for p in &mut self.particles {
            p.log_weight = lw;
        }
    }

    /// Normalized linear weights (computed from the log weights; assumes normalized).
    pub fn weights(&self) -> Vec<f64> {
        self.particles.iter().map(|p| p.log_weight.exp()).collect()
    }

    /// Effective sample size `1 / Σ wᵢ²` of the (normalized) weights.
    pub fn effective_sample_size(&self) -> f64 {
        effective_sample_size(&self.weights())
    }

    /// Low-variance resampling with an RNG-drawn comb offset. Weights become uniform.
    pub fn resample_low_variance<R: Rng + ?Sized>(
        &mut self,
        rng: &mut R,
    ) -> Result<(), FilterError> {
        let m = self.particles.len();
        let r: f64 = rng.random::<f64>() / m as f64;
        self.resample_with_offset(r)
    }

    /// Low-variance resampling with an explicit comb offset `r ∈ [0, 1/M)` (deterministic,
    /// used by tests and replay).
    pub fn resample_with_offset(&mut self, r: f64) -> Result<(), FilterError> {
        self.normalize()?;
        let idx = low_variance_indices(&self.weights(), r)?;
        let lw = -(self.particles.len() as f64).ln();
        self.particles = idx
            .into_iter()
            .map(|i| Particle {
                state: self.particles[i].state.clone(),
                log_weight: lw,
            })
            .collect();
        Ok(())
    }

    /// Multinomial resampling: `M` independent draws with probability `wᵢ` (Thrun
    /// Table 4.3's reference scheme; higher variance than the comb). Weights become
    /// uniform.
    pub fn resample_multinomial<R: Rng + ?Sized>(
        &mut self,
        rng: &mut R,
    ) -> Result<(), FilterError> {
        self.normalize()?;
        let idx = multinomial_indices(&self.weights(), self.particles.len(), rng)?;
        let lw = -(self.particles.len() as f64).ln();
        self.particles = idx
            .into_iter()
            .map(|i| Particle {
                state: self.particles[i].state.clone(),
                log_weight: lw,
            })
            .collect();
        Ok(())
    }

    /// Replace the particles with new states of equal weight `1/M'` (used by adaptive
    /// schemes such as KLD sampling, where `M'` may differ from the current size).
    pub fn replace_uniform(&mut self, states: Vec<S>) -> Result<(), FilterError> {
        *self = Self::from_states(states)?;
        Ok(())
    }

    /// Resample only when `ESS < threshold_ratio · M` (the book uses 0.5). Returns whether
    /// resampling happened.
    pub fn resample_if_needed<R: Rng + ?Sized>(
        &mut self,
        threshold_ratio: f64,
        rng: &mut R,
    ) -> Result<bool, FilterError> {
        FilterError::check_non_negative("threshold_ratio", threshold_ratio)?;
        let n = self.normalize()?;
        if n.ess < threshold_ratio * self.particles.len() as f64 {
            self.resample_low_variance(rng)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }
}

/// `1 / Σ wᵢ²` for normalized weights (0 for an empty slice).
pub fn effective_sample_size(weights: &[f64]) -> f64 {
    let s: f64 = weights.iter().map(|w| w * w).sum();
    if s > 0.0 {
        1.0 / s
    } else {
        0.0
    }
}

/// Inverse-CDF sampler over non-negative weights (normalized internally): each draw is
/// one uniform number and a binary search, `O(log M)`.
#[derive(Debug, Clone, PartialEq)]
pub struct CategoricalSampler {
    cumulative: Vec<f64>,
}

impl CategoricalSampler {
    /// Build from weights; they must be finite, non-negative and not all zero.
    pub fn new(weights: &[f64]) -> Result<Self, FilterError> {
        if weights.is_empty() {
            return Err(FilterError::DegenerateWeights);
        }
        if weights.iter().any(|w| !w.is_finite() || *w < 0.0) {
            return Err(FilterError::NonFinite("weight"));
        }
        let total: f64 = weights.iter().sum();
        if total <= 0.0 {
            return Err(FilterError::DegenerateWeights);
        }
        let mut acc = 0.0;
        let cumulative = weights
            .iter()
            .map(|w| {
                acc += w / total;
                acc
            })
            .collect();
        Ok(Self { cumulative })
    }

    /// Draw one index.
    pub fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> usize {
        let u: f64 = rng.random::<f64>();
        // First index whose cumulative weight exceeds u; the clamp guards the last bin
        // against floating-point shortfall of the running sum.
        self.cumulative
            .partition_point(|c| *c <= u)
            .min(self.cumulative.len() - 1)
    }
}

/// `count` independent draws from the categorical distribution `w / Σw`.
pub fn multinomial_indices<R: Rng + ?Sized>(
    weights: &[f64],
    count: usize,
    rng: &mut R,
) -> Result<Vec<usize>, FilterError> {
    let s = CategoricalSampler::new(weights)?;
    Ok((0..count).map(|_| s.sample(rng)).collect())
}

/// Indices chosen by the low-variance comb `u_k = r + k/M`, `r ∈ [0, 1/M)`.
///
/// Weights must be non-negative and finite; they are normalized internally. The comb
/// draws a single random number, so it has lower variance than multinomial resampling and
/// every particle with weight `≥ 1/M` is guaranteed to survive.
pub fn low_variance_indices(weights: &[f64], r: f64) -> Result<Vec<usize>, FilterError> {
    let m = weights.len();
    if m == 0 {
        return Err(FilterError::DegenerateWeights);
    }
    if weights.iter().any(|w| !w.is_finite() || *w < 0.0) {
        return Err(FilterError::NonFinite("weight"));
    }
    let total: f64 = weights.iter().sum();
    if total <= 0.0 {
        return Err(FilterError::DegenerateWeights);
    }
    let step = 1.0 / m as f64;
    if !(0.0..step).contains(&r) {
        return Err(FilterError::InvalidNoiseParameter {
            what: "comb offset",
            value: r,
        });
    }
    let mut out = Vec::with_capacity(m);
    let mut i = 0;
    let mut c = weights[0] / total;
    for k in 0..m {
        let u = r + k as f64 * step;
        while u > c && i + 1 < m {
            i += 1;
            c += weights[i] / total;
        }
        out.push(i);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use robotics_core::seeded_rng;

    fn set(ws: &[f64]) -> ParticleSet<usize> {
        let mut s = ParticleSet::from_states((0..ws.len()).collect()).unwrap();
        for (p, w) in s.particles.iter_mut().zip(ws) {
            p.log_weight = w.ln();
        }
        s
    }

    /// Chapter 8 worked example: w = [0.10, 0.30, 0.05, 0.40, 0.15] → ESS = 1/0.285 and the
    /// comb offset r = 0.15 (u_k = 0.15 + k/5) yields offspring [0, 2, 0, 2, 1].
    #[test]
    fn worked_example_ch8_ess_and_comb() {
        let w = [0.10, 0.30, 0.05, 0.40, 0.15];
        assert_relative_eq!(effective_sample_size(&w), 3.50877192982456, epsilon = 1e-12);
        // The book's r = 0.15 is on the unit interval; our offset is in [0, 1/M): 0.15 is
        // already < 0.2, so u_k = 0.15 + k·0.2.
        let idx = low_variance_indices(&w, 0.15).unwrap();
        let mut offspring = [0usize; 5];
        for i in idx {
            offspring[i] += 1;
        }
        assert_eq!(offspring, [0, 2, 0, 2, 1]);
    }

    #[test]
    fn weights_normalize_to_one_in_log_space() {
        let mut s = ParticleSet::from_states(vec![0.0f64; 4]).unwrap();
        s.reweight(|_| -1000.0).unwrap(); // would underflow in linear space
        s.particles[2].log_weight += 2.0;
        let n = s.normalize().unwrap();
        assert!(n.log_normalizer.is_finite());
        assert_relative_eq!(s.weights().iter().sum::<f64>(), 1.0, epsilon = 1e-12);
        assert!(s.weights()[2] > s.weights()[0]);
    }

    #[test]
    fn degenerate_and_nan_weights_are_reported() {
        let mut s = ParticleSet::from_states(vec![1u8, 2, 3]).unwrap();
        s.reweight(|_| f64::NEG_INFINITY).unwrap();
        assert_eq!(s.normalize().unwrap_err(), FilterError::DegenerateWeights);
        s.reset_uniform();
        assert_relative_eq!(s.weights().iter().sum::<f64>(), 1.0, epsilon = 1e-12);
        let before = s.clone();
        assert!(s
            .reweight(|x| if *x == 2 { f64::NAN } else { 0.0 })
            .is_err());
        assert_eq!(s, before, "a rejected reweight must not modify the set");
        assert!(low_variance_indices(&[0.0, 0.0], 0.1).is_err());
        assert!(low_variance_indices(&[0.5, f64::NAN], 0.1).is_err());
        assert!(low_variance_indices(&[], 0.0).is_err());
        assert!(ParticleSet::<u8>::from_states(vec![]).is_err());
    }

    #[test]
    fn heavy_particles_always_survive() {
        // Book check #12: weights [.1,.4,.4,.1], M = 4: particles with w ≥ 1/M survive for
        // every comb offset.
        let w = [0.1, 0.4, 0.4, 0.1];
        for k in 0..64 {
            let r = (k as f64 + 0.5) / 64.0 * 0.25;
            let idx = low_variance_indices(&w, r).unwrap();
            assert_eq!(idx.len(), 4);
            assert!(idx.contains(&1) && idx.contains(&2));
        }
    }

    #[test]
    fn deterministic_resampling_under_fixed_seed() {
        let w: Vec<f64> = (0..50)
            .map(|i| ((i as f64) * 0.37).sin().abs() + 0.01)
            .collect();
        let mut a = set(&w);
        let mut b = set(&w);
        a.resample_low_variance(&mut seeded_rng(42)).unwrap();
        b.resample_low_variance(&mut seeded_rng(42)).unwrap();
        assert_eq!(a, b);
        assert_relative_eq!(a.effective_sample_size(), 50.0, epsilon = 1e-9);
    }

    #[test]
    fn resampling_is_unbiased_on_a_large_set() {
        // Invariant of the comb: for every particle *and every contiguous prefix* the number
        // of offspring is within one of the expected count M·Σwᵢ. (Class-wise sums over a
        // periodic weight pattern can alias with the comb, a known property of systematic
        // resampling, so the test uses random weights.)
        let m = 10_000;
        let mut rng = seeded_rng(7);
        let w: Vec<f64> = (0..m).map(|_| rng.random::<f64>() + 1e-3).collect();
        let total: f64 = w.iter().sum();
        let r = rng.random::<f64>() / m as f64;
        let idx = low_variance_indices(&w, r).unwrap();
        assert_eq!(idx.len(), m);
        let mut counts = vec![0usize; m];
        for i in idx {
            counts[i] += 1;
        }
        let (mut got, mut expect) = (0usize, 0.0f64);
        for i in 0..m {
            let e = m as f64 * w[i] / total;
            assert!((counts[i] as f64 - e).abs() < 1.0 + 1e-9);
            got += counts[i];
            expect += e;
            assert!((got as f64 - expect).abs() < 1.0 + 1e-6, "prefix {i}");
        }
    }

    #[test]
    fn multinomial_draws_follow_the_weights() {
        let w = [0.1, 0.6, 0.0, 0.3];
        let mut rng = seeded_rng(4);
        let idx = multinomial_indices(&w, 100_000, &mut rng).unwrap();
        let mut counts = [0usize; 4];
        for i in idx {
            counts[i] += 1;
        }
        assert_eq!(counts[2], 0, "zero-weight particles are never drawn");
        for (c, w) in counts.iter().zip(w) {
            assert!((*c as f64 / 1e5 - w).abs() < 0.01);
        }
        assert!(multinomial_indices(&[0.0, 0.0], 3, &mut rng).is_err());
        assert!(multinomial_indices(&[1.0, -1.0], 3, &mut rng).is_err());

        let mut a = set(&[0.7, 0.1, 0.1, 0.1]);
        let mut b = a.clone();
        a.resample_multinomial(&mut seeded_rng(5)).unwrap();
        b.resample_multinomial(&mut seeded_rng(5)).unwrap();
        assert_eq!(a, b, "seeded multinomial resampling is deterministic");
        assert_eq!(a.len(), 4);
        assert_relative_eq!(a.weights().iter().sum::<f64>(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn resample_if_needed_respects_threshold() {
        let mut rng = seeded_rng(1);
        let mut even = set(&[0.25, 0.25, 0.25, 0.25]);
        assert!(!even.resample_if_needed(0.5, &mut rng).unwrap());
        let mut skewed = set(&[0.97, 0.01, 0.01, 0.01]);
        assert!(skewed.resample_if_needed(0.5, &mut rng).unwrap());
        assert!(skewed.states().all(|s| *s == 0));
    }
}
