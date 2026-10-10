//! KLD-adaptive particle-set sizing (KLD sampling).
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 8 (`KLD_sample_size(k, ε, δ)`);
//! Fox, "Adapting the Sample Size in Particle Filters Through KLD-Sampling" (IJRR 2003),
//! eq. 7; Thrun, Burgard & Fox, Table 8.4.
//!
//! # The bound
//!
//! If the true posterior is a discrete distribution over `k` occupied bins and `M`
//! samples are drawn from it, `2M·KL(p̂ ‖ p)` is asymptotically `χ²_{k−1}`. Requiring
//! `P(KL ≤ ε) = 1 − δ` gives `M = χ²_{k−1, 1−δ} / 2ε`, and the Wilson–Hilferty
//! approximation of the quantile yields the formula implemented by [`kld_bound`]:
//!
//! ```text
//!           k − 1  ⎡      2           ⎛    2     ⎞½       ⎤³
//!   M_χ  =  ─────  ⎢ 1 − ─────── +    ⎜ ──────── ⎟  z₁₋δ  ⎥ ,     z₁₋δ = Φ⁻¹(1 − δ)
//!            2ε    ⎣     9(k−1)       ⎝  9(k−1)  ⎠        ⎦
//! ```
//!
//! and `M = clamp(⌈M_χ⌉, M_min, M_max)` ([`KldConfig::required`]). `k ≤ 1` gives
//! `M_χ = 0`, i.e. `M_min`. **`k` counts occupied bins, not grid bins**, so the bound falls
//! as the belief condenses.
//!
//! # Binning is a policy, not part of the algorithm
//!
//! [`ParticleBinning`] maps a state to a hashable bin key. The algorithm never assumes a
//! state type: [`GridBinning`] bins vectors, [`Se2Binning`] bins planar poses (the MCL
//! choice, one implementation among others), and [`BinFn`] wraps any closure.
//!
//! # The sampling loop ([`kld_resample`])
//!
//! Draw an index by weight, push the drawn state through the proposal (typically the
//! motion model), bin it, and if its bin is new recompute `M_χ`; stop once
//! `M ≥ max(M_min, M_χ)` or `M = M_max`. The returned set has uniform weights: the caller
//! applies the measurement update afterwards (Table 8.4 does both in one loop; separating
//! them keeps the sensor model out of the particle machinery).

use crate::bayes::FilterError;
use crate::particle::{CategoricalSampler, ParticleSet};
use nalgebra::SVector;
use rand::Rng;
use robotics_core::prob::normal_quantile;
use robotics_core::Se2;
use std::collections::HashSet;
use std::f64::consts::{PI, TAU};
use std::hash::Hash;

/// Maps a particle state to the histogram bin it occupies.
pub trait ParticleBinning<S> {
    /// Bin identifier.
    type Bin: Hash + Eq;
    /// The bin of `state`.
    fn bin(&self, state: &S) -> Self::Bin;
}

/// Adapter turning a closure `Fn(&S) -> K` into a [`ParticleBinning`].
#[derive(Debug, Clone, Copy)]
pub struct BinFn<F>(pub F);

impl<S, K: Hash + Eq, F: Fn(&S) -> K> ParticleBinning<S> for BinFn<F> {
    type Bin = K;
    fn bin(&self, state: &S) -> K {
        (self.0)(state)
    }
}

/// Axis-aligned grid over `ℝ^N` with per-axis resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridBinning<const N: usize> {
    resolution: SVector<f64, N>,
}

impl<const N: usize> GridBinning<N> {
    /// Every resolution must be positive and finite.
    pub fn new(resolution: SVector<f64, N>) -> Result<Self, FilterError> {
        for r in resolution.iter() {
            FilterError::check_positive("bin resolution", *r)?;
        }
        Ok(Self { resolution })
    }
}

impl<const N: usize> ParticleBinning<SVector<f64, N>> for GridBinning<N> {
    type Bin = [i64; N];
    fn bin(&self, s: &SVector<f64, N>) -> [i64; N] {
        std::array::from_fn(|i| (s[i] / self.resolution[i]).floor() as i64)
    }
}

/// Planar-pose bins: square `xy` cells and heading sectors that wrap at ±π, so poses on
/// either side of the branch cut share a bin.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Se2Binning {
    xy: f64,
    theta: f64,
    sectors: i64,
}

impl Se2Binning {
    /// `xy` cell size (m) and heading sector (rad); the book and AMCL use 0.5 m / 10°.
    pub fn new(xy: f64, theta: f64) -> Result<Self, FilterError> {
        FilterError::check_positive("xy bin size", xy)?;
        FilterError::check_positive("heading bin size", theta)?;
        if theta > TAU {
            return Err(FilterError::InvalidNoiseParameter {
                what: "heading bin size",
                value: theta,
            });
        }
        Ok(Self {
            xy,
            theta,
            sectors: (TAU / theta).ceil() as i64,
        })
    }
}

impl ParticleBinning<Se2> for Se2Binning {
    type Bin = (i64, i64, i64);
    fn bin(&self, p: &Se2) -> (i64, i64, i64) {
        let sector = (((p.theta() + PI) / self.theta).floor() as i64).rem_euclid(self.sectors);
        (
            (p.x() / self.xy).floor() as i64,
            (p.y() / self.xy).floor() as i64,
            sector,
        )
    }
}

/// KLD sampling parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KldConfig {
    /// Lower bound on the particle count (`M_min ≥ 1`).
    pub min_particles: usize,
    /// Upper bound on the particle count (`M_max ≥ M_min`), the compute budget.
    pub max_particles: usize,
    /// Tolerated KL divergence `ε > 0` between the sample-based and the true posterior.
    pub epsilon: f64,
    /// Failure probability `δ ∈ (0, 1)`; the bound holds with confidence `1 − δ`.
    pub delta: f64,
}

impl Default for KldConfig {
    /// The book's values: `ε = 0.05`, `δ = 0.01`, `M ∈ [100, 5000]` (AMCL defaults
    /// `kld_err = 0.01`, `kld_z = 0.99` are a different operating point).
    fn default() -> Self {
        Self {
            min_particles: 100,
            max_particles: 5000,
            epsilon: 0.05,
            delta: 0.01,
        }
    }
}

impl KldConfig {
    /// Validate the configuration.
    pub fn validated(self) -> Result<Self, FilterError> {
        if self.min_particles == 0 {
            return Err(FilterError::InvalidNoiseParameter {
                what: "min_particles",
                value: 0.0,
            });
        }
        if self.max_particles < self.min_particles {
            return Err(FilterError::InvalidNoiseParameter {
                what: "max_particles",
                value: self.max_particles as f64,
            });
        }
        FilterError::check_positive("epsilon", self.epsilon)?;
        if !(self.delta > 0.0 && self.delta < 1.0) {
            return Err(FilterError::InvalidNoiseParameter {
                what: "delta",
                value: self.delta,
            });
        }
        Ok(self)
    }

    /// `z₁₋δ = Φ⁻¹(1 − δ)`.
    pub fn z(&self) -> Result<f64, FilterError> {
        Ok(normal_quantile(1.0 - self.delta)?)
    }

    /// `clamp(⌈M_χ(k)⌉, M_min, M_max)`.
    pub fn required(&self, occupied_bins: usize) -> Result<usize, FilterError> {
        let cfg = self.validated()?;
        let m = kld_sample_size(occupied_bins, cfg.epsilon, cfg.delta)?;
        Ok(m.clamp(cfg.min_particles, cfg.max_particles))
    }
}

/// The raw real-valued bound `M_χ(k)` for a given `z₁₋δ` (0 for `k ≤ 1`).
pub fn kld_bound(occupied_bins: usize, epsilon: f64, z: f64) -> f64 {
    if occupied_bins <= 1 {
        return 0.0;
    }
    let k1 = (occupied_bins - 1) as f64;
    let a = 2.0 / (9.0 * k1);
    let c = 1.0 - a + a.sqrt() * z;
    k1 / (2.0 * epsilon) * c * c * c
}

/// `KLD_sample_size(k, ε, δ) = ⌈M_χ(k)⌉` without clamping (0 for `k ≤ 1`).
pub fn kld_sample_size(
    occupied_bins: usize,
    epsilon: f64,
    delta: f64,
) -> Result<usize, FilterError> {
    FilterError::check_positive("epsilon", epsilon)?;
    let z = normal_quantile(1.0 - delta)?;
    Ok(kld_bound(occupied_bins, epsilon, z).ceil() as usize)
}

/// Number of distinct bins occupied by `states`.
pub fn occupied_bins<'a, S: 'a, B: ParticleBinning<S>>(
    states: impl IntoIterator<Item = &'a S>,
    binning: &B,
) -> usize {
    states
        .into_iter()
        .map(|s| binning.bin(s))
        .collect::<HashSet<_>>()
        .len()
}

/// Outcome of one KLD sampling pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KldReport {
    /// Particles drawn (the new set size).
    pub particles: usize,
    /// Bins occupied by the new set.
    pub occupied_bins: usize,
    /// `⌈M_χ⌉` at the end of the loop (unclamped).
    pub bound: usize,
    /// Whether the loop stopped at `M_max` rather than at the bound.
    pub hit_max: bool,
}

/// KLD sampling: draw from `set` by weight, apply `propose`, and grow the new set until
/// it satisfies the KLD bound for the bins it occupies. See the module documentation.
pub fn kld_resample<S, B, R, P>(
    set: &ParticleSet<S>,
    cfg: &KldConfig,
    binning: &B,
    mut propose: P,
    rng: &mut R,
) -> Result<(ParticleSet<S>, KldReport), FilterError>
where
    S: Clone,
    B: ParticleBinning<S>,
    R: Rng + ?Sized,
    P: FnMut(&S, &mut R) -> S,
{
    let cfg = cfg.validated()?;
    let z = cfg.z()?;
    // Linear weights from log weights, shifted by the maximum (normalization is implicit).
    let max = set
        .particles()
        .iter()
        .map(|p| p.log_weight)
        .fold(f64::NEG_INFINITY, f64::max);
    if !max.is_finite() {
        return Err(if max.is_nan() || max == f64::INFINITY {
            FilterError::NonFinite("log weight")
        } else {
            FilterError::DegenerateWeights
        });
    }
    let weights: Vec<f64> = set
        .particles()
        .iter()
        .map(|p| (p.log_weight - max).exp())
        .collect();
    let sampler = CategoricalSampler::new(&weights)?;

    let mut out = Vec::with_capacity(cfg.min_particles);
    let mut bins = HashSet::new();
    let mut bound = 0usize;
    loop {
        let i = sampler.sample(rng);
        let x = propose(&set.particles()[i].state, rng);
        if bins.insert(binning.bin(&x)) {
            bound = kld_bound(bins.len(), cfg.epsilon, z).ceil() as usize;
        }
        out.push(x);
        let m = out.len();
        if m >= cfg.max_particles || (m >= cfg.min_particles && m >= bound) {
            break;
        }
    }
    let report = KldReport {
        particles: out.len(),
        occupied_bins: bins.len(),
        bound,
        hit_max: out.len() >= cfg.max_particles && bound > cfg.max_particles,
    };
    Ok((ParticleSet::from_states(out)?, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector1;
    use rand::RngExt;
    use rand_distr::{Distribution, Normal};
    use robotics_core::seeded_rng;

    /// Chapter 8 table: ε = 0.05, δ = 0.01 (z = 2.3263).
    #[test]
    fn worked_example_ch8_table() {
        assert_eq!(kld_sample_size(3, 0.05, 0.01).unwrap(), 93);
        assert_eq!(kld_sample_size(10, 0.05, 0.01).unwrap(), 217);
        assert_eq!(kld_sample_size(100, 0.05, 0.01).unwrap(), 1347);
        assert_eq!(kld_sample_size(500, 0.05, 0.01).unwrap(), 5755);
        assert_eq!(kld_sample_size(1, 0.05, 0.01).unwrap(), 0);
        assert_eq!(kld_sample_size(0, 0.05, 0.01).unwrap(), 0);
        // M_χ ≈ χ²_{k−1,1−δ}/2ε: for k − 1 = 100 the exact quantile is 135.807.
        let z = normal_quantile(0.99).unwrap();
        assert!((kld_bound(101, 0.05, z) - 1358.07).abs() < 1.0);
    }

    #[test]
    fn bound_grows_with_occupied_bins_and_is_clamped() {
        let cfg = KldConfig {
            min_particles: 50,
            max_particles: 2000,
            ..KldConfig::default()
        };
        let mut last = 0;
        for k in 1..400 {
            let m = cfg.required(k).unwrap();
            assert!(m >= last, "k={k}");
            assert!((50..=2000).contains(&m));
            last = m;
        }
        assert_eq!(cfg.required(1).unwrap(), 50);
        assert_eq!(cfg.required(1000).unwrap(), 2000);
    }

    #[test]
    fn invalid_configuration_is_rejected() {
        let d = KldConfig::default();
        for bad in [
            KldConfig {
                min_particles: 0,
                ..d
            },
            KldConfig {
                max_particles: 10,
                min_particles: 20,
                ..d
            },
            KldConfig { epsilon: 0.0, ..d },
            KldConfig {
                epsilon: f64::NAN,
                ..d
            },
            KldConfig { delta: 0.0, ..d },
            KldConfig { delta: 1.0, ..d },
        ] {
            assert!(bad.validated().is_err(), "{bad:?}");
        }
        assert!(GridBinning::new(Vector1::new(0.0)).is_err());
        assert!(Se2Binning::new(0.5, 0.0).is_err());
    }

    /// 500 samples centred in the middle of the bin `[0, 0.1)`.
    fn cloud(spread: f64) -> ParticleSet<Vector1<f64>> {
        let mut rng = seeded_rng(1);
        let n = Normal::new(0.05, spread).unwrap();
        ParticleSet::from_states((0..500).map(|_| Vector1::new(n.sample(&mut rng))).collect())
            .unwrap()
    }

    fn run(
        spread: f64,
        noise: f64,
        seed: u64,
        cfg: &KldConfig,
    ) -> (ParticleSet<Vector1<f64>>, KldReport) {
        let binning = GridBinning::new(Vector1::new(0.1)).unwrap();
        let jitter = Normal::new(0.0, noise).unwrap();
        kld_resample(
            &cloud(spread),
            cfg,
            &binning,
            |x: &Vector1<f64>, rng: &mut rand::rngs::SmallRng| {
                Vector1::new(x[0] + jitter.sample(rng))
            },
            &mut seeded_rng(seed),
        )
        .unwrap()
    }

    #[test]
    fn sample_count_follows_the_spread_of_the_belief() {
        let cfg = KldConfig {
            min_particles: 20,
            max_particles: 10_000,
            ..KldConfig::default()
        };
        // Concentrated in one bin → minimum.
        let (tight, r) = run(1e-4, 0.0, 3, &cfg);
        assert_eq!(r.occupied_bins, 1);
        assert_eq!(tight.len(), 20);
        // Wider beliefs occupy more bins and need more particles.
        let (_, r1) = run(0.2, 0.01, 3, &cfg);
        let (_, r2) = run(1.0, 0.01, 3, &cfg);
        let (_, r3) = run(5.0, 0.01, 3, &cfg);
        assert!(r1.occupied_bins < r2.occupied_bins && r2.occupied_bins < r3.occupied_bins);
        assert!(r1.particles < r2.particles && r2.particles < r3.particles);
        for r in [r1, r2, r3] {
            // Stopped exactly when the bound was met.
            assert!(r.particles >= r.bound && !r.hit_max, "{r:?}");
        }
    }

    #[test]
    fn maximum_is_enforced() {
        let cfg = KldConfig {
            min_particles: 10,
            max_particles: 300,
            ..KldConfig::default()
        };
        // Every proposal lands in a fresh bin: the bound outruns M, so M_max stops it.
        let mut counter = 0u64;
        let set = ParticleSet::from_states(vec![0u64; 5]).unwrap();
        let (out, r) = kld_resample(
            &set,
            &cfg,
            &BinFn(|x: &u64| *x),
            |_: &u64, _: &mut rand::rngs::SmallRng| {
                counter += 1;
                counter
            },
            &mut seeded_rng(0),
        )
        .unwrap();
        assert_eq!(out.len(), 300);
        assert!(r.hit_max);
        assert_eq!(r.occupied_bins, 300);
    }

    #[test]
    fn deterministic_under_a_fixed_seed() {
        let cfg = KldConfig::default();
        let (a, ra) = run(1.0, 0.05, 42, &cfg);
        let (b, rb) = run(1.0, 0.05, 42, &cfg);
        assert_eq!(ra, rb);
        assert_eq!(a, b);
        let (c, _) = run(1.0, 0.05, 43, &cfg);
        assert_ne!(a, c, "a different seed draws a different set");
    }

    #[test]
    fn draws_follow_the_weights() {
        // One particle carries all the weight: every offspring descends from it.
        let mut set = ParticleSet::from_states(vec![1u8, 2, 3]).unwrap();
        set.reweight(|x| if *x == 2 { 0.0 } else { f64::NEG_INFINITY })
            .unwrap();
        let (out, _) = kld_resample(
            &set,
            &KldConfig {
                min_particles: 50,
                ..KldConfig::default()
            },
            &BinFn(|x: &u8| *x),
            |x: &u8, _: &mut rand::rngs::SmallRng| *x,
            &mut seeded_rng(1),
        )
        .unwrap();
        assert!(out.states().all(|x| *x == 2));
        let mut dead = set.clone();
        dead.reweight(|_| f64::NEG_INFINITY).unwrap();
        assert_eq!(
            kld_resample(
                &dead,
                &KldConfig::default(),
                &BinFn(|x: &u8| *x),
                |x: &u8, _: &mut rand::rngs::SmallRng| *x,
                &mut seeded_rng(1)
            )
            .unwrap_err(),
            FilterError::DegenerateWeights
        );
    }

    #[test]
    fn se2_bins_wrap_heading() {
        let b = Se2Binning::new(0.5, 10f64.to_radians()).unwrap();
        assert_eq!(
            b.bin(&Se2::new(0.2, 0.2, PI)),
            b.bin(&Se2::new(0.2, 0.2, -PI + 1e-9))
        );
        assert_ne!(
            b.bin(&Se2::new(0.2, 0.2, 0.0)),
            b.bin(&Se2::new(0.2, 0.2, 0.2))
        );
        assert_ne!(
            b.bin(&Se2::new(0.2, 0.2, 0.0)),
            b.bin(&Se2::new(0.7, 0.2, 0.0))
        );
        assert_eq!(b.bin(&Se2::new(-0.2, 0.0, 0.0)).0, -1);
        let mut rng = seeded_rng(2);
        let poses: Vec<Se2> = (0..100)
            .map(|_| Se2::new(rng.random::<f64>() * 0.4, rng.random::<f64>() * 0.4, 0.01))
            .collect();
        assert_eq!(occupied_bins(&poses, &b), 1);
    }
}
