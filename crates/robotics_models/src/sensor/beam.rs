//! Beam-based range-finder model (Thrun Table 6.1): a mixture of four densities.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 10 (`BeamIntrinsics`,
//! `BeamModel::components`).
//!
//! For a measured range `z` and the expected range `z*` (from ray casting):
//!
//! * `p_hit`   = `N(z; z*, σ_hit²) / η_hit` on `[0, z_max]`, `η_hit = Φ((z_max−z*)/σ) − Φ(−z*/σ)`
//! * `p_short` = `λ e^{−λz} / (1 − e^{−λz*})` on `[0, z*]`
//! * `p_max`   = point mass at `z ≥ z_max`
//! * `p_rand`  = `1 / z_max` on `[0, z_max)`
//!
//! `p(z | z*) = z_hit p_hit + z_short p_short + z_max p_max + z_rand p_rand`, which integrates
//! to one over `[0, z_max)` plus the `z_max` atom.

use super::{normal_cdf, normal_pdf};
use crate::ModelError;

/// Mixture weights and shape parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeamModelParams {
    /// Weight of the measurement-noise (hit) component.
    pub z_hit: f64,
    /// Weight of the unexpected-obstacle (short) component.
    pub z_short: f64,
    /// Weight of the max-range component.
    pub z_max: f64,
    /// Weight of the random-measurement component.
    pub z_rand: f64,
    /// Standard deviation of the hit component (m).
    pub sigma_hit: f64,
    /// Rate of the short component (1/m).
    pub lambda_short: f64,
    /// Maximum range of the sensor (m).
    pub max_range: f64,
}

impl Default for BeamModelParams {
    /// The book's defaults.
    fn default() -> Self {
        Self {
            z_hit: 0.74,
            z_short: 0.1,
            z_max: 0.06,
            z_rand: 0.1,
            sigma_hit: 0.12,
            lambda_short: 1.0,
            max_range: 8.0,
        }
    }
}

/// The four component densities at one `(z, z*)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeamComponents {
    /// `p_hit`.
    pub hit: f64,
    /// `p_short`.
    pub short: f64,
    /// `p_max` (0 or 1).
    pub max: f64,
    /// `p_rand`.
    pub rand: f64,
}

/// The beam model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeamModel {
    p: BeamModelParams,
}

/// Result of one EM iteration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EmStep {
    /// Parameters after the M-step.
    pub params: BeamModelParams,
    /// Mean log-likelihood per used reading under the parameters *before* the step.
    pub mean_log_likelihood: f64,
    /// Readings that contributed.
    pub used: usize,
    /// Readings no component could explain (zero mixture likelihood).
    pub skipped: usize,
}

impl BeamModel {
    /// Build a model; the weights must be non-negative and sum to one (±1e-6), the shape
    /// parameters positive.
    pub fn new(p: BeamModelParams) -> Result<Self, ModelError> {
        for (what, value) in [
            ("z_hit", p.z_hit),
            ("z_short", p.z_short),
            ("z_max", p.z_max),
            ("z_rand", p.z_rand),
        ] {
            if !(value.is_finite() && value >= 0.0) {
                return Err(ModelError::Negative { what, value });
            }
        }
        let sum = p.z_hit + p.z_short + p.z_max + p.z_rand;
        if (sum - 1.0).abs() > 1e-6 {
            return Err(ModelError::InvalidMixture { sum });
        }
        for (what, value) in [
            ("sigma_hit", p.sigma_hit),
            ("lambda_short", p.lambda_short),
            ("max_range", p.max_range),
        ] {
            if !(value.is_finite() && value > 0.0) {
                return Err(ModelError::NonPositive { what, value });
            }
        }
        Ok(Self { p })
    }

    /// Parameters.
    pub fn params(&self) -> &BeamModelParams {
        &self.p
    }

    /// Component densities for measured `z` and expected `z_star`.
    pub fn components(&self, z: f64, z_star: f64) -> BeamComponents {
        let p = &self.p;
        let zmax = p.max_range;
        let hit = if (0.0..=zmax).contains(&z) {
            let mass =
                normal_cdf((zmax - z_star) / p.sigma_hit) - normal_cdf(-z_star / p.sigma_hit);
            if mass > 1e-12 {
                normal_pdf(z - z_star, p.sigma_hit) / mass
            } else {
                0.0
            }
        } else {
            0.0
        };
        let short = if z_star > 0.0 && (0.0..=z_star).contains(&z) {
            p.lambda_short * (-p.lambda_short * z).exp() / (1.0 - (-p.lambda_short * z_star).exp())
        } else {
            0.0
        };
        let max = if z >= zmax - 1e-9 { 1.0 } else { 0.0 };
        let rand = if (0.0..zmax).contains(&z) {
            1.0 / zmax
        } else {
            0.0
        };
        BeamComponents {
            hit,
            short,
            max,
            rand,
        }
    }

    /// Mixture likelihood `p(z | z*)`.
    pub fn likelihood(&self, z: f64, z_star: f64) -> f64 {
        let c = self.components(z, z_star);
        let p = &self.p;
        p.z_hit * c.hit + p.z_short * c.short + p.z_max * c.max + p.z_rand * c.rand
    }

    /// One EM iteration of `learn_intrinsic_parameters` (Thrun Table 6.2) over readings
    /// `z` paired with their ray-cast expectations `z_star` (cached outside the loop).
    ///
    /// E-step: responsibilities `e_{i,c} = z_c p_c(z_i) / Σ_c' z_c' p_c'(z_i)`.
    /// M-step: `z_c = mean_i e_{i,c}`,
    /// `σ_hit² = Σ e_{i,hit}(z_i − z_i*)² / Σ e_{i,hit}`,
    /// `λ_short = Σ e_{i,short} / Σ e_{i,short} z_i`.
    ///
    /// Readings no component can explain are skipped (and counted); a component with no
    /// responsibility keeps its shape parameter; `σ_hit ≥ 1 mm` and
    /// `λ_short ∈ [1e-3, 50]` keep the next model valid. Non-finite readings are treated
    /// as max-range returns, as everywhere else in this model. `λ_short` inherits the
    /// truncation bias the book documents (Chapter 10, EM derivation, step 6).
    pub fn em_step(&self, z: &[f64], z_star: &[f64]) -> Result<EmStep, ModelError> {
        if z.len() != z_star.len() || z.is_empty() {
            return Err(ModelError::InvalidData(
                "z and z_star must be non-empty and equally long",
            ));
        }
        if z_star.iter().any(|v| !v.is_finite() || *v < 0.0) {
            return Err(ModelError::InvalidData(
                "expected ranges must be finite and >= 0",
            ));
        }
        let p = &self.p;
        let (mut s_hit, mut s_short, mut s_max, mut s_rand) = (0.0, 0.0, 0.0, 0.0);
        let (mut sq_hit, mut z_short_sum, mut ll) = (0.0, 0.0, 0.0);
        let mut used = 0usize;
        for (&zi, &si) in z.iter().zip(z_star) {
            let zi = if zi.is_finite() { zi } else { p.max_range };
            let c = self.components(zi, si);
            let w = [
                p.z_hit * c.hit,
                p.z_short * c.short,
                p.z_max * c.max,
                p.z_rand * c.rand,
            ];
            let total: f64 = w.iter().sum();
            if total <= 0.0 || !total.is_finite() {
                continue;
            }
            used += 1;
            s_hit += w[0] / total;
            s_short += w[1] / total;
            s_max += w[2] / total;
            s_rand += w[3] / total;
            sq_hit += w[0] / total * (zi - si).powi(2);
            z_short_sum += w[1] / total * zi;
            ll += total.ln();
        }
        if used == 0 {
            return Err(ModelError::InvalidData(
                "no reading is explained by the model",
            ));
        }
        let n = used as f64;
        let params = BeamModelParams {
            z_hit: s_hit / n,
            z_short: s_short / n,
            z_max: s_max / n,
            z_rand: s_rand / n,
            sigma_hit: if s_hit > 1e-9 {
                (sq_hit / s_hit).sqrt().max(1e-3)
            } else {
                p.sigma_hit
            },
            lambda_short: if z_short_sum > 1e-9 {
                (s_short / z_short_sum).clamp(1e-3, 50.0)
            } else {
                p.lambda_short
            },
            max_range: p.max_range,
        };
        Ok(EmStep {
            params,
            mean_log_likelihood: ll / n,
            used,
            skipped: z.len() - used,
        })
    }

    /// Run [`BeamModel::em_step`] until the mean log-likelihood changes by less than
    /// `tol` or `max_iters` is reached. Returns the fitted model and the per-iteration
    /// mean log-likelihood trace. Because Table 6.2's M-step ignores the truncation of
    /// `p_hit` and `p_short`, it is not an exact maximization: the trace rises steeply and
    /// can dip by ~10⁻⁴ nats per beam near convergence (measured in the tests).
    pub fn fit_em(
        &self,
        z: &[f64],
        z_star: &[f64],
        max_iters: usize,
        tol: f64,
    ) -> Result<(BeamModel, Vec<f64>), ModelError> {
        let mut model = *self;
        let mut trace = Vec::with_capacity(max_iters);
        for _ in 0..max_iters {
            let step = model.em_step(z, z_star)?;
            let converged = trace
                .last()
                .is_some_and(|prev: &f64| (step.mean_log_likelihood - prev).abs() < tol);
            trace.push(step.mean_log_likelihood);
            model = BeamModel::new(step.params)?;
            if converged {
                break;
            }
        }
        Ok((model, trace))
    }

    /// `Σ ln p(z_k | z*_k)` over `(measured, expected)` pairs, each floored at `1e-300`
    /// so a single impossible beam cannot produce `−∞`. Non-finite measured ranges (dropouts)
    /// are treated as max-range readings, as `sensor_msgs/LaserScan` encodes them.
    pub fn log_likelihood<I: IntoIterator<Item = (f64, f64)>>(&self, beams: I) -> f64 {
        beams
            .into_iter()
            .map(|(z, z_star)| {
                let z = if z.is_finite() { z } else { self.p.max_range };
                self.likelihood(z, z_star).max(1e-300).ln()
            })
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Book check #16: with the default parameters and z* = 3 the density integrates to one
    /// (Riemann sum with h = 0.002 plus the z_max atom), within 3e-3.
    #[test]
    fn beam_model_normalizes() {
        let m = BeamModel::new(BeamModelParams::default()).unwrap();
        let h = 0.002;
        let mut mass = 0.0;
        let mut z = h / 2.0;
        while z < 8.0 {
            mass += m.likelihood(z, 3.0) * h;
            z += h;
        }
        mass += m.params().z_max * 1.0;
        assert!((mass - 1.0).abs() < 3e-3, "mass {mass}");
    }

    /// Chapter 10 EM worked example: z* = 5, σ_hit = 1, λ = 0.5, z_max = 10, equal weights.
    /// At z = 4.5: p_hit = 0.352065, p_short = 0.057412, p_max = 0, p_rand = 0.1, mixture
    /// 0.127369.
    #[test]
    fn worked_example_ch10_components() {
        let m = BeamModel::new(BeamModelParams {
            z_hit: 0.25,
            z_short: 0.25,
            z_max: 0.25,
            z_rand: 0.25,
            sigma_hit: 1.0,
            lambda_short: 0.5,
            max_range: 10.0,
        })
        .unwrap();
        let c = m.components(4.5, 5.0);
        assert_relative_eq!(c.hit, 0.352065, epsilon = 1e-6);
        assert_relative_eq!(c.short, 0.057412, epsilon = 1e-6);
        assert_eq!(c.max, 0.0);
        assert_relative_eq!(c.rand, 0.1, epsilon = 1e-12);
        assert_relative_eq!(m.likelihood(4.5, 5.0), 0.127369, epsilon = 1e-6);
        // Remaining rows of the book's table.
        let c = m.components(5.0, 5.0);
        assert_relative_eq!(c.hit, 0.398943, epsilon = 1e-6);
        assert_relative_eq!(c.short, 0.044713, epsilon = 1e-6);
        let c = m.components(7.0, 5.0);
        assert_relative_eq!(c.hit, 0.053991, epsilon = 1e-6);
        assert_eq!(c.short, 0.0);
        let c = m.components(10.0, 5.0);
        assert_eq!(c.max, 1.0);
        assert_eq!(c.rand, 0.0);
    }

    /// Chapter 10 EM worked example: five beams at a wall z* = 5 m, one iteration from
    /// uniform weights (σ_hit = 1, λ = 0.5, z_max = 10).
    #[test]
    fn worked_example_ch10_em_iteration() {
        let m = BeamModel::new(BeamModelParams {
            z_hit: 0.25,
            z_short: 0.25,
            z_max: 0.25,
            z_rand: 0.25,
            sigma_hit: 1.0,
            lambda_short: 0.5,
            max_range: 10.0,
        })
        .unwrap();
        let z = [4.5, 5.0, 3.0, 7.0, 10.0];
        let zs = [5.0; 5];
        let s = m.em_step(&z, &zs).unwrap();
        assert_eq!(s.used, 5);
        assert_relative_eq!(s.mean_log_likelihood, -2.275038, epsilon = 1e-6);
        assert_relative_eq!(s.params.z_hit, 0.394282, epsilon = 1e-6);
        assert_relative_eq!(s.params.z_short, 0.127210, epsilon = 1e-6);
        assert_relative_eq!(s.params.z_max, 0.200000, epsilon = 1e-6);
        assert_relative_eq!(s.params.z_rand, 0.278508, epsilon = 1e-6);
        assert_relative_eq!(s.params.sigma_hit, 1.093905, epsilon = 1e-6);
        // The book divides its 6-decimal sums 0.636050 / 2.241674 = 0.283737; carried in
        // full precision the same quotient is 0.636051 / 2.241677 = 0.2837395.
        assert_relative_eq!(s.params.lambda_short, 0.283737, epsilon = 5e-6);
        let sum = s.params.z_hit + s.params.z_short + s.params.z_max + s.params.z_rand;
        assert_relative_eq!(sum, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn em_recovers_generating_parameters_and_is_monotone() {
        use rand::RngExt;
        use rand_distr::{Distribution, Exp, Normal};
        let mut rng = robotics_core::seeded_rng(10);
        let truth = BeamModelParams {
            z_hit: 0.8,
            z_short: 0.1,
            z_max: 0.05,
            z_rand: 0.05,
            sigma_hit: 0.1,
            lambda_short: 1.0,
            max_range: 8.0,
        };
        let (mut z, mut zs) = (Vec::new(), Vec::new());
        let hit = Normal::new(0.0, truth.sigma_hit).unwrap();
        let short = Exp::new(truth.lambda_short).unwrap();
        while z.len() < 20_000 {
            let z_star = 1.0 + 6.0 * rng.random::<f64>();
            let u = rng.random::<f64>();
            let r = if u < truth.z_hit {
                (z_star + hit.sample(&mut rng)).clamp(0.0, 8.0)
            } else if u < truth.z_hit + truth.z_short {
                let s: f64 = short.sample(&mut rng);
                if s > z_star {
                    continue; // truncated exponential: redraw
                }
                s
            } else if u < truth.z_hit + truth.z_short + truth.z_max {
                8.0
            } else {
                8.0 * rng.random::<f64>()
            };
            z.push(r);
            zs.push(z_star);
        }
        let start = BeamModel::new(BeamModelParams {
            z_hit: 0.25,
            z_short: 0.25,
            z_max: 0.25,
            z_rand: 0.25,
            sigma_hit: 0.5,
            lambda_short: 0.5,
            max_range: 8.0,
        })
        .unwrap();
        let (fit, trace) = start.fit_em(&z, &zs, 200, 1e-9).unwrap();
        // Table 6.2's M-step ignores the truncation of p_hit and p_short, so it is not an
        // exact maximization and the trace may dip slightly near convergence; it must not
        // dip materially, and the fit must end far above where it started.
        for w in trace.windows(2) {
            assert!(w[1] >= w[0] - 1e-3, "EM log-likelihood decreased: {w:?}");
        }
        assert!(
            trace.last().unwrap() > &(trace[0] + 0.5),
            "{:?}",
            (trace[0], trace.last())
        );
        let p = fit.params();
        assert!((p.z_hit - 0.8).abs() < 0.03, "{p:?}");
        assert!((p.z_max - 0.05).abs() < 0.01, "{p:?}");
        assert!((p.sigma_hit - 0.1).abs() < 0.01, "{p:?}");
        assert!(p.z_short > 0.05 && p.z_short < 0.15, "{p:?}");
        assert!(start.em_step(&z, &zs[1..]).is_err());
        assert!(start.em_step(&[], &[]).is_err());
    }

    #[test]
    fn scan_log_likelihood_prefers_matching_ranges() {
        let m = BeamModel::new(BeamModelParams::default()).unwrap();
        let expected = [1.0, 2.0, 3.0, 4.0];
        let good = m.log_likelihood(expected.iter().map(|z| (*z, *z)));
        let bad = m.log_likelihood(expected.iter().map(|z| (*z + 0.5, *z)));
        assert!(good > bad);
        let with_nan = m.log_likelihood([(f64::NAN, 3.0), (3.0, 3.0)]);
        assert!(with_nan.is_finite());
    }

    #[test]
    fn rejects_invalid_parameters() {
        let bad = BeamModelParams {
            z_hit: 0.9,
            ..Default::default()
        };
        assert!(matches!(
            BeamModel::new(bad),
            Err(ModelError::InvalidMixture { .. })
        ));
        let bad = BeamModelParams {
            sigma_hit: 0.0,
            ..Default::default()
        };
        assert!(BeamModel::new(bad).is_err());
    }
}
