//! Planar gyro (yaw-rate) noise model.
//!
//! Measurement: `ω_m = ω + b + n`, `n ~ N(0, σ_d² · f_s)` per sample for a white-noise
//! density `σ_d` (rad/s/√Hz) sampled at `f_s` Hz, and a bias `b` following a random walk
//! `ḃ ~ N(0, σ_bw²)` (rad/s/√s). This is the planar slice of the standard IMU model; a full
//! 3-D inertial model is deliberately out of scope for Phase 2.

use crate::ModelError;

/// Gyro noise parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GyroNoise {
    /// White-noise density `σ_d` (rad/s/√Hz).
    pub noise_density: f64,
    /// Bias random-walk density `σ_bw` (rad/s/√s).
    pub bias_random_walk: f64,
}

impl Default for GyroNoise {
    /// Consumer-grade MEMS values: 0.005 rad/s/√Hz (≈ 0.29 °/s/√Hz), 1e-4 rad/s/√s.
    fn default() -> Self {
        Self {
            noise_density: 0.005,
            bias_random_walk: 1e-4,
        }
    }
}

impl GyroNoise {
    /// Validate (finite, non-negative).
    pub fn validated(self) -> Result<Self, ModelError> {
        for (what, value) in [
            ("noise_density", self.noise_density),
            ("bias_random_walk", self.bias_random_walk),
        ] {
            if !(value.is_finite() && value >= 0.0) {
                return Err(ModelError::Negative { what, value });
            }
        }
        Ok(self)
    }

    /// Per-sample standard deviation for a sample rate `rate_hz`: `σ_d · √f_s`.
    pub fn sample_sigma(&self, rate_hz: f64) -> f64 {
        self.noise_density * rate_hz.max(0.0).sqrt()
    }

    /// Standard deviation of the bias increment over `dt` seconds: `σ_bw · √dt`.
    pub fn bias_step_sigma(&self, dt: f64) -> f64 {
        self.bias_random_walk * dt.max(0.0).sqrt()
    }

    /// `ln p(ω_m | ω, b)` for one sample at `rate_hz`.
    pub fn log_likelihood(&self, measured: f64, true_rate: f64, bias: f64, rate_hz: f64) -> f64 {
        let s = self.sample_sigma(rate_hz).max(1e-12);
        let e = measured - true_rate - bias;
        -0.5 * (e * e / (s * s)) - s.ln() - 0.5 * (2.0 * std::f64::consts::PI).ln()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sigma_scaling() {
        let g = GyroNoise::default();
        assert!((g.sample_sigma(100.0) - 0.05).abs() < 1e-12);
        assert!((g.bias_step_sigma(4.0) - 2e-4).abs() < 1e-15);
        assert!(g.log_likelihood(0.1, 0.1, 0.0, 100.0) > g.log_likelihood(0.2, 0.1, 0.0, 100.0));
        assert!(GyroNoise {
            noise_density: -1.0,
            bias_random_walk: 0.0
        }
        .validated()
        .is_err());
    }
}
