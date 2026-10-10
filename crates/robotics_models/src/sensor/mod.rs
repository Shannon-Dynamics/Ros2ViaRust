//! Probabilistic sensor models `p(z | x, m)`.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 10 — Probabilistic Sensor Models
//! (beam model as a four-way mixture, likelihood field over an exact distance transform);
//! Thrun, Burgard & Fox, Tables 6.1 and 6.3.
//!
//! * [`gyro`]: planar gyro noise model shared by the simulator and the estimator config.
//! * [`beam`]: the hit/short/max/rand beam mixture and its EM parameter fit (Table 6.2).
//! * [`likelihood_field`]: exact Euclidean distance transform + likelihood-field model.
//! * [`landmark`]: range–bearing landmark model with known correspondence (Tables 6.4,
//!   6.5), including the SE(2) pose Jacobian.
//! * [`tempering`]: `p(z | x)^β` for any of the above, in log space.
//!
//! These models take expected ranges, distance fields or landmark positions as inputs.
//! Ray casting lives in `robotics_sim::World` (the simulator's ground truth, reusable as
//! the `z*` source); map-based localization (MCL) is Phase 3.

pub mod beam;
pub mod gyro;
pub mod landmark;
pub mod likelihood_field;
pub mod tempering;

/// Standard normal CDF `Φ(x)` via Abramowitz & Stegun 7.1.26 (|error| < 1.5·10⁻⁷).
pub fn normal_cdf(x: f64) -> f64 {
    0.5 * (1.0 + erf(x / std::f64::consts::SQRT_2))
}

/// Error function, Abramowitz & Stegun 7.1.26 (|error| < 1.5·10⁻⁷).
pub fn erf(x: f64) -> f64 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.327_591_1 * x);
    let y = 1.0
        - (((((1.061_405_429 * t - 1.453_152_027) * t) + 1.421_413_741) * t - 0.284_496_736) * t
            + 0.254_829_592)
            * t
            * (-x * x).exp();
    sign * y
}

/// Normal density `N(x; 0, σ²)`.
#[inline]
pub fn normal_pdf(x: f64, sigma: f64) -> f64 {
    (-0.5 * (x / sigma).powi(2)).exp() / (sigma * (2.0 * std::f64::consts::PI).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn erf_reference_values() {
        for (x, e) in [
            (0.0, 0.0),
            (0.5, 0.520_499_877_8),
            (1.0, 0.842_700_792_9),
            (2.0, 0.995_322_265_0),
            (-1.0, -0.842_700_792_9),
        ] {
            assert!((erf(x) - e).abs() < 2e-7, "erf({x})");
        }
        assert!((normal_cdf(0.0) - 0.5).abs() < 1e-9);
        assert!((normal_cdf(1.96) - 0.975_002_1).abs() < 2e-7);
    }
}
