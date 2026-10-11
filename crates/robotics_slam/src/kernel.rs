//! Robust kernels for the factor-graph optimizer.
//!
//! A factor with whitened residual `r` has `e = ‖r‖` and costs `ρ(e)` instead of `½e²`. The
//! optimizer minimizes the sum of those costs by iteratively reweighted least squares: a
//! factor's contribution to the normal equations (`H` and `b`, both) is multiplied by
//! `w(e) = ρ'(e)/e`, the weight itself and not its square root (decision DS9; the two
//! conventions give different normal equations and the reference sources use both).
//!
//! * [`RobustKernel::Huber`] is convex, so it cannot lock a bad start onto a wrong optimum, and
//!   it is the default of the loop edges. It limits the pull of an outlier; it does not reject
//!   a gross one.
//! * [`RobustKernel::Cauchy`] and [`RobustKernel::GemanMcClure`] redescend: a large residual is
//!   nearly ignored. They are non-convex and need a reasonable starting point.

use crate::SlamError;

/// The shape of the cost a factor contributes as a function of its whitened residual norm.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum RobustKernel {
    /// Plain least squares, `ρ(e) = ½e²`, weight 1.
    #[default]
    L2,
    /// `ρ(e) = ½e²` for `e ≤ k`, `k(e − k/2)` beyond; weight `min(1, k/e)`.
    Huber {
        /// Threshold `k`, in standard deviations.
        k: f64,
    },
    /// `ρ(e) = (c²/2) ln(1 + e²/c²)`; weight `1/(1 + e²/c²)`.
    Cauchy {
        /// Scale `c`, in standard deviations.
        c: f64,
    },
    /// `ρ(e) = c²e² / (2(c² + e²))`; weight `c⁴/(c² + e²)²`.
    GemanMcClure {
        /// Scale `c`, in standard deviations.
        c: f64,
    },
}

impl RobustKernel {
    /// Check the scale parameter.
    pub fn validate(&self) -> Result<(), SlamError> {
        let s = match *self {
            RobustKernel::L2 => return Ok(()),
            RobustKernel::Huber { k } => k,
            RobustKernel::Cauchy { c } | RobustKernel::GemanMcClure { c } => c,
        };
        if s.is_finite() && s > 0.0 {
            Ok(())
        } else {
            Err(SlamError::InvalidParameter {
                what: "kernel scale",
                value: s,
            })
        }
    }

    /// The IRLS weight `ρ'(e)/e` at residual norm `e ≥ 0`.
    pub fn weight(&self, e: f64) -> f64 {
        match *self {
            RobustKernel::L2 => 1.0,
            RobustKernel::Huber { k } => {
                if e <= k {
                    1.0
                } else {
                    k / e
                }
            }
            RobustKernel::Cauchy { c } => 1.0 / (1.0 + e * e / (c * c)),
            RobustKernel::GemanMcClure { c } => {
                let c2 = c * c;
                let d = c2 + e * e;
                c2 * c2 / (d * d)
            }
        }
    }

    /// The cost `ρ(e)` at residual norm `e ≥ 0`.
    pub fn cost(&self, e: f64) -> f64 {
        match *self {
            RobustKernel::L2 => 0.5 * e * e,
            RobustKernel::Huber { k } => {
                if e <= k {
                    0.5 * e * e
                } else {
                    k * (e - 0.5 * k)
                }
            }
            RobustKernel::Cauchy { c } => 0.5 * c * c * (1.0 + e * e / (c * c)).ln(),
            RobustKernel::GemanMcClure { c } => {
                let c2 = c * c;
                0.5 * c2 * e * e / (c2 + e * e)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const KERNELS: [RobustKernel; 4] = [
        RobustKernel::L2,
        RobustKernel::Huber { k: 1.3 },
        RobustKernel::Cauchy { c: 0.9 },
        RobustKernel::GemanMcClure { c: 1.1 },
    ];

    #[test]
    fn weight_is_the_derivative_of_the_cost_over_e() {
        for kernel in KERNELS {
            for e in [0.05, 0.4, 1.0, 1.29, 1.31, 2.5, 7.0, 40.0] {
                let h = 1e-6;
                let d = (kernel.cost(e + h) - kernel.cost(e - h)) / (2.0 * h);
                assert_relative_eq!(kernel.weight(e) * e, d, epsilon = 1e-6, max_relative = 1e-6);
            }
        }
    }

    #[test]
    fn every_kernel_is_least_squares_near_zero() {
        for kernel in KERNELS {
            assert_relative_eq!(kernel.weight(1e-9), 1.0, epsilon = 1e-9);
            assert_relative_eq!(kernel.cost(1e-3), 0.5e-6, max_relative = 1e-3);
        }
    }

    #[test]
    fn huber_is_continuous_at_its_threshold_and_never_redescends() {
        let k = RobustKernel::Huber { k: 1.3 };
        assert_relative_eq!(k.cost(1.3 - 1e-12), k.cost(1.3 + 1e-12), epsilon = 1e-9);
        let (a, b) = (k.weight(2.0), k.weight(20.0));
        assert!(a > b && b > 0.0);
        // The influence ρ'(e) = w e stays bounded below by a constant: it never redescends.
        assert_relative_eq!(k.weight(20.0) * 20.0, 1.3, epsilon = 1e-12);
    }

    #[test]
    fn redescending_kernels_have_a_bounded_influence_that_falls() {
        for k in [
            RobustKernel::Cauchy { c: 1.0 },
            RobustKernel::GemanMcClure { c: 1.0 },
        ] {
            let infl = |e: f64| k.weight(e) * e;
            assert!(infl(30.0) < infl(3.0));
        }
    }

    #[test]
    fn weights_are_in_zero_one_and_decrease_with_the_residual() {
        for kernel in KERNELS {
            let mut last = 1.0 + 1e-12;
            for i in 0..60 {
                let w = kernel.weight(i as f64 * 0.5);
                assert!(w > 0.0 && w <= last, "{kernel:?} {i}");
                last = w;
            }
        }
    }

    #[test]
    fn invalid_scales_are_refused() {
        assert!(RobustKernel::Huber { k: 0.0 }.validate().is_err());
        assert!(RobustKernel::Cauchy { c: f64::NAN }.validate().is_err());
        assert!(RobustKernel::GemanMcClure { c: -1.0 }.validate().is_err());
        assert!(RobustKernel::L2.validate().is_ok());
    }
}
