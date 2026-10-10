//! The Bayes filter interface.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 5:
//! `trait BayesFilter { fn predict(&mut self, u); fn correct(&mut self, z) -> evidence; }`.

use robotics_core::CoreError;

/// Errors an estimator can report instead of panicking.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum FilterError {
    /// The innovation covariance (or another matrix that must be inverted) was singular.
    #[error("singular innovation covariance")]
    SingularInnovation,
    /// A measurement or control contained NaN/inf.
    #[error("non-finite input in {0}")]
    NonFinite(&'static str),
    /// Every hypothesis received zero likelihood; the belief could not be normalized.
    #[error("zero total likelihood: belief cannot be normalized")]
    ZeroLikelihood,
    /// A covariance matrix was not finite, not symmetric, or not positive (semi-)definite.
    #[error("invalid covariance: {0}")]
    InvalidCovariance(&'static str),
    /// A time step was negative, NaN, or larger than the configured maximum.
    #[error("invalid time step {dt} s")]
    InvalidTimeDelta {
        /// The rejected time step (s).
        dt: f64,
    },
    /// A measurement arrived with a timestamp older than the filter state by more than the
    /// accepted delay.
    #[error("out-of-order measurement: {age} s older than the filter state")]
    OutOfOrderMeasurement {
        /// How far behind the filter time the measurement is (s).
        age: f64,
    },
    /// A measurement carried a zero or otherwise unusable timestamp.
    #[error("invalid timestamp")]
    InvalidTimestamp,
    /// All particle weights are zero, NaN or −∞; the set cannot be normalized.
    #[error("degenerate particle weights")]
    DegenerateWeights,
    /// A noise or tuning parameter was negative, zero where it must be positive, or non-finite.
    #[error("invalid noise parameter {what} = {value}")]
    InvalidNoiseParameter {
        /// Parameter name.
        what: &'static str,
        /// Offending value.
        value: f64,
    },
    /// The filter has not been initialized yet.
    #[error("filter not initialized")]
    NotInitialized,
    /// An error from the core numerical layer.
    #[error(transparent)]
    Core(#[from] CoreError),
}

impl FilterError {
    /// Validate that a noise/tuning parameter is finite and `>= 0`.
    pub fn check_non_negative(what: &'static str, value: f64) -> Result<f64, FilterError> {
        if value.is_finite() && value >= 0.0 {
            Ok(value)
        } else {
            Err(FilterError::InvalidNoiseParameter { what, value })
        }
    }

    /// Validate that a noise/tuning parameter is finite and `> 0`.
    pub fn check_positive(what: &'static str, value: f64) -> Result<f64, FilterError> {
        if value.is_finite() && value > 0.0 {
            Ok(value)
        } else {
            Err(FilterError::InvalidNoiseParameter { what, value })
        }
    }
}

/// A recursive Bayesian estimator: alternate `predict` (motion) and `correct` (sensing).
///
/// `correct` returns the **evidence** `p(z_t | z_{1:t−1}, u_{1:t})`, the normalizer of the
/// Bayes update, which diagnostics use to detect surprising measurements.
pub trait BayesFilter {
    /// The belief representation (a grid, a Gaussian, a particle set, …).
    type Belief;
    /// The control input type.
    type Control;
    /// The measurement type.
    type Measurement;

    /// Incorporate a control: `bel̄(x_t) = ∫ p(x_t | u_t, x_{t−1}) bel(x_{t−1}) dx_{t−1}`.
    fn predict(&mut self, u: &Self::Control) -> Result<(), FilterError>;

    /// Incorporate a measurement: `bel(x_t) = η p(z_t | x_t) bel̄(x_t)`; returns `1/η`.
    fn correct(&mut self, z: &Self::Measurement) -> Result<f64, FilterError>;

    /// The current belief.
    fn belief(&self) -> &Self::Belief;
}
