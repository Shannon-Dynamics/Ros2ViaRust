//! Error type shared by the core crate and reused by algorithm crates.

/// Errors produced by core numerical routines.
///
/// Library code returns these instead of panicking so ROS nodes can log and drop a bad
/// measurement rather than crash the process.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CoreError {
    /// A covariance / information matrix was expected to be symmetric positive definite.
    #[error("matrix is not symmetric positive definite")]
    NotPositiveDefinite,
    /// Two containers that must agree in size did not.
    #[error("dimension mismatch: expected {expected}, got {got}")]
    DimensionMismatch {
        /// Size that was required.
        expected: usize,
        /// Size that was supplied.
        got: usize,
    },
    /// A scalar input was outside its valid domain.
    #[error("invalid value for {what}: {value}")]
    InvalidValue {
        /// Name of the offending quantity.
        what: &'static str,
        /// The value that was rejected.
        value: f64,
    },
    /// A NaN or infinity appeared where a finite number is required.
    #[error("non-finite number in {0}")]
    NonFinite(&'static str),
}
