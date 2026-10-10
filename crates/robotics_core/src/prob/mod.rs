//! Probability primitives: Gaussians in moments and canonical form, seeded RNGs, and the
//! normal / χ² quantiles used by consistency tests and KLD sampling.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 2 — Probability: The Language of
//! Uncertainty.

mod canonical;
mod gaussian;
mod rng;
pub mod stats;

pub use canonical::Canonical;
pub use gaussian::Gaussian;
pub use rng::{seeded_rng, BOOK_SEED};
pub use stats::{chi2_mean_interval, chi2_quantile, normal_quantile};
