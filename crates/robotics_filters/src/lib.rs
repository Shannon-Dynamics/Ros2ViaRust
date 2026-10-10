//! `robotics_filters` — recursive state estimation, independent of ROS.
//!
//! * [`BayesFilter`]: the trait every estimator in the framework implements
//!   (*Probabilistic Robotics via Rust*, Chapter 5 — The Bayes Filter).
//! * [`HistogramFilter1D`]: the discrete Bayes filter on a 1-D grid (Chapter 5).
//! * [`KalmanFilter`]: the linear-Gaussian filter in Joseph form (Chapter 6).
//! * [`InformationFilter`]: the same filter in canonical form `(ξ, Ω)` (Chapter 6).
//! * [`RtsSmoother`]: Rauch–Tung–Striebel fixed-interval smoothing (Chapter 6).
//! * [`consistency`]: NEES / NIS with χ² acceptance bounds (Chapter 6).
//! * [`Ekf`] / [`Ukf`]: generic over the state manifold (Chapter 7).
//! * [`EskfSe2`]: error-state filter on SE(2) with speed and gyro bias (Chapter 7).
//! * [`OdomImuFusion`]: asynchronous wheel + gyro fusion around the ESKF.
//! * [`ParticleSet`]: log-weight particle machinery, low-variance and multinomial
//!   resampling, ESS (Chapter 8).
//! * [`kld`]: KLD-adaptive particle-set sizing with pluggable binning (Chapter 8).

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod bayes;
pub mod consistency;
pub mod ekf;
pub mod eskf_se2;
pub mod fusion;
pub mod histogram;
pub mod information;
pub mod kalman;
pub mod kld;
pub mod particle;
pub mod smoother;
pub mod ukf;
pub mod update;

pub use bayes::{BayesFilter, FilterError};
pub use consistency::{nees, nis, ConsistencyReport, ConsistencyStats};
pub use ekf::{Ekf, MeasurementJacobian, MeasurementModel, ProcessJacobian, ProcessModel};
pub use eskf_se2::{EskfSe2, PlanarState, ProcessNoise};
pub use fusion::{DegradedReason, FusedEstimate, FusionConfig, Health, OdomImuFusion, Outcome};
pub use histogram::HistogramFilter1D;
pub use information::InformationFilter;
pub use kalman::{KalmanFilter, KalmanUpdate};
pub use kld::{
    kld_resample, kld_sample_size, BinFn, GridBinning, KldConfig, KldReport, ParticleBinning,
    Se2Binning,
};
pub use particle::{
    effective_sample_size, low_variance_indices, multinomial_indices, CategoricalSampler, Particle,
    ParticleSet,
};
pub use smoother::{rts_smooth, ForwardStep, RtsSmoother};
pub use ukf::{sigma_points, SigmaPoints, Ukf, UtParams};
pub use update::{GaussianCorrection, Tolerances};
