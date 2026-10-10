//! End-to-end regression: simulator → wheel odometry + gyro → ESKF fusion → ground truth.

mod common;

use common::pipeline::run_pipeline;
use robotics_filters::FusionConfig;
use robotics_sim::RustyConfig;

#[test]
fn fusion_pipeline_is_deterministic_and_accurate() {
    let m = run_pipeline(RustyConfig::default(), FusionConfig::default(), 120.0);
    let again = run_pipeline(RustyConfig::default(), FusionConfig::default(), 120.0);
    eprintln!("{m:#?}");
    assert_eq!(m.raw_pos_rmse.to_bits(), again.raw_pos_rmse.to_bits());
    assert_eq!(m.fused_pos_rmse.to_bits(), again.fused_pos_rmse.to_bits());
    assert_eq!(m.rejected, 0);
    assert_eq!(m.samples, 6000);
    for v in [
        m.raw_pos_rmse,
        m.raw_yaw_rmse,
        m.fused_pos_rmse,
        m.fused_yaw_rmse,
        m.raw_nees,
        m.fused_nees,
        m.fused_nis2,
    ] {
        assert!(v.is_finite());
    }
    // Heading: the gyro observes yaw rate directly while the wheels carry a 0.2 % radius
    // error and 2 % slip, so the fused heading must beat wheel-only dead reckoning.
    assert!(m.fused_yaw_rmse < m.raw_yaw_rmse, "{m:?}");
    // Consistency: the fused pose covariance must not be over-confident (mean 3-DoF NEES
    // well below the χ² tail); the Phase 1 wheel covariance, which models neither the
    // radius error nor slip, is (documented in docs/STATE_ESTIMATION.md).
    assert!(m.fused_nees < 6.0, "{m:?}");
    assert!(m.raw_nees > m.fused_nees, "{m:?}");
    assert!(m.nis2_samples > 100 && m.fused_nis2 < 4.0, "{m:?}");
}

#[test]
fn noiseless_sensors_give_near_zero_error() {
    let m = run_pipeline(RustyConfig::noiseless(), FusionConfig::default(), 60.0);
    eprintln!("{m:#?}");
    assert!(m.raw_pos_rmse < 1e-3);
    // Heading is exact: gyro samples are integrated over the interval they describe.
    assert!(m.fused_yaw_rmse < 1e-6);
    // Position carries a known one-wheel-sample speed latency at each commanded speed
    // step (≈ Δv·20 ms ≈ 6 mm per step); see docs/STATE_ESTIMATION.md, limitations.
    assert!(m.fused_pos_rmse < 1e-2);
}
