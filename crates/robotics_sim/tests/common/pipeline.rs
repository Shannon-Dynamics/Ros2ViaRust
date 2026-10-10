//! Offline replica of the ROS Phase 2 pipeline, used by the integration test and the
//! `estimator_accuracy` example:
//!
//! ```text
//! RustySim ─ wheel angles ─► WheelOdometry (raw) ─ twist ─┐
//!          ─ gyro ───────────────────────────────────────┴► OdomImuFusion (fused)
//!          ─ ground truth ─► error metrics, NEES (raw and fused), NIS (fused)
//! ```
//!
//! It performs exactly the conversions the ROS nodes perform (wheel angles → twist from
//! the Phase 1 integrator, gyro → `ImuMeasurement`), without DDS in between.

#![allow(dead_code)]

use robotics_core::angle::angle_diff;
use robotics_core::Manifold;
use robotics_filters::consistency::{nees, ConsistencyStats};
use robotics_filters::{FusionConfig, OdomImuFusion};
use robotics_models::{OdometryNoise, WheelOdometry};
use robotics_sim::{RustyConfig, RustySim, Trajectory};

/// Error statistics against ground truth, sampled at every wheel sample.
#[derive(Debug, Clone, Copy, Default)]
pub struct Metrics {
    pub samples: usize,
    pub raw_pos_rmse: f64,
    pub raw_yaw_rmse: f64,
    pub raw_final_pos: f64,
    pub raw_final_yaw: f64,
    pub fused_pos_rmse: f64,
    pub fused_yaw_rmse: f64,
    pub fused_final_pos: f64,
    pub fused_final_yaw: f64,
    pub true_bias: f64,
    pub est_bias: f64,
    pub est_bias_sigma: f64,
    pub yaw_within_3sigma: f64,
    pub rejected: u64,
    /// Mean 3-DoF pose NEES of the raw wheel odometry (expected 3 if consistent).
    pub raw_nees: f64,
    /// Mean 3-DoF pose NEES of the fused estimate (expected 3 if consistent).
    pub fused_nees: f64,
    /// Mean NIS of the 2-DoF wheel+gyro corrections (expected 2).
    pub fused_nis2: f64,
    /// Number of 2-DoF corrections.
    pub nis2_samples: usize,
    /// Mean NIS of the 1-DoF speed-only corrections (expected 1).
    pub fused_nis1: f64,
    /// Number of 1-DoF corrections.
    pub nis1_samples: usize,
}

pub fn run_pipeline(cfg: RustyConfig, fusion_cfg: FusionConfig, seconds: f64) -> Metrics {
    let mut sim = RustySim::new(cfg.clone(), Trajectory::mixed(true)).expect("valid sim");
    let mut odom = WheelOdometry::new(cfg.params, OdometryNoise::default()).expect("valid odom");
    let mut fusion = OdomImuFusion::new(fusion_cfg).expect("valid fusion config");
    let w0 = sim.initial_wheel_sample();
    odom.update_from_wheel_angles(w0.left_angle, w0.right_angle, w0.stamp);

    let (mut rp, mut ry, mut fp, mut fy) = (0.0, 0.0, 0.0, 0.0);
    let mut n = 0usize;
    let mut within = 0usize;
    let mut m = Metrics::default();
    let mut raw_nees = ConsistencyStats::new(3).unwrap();
    let mut fused_nees = ConsistencyStats::new(3).unwrap();
    let mut nis1 = ConsistencyStats::new(1).unwrap();
    let mut nis2 = ConsistencyStats::new(2).unwrap();
    let steps = (seconds / cfg.physics_dt).round() as usize;
    for _ in 0..steps {
        let s = sim.step();
        if let Some(imu) = &s.imu {
            if fusion.on_imu(imu).is_err() {
                m.rejected += 1;
            }
        }
        if let Some(w) = s.wheel {
            let Some(step) = odom.update_from_wheel_angles(w.left_angle, w.right_angle, w.stamp)
            else {
                continue;
            };
            if fusion.on_wheel(w.stamp, &step.twist).is_err() {
                m.rejected += 1;
            } else if let Some((v, dof)) = fusion.last_nis() {
                let _ = if dof == 2 { nis2.push(v) } else { nis1.push(v) };
            }
            let Some(est) = fusion.estimate() else {
                continue;
            };
            let truth = s.truth.pose;
            let raw = odom.pose();
            let e_rp = raw.distance_to(&truth);
            let e_ry = angle_diff(raw.theta(), truth.theta());
            let e_fp = est.pose.distance_to(&truth);
            let e_fy = angle_diff(est.pose.theta(), truth.theta());
            rp += e_rp * e_rp;
            ry += e_ry * e_ry;
            fp += e_fp * e_fp;
            fy += e_fy * e_fy;
            if e_fy.abs() <= 3.0 * est.pose_cov[(2, 2)].sqrt() {
                within += 1;
            }
            // NEES in the tangent space at each estimate (right perturbation, body frame),
            // the space both covariances live in. The raw covariance starts at zero and
            // is skipped until positive definite.
            if let Ok(v) = nees(&truth.boxminus(&raw), &odom.covariance()) {
                let _ = raw_nees.push(v);
            }
            if let Some(f) = fusion.filter() {
                if let Ok(v) = nees(&truth.boxminus(&est.pose), &f.pose_covariance_body()) {
                    let _ = fused_nees.push(v);
                }
            }
            n += 1;
            m.raw_final_pos = e_rp;
            m.raw_final_yaw = e_ry;
            m.fused_final_pos = e_fp;
            m.fused_final_yaw = e_fy;
            m.true_bias = s.truth.gyro_bias;
            m.est_bias = est.gyro_bias;
            m.est_bias_sigma = est.gyro_bias_sigma;
        }
    }
    let nf = n.max(1) as f64;
    m.samples = n;
    m.raw_pos_rmse = (rp / nf).sqrt();
    m.raw_yaw_rmse = (ry / nf).sqrt();
    m.fused_pos_rmse = (fp / nf).sqrt();
    m.fused_yaw_rmse = (fy / nf).sqrt();
    m.yaw_within_3sigma = within as f64 / nf;
    m.raw_nees = raw_nees.mean();
    m.fused_nees = fused_nees.mean();
    m.fused_nis2 = nis2.mean();
    m.nis2_samples = nis2.samples();
    m.fused_nis1 = nis1.mean();
    m.nis1_samples = nis1.samples();
    m
}
