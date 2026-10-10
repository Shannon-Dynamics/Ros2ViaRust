//! Print raw-wheel versus fused (wheel + gyro ESKF) accuracy and consistency on the
//! deterministic Rusty simulator for several seeds.
//!
//! ```text
//! cargo run --release -p robotics_sim --example estimator_accuracy
//! ```
//!
//! The mixed trajectory contains straight runs, turns in place, left/right arcs, a spin
//! across ±π and stops. Reported per seed: position / yaw RMSE and final errors (raw and
//! fused), gyro bias estimate, the fraction of samples whose yaw error is inside ±3σ, mean
//! pose NEES (3 DoF; 3 is consistent) for raw and fused, and mean NIS of the fused
//! corrections (2-DoF wheel+gyro, expected 2).

#[path = "../tests/common/pipeline.rs"]
mod pipeline;

use robotics_filters::FusionConfig;
use robotics_sim::RustyConfig;

fn main() {
    let seconds = 180.0;
    println!("Rusty mixed trajectory, {seconds} s, default noise (slip 2 %/step, right radius +0.2 %, gyro 0.005 rad/s/sqrt(Hz), bias 0.005 rad/s + RW)");
    println!(
        "{:>4} | {:>9} {:>9} {:>9} | {:>9} {:>9} {:>9} | {:>8} {:>8} {:>5} | {:>8} {:>8} {:>6}",
        "seed",
        "raw pos",
        "raw yaw",
        "raw end",
        "fus pos",
        "fus yaw",
        "fus end",
        "bias",
        "b est",
        "3σ%",
        "NEES raw",
        "NEES fus",
        "NIS2"
    );
    let variants = [
        (
            "bias update at low speed only (default)",
            FusionConfig::default(),
        ),
        (
            "bias update at every speed",
            FusionConfig {
                bias_update_max_speed: f64::INFINITY,
                ..FusionConfig::default()
            },
        ),
    ];
    for (name, fusion_cfg) in variants {
        println!("-- {name}   (RMSE in m / rad)");
        for seed in [1u64, 2, 3, 4, 5] {
            let cfg = RustyConfig {
                seed,
                ..RustyConfig::default()
            };
            let m = pipeline::run_pipeline(cfg, fusion_cfg, seconds);
            println!(
                "{:>4} | {:>9.3} {:>9.4} {:>9.3} | {:>9.3} {:>9.4} {:>9.3} | {:>8.5} {:>8.5} {:>5.1} | {:>8.2} {:>8.2} {:>6.2}",
                seed,
                m.raw_pos_rmse,
                m.raw_yaw_rmse,
                m.raw_final_pos,
                m.fused_pos_rmse,
                m.fused_yaw_rmse,
                m.fused_final_pos,
                m.true_bias,
                m.est_bias,
                100.0 * m.yaw_within_3sigma,
                m.raw_nees,
                m.fused_nees,
                m.fused_nis2
            );
        }
    }
}
