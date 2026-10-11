//! Integration test of the grid RBPF in a synthetic asymmetric room.
//!
//! The robot drives a short route with a turn; the odometry handed to the filter carries a
//! systematic yaw error (design below). The filter is compared with dead reckoning of the
//! same odometry.

use nalgebra::Vector2;
use rand::RngExt;
use robotics_core::{seeded_rng, LaserScan, Se2, Timestamp};
use robotics_sim::{Lidar, LidarConfig, World};
use robotics_slam::rbpf::{Rbpf, RbpfConfig};
use robotics_slam::SlamError;

/// Standard normal by Box-Muller from the repo's seeded generator.
fn gauss(rng: &mut impl RngExt) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
}

/// An asymmetric 6 m × 4.5 m room with two obstacles.
fn room() -> World {
    let mut w = World::empty();
    w.add_rectangle(Vector2::new(-3.0, -2.0), Vector2::new(3.0, 2.5))
        .unwrap();
    w.add_box(Vector2::new(1.0, 0.8), Vector2::new(0.4, 0.3), 0.4)
        .unwrap();
    w.add_box(Vector2::new(-1.5, 1.4), Vector2::new(0.5, 0.2), 0.0)
        .unwrap();
    w
}

/// Body-frame increments of the true route: 2.4 m east in 0.2 m steps, a 90 degree left turn
/// in three 30 degree steps, then 1.6 m north: about 4 m of path.
fn route() -> Vec<Se2> {
    let mut v = vec![Se2::new(0.2, 0.0, 0.0); 12];
    v.extend(vec![
        Se2::new(0.0, 0.0, std::f64::consts::FRAC_PI_2 / 3.0);
        3
    ]);
    v.extend(vec![Se2::new(0.2, 0.0, 0.0); 8]);
    v
}

struct Outcome {
    rbpf_err: f64,
    dr_err: f64,
    resamples: usize,
    min_n_eff: f64,
    max_n_eff: f64,
}

fn lidar_cfg() -> LidarConfig {
    let beams = 180;
    LidarConfig {
        beams,
        angle_max: -std::f64::consts::PI
            + std::f64::consts::TAU * (beams - 1) as f64 / beams as f64,
        ..LidarConfig::default()
    }
}

fn run(seed: u64, cfg: RbpfConfig, yaw_scale: f64, yaw_bias: f64) -> Outcome {
    run_keep(seed, cfg, yaw_scale, yaw_bias).0
}

/// Drive the route and return the outcome together with the final filter.
fn run_keep(seed: u64, cfg: RbpfConfig, yaw_scale: f64, yaw_bias: f64) -> (Outcome, Rbpf) {
    let world = room();
    let mut lidar = Lidar::new(lidar_cfg(), seed).unwrap();
    let mount = lidar.config().mount;
    let start = Se2::new(-2.2, -1.5, 0.0);
    let mut odo_rng = seeded_rng(seed ^ 0x0D0);
    let mut f = Rbpf::new(cfg, start, seed).unwrap();
    let mut truth = start;
    let mut dr = start;
    let stamp = Timestamp::ZERO;
    let scan0 = lidar.scan(&world, &truth, stamp);
    f.update(&scan0, &mount).unwrap();
    let (mut resamples, mut min_n_eff, mut max_n_eff) = (0, f64::INFINITY, 0.0_f64);
    for d in route() {
        truth = truth.compose(&d);
        let len = d.translation().norm();
        let noisy = Se2::new(
            d.x() * (1.0 + 0.02 * gauss(&mut odo_rng)),
            d.y(),
            yaw_scale * d.theta() + yaw_bias * len + 0.004 * gauss(&mut odo_rng),
        );
        dr = dr.compose(&noisy);
        f.predict(&noisy).unwrap();
        let scan = lidar.scan(&world, &truth, stamp);
        let r = f.update(&scan, &mount).unwrap();
        resamples += usize::from(r.resampled);
        min_n_eff = min_n_eff.min(r.n_eff_before);
        max_n_eff = max_n_eff.max(r.n_eff_before);
    }
    let est = f.pose_estimate();
    let outcome = Outcome {
        rbpf_err: (est.translation() - truth.translation()).norm(),
        dr_err: (dr.translation() - truth.translation()).norm(),
        resamples,
        min_n_eff,
        max_n_eff,
    };
    (outcome, f)
}

fn cfg(n: usize) -> RbpfConfig {
    RbpfConfig {
        n_particles: n,
        resolution: 0.1,
        half_extent: 4.0,
        ..RbpfConfig::default()
    }
}

const SEEDS: std::ops::RangeInclusive<u64> = 1..=8;
/// Yaw scale error of the odometry (3 %).
const YAW_SCALE: f64 = 1.03;
/// Yaw drift of the odometry per meter driven (rad/m), about 1.7 degrees per meter.
const YAW_BIAS: f64 = 0.03;

#[test]
fn rbpf_beats_dead_reckoning_with_biased_odometry() {
    let n = 12;
    let (mut sum_rbpf, mut sum_dr, mut worst_ratio) = (0.0, 0.0, 0.0_f64);
    for seed in SEEDS {
        let o = run(seed, cfg(n), YAW_SCALE, YAW_BIAS);
        println!(
            "seed {seed}: rbpf {:.3} m, dead reckoning {:.3} m, resamples {}, n_eff before in [{:.2}, {:.2}]",
            o.rbpf_err, o.dr_err, o.resamples, o.min_n_eff, o.max_n_eff
        );
        assert!(o.resamples >= 1, "seed {seed}: never resampled");
        assert!(o.min_n_eff.is_finite() && o.min_n_eff >= 1.0 - 1e-9);
        assert!(o.max_n_eff <= n as f64 + 1e-9);
        assert!(o.rbpf_err.is_finite() && o.dr_err > 0.05);
        worst_ratio = worst_ratio.max(o.rbpf_err / o.dr_err);
        sum_rbpf += o.rbpf_err;
        sum_dr += o.dr_err;
    }
    let k = SEEDS.count() as f64;
    println!(
        "mean rbpf {:.3} m, mean dead reckoning {:.3} m, worst per-seed ratio {worst_ratio:.3}",
        sum_rbpf / k,
        sum_dr / k
    );
    // Measured over these eight seeds: the filter was better on every one, by at worst the
    // ratio asserted here; the means differ by more than a factor of two.
    assert!(worst_ratio < 0.85, "worst ratio {worst_ratio}");
    assert!(sum_rbpf < 0.6 * sum_dr);
}

#[test]
fn best_particle_map_contains_the_walls() {
    let o = run_keep(2, cfg(12), YAW_SCALE, YAW_BIAS);
    let f = &o.1;
    let best = f.best_particle();
    assert!(best < f.particle_count());
    assert!(f.allocated_chunks(best) > 4);
    // South wall y = -2 and east wall x = 3: the best map holds occupied cells within one
    // cell of them; the room interior is not occupied.
    let near = |pts: &[Vector2<f64>]| {
        pts.iter()
            .filter_map(|p| f.particle_occupancy_at(best, p))
            .fold(0.0, f64::max)
    };
    let south: Vec<_> = (-6..=-4)
        .map(|k| Vector2::new(-1.0, -2.0 + 0.1 * (k + 5) as f64))
        .collect();
    let east: Vec<_> = (-1..=1)
        .map(|k| Vector2::new(3.0 + 0.1 * k as f64, -1.0))
        .collect();
    assert!(near(&south) > 0.6, "south wall {}", near(&south));
    assert!(near(&east) > 0.6, "east wall {}", near(&east));
    let inside = f.particle_occupancy_at(best, &Vector2::new(-1.0, -1.0));
    assert!(inside.is_none_or(|p| p < 0.5), "{inside:?}");
}

#[test]
fn chunk_sharing_is_bounded_by_allocation() {
    let o = run_keep(2, cfg(12), YAW_SCALE, YAW_BIAS);
    let f = &o.1;
    let total = f.allocated_chunks(0);
    let shared = (1..f.particle_count())
        .map(|j| f.chunks_shared(0, j))
        .max()
        .unwrap();
    println!("allocated {total} chunks, at most {shared} shared with particle 0");
    assert!(shared <= total);
    assert_eq!(f.chunks_shared(0, 0), total);
}

#[test]
fn whole_run_is_deterministic_per_seed() {
    let a = run(3, cfg(12), YAW_SCALE, YAW_BIAS);
    let b = run(3, cfg(12), YAW_SCALE, YAW_BIAS);
    let c = run(4, cfg(12), YAW_SCALE, YAW_BIAS);
    assert_eq!(a.rbpf_err.to_bits(), b.rbpf_err.to_bits());
    assert_eq!(a.min_n_eff.to_bits(), b.min_n_eff.to_bits());
    assert_ne!(a.rbpf_err.to_bits(), c.rbpf_err.to_bits());
}

#[test]
fn bad_inputs_are_errors_not_panics() {
    let ok = cfg(4);
    let invalid = |c: RbpfConfig| {
        matches!(
            Rbpf::new(c, Se2::IDENTITY, 1),
            Err(SlamError::InvalidParameter { .. })
        )
    };
    assert!(invalid(RbpfConfig {
        n_particles: 0,
        ..ok.clone()
    }));
    assert!(invalid(RbpfConfig {
        resolution: 0.0,
        ..ok.clone()
    }));
    assert!(invalid(RbpfConfig {
        resolution: -0.1,
        ..ok.clone()
    }));
    assert!(invalid(RbpfConfig {
        temper: 0.0,
        ..ok.clone()
    }));
    assert!(invalid(RbpfConfig {
        temper: 1.01,
        ..ok.clone()
    }));
    assert!(invalid(RbpfConfig {
        resample_ratio: 0.0,
        ..ok.clone()
    }));
    assert!(invalid(RbpfConfig {
        resample_ratio: 2.0,
        ..ok.clone()
    }));
    let mut f = Rbpf::new(ok, Se2::IDENTITY, 1).unwrap();
    assert!(matches!(
        f.predict(&Se2::new(f64::NAN, 0.0, 0.0)),
        Err(SlamError::NonFinite(_))
    ));
    assert!(matches!(
        f.update(&LaserScan::default(), &Se2::IDENTITY),
        Err(SlamError::EmptyInput(_))
    ));
    // The filter is still usable after the rejected inputs.
    let mut lidar = Lidar::new(lidar_cfg(), 1).unwrap();
    let scan = lidar.scan(&room(), &Se2::new(-2.2, -1.5, 0.0), Timestamp::ZERO);
    let r = f.update(&scan, &lidar.config().mount).unwrap();
    assert_eq!(r.particles, 4);
    assert!(!r.resampled);
}
