//! A seeded Rusty run along a route file that writes `slam-metrics.yaml` and the recorded scan
//! stream `scans.csv` it was graded on. The only input of `scripts/phase5_verdict.py`.
//!
//! ```text
//! cargo run --release -p robotics_slam --example slam_synthetic_run -- \
//!     --out DIR --route sim/webots/config/route_long.yaml [--laps 1] [--angular 0.8] \
//!     [--seed 1] [--yaw-bias 0.0] [--role development] [--info-inflation K]
//! ```
//!
//! The world is the Phase 3 room (walls at ±3 m, four 12-sided pillars of radius 0.15 m) scanned
//! by a lidar shaped like the Webots one (360 beams, 0.12 to 3.5 m, 15 mm of range noise, no
//! dropout). The robot drives the waypoints the way `route_driver` does: a turn in place at
//! `--angular` rad/s, then a straight leg at 0.18 m/s, one scan every 0.2 s, exact kinematics and
//! exact stamps. Ground truth is exact. The scans are written to `scans.csv` and read back, and
//! everything below is graded on what was read back.
//!
//! * S1a, S2: every consecutive pair of scans registered from zero motion (threshold 0.3 m)
//!   against the ground-truth relative pose, and the NEES of each under the reported information.
//! * S1b: seeded trial sets of ICP from random initial offsets of growing size, on one pair.
//! * S4, S5: the pose graph on LiDAR-odometry edges (edge information is the compounded ICP
//!   information, scaled by the frozen `info_inflation`), with and without verified loop closures.
//!   `--yaw-bias` injects a systematic yaw error per keyframe into the edges the graph is handed
//!   and says so in the file; 0 leaves only the natural drift of the scan matcher.
//! * S6: whether a second run of the whole pipeline on the recorded file is bit-identical, and
//!   the wall time of every registration and every optimization (the one place in the crate that
//!   reads a clock).
//! * S7: EKF-SLAM and the grid RBPF on the same route, fed SYNTHETIC wheel-odometry increments
//!   (ground-truth increments with a declared noise and a declared systematic yaw error per
//!   metre; Webots' odometry has none, see docs/phase-3-localization/theory.md section 6) and,
//!   for EKF-SLAM, the pillar detections of `robotics_localization::pillars` on the same scans.
//!   Informative: who does better is a statement about that odometry setting.
//!
//! It writes nothing outside `--out`, and it does not know the tolerances.

mod common;

use common::*;
use nalgebra::{Vector2, Vector3};
use rand::RngExt;
use robotics_core::angle::normalize_angle;
use robotics_core::{seeded_rng, Manifold, Se2, Timestamp};
use robotics_localization::pillars::{detect, PillarDetector};
use robotics_sim::{Lidar, LidarConfig, World};
use robotics_slam::ekf_slam::{EkfSlam, EkfSlamConfig, RangeBearing};
use robotics_slam::icp::register;
use robotics_slam::rbpf::{Rbpf, RbpfConfig};
use std::fmt::Write as _;

const PILLARS: [(f64, f64); 4] = [(1.8, 1.2), (-1.4, 2.0), (-2.0, -1.0), (0.9, -2.1)];
const LINEAR: f64 = 0.18;
const SCAN_PERIOD: f64 = 0.2;

fn v(x: f64, y: f64) -> Vector2<f64> {
    Vector2::new(x, y)
}

fn pillar_room() -> World {
    let mut w = World::empty();
    w.add_rectangle(v(-3.0, -3.0), v(3.0, 3.0)).unwrap();
    for (cx, cy) in PILLARS {
        let ring: Vec<_> = (0..12)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 12.0;
                v(cx + 0.15 * a.cos(), cy + 0.15 * a.sin())
            })
            .collect();
        w.add_polyline(&ring, true).unwrap();
    }
    w
}

fn lidar(seed: u64) -> Lidar {
    Lidar::new(
        LidarConfig {
            mount: Se2::IDENTITY,
            range_min: 0.12,
            range_max: 3.5,
            range_sigma: 0.015,
            dropout_probability: 0.0,
            rate_hz: 5.0,
            ..LidarConfig::default()
        },
        seed,
    )
    .unwrap()
}

fn waypoints(path: &str) -> Vec<(f64, f64)> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut out = Vec::new();
    for line in text.lines() {
        let t = line.trim();
        if t.starts_with('#') {
            continue;
        }
        if let (Some(xi), Some(yi)) = (t.find("x:"), t.find("y:")) {
            let num = |s: &str| -> f64 {
                s.trim_start()
                    .split([',', '}'])
                    .next()
                    .unwrap()
                    .trim()
                    .parse()
                    .expect("waypoint number")
            };
            out.push((num(&t[xi + 2..]), num(&t[yi + 2..])));
        }
    }
    assert!(!out.is_empty(), "{path}: no waypoints");
    out
}

/// Poses at scan rate along the waypoints: a turn in place, then a straight leg, per waypoint.
fn trajectory(wps: &[(f64, f64)], angular: f64) -> Vec<Se2> {
    let (mut x, mut y, mut th) = (0.0f64, 0.0f64, 0.0f64);
    let mut out = vec![Se2::new(x, y, th)];
    for &(wx, wy) in wps {
        let target = (wy - y).atan2(wx - x);
        let d = normalize_angle(target - th);
        let n = ((d.abs() / (angular * SCAN_PERIOD)) - 1e-9).ceil().max(0.0) as usize;
        for _ in 0..n {
            th = normalize_angle(th + d / n as f64);
            out.push(Se2::new(x, y, th));
        }
        let dist = (wx - x).hypot(wy - y);
        let n = ((dist / (LINEAR * SCAN_PERIOD)) - 1e-9).ceil().max(0.0) as usize;
        for i in 0..n {
            x += (wx - x) / (n - i) as f64;
            y += (wy - y) / (n - i) as f64;
            out.push(Se2::new(x, y, th));
        }
        th = target;
    }
    out
}

fn gauss(rng: &mut impl rand::Rng) -> f64 {
    let (u1, u2): (f64, f64) = (rng.random::<f64>().max(1e-300), rng.random::<f64>());
    (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
}

/// The declared synthetic odometry of S7. Per increment of translation `d` and rotation `φ`,
/// body-frame noise `σx = 0.02 d + 0.001`, `σy = 0.01 d + 0.001`, `σθ = 0.03 |φ| + 0.005 d + 0.002`,
/// plus a systematic yaw error of `YAW_BIAS_PER_M` rad per metre travelled.
const NOISE: [f64; 6] = [0.02, 0.001, 0.01, 0.001, 0.03, 0.005];
const NOISE_ROT_FLOOR: f64 = 0.002;
const YAW_BIAS_PER_M: f64 = 0.005;

fn noisy_increment(inc: &Se2, rng: &mut impl rand::Rng) -> (Se2, nalgebra::Matrix3<f64>) {
    let d = inc.translation().norm();
    let phi = inc.theta().abs();
    let sx = NOISE[0] * d + NOISE[1];
    let sy = NOISE[2] * d + NOISE[3];
    let st = NOISE[4] * phi + NOISE[5] * d + NOISE_ROT_FLOOR;
    let eps = Vector3::new(
        sx * gauss(rng),
        sy * gauss(rng),
        st * gauss(rng) + YAW_BIAS_PER_M * d,
    );
    let r = nalgebra::Matrix3::from_diagonal(&Vector3::new(sx * sx, sy * sy, st * st));
    (inc.boxplus(&eps), r)
}

fn traj_summary(est: &[Se2], truth: &[Se2]) -> (f64, f64, f64) {
    let errs: Vec<f64> = est
        .iter()
        .zip(truth)
        .map(|(a, b)| a.distance_to(b))
        .collect();
    let rms = (errs.iter().map(|e| e * e).sum::<f64>() / errs.len() as f64).sqrt();
    (
        rms,
        errs.iter().cloned().fold(0.0, f64::max),
        *errs.last().unwrap(),
    )
}

fn s7(frames: &[Frame], seed: u64, out: &mut String) {
    let origin = frames[0].gt;
    let truth: Vec<Se2> = frames.iter().map(|f| origin.between(&f.gt)).collect();
    let mut rng = seeded_rng(seed.wrapping_add(2000));
    let mut incs = Vec::new();
    for w in truth.windows(2) {
        incs.push(noisy_increment(&w[0].between(&w[1]), &mut rng));
    }
    let mut dead = vec![Se2::IDENTITY];
    for (d, _) in &incs {
        let next = dead.last().unwrap().compose(d);
        dead.push(next);
    }

    // EKF-SLAM with the pillar detections of the same scans.
    let mut ekf = EkfSlam::new(Se2::IDENTITY, EkfSlamConfig::default()).unwrap();
    let det = PillarDetector::default();
    let mut ekf_poses = vec![Se2::IDENTITY];
    let mut nees = Vec::new();
    let (mut within, mut counted) = (0usize, 0usize);
    let bound = robotics_core::prob::chi2_quantile(3, 0.975).unwrap();
    for (k, fr) in frames.iter().enumerate().skip(1) {
        let (d, r) = &incs[k - 1];
        ekf.predict(d, r).unwrap();
        let obs: Vec<RangeBearing> = detect(&fr.scan, &det)
            .iter()
            .map(|x| RangeBearing {
                range: x.z.range,
                bearing: x.z.bearing,
            })
            .collect();
        ekf.update(&obs).unwrap();
        let est = ekf.pose();
        ekf_poses.push(est);
        if let Some(pi) = ekf.pose_covariance().try_inverse() {
            let e = Vector3::new(
                truth[k].x() - est.x(),
                truth[k].y() - est.y(),
                normalize_angle(truth[k].theta() - est.theta()),
            );
            let n = e.dot(&(pi * e));
            nees.push(n);
            counted += 1;
            if n <= bound {
                within += 1;
            }
        }
    }
    let pill: Vec<Vector2<f64>> = PILLARS
        .iter()
        .map(|&(x, y)| origin.act_inverse(&v(x, y)))
        .collect();
    let mut lm_err = 0.0f64;
    let mut distinct = std::collections::BTreeSet::new();
    for i in 0..ekf.landmark_count() {
        let l = ekf.landmark(i);
        let (j, d) = pill
            .iter()
            .enumerate()
            .map(|(j, p)| (j, (p - l).norm()))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();
        lm_err = lm_err.max(d);
        distinct.insert(j);
    }

    // The grid RBPF on the same increments and the same scans.
    let mut rbpf = Rbpf::new(
        RbpfConfig::default(),
        Se2::IDENTITY,
        seed.wrapping_add(3000),
    )
    .unwrap();
    let mut rb_poses = vec![Se2::IDENTITY];
    let (mut resamples, mut neff_sum) = (0usize, 0.0f64);
    rbpf.update(&frames[0].scan, &Se2::IDENTITY).unwrap();
    for (k, fr) in frames.iter().enumerate().skip(1) {
        rbpf.predict(&incs[k - 1].0).unwrap();
        let rep = rbpf.update(&fr.scan, &Se2::IDENTITY).unwrap();
        resamples += usize::from(rep.resampled);
        neff_sum += rep.n_eff_before;
        rb_poses.push(rbpf.pose_estimate());
    }

    let (dr, ek, rb) = (
        traj_summary(&dead, &truth),
        traj_summary(&ekf_poses, &truth),
        traj_summary(&rb_poses, &truth),
    );
    let _ = writeln!(out, "s7:");
    let _ = writeln!(
        out,
        "  odometry_noise: {{trans_frac: {}, trans_floor_m: {}, lateral_frac: {}, lateral_floor_m: {}, rot_per_rad: {}, rot_per_m: {}, rot_floor_rad: {NOISE_ROT_FLOOR}, yaw_bias_rad_per_m: {YAW_BIAS_PER_M}}}",
        NOISE[0], NOISE[1], NOISE[2], NOISE[3], NOISE[4], NOISE[5]
    );
    let _ = writeln!(
        out,
        "  dead_reckoning: {{ate_rms_m: {:.12}, ate_max_m: {:.12}, final_error_m: {:.12}}}",
        dr.0, dr.1, dr.2
    );
    let _ = writeln!(
        out,
        "  ekf_slam: {{ate_rms_m: {:.12}, ate_max_m: {:.12}, final_error_m: {:.12}, mean_nees: {:.12}, nees_samples: {counted}, nees_within_975_bound_fraction: {:.12}, landmarks: {}, distinct_pillars: {}, landmark_error_max_m: {:.12}, provisional_left: {}}}",
        ek.0,
        ek.1,
        ek.2,
        nees.iter().sum::<f64>() / nees.len().max(1) as f64,
        within as f64 / counted.max(1) as f64,
        ekf.landmark_count(),
        distinct.len(),
        lm_err,
        ekf.provisional_count()
    );
    let _ = writeln!(
        out,
        "  rbpf: {{particles: {}, ate_rms_m: {:.12}, ate_max_m: {:.12}, final_error_m: {:.12}, resamples: {resamples}, mean_n_eff_before: {:.12}}}",
        rbpf.particle_count(),
        rb.0,
        rb.1,
        rb.2,
        neff_sum / (frames.len() - 1) as f64
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |name: &str, default: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
            .unwrap_or_else(|| default.to_string())
    };
    let out_dir = get("--out", "");
    let route = get("--route", "");
    if out_dir.is_empty() || route.is_empty() {
        eprintln!("usage: slam_synthetic_run --out DIR --route ROUTE.yaml [--laps N] [--angular R] [--seed N] [--yaw-bias R] [--role development|evaluation] [--info-inflation K]");
        std::process::exit(2);
    }
    let seed: u64 = get("--seed", "1").parse().expect("--seed");
    let laps: usize = get("--laps", "1").parse().expect("--laps");
    let angular: f64 = get("--angular", "0.8").parse().expect("--angular");
    let yaw_bias: f64 = get("--yaw-bias", "0.0").parse().expect("--yaw-bias");
    let role = get("--role", "development");
    let inflation: Option<f64> = args
        .iter()
        .position(|a| a == "--info-inflation")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.parse().expect("--info-inflation"));
    std::fs::create_dir_all(&out_dir).expect("create --out");

    // Drive the route and record the scans.
    let world = pillar_room();
    let wps: Vec<(f64, f64)> = (0..laps).flat_map(|_| waypoints(&route)).collect();
    let poses = trajectory(&wps, angular);
    let mut lid = lidar(seed);
    let recorded: Vec<Frame> = poses
        .iter()
        .enumerate()
        .map(|(k, p)| {
            let stamp = SCAN_PERIOD * k as f64;
            Frame {
                stamp,
                gt: *p,
                scan: lid.scan(&world, p, Timestamp::from_secs_f64(stamp)),
            }
        })
        .collect();
    let csv = format!("{out_dir}/scans.csv");
    write_frames(&csv, &recorded).expect("write scans.csv");
    // Everything below is graded on the file, not on the process that made it.
    let frames = read_frames(&csv).expect("read scans.csv");
    let cfg = config(inflation);

    let mut out = String::new();
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let route_name = std::path::Path::new(&route)
        .file_name()
        .unwrap()
        .to_string_lossy()
        .to_string();
    let _ = writeln!(out, "schema: phase5-slam-metrics/1");
    let _ = writeln!(out, "run:");
    let _ = writeln!(out, "  data_source: synthetic");
    let _ = writeln!(out, "  ground_truth: exact");
    let _ = writeln!(out, "  role: {role}");
    let _ = writeln!(out, "  seed: {seed}");
    let _ = writeln!(
        out,
        "  route: {route_name} laps={laps} angular_rad_s={angular}"
    );
    let _ = writeln!(out, "  injected_yaw_bias_rad_per_keyframe: {yaw_bias:.12}");
    let _ = writeln!(out, "  info_inflation: {:.12}", cfg.info_inflation);
    let _ = writeln!(out, "  robotics_mapping_dependency: false");
    let _ = writeln!(out, "  build_profile: {profile}");

    let s1a = s1a_pass(&frames, |_, _| true, &cfg.s1a_icp);
    write_s1a_s2(&mut out, &s1a);

    // S1b.
    let mid = frames.len() / 3;
    let (src, tgt) = (
        points(&frames[mid + 5].scan),
        target_map(&points(&frames[mid].scan)),
    );
    let truth_rel = frames[mid].gt.between(&frames[mid + 5].gt);
    let mut rng = seeded_rng(seed.wrapping_add(1000));
    let _ = writeln!(out, "s1b:");
    let _ = writeln!(out, "  success_error_m: 0.02");
    let _ = writeln!(out, "  success_error_rad: 0.01");
    let _ = writeln!(out, "  ranges:");
    // The first four ranges were fixed in advance (benchmark.md section 6). The last two were added
    // after the development run showed 100 of 100 successes at the largest of them, to find where
    // the basin ends; they are reported and never graded.
    for (rt, rr) in [
        (0.05, 0.05),
        (0.10, 0.10),
        (0.20, 0.15),
        (0.30, 0.25),
        (0.50, 0.40),
        (1.00, 0.80),
    ] {
        let (n, mut ok) = (100usize, 0usize);
        for _ in 0..n {
            let ang: f64 = rng.random_range(0.0..std::f64::consts::TAU);
            let rad: f64 = rt * rng.random::<f64>().sqrt();
            let dth: f64 = rng.random_range(-rr..rr);
            let init = truth_rel.boxplus(&Vector3::new(rad * ang.cos(), rad * ang.sin(), dth));
            let r = register(&src, &tgt, init, &cfg.s1a_icp).unwrap();
            let e = truth_rel.between(&r.pose).log();
            if r.settled && e.xy().norm() <= 0.02 && e[2].abs() <= 0.01 {
                ok += 1;
            }
        }
        let _ = writeln!(
            out,
            "    - {{offset_m: {rt}, offset_rad: {rr}, trials: {n}, successes: {ok}}}"
        );
    }

    // S4, S5, S6 on the recorded stream, run twice.
    let a = pipeline(&frames, yaw_bias, &cfg);
    let b = pipeline(&frames, yaw_bias, &cfg);
    let s1a_again = s1a_pass(&frames, |_, _| true, &cfg.s1a_icp);
    let identical = pipeline_bits(&a) == pipeline_bits(&b) && s1a.bits == s1a_again.bits;
    write_s4_s5(&mut out, &a);
    let _ = writeln!(out, "s6:");
    let _ = writeln!(out, "  replay_identical: {identical}");
    let _ = writeln!(out, "  scan_period_s: {SCAN_PERIOD}");
    stats("registration_latency_s", &a.registration_s, &mut out, "  ");
    stats("optimization_latency_s", &a.optimization_s, &mut out, "  ");
    let _ = writeln!(
        out,
        "# pipeline: {} scans, {} registrations lost to prediction",
        frames.len(),
        a.lost_registrations
    );

    s7(&frames, seed, &mut out);

    std::fs::write(format!("{out_dir}/slam-metrics.yaml"), &out).expect("write metrics");
    print!("{out}");
}
