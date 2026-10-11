//! A seeded Rusty run that writes `slam-metrics.yaml`, the only input of
//! `scripts/phase5_verdict.py` (schema in that file's header).
//!
//! ```text
//! cargo run --release -p robotics_slam --example slam_synthetic_run -- \
//!     --out DIR [--seed 1] [--laps 2.0] [--yaw-bias 0.0] [--role development]
//! ```
//!
//! The world is the Phase 3 room (walls at ±3 m, four pillars) scanned by a lidar shaped like the
//! Webots one. Ground truth is exact. What is measured, by claim:
//!
//! * S1a, S2: every registration of scan `k+1` against scan `k` along the route, started from
//!   zero motion, against the ground-truth relative pose; and the NEES of each under the
//!   reported information.
//! * S1b: seeded trial sets of ICP from random initial offsets of growing size.
//! * S4: the keyframe trajectory of the pose graph against ground truth, with and without the
//!   verified loop closures. `--yaw-bias` injects a systematic yaw error per keyframe into the
//!   odometry edges the graph is handed (and says so in the file); 0 leaves only the natural
//!   drift of the scan matcher.
//! * S5: every loop candidate, labelled by the error of the accepted measurement against ground
//!   truth (a wrong constraint is the harm), and the reason for each rejection.
//! * S6: whether a second run is bit-identical, and the latency of registrations and
//!   optimizations (wall time: this is the one place in the crate that reads a clock).
//!
//! It writes nothing outside `--out`, and it does not know the tolerances.

use nalgebra::{Matrix3, Vector2, Vector3};
use rand::RngExt;
use robotics_core::{seeded_rng, LaserScan, Manifold, Se2, Timestamp};
use robotics_sim::{Lidar, LidarConfig, World};
use robotics_slam::icp::{register, scan_points, IcpConfig};
use robotics_slam::odometry::{EdgeAccumulator, LidarOdometry, OdometryConfig};
use robotics_slam::pose_graph::{LoopOutcome, PoseGraph, PoseGraphConfig, Reason, Verification};
use robotics_slam::voxel::{downsample, VoxelMap};
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::time::Instant;

fn v(x: f64, y: f64) -> Vector2<f64> {
    Vector2::new(x, y)
}

fn pillar_room() -> World {
    let mut w = World::empty();
    w.add_rectangle(v(-3.0, -3.0), v(3.0, 3.0)).unwrap();
    for (cx, cy) in [(1.8, 1.2), (-1.4, 2.0), (-2.0, -1.0), (0.9, -2.1)] {
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

fn circle_route(radius: f64, step: f64, laps: f64) -> Vec<Se2> {
    let n = (std::f64::consts::TAU * radius * laps / step).round() as usize;
    (0..n)
        .map(|k| {
            let a = k as f64 * step / radius;
            Se2::new(
                radius * a.cos(),
                radius * a.sin(),
                a + std::f64::consts::FRAC_PI_2,
            )
        })
        .collect()
}

fn points(scan: &LaserScan) -> Vec<Vector2<f64>> {
    downsample(&scan_points(scan), 0.05).unwrap()
}

fn target_map(points: &[Vector2<f64>]) -> VoxelMap {
    let mut m = VoxelMap::new(0.1, 8).unwrap();
    m.insert_all(points);
    m
}

fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let rank = ((q * sorted.len() as f64).ceil() as usize).clamp(1, sorted.len());
    sorted[rank - 1]
}

fn sorted(mut x: Vec<f64>) -> Vec<f64> {
    x.sort_by(|a, b| a.total_cmp(b));
    x
}

fn stats(label: &str, x: &[f64], out: &mut String, indent: &str) {
    let s = sorted(x.to_vec());
    if s.is_empty() {
        let _ = writeln!(
            out,
            "{indent}{label}: {{median: 0.0, p95: 0.0, p99: 0.0, max: 0.0, count: 0}}"
        );
        return;
    }
    let _ = writeln!(
        out,
        "{indent}{label}: {{median: {:.12}, p95: {:.12}, p99: {:.12}, max: {:.12}, count: {}}}",
        percentile(&s, 0.5),
        percentile(&s, 0.95),
        percentile(&s, 0.99),
        s.last().copied().unwrap_or(f64::NAN),
        s.len()
    );
}

fn info(sx: f64, sth: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(
        1.0 / (sx * sx),
        1.0 / (sx * sx),
        1.0 / (sth * sth),
    ))
}

struct Pipeline {
    graph: PoseGraph,
    truth: Vec<Se2>,
    dead: Vec<Se2>,
    outcomes: Vec<LoopOutcome>,
    registration_s: Vec<f64>,
    optimization_s: Vec<f64>,
    path_length: f64,
}

fn pipeline(seed: u64, laps: f64, yaw_bias: f64) -> Pipeline {
    let world = pillar_room();
    let route = circle_route(1.4, 0.06, laps);
    let mut lid = lidar(seed);
    let mut odo = LidarOdometry::new(OdometryConfig::default()).unwrap();
    let mut graph = PoseGraph::new(PoseGraphConfig::default()).unwrap();
    let mut acc = EdgeAccumulator::new();
    let origin = route[0];
    let (mut truth, mut outcomes) = (Vec::new(), Vec::new());
    let (mut registration_s, mut optimization_s) = (Vec::new(), Vec::new());
    for (k, p) in route.iter().enumerate() {
        let scan = lid.scan(&world, p, Timestamp::from_secs_f64(0.2 * k as f64));
        let t0 = Instant::now();
        let step = odo.step(&scan).unwrap();
        registration_s.push(t0.elapsed().as_secs_f64());
        if k == 0 {
            graph
                .add_keyframe(points(&scan), &Se2::IDENTITY, &info(0.03, 0.01))
                .unwrap();
            truth.push(origin.between(p));
            continue;
        }
        acc.push(&step.delta, &step.information).unwrap();
        if graph.wants_keyframe(&acc.delta()) {
            let (delta, _) = acc.take().unwrap();
            let biased = delta.boxplus(&Vector3::new(0.0, 0.0, yaw_bias));
            let id = graph
                .add_keyframe(points(&scan), &biased, &info(0.03, 0.01))
                .unwrap();
            truth.push(origin.between(p));
            let t0 = Instant::now();
            let out = graph.process_loops(id).unwrap();
            if out
                .iter()
                .any(|o| matches!(o.verification, Verification::Accepted { .. }))
            {
                optimization_s.push(t0.elapsed().as_secs_f64());
            }
            outcomes.extend(out);
        }
    }
    let mut dead = vec![Se2::IDENTITY];
    for f in graph.factor_graph().factors().iter().skip(1) {
        if let robotics_slam::factor_graph::FactorKind::Between {
            from,
            to,
            measurement,
        } = f.kind()
        {
            if *to == *from + 1 && dead.len() == *to {
                let next = dead[*from].compose(measurement);
                dead.push(next);
            }
        }
    }
    let path_length = truth.windows(2).map(|w| w[0].distance_to(&w[1])).sum();
    Pipeline {
        graph,
        truth,
        dead,
        outcomes,
        registration_s,
        optimization_s,
        path_length,
    }
}

fn trajectory_block(label: &str, est: &[Se2], truth: &[Se2], path: f64, out: &mut String) {
    let errs: Vec<f64> = est
        .iter()
        .zip(truth)
        .map(|(a, b)| a.distance_to(b))
        .collect();
    let rms = (errs.iter().map(|e| e * e).sum::<f64>() / errs.len() as f64).sqrt();
    let max = errs.iter().cloned().fold(0.0, f64::max);
    let last = errs.last().copied().unwrap_or(f64::NAN);
    let heading = est
        .last()
        .zip(truth.last())
        .map(|(a, b)| a.between(b).log()[2].abs())
        .unwrap_or(f64::NAN);
    let _ = writeln!(
        out,
        "  {label}: {{ate_rms_m: {rms:.12}, ate_max_m: {max:.12}, final_error_m: {last:.12}, drift_m_per_m: {:.12}, heading_drift_rad_per_m: {:.12}}}",
        last / path,
        heading / path
    );
}

fn bits(p: &Pipeline) -> Vec<u64> {
    p.graph
        .poses()
        .iter()
        .flat_map(|q| [q.x().to_bits(), q.y().to_bits(), q.theta().to_bits()])
        .collect()
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
    if out_dir.is_empty() {
        eprintln!("usage: slam_synthetic_run --out DIR [--seed N] [--laps X] [--yaw-bias R] [--role development|evaluation]");
        std::process::exit(2);
    }
    let seed: u64 = get("--seed", "1").parse().expect("--seed");
    let laps: f64 = get("--laps", "2.0").parse().expect("--laps");
    let yaw_bias: f64 = get("--yaw-bias", "0.0").parse().expect("--yaw-bias");
    let role = get("--role", "development");

    let world = pillar_room();
    let route = circle_route(1.4, 0.06, laps);
    let mut out = String::new();
    let _ = writeln!(out, "schema: phase5-slam-metrics/1");
    let _ = writeln!(out, "run:");
    let _ = writeln!(out, "  data_source: synthetic");
    let _ = writeln!(out, "  ground_truth: exact");
    let _ = writeln!(out, "  role: {role}");
    let _ = writeln!(out, "  seed: {seed}");
    let _ = writeln!(out, "  route: circle_r1.4_step0.06_laps{laps}");
    let _ = writeln!(out, "  injected_yaw_bias_rad_per_keyframe: {yaw_bias:.12}");
    let _ = writeln!(out, "  robotics_mapping_dependency: false");
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let _ = writeln!(out, "  build_profile: {profile}");

    // S1a and S2: consecutive scans, started from zero motion.
    let mut lid = lidar(seed);
    let scans: Vec<LaserScan> = route
        .iter()
        .enumerate()
        .map(|(k, p)| lid.scan(&world, p, Timestamp::from_secs_f64(0.2 * k as f64)))
        .collect();
    let cfg = IcpConfig {
        threshold: 0.3,
        ..Default::default()
    };
    let (mut te, mut re, mut nees) = (Vec::new(), Vec::new(), Vec::new());
    let mut made = 0;
    for k in 0..scans.len() - 1 {
        made += 1;
        let r = register(
            &points(&scans[k + 1]),
            &target_map(&points(&scans[k])),
            Se2::IDENTITY,
            &cfg,
        )
        .unwrap();
        if !r.settled {
            continue;
        }
        let gt = route[k].between(&route[k + 1]);
        let e = gt.between(&r.pose).log();
        te.push(e.xy().norm());
        re.push(e[2].abs());
        nees.push(e.dot(&(r.information * e)));
    }
    let _ = writeln!(out, "s1a:");
    let _ = writeln!(out, "  registrations_made: {made}");
    let _ = writeln!(out, "  registrations_graded: {}", te.len());
    stats("translation_error_m", &te, &mut out, "  ");
    stats("rotation_error_rad", &re, &mut out, "  ");
    let _ = writeln!(out, "s2:");
    let _ = writeln!(out, "  dof: 3");
    let _ = writeln!(out, "  samples: {}", nees.len());
    let _ = writeln!(
        out,
        "  mean_nees: {:.12}",
        nees.iter().sum::<f64>() / nees.len().max(1) as f64
    );

    // S1b: seeded trial sets from random offsets.
    let mid = scans.len() / 3;
    let (src, tgt) = (points(&scans[mid + 5]), target_map(&points(&scans[mid])));
    let truth_rel = route[mid].between(&route[mid + 5]);
    let mut rng = seeded_rng(seed.wrapping_add(1000));
    let _ = writeln!(out, "s1b:");
    let _ = writeln!(out, "  success_error_m: 0.02");
    let _ = writeln!(out, "  success_error_rad: 0.01");
    let _ = writeln!(out, "  ranges:");
    for (rt, rr) in [(0.05, 0.05), (0.10, 0.10), (0.20, 0.15), (0.30, 0.25)] {
        let (n, mut ok) = (100usize, 0usize);
        for _ in 0..n {
            let ang: f64 = rng.random_range(0.0..std::f64::consts::TAU);
            let rad: f64 = rt * rng.random::<f64>().sqrt();
            let dth: f64 = rng.random_range(-rr..rr);
            let init = truth_rel.boxplus(&Vector3::new(rad * ang.cos(), rad * ang.sin(), dth));
            let r = register(&src, &tgt, init, &cfg).unwrap();
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

    // S4, S5, S6.
    let a = pipeline(seed, laps, yaw_bias);
    let b = pipeline(seed, laps, yaw_bias);
    let identical = bits(&a) == bits(&b);
    let _ = writeln!(out, "s4:");
    let _ = writeln!(out, "  keyframes_made: {}", a.graph.keyframes().len());
    let _ = writeln!(out, "  keyframes_graded: {}", a.truth.len());
    let _ = writeln!(out, "  path_length_m: {:.12}", a.path_length);
    trajectory_block("odometry", &a.dead, &a.truth, a.path_length, &mut out);
    trajectory_block("graph", a.graph.poses(), &a.truth, a.path_length, &mut out);

    let (mut acc_ok, mut acc_wrong, mut true_prop, mut true_acc) = (0, 0, 0, 0);
    let mut reasons: BTreeMap<&str, usize> = BTreeMap::new();
    for o in &a.outcomes {
        let c = &o.candidate;
        let gt = a.truth[c.from].between(&a.truth[c.to]);
        let true_loop = gt.translation().norm() < 1.5;
        if true_loop {
            true_prop += 1;
        }
        match &o.verification {
            Verification::Accepted { measurement, .. } => {
                let e = gt.between(measurement).log();
                if e.xy().norm() < 0.05 && e[2].abs() < 0.03 {
                    acc_ok += 1;
                } else {
                    acc_wrong += 1;
                }
                if true_loop {
                    true_acc += 1;
                }
            }
            Verification::Rejected { reason, .. } => {
                let name = match reason {
                    Reason::NoOverlap => "no_overlap",
                    Reason::NotConverged => "not_converged",
                    Reason::PoorFit => "poor_fit",
                    Reason::Degenerate => "degenerate",
                    Reason::GateExceeded { .. } => "gate_exceeded",
                    Reason::Ambiguous => "ambiguous",
                };
                *reasons.entry(name).or_default() += 1;
            }
        }
    }
    let _ = writeln!(out, "s5:");
    let _ = writeln!(out, "  candidates: {}", a.outcomes.len());
    let _ = writeln!(out, "  accepted_correct: {acc_ok}");
    let _ = writeln!(out, "  accepted_wrong: {acc_wrong}");
    let _ = writeln!(out, "  true_loops_proposed: {true_prop}");
    let _ = writeln!(out, "  true_loops_accepted: {true_acc}");
    let _ = writeln!(out, "  wrong_constraint_error_m: 0.05");
    let _ = writeln!(out, "  wrong_constraint_error_rad: 0.03");
    let _ = writeln!(out, "  rejected_by_reason:");
    for name in [
        "no_overlap",
        "not_converged",
        "poor_fit",
        "degenerate",
        "gate_exceeded",
        "ambiguous",
    ] {
        let _ = writeln!(
            out,
            "    {name}: {}",
            reasons.get(name).copied().unwrap_or(0)
        );
    }
    let _ = writeln!(out, "s6:");
    let _ = writeln!(out, "  replay_identical: {identical}");
    let _ = writeln!(out, "  scan_period_s: 0.2");
    stats("registration_latency_s", &a.registration_s, &mut out, "  ");
    stats("optimization_latency_s", &a.optimization_s, &mut out, "  ");

    std::fs::create_dir_all(&out_dir).expect("create --out");
    std::fs::write(format!("{out_dir}/slam-metrics.yaml"), &out).expect("write metrics");
    print!("{out}");
}
