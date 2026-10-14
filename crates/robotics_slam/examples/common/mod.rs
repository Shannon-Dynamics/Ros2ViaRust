//! Shared by the two evidence harnesses, `slam_synthetic_run` and `slam_recorded_run`.
//!
//! A recorded scan stream is a CSV in the format of `integrated-scans.csv` (Phase 4): one scan per
//! line, `stamp_s, sensor_x, sensor_y, sensor_theta, angle_min, angle_increment, range_min,
//! range_max, n, ranges...`, where the sensor pose is the ground truth the claim is graded
//! against. The synthetic harness writes its scans in that format and then reads them back, so
//! every claim is graded on the recorded file and not on the process that made it.
#![allow(dead_code)]

use nalgebra::{Matrix3, Vector2, Vector3};
use robotics_core::{LaserScan, Manifold, Se2, Timestamp};
use robotics_slam::factor_graph::FactorKind;
use robotics_slam::icp::{register, scan_points, IcpConfig};
use robotics_slam::odometry::{EdgeAccumulator, LidarOdometry, OdometryConfig};
use robotics_slam::pose_graph::{LoopOutcome, PoseGraph, PoseGraphConfig, Reason, Verification};
use robotics_slam::voxel::{downsample, VoxelMap};
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::time::Instant;

/// One recorded scan with the ground-truth pose of its sensor.
#[derive(Clone)]
pub struct Frame {
    pub stamp: f64,
    pub gt: Se2,
    pub scan: LaserScan,
}

pub fn read_frames(path: &str) -> Result<Vec<Frame>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let mut out = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 9 {
            return Err(format!("{path}:{}: too few fields", ln + 1));
        }
        let num = |i: usize| -> Result<f64, String> {
            f[i].trim()
                .parse::<f64>()
                .map_err(|e| format!("{path}:{}: field {i}: {e}", ln + 1))
        };
        let n = num(8)? as usize;
        if f.len() != 9 + n {
            return Err(format!(
                "{path}:{}: {} ranges, header says {n}",
                ln + 1,
                f.len() - 9
            ));
        }
        let mut ranges = Vec::with_capacity(n);
        for r in &f[9..] {
            ranges.push(
                r.trim()
                    .parse::<f32>()
                    .map_err(|e| format!("{path}:{}: {e}", ln + 1))?,
            );
        }
        out.push(Frame {
            stamp: num(0)?,
            gt: Se2::new(num(1)?, num(2)?, num(3)?),
            scan: LaserScan {
                stamp: Timestamp::from_secs_f64(num(0)?),
                angle_min: num(4)?,
                angle_increment: num(5)?,
                range_min: num(6)?,
                range_max: num(7)?,
                ranges,
                ..LaserScan::default()
            },
        });
    }
    Ok(out)
}

pub fn write_frames(path: &str, frames: &[Frame]) -> std::io::Result<()> {
    let mut s = String::from(
        "# stamp_s,sensor_x,sensor_y,sensor_theta,angle_min,angle_increment,range_min,range_max,n,ranges...\n",
    );
    for fr in frames {
        let sc = &fr.scan;
        let _ = write!(
            s,
            "{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{}",
            fr.stamp,
            fr.gt.x(),
            fr.gt.y(),
            fr.gt.theta(),
            sc.angle_min,
            sc.angle_increment,
            sc.range_min,
            sc.range_max,
            sc.ranges.len()
        );
        for r in &sc.ranges {
            if r.is_finite() {
                let _ = write!(s, ",{r:.4}");
            } else {
                s.push_str(",inf");
            }
        }
        s.push('\n');
    }
    std::fs::write(path, s)
}

pub fn points(scan: &LaserScan) -> Vec<Vector2<f64>> {
    downsample(&scan_points(scan), 0.05).unwrap()
}

pub fn target_map(points: &[Vector2<f64>]) -> VoxelMap {
    let mut m = VoxelMap::new(0.1, 8).unwrap();
    m.insert_all(points);
    m
}

pub fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let rank = ((q * sorted.len() as f64).ceil() as usize).clamp(1, sorted.len());
    sorted[rank - 1]
}

pub fn stats(label: &str, x: &[f64], out: &mut String, indent: &str) {
    let mut s = x.to_vec();
    s.sort_by(|a, b| a.total_cmp(b));
    let _ = writeln!(
        out,
        "{indent}{label}: {{median: {:.12}, p95: {:.12}, p99: {:.12}, max: {:.12}, count: {}}}",
        percentile(&s, 0.5),
        percentile(&s, 0.95),
        percentile(&s, 0.99),
        s.last().copied().unwrap_or(0.0),
        s.len()
    );
}

pub fn info(sx: f64, sth: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(
        1.0 / (sx * sx),
        1.0 / (sx * sx),
        1.0 / (sth * sth),
    ))
}

/// The configuration every claim is graded with. `inflation` overrides `info_inflation` in the
/// front end, the verifier and the S1a registrations alike; `None` keeps the crate default.
pub struct Config {
    pub s1a_icp: IcpConfig,
    pub odometry: OdometryConfig,
    pub graph: PoseGraphConfig,
    pub info_inflation: f64,
}

pub fn config(inflation: Option<f64>) -> Config {
    let mut s1a_icp = IcpConfig {
        threshold: 0.3,
        ..IcpConfig::default()
    };
    let mut odometry = OdometryConfig::default();
    let mut graph = PoseGraphConfig::default();
    if let Some(k) = inflation {
        s1a_icp.info_inflation = k;
        odometry.icp.info_inflation = k;
        graph.loops.icp.info_inflation = k;
    }
    let info_inflation = s1a_icp.info_inflation;
    Config {
        s1a_icp,
        odometry,
        graph,
        info_inflation,
    }
}

/// S1a and S2: register each eligible consecutive pair from zero motion and grade it against the
/// ground-truth relative pose.
pub struct S1a {
    pub made: usize,
    pub translation_error: Vec<f64>,
    pub rotation_error: Vec<f64>,
    pub nees: Vec<f64>,
    pub seconds: Vec<f64>,
    pub bits: Vec<u64>,
}

pub fn s1a_pass(
    frames: &[Frame],
    eligible: impl Fn(&Frame, &Frame) -> bool,
    cfg: &IcpConfig,
) -> S1a {
    let mut r = S1a {
        made: 0,
        translation_error: vec![],
        rotation_error: vec![],
        nees: vec![],
        seconds: vec![],
        bits: vec![],
    };
    for w in frames.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        if !eligible(a, b) {
            continue;
        }
        r.made += 1;
        let (src, tgt) = (points(&b.scan), target_map(&points(&a.scan)));
        let t0 = Instant::now();
        let res = register(&src, &tgt, Se2::IDENTITY, cfg).unwrap();
        r.seconds.push(t0.elapsed().as_secs_f64());
        r.bits.extend([
            res.pose.x().to_bits(),
            res.pose.y().to_bits(),
            res.pose.theta().to_bits(),
        ]);
        if !res.settled {
            continue;
        }
        let gt = a.gt.between(&b.gt);
        let e = gt.between(&res.pose).log();
        r.translation_error.push(e.xy().norm());
        r.rotation_error.push(e[2].abs());
        r.nees.push(e.dot(&(res.information * e)));
    }
    r
}

pub fn write_s1a_s2(out: &mut String, s: &S1a) {
    let _ = writeln!(out, "s1a:");
    let _ = writeln!(out, "  registrations_made: {}", s.made);
    let _ = writeln!(out, "  registrations_graded: {}", s.translation_error.len());
    stats("translation_error_m", &s.translation_error, out, "  ");
    stats("rotation_error_rad", &s.rotation_error, out, "  ");
    let _ = writeln!(out, "s2:");
    let _ = writeln!(out, "  dof: 3");
    let _ = writeln!(out, "  samples: {}", s.nees.len());
    let _ = writeln!(
        out,
        "  mean_nees: {:.12}",
        s.nees.iter().sum::<f64>() / s.nees.len().max(1) as f64
    );
}

/// S4, S5 and the pipeline half of S6 on one recorded stream.
pub struct PipelineRun {
    pub graph: PoseGraph,
    pub truth: Vec<Se2>,
    pub dead: Vec<Se2>,
    pub outcomes: Vec<LoopOutcome>,
    pub registration_s: Vec<f64>,
    pub optimization_s: Vec<f64>,
    pub path_length: f64,
    pub lost_registrations: usize,
}

pub fn pipeline(frames: &[Frame], yaw_bias: f64, cfg: &Config) -> PipelineRun {
    let mut odo = LidarOdometry::new(cfg.odometry).unwrap();
    let mut graph = PoseGraph::new(cfg.graph).unwrap();
    let mut acc = EdgeAccumulator::new();
    let origin = frames[0].gt;
    let (mut truth, mut outcomes) = (Vec::new(), Vec::new());
    let (mut registration_s, mut optimization_s) = (Vec::new(), Vec::new());
    let mut lost = 0;
    for (k, fr) in frames.iter().enumerate() {
        let t0 = Instant::now();
        let step = odo.step(&fr.scan).unwrap();
        registration_s.push(t0.elapsed().as_secs_f64());
        if step.status == robotics_slam::odometry::TrackingStatus::Predicted {
            lost += 1;
        }
        if k == 0 {
            graph
                .add_keyframe(points(&fr.scan), &Se2::IDENTITY, &info(0.03, 0.01))
                .unwrap();
            truth.push(origin.between(&fr.gt));
            continue;
        }
        acc.push(&step.delta, &step.information).unwrap();
        if graph.wants_keyframe(&acc.delta()) {
            let (delta, chain_info) = acc.take().unwrap();
            let delta = delta.boxplus(&Vector3::new(0.0, 0.0, yaw_bias));
            // With an injected systematic error the compounded ICP information is overconfident
            // by construction (it knows nothing of the injection) and the gate rightly refuses
            // every loop. An injected run therefore hands the graph the information its injected
            // error warrants, as the integration tests do: 3 cm and 0.01 rad per edge. Declared in
            // benchmark.md section 6.
            let chain_info = if yaw_bias != 0.0 {
                info(0.03, 0.01)
            } else {
                chain_info
            };
            let id = graph
                .add_keyframe(points(&fr.scan), &delta, &chain_info)
                .unwrap();
            truth.push(origin.between(&fr.gt));
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
        if let FactorKind::Between {
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
    PipelineRun {
        graph,
        truth,
        dead,
        outcomes,
        registration_s,
        optimization_s,
        path_length,
        lost_registrations: lost,
    }
}

pub fn pipeline_bits(p: &PipelineRun) -> Vec<u64> {
    p.graph
        .poses()
        .iter()
        .flat_map(|q| [q.x().to_bits(), q.y().to_bits(), q.theta().to_bits()])
        .collect()
}

pub fn trajectory_block(label: &str, est: &[Se2], truth: &[Se2], path: f64, out: &mut String) {
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

/// The label of a loop constraint: wrong when its error against the ground-truth relative pose
/// exceeds these (the verdict script refuses any other limits).
pub const WRONG_M: f64 = 0.05;
pub const WRONG_RAD: f64 = 0.03;
/// A candidate is a true loop, for the recall only, when its keyframes were this close.
pub const TRUE_LOOP_M: f64 = 1.5;
pub const MIN_PATH_GAP_M: f64 = 6.0;

pub fn write_s4_s5(out: &mut String, a: &PipelineRun) {
    let _ = writeln!(out, "s4:");
    let _ = writeln!(out, "  keyframes_made: {}", a.graph.keyframes().len());
    let _ = writeln!(out, "  keyframes_graded: {}", a.truth.len());
    let _ = writeln!(out, "  path_length_m: {:.12}", a.path_length);
    trajectory_block("odometry", &a.dead, &a.truth, a.path_length, out);
    trajectory_block("graph", a.graph.poses(), &a.truth, a.path_length, out);

    let (mut ok, mut wrong, mut true_prop, mut true_acc) = (0, 0, 0, 0);
    let mut reasons: BTreeMap<&str, usize> = BTreeMap::new();
    let mut true_missed: BTreeMap<&str, usize> = BTreeMap::new();
    let mut accepted_to = std::collections::BTreeSet::new();
    for o in &a.outcomes {
        let c = &o.candidate;
        let gt = a.truth[c.from].between(&a.truth[c.to]);
        let true_loop = gt.translation().norm() < TRUE_LOOP_M;
        if true_loop {
            true_prop += 1;
        }
        match &o.verification {
            Verification::Accepted { measurement, .. } => {
                let e = gt.between(measurement).log();
                if e.xy().norm() < WRONG_M && e[2].abs() < WRONG_RAD {
                    ok += 1;
                } else {
                    wrong += 1;
                }
                if true_loop {
                    true_acc += 1;
                }
                accepted_to.insert(c.to);
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
                if true_loop {
                    *true_missed.entry(name).or_default() += 1;
                }
            }
        }
    }
    // Keyframes that had a true-loop partner by ground truth, whether or not the detector
    // proposed it, and how many of them ended up with an accepted loop edge.
    let arcs: Vec<f64> = a.graph.keyframes().iter().map(|k| k.arc_length).collect();
    let mut with_partner = 0;
    let mut with_partner_accepted = 0;
    for k in 0..a.truth.len() {
        let has = (0..k).any(|j| {
            arcs[k] - arcs[j] >= MIN_PATH_GAP_M && a.truth[j].distance_to(&a.truth[k]) < TRUE_LOOP_M
        });
        if has {
            with_partner += 1;
            if accepted_to.contains(&k) {
                with_partner_accepted += 1;
            }
        }
    }
    let names = [
        "no_overlap",
        "not_converged",
        "poor_fit",
        "degenerate",
        "gate_exceeded",
        "ambiguous",
    ];
    let _ = writeln!(out, "s5:");
    let _ = writeln!(out, "  candidates: {}", a.outcomes.len());
    let _ = writeln!(out, "  accepted_correct: {ok}");
    let _ = writeln!(out, "  accepted_wrong: {wrong}");
    let _ = writeln!(out, "  true_loops_proposed: {true_prop}");
    let _ = writeln!(out, "  true_loops_accepted: {true_acc}");
    let _ = writeln!(out, "  keyframes_with_true_partner: {with_partner}");
    let _ = writeln!(
        out,
        "  keyframes_with_accepted_loop: {with_partner_accepted}"
    );
    let _ = writeln!(out, "  wrong_constraint_error_m: {WRONG_M}");
    let _ = writeln!(out, "  wrong_constraint_error_rad: {WRONG_RAD}");
    let _ = writeln!(out, "  rejected_by_reason:");
    for n in names {
        let _ = writeln!(out, "    {n}: {}", reasons.get(n).copied().unwrap_or(0));
    }
    let _ = writeln!(out, "  true_loops_missed_by_reason:");
    for n in names {
        let _ = writeln!(out, "    {n}: {}", true_missed.get(n).copied().unwrap_or(0));
    }
}
