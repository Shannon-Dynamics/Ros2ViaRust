//! The Webots half of claim S1a, S2 and the replay of S6, and, **for a recording that has turning
//! scans**, S4 and S5: a recorded scan stream with ground-truth sensor poses in, `slam-metrics.yaml`
//! out.
//!
//! ```text
//! cargo run --release -p robotics_slam --example slam_recorded_run -- \
//!     --scans evidence/phase-4-mapping/2026-10-11-development-3-route-long/integrated-scans.csv \
//!     --out DIR [--role development] [--info-inflation K]
//!
//! cargo run --release -p robotics_slam --example slam_recorded_run -- \
//!     --scans evidence/phase-5-slam/2026-10-14-webots-rectangle-recording/integrated-scans.csv \
//!     --out DIR --role evaluation --pipeline --emulate-motion-gate
//! ```
//!
//! `--pipeline` also runs the front end, the pose graph and the loop closure over the whole stream
//! (the same `pipeline` function the synthetic harness grades S4 and S5 with) and writes the `s4` and
//! `s5` sections and the pipeline half of `s6`. It is for a stream with turning scans
//! (`scripts/run_phase5_record.sh`); on a stream without them the front end loses track and the
//! sections would measure that.
//!
//! `--emulate-motion-gate` restricts S1a to the pairs the Phase 4 mapper would have admitted had it
//! been recording: both scans move at most 0.05 m and turn at most 0.015 rad since the previous
//! recorded scan (about the 0.2 s window of the mapper's gate). Without it S1a is over every
//! consecutive pair at most 0.5 s apart. `--turning-report FILE` additionally registers the pairs
//! the gate would have refused (heading step above 0.015 rad) and writes how well they agree with
//! the recorded ground truth to FILE, outside the metrics file: it is **not graded**, and it is the
//! measurement of how good the recorded pose of a turning scan is.
//!
//! The input is the `integrated-scans.csv` that Phase 4's `mapping_node` wrote: the scans it
//! integrated, each with the ground-truth sensor pose it integrated them at. Two facts about that
//! file decide what this run can say.
//!
//! * The pose of each scan is the ground truth of `map -> base_link` at the scan's stamp less the
//!   0.06 s mean lag of the Webots lidar (Phase 4, D15), and the mapper only integrated scans whose
//!   sensor moved at most 5 cm and 0.015 rad across the window the stamp leaves open (D13). So
//!   every scan in the file already passed the motion gate: that is the "motion-gated ground-truth
//!   join" of problem.md section 4.2, done by Phase 4 and read here, not rebuilt. The gate is a
//!   function of poses; no occupancy-grid value is read, and `robotics_mapping` is not a dependency
//!   of this program.
//! * Scans the mapper skipped (the turns, and stationary repeats) are not in the file. A registration
//!   is therefore only made between consecutive lines at most `MAX_GAP_S` apart (0.5 s, two and a
//!   half scan periods, fixed before the first run); a longer gap is a turn the file does not contain,
//!   and no pair is formed across it. How many pairs were made is reported.
//!
//! The pipeline half of S4 and S5 cannot be run on a stream from the Phase 4 mapper: the front end
//! needs the scans of the turns, which the file does not have. That is reported by leaving those
//! sections out, and they are only written with `--pipeline`.

mod common;

use common::*;
use std::fmt::Write as _;

const MAX_GAP_S: f64 = 0.5;
/// The Phase 4 mapper's motion gate (`robotics_mapping::MotionGate`, `motion_gate.max_*`).
const GATE_TRANSLATION_M: f64 = 0.05;
const GATE_YAW_RAD: f64 = 0.015;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |name: &str, default: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
            .unwrap_or_else(|| default.to_string())
    };
    let (scans, out_dir) = (get("--scans", ""), get("--out", ""));
    if scans.is_empty() || out_dir.is_empty() {
        eprintln!("usage: slam_recorded_run --scans integrated-scans.csv --out DIR [--role development|evaluation] [--info-inflation K]");
        std::process::exit(2);
    }
    let role = get("--role", "development");
    let inflation: Option<f64> = args
        .iter()
        .position(|a| a == "--info-inflation")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.parse().expect("--info-inflation"));
    let has = |flag: &str| args.iter().any(|a| a == flag);
    let (pipeline_run, emulate_gate) = (has("--pipeline"), has("--emulate-motion-gate"));
    let turning_report = get("--turning-report", "");
    let frames = read_frames(&scans).expect("read scans");
    let cfg = config(inflation);
    std::fs::create_dir_all(&out_dir).expect("create --out");

    // The mapper's gate, from the recorded poses: a scan is admitted if the sensor moved at most
    // 0.05 m and turned at most 0.015 rad since the previous recorded scan (the first scan is).
    let gated: std::collections::BTreeSet<u64> = frames
        .iter()
        .enumerate()
        .filter(|(k, f)| {
            *k == 0 || {
                let d = frames[k - 1].gt.between(&f.gt);
                d.translation().norm() <= GATE_TRANSLATION_M && d.theta().abs() <= GATE_YAW_RAD
            }
        })
        .map(|(_, f)| f.stamp.to_bits())
        .collect();
    let pair = |a: &Frame, b: &Frame| b.stamp - a.stamp <= MAX_GAP_S && b.stamp > a.stamp;
    let eligible = |a: &Frame, b: &Frame| {
        pair(a, b)
            && (!emulate_gate
                || (gated.contains(&a.stamp.to_bits()) && gated.contains(&b.stamp.to_bits())))
    };
    let first = s1a_pass(&frames, eligible, &cfg.s1a_icp);
    let second = s1a_pass(&frames, eligible, &cfg.s1a_icp);
    let mut identical = first.bits == second.bits;
    if !turning_report.is_empty() {
        let turning =
            |a: &Frame, b: &Frame| pair(a, b) && a.gt.between(&b.gt).theta().abs() > GATE_YAW_RAD;
        let t = s1a_pass(&frames, turning, &cfg.s1a_icp);
        let mut r = String::new();
        let _ = writeln!(
            r,
            "# S1a-style comparison over the PAIRS A MOTION GATE WOULD HAVE REFUSED (recorded heading step above {GATE_YAW_RAD} rad). NOT GRADED."
        );
        let _ = writeln!(
            r,
            "# What it measures: how far the front end's registration of two scans taken while turning is from the"
        );
        let _ = writeln!(
            r,
            "# recorded ground-truth relative pose. The registration does not use the recorded poses; if it is accurate"
        );
        let _ = writeln!(
            r,
            "# (S1a on gated pairs: millimetres), the disagreement here is the recorded pose's error at those scans."
        );
        let _ = writeln!(r, "turning_pairs_registered: {}", t.made);
        let _ = writeln!(r, "turning_pairs_settled: {}", t.translation_error.len());
        stats("translation_error_m", &t.translation_error, &mut r, "");
        stats("rotation_error_rad", &t.rotation_error, &mut r, "");
        std::fs::write(&turning_report, r).expect("write --turning-report");
    }

    let travelled: f64 = frames
        .windows(2)
        .map(|w| w[0].gt.distance_to(&w[1].gt))
        .sum();
    let mut out = String::new();
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let name = std::path::Path::new(&scans)
        .parent()
        .and_then(|p| p.file_name())
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "recording".into());
    let _ = writeln!(out, "schema: phase5-slam-metrics/1");
    let _ = writeln!(out, "run:");
    let _ = writeln!(out, "  data_source: webots_recorded");
    let _ = writeln!(out, "  ground_truth: webots_gated");
    let _ = writeln!(out, "  role: {role}");
    let _ = writeln!(out, "  seed: 0");
    let _ = writeln!(
        out,
        "  route: {name}/integrated-scans.csv max_gap_s={MAX_GAP_S}"
    );
    let _ = writeln!(out, "  injected_yaw_bias_rad_per_keyframe: 0.0");
    let _ = writeln!(
        out,
        "# harness flags: pipeline {pipeline_run}, emulate-motion-gate {emulate_gate}; S1a pairs made {}",
        first.made
    );
    let _ = writeln!(out, "  info_inflation: {:.12}", cfg.info_inflation);
    let _ = writeln!(out, "  robotics_mapping_dependency: false");
    let _ = writeln!(out, "  build_profile: {profile}");
    let _ = writeln!(
        out,
        "# path travelled by the recorded sensor poses: {travelled:.3} m over {} scans",
        frames.len()
    );
    write_s1a_s2(&mut out, &first);
    // The pipeline half: the front end, the graph and the loop closure on every scan of the stream.
    let (registration_s, optimization_s);
    if pipeline_run {
        let a = pipeline(&frames, 0.0, &cfg);
        let b = pipeline(&frames, 0.0, &cfg);
        identical = identical && pipeline_bits(&a) == pipeline_bits(&b);
        let _ = writeln!(
            out,
            "# front end: {} of {} registrations lost track (status Predicted)",
            a.lost_registrations,
            frames.len()
        );
        write_s4_s5(&mut out, &a);
        registration_s = a.registration_s;
        optimization_s = a.optimization_s;
    } else {
        registration_s = first.seconds.clone();
        optimization_s = Vec::new();
    }
    let _ = writeln!(out, "s6:");
    let _ = writeln!(out, "  replay_identical: {identical}");
    let _ = writeln!(out, "  scan_period_s: 0.2");
    stats("registration_latency_s", &registration_s, &mut out, "  ");
    stats("optimization_latency_s", &optimization_s, &mut out, "  ");
    std::fs::write(format!("{out_dir}/slam-metrics.yaml"), &out).expect("write metrics");
    print!("{out}");
}
