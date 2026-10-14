//! The in-process reference of the two SLAM nodes' replay test: the same library calls on the
//! same recorded scan file, no ROS, writing the same two CSVs the nodes write.
//!
//! ```text
//! cargo run --release -p robotics_slam --example slam_replay_reference -- --scans scans.csv --out DIR
//! ```
//!
//! writes `DIR/registrations.csv` (what `scan_matcher_node` writes with
//! `output.registrations_csv`, without the latency column) and `DIR/keyframes.csv` (what
//! `pose_graph_node` writes with `output.keyframes_csv`). `scripts/verify_phase5_slam_nodes.sh`
//! compares them with the nodes' files. The only deliberate step that mimics the ROS path is the
//! rounding of the scan metadata (`angle_min`, `angle_increment`, `range_min`, `range_max`) to `f32`
//! and back, because a `sensor_msgs/LaserScan` carries them as `float32`; without it the reference
//! would see bearings that differ from the nodes' by about 1e-7 rad. Ranges are `f32` already.
//!
//! `--f64-metadata` skips that rounding: the reference then sees the file's float64 metadata, exactly
//! as `slam_synthetic_run` did when it made the graded evidence run of the same stream, and must
//! reproduce that run's front-end-alone trajectory to the digits of its metrics file (check E1 of
//! `scripts/phase5_node_replay_check.py`). The difference between the two modes is the effect of
//! `sensor_msgs/LaserScan`'s `float32` fields on the front end, and nothing else.
//!
//! Like the nodes it uses `OdometryConfig::default()` and `PoseGraphConfig::default()`, builds
//! keyframes and odometry edges only, and never calls the loop detector or the verifier.

mod common;

use common::*;
use robotics_core::Se2;
use robotics_slam::odometry::{EdgeAccumulator, LidarOdometry, OdometryConfig};
use robotics_slam::pose_graph::{PoseGraph, PoseGraphConfig};
use std::fmt::Write as _;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |n: &str| {
        args.iter()
            .position(|a| a == n)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let (Some(scans), Some(out)) = (get("--scans"), get("--out")) else {
        eprintln!("usage: slam_replay_reference --scans scans.csv --out DIR");
        std::process::exit(2);
    };
    let mut frames = read_frames(&scans).expect("read scans");
    let exact = args.iter().any(|a| a == "--f64-metadata");
    for f in frames.iter_mut().filter(|_| !exact) {
        let s = &mut f.scan;
        s.angle_min = s.angle_min as f32 as f64;
        s.angle_increment = s.angle_increment as f32 as f64;
        s.range_min = s.range_min as f32 as f64;
        s.range_max = s.range_max as f32 as f64;
    }
    std::fs::create_dir_all(&out).expect("create --out");

    let mut odo = LidarOdometry::new(OdometryConfig::default()).unwrap();
    let mut graph = PoseGraph::new(PoseGraphConfig::default()).unwrap();
    let mut acc = EdgeAccumulator::new();
    let mut regs = String::from(
        "stamp_ns,status,retried,threshold_m,iterations,correspondences,rmse_m,degenerate,dx,dy,dtheta\n",
    );
    let mut kfs = String::from("id,stamp_ns,x,y,theta,arc_length_m\n");
    let mut started = false;
    for fr in &frames {
        let stamp_ns = robotics_core::Timestamp::from_secs_f64(fr.stamp).as_nanos();
        let Ok(step) = odo.step(&fr.scan) else {
            continue; // too few points: the node does not publish a registration either
        };
        let (it, corr, rmse, deg) = step
            .icp
            .map(|i| (i.iterations, i.correspondences, i.rmse, i.degenerate))
            .unwrap_or((0, 0, 0.0, false));
        let _ = writeln!(
            regs,
            "{stamp_ns},{:?},{},{},{it},{corr},{rmse},{deg},{},{},{}",
            step.status,
            step.retried,
            step.threshold,
            step.delta.x(),
            step.delta.y(),
            step.delta.theta()
        );
        // The same error handling as the pose graph node: an edge the library refuses is skipped.
        let new = if !started {
            started = true;
            graph
                .add_keyframe(
                    points(&fr.scan),
                    &Se2::IDENTITY,
                    &nalgebra::Matrix3::identity(),
                )
                .is_ok()
        } else if acc.push(&step.delta, &step.information).is_err() {
            false
        } else if graph.wants_keyframe(&acc.delta()) {
            match acc.take() {
                Ok((d, i)) => graph.add_keyframe(points(&fr.scan), &d, &i).is_ok(),
                Err(_) => false,
            }
        } else {
            false
        };
        if new {
            let id = graph.keyframes().len() - 1;
            let p = graph.poses()[id];
            let _ = writeln!(
                kfs,
                "{id},{stamp_ns},{},{},{},{}",
                p.x(),
                p.y(),
                p.theta(),
                graph.keyframes()[id].arc_length
            );
        }
    }
    std::fs::write(format!("{out}/registrations.csv"), regs).unwrap();
    std::fs::write(format!("{out}/keyframes.csv"), kfs).unwrap();
    println!(
        "{} scans, {} keyframes, loop edges {}",
        frames.len(),
        graph.keyframes().len(),
        graph.loop_edge_count()
    );
}
