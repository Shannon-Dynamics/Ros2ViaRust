//! `mapping_evaluator` — grades the published map against the analytic geometry of the world.
//!
//! ```text
//!   nav_msgs/OccupancyGrid   (topics.map, latched)   ─┐
//!   geometry_msgs/PoseStamped (topics.ground_truth) ──┼─► at shutdown: robotics_mapping::evaluate
//!   /tf  base_link -> <laser frame>                  ─┘     ─► map-metrics.yaml, trajectory.csv,
//!                                                              final-map-states.txt
//! ```
//!
//! Like `localization_evaluator` in Phase 3, this node is allowed to read ground truth, and the
//! mapper is not: it needs the trajectory to decide which surfaces were observable (an exact
//! visibility test over every pose of the run, so that the map is not charged for what it
//! could not have seen), and the world's geometry from the same file the world was built
//! around (`sim/webots/config/landmarks.yaml`). `mapping_node` never sees either.
//!
//! It grades the **published** map, that is what a consumer would get: the three states of
//! `nav_msgs/OccupancyGrid`, not the log-odds inside the mapper. The exact grid is a separate
//! output of `mapping_node` (`output.final_grid`).
//!
//! The metrics are computed once, at shutdown, from the last map and every lidar position the
//! run had (a position is kept when it is at least `trajectory.min_step_m` from the last kept
//! one). The tolerance `tau_cells` is an input: it is fixed in `scripts/phase4_verdict.py`,
//! which refuses a run whose metrics carry a different one.

use rclrs::*;
use robotics_core::nalgebra::Vector2;
use robotics_core::tf::{LookupTime, TransformBuffer, TransformBufferConfig};
use robotics_core::{FrameId, Se2};
use robotics_mapping::{
    evaluate, EvalParams, GridGeometry, GroundTruth, OccupancyGrid, OccupancyParams,
};
use robotics_ros_bridge::convert::pose_from_ros;
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::tf::TfListener;
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::{geometry_msgs, nav_msgs};
use std::io::Write;
use std::sync::Arc;

struct LatestMap {
    geometry: GridGeometry,
    data: Vec<i8>,
    stamp: f64,
}

struct State {
    map: Option<LatestMap>,
    maps_received: u64,
    positions: Vec<Vector2<f64>>,
    min_step: f64,
    gt_received: u64,
    laser_mount: Option<Se2>,
    base_frame: FrameId,
    laser_frame: FrameId,
    buffer: Shared<TransformBuffer>,
    rejected: u64,
}

fn invalid(node: &Node, what: &str) -> RclrsError {
    log_error!(node.logger(), "{what}");
    RclrsError::RclError {
        code: RclReturnCode::InvalidArgument,
        msg: None,
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("mapping_evaluator")?;
    let mut p = ParamStore::new();

    let map_topic = p.string(&node, "topics.map", "map", "Occupancy grid to grade")?;
    let gt_topic = p.string(
        &node,
        "topics.ground_truth",
        "ground_truth/pose",
        "Ground truth pose of base_link (geometry_msgs/PoseStamped)",
    )?;
    let base_frame = p.string(&node, "frames.base_link", "base_link", "Body frame")?;
    let laser_frame = p.string(
        &node,
        "frames.laser",
        "LDS-01",
        "Laser frame, looked up in TF",
    )?;
    let map_file = p.string(
        &node,
        "map_file",
        "",
        "Path to landmarks.yaml, the ground truth of the world (required)",
    )?;
    let output_dir = p.string(
        &node,
        "output_dir",
        "",
        "Directory the results are written to (required)",
    )?;
    let tau_cells = p.f64(
        &node,
        "tau_cells",
        1.0,
        0.01,
        100.0,
        "Tolerance in cells; fixed in scripts/phase4_verdict.py, which refuses a different one",
    )?;
    let spacing = p.f64(
        &node,
        "sample_spacing_m",
        0.01,
        1e-4,
        1.0,
        "Spacing of the ground-truth surface samples (m)",
    )?;
    let max_range = p.f64(
        &node,
        "max_range_m",
        3.5,
        0.1,
        100.0,
        "Sensor range limit (m)",
    )?;
    let stride = p.i64(
        &node,
        "coverage_stride",
        10,
        1,
        10_000,
        "Use every n-th position for the coverage test",
    )? as usize;
    let min_step = p.f64(
        &node,
        "trajectory.min_step_m",
        0.02,
        0.0,
        10.0,
        "Keep a lidar position only if it moved this far (m)",
    )?;
    let occ_p_hit = p.f64(
        &node,
        "model.p_hit",
        0.7,
        1e-3,
        0.999,
        "Must equal the mapper's, so states read back as themselves",
    )?;
    let occ_p_miss = p.f64(
        &node,
        "model.p_miss",
        0.4,
        1e-3,
        0.999,
        "Must equal the mapper's",
    )?;
    let p_min = p.f64(
        &node,
        "model.p_min",
        0.12,
        1e-3,
        0.999,
        "Must equal the mapper's",
    )?;
    let p_max = p.f64(
        &node,
        "model.p_max",
        0.97,
        1e-3,
        0.999,
        "Must equal the mapper's",
    )?;
    let p_occupied = p.f64(
        &node,
        "model.p_occupied",
        0.65,
        1e-3,
        0.999,
        "Must equal the mapper's",
    )?;
    let p_free = p.f64(
        &node,
        "model.p_free",
        0.35,
        1e-3,
        0.999,
        "Must equal the mapper's",
    )?;

    if map_file.is_empty() || output_dir.is_empty() {
        return Err(invalid(&node, "map_file and output_dir are required"));
    }
    let text = std::fs::read_to_string(&map_file)
        .map_err(|e| invalid(&node, &format!("cannot read {map_file}: {e}")))?;
    let truth = GroundTruth::from_landmarks_yaml(&text)
        .map_err(|e| invalid(&node, &format!("{map_file}: {e}")))?;
    std::fs::create_dir_all(&output_dir)
        .map_err(|e| invalid(&node, &format!("cannot create {output_dir}: {e}")))?;
    let params = OccupancyParams::from_probabilities(
        occ_p_hit, occ_p_miss, p_min, p_max, p_occupied, p_free,
    );

    let buffer = Shared::new(
        TransformBuffer::new(TransformBufferConfig::default()).expect("default TF buffer config"),
    );
    let _tf_listener = TfListener::new(&node, buffer.clone())?;
    let shared = Shared::new(State {
        map: None,
        maps_received: 0,
        positions: Vec::new(),
        min_step,
        gt_received: 0,
        laser_mount: None,
        base_frame: FrameId::new(&base_frame),
        laser_frame: FrameId::new(&laser_frame),
        buffer,
        rejected: 0,
    });

    let _map_sub = node.create_subscription::<nav_msgs::msg::OccupancyGrid, _>(
        map_topic.as_str().qos(qos::map_latched()),
        {
            let shared = shared.clone();
            move |msg: nav_msgs::msg::OccupancyGrid| {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                st.maps_received += 1;
                // The origin pose is axis aligned in this stack; a rotated map is refused.
                let o = &msg.info.origin;
                let yaw_ok =
                    o.orientation.x == 0.0 && o.orientation.y == 0.0 && o.orientation.z == 0.0;
                if !yaw_ok {
                    st.rejected += 1;
                    return;
                }
                st.map = Some(LatestMap {
                    geometry: GridGeometry {
                        width: msg.info.width as usize,
                        height: msg.info.height as usize,
                        resolution: msg.info.resolution as f64,
                        origin: Vector2::new(o.position.x, o.position.y),
                    },
                    data: msg.data.clone(),
                    stamp: msg.header.stamp.sec as f64 + msg.header.stamp.nanosec as f64 * 1e-9,
                });
            }
        },
    )?;

    let n = Arc::clone(&node);
    let _gt_sub = node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
        gt_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |msg: geometry_msgs::msg::PoseStamped| {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                st.gt_received += 1;
                let Ok(pose) = pose_from_ros(&msg.pose) else {
                    st.rejected += 1;
                    return;
                };
                if st.laser_mount.is_none() {
                    // A static transform: the first successful lookup is the mount.
                    if let Ok(t) = st.buffer.lock().lookup_transform(
                        &st.base_frame,
                        &st.laser_frame,
                        LookupTime::Latest,
                    ) {
                        st.laser_mount = Some(t.transform);
                        log_info!(
                            n.logger(),
                            "laser mount from TF: ({:.3}, {:.3}, {:.3})",
                            t.transform.x(),
                            t.transform.y(),
                            t.transform.theta()
                        );
                    } else {
                        return; // no mount yet: a position without it would be the wrong point
                    }
                }
                let mount = st.laser_mount.expect("set above");
                let at = pose.compose(&mount);
                let here = Vector2::new(at.x(), at.y());
                let keep = st
                    .positions
                    .last()
                    .is_none_or(|last| (here - last).norm() >= st.min_step);
                if keep {
                    st.positions.push(here);
                }
            }
        },
    )?;

    log_info!(
        node.logger(),
        "mapping_evaluator up: grading {} against {} ({} pillars), tau {} cells, results to {}, {} parameters",
        map_topic,
        map_file,
        truth.pillar_count(),
        tau_cells,
        output_dir,
        p.len()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);

    // Grade once, from the last map and every position of the run.
    let guard = shared.lock();
    let st: &State = &guard;
    let dir = std::path::Path::new(&output_dir);
    let mut summary = String::new();
    summary.push_str(&format!("maps_received: {}\n", st.maps_received));
    summary.push_str(&format!("ground_truth_received: {}\n", st.gt_received));
    summary.push_str(&format!("positions_kept: {}\n", st.positions.len()));
    summary.push_str(&format!("rejected: {}\n", st.rejected));
    match (&st.map, st.positions.is_empty()) {
        (Some(m), false) => {
            let grid = OccupancyGrid::from_ros_data(m.geometry, params, &m.data)
                .map_err(|e| invalid(&node, &format!("the published map: {e}")))?;
            let metrics = evaluate(
                &grid,
                &truth,
                &st.positions,
                &EvalParams {
                    tau: tau_cells * m.geometry.resolution,
                    sample_spacing: spacing,
                    max_range,
                    coverage_stride: stride,
                },
            )
            .map_err(|e| invalid(&node, &format!("evaluation: {e}")))?;
            summary.push_str(&format!("map_stamp_s: {}\n", m.stamp));
            let write = |name: &str, body: String| {
                if let Err(e) = std::fs::write(dir.join(name), body) {
                    log_error!(node.logger(), "cannot write {name}: {e}");
                }
            };
            write(
                "map-metrics.yaml",
                format!("{summary}{}", metrics.to_yaml()),
            );
            write("final-map-states.txt", grid.to_text());
            let mut traj = String::from("x,y\n");
            for q in &st.positions {
                traj.push_str(&format!("{},{}\n", q.x, q.y));
            }
            write("trajectory.csv", traj);
            log_info!(
                node.logger(),
                "wrote map-metrics.yaml, final-map-states.txt, trajectory.csv to {output_dir}"
            );
        }
        _ => {
            summary
                .push_str("error: no map or no lidar position was received, nothing was graded\n");
            let _ = std::fs::File::create(dir.join("map-metrics.yaml"))
                .and_then(|mut f| f.write_all(summary.as_bytes()));
            log_error!(
                node.logger(),
                "nothing to grade: map received {}, positions {}",
                st.map.is_some(),
                st.positions.len()
            );
        }
    }
    drop(guard);
    drop(p);
    result
}
