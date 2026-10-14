//! `pose_graph_node` — keyframes, odometry edges, optional loop closure, a rendered `/map` and
//! `map -> odom` (Phase 5).
//!
//! ```text
//!   registration (PoseWithCovarianceStamped) ─┐
//!                                              ├─ joined by stamp ─► GraphBuilder ─► robotics_slam::PoseGraph
//!   scan_matched (LaserScan, reliable) ───────┘                       │
//!        slam/pose (PoseWithCovarianceStamped, newest keyframe, marginal covariance)  ◄──┤
//!        slam/path (nav_msgs/Path, every keyframe, latched; republished whole after a    │
//!                   loop closure, since the closure moves them)                       ◄──┤
//!        /map (nav_msgs/OccupancyGrid, latched)   rendered by replay, DS17            ◄──┤
//!        /tf  map -> odom   (this node is the owner of that edge)                     ◄──┤
//!        /diagnostics  (including  loop_closure: unsupported | enabled)               ◄──┘
//! ```
//!
//! **Loop closure is off unless a parameter turns it on** (`loop_closure.enabled`, default
//! `false`). Off, this node never calls the detector or the verifier and says
//! `loop_closure: unsupported` in every diagnostics message, as it did before the parameter
//! existed; on, it runs `robotics_slam`'s detector and verifier on every new keyframe, adds the
//! accepted loops and optimizes, and says `loop_closure: enabled`. Whether it should ever be the
//! default is the owner's decision of `docs/phase-5-slam/problem.md` section 5, taken on the
//! evidence of claim S5; this file does not take it.
//!
//! **The map** (`map.enabled`, default `true`). Decision DS17: the map is a *replay*. The node
//! keeps every keyframe's scan and, after a loop closure that changed the graph and once more at
//! the end of a run, integrates all of them at their current optimized poses into a fresh Phase 4
//! `OccupancyGrid` with Phase 4's `ScanIntegrator` ([`robotics_slam_ros::render`]). Nothing is
//! repaired incrementally. `map.render_period_s` > 0 additionally renders every that many seconds
//! while keyframes are being added (off by default; not part of DS17, a convenience for a viewer).
//!
//! **`map -> odom`** (`tf.publish_map_odom`, default `true`). The node is the owner of that edge
//! (the precedent is `ground_truth_tf_adapter`, which owns it for Phase 4 and must not run
//! together with this node: `TfListener`'s authority-conflict detection is in the diagnostics).
//! `map` is the frame of the *first keyframe's sensor pose* (the graph's gauge), not the Webots
//! world's `map`: a SLAM estimate has no world to be anchored in. For every registered scan the
//! node composes its best estimate of the sensor pose in `map` (the newest keyframe's pose with the
//! increments since it) with the inverse of `odom -> <scan frame>` looked up at the instant the
//! scan was measured (`scan_time_offset_s` after its stamp), which gives `map -> odom`:
//! `T_map_odom = T_map_sensor * T_odom_sensor^-1`. `odom` is whatever the TF tree says it is
//! (the Webots driver's `odom -> base_link`); nothing here reads `/odom`. When that lookup is not
//! available yet the estimate waits, and one that never gets it is dropped and counted; the node
//! never publishes a `map -> odom` it could not compute. The resulting tree is
//! `map -> odom -> base_link -> <sensor>`.
//!
//! The poses are published in `frames.slam`: `auto` (the default) means `frames.map` when the map
//! or `map -> odom` is on and `slam_origin` when neither is, which is the node as it was.
//!
//! Optional files, off unless a path is given: `output.keyframes_csv` (one row per keyframe, as
//! created), `output.keyframes_final_csv` (the optimized poses, written at shutdown),
//! `output.loops_csv` (every loop candidate and the verifier's verdict), `output.final_map` (the
//! grid rendered at shutdown, in `robotics_mapping`'s text format).

use rclrs::*;
use robotics_core::tf::{LookupTime, TransformBuffer, TransformBufferConfig};
use robotics_core::{FrameId, Se2, Timestamp};
use robotics_ros_bridge::convert::{
    header, laser_scan_from_ros, now, pose_to_ros, pose_with_covariance_stamped_to_ros,
};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::tf::{TfBroadcaster, TfListener};
use robotics_ros_bridge::{qos, ParamStore};
use robotics_slam::icp::scan_points;
use robotics_slam::pose_graph::Verification;
use robotics_slam::voxel::downsample;
use robotics_slam_ros::render::{render_map, RenderConfig};
use robotics_slam_ros::{
    registration_from_ros, GraphBuilder, GraphBuilderConfig, LoopEvent, NewKeyframe,
    LOOP_CLOSURE_ENABLED_REASON, LOOP_CLOSURE_REASON,
};
use ros_env::{builtin_interfaces, geometry_msgs, nav_msgs, sensor_msgs};
use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

struct State {
    builder: GraphBuilder,
    frame: FrameId,
    map_frame: FrameId,
    odom_frame: FrameId,
    downsample: f64,
    scans_malformed: u64,
    registrations_malformed: u64,
    last_error: String,
    path: nav_msgs::msg::Path,
    pose_pub: Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>,
    path_pub: Publisher<nav_msgs::msg::Path>,
    csv: Option<BufWriter<File>>,
    csv_rows: u64,
    loops_csv: Option<BufWriter<File>>,
    /// The epoch of `builder.pose_epoch()` the published path corresponds to.
    path_epoch: u64,
    // The map.
    map_pub: Option<Publisher<nav_msgs::msg::OccupancyGrid>>,
    render_cfg: RenderConfig,
    maps_published: u64,
    last_render_keyframes: usize,
    last_render_note: String,
    // map -> odom.
    tf: Option<TfBroadcaster>,
    tf_buffer: Shared<TransformBuffer>,
    scan_frame: Option<FrameId>,
    time_offset: f64,
    max_wait: f64,
    /// (instant to look `odom -> sensor` up at, estimate of the sensor in `map`, when it was queued).
    /// The wait is wall-clock time: stamp time runs as fast as the replay or the simulator does.
    pending_tf: VecDeque<(Timestamp, Se2, std::time::Instant)>,
    newest_stamp: Option<Timestamp>,
    map_odom_published: u64,
    map_odom_dropped: u64,
}

fn stamp_of(ns: i64) -> Timestamp {
    Timestamp::from_nanos(ns)
}

impl State {
    /// Publish the new keyframes, or the whole path again when an optimization moved the old ones.
    fn publish_keyframes(&mut self, new: Vec<NewKeyframe>) {
        let epoch = self.builder.pose_epoch();
        if epoch != self.path_epoch {
            self.path_epoch = epoch;
            self.republish_path();
        }
        for kf in new {
            let stamp = stamp_of(kf.stamp_ns);
            // After an optimization the keyframe's pose is the optimized one, not what `kf` carried.
            let pose = self
                .builder
                .keyframes()
                .0
                .get(kf.id)
                .copied()
                .unwrap_or(kf.pose);
            if self.path.poses.len() <= kf.id {
                self.path.header = header(stamp, &self.frame);
                self.path.poses.push(geometry_msgs::msg::PoseStamped {
                    header: header(stamp, &self.frame),
                    pose: pose_to_ros(&pose),
                });
            }
            let msg = match self.builder.newest_covariance() {
                Some(cov) => pose_with_covariance_stamped_to_ros(stamp, &self.frame, &pose, &cov),
                None => {
                    // The message convention for "covariance unknown": a leading -1.
                    let mut m = pose_with_covariance_stamped_to_ros(
                        stamp,
                        &self.frame,
                        &pose,
                        &robotics_core::nalgebra::Matrix3::zeros(),
                    );
                    m.pose.covariance[0] = -1.0;
                    m
                }
            };
            if let Err(e) = self
                .pose_pub
                .publish(&msg)
                .and_then(|_| self.path_pub.publish(&self.path))
            {
                self.last_error = format!("publish: {e}");
            }
            if let Some(w) = self.csv.as_mut() {
                let _ = writeln!(
                    w,
                    "{},{},{},{},{},{}",
                    kf.id,
                    kf.stamp_ns,
                    kf.pose.x(),
                    kf.pose.y(),
                    kf.pose.theta(),
                    kf.arc_length
                );
                self.csv_rows += 1;
                if self.csv_rows % 50 == 0 {
                    let _ = w.flush();
                }
            }
        }
    }

    /// An optimization moved every keyframe: rebuild the path from the graph and publish it.
    fn republish_path(&mut self) {
        let (poses, stamps) = self.builder.keyframes();
        let mut path = nav_msgs::msg::Path::default();
        for (pose, ns) in poses.iter().zip(stamps) {
            let stamp = stamp_of(*ns);
            path.header = header(stamp, &self.frame);
            path.poses.push(geometry_msgs::msg::PoseStamped {
                header: header(stamp, &self.frame),
                pose: pose_to_ros(pose),
            });
        }
        self.path = path;
        if let Err(e) = self.path_pub.publish(&self.path) {
            self.last_error = format!("path publish: {e}");
        }
    }

    /// Write the verifier's verdict on every candidate of a keyframe and say what it accepted.
    fn record_loops(&mut self, node: &Node, events: &[LoopEvent]) {
        for e in events {
            for o in &e.outcomes {
                let c = &o.candidate;
                if let Some(w) = self.loops_csv.as_mut() {
                    let row = match &o.verification {
                        Verification::Accepted {
                            measurement, chi2, ..
                        } => format!(
                            "{},{},{},accepted,,{chi2},{},{},{},{},{}",
                            c.from,
                            c.to,
                            e.stamp_ns,
                            c.distance,
                            c.sigma_position,
                            measurement.x(),
                            measurement.y(),
                            measurement.theta()
                        ),
                        Verification::Rejected { reason, .. } => format!(
                            "{},{},{},rejected,{:?},,{},{},,,",
                            c.from, c.to, e.stamp_ns, reason, c.distance, c.sigma_position
                        ),
                    };
                    let _ = writeln!(w, "{row}");
                }
            }
            if e.accepted() > 0 {
                log_info!(
                    node.logger(),
                    "loop closure at keyframe {}: {} of {} candidate(s) accepted, graph optimized ({} loop edges)",
                    e.keyframe,
                    e.accepted(),
                    e.outcomes.len(),
                    self.builder.loop_edge_count()
                );
            }
        }
        if let Some(w) = self.loops_csv.as_mut() {
            let _ = w.flush();
        }
    }

    /// Render the whole map again from the keyframes' scans at their current poses (DS17) and
    /// publish it. Returns the grid's text form for a caller that writes a file.
    fn render_and_publish(&mut self, node: &Node, why: &str) -> Option<String> {
        let (poses, stamps) = self.builder.keyframes();
        let stamp = stamps.last().copied().map(stamp_of)?;
        let started = std::time::Instant::now();
        let scans = self.builder.keyframe_scans();
        let (grid, report) = match render_map(poses, &scans, &self.render_cfg) {
            Ok(x) => x,
            Err(e) => {
                self.last_error = format!("render: {e}");
                return None;
            }
        };
        let g = *grid.geometry();
        self.last_render_keyframes = poses.len();
        self.last_render_note = format!(
            "{why}: {} of {} keyframes integrated ({} without a stored scan, {} rejected) in {:.0} ms",
            report.integrated,
            poses.len(),
            report.skipped_no_scan,
            report.skipped_rejected,
            started.elapsed().as_secs_f64() * 1e3
        );
        if report.integrated < poses.len() {
            // A map from fewer scans than keyframes is wrong in a way no one would see: say so.
            self.last_error = format!("render skipped keyframes: {}", self.last_render_note);
            log_warn!(
                node.logger().throttle(Duration::from_secs(5)),
                "{}",
                self.last_error
            );
        }
        if let Some(p) = self.map_pub.as_ref() {
            let msg = nav_msgs::msg::OccupancyGrid {
                header: header(stamp, &self.map_frame),
                info: nav_msgs::msg::MapMetaData {
                    map_load_time: builtin_interfaces::msg::Time::default(),
                    resolution: g.resolution as f32,
                    width: g.width as u32,
                    height: g.height as u32,
                    origin: pose_to_ros(&Se2::new(g.origin.x, g.origin.y, 0.0)),
                },
                data: grid.to_ros_data(),
            };
            match p.publish(msg) {
                Ok(()) => self.maps_published += 1,
                Err(e) => self.last_error = format!("map publish: {e}"),
            }
        }
        log_info!(node.logger(), "map rendered, {}", self.last_render_note);
        Some(grid.to_text())
    }

    /// Queue the estimate of the sensor pose at the newest registered scan for `map -> odom`.
    fn queue_map_odom(&mut self) {
        if self.tf.is_none() {
            return;
        }
        let Some((key, est)) = self.builder.current_estimate() else {
            return;
        };
        let stamp = stamp_of(key);
        if self.newest_stamp.is_some_and(|n| n >= stamp) {
            return;
        }
        self.newest_stamp = Some(stamp);
        self.pending_tf.push_back((
            stamp.plus_secs_f64(self.time_offset),
            est,
            std::time::Instant::now(),
        ));
        while self.pending_tf.len() > 500 {
            self.pending_tf.pop_front();
            self.map_odom_dropped += 1;
        }
    }

    /// Publish every queued `map -> odom` whose `odom -> sensor` is available, oldest first. One
    /// that cannot be resolved blocks the newer ones until it has waited `max_wait` seconds of wall
    /// time (the odometry travels separately and may be late; stamp time is not a clock for that,
    /// since a replay runs it hundreds of times faster than real time), then it is dropped and
    /// counted.
    fn flush_map_odom(&mut self, node: &Node) {
        let (Some(tf), Some(frame)) = (self.tf.as_ref(), self.scan_frame.clone()) else {
            return;
        };
        while let Some(&(at, est, queued)) = self.pending_tf.front() {
            let looked_up = self.tf_buffer.lock().lookup_transform(
                &self.odom_frame,
                &frame,
                LookupTime::At(at),
            );
            match looked_up {
                Ok(t) => {
                    // T_map_odom = T_map_sensor * T_odom_sensor^-1
                    let correction = est.compose(&t.transform.inverse());
                    match tf.send(at, &self.map_frame, &self.odom_frame, &correction) {
                        Ok(()) => self.map_odom_published += 1,
                        Err(e) => {
                            self.last_error = format!("map -> odom broadcast: {e}");
                            log_error!(
                                node.logger().throttle(Duration::from_secs(5)),
                                "map -> odom broadcast failed: {e}"
                            );
                        }
                    }
                    self.pending_tf.pop_front();
                }
                Err(e) => {
                    if queued.elapsed().as_secs_f64() > self.max_wait {
                        self.map_odom_dropped += 1;
                        self.last_error = format!("{} -> {frame} at {at:?}: {e}", self.odom_frame);
                        self.pending_tf.pop_front();
                    } else {
                        break;
                    }
                }
            }
        }
    }

    /// Everything that follows a message: keyframes and path, loop events, a map after a closure,
    /// the queue of `map -> odom`.
    fn after_update(&mut self, node: &Node, new: Vec<NewKeyframe>) {
        self.publish_keyframes(new);
        let events = self.builder.take_loop_events();
        if !events.is_empty() {
            self.record_loops(node, &events);
            if events.iter().any(|e| e.accepted() > 0) && self.map_pub.is_some() {
                self.render_and_publish(node, "after a loop closure");
            }
        }
        self.queue_map_odom();
        self.flush_map_odom(node);
    }
}

fn invalid(node: &Node, what: &str) -> RclrsError {
    log_error!(node.logger(), "{what}");
    RclrsError::RclError {
        code: RclReturnCode::InvalidArgument,
        msg: None,
    }
}

fn open_csv(
    node: &Node,
    path: &str,
    header_row: &str,
) -> Result<Option<BufWriter<File>>, RclrsError> {
    if path.is_empty() {
        return Ok(None);
    }
    let f = File::create(path).map_err(|e| invalid(node, &format!("cannot create {path}: {e}")))?;
    let mut w = BufWriter::new(f);
    let _ = writeln!(w, "{header_row}");
    Ok(Some(w))
}

fn off(s: String) -> String {
    if s == "none" {
        String::new()
    } else {
        s
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("pose_graph_node")?;
    let mut p = ParamStore::new();
    let d = GraphBuilderConfig::default();

    let registration_topic = p.string(
        &node,
        "topics.registration",
        "registration",
        "Registration input from scan_matcher_node (reliable)",
    )?;
    let matched_topic = p.string(
        &node,
        "topics.scan_matched",
        "scan_matched",
        "The registered scans from scan_matcher_node (reliable)",
    )?;
    let pose_topic = p.string(
        &node,
        "topics.pose",
        "slam/pose",
        "Newest keyframe pose with its marginal covariance",
    )?;
    let path_topic = p.string(
        &node,
        "topics.path",
        "slam/path",
        "Every keyframe pose (nav_msgs/Path, latched)",
    )?;
    let map_topic = p.string(
        &node,
        "topics.map",
        "map",
        "The rendered map (nav_msgs/OccupancyGrid, latched)",
    )?;
    let map_frame = p.string(
        &node,
        "frames.map",
        "map",
        "Frame of the map and parent of the map -> odom edge this node owns; its origin is the first keyframe's sensor pose",
    )?;
    let odom_frame = p.string(
        &node,
        "frames.odom",
        "odom",
        "Odometry frame, the child of map -> odom (odom -> <scan frame> is looked up in TF)",
    )?;
    let slam_frame_param = p.string(
        &node,
        "frames.slam",
        "auto",
        "Frame of slam/pose and slam/path. auto: frames.map when the map or map -> odom is on, else slam_origin (the first keyframe's sensor pose, not in the TF tree)",
    )?;

    let loop_closure = p.bool(
        &node,
        "loop_closure.enabled",
        false,
        "Run the loop detector and verifier on every new keyframe and optimize on an accepted loop. Off: loop_closure: unsupported",
    )?;
    let map_enabled = p.bool(
        &node,
        "map.enabled",
        true,
        "Render and publish /map (after a loop closure and at the end of a run)",
    )?;
    let tf_enabled = p.bool(
        &node,
        "tf.publish_map_odom",
        true,
        "Own and publish the map -> odom edge. Never together with another owner of that edge",
    )?;
    let time_offset = p.f64(
        &node,
        "scan_time_offset_s",
        -0.06,
        -1.0,
        0.0,
        "Seconds added to a scan's stamp to get the instant it was measured, where odom -> sensor is looked up (the Webots lidar: -0.06)",
    )?;
    let max_wait = p.f64(
        &node,
        "tf.max_wait_s",
        0.5,
        0.0,
        10.0,
        "How long an estimate waits for odom -> <scan frame> at its instant before it is dropped (wall-clock s)",
    )?;
    let mut render_cfg = RenderConfig::default();
    render_cfg.size_m = p.f64(
        &node,
        "map.size_m",
        render_cfg.size_m,
        1.0,
        200.0,
        "Edge of the square map, centred on the origin of the map frame (m)",
    )?;
    render_cfg.resolution = p.f64(
        &node,
        "map.resolution_m",
        render_cfg.resolution,
        0.01,
        1.0,
        "Cell edge of the map (m)",
    )?;
    let layout = p.string(
        &node,
        "map.beam_layout",
        render_cfg.layout.name(),
        "Bearings of the beams in the render: published | pixel_centres (the Webots lidar, Phase 4's frozen choice)",
    )?;
    render_cfg.layout = robotics_mapping::BeamLayout::parse(&layout)
        .map_err(|e| invalid(&node, &format!("map.beam_layout: {e}")))?;
    let render_period = p.f64(
        &node,
        "map.render_period_s",
        0.0,
        0.0,
        3600.0,
        "Also render every this many seconds while keyframes are being added (0: only after a loop closure and at the end)",
    )?;

    let mut cfg = d;
    cfg.loop_closure = loop_closure;
    cfg.graph.keyframe_translation = p.f64(
        &node,
        "keyframe.translation_m",
        d.graph.keyframe_translation,
        0.01,
        100.0,
        "A keyframe after this much translation since the last (m)",
    )?;
    cfg.graph.keyframe_rotation = p.f64(
        &node,
        "keyframe.rotation_rad",
        d.graph.keyframe_rotation,
        0.01,
        3.2,
        "... or this much rotation (rad)",
    )?;
    cfg.graph.prior_sigma = p.f64(
        &node,
        "graph.prior_sigma",
        d.graph.prior_sigma,
        1e-6,
        10.0,
        "Standard deviation of the gauge prior on the first keyframe",
    )?;
    cfg.max_buffered = p.i64(
        &node,
        "buffer.max_scans",
        d.max_buffered as i64,
        4,
        100_000,
        "Scans or registrations kept waiting for their other half",
    )? as usize;
    let downsample_m = p.f64(&node, "keyframe.downsample_m", 0.05, 0.005, 1.0, "Voxel size of the points kept for a keyframe (m); the harness of the evidence runs uses 0.05")?;
    let csv_path = off(p.string(
        &node,
        "output.keyframes_csv",
        "none",
        "Per-keyframe CSV, rows as the keyframes were created (none: off)",
    )?);
    let final_csv_path = off(p.string(
        &node,
        "output.keyframes_final_csv",
        "none",
        "Per-keyframe CSV of the optimized poses, written at shutdown (none: off)",
    )?);
    let loops_csv_path = off(p.string(
        &node,
        "output.loops_csv",
        "none",
        "One row per loop candidate with the verifier's verdict (none: off)",
    )?);
    let final_map_path = off(p.string(
        &node,
        "output.final_map",
        "none",
        "The grid rendered at shutdown, robotics_mapping text format (none: off)",
    )?);
    let diag_period = p.i64(
        &node,
        "diagnostics_period_s",
        1,
        1,
        60,
        "Diagnostics period (s)",
    )?;

    let builder =
        GraphBuilder::new(cfg).map_err(|e| invalid(&node, &format!("graph parameters: {e}")))?;
    let csv = open_csv(&node, &csv_path, "id,stamp_ns,x,y,theta,arc_length_m")?;
    let loops_csv = open_csv(
        &node,
        &loops_csv_path,
        "from,to,stamp_ns,verdict,reason,chi2,distance_m,sigma_position_m,meas_x,meas_y,meas_theta",
    )?;
    let slam_frame = if slam_frame_param != "auto" {
        slam_frame_param
    } else if map_enabled || tf_enabled {
        map_frame.clone()
    } else {
        "slam_origin".to_string()
    };
    let diagnostics = DiagnosticsPublisher::new(&node)?;
    let frame = FrameId::new(&slam_frame);
    let tf_buffer = Shared::new(
        TransformBuffer::new(TransformBufferConfig::default()).expect("default TF buffer config"),
    );
    // The listener exists only when the node owns map -> odom: it needs odom -> sensor, and its
    // authority-conflict check is what reports a second owner of the edge.
    let tf_listener = if tf_enabled {
        Some(Arc::new(TfListener::new(&node, tf_buffer.clone())?))
    } else {
        None
    };
    let shared = Shared::new(State {
        builder,
        frame: frame.clone(),
        map_frame: FrameId::new(&map_frame),
        odom_frame: FrameId::new(&odom_frame),
        downsample: downsample_m,
        scans_malformed: 0,
        registrations_malformed: 0,
        last_error: String::new(),
        path: nav_msgs::msg::Path::default(),
        pose_pub: node.create_publisher::<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic.as_str().qos(qos::odometry()),
        )?,
        path_pub: node
            .create_publisher::<nav_msgs::msg::Path>(path_topic.as_str().qos(qos::map_latched()))?,
        csv,
        csv_rows: 0,
        loops_csv,
        path_epoch: 0,
        map_pub: if map_enabled {
            Some(node.create_publisher::<nav_msgs::msg::OccupancyGrid>(
                map_topic.as_str().qos(qos::map_latched()),
            )?)
        } else {
            None
        },
        render_cfg,
        maps_published: 0,
        last_render_keyframes: 0,
        last_render_note: String::new(),
        tf: if tf_enabled {
            Some(TfBroadcaster::new(&node)?)
        } else {
            None
        },
        tf_buffer,
        scan_frame: None,
        time_offset,
        max_wait,
        pending_tf: VecDeque::new(),
        newest_stamp: None,
        map_odom_published: 0,
        map_odom_dropped: 0,
    });
    let reliable = rclrs::QoSProfile::default()
        .reliable()
        .volatile()
        .keep_last(200);

    let n = Arc::clone(&node);
    let _scan_sub = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        matched_topic.as_str().qos(reliable),
        {
            let shared = shared.clone();
            let stop = Arc::clone(&shutdown);
            move |msg: sensor_msgs::msg::LaserScan| {
                if stop.load(Ordering::SeqCst) {
                    return;
                }
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                let scan = match laser_scan_from_ros(&msg) {
                    Ok(s) => s,
                    Err(e) => {
                        st.scans_malformed += 1;
                        st.last_error = format!("scan: {e}");
                        log_warn!(
                            n.logger().throttle(Duration::from_secs(5)),
                            "scan rejected: {e}"
                        );
                        return;
                    }
                };
                let points = match downsample(&scan_points(&scan), st.downsample) {
                    Ok(p) => p,
                    Err(e) => {
                        st.scans_malformed += 1;
                        st.last_error = format!("downsample: {e}");
                        return;
                    }
                };
                st.scan_frame = Some(scan.frame.clone());
                let stamp = scan.stamp;
                let new = st.builder.on_scan_with_raw(stamp, points, scan);
                st.after_update(&n, new);
            }
        },
    )?;
    let n = Arc::clone(&node);
    let _reg_sub = node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
        registration_topic.as_str().qos(reliable),
        {
            let shared = shared.clone();
            let stop = Arc::clone(&shutdown);
            move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
                if stop.load(Ordering::SeqCst) {
                    return;
                }
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                let reg = match registration_from_ros(&msg) {
                    Ok(r) => r,
                    Err(e) => {
                        st.registrations_malformed += 1;
                        st.last_error = format!("registration: {e}");
                        log_warn!(
                            n.logger().throttle(Duration::from_secs(5)),
                            "registration rejected: {e}"
                        );
                        return;
                    }
                };
                let new = st.builder.on_registration(reg);
                st.after_update(&n, new);
            }
        },
    )?;

    // The estimate of a scan often waits a moment for odom -> sensor at its instant, and no
    // further message may come to flush it.
    let n = Arc::clone(&node);
    let _retry = node.create_timer_repeating(Duration::from_millis(10), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            st.flush_map_odom(&n);
        }
    })?;

    // Optional periodic render, not part of DS17 (off by default).
    let _render_timer = if render_period > 0.0 && map_enabled {
        let n = Arc::clone(&node);
        let shared = shared.clone();
        Some(
            node.create_timer_repeating(Duration::from_secs_f64(render_period), move || {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                if st.builder.keyframe_count() != st.last_render_keyframes {
                    st.render_and_publish(&n, "periodic");
                }
            })?,
        )
    } else {
        None
    };

    let n = Arc::clone(&node);
    let listener = tf_listener.clone();
    let _diag = node.create_timer_repeating(Duration::from_secs(diag_period as u64), {
        let shared = shared.clone();
        move || {
            let guard = shared.lock();
            let st: &State = &guard;
            let stamp = match now(&n) {
                Ok(t) => t,
                Err(e) => {
                    log_error!(n.logger(), "clock error: {e}");
                    return;
                }
            };
            let c = st.builder.counters();
            let (waiting_scans, waiting_regs) = st.builder.waiting();
            let conflicts = listener
                .as_ref()
                .map(|l| l.authority_conflicts())
                .unwrap_or_default();
            let (level, message) = if !conflicts.is_empty() {
                (
                    DiagnosticLevel::Error,
                    format!("TF AUTHORITY CONFLICT on {conflicts:?}: two owners of one edge"),
                )
            } else if let Some(why) = st.builder.broken() {
                (DiagnosticLevel::Error, format!("BROKEN: {why}"))
            } else if st.builder.keyframe_count() == 0 {
                (
                    DiagnosticLevel::Warn,
                    "WAITING: no keyframe yet".to_string(),
                )
            } else {
                (
                    DiagnosticLevel::Ok,
                    format!("RUNNING (loop_closure: {})", st.builder.loop_closure()),
                )
            };
            let count_or_none = |on: bool, n: u64| {
                if on && n > 0 {
                    n.to_string()
                } else {
                    "none".to_string()
                }
            };
            let reason = if loop_closure {
                LOOP_CLOSURE_ENABLED_REASON
            } else {
                LOOP_CLOSURE_REASON
            };
            let b = StatusBuilder::new("pose_graph_node", level, &message)
                // The one line this node must always carry; see the module documentation.
                .value("loop_closure", st.builder.loop_closure())
                .value("loop_closure_reason", reason)
                .value("keyframes", st.builder.keyframe_count())
                .value("scans_received", c.scans)
                .value("registrations_received", c.registrations)
                .value("registrations_processed", c.processed)
                .value("registrations_out_of_order", c.out_of_order)
                .value("scans_dropped_unjoined", c.scans_dropped)
                .value("waiting_scans", waiting_scans)
                .value("waiting_registrations", waiting_regs)
                .value("edge_errors", c.edge_errors)
                .value("scans_malformed", st.scans_malformed)
                .value("registrations_malformed", st.registrations_malformed)
                .value("loop_candidates", c.loop_candidates)
                .value("loop_accepted", c.loop_accepted)
                .value("loop_rejected", c.loop_rejected)
                .value("loop_errors", c.loop_errors)
                .value("loop_edges", st.builder.loop_edge_count())
                .value("optimizations", c.optimizations)
                .value("pose_epoch", st.builder.pose_epoch())
                .value("pose_frame", st.frame.as_str())
                .value("map_frame", st.map_frame.as_str())
                .value(
                    "map_frame_origin",
                    "first keyframe sensor pose (not the world frame)",
                )
                .value(
                    "tf_published",
                    if st.tf.is_some() && st.map_odom_published > 0 {
                        "map -> odom"
                    } else {
                        "none"
                    },
                )
                .value(
                    "map_published",
                    count_or_none(st.map_pub.is_some(), st.maps_published),
                )
                .value(
                    "map_odom_published",
                    count_or_none(st.tf.is_some(), st.map_odom_published),
                )
                .value("map_odom_pending", st.pending_tf.len())
                .value("map_odom_dropped", st.map_odom_dropped)
                .value("authority_conflicts", conflicts.len())
                .value("last_render", &st.last_render_note)
                .value("last_error", &st.last_error)
                .value("last_loop_error", st.builder.last_loop_error());
            if let Err(e) = diagnostics.publish(stamp, vec![b.build()]) {
                log_error!(n.logger(), "diagnostics publish failed: {e}");
            }
        }
    })?;

    log_info!(
        node.logger(),
        "pose_graph_node up: {} + {} -> {} / {}, poses in frame {}, keyframes every {} m or {} rad, loop_closure: {}, map: {}, map -> odom: {}, {} parameters",
        registration_topic,
        matched_topic,
        pose_topic,
        path_topic,
        slam_frame,
        cfg.graph.keyframe_translation,
        cfg.graph.keyframe_rotation,
        if loop_closure {
            "enabled"
        } else {
            "unsupported"
        },
        if map_enabled { map_topic.as_str() } else { "off" },
        if tf_enabled { "owned" } else { "off" },
        p.len()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    {
        let mut guard = shared.lock();
        let st: &mut State = &mut guard;
        if let Some(w) = st.csv.as_mut() {
            let _ = w.flush();
        }
        // The end-of-run render (DS17): one more replay of every keyframe at its final pose.
        if (map_enabled || !final_map_path.is_empty()) && st.builder.keyframe_count() > 0 {
            if let Some(text) = st.render_and_publish(&node, "at the end of the run") {
                if !final_map_path.is_empty() {
                    match std::fs::write(&final_map_path, text) {
                        Ok(()) => {
                            log_info!(node.logger(), "wrote the final map to {final_map_path}")
                        }
                        Err(e) => log_error!(node.logger(), "cannot write {final_map_path}: {e}"),
                    }
                }
            }
        }
        if !final_csv_path.is_empty() {
            let (poses, stamps) = st.builder.keyframes();
            let mut text = String::from("id,stamp_ns,x,y,theta\n");
            for (i, (pose, ns)) in poses.iter().zip(stamps).enumerate() {
                text.push_str(&format!(
                    "{i},{ns},{},{},{}\n",
                    pose.x(),
                    pose.y(),
                    pose.theta()
                ));
            }
            match std::fs::write(&final_csv_path, text) {
                Ok(()) => log_info!(
                    node.logger(),
                    "wrote the optimized keyframes to {final_csv_path}"
                ),
                Err(e) => log_error!(node.logger(), "cannot write {final_csv_path}: {e}"),
            }
        }
        if let Some(w) = st.loops_csv.as_mut() {
            let _ = w.flush();
        }
        let c = st.builder.counters();
        log_info!(
            node.logger(),
            "pose_graph_node shutting down: {} keyframes from {} registrations ({} processed, {} out of order, {} scans dropped unjoined), loop_closure: {} ({} candidates, {} accepted, {} rejected, {} loop edges), {} map(s), {} map -> odom",
            st.builder.keyframe_count(),
            c.registrations,
            c.processed,
            c.out_of_order,
            c.scans_dropped,
            st.builder.loop_closure(),
            c.loop_candidates,
            c.loop_accepted,
            c.loop_rejected,
            st.builder.loop_edge_count(),
            st.maps_published,
            st.map_odom_published
        );
    }
    drop(p);
    result
}
