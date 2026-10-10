//! `mapping_node` — occupancy grid mapping with known poses (Phase 4).
//!
//! ```text
//!   sensor_msgs/LaserScan (topics.scan)  ─┐
//!   /tf  map -> ... -> <scan frame>        ┴─► robotics_mapping::ScanIntegrator ─► OccupancyGrid
//!        looked up AT THE SCAN'S STAMP                                               │
//!                          nav_msgs/OccupancyGrid  (topics.map, latched)  ◄──────────┤
//!                          sensor_msgs/Image 32FC1 (topics.esdf)          ◄──────────┤
//!                          std_msgs/Float64        (topics.entropy)       ◄──────────┘
//!                          + /diagnostics
//! ```
//!
//! **The pose comes from TF and from nothing else.** There is no pose-source parameter, no
//! subscription to `/ground_truth/pose` and no odometry input: who owns `map -> odom` is
//! decided by what the launch file starts, so the same binary runs in simulation (with
//! `ground_truth_tf_adapter`) and with an estimator, and cannot quietly use a different pose
//! in one of them (decision D2). This node publishes no TF and owns no TF edge.
//!
//! A scan is integrated at the pose TF gives for its own header stamp (one instant, no
//! deskewing: decision D9). A scan whose transform is not available is kept for a short while,
//! because the scan and the transform travel separately, and is then skipped and counted. It
//! is never integrated at a guessed pose.
//!
//! Optional files, all off unless a path is given, for the evidence of a run:
//! `output.latency_csv` (one row per scan: what it did and how long it took, for claim M4),
//! `output.trace_csv` (entropy and state counts over time), `output.record_csv` (every
//! integrated scan with the sensor pose it was integrated at, so that a replay can check
//! determinism, claim M2) and `output.final_grid` (the exact log-odds grid at shutdown).
//!
//! Not built: the `~/reset` service the plan listed. The Rust bindings of `std_srvs` are not in
//! the interface underlay this repository builds (`scripts/interfaces.repos`), and adding them
//! means rebuilding the underlay, which is outside this task; a mapper is reset by restarting it.

use rclrs::*;
use robotics_core::tf::{LookupTime, TransformBuffer, TransformBufferConfig};
use robotics_core::{FrameId, LaserScan, Se2, Timestamp};
use robotics_mapping::{
    grid_entropy, signed_esdf, GridGeometry, MotionFilter, MotionGate, NoReturn, OccupancyGrid,
    OccupancyParams, RayTracer, ScanIntegrator,
};
use robotics_ros_bridge::convert::{header, laser_scan_from_ros, now, pose_to_ros};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::tf::TfListener;
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::{builtin_interfaces, nav_msgs, sensor_msgs, std_msgs};
use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::Arc;
use std::time::{Duration, Instant};

struct Pending {
    scan: LaserScan,
    received: Instant,
}

#[derive(Default)]
struct Counters {
    received: u64,
    integrated: u64,
    skipped_malformed: u64,
    skipped_no_transform: u64,
    skipped_motion: u64,
    skipped_stationary: u64,
    skipped_sensor_outside_grid: u64,
    hits: u64,
    no_return_cleared: u64,
    invalid_beams: u64,
}

struct State {
    grid: OccupancyGrid,
    integrator: ScanIntegrator,
    map_frame: FrameId,
    esdf_truncation: f64,
    /// Window of a scan's timestamp uncertainty and the gate over it; `None` disables the gate.
    gate: Option<(f64, MotionGate)>,
    /// Seconds added to a scan's stamp to get the instant its beams were measured (<= 0).
    time_offset: f64,
    motion_filter: Option<MotionFilter>,
    last_integrated: Option<Se2>,
    pending: VecDeque<Pending>,
    max_pending: usize,
    max_age: Duration,
    c: Counters,
    last_stamp: Option<Timestamp>,
    last_error: String,
    recent_latency_us: VecDeque<f64>,
    map_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    esdf_pub: Publisher<sensor_msgs::msg::Image>,
    entropy_pub: Publisher<std_msgs::msg::Float64>,
    latency_csv: Option<BufWriter<File>>,
    trace_csv: Option<BufWriter<File>>,
    record_csv: Option<BufWriter<File>>,
    buffer: Shared<TransformBuffer>,
}

fn open(path: &str) -> Result<Option<BufWriter<File>>, RclrsError> {
    if path.is_empty() {
        return Ok(None);
    }
    File::create(path)
        .map(|f| Some(BufWriter::new(f)))
        .map_err(|e| {
            rclrs::log_error!("mapping_node", "cannot create {path}: {e}");
            RclrsError::RclError {
                code: RclReturnCode::InvalidArgument,
                msg: None,
            }
        })
}

fn invalid(node: &Node, what: &str) -> RclrsError {
    log_error!(node.logger(), "{what}");
    RclrsError::RclError {
        code: RclReturnCode::InvalidArgument,
        msg: None,
    }
}

enum Outcome {
    Integrated,
    /// The pose changed too much across the scan's timestamp window: skipped, by design.
    SkippedMotion,
    /// The sensor has not moved since the last integrated scan: skipped, by design.
    SkippedStationary,
    /// A transform is missing (yet).
    NotReady,
}

impl State {
    /// Look the sensor pose up at the scan's stamp, check that it was steady across the window
    /// the stamp leaves open, and integrate. `Err` means the scan can never be integrated.
    fn try_integrate(&mut self, p: &Pending) -> Result<Outcome, String> {
        let started = Instant::now();
        let t = match self.buffer.lock().lookup_transform(
            &self.map_frame,
            &p.scan.frame,
            LookupTime::At(p.scan.stamp),
        ) {
            Ok(t) => t,
            Err(_) => return Ok(Outcome::NotReady),
        };
        let at_stamp: Se2 = t.transform;
        // The pose the scan is integrated at: where the sensor was when the beams were
        // measured, which is `time_offset` before the stamp (decision D15).
        let sensor: Se2 = if self.time_offset == 0.0 {
            at_stamp
        } else {
            match self.buffer.lock().lookup_transform(
                &self.map_frame,
                &p.scan.frame,
                LookupTime::At(p.scan.stamp.plus_secs_f64(self.time_offset)),
            ) {
                Ok(t) => t.transform,
                Err(_) => return Ok(Outcome::NotReady),
            }
        };
        let mut window_change: Option<(f64, f64)> = None;
        if let Some((window, gate)) = self.gate {
            // The beams were measured somewhere in [stamp - window, stamp] (see the gate's
            // module documentation), so the pose at the start of that window bounds the error.
            let earlier = match self.buffer.lock().lookup_transform(
                &self.map_frame,
                &p.scan.frame,
                LookupTime::At(p.scan.stamp.plus_secs_f64(-window)),
            ) {
                Ok(t) => t.transform,
                Err(_) => return Ok(Outcome::NotReady),
            };
            window_change = Some(MotionGate::change(&earlier, &at_stamp));
            if !gate.admits(&earlier, &at_stamp) {
                self.c.skipped_motion += 1;
                if let Some(w) = self.latency_csv.as_mut() {
                    let (dt, dy) = MotionGate::change(&earlier, &at_stamp);
                    let _ = writeln!(
                        w,
                        "{:.6},skipped_motion,{:.1},{:.1},{},,,,,,{dt:.4},{dy:.4}",
                        p.scan.stamp.as_secs_f64(),
                        started.elapsed().as_secs_f64() * 1e6,
                        p.received.elapsed().as_secs_f64() * 1e6,
                        p.scan.ranges.len()
                    );
                }
                return Ok(Outcome::SkippedMotion);
            }
        }
        if let Some(filter) = self.motion_filter {
            if !filter.admits(self.last_integrated.as_ref(), &sensor) {
                self.c.skipped_stationary += 1;
                if let Some(w) = self.latency_csv.as_mut() {
                    let (dt, dy) = self
                        .last_integrated
                        .map_or((0.0, 0.0), |last| MotionGate::change(&last, &sensor));
                    let _ = writeln!(
                        w,
                        "{:.6},skipped_stationary,{:.1},{:.1},{},,,,,,{dt:.4},{dy:.4}",
                        p.scan.stamp.as_secs_f64(),
                        started.elapsed().as_secs_f64() * 1e6,
                        p.received.elapsed().as_secs_f64() * 1e6,
                        p.scan.ranges.len()
                    );
                }
                return Ok(Outcome::SkippedStationary);
            }
        }
        let report = match self.integrator.integrate(&mut self.grid, &sensor, &p.scan) {
            Ok(r) => r,
            Err(robotics_mapping::MappingError::SensorOutsideGrid { .. }) => {
                self.c.skipped_sensor_outside_grid += 1;
                return Err("sensor outside the grid".into());
            }
            Err(e) => return Err(e.to_string()),
        };
        let latency_us = started.elapsed().as_secs_f64() * 1e6;
        let wait_us = p.received.elapsed().as_secs_f64() * 1e6;
        self.c.integrated += 1;
        self.last_integrated = Some(sensor);
        self.c.hits += report.hits as u64;
        self.c.no_return_cleared += report.no_return_cleared as u64;
        self.c.invalid_beams += report.invalid as u64;
        self.last_stamp = Some(p.scan.stamp);
        self.recent_latency_us.push_back(latency_us);
        if self.recent_latency_us.len() > 2000 {
            self.recent_latency_us.pop_front();
        }
        if let Some(w) = self.latency_csv.as_mut() {
            let _ = writeln!(
                w,
                "{:.6},integrated,{latency_us:.1},{wait_us:.1},{},{},{},{},{},{},{}",
                p.scan.stamp.as_secs_f64(),
                report.beams,
                report.hits,
                report.no_return_cleared,
                report.invalid,
                report.cells_free,
                report.cells_occupied,
                window_change
                    .map_or_else(|| ",".to_string(), |(dt, dy)| format!("{dt:.4},{dy:.4}"))
            );
        }
        if let Some(w) = self.record_csv.as_mut() {
            let s = &p.scan;
            let _ = write!(
                w,
                "{:.9},{},{},{},{},{},{},{},{}",
                s.stamp.as_secs_f64(),
                sensor.x(),
                sensor.y(),
                sensor.theta(),
                s.angle_min,
                s.angle_increment,
                s.range_min,
                s.range_max,
                s.ranges.len()
            );
            for r in &s.ranges {
                let _ = write!(w, ",{r}");
            }
            let _ = writeln!(w);
        }
        Ok(Outcome::Integrated)
    }

    /// Try the oldest pending scans in order; drop the ones that waited too long.
    fn drain(&mut self) {
        while let Some(front) = self.pending.pop_front() {
            match self.try_integrate(&front) {
                Ok(Outcome::Integrated)
                | Ok(Outcome::SkippedMotion)
                | Ok(Outcome::SkippedStationary) => {}
                Ok(Outcome::NotReady) => {
                    if front.received.elapsed() > self.max_age {
                        self.c.skipped_no_transform += 1;
                        if let Some(w) = self.latency_csv.as_mut() {
                            let _ = writeln!(
                                w,
                                "{:.6},skipped_no_transform,,{:.1},{},,,,,,,",
                                front.scan.stamp.as_secs_f64(),
                                front.received.elapsed().as_secs_f64() * 1e6,
                                front.scan.ranges.len()
                            );
                        }
                    } else {
                        // Not yet, and younger scans cannot be ready before an older one.
                        self.pending.push_front(front);
                        break;
                    }
                }
                Err(e) => {
                    // A sensor outside the grid was counted by its own counter already.
                    if !e.starts_with("sensor outside") {
                        self.c.skipped_malformed += 1;
                    }
                    self.last_error = e;
                }
            }
        }
    }

    fn percentile_us(&self, q: f64) -> Option<f64> {
        if self.recent_latency_us.is_empty() {
            return None;
        }
        let mut v: Vec<f64> = self.recent_latency_us.iter().copied().collect();
        v.sort_by(|a, b| a.partial_cmp(b).expect("finite latencies"));
        let rank = ((q * v.len() as f64).ceil() as usize).clamp(1, v.len());
        Some(v[rank - 1])
    }

    /// Publish the map, the ESDF and the entropy, all stamped with the last integrated scan.
    fn publish_all(&mut self, node: &Node) {
        let Some(stamp) = self.last_stamp else { return };
        let g = *self.grid.geometry();
        let hdr = header(stamp, &self.map_frame);
        let origin = Se2::new(g.origin.x, g.origin.y, 0.0);
        let map = nav_msgs::msg::OccupancyGrid {
            header: hdr.clone(),
            info: nav_msgs::msg::MapMetaData {
                map_load_time: builtin_interfaces::msg::Time::default(),
                resolution: g.resolution as f32,
                width: g.width as u32,
                height: g.height as u32,
                origin: pose_to_ros(&origin),
            },
            data: self.grid.to_ros_data(),
        };
        if let Err(e) = self.map_pub.publish(map) {
            log_error!(
                node.logger().throttle(Duration::from_secs(5)),
                "map publish failed: {e}"
            );
        }
        match signed_esdf(&self.grid, self.esdf_truncation) {
            Ok(esdf) => {
                let mut bytes = Vec::with_capacity(esdf.values().len() * 4);
                for v in esdf.values() {
                    bytes.extend_from_slice(&v.to_le_bytes());
                }
                let img = sensor_msgs::msg::Image {
                    header: hdr,
                    height: g.height as u32,
                    width: g.width as u32,
                    encoding: "32FC1".to_string(),
                    is_bigendian: 0,
                    step: (g.width * 4) as u32,
                    data: bytes,
                };
                if let Err(e) = self.esdf_pub.publish(img) {
                    log_error!(
                        node.logger().throttle(Duration::from_secs(5)),
                        "esdf publish failed: {e}"
                    );
                }
            }
            Err(e) => self.last_error = format!("esdf: {e}"),
        }
        let e = grid_entropy(&self.grid);
        let _ = self
            .entropy_pub
            .publish(std_msgs::msg::Float64 { data: e.total_bits });
        if let Some(w) = self.trace_csv.as_mut() {
            let (free, occ, unk) = self.grid.count_states();
            let _ = writeln!(
                w,
                "{:.6},{},{},{},{},{},{},{}",
                stamp.as_secs_f64(),
                e.total_bits,
                e.decided_bits,
                e.decided_cells,
                free,
                occ,
                unk,
                self.c.integrated
            );
            let _ = w.flush();
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("mapping_node")?;
    let mut p = ParamStore::new();

    let scan_topic = p.string(&node, "topics.scan", "scan", "Laser scan input")?;
    let map_topic = p.string(
        &node,
        "topics.map",
        "map",
        "Occupancy grid output (latched)",
    )?;
    let esdf_topic = p.string(
        &node,
        "topics.esdf",
        "esdf",
        "Signed ESDF output (sensor_msgs/Image, 32FC1, metres), paired by stamp with the map",
    )?;
    let entropy_topic = p.string(
        &node,
        "topics.entropy",
        "map_entropy",
        "Total grid entropy output, bits (std_msgs/Float64)",
    )?;
    let map_frame = p.string(
        &node,
        "frames.map",
        "map",
        "Frame of the grid; the sensor pose is looked up in it through TF",
    )?;

    let origin_x = p.f64(
        &node,
        "grid.origin_x",
        -3.5,
        -1e4,
        1e4,
        "x of the grid's lower-left corner (m)",
    )?;
    let origin_y = p.f64(
        &node,
        "grid.origin_y",
        -3.5,
        -1e4,
        1e4,
        "y of the grid's lower-left corner (m)",
    )?;
    let size_x = p.f64(
        &node,
        "grid.size_x_m",
        7.0,
        0.1,
        1e3,
        "Grid extent along x (m)",
    )?;
    let size_y = p.f64(
        &node,
        "grid.size_y_m",
        7.0,
        0.1,
        1e3,
        "Grid extent along y (m)",
    )?;
    let resolution = p.f64(&node, "grid.resolution", 0.05, 0.005, 5.0, "Cell edge (m)")?;

    let prob = |p: &mut ParamStore, name: &str, default: f64, what: &str| {
        p.f64(&node, name, default, 1e-3, 1.0 - 1e-3, what)
    };
    let p_hit = prob(
        &mut p,
        "model.p_hit",
        0.7,
        "Occupancy probability a hit adds",
    )?;
    let p_miss = prob(
        &mut p,
        "model.p_miss",
        0.4,
        "Occupancy probability a crossing adds (below 0.5)",
    )?;
    let p_min = prob(&mut p, "model.p_min", 0.12, "Lower clamp probability")?;
    let p_max = prob(&mut p, "model.p_max", 0.97, "Upper clamp probability")?;
    let p_occupied = prob(
        &mut p,
        "model.p_occupied",
        0.95,
        "A cell at or above this is occupied",
    )?;
    let p_free = prob(
        &mut p,
        "model.p_free",
        0.35,
        "A cell at or below this is free",
    )?;
    let no_return = match p
        .string(
            &node,
            "model.no_return",
            "clear_to_max_range",
            "Beams with no return: clear_to_max_range | drop",
        )?
        .as_str()
    {
        "clear_to_max_range" => NoReturn::ClearToMaxRange,
        "drop" => NoReturn::Drop,
        other => {
            return Err(invalid(
                &node,
                &format!("model.no_return: unknown policy {other:?}"),
            ))
        }
    };
    let clear_margin = p.f64(
        &node,
        "model.clear_margin_m",
        robotics_mapping::DEFAULT_CLEAR_MARGIN,
        0.0,
        10.0,
        "A no-return beam clears free space up to range_max less this margin (m)",
    )?;
    let tracer = match p
        .string(
            &node,
            "model.tracer",
            "supercover",
            "Ray traversal: supercover | bresenham",
        )?
        .as_str()
    {
        "supercover" => RayTracer::Supercover,
        "bresenham" => RayTracer::Bresenham,
        other => {
            return Err(invalid(
                &node,
                &format!("model.tracer: unknown traversal {other:?}"),
            ))
        }
    };
    let gate_window = p.f64(
        &node,
        "motion_gate.window_s",
        0.2,
        0.0,
        10.0,
        "Window the scan stamp leaves open for the measurement instant (s); 0 turns the gate off",
    )?;
    let gate_translation = p.f64(
        &node,
        "motion_gate.max_translation_m",
        0.05,
        1e-4,
        10.0,
        "Skip a scan if the sensor moved farther than this across the window (m)",
    )?;
    let gate_yaw = p.f64(
        &node,
        "motion_gate.max_yaw_rad",
        0.015,
        1e-5,
        3.2,
        "Skip a scan if the sensor turned more than this across the window (rad)",
    )?;
    let time_offset = p.f64(
        &node,
        "scan_time_offset_s",
        0.0,
        -1.0,
        0.0,
        "Seconds added to a scan's stamp to get the instant it was measured (the Webots lidar: -0.06)",
    )?;
    let filter_translation = p.f64(
        &node,
        "motion_filter.min_translation_m",
        0.02,
        0.0,
        10.0,
        "Integrate only if the sensor moved this far since the last integrated scan (m); 0 turns the filter off",
    )?;
    let filter_yaw = p.f64(
        &node,
        "motion_filter.min_yaw_rad",
        0.02,
        0.0,
        3.2,
        "... or turned this far (rad); 0 turns the filter off",
    )?;
    let esdf_truncation = p.f64(
        &node,
        "esdf.truncation_m",
        2.0,
        0.01,
        100.0,
        "ESDF saturation distance (m)",
    )?;
    let publish_period = p.f64(
        &node,
        "publish.period_s",
        1.0,
        0.05,
        60.0,
        "Map, ESDF and entropy period (s)",
    )?;
    let max_age = p.f64(
        &node,
        "pending.max_age_s",
        0.5,
        0.0,
        10.0,
        "How long a scan waits for its transform (s)",
    )?;
    let max_pending = p.i64(
        &node,
        "pending.max_scans",
        8,
        1,
        1000,
        "Scans kept waiting for a transform",
    )? as usize;
    let latency_csv = p.string(
        &node,
        "output.latency_csv",
        "",
        "Per-scan outcome and latency CSV (empty: off)",
    )?;
    let trace_csv = p.string(
        &node,
        "output.trace_csv",
        "",
        "Entropy and state-count trace CSV (empty: off)",
    )?;
    let record_csv = p.string(
        &node,
        "output.record_csv",
        "",
        "Integrated scans with their poses, for replay (empty: off)",
    )?;
    let final_grid = p.string(
        &node,
        "output.final_grid",
        "",
        "Exact log-odds grid written at shutdown (empty: off)",
    )?;
    let diag_period = p.i64(
        &node,
        "diagnostics_period_s",
        1,
        1,
        60,
        "Diagnostics period (s)",
    )?;

    let geometry = GridGeometry {
        width: (size_x / resolution).round() as usize,
        height: (size_y / resolution).round() as usize,
        resolution,
        origin: robotics_core::nalgebra::Vector2::new(origin_x, origin_y),
    };
    let gate = if gate_window > 0.0 {
        Some((
            gate_window,
            MotionGate::new(gate_translation, gate_yaw)
                .map_err(|e| invalid(&node, &format!("motion gate: {e}")))?,
        ))
    } else {
        None
    };
    let motion_filter = if filter_translation > 0.0 && filter_yaw > 0.0 {
        Some(
            MotionFilter::new(filter_translation, filter_yaw)
                .map_err(|e| invalid(&node, &format!("motion filter: {e}")))?,
        )
    } else {
        None
    };
    let params =
        OccupancyParams::from_probabilities(p_hit, p_miss, p_min, p_max, p_occupied, p_free);
    let grid = OccupancyGrid::new(geometry, params)
        .map_err(|e| invalid(&node, &format!("grid or model parameters: {e}")))?;

    let mut latency_csv_w = open(&latency_csv)?;
    if let Some(w) = latency_csv_w.as_mut() {
        let _ = writeln!(
            w,
            "stamp_s,outcome,latency_us,queue_wait_us,beams,hits,no_return_cleared,invalid,cells_free,cells_occupied,window_translation_m,window_yaw_rad"
        );
    }
    let mut trace_csv_w = open(&trace_csv)?;
    if let Some(w) = trace_csv_w.as_mut() {
        let _ = writeln!(
            w,
            "stamp_s,entropy_total_bits,entropy_decided_bits,decided_cells,free_cells,occupied_cells,unknown_cells,scans_integrated"
        );
    }
    let mut record_csv_w = open(&record_csv)?;
    if let Some(w) = record_csv_w.as_mut() {
        let _ = writeln!(
            w,
            "# stamp_s,sensor_x,sensor_y,sensor_theta,angle_min,angle_increment,range_min,range_max,n,ranges..."
        );
    }

    let buffer = Shared::new(
        TransformBuffer::new(TransformBufferConfig::default()).expect("default TF buffer config"),
    );
    let _tf_listener = TfListener::new(&node, buffer.clone())?;
    let diagnostics = DiagnosticsPublisher::new(&node)?;
    let shared = Shared::new(State {
        grid,
        integrator: ScanIntegrator::new(no_return, tracer)
            .with_clear_margin(clear_margin)
            .map_err(|e| invalid(&node, &format!("model.clear_margin_m: {e}")))?,
        map_frame: FrameId::new(&map_frame),
        esdf_truncation,
        gate,
        time_offset,
        motion_filter,
        last_integrated: None,
        pending: VecDeque::new(),
        max_pending,
        max_age: Duration::from_secs_f64(max_age),
        c: Counters::default(),
        last_stamp: None,
        last_error: String::new(),
        recent_latency_us: VecDeque::new(),
        map_pub: node.create_publisher::<nav_msgs::msg::OccupancyGrid>(
            map_topic.as_str().qos(qos::map_latched()),
        )?,
        esdf_pub: node.create_publisher::<sensor_msgs::msg::Image>(
            esdf_topic.as_str().qos(qos::map_latched()),
        )?,
        entropy_pub: node.create_publisher::<std_msgs::msg::Float64>(
            entropy_topic.as_str().qos(qos::odometry()),
        )?,
        latency_csv: latency_csv_w,
        trace_csv: trace_csv_w,
        record_csv: record_csv_w,
        buffer,
    });

    let n = Arc::clone(&node);
    let _scan_sub = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        scan_topic.as_str().qos(qos::sensor_data()),
        {
            let shared = shared.clone();
            move |msg: sensor_msgs::msg::LaserScan| {
                let received = Instant::now();
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                st.c.received += 1;
                let scan = match laser_scan_from_ros(&msg) {
                    Ok(s) => s,
                    Err(e) => {
                        st.c.skipped_malformed += 1;
                        st.last_error = format!("scan: {e}");
                        log_warn!(
                            n.logger().throttle(Duration::from_secs(5)),
                            "scan rejected: {e}"
                        );
                        return;
                    }
                };
                st.pending.push_back(Pending { scan, received });
                while st.pending.len() > st.max_pending {
                    st.pending.pop_front();
                    st.c.skipped_no_transform += 1;
                }
                st.drain();
            }
        },
    )?;

    // Scans wait for their transform; the transform of the newest scan usually arrives after
    // the scan, and no further scan may come to retry it.
    let _retry = node.create_timer_repeating(Duration::from_millis(20), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            st.drain();
        }
    })?;

    let n = Arc::clone(&node);
    let _publish = node.create_timer_repeating(Duration::from_secs_f64(publish_period), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            st.publish_all(&n);
        }
    })?;

    let n = Arc::clone(&node);
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
            let (level, message) = if st.c.integrated == 0 {
                (
                    DiagnosticLevel::Warn,
                    "WAITING: no scan integrated yet".to_string(),
                )
            } else if st.c.skipped_no_transform > st.c.integrated {
                (
                    DiagnosticLevel::Warn,
                    "MOSTLY SKIPPING: no transform at the scan stamps".to_string(),
                )
            } else {
                (DiagnosticLevel::Ok, "RUNNING".to_string())
            };
            let e = grid_entropy(&st.grid);
            let (free, occ, unk) = st.grid.count_states();
            let mut b = StatusBuilder::new("mapping_node", level, &message)
                .value("scans_received", st.c.received)
                .value("scans_integrated", st.c.integrated)
                .value("skipped_no_transform", st.c.skipped_no_transform)
                .value("skipped_motion", st.c.skipped_motion)
                .value("skipped_stationary", st.c.skipped_stationary)
                .value("skipped_malformed", st.c.skipped_malformed)
                .value(
                    "skipped_sensor_outside_grid",
                    st.c.skipped_sensor_outside_grid,
                )
                .value("pending", st.pending.len())
                .value("hits", st.c.hits)
                .value("no_return_cleared", st.c.no_return_cleared)
                .value("invalid_beams", st.c.invalid_beams)
                .value("free_cells", free)
                .value("occupied_cells", occ)
                .value("unknown_cells", unk)
                .value("entropy_bits", format!("{:.1}", e.total_bits))
                .value("pose_source", "tf")
                .value("last_error", &st.last_error);
            if let (Some(p50), Some(p95), Some(p99)) = (
                st.percentile_us(0.5),
                st.percentile_us(0.95),
                st.percentile_us(0.99),
            ) {
                b = b
                    .value("update_latency_us_p50", format!("{p50:.0}"))
                    .value("update_latency_us_p95", format!("{p95:.0}"))
                    .value("update_latency_us_p99", format!("{p99:.0}"));
            }
            if let Err(e) = diagnostics.publish(stamp, vec![b.build()]) {
                log_error!(n.logger(), "diagnostics publish failed: {e}");
            }
        }
    })?;

    log_info!(
        node.logger(),
        "mapping_node up: {} -> {} / {} / {}, grid {}x{} cells of {} m at ({}, {}), pose from TF only, \
         no return: {:?}, traversal: {:?}, {} parameters",
        scan_topic,
        map_topic,
        esdf_topic,
        entropy_topic,
        geometry.width,
        geometry.height,
        resolution,
        origin_x,
        origin_y,
        no_return,
        tracer,
        p.len()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);

    // Everything the run recorded is flushed or written here, once, so that a SIGINT leaves
    // complete files.
    {
        let mut guard = shared.lock();
        let st: &mut State = &mut guard;
        st.drain();
        st.publish_all(&node);
        for w in [&mut st.latency_csv, &mut st.trace_csv, &mut st.record_csv]
            .into_iter()
            .flatten()
        {
            let _ = w.flush();
        }
        if !final_grid.is_empty() {
            match std::fs::write(&final_grid, st.grid.to_text()) {
                Ok(()) => log_info!(node.logger(), "wrote the final grid to {final_grid}"),
                Err(e) => log_error!(node.logger(), "cannot write {final_grid}: {e}"),
            }
        }
        log_info!(
            node.logger(),
            "mapping_node shutting down: {} scans received, {} integrated, {} skipped for motion, {} skipped as stationary repeats, {} skipped for want of a transform",
            st.c.received,
            st.c.integrated,
            st.c.skipped_motion,
            st.c.skipped_stationary,
            st.c.skipped_no_transform
        );
    }
    drop(p);
    result
}
