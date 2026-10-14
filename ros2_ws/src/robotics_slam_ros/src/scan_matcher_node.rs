//! `scan_matcher_node` — LiDAR odometry (Phase 5): the tested `robotics_slam` front end as a node.
//!
//! ```text
//!   sensor_msgs/LaserScan (topics.scan) ──► LidarOdometry ──► geometry_msgs/PoseWithCovarianceStamped
//!                                           (voxel-hash map,    (topics.registration: the increment
//!                                            point-to-plane      from the previous scan, with the
//!                                            ICP, adaptive       inverse ICP information as covariance)
//!                                            threshold)      └─► sensor_msgs/LaserScan
//!                                                                (topics.scan_matched: the same scan,
//!                                                                 reliable, for the pose graph)
//!                                                              + /diagnostics
//! ```
//!
//! No algorithm is written here: `robotics_slam::odometry::LidarOdometry` does the work and this
//! file converts messages, publishes and counts. Its parameters default to the library's own
//! defaults (`OdometryConfig::default()`, including the frozen `info_inflation`), so a run of this
//! node and a run of the library harness (`slam_synthetic_run`) use the same configuration.
//!
//! **Inputs.** Scans only. There is no odometry input: the front end predicts with the last
//! per-scan increment (decision DS6), and the Webots odometry is both calibrated to its world
//! and stamped on a clock that does not follow the lidar's measurements (Phase 4,
//! implementation 5.2 item 2), so using it as a prior would inject that lag and change the
//! algorithm the evidence runs graded. Publishes no TF and no map.
//!
//! The scans are republished (`topics.scan_matched`) with reliable QoS because the pose graph needs
//! the points of exactly the scans that were registered, and a reliable subscriber cannot connect
//! to a best-effort sensor topic.
//!
//! Optional file, off unless a path is given: `output.registrations_csv`, one row per scan.

use rclrs::*;
use robotics_core::FrameId;
use robotics_ros_bridge::convert::{laser_scan_from_ros, now};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::{qos, ParamStore};
use robotics_slam::odometry::{LidarOdometry, OdometryConfig, TrackingStatus};
use robotics_slam_ros::registration_to_ros;
use ros_env::{geometry_msgs, sensor_msgs};
use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Default)]
struct Counters {
    received: u64,
    published: u64,
    initialized: u64,
    tracked: u64,
    predicted: u64,
    retried: u64,
    information_fallback: u64,
    skipped_malformed: u64,
    skipped_few_points: u64,
    publish_errors: u64,
}

struct State {
    odo: LidarOdometry,
    c: Counters,
    last_error: String,
    last_threshold: f64,
    latencies_us: VecDeque<f64>,
    registration_pub: Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>,
    matched_pub: Publisher<sensor_msgs::msg::LaserScan>,
    csv: Option<BufWriter<File>>,
}

impl State {
    fn percentile_us(&self, q: f64) -> Option<f64> {
        if self.latencies_us.is_empty() {
            return None;
        }
        let mut v: Vec<f64> = self.latencies_us.iter().copied().collect();
        v.sort_by(|a, b| a.total_cmp(b));
        let rank = ((q * v.len() as f64).ceil() as usize).clamp(1, v.len());
        Some(v[rank - 1])
    }
}

fn invalid(node: &Node, what: &str) -> RclrsError {
    log_error!(node.logger(), "{what}");
    RclrsError::RclError {
        code: RclReturnCode::InvalidArgument,
        msg: None,
    }
}

fn open(path: &str) -> Result<Option<BufWriter<File>>, RclrsError> {
    if path.is_empty() {
        return Ok(None);
    }
    File::create(path)
        .map(|f| Some(BufWriter::new(f)))
        .map_err(|e| {
            log_error!("scan_matcher_node", "cannot create {path}: {e}");
            RclrsError::RclError {
                code: RclReturnCode::Error,
                msg: None,
            }
        })
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("scan_matcher_node")?;
    let mut p = ParamStore::new();
    let d = OdometryConfig::default();

    let scan_topic = p.string(
        &node,
        "topics.scan",
        "scan",
        "Laser scan input (sensor QoS)",
    )?;
    let registration_topic = p.string(
        &node,
        "topics.registration",
        "registration",
        "Relative-pose registration output (PoseWithCovarianceStamped, reliable)",
    )?;
    let matched_topic = p.string(
        &node,
        "topics.scan_matched",
        "scan_matched",
        "The registered scans, republished with reliable QoS for the pose graph",
    )?;

    let mut cfg = d;
    cfg.downsample = p.f64(
        &node,
        "front_end.downsample_m",
        d.downsample,
        0.005,
        1.0,
        "Voxel size of the scan downsampling (m)",
    )?;
    cfg.map_cell = p.f64(
        &node,
        "front_end.map_cell_m",
        d.map_cell,
        0.01,
        2.0,
        "Cell of the local point map (m)",
    )?;
    cfg.map_max_per_cell = p.i64(
        &node,
        "front_end.map_max_per_cell",
        d.map_max_per_cell as i64,
        1,
        1000,
        "Points kept per local-map cell",
    )? as usize;
    cfg.map_radius = p.f64(
        &node,
        "front_end.map_radius_m",
        d.map_radius,
        0.5,
        1e3,
        "Points farther than this from the sensor are dropped from the map (m)",
    )?;
    cfg.min_points = p.i64(
        &node,
        "front_end.min_points",
        d.min_points as i64,
        3,
        100_000,
        "Fewest downsampled points for a scan to be registered",
    )? as usize;
    cfg.fallback_sigma_trans = p.f64(
        &node,
        "front_end.fallback_sigma_trans_m",
        d.fallback_sigma_trans,
        1e-4,
        10.0,
        "Translation sigma reported for a predicted (unregistered) step (m)",
    )?;
    cfg.fallback_sigma_rot = p.f64(
        &node,
        "front_end.fallback_sigma_rot_rad",
        d.fallback_sigma_rot,
        1e-4,
        3.2,
        "Rotation sigma reported for a predicted step (rad)",
    )?;
    cfg.adaptive.initial_threshold = p.f64(
        &node,
        "adaptive.initial_threshold_m",
        d.adaptive.initial_threshold,
        0.01,
        10.0,
        "Correspondence threshold before any sample, and the retry threshold (m)",
    )?;
    cfg.adaptive.min_motion = p.f64(
        &node,
        "adaptive.min_motion_m",
        d.adaptive.min_motion,
        1e-4,
        1.0,
        "Prediction errors below this do not enter the threshold (m)",
    )?;
    cfg.adaptive.min_threshold = p.f64(
        &node,
        "adaptive.min_threshold_m",
        d.adaptive.min_threshold,
        0.001,
        10.0,
        "Lower bound of the adaptive threshold (m)",
    )?;
    cfg.adaptive.max_threshold = p.f64(
        &node,
        "adaptive.max_threshold_m",
        d.adaptive.max_threshold,
        0.01,
        50.0,
        "Upper bound of the adaptive threshold (m)",
    )?;
    cfg.icp.kernel_ratio = p.f64(
        &node,
        "icp.kernel_ratio",
        d.icp.kernel_ratio,
        0.01,
        1.0,
        "Geman-McClure scale as a fraction of the threshold",
    )?;
    cfg.icp.max_iterations = p.i64(
        &node,
        "icp.max_iterations",
        d.icp.max_iterations as i64,
        1,
        10_000,
        "ICP iteration limit",
    )? as usize;
    cfg.icp.trans_tol = p.f64(
        &node,
        "icp.trans_tol_m",
        d.icp.trans_tol,
        1e-9,
        1.0,
        "ICP convergence: translation step (m)",
    )?;
    cfg.icp.rot_tol = p.f64(
        &node,
        "icp.rot_tol_rad",
        d.icp.rot_tol,
        1e-9,
        1.0,
        "ICP convergence: rotation step (rad)",
    )?;
    cfg.icp.normal_radius = p.f64(
        &node,
        "icp.normal_radius_m",
        d.icp.normal_radius,
        0.01,
        5.0,
        "Neighbourhood of the target normal (m)",
    )?;
    cfg.icp.normal_min_points = p.i64(
        &node,
        "icp.normal_min_points",
        d.icp.normal_min_points as i64,
        3,
        1000,
        "Fewest neighbours for a normal",
    )? as usize;
    cfg.icp.planarity_ratio = p.f64(
        &node,
        "icp.planarity_ratio",
        d.icp.planarity_ratio,
        1e-4,
        1.0,
        "Largest smaller/larger eigenvalue ratio for a line",
    )?;
    cfg.icp.sigma_floor = p.f64(
        &node,
        "icp.sigma_floor_m",
        d.icp.sigma_floor,
        1e-5,
        1.0,
        "Floor on the residual sigma used for the information matrix (m)",
    )?;
    cfg.icp.info_inflation = p.f64(&node, "icp.info_inflation", d.icp.info_inflation, 0.01, 1e3, "Divides the information matrix; frozen from the development run (docs/phase-5-slam/benchmark.md section 6)")?;
    cfg.icp.ridge = p.f64(
        &node,
        "icp.ridge",
        d.icp.ridge,
        0.0,
        1.0,
        "Ridge of the normal equations and the information, relative to the trace",
    )?;
    cfg.icp.degeneracy_ratio = p.f64(
        &node,
        "icp.degeneracy_ratio",
        d.icp.degeneracy_ratio,
        1e-6,
        1.0,
        "Eigenvalue ratio below which a registration is flagged degenerate",
    )?;
    cfg.icp.min_correspondences = p.i64(
        &node,
        "icp.min_correspondences",
        d.icp.min_correspondences as i64,
        3,
        100_000,
        "Fewest correspondences for a result",
    )? as usize;

    let csv_path = p.string(
        &node,
        "output.registrations_csv",
        "none",
        "Per-scan registration CSV (none: off)",
    )?;
    let csv_path = if csv_path == "none" {
        String::new()
    } else {
        csv_path
    };
    let diag_period = p.i64(
        &node,
        "diagnostics_period_s",
        1,
        1,
        60,
        "Diagnostics period (s)",
    )?;

    let odo = LidarOdometry::new(cfg)
        .map_err(|e| invalid(&node, &format!("front-end parameters: {e}")))?;
    let mut csv = open(&csv_path)?;
    if let Some(w) = csv.as_mut() {
        let _ = writeln!(
            w,
            "stamp_ns,status,retried,threshold_m,iterations,correspondences,rmse_m,degenerate,dx,dy,dtheta,latency_us"
        );
    }
    let diagnostics = DiagnosticsPublisher::new(&node)?;
    let reliable = rclrs::QoSProfile::default()
        .reliable()
        .volatile()
        .keep_last(200);
    let shared = Shared::new(State {
        odo,
        c: Counters::default(),
        last_error: String::new(),
        last_threshold: cfg.adaptive.initial_threshold,
        latencies_us: VecDeque::new(),
        registration_pub: node.create_publisher::<geometry_msgs::msg::PoseWithCovarianceStamped>(
            registration_topic.as_str().qos(reliable),
        )?,
        matched_pub: node.create_publisher::<sensor_msgs::msg::LaserScan>(
            matched_topic.as_str().qos(reliable),
        )?,
        csv,
    });

    let n = Arc::clone(&node);
    let _scan_sub = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        scan_topic.as_str().qos(qos::sensor_data()),
        {
            let shared = shared.clone();
            let stop = Arc::clone(&shutdown);
            move |msg: sensor_msgs::msg::LaserScan| {
                // After SIGINT the executor still hands over the scans already queued; a front end
                // that has lost track spends seconds on each (debug build), so drop them instead of
                // working through them: the node stops after the step in progress.
                if stop.load(Ordering::SeqCst) {
                    return;
                }
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
                let t0 = Instant::now();
                let step = match st.odo.step(&scan) {
                    Ok(s) => s,
                    Err(e) => {
                        st.c.skipped_few_points += 1;
                        st.last_error = format!("registration: {e}");
                        log_warn!(
                            n.logger().throttle(Duration::from_secs(5)),
                            "scan not registered: {e}"
                        );
                        return;
                    }
                };
                let latency_us = t0.elapsed().as_secs_f64() * 1e6;
                let frame = FrameId::new(&msg.header.frame_id);
                let reg =
                    match registration_to_ros(scan.stamp, &frame, &step.delta, &step.information) {
                        Ok(m) => m,
                        Err(e) => {
                            st.c.publish_errors += 1;
                            st.last_error = format!("registration message: {e}");
                            return;
                        }
                    };
                // The scan first, so that the pose graph normally has it when the registration arrives.
                let sent = st
                    .matched_pub
                    .publish(&msg)
                    .and_then(|_| st.registration_pub.publish(&reg));
                if let Err(e) = sent {
                    st.c.publish_errors += 1;
                    st.last_error = format!("publish: {e}");
                    return;
                }
                st.c.published += 1;
                match step.status {
                    TrackingStatus::Initialized => st.c.initialized += 1,
                    TrackingStatus::Tracked => st.c.tracked += 1,
                    TrackingStatus::Predicted => st.c.predicted += 1,
                }
                st.c.retried += u64::from(step.retried);
                st.c.information_fallback +=
                    u64::from(step.information_fallback && step.status == TrackingStatus::Tracked);
                st.last_threshold = step.threshold;
                st.latencies_us.push_back(latency_us);
                while st.latencies_us.len() > 2000 {
                    st.latencies_us.pop_front();
                }
                if let Some(w) = st.csv.as_mut() {
                    let (it, corr, rmse, deg) = step
                        .icp
                        .map(|i| (i.iterations, i.correspondences, i.rmse, i.degenerate))
                        .unwrap_or((0, 0, 0.0, false));
                    let _ = writeln!(
                        w,
                        "{},{:?},{},{},{},{},{},{},{},{},{},{:.0}",
                        scan.stamp.as_nanos(),
                        step.status,
                        step.retried,
                        step.threshold,
                        it,
                        corr,
                        rmse,
                        deg,
                        step.delta.x(),
                        step.delta.y(),
                        step.delta.theta(),
                        latency_us
                    );
                }
            }
        },
    )?;

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
            let (level, message) = if st.c.publish_errors > 0 {
                // A scan that was registered but not published is an increment the pose graph will
                // never see: its chain of increments is broken from here on.
                (
                    DiagnosticLevel::Error,
                    "PUBLISH ERRORS: the chain of increments is broken".to_string(),
                )
            } else if st.c.published == 0 {
                (
                    DiagnosticLevel::Warn,
                    "WAITING: no scan registered yet".to_string(),
                )
            } else if st.c.published >= 20 && st.c.predicted * 10 > st.c.published {
                (
                    DiagnosticLevel::Warn,
                    "MANY STEPS PREDICTED: registrations are failing".to_string(),
                )
            } else {
                (DiagnosticLevel::Ok, "RUNNING".to_string())
            };
            let mut b = StatusBuilder::new("scan_matcher_node", level, &message)
                .value("scans_received", st.c.received)
                .value("registrations_published", st.c.published)
                .value("initialized", st.c.initialized)
                .value("tracked", st.c.tracked)
                .value("predicted", st.c.predicted)
                .value("retried_at_wide_threshold", st.c.retried)
                .value(
                    "tracked_with_fallback_information",
                    st.c.information_fallback,
                )
                .value("skipped_malformed", st.c.skipped_malformed)
                .value("skipped_few_points", st.c.skipped_few_points)
                .value("publish_errors", st.c.publish_errors)
                .value("threshold_m", format!("{:.4}", st.last_threshold))
                .value("inputs", "scan only, no odometry")
                .value("tf_published", "none")
                .value("last_error", &st.last_error);
            if let (Some(p50), Some(p95), Some(p99)) = (
                st.percentile_us(0.5),
                st.percentile_us(0.95),
                st.percentile_us(0.99),
            ) {
                b = b
                    .value("registration_latency_us_p50", format!("{p50:.0}"))
                    .value("registration_latency_us_p95", format!("{p95:.0}"))
                    .value("registration_latency_us_p99", format!("{p99:.0}"));
            }
            if let Err(e) = diagnostics.publish(stamp, vec![b.build()]) {
                log_error!(n.logger(), "diagnostics publish failed: {e}");
            }
        }
    })?;

    log_info!(
        node.logger(),
        "scan_matcher_node up: {} -> {} / {}, scan only (no odometry input), no TF, {} parameters, info_inflation {}",
        scan_topic,
        registration_topic,
        matched_topic,
        p.len(),
        cfg.icp.info_inflation
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    {
        let mut guard = shared.lock();
        let st: &mut State = &mut guard;
        if let Some(w) = st.csv.as_mut() {
            let _ = w.flush();
        }
        log_info!(
            node.logger(),
            "scan_matcher_node shutting down: {} scans received, {} registrations published ({} tracked, {} predicted, {} retried wide, {} tracked with fallback information), {} not registered",
            st.c.received,
            st.c.published,
            st.c.tracked,
            st.c.predicted,
            st.c.retried,
            st.c.information_fallback,
            st.c.skipped_few_points
        );
    }
    drop(p);
    result
}
