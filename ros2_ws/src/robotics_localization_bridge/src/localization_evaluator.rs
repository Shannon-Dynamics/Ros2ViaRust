//! `localization_evaluator` — compares a pose estimate against simulator ground truth.
//!
//! ```text
//!   geometry_msgs/PoseStamped               (topics.ground_truth) ─┐
//!   geometry_msgs/PoseWithCovarianceStamped (topics.estimate) ─────┼─► per sample row (CSV)
//!   nav_msgs/Odometry                       (topics.odom) ─────────┘   + summary (JSON)
//!   geometry_msgs/PoseStamped               (topics.kidnap_event) ──► marks the rows after it
//! ```
//!
//! This node exists so that no estimator has to grade itself. It is the only node that
//! subscribes to ground truth; `localization_node` never sees it.
//!
//! Every estimate is paired with the ground truth sample closest in time, within
//! `pair_tolerance_s`, and one row is written per pair. Raw odometry is carried along in
//! the same row as the dead reckoning baseline the filter has to beat. The summary is
//! written once, at shutdown, from the rows that were actually paired. No number is
//! computed anywhere else: the CSV is the measurement and the JSON is its summary.

use rclrs::*;
use robotics_core::{Se2, Timestamp};
use robotics_ros_bridge::convert::{odometry_from_ros, pose_from_ros};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::{geometry_msgs, nav_msgs};
use std::io::Write;
use std::sync::Arc;

fn stamp_of(h: &ros_env::std_msgs::msg::Header) -> Timestamp {
    Timestamp::from_sec_nanosec(h.stamp.sec, h.stamp.nanosec)
}

/// Wrap an angle difference into `(-pi, pi]`.
fn angle_error(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > std::f64::consts::PI {
        d -= std::f64::consts::TAU;
    }
    while d <= -std::f64::consts::PI {
        d += std::f64::consts::TAU;
    }
    d
}

#[derive(Default)]
struct Accumulator {
    samples: u64,
    sum_sq_pos: f64,
    sum_sq_yaw: f64,
    max_pos: f64,
    sum_sq_odom_pos: f64,
    last_pos: f64,
    last_yaw: f64,
}

impl Accumulator {
    fn push(&mut self, pos_err: f64, yaw_err: f64, odom_pos_err: Option<f64>) {
        self.samples += 1;
        self.sum_sq_pos += pos_err * pos_err;
        self.sum_sq_yaw += yaw_err * yaw_err;
        self.max_pos = self.max_pos.max(pos_err);
        if let Some(o) = odom_pos_err {
            self.sum_sq_odom_pos += o * o;
        }
        self.last_pos = pos_err;
        self.last_yaw = yaw_err;
    }
    fn rmse_pos(&self) -> Option<f64> {
        (self.samples > 0).then(|| (self.sum_sq_pos / self.samples as f64).sqrt())
    }
    fn rmse_yaw(&self) -> Option<f64> {
        (self.samples > 0).then(|| (self.sum_sq_yaw / self.samples as f64).sqrt())
    }
    fn rmse_odom_pos(&self) -> Option<f64> {
        (self.samples > 0).then(|| (self.sum_sq_odom_pos / self.samples as f64).sqrt())
    }
}

struct State {
    csv: Option<std::io::BufWriter<std::fs::File>>,
    csv_path: String,
    json_path: String,
    tolerance: f64,
    ground_truth: Option<(Timestamp, Se2)>,
    odom: Option<(Timestamp, Se2)>,
    kidnap: Option<(Timestamp, Se2)>,
    rows: u64,
    unpaired: u64,
    estimates: u64,
    before: Accumulator,
    after: Accumulator,
    first_stamp: Option<Timestamp>,
    /// Smallest position error reached after the kidnap, and when.
    best_after: Option<(f64, f64)>,
}

impl State {
    fn write_row(&mut self, node: &Node, stamp: Timestamp, est: &Se2, est_sigma_xy: f64) {
        self.estimates += 1;
        let Some((gt_stamp, gt)) = self.ground_truth else {
            self.unpaired += 1;
            return;
        };
        let dt = (stamp - gt_stamp).abs();
        if dt > self.tolerance {
            self.unpaired += 1;
            return;
        }
        let pos_err = ((est.x() - gt.x()).powi(2) + (est.y() - gt.y()).powi(2)).sqrt();
        let yaw_err = angle_error(est.theta(), gt.theta());
        let odom = self.odom.map(|(_, o)| o);
        let odom_err =
            odom.map(|o| ((o.x() - gt.x()).powi(2) + (o.y() - gt.y()).powi(2)).sqrt());
        let kidnapped = self.kidnap.map(|(k, _)| stamp >= k).unwrap_or(false);

        if self.first_stamp.is_none() {
            self.first_stamp = Some(stamp);
        }
        let t = stamp - self.first_stamp.unwrap();
        if kidnapped {
            self.after.push(pos_err, yaw_err, odom_err);
            let better = self.best_after.map(|(b, _)| pos_err < b).unwrap_or(true);
            if better {
                self.best_after = Some((pos_err, t));
            }
        } else {
            self.before.push(pos_err, yaw_err, odom_err);
        }

        if let Some(w) = self.csv.as_mut() {
            let o = odom.unwrap_or(Se2::IDENTITY);
            let line = format!(
                "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{}\n",
                stamp.as_secs_f64(),
                t,
                gt.x(),
                gt.y(),
                gt.theta(),
                est.x(),
                est.y(),
                est.theta(),
                pos_err,
                yaw_err,
                o.x(),
                o.y(),
                o.theta(),
                odom_err.unwrap_or(f64::NAN),
                est_sigma_xy,
                u8::from(kidnapped),
            );
            if let Err(e) = w.write_all(line.as_bytes()) {
                log_error!(node.logger(), "cannot write {}: {e}", self.csv_path);
            }
            self.rows += 1;
        }
    }

    fn write_summary(&mut self, node: &Node) {
        if let Some(w) = self.csv.as_mut() {
            let _ = w.flush();
        }
        let f = |v: Option<f64>| match v {
            Some(x) if x.is_finite() => format!("{x:.6}"),
            _ => "null".to_string(),
        };
        let kidnap = match self.kidnap {
            Some((t, p)) => format!(
                "{{\"stamp_s\": {:.6}, \"seconds_into_run\": {:.6}, \"target\": {{\"x\": {:.6}, \"y\": {:.6}, \"theta\": {:.6}}}}}",
                t.as_secs_f64(),
                t - self.first_stamp.unwrap_or(t),
                p.x(),
                p.y(),
                p.theta()
            ),
            None => "null".to_string(),
        };
        let best_after = match self.best_after {
            Some((e, t)) => format!(
                "{{\"position_error_m\": {e:.6}, \"seconds_into_run\": {t:.6}}}"
            ),
            None => "null".to_string(),
        };
        let json = format!(
            "{{\n  \"csv\": \"{}\",\n  \"estimates_received\": {},\n  \"rows\": {},\n  \
             \"unpaired_estimates\": {},\n  \"pair_tolerance_s\": {},\n  \
             \"kidnap\": {},\n  \
             \"before_kidnap\": {{\"samples\": {}, \"rmse_position_m\": {}, \"rmse_yaw_rad\": {}, \
             \"max_position_error_m\": {}, \"final_position_error_m\": {}, \
             \"odom_only_rmse_position_m\": {}}},\n  \
             \"after_kidnap\": {{\"samples\": {}, \"rmse_position_m\": {}, \"rmse_yaw_rad\": {}, \
             \"max_position_error_m\": {}, \"final_position_error_m\": {}, \
             \"odom_only_rmse_position_m\": {}, \"best_position_error_after_kidnap\": {}}}\n}}\n",
            self.csv_path,
            self.estimates,
            self.rows,
            self.unpaired,
            self.tolerance,
            kidnap,
            self.before.samples,
            f(self.before.rmse_pos()),
            f(self.before.rmse_yaw()),
            f((self.before.samples > 0).then_some(self.before.max_pos)),
            f((self.before.samples > 0).then_some(self.before.last_pos)),
            f(self.before.rmse_odom_pos()),
            self.after.samples,
            f(self.after.rmse_pos()),
            f(self.after.rmse_yaw()),
            f((self.after.samples > 0).then_some(self.after.max_pos)),
            f((self.after.samples > 0).then_some(self.after.last_pos)),
            f(self.after.rmse_odom_pos()),
            best_after,
        );
        match std::fs::write(&self.json_path, json) {
            Ok(()) => log_info!(node.logger(), "summary written to {}", self.json_path),
            Err(e) => log_error!(node.logger(), "cannot write {}: {e}", self.json_path),
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("localization_evaluator")?;
    let mut p = ParamStore::new();

    let gt_topic = p.string(
        &node,
        "topics.ground_truth",
        "ground_truth/pose",
        "Ground truth pose from the simulator (geometry_msgs/PoseStamped)",
    )?;
    let est_topic = p.string(
        &node,
        "topics.estimate",
        "localization/pose",
        "Pose estimate under test (geometry_msgs/PoseWithCovarianceStamped)",
    )?;
    let odom_topic = p.string(&node, "topics.odom", "odom", "Dead reckoning baseline")?;
    let kidnap_topic = p.string(
        &node,
        "topics.kidnap_event",
        "kidnap/events",
        "Teleport events (geometry_msgs/PoseStamped), used only to split the run",
    )?;
    let csv_path = p.string(&node, "output_csv", "", "Where to write the per sample rows")?;
    let json_path = p.string(&node, "output_json", "", "Where to write the summary")?;
    let tolerance = p.f64(
        &node,
        "pair_tolerance_s",
        0.05,
        1e-4,
        5.0,
        "Largest time difference between an estimate and the ground truth it is paired with",
    )?;

    if csv_path.is_empty() || json_path.is_empty() {
        log_error!(node.logger(), "output_csv and output_json are required");
        return Err(RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        });
    }
    let file = std::fs::File::create(&csv_path).map_err(|e| {
        log_error!(node.logger(), "cannot create {csv_path}: {e}");
        RclrsError::RclError {
            code: RclReturnCode::Error,
            msg: None,
        }
    })?;
    let mut csv = std::io::BufWriter::new(file);
    csv.write_all(
        b"stamp_s,t_s,gt_x,gt_y,gt_theta,est_x,est_y,est_theta,position_error_m,yaw_error_rad,\
odom_x,odom_y,odom_theta,odom_position_error_m,est_sigma_xy_m,after_kidnap\n",
    )
    .ok();

    let shared = Shared::new(State {
        csv: Some(csv),
        csv_path: csv_path.clone(),
        json_path: json_path.clone(),
        tolerance,
        ground_truth: None,
        odom: None,
        kidnap: None,
        rows: 0,
        unpaired: 0,
        estimates: 0,
        before: Accumulator::default(),
        after: Accumulator::default(),
        first_stamp: None,
        best_after: None,
    });

    let _gt_sub = node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
        gt_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |msg: geometry_msgs::msg::PoseStamped| {
                if let Ok(pose) = pose_from_ros(&msg.pose) {
                    shared.lock().ground_truth = Some((stamp_of(&msg.header), pose));
                }
            }
        },
    )?;

    let _odom_sub = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        odom_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |msg: nav_msgs::msg::Odometry| {
                if let Ok(o) = odometry_from_ros(&msg) {
                    shared.lock().odom = Some((o.stamp, o.pose));
                }
            }
        },
    )?;

    let n = Arc::clone(&node);
    let _kidnap_sub = node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
        kidnap_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |msg: geometry_msgs::msg::PoseStamped| {
                if let Ok(pose) = pose_from_ros(&msg.pose) {
                    let stamp = stamp_of(&msg.header);
                    shared.lock().kidnap = Some((stamp, pose));
                    log_info!(
                        n.logger(),
                        "kidnap event at t={:.3}: robot moved to ({:.3}, {:.3}, {:.3})",
                        stamp.as_secs_f64(),
                        pose.x(),
                        pose.y(),
                        pose.theta()
                    );
                }
            }
        },
    )?;

    let n = Arc::clone(&node);
    let _est_sub = node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
        est_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
                let Ok(pose) = pose_from_ros(&msg.pose.pose) else {
                    return;
                };
                let sigma_xy = msg.pose.covariance[0].max(msg.pose.covariance[7]).sqrt();
                let stamp = stamp_of(&msg.header);
                shared.lock().write_row(&n, stamp, &pose, sigma_xy);
            }
        },
    )?;

    log_info!(
        node.logger(),
        "localization_evaluator up: {} vs {} (baseline {}), rows -> {}, summary -> {}",
        est_topic,
        gt_topic,
        odom_topic,
        csv_path,
        json_path
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    {
        let mut guard = shared.lock();
        guard.write_summary(&node);
    }
    log_info!(node.logger(), "localization_evaluator shutting down");
    drop(p);
    result
}
