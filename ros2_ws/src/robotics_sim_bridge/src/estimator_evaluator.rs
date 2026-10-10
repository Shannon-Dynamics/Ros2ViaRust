//! `estimator_evaluator` — compares odometry streams against simulator ground truth.
//!
//! Subscribes to `ground_truth/odom`, the raw wheel odometry and the fused odometry,
//! matches each estimate to the ground-truth sample with the same stamp (±`match_tolerance`),
//! and reports running position / yaw RMSE and current errors for both streams on
//! `/diagnostics` and in the log. Evaluation only: nothing here feeds back into the stack.

use rclrs::*;
use robotics_core::angle::angle_diff;
use robotics_core::{Se2, Timestamp};
use robotics_ros_bridge::convert::{now, odometry_from_ros};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::nav_msgs;
use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Duration;

#[derive(Default, Clone, Copy)]
struct Errors {
    n: u64,
    sum_pos2: f64,
    sum_yaw2: f64,
    last_pos: f64,
    last_yaw: f64,
}

impl Errors {
    fn add(&mut self, est: &Se2, truth: &Se2) {
        let ep = est.distance_to(truth);
        let ey = angle_diff(est.theta(), truth.theta());
        self.n += 1;
        self.sum_pos2 += ep * ep;
        self.sum_yaw2 += ey * ey;
        self.last_pos = ep;
        self.last_yaw = ey;
    }
    fn rmse(&self) -> (f64, f64) {
        let n = self.n.max(1) as f64;
        ((self.sum_pos2 / n).sqrt(), (self.sum_yaw2 / n).sqrt())
    }
}

struct Eval {
    truth: VecDeque<(Timestamp, Se2)>,
    tolerance: f64,
    raw: Errors,
    fused: Errors,
    unmatched: u64,
}

impl Eval {
    fn lookup(&self, t: Timestamp) -> Option<Se2> {
        self.truth
            .iter()
            .min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs()))
            .filter(|(s, _)| (*s - t).abs() <= self.tolerance)
            .map(|(_, p)| *p)
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("estimator_evaluator")?;
    let mut p = ParamStore::new();
    let truth_topic = p.string(
        &node,
        "topics.ground_truth",
        "ground_truth/odom",
        "Ground truth odometry",
    )?;
    let raw_topic = p.string(&node, "topics.raw", "wheel/odometry", "Raw wheel odometry")?;
    let fused_topic = p.string(&node, "topics.fused", "odom", "Fused odometry")?;
    let tolerance = p.f64(
        &node,
        "match_tolerance",
        0.006,
        0.0,
        1.0,
        "Max stamp difference for truth matching (s)",
    )?;
    let report_s = p.i64(&node, "report_period_s", 5, 1, 600, "Report period (s)")?;

    let diagnostics = DiagnosticsPublisher::new(&node)?;
    let shared = Shared::new(Eval {
        truth: VecDeque::with_capacity(512),
        tolerance,
        raw: Errors::default(),
        fused: Errors::default(),
        unmatched: 0,
    });

    let _truth = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        truth_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |m: nav_msgs::msg::Odometry| {
                let mut guard = shared.lock();
                let e: &mut Eval = &mut guard;
                {
                    if let Ok(o) = odometry_from_ros(&m) {
                        e.truth.push_back((o.stamp, o.pose));
                        while e.truth.len() > 500 {
                            e.truth.pop_front();
                        }
                    }
                }
            }
        },
    )?;
    let _raw = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        raw_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |m: nav_msgs::msg::Odometry| {
                let mut guard = shared.lock();
                let e: &mut Eval = &mut guard;
                {
                    if let Ok(o) = odometry_from_ros(&m) {
                        match e.lookup(o.stamp) {
                            Some(t) => e.raw.add(&o.pose, &t),
                            None => e.unmatched += 1,
                        }
                    }
                }
            }
        },
    )?;
    let _fused = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        fused_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |m: nav_msgs::msg::Odometry| {
                let mut guard = shared.lock();
                let e: &mut Eval = &mut guard;
                {
                    if let Ok(o) = odometry_from_ros(&m) {
                        match e.lookup(o.stamp) {
                            Some(t) => e.fused.add(&o.pose, &t),
                            None => e.unmatched += 1,
                        }
                    }
                }
            }
        },
    )?;

    let n = Arc::clone(&node);
    let _report = node.create_timer_repeating(Duration::from_secs(report_s as u64), { let shared = shared.clone(); move || { let mut guard = shared.lock(); let e: &mut Eval = &mut guard; {
        let (rp, ry) = e.raw.rmse();
        let (fp, fy) = e.fused.rmse();
        log_info!(
            n.logger(),
            "vs ground truth | raw wheel: {} samples, pos RMSE {:.3} m, yaw RMSE {:.4} rad, now {:.3} m / {:.4} rad | fused: {} samples, pos RMSE {:.3} m, yaw RMSE {:.4} rad, now {:.3} m / {:.4} rad | unmatched {}",
            e.raw.n, rp, ry, e.raw.last_pos, e.raw.last_yaw, e.fused.n, fp, fy, e.fused.last_pos, e.fused.last_yaw, e.unmatched
        );
        let level = if e.fused.n > 0 { DiagnosticLevel::Ok } else { DiagnosticLevel::Stale };
        let s = StatusBuilder::new("estimator_evaluator: accuracy", level, "errors vs simulator ground truth")
            .value("raw_samples", e.raw.n)
            .value("raw_pos_rmse_m", format!("{rp:.4}"))
            .value("raw_yaw_rmse_rad", format!("{ry:.5}"))
            .value("fused_samples", e.fused.n)
            .value("fused_pos_rmse_m", format!("{fp:.4}"))
            .value("fused_yaw_rmse_rad", format!("{fy:.5}"))
            .value("unmatched", e.unmatched)
            .build();
        match now(&n) {
            Ok(t) => {
                if let Err(err) = diagnostics.publish(t, vec![s]) {
                    log_error!(n.logger(), "diagnostics publish failed: {err}");
                }
            }
            Err(err) => log_error!(n.logger(), "clock error: {err}"),
        }
    } } }
    )?;

    log_info!(
        node.logger(),
        "estimator_evaluator up: truth={truth_topic} raw={raw_topic} fused={fused_topic}"
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "estimator_evaluator shutting down");
    drop(p);
    result
}
