//! `ground_truth_tf_adapter` — SIMULATION ONLY. Publishes `map -> odom` from the simulator's
//! ground truth, so that the mapper can be tested before, and apart from, any estimator.
//!
//! ```text
//!   geometry_msgs/PoseStamped (topics.ground_truth, frame `map`) ─┐
//!   /tf  odom -> base_link   (diff_drive_controller, at that stamp) ┴─► /tf  map -> odom
//!                                                                    + /diagnostics
//! ```
//!
//! `map -> odom = T_map_base * inverse(T_odom_base)`. The adapter publishes `map -> odom` and
//! not `map -> base_link` because `base_link` already has a parent, `odom`, and a TF frame has
//! exactly one; the chain `map -> odom -> base_link` then reproduces the ground-truth pose at
//! every stamp where `odom -> base_link` is known (decision D2 of
//! `docs/phase-4-mapping/implementation.md`).
//!
//! **The name says what it is, and a launch file that starts it for a real robot is a bug.**
//! It is the only owner of `map -> odom` in its run, so it must never run together with
//! `localization_node`; `TfListener`'s authority conflict detection is reported in its
//! diagnostics so that a run with two owners says so. Like `localization_evaluator` and
//! `mapping_evaluator`, it is allowed to read `/ground_truth/pose`; `mapping_node` is not.
//!
//! A ground-truth sample is published only once `odom -> base_link` can be looked up at its
//! own stamp, so the correction is never built from a transform of a different instant. A
//! sample whose transform has not arrived is kept for a short while (the two streams travel
//! separately), and one that never gets it is dropped and counted.

use rclrs::*;
use robotics_core::tf::{LookupTime, TransformBuffer, TransformBufferConfig};
use robotics_core::{FrameId, Se2, Timestamp};
use robotics_ros_bridge::convert::{now, pose_from_ros};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::tf::{TfBroadcaster, TfListener};
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::geometry_msgs;
use std::collections::VecDeque;
use std::sync::Arc;
use std::time::Duration;

struct State {
    map_frame: FrameId,
    odom_frame: FrameId,
    base_frame: FrameId,
    max_wait: f64,
    pending: VecDeque<(Timestamp, Se2)>,
    newest_stamp: Option<Timestamp>,
    tf: TfBroadcaster,
    buffer: Shared<TransformBuffer>,
    received: u64,
    published: u64,
    dropped_no_odom_tf: u64,
    rejected_frame: u64,
    rejected_pose: u64,
    last_error: String,
}

impl State {
    /// Publish every pending sample whose `odom -> base_link` is available, oldest first. A
    /// sample that cannot be resolved blocks the ones behind it (they are newer) until it is
    /// resolved or older than `max_wait` relative to the newest ground truth, then dropped.
    fn flush(&mut self, node: &Node) {
        while let Some(&(stamp, truth)) = self.pending.front() {
            let looked_up = self.buffer.lock().lookup_transform(
                &self.odom_frame,
                &self.base_frame,
                LookupTime::At(stamp),
            );
            match looked_up {
                Ok(t) => {
                    // T_map_odom = T_map_base * T_odom_base^-1
                    let correction = truth.compose(&t.transform.inverse());
                    match self
                        .tf
                        .send(stamp, &self.map_frame, &self.odom_frame, &correction)
                    {
                        Ok(()) => self.published += 1,
                        Err(e) => {
                            self.last_error = format!("map -> odom broadcast: {e}");
                            log_error!(
                                node.logger().throttle(Duration::from_secs(5)),
                                "map -> odom broadcast failed: {e}"
                            );
                        }
                    }
                    self.pending.pop_front();
                }
                Err(e) => {
                    let age = self.newest_stamp.map_or(0.0, |n| n - stamp);
                    if age > self.max_wait {
                        self.dropped_no_odom_tf += 1;
                        self.last_error = format!("odom -> base_link at {stamp:?}: {e}");
                        self.pending.pop_front();
                    } else {
                        break;
                    }
                }
            }
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("ground_truth_tf_adapter")?;
    let mut p = ParamStore::new();

    let gt_topic = p.string(
        &node,
        "topics.ground_truth",
        "ground_truth/pose",
        "Ground truth pose of base_link in the map frame (geometry_msgs/PoseStamped)",
    )?;
    let map_frame = p.string(&node, "frames.map", "map", "Frame of the ground truth pose")?;
    let odom_frame = p.string(
        &node,
        "frames.odom",
        "odom",
        "Odometry frame; the child of the `map -> odom` edge this node owns",
    )?;
    let base_frame = p.string(&node, "frames.base_link", "base_link", "Body frame")?;
    let max_wait = p.f64(
        &node,
        "max_wait_s",
        0.5,
        0.0,
        10.0,
        "How long a sample waits for odom -> base_link at its stamp before it is dropped (s)",
    )?;
    let diag_period = p.i64(
        &node,
        "diagnostics_period_s",
        1,
        1,
        60,
        "Diagnostics period (s)",
    )?;

    let diagnostics = DiagnosticsPublisher::new(&node)?;
    let buffer = Shared::new(
        TransformBuffer::new(TransformBufferConfig::default()).expect("default TF buffer config"),
    );
    let tf_listener = Arc::new(TfListener::new(&node, buffer.clone())?);
    let shared = Shared::new(State {
        map_frame: FrameId::new(&map_frame),
        odom_frame: FrameId::new(&odom_frame),
        base_frame: FrameId::new(&base_frame),
        max_wait,
        pending: VecDeque::new(),
        newest_stamp: None,
        tf: TfBroadcaster::new(&node)?,
        buffer,
        received: 0,
        published: 0,
        dropped_no_odom_tf: 0,
        rejected_frame: 0,
        rejected_pose: 0,
        last_error: String::new(),
    });

    let n = Arc::clone(&node);
    let _gt_sub = node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
        gt_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |msg: geometry_msgs::msg::PoseStamped| {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                st.received += 1;
                if msg.header.frame_id != st.map_frame.as_str() {
                    st.rejected_frame += 1;
                    st.last_error = format!(
                        "ground truth in frame {:?}, expected {:?}",
                        msg.header.frame_id,
                        st.map_frame.as_str()
                    );
                    return;
                }
                let truth = match pose_from_ros(&msg.pose) {
                    Ok(t) => t,
                    Err(e) => {
                        st.rejected_pose += 1;
                        st.last_error = format!("ground truth pose: {e}");
                        return;
                    }
                };
                let stamp =
                    Timestamp::from_sec_nanosec(msg.header.stamp.sec, msg.header.stamp.nanosec);
                st.newest_stamp = Some(stamp);
                st.pending.push_back((stamp, truth));
                while st.pending.len() > 500 {
                    st.pending.pop_front();
                    st.dropped_no_odom_tf += 1;
                }
                st.flush(&n);
            }
        },
    )?;

    // Retry on a timer too: the transform for the newest sample often arrives a moment after
    // the sample, and no further ground truth message may come to flush it.
    let n = Arc::clone(&node);
    let _retry = node.create_timer_repeating(Duration::from_millis(10), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            st.flush(&n);
        }
    })?;

    let n = Arc::clone(&node);
    let listener = Arc::clone(&tf_listener);
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
            let conflicts = listener.authority_conflicts();
            let (level, message) = if !conflicts.is_empty() {
                (
                    DiagnosticLevel::Error,
                    format!("TF AUTHORITY CONFLICT on {conflicts:?}: two owners of one edge"),
                )
            } else if st.published == 0 {
                (DiagnosticLevel::Warn, "WAITING".to_string())
            } else {
                (DiagnosticLevel::Ok, "RUNNING".to_string())
            };
            let b = StatusBuilder::new("ground_truth_tf_adapter", level, &message)
                .hardware_id("simulation")
                .value("provenance", "ground_truth")
                .value(
                    "owns_edge",
                    format!("{} -> {}", st.map_frame, st.odom_frame),
                )
                .value("received", st.received)
                .value("published", st.published)
                .value("pending", st.pending.len())
                .value("dropped_no_odom_tf", st.dropped_no_odom_tf)
                .value("rejected_frame", st.rejected_frame)
                .value("rejected_pose", st.rejected_pose)
                .value("authority_conflicts", conflicts.len())
                .value("last_error", &st.last_error);
            if let Err(e) = diagnostics.publish(stamp, vec![b.build()]) {
                log_error!(n.logger(), "diagnostics publish failed: {e}");
            }
        }
    })?;

    log_warn!(
        node.logger(),
        "ground_truth_tf_adapter up: SIMULATION ONLY. {} + odom -> base_link => {} -> {} \
         from ground truth; do not run it with an estimator that owns that edge",
        gt_topic,
        map_frame,
        odom_frame
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "ground_truth_tf_adapter shutting down");
    drop(p);
    result
}
