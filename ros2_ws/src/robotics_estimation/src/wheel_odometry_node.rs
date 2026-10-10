//! `wheel_odometry_node` — differential-drive dead reckoning.
//!
//! ```text
//!   sensor_msgs/JointState (joint_states)
//!        │  convert: joint names → wheel angles, header → Timestamp
//!        ▼
//!   robotics_models::WheelOdometry   (pure Rust, Chapter 4 kinematics)
//!        │  robotics_core::Odometry2
//!        ▼
//!   nav_msgs/Odometry (odom)  +  TF odom → base_link  +  /diagnostics
//! ```
//!
//! This node **owns the `odom → base_link` transform**. Localization/SLAM nodes own
//! `map → odom` and must never publish `odom → base_link`.
//!
//! Parameters (all read at startup)
//! * `wheel_radius` (m, default 0.033), `wheel_separation` (m, default 0.16)
//! * `left_joint`, `right_joint`: joint names to look up in the message
//! * `odom_frame` (default `odom`), `base_frame` (default `base_link`)
//! * `publish_tf` (bool, default true)
//! * `use_header_stamp` (bool, default true): stamp outputs with the joint-state stamp;
//!   when false (or the stamp is zero) the node clock is used
//! * `noise_trans_per_m`, `noise_lateral_per_m`, `noise_rot_per_rad`, `noise_rot_per_m`:
//!   growth of the published pose covariance (see `OdometryNoise`)
//! * `diagnostics_period_s` (int, default 1)

use rclrs::*;
use robotics_core::{FrameId, Timestamp};
use robotics_models::{DiffDriveParams, OdometryNoise, WheelOdometry};
use robotics_ros_bridge::convert::{now, odometry_to_ros, time_from_ros};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::{qos, TfBroadcaster};
use ros_env::{nav_msgs, sensor_msgs};
use std::sync::Arc;
use std::time::{Duration, Instant};

struct State {
    odom: WheelOdometry,
    left_joint: String,
    right_joint: String,
    odom_frame: FrameId,
    base_frame: FrameId,
    use_header_stamp: bool,
    publisher: Publisher<nav_msgs::msg::Odometry>,
    tf: Option<TfBroadcaster>,
    received: u64,
    published: u64,
    dropped: u64,
    last_input: Option<Instant>,
    window: (Instant, u64),
}

fn invalid(node: &Node, what: &str, e: impl std::fmt::Display) -> RclrsError {
    log_error!(node.logger(), "{what}: {e}");
    RclrsError::RclError {
        code: RclReturnCode::InvalidArgument,
        msg: None,
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("wheel_odometry_node")?;

    let wheel_radius: MandatoryParameter<f64> = node
        .declare_parameter("wheel_radius")
        .default(0.033)
        .mandatory()?;
    let wheel_separation: MandatoryParameter<f64> = node
        .declare_parameter("wheel_separation")
        .default(0.16)
        .mandatory()?;
    let left_joint: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("left_joint")
        .default("left_wheel_joint".into())
        .read_only()?;
    let right_joint: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("right_joint")
        .default("right_wheel_joint".into())
        .read_only()?;
    let odom_frame: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("odom_frame")
        .default("odom".into())
        .read_only()?;
    let base_frame: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("base_frame")
        .default("base_link".into())
        .read_only()?;
    let publish_tf: ReadOnlyParameter<bool> = node
        .declare_parameter("publish_tf")
        .default(true)
        .read_only()?;
    let use_header_stamp: ReadOnlyParameter<bool> = node
        .declare_parameter("use_header_stamp")
        .default(true)
        .read_only()?;
    let noise_trans_per_m: MandatoryParameter<f64> = node
        .declare_parameter("noise_trans_per_m")
        .default(0.02)
        .mandatory()?;
    let noise_lateral_per_m: MandatoryParameter<f64> = node
        .declare_parameter("noise_lateral_per_m")
        .default(0.01)
        .mandatory()?;
    let noise_rot_per_rad: MandatoryParameter<f64> = node
        .declare_parameter("noise_rot_per_rad")
        .default(0.02)
        .mandatory()?;
    let noise_rot_per_m: MandatoryParameter<f64> = node
        .declare_parameter("noise_rot_per_m")
        .default(0.01)
        .mandatory()?;
    let diagnostics_period_s: ReadOnlyParameter<i64> = node
        .declare_parameter("diagnostics_period_s")
        .default(1)
        .range(ParameterRange {
            lower: Some(1),
            upper: Some(60),
            step: None,
        })
        .read_only()?;

    let params = DiffDriveParams {
        wheel_radius: wheel_radius.get(),
        wheel_separation: wheel_separation.get(),
    };
    let noise = OdometryNoise {
        trans_per_m: noise_trans_per_m.get(),
        lateral_per_m: noise_lateral_per_m.get(),
        rot_per_rad: noise_rot_per_rad.get(),
        rot_per_m: noise_rot_per_m.get(),
        ..OdometryNoise::default()
    };
    let odom = WheelOdometry::new(params, noise)
        .map_err(|e| invalid(&node, "invalid odometry parameters", e))?;

    let publisher =
        node.create_publisher::<nav_msgs::msg::Odometry>("odom".qos(qos::odometry()))?;
    let tf = if publish_tf.get() {
        Some(TfBroadcaster::new(&node)?)
    } else {
        None
    };
    let diagnostics = DiagnosticsPublisher::new(&node)?;

    let shared = Shared::new(State {
        odom,
        left_joint: left_joint.get().to_string(),
        right_joint: right_joint.get().to_string(),
        odom_frame: FrameId::new(odom_frame.get()),
        base_frame: FrameId::new(base_frame.get()),
        use_header_stamp: use_header_stamp.get(),
        publisher,
        tf,
        received: 0,
        published: 0,
        dropped: 0,
        last_input: None,
        window: (Instant::now(), 0),
    });

    let node_for_sub = Arc::clone(&node);
    let _subscription = node.create_subscription::<sensor_msgs::msg::JointState, _>(
        "joint_states".qos(qos::sensor_data()),
        {
            let shared = shared.clone();
            move |msg: sensor_msgs::msg::JointState| {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                {
                    st.received += 1;
                    st.last_input = Some(Instant::now());
                    let logger = node_for_sub.logger();

                    let find = |name: &str| msg.name.iter().position(|n| n == name);
                    let (Some(li), Some(ri)) = (find(&st.left_joint), find(&st.right_joint)) else {
                        st.dropped += 1;
                        log_warn!(
                            logger.throttle(Duration::from_secs(5)),
                            "joint_states lacks '{}'/'{}' (has {:?})",
                            st.left_joint,
                            st.right_joint,
                            msg.name
                        );
                        return;
                    };
                    let (Some(&left), Some(&right)) = (msg.position.get(li), msg.position.get(ri))
                    else {
                        st.dropped += 1;
                        log_warn!(
                            logger.throttle(Duration::from_secs(5)),
                            "joint_states positions too short"
                        );
                        return;
                    };

                    let header_stamp = time_from_ros(&msg.header.stamp);
                    let stamp = if st.use_header_stamp && header_stamp != Timestamp::ZERO {
                        header_stamp
                    } else {
                        match now(&node_for_sub) {
                            Ok(t) => t,
                            Err(e) => {
                                st.dropped += 1;
                                log_error!(logger, "clock error: {e}");
                                return;
                            }
                        }
                    };

                    let Some(step) = st.odom.update_from_wheel_angles(left, right, stamp) else {
                        log_info!(
                            logger,
                            "wheel odometry initialized at L={left:.4} R={right:.4} rad"
                        );
                        return;
                    };
                    if step.dt < 0.0 {
                        log_warn!(
                            logger.throttle(Duration::from_secs(5)),
                            "joint_states went backwards in time by {:.3} s; integrating anyway",
                            -step.dt
                        );
                    }

                    let odom =
                        st.odom
                            .odometry(stamp, st.odom_frame.clone(), st.base_frame.clone());
                    match st.publisher.publish(odometry_to_ros(&odom)) {
                        Ok(()) => st.published += 1,
                        Err(e) => {
                            st.dropped += 1;
                            log_error!(logger, "odom publish failed: {e}");
                        }
                    }
                    if let Some(tf) = &st.tf {
                        if let Err(e) = tf.send(stamp, &st.odom_frame, &st.base_frame, &odom.pose) {
                            log_error!(logger, "tf publish failed: {e}");
                        }
                    }
                    log_info!(
                        logger.throttle(Duration::from_secs(5)),
                        "odom pose x={:.3} y={:.3} th={:.3} | v={:.3} w={:.3} | sigma_xy={:.4}",
                        odom.pose.x(),
                        odom.pose.y(),
                        odom.pose.theta(),
                        odom.twist.vx,
                        odom.twist.omega,
                        odom.pose_cov[(0, 0)].sqrt()
                    );
                }
            }
        },
    )?;

    let node_for_diag = Arc::clone(&node);
    let period = diagnostics_period_s.get() as u64;
    let _diag_timer = node.create_timer_repeating(Duration::from_secs(period), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            {
                let elapsed = st.window.0.elapsed().as_secs_f64().max(1e-9);
                let rate = (st.received - st.window.1) as f64 / elapsed;
                st.window = (Instant::now(), st.received);
                let stale = st
                    .last_input
                    .map(|t| t.elapsed() > Duration::from_secs(2))
                    .unwrap_or(true);
                let (level, message) = if stale {
                    (DiagnosticLevel::Stale, "no joint_states received recently")
                } else if st.dropped > 0 && st.dropped * 10 > st.received {
                    (DiagnosticLevel::Warn, "more than 10% of inputs dropped")
                } else {
                    (DiagnosticLevel::Ok, "integrating")
                };
                let pose = st.odom.pose();
                let status = StatusBuilder::new("wheel_odometry_node: odometry", level, message)
                    .value("input_rate_hz", format!("{rate:.1}"))
                    .value("received", st.received)
                    .value("published", st.published)
                    .value("dropped", st.dropped)
                    .value("x", format!("{:.3}", pose.x()))
                    .value("y", format!("{:.3}", pose.y()))
                    .value("theta", format!("{:.3}", pose.theta()))
                    .build();
                match now(&node_for_diag) {
                    Ok(stamp) => {
                        if let Err(e) = diagnostics.publish(stamp, vec![status]) {
                            log_error!(node_for_diag.logger(), "diagnostics publish failed: {e}");
                        }
                    }
                    Err(e) => log_error!(node_for_diag.logger(), "clock error: {e}"),
                }
            }
        }
    })?;

    log_info!(
        node.logger(),
        "wheel_odometry_node up: r={} m, l={} m, joints=[{}, {}], frames {} -> {}, tf={}",
        params.wheel_radius,
        params.wheel_separation,
        left_joint.get(),
        right_joint.get(),
        odom_frame.get(),
        base_frame.get(),
        publish_tf.get()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "wheel_odometry_node shutting down");
    result
}
