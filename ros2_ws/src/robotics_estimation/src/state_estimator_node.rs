//! `state_estimator_node` — wheel odometry + gyro fusion with the SE(2) ESKF.
//!
//! ```text
//!   nav_msgs/Odometry (topics.wheel_odom) ─┐  convert: stamp + body twist
//!                                          ├─► robotics_filters::OdomImuFusion (pure Rust)
//!   sensor_msgs/Imu  (topics.imu) ─────────┘  convert: strict planar ImuMeasurement
//!                                                   │
//!               nav_msgs/Odometry (topics.output_odom)
//!             + geometry_msgs/PoseWithCovarianceStamped (topics.state_estimate)
//!             + TF odom → base_link + /diagnostics
//! ```
//!
//! `/odom` and `/state_estimate` carry the same estimate at the same stamp: the first for
//! consumers that want the twist (controllers, Nav2), the second for those that want only
//! the pose with covariance (localization initialisation, RViz, loggers).
//!
//! This node **owns `odom → base_link`** in the Phase 2 stack. The raw wheel odometry
//! source must run with `publish_tf:=false` (see `robotics_bringup/launch/
//! estimator_sim.launch.xml`). All filter time comes from message stamps; the timer only
//! drives diagnostics. Timing policies (late, out-of-order, duplicate, gaps) are documented
//! in `robotics_filters::fusion` and `docs/STATE_ESTIMATION.md`.

use rclrs::*;
use robotics_core::{FrameId, Odometry2, Timestamp};
use robotics_filters::{FusionConfig, Health, OdomImuFusion, ProcessNoise};
use robotics_ros_bridge::convert::{
    imu_measurement_from_ros, now, odometry_from_ros, odometry_to_ros,
    pose_with_covariance_stamped_to_ros,
};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::{qos, ParamStore, TfBroadcaster};
use ros_env::{geometry_msgs, nav_msgs, sensor_msgs};
use std::sync::Arc;
use std::time::Duration;

struct State {
    fusion: OdomImuFusion,
    odom_frame: FrameId,
    base_frame: FrameId,
    use_imu_covariance: bool,
    min_publish_interval: f64,
    last_publish: Option<Timestamp>,
    publisher: Publisher<nav_msgs::msg::Odometry>,
    estimate_publisher: Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>,
    tf: Option<TfBroadcaster>,
    published: u64,
    conversion_rejects: u64,
    last_error: String,
    warned_child_frame: bool,
    last_health: &'static str,
    /// Running NIS sums of the wheel corrections: `[speed-only (1 DoF), wheel+gyro (2 DoF)]`.
    nis_sum: [f64; 2],
    nis_count: [u64; 2],
}

impl State {
    fn maybe_publish(&mut self, node: &Node) {
        let Some(est) = self.fusion.estimate() else {
            return;
        };
        if let Some(last) = self.last_publish {
            let since = est.stamp - last;
            if since >= 0.0 && since < self.min_publish_interval {
                return;
            }
        }
        let odom = Odometry2 {
            stamp: est.stamp,
            frame: self.odom_frame.clone(),
            child_frame: self.base_frame.clone(),
            pose: est.pose,
            pose_cov: est.pose_cov,
            twist: est.twist,
            twist_cov: est.twist_cov,
        };
        if let Err(e) = self.publisher.publish(odometry_to_ros(&odom)) {
            log_error!(
                node.logger().throttle(Duration::from_secs(5)),
                "odom publish failed: {e}"
            );
            return;
        }
        let estimate = pose_with_covariance_stamped_to_ros(
            est.stamp,
            &self.odom_frame,
            &est.pose,
            &est.pose_cov,
        );
        if let Err(e) = self.estimate_publisher.publish(estimate) {
            log_error!(
                node.logger().throttle(Duration::from_secs(5)),
                "state_estimate publish failed: {e}"
            );
        }
        if let Some(tf) = &self.tf {
            if let Err(e) = tf.send(est.stamp, &self.odom_frame, &self.base_frame, &est.pose) {
                log_error!(
                    node.logger().throttle(Duration::from_secs(5)),
                    "tf publish failed: {e}"
                );
            }
        }
        self.published += 1;
        self.last_publish = Some(est.stamp);
    }

    fn reject(&mut self, node: &Node, source: &str, err: impl std::fmt::Display) {
        self.last_error = format!("{source}: {err}");
        log_warn!(
            node.logger().throttle(Duration::from_secs(5)),
            "rejected {source} measurement: {err}"
        );
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("state_estimator_node")?;
    let mut p = ParamStore::new();

    let odom_frame = p.string(
        &node,
        "frames.odom",
        "odom",
        "Parent frame of the fused odometry",
    )?;
    let base_frame = p.string(&node, "frames.base_link", "base_link", "Body frame")?;
    let wheel_topic = p.string(
        &node,
        "topics.wheel_odom",
        "wheel/odometry",
        "Raw wheel odometry input (nav_msgs/Odometry)",
    )?;
    let imu_topic = p.string(
        &node,
        "topics.imu",
        "imu/data",
        "IMU input (sensor_msgs/Imu)",
    )?;
    let out_topic = p.string(&node, "topics.output_odom", "odom", "Fused odometry output")?;
    let estimate_topic = p.string(
        &node,
        "topics.state_estimate",
        "state_estimate",
        "Fused pose with covariance (geometry_msgs/PoseWithCovarianceStamped)",
    )?;
    let publish_tf = p.bool(
        &node,
        "publish_tf",
        true,
        "Broadcast odom -> base_link (exactly one node may)",
    )?;

    let d = FusionConfig::default();
    let pn = ProcessNoise::default();
    let process = ProcessNoise {
        gyro_noise_density: p.f64(
            &node,
            "process_noise.gyro_noise_density",
            pn.gyro_noise_density,
            0.0,
            1.0,
            "Gyro white noise density (rad/s/sqrt(Hz))",
        )?,
        accel_noise_density: p.f64(
            &node,
            "process_noise.accel_noise_density",
            pn.accel_noise_density,
            0.0,
            20.0,
            "Speed random walk density (m/s^2/sqrt(Hz))",
        )?,
        gyro_bias_random_walk: p.f64(
            &node,
            "process_noise.gyro_bias_random_walk",
            pn.gyro_bias_random_walk,
            0.0,
            0.1,
            "Gyro bias random walk (rad/s/sqrt(s))",
        )?,
        slip_diffusion: p.f64(
            &node,
            "process_noise.slip_diffusion",
            pn.slip_diffusion,
            0.0,
            1.0,
            "Planar slip diffusion (m/sqrt(s))",
        )?,
    };
    let cfg = FusionConfig {
        process,
        wheel_speed_sigma: p.f64(
            &node,
            "wheel_noise.speed_sigma",
            d.wheel_speed_sigma,
            1e-6,
            5.0,
            "Wheel speed noise (m/s)",
        )?,
        wheel_yaw_rate_sigma: p.f64(
            &node,
            "wheel_noise.yaw_rate_sigma",
            d.wheel_yaw_rate_sigma,
            1e-6,
            5.0,
            "Wheel yaw-rate noise at standstill (rad/s)",
        )?,
        wheel_yaw_rate_per_speed: p.f64(
            &node,
            "wheel_noise.yaw_rate_per_speed",
            d.wheel_yaw_rate_per_speed,
            0.0,
            10.0,
            "Wheel yaw-rate noise per unit speed (rad/m)",
        )?,
        wheel_yaw_rate_per_turn: p.f64(
            &node,
            "wheel_noise.yaw_rate_per_turn",
            d.wheel_yaw_rate_per_turn,
            0.0,
            1.0,
            "Wheel yaw-rate noise relative to yaw rate",
        )?,
        bias_update_max_speed: p.f64(
            &node,
            "wheel_noise.bias_update_max_speed",
            d.bias_update_max_speed,
            0.0,
            100.0,
            "Max |v| (m/s) at which wheel yaw rate updates the gyro bias",
        )?,
        gyro_sigma: p.f64(
            &node,
            "imu_noise.yaw_rate_sigma",
            d.gyro_sigma,
            1e-6,
            5.0,
            "Gyro per-sample sigma when the message has no covariance (rad/s)",
        )?,
        wheel_only_yaw_var_per_s: p.f64(
            &node,
            "wheel_noise.wheel_only_yaw_var_per_s",
            d.wheel_only_yaw_var_per_s,
            0.0,
            10.0,
            "Heading variance density without IMU (rad^2/s)",
        )?,
        initial_sigma_xy: p.f64(
            &node,
            "initial_covariance.sigma_xy",
            d.initial_sigma_xy,
            1e-6,
            100.0,
            "Initial position sigma (m)",
        )?,
        initial_sigma_theta: p.f64(
            &node,
            "initial_covariance.sigma_theta",
            d.initial_sigma_theta,
            1e-6,
            10.0,
            "Initial heading sigma (rad)",
        )?,
        initial_sigma_speed: p.f64(
            &node,
            "initial_covariance.sigma_speed",
            d.initial_sigma_speed,
            1e-6,
            10.0,
            "Initial speed sigma (m/s)",
        )?,
        initial_sigma_bias: p.f64(
            &node,
            "initial_covariance.sigma_bias",
            d.initial_sigma_bias,
            1e-6,
            1.0,
            "Initial gyro bias sigma (rad/s)",
        )?,
        max_dt: p.f64(
            &node,
            "max_dt",
            d.max_dt,
            1e-3,
            10.0,
            "Largest gap integrated in one step (s)",
        )?,
        max_measurement_delay: p.f64(
            &node,
            "max_measurement_delay",
            d.max_measurement_delay,
            0.0,
            1.0,
            "Largest accepted lateness (s)",
        )?,
        imu_timeout: p.f64(
            &node,
            "imu_timeout",
            d.imu_timeout,
            1e-3,
            10.0,
            "IMU staleness limit (s)",
        )?,
        wheel_timeout: p.f64(
            &node,
            "wheel_timeout",
            d.wheel_timeout,
            1e-3,
            10.0,
            "Wheel staleness limit (s)",
        )?,
        fault_hold: d.fault_hold,
    };
    let use_imu_covariance = p.bool(
        &node,
        "imu_noise.use_message_covariance",
        true,
        "Use the IMU message covariance when it is provided",
    )?;
    let max_rate = p.f64(
        &node,
        "max_publish_rate_hz",
        50.0,
        1.0,
        1000.0,
        "Upper bound on the fused odometry rate (Hz)",
    )?;
    let diag_period = p.i64(
        &node,
        "diagnostics_period_s",
        1,
        1,
        60,
        "Diagnostics period (s)",
    )?;

    let fusion = OdomImuFusion::new(cfg).map_err(|e| {
        log_error!(node.logger(), "invalid estimator configuration: {e}");
        RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        }
    })?;

    let publisher =
        node.create_publisher::<nav_msgs::msg::Odometry>(out_topic.as_str().qos(qos::odometry()))?;
    let estimate_publisher = node
        .create_publisher::<geometry_msgs::msg::PoseWithCovarianceStamped>(
            estimate_topic.as_str().qos(qos::odometry()),
        )?;
    let tf = if publish_tf {
        Some(TfBroadcaster::new(&node)?)
    } else {
        None
    };
    let diagnostics = DiagnosticsPublisher::new(&node)?;

    let shared = Shared::new(State {
        fusion,
        odom_frame: FrameId::new(&odom_frame),
        base_frame: FrameId::new(&base_frame),
        use_imu_covariance,
        min_publish_interval: 1.0 / max_rate,
        last_publish: None,
        publisher,
        estimate_publisher,
        tf,
        published: 0,
        conversion_rejects: 0,
        last_error: String::new(),
        warned_child_frame: false,
        last_health: "INITIALIZING",
        nis_sum: [0.0; 2],
        nis_count: [0; 2],
    });

    let n = Arc::clone(&node);
    let _imu_sub = node.create_subscription::<sensor_msgs::msg::Imu, _>(
        imu_topic.as_str().qos(qos::sensor_data()),
        {
            let shared = shared.clone();
            move |msg: sensor_msgs::msg::Imu| {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                {
                    let mut m = match imu_measurement_from_ros(&msg) {
                        Ok(m) => m,
                        Err(e) => {
                            st.conversion_rejects += 1;
                            st.reject(&n, "imu", e);
                            return;
                        }
                    };
                    if !st.use_imu_covariance {
                        m.yaw_rate_var = None;
                    }
                    match st.fusion.on_imu(&m) {
                        Ok(_) => st.maybe_publish(&n),
                        Err(e) => st.reject(&n, "imu", e),
                    }
                }
            }
        },
    )?;

    let n = Arc::clone(&node);
    let _wheel_sub = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        wheel_topic.as_str().qos(qos::odometry()),
        { let shared = shared.clone(); move |msg: nav_msgs::msg::Odometry| { let mut guard = shared.lock(); let st: &mut State = &mut guard; {
            let odom = match odometry_from_ros(&msg) {
                Ok(o) => o,
                Err(e) => {
                    st.conversion_rejects += 1;
                    st.reject(&n, "wheel", e);
                    return;
                }
            };
            if !st.warned_child_frame && odom.child_frame != st.base_frame {
                st.warned_child_frame = true;
                log_warn!(
                    n.logger(),
                    "wheel odometry child_frame_id '{}' differs from '{}'; its twist is used as the base twist",
                    odom.child_frame,
                    st.base_frame
                );
            }
            match st.fusion.on_wheel(odom.stamp, &odom.twist) {
                Ok(_) => {
                    if let Some((v, dof)) = st.fusion.last_nis() {
                        let i = dof.clamp(1, 2) - 1;
                        st.nis_sum[i] += v;
                        st.nis_count[i] += 1;
                    }
                    st.maybe_publish(&n)
                }
                Err(e) => st.reject(&n, "wheel", e),
            }
        } } },
    )?;

    let n = Arc::clone(&node);
    let _diag = node.create_timer_repeating(Duration::from_secs(diag_period as u64), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            {
                let stamp = match now(&n) {
                    Ok(t) => t,
                    Err(e) => {
                        log_error!(n.logger(), "clock error: {e}");
                        return;
                    }
                };
                let health = st.fusion.health(stamp);
                let (level, message) = match &health {
                    Health::Initializing => (DiagnosticLevel::Ok, "INITIALIZING".to_string()),
                    Health::Running => (DiagnosticLevel::Ok, "RUNNING".to_string()),
                    Health::Degraded(r) => (
                        DiagnosticLevel::Warn,
                        format!(
                            "DEGRADED: {}",
                            r.iter().map(|x| x.as_str()).collect::<Vec<_>>().join(", ")
                        ),
                    ),
                };
                if health.label() != st.last_health {
                    log_info!(
                        n.logger(),
                        "estimator state {} -> {}",
                        st.last_health,
                        message
                    );
                    st.last_health = health.label();
                }
                let s = st.fusion.stats();
                let mut b = StatusBuilder::new("state_estimator_node: fusion", level, &message)
                    .value("state", health.label())
                    .value("imu_accepted", s.imu_accepted)
                    .value("wheel_accepted", s.wheel_accepted)
                    .value("published", st.published)
                    .value(
                        "rejected_invalid",
                        s.rejected_invalid + st.conversion_rejects,
                    )
                    .value("rejected_timestamp", s.rejected_timestamp)
                    .value("rejected_out_of_order", s.rejected_out_of_order)
                    .value("applied_late", s.applied_late)
                    .value("gaps", s.gaps)
                    .value("numerical_failures", s.numerical_failures)
                    .value("wheel_only_updates", s.wheel_only_updates)
                    .value("last_error", &st.last_error);
                // Consistency: mean NIS of the wheel corrections (expected 1 and 2).
                for (i, key) in ["nis_mean_speed_1dof", "nis_mean_wheel_gyro_2dof"]
                    .into_iter()
                    .enumerate()
                {
                    if st.nis_count[i] > 0 {
                        b = b.value(
                            key,
                            format!(
                                "{:.3} (n={})",
                                st.nis_sum[i] / st.nis_count[i] as f64,
                                st.nis_count[i]
                            ),
                        );
                    }
                }
                if let Some(est) = st.fusion.estimate() {
                    b = b
                        .value("x", format!("{:.3}", est.pose.x()))
                        .value("y", format!("{:.3}", est.pose.y()))
                        .value("theta", format!("{:.4}", est.pose.theta()))
                        .value("speed", format!("{:.3}", est.twist.vx))
                        .value("gyro_bias", format!("{:.5}", est.gyro_bias))
                        .value("gyro_bias_sigma", format!("{:.5}", est.gyro_bias_sigma))
                        .value(
                            "sigma_xy",
                            format!(
                                "{:.4}",
                                est.pose_cov[(0, 0)].max(est.pose_cov[(1, 1)]).sqrt()
                            ),
                        )
                        .value("sigma_theta", format!("{:.4}", est.pose_cov[(2, 2)].sqrt()));
                }
                if let Err(e) = diagnostics.publish(stamp, vec![b.build()]) {
                    log_error!(n.logger(), "diagnostics publish failed: {e}");
                }
            }
        }
    })?;

    log_info!(
        node.logger(),
        "state_estimator_node up: {} + {} -> {} + {} ({} -> {}), tf={}, {} parameters",
        wheel_topic,
        imu_topic,
        out_topic,
        estimate_topic,
        odom_frame,
        base_frame,
        publish_tf,
        p.len()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "state_estimator_node shutting down");
    drop(p);
    result
}
