//! `fake_wheel_encoders` — a stand-in for a differential-drive base driver.
//!
//! Integrates the commanded twist through the inverse kinematics of
//! [`robotics_models::DiffDriveParams`] and publishes cumulative wheel angles as
//! `sensor_msgs/JointState` on `joint_states`, exactly like `ros2_control`'s
//! `joint_state_broadcaster` would. Feed it `geometry_msgs/Twist` on `cmd_vel`, or let it
//! drive the default circle.
//!
//! Parameters
//! * `wheel_radius` (m, default 0.033), `wheel_separation` (m, default 0.16): Rusty.
//! * `rate_hz` (int, default 50): publish rate.
//! * `left_joint`, `right_joint` (string): joint names in the message.
//! * `v`, `omega` (default 0.3 m/s, 0.5 rad/s): initial command.
//! * `frame_id` (default `base_link`).

use rclrs::*;
use robotics_core::Twist2;
use robotics_models::DiffDriveParams;
use robotics_ros_bridge::convert::{header, now, twist_from_ros};
use robotics_ros_bridge::qos;
use robotics_ros_bridge::shared::Shared;
use ros_env::{geometry_msgs, sensor_msgs};
use std::sync::Arc;
use std::time::Duration;

struct Sim {
    params: DiffDriveParams,
    cmd: Twist2,
    left: f64,
    right: f64,
    ticks: u64,
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("fake_wheel_encoders")?;

    let wheel_radius: MandatoryParameter<f64> = node
        .declare_parameter("wheel_radius")
        .default(0.033)
        .mandatory()?;
    let wheel_separation: MandatoryParameter<f64> = node
        .declare_parameter("wheel_separation")
        .default(0.16)
        .mandatory()?;
    let rate_hz: ReadOnlyParameter<i64> = node
        .declare_parameter("rate_hz")
        .default(50)
        .range(ParameterRange {
            lower: Some(1),
            upper: Some(1000),
            step: None,
        })
        .read_only()?;
    let left_joint: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("left_joint")
        .default("left_wheel_joint".into())
        .read_only()?;
    let right_joint: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("right_joint")
        .default("right_wheel_joint".into())
        .read_only()?;
    let v0: MandatoryParameter<f64> = node.declare_parameter("v").default(0.3).mandatory()?;
    let omega0: MandatoryParameter<f64> =
        node.declare_parameter("omega").default(0.5).mandatory()?;
    let frame_id: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("frame_id")
        .default("base_link".into())
        .read_only()?;

    let params = DiffDriveParams {
        wheel_radius: wheel_radius.get(),
        wheel_separation: wheel_separation.get(),
    }
    .validated()
    .map_err(|e| {
        log_error!(node.logger(), "invalid kinematic parameters: {e}");
        RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        }
    })?;

    let publisher = node
        .create_publisher::<sensor_msgs::msg::JointState>("joint_states".qos(qos::sensor_data()))?;
    let shared = Shared::new(Sim {
        params,
        cmd: Twist2::unicycle(v0.get(), omega0.get()),
        left: 0.0,
        right: 0.0,
        ticks: 0,
    });

    let node_for_cmd = Arc::clone(&node);
    let _cmd_sub =
        node.create_subscription::<geometry_msgs::msg::Twist, _>("cmd_vel".qos(qos::command()), {
            let shared = shared.clone();
            move |msg: geometry_msgs::msg::Twist| {
                let mut guard = shared.lock();
                let sim: &mut Sim = &mut guard;
                match twist_from_ros(&msg) {
                    Ok(t) => {
                        sim.cmd = t;
                        log_info!(
                            node_for_cmd.logger(),
                            "new command v={:.3} omega={:.3}",
                            t.vx,
                            t.omega
                        );
                    }
                    Err(e) => log_warn!(node_for_cmd.logger(), "ignoring cmd_vel: {e}"),
                }
            }
        })?;

    let dt = 1.0 / rate_hz.get() as f64;
    let node_for_timer = Arc::clone(&node);
    let names = vec![left_joint.get().to_string(), right_joint.get().to_string()];
    let frame = robotics_core::FrameId::new(frame_id.get());
    let _timer = node.create_timer_repeating(Duration::from_secs_f64(dt), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let sim: &mut Sim = &mut guard;
            {
                let w = sim.params.inverse_kinematics(sim.cmd);
                sim.left += w.left * dt;
                sim.right += w.right * dt;
                sim.ticks += 1;
                let stamp = match now(&node_for_timer) {
                    Ok(t) => t,
                    Err(e) => {
                        log_error!(node_for_timer.logger(), "clock error: {e}");
                        return;
                    }
                };
                let msg = sensor_msgs::msg::JointState {
                    header: header(stamp, &frame),
                    name: names.clone(),
                    position: vec![sim.left, sim.right],
                    velocity: vec![w.left, w.right],
                    effort: vec![],
                };
                if let Err(e) = publisher.publish(&msg) {
                    log_error!(node_for_timer.logger(), "publish failed: {e}");
                }
                log_info!(
                    node_for_timer.logger().throttle(Duration::from_secs(5)),
                    "tick {}: wheel angles L={:.3} R={:.3} rad (cmd v={:.2} omega={:.2})",
                    sim.ticks,
                    sim.left,
                    sim.right,
                    sim.cmd.vx,
                    sim.cmd.omega
                );
            }
        }
    })?;

    log_info!(
        node.logger(),
        "fake_wheel_encoders up: {} Hz, r={} m, l={} m, joints=[{}, {}]",
        rate_hz.get(),
        params.wheel_radius,
        params.wheel_separation,
        left_joint.get(),
        right_joint.get()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "fake_wheel_encoders shutting down");
    result
}
