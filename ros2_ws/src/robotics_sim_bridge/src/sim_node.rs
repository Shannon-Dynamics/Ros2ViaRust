//! `sim_node` — publishes the deterministic Rusty simulator on standard ROS topics.
//!
//! ```text
//!   robotics_sim::RustySim (pure Rust physics, seeded)
//!        │ WheelSample / ImuMeasurement / GroundTruth
//!        ▼  robotics_ros_bridge converters
//!   joint_states (sensor_msgs/JointState)   ← like a base driver / ros2_control
//!   imu/data     (sensor_msgs/Imu)          ← like an IMU driver
//!   scan         (sensor_msgs/LaserScan)    ← like a LiDAR driver (ray cast in the world)
//!   ground_truth/odom (nav_msgs/Odometry)   ← for evaluation only
//!   /tf_static   base_link → imu_link, base_link → laser   (one message, this node owns both)
//! ```
//!
//! Simulation time is anchored to the node clock at startup; each timer tick advances the
//! physics until it catches up with the clock (bounded), so message stamps are
//! `start + k·dt` exactly and the data sequence is identical for a given seed regardless of
//! scheduling jitter. Every sensor sample of a physics step (joint states, IMU, scan) and
//! the ground truth of that step carry the same stamp. The node never broadcasts
//! `odom → base_link` (the estimator owns it).
//! With `use_cmd_vel:=true` a `geometry_msgs/Twist` on `cmd_vel` overrides the script.

use rclrs::*;
use robotics_core::{FrameId, Odometry2, Se2, Timestamp};
use robotics_models::sensor::gyro::GyroNoise;
use robotics_models::DiffDriveParams;
use robotics_ros_bridge::convert::{
    header, imu_measurement_to_ros, laser_scan_to_ros, now, odometry_to_ros, twist_from_ros,
};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::{qos, ParamStore, TfBroadcaster};
use robotics_sim::{LidarConfig, NoReturnPolicy, RustyConfig, RustySim, Trajectory, World};
use ros_env::{geometry_msgs, nav_msgs, sensor_msgs};
use std::sync::Arc;
use std::time::Duration;

struct Sim {
    sim: RustySim,
    start: Timestamp,
    joint_pub: Publisher<sensor_msgs::msg::JointState>,
    imu_pub: Publisher<sensor_msgs::msg::Imu>,
    scan_pub: Option<Publisher<sensor_msgs::msg::LaserScan>>,
    scans: u64,
    truth_pub: Publisher<nav_msgs::msg::Odometry>,
    joint_names: [String; 2],
    base_frame: FrameId,
    odom_frame: FrameId,
    truth_child: FrameId,
    max_catch_up_steps: usize,
    steps: u64,
    lagging: u64,
}

impl Sim {
    fn advance_to(&mut self, node: &Node, target: Timestamp) {
        let mut n = 0;
        while self.sim.stamp() < target && n < self.max_catch_up_steps {
            let s = self.sim.step();
            self.steps += 1;
            n += 1;
            if let Some(w) = s.wheel {
                let msg = sensor_msgs::msg::JointState {
                    header: header(w.stamp, &self.base_frame),
                    name: self.joint_names.to_vec(),
                    position: vec![w.left_angle, w.right_angle],
                    velocity: vec![w.left_rate, w.right_rate],
                    effort: vec![],
                };
                if let Err(e) = self.joint_pub.publish(msg) {
                    log_error!(
                        node.logger().throttle(Duration::from_secs(5)),
                        "joint_states publish failed: {e}"
                    );
                }
            }
            // Ground truth at every sensor instant (wheel or IMU), so both the raw
            // (wheel-stamped) and fused (IMU-stamped) estimates match exactly.
            if s.wheel.is_some() || s.imu.is_some() {
                let truth = Odometry2 {
                    stamp: s.truth.stamp,
                    frame: self.odom_frame.clone(),
                    child_frame: self.truth_child.clone(),
                    pose: s.truth.pose,
                    pose_cov: Default::default(),
                    twist: s.truth.twist,
                    twist_cov: Default::default(),
                };
                if let Err(e) = self.truth_pub.publish(odometry_to_ros(&truth)) {
                    log_error!(
                        node.logger().throttle(Duration::from_secs(5)),
                        "ground truth publish failed: {e}"
                    );
                }
            }
            if let Some(m) = &s.imu {
                if let Err(e) = self.imu_pub.publish(imu_measurement_to_ros(m)) {
                    log_error!(
                        node.logger().throttle(Duration::from_secs(5)),
                        "imu publish failed: {e}"
                    );
                }
            }
            if let (Some(scan), Some(publisher)) = (&s.scan, &self.scan_pub) {
                match publisher.publish(laser_scan_to_ros(scan)) {
                    Ok(()) => self.scans += 1,
                    Err(e) => log_error!(
                        node.logger().throttle(Duration::from_secs(5)),
                        "scan publish failed: {e}"
                    ),
                }
            }
        }
        if self.sim.stamp() < target {
            self.lagging += 1;
            log_warn!(
                node.logger().throttle(Duration::from_secs(5)),
                "simulation is {:.3} s behind the clock (catch-up limited to {} steps per tick)",
                target - self.sim.stamp(),
                self.max_catch_up_steps
            );
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("sim_node")?;
    let mut p = ParamStore::new();
    let d = RustyConfig::default();

    let seed = p.i64(
        &node,
        "seed",
        1,
        0,
        i64::MAX,
        "Master seed of every noise stream",
    )?;
    let physics_hz = p.i64(&node, "physics_rate_hz", 200, 10, 5000, "Physics rate (Hz)")?;
    let wheel_hz = p.i64(
        &node,
        "wheel_rate_hz",
        50,
        1,
        1000,
        "joint_states rate (Hz); must divide physics_rate_hz",
    )?;
    let imu_hz = p.i64(
        &node,
        "imu_rate_hz",
        100,
        1,
        1000,
        "IMU rate (Hz); must divide physics_rate_hz",
    )?;
    let wheel_radius = p.f64(
        &node,
        "robot.wheel_radius",
        d.params.wheel_radius,
        1e-3,
        2.0,
        "Nominal wheel radius (m)",
    )?;
    let wheel_sep = p.f64(
        &node,
        "robot.wheel_separation",
        d.params.wheel_separation,
        1e-3,
        5.0,
        "Nominal wheel separation (m)",
    )?;
    let ticks = p.i64(
        &node,
        "robot.ticks_per_rev",
        d.ticks_per_rev as i64,
        1,
        1 << 30,
        "Encoder ticks per wheel revolution",
    )?;
    let slip = p.f64(
        &node,
        "noise.slip_sigma",
        d.slip_sigma,
        0.0,
        1.0,
        "Per-wheel multiplicative slip sigma per physics step",
    )?;
    let radius_err = p.f64(
        &node,
        "noise.right_radius_error",
        d.right_radius_error,
        -0.2,
        0.2,
        "True right-wheel radius error (fraction)",
    )?;
    let gyro_nd = p.f64(
        &node,
        "noise.gyro_noise_density",
        d.gyro.noise_density,
        0.0,
        1.0,
        "Gyro white noise density (rad/s/sqrt(Hz))",
    )?;
    let gyro_rw = p.f64(
        &node,
        "noise.gyro_bias_random_walk",
        d.gyro.bias_random_walk,
        0.0,
        0.1,
        "Gyro bias random walk (rad/s/sqrt(s))",
    )?;
    let gyro_b0 = p.f64(
        &node,
        "noise.gyro_initial_bias",
        d.gyro_initial_bias,
        -1.0,
        1.0,
        "Initial gyro bias (rad/s)",
    )?;
    let repeat = p.bool(
        &node,
        "trajectory.repeat",
        true,
        "Loop the scripted mixed trajectory",
    )?;
    let use_cmd_vel = p.bool(
        &node,
        "use_cmd_vel",
        false,
        "Drive from cmd_vel instead of the script",
    )?;
    let odom_frame = p.string(
        &node,
        "frames.odom",
        "odom",
        "Frame of the ground-truth pose (identical to odom at start)",
    )?;
    let base_frame = p.string(&node, "frames.base_link", "base_link", "Robot body frame")?;
    let imu_frame = p.string(
        &node,
        "frames.imu",
        "imu_link",
        "IMU frame (static, aligned with base_link)",
    )?;
    let truth_child = p.string(
        &node,
        "frames.ground_truth",
        "base_link_ground_truth",
        "child_frame_id of ground_truth/odom",
    )?;
    let left_joint = p.string(
        &node,
        "joints.left",
        "left_wheel_joint",
        "Left wheel joint name",
    )?;
    let right_joint = p.string(
        &node,
        "joints.right",
        "right_wheel_joint",
        "Right wheel joint name",
    )?;
    let ld = LidarConfig::default();
    let lidar_enabled = p.bool(
        &node,
        "lidar.enabled",
        true,
        "Simulate the 2-D LiDAR and publish scan",
    )?;
    let lidar_hz = p.i64(
        &node,
        "lidar.rate_hz",
        ld.rate_hz as i64,
        1,
        100,
        "LiDAR scan rate (Hz); must divide physics_rate_hz",
    )?;
    let lidar_beams = p.i64(
        &node,
        "lidar.beams",
        ld.beams as i64,
        1,
        10_000,
        "Beams per scan",
    )?;
    let lidar_angle_min = p.f64(
        &node,
        "lidar.angle_min",
        ld.angle_min,
        -std::f64::consts::PI,
        std::f64::consts::PI,
        "Bearing of the first beam (rad)",
    )?;
    let lidar_angle_max = p.f64(
        &node,
        "lidar.angle_max",
        ld.angle_max,
        -std::f64::consts::PI,
        std::f64::consts::PI,
        "Bearing of the last beam, inclusive (rad)",
    )?;
    let lidar_range_min = p.f64(
        &node,
        "lidar.range_min",
        ld.range_min,
        0.0,
        10.0,
        "Minimum valid range (m); closer returns are published as -inf (REP 117)",
    )?;
    let lidar_range_max = p.f64(
        &node,
        "lidar.range_max",
        ld.range_max,
        0.1,
        100.0,
        "Maximum range (m); no return is published as +inf (REP 117)",
    )?;
    let lidar_sigma = p.f64(
        &node,
        "lidar.range_sigma",
        ld.range_sigma,
        0.0,
        1.0,
        "Range noise sigma (m)",
    )?;
    let lidar_dropout = p.f64(
        &node,
        "lidar.dropout_probability",
        ld.dropout_probability,
        0.0,
        1.0,
        "Probability that a beam returns nothing",
    )?;
    let lidar_mount_x = p.f64(
        &node,
        "lidar.mount_x",
        ld.mount.x(),
        -5.0,
        5.0,
        "base_link -> laser x (m)",
    )?;
    let lidar_mount_y = p.f64(
        &node,
        "lidar.mount_y",
        ld.mount.y(),
        -5.0,
        5.0,
        "base_link -> laser y (m)",
    )?;
    let lidar_mount_yaw = p.f64(
        &node,
        "lidar.mount_yaw",
        ld.mount.theta(),
        -std::f64::consts::PI,
        std::f64::consts::PI,
        "base_link -> laser yaw (rad)",
    )?;
    let laser_frame = p.string(
        &node,
        "frames.laser",
        "laser",
        "LiDAR frame (static child of base_link, published by this node)",
    )?;
    let world_name = p.string(
        &node,
        "world",
        "rusty_arena",
        "Simulated world: rusty_arena | empty",
    )?;

    let invalid = |what: &str, e: &dyn std::fmt::Display| {
        log_error!(node.logger(), "{what}: {e}");
        RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        }
    };
    let start = now(&node).map_err(|e| invalid("clock", &e))?;
    let cfg = RustyConfig {
        params: DiffDriveParams {
            wheel_radius,
            wheel_separation: wheel_sep,
        },
        physics_dt: 1.0 / physics_hz as f64,
        wheel_rate_hz: wheel_hz as f64,
        imu_rate_hz: imu_hz as f64,
        ticks_per_rev: ticks as u32,
        slip_sigma: slip,
        right_radius_error: radius_err,
        gyro: GyroNoise {
            noise_density: gyro_nd,
            bias_random_walk: gyro_rw,
        },
        gyro_initial_bias: gyro_b0,
        seed: seed as u64,
        start,
        imu_frame: FrameId::new(&imu_frame),
        lidar: lidar_enabled.then(|| LidarConfig {
            frame: FrameId::new(&laser_frame),
            mount: Se2::new(lidar_mount_x, lidar_mount_y, lidar_mount_yaw),
            angle_min: lidar_angle_min,
            angle_max: lidar_angle_max,
            beams: lidar_beams as usize,
            range_min: lidar_range_min,
            range_max: lidar_range_max,
            rate_hz: lidar_hz as f64,
            range_sigma: lidar_sigma,
            dropout_probability: lidar_dropout,
            no_return: NoReturnPolicy::Infinity,
        }),
    };
    let world = match world_name.as_str() {
        "rusty_arena" => World::rusty_arena(),
        "empty" => World::empty(),
        other => {
            return Err(invalid(
                "unknown world",
                &format!("'{other}' (expected rusty_arena or empty)"),
            ))
        }
    };
    let mount = cfg.lidar.as_ref().map(|l| l.mount);
    let sim = RustySim::with_world(cfg, Trajectory::mixed(repeat), world)
        .map_err(|e| invalid("invalid simulator configuration", &e))?;

    // All static sensor frames go out in ONE transient-local message: a late subscriber
    // receives only the last message of each publisher.
    let static_tf = TfBroadcaster::new_static(&node)?;
    let base = FrameId::new(&base_frame);
    let mut statics = vec![(base.clone(), FrameId::new(&imu_frame), Se2::IDENTITY)];
    if let Some(m) = mount {
        statics.push((base.clone(), FrameId::new(&laser_frame), m));
    }
    static_tf.send_many(start, &statics)?;

    let joint_pub = node
        .create_publisher::<sensor_msgs::msg::JointState>("joint_states".qos(qos::sensor_data()))?;
    let imu_pub =
        node.create_publisher::<sensor_msgs::msg::Imu>("imu/data".qos(qos::sensor_data()))?;
    let truth_pub =
        node.create_publisher::<nav_msgs::msg::Odometry>("ground_truth/odom".qos(qos::odometry()))?;
    let scan_pub = if lidar_enabled {
        Some(node.create_publisher::<sensor_msgs::msg::LaserScan>("scan".qos(qos::sensor_data()))?)
    } else {
        None
    };

    let shared = Shared::new(Sim {
        sim,
        start,
        joint_pub,
        imu_pub,
        scan_pub,
        scans: 0,
        truth_pub,
        joint_names: [left_joint.clone(), right_joint.clone()],
        base_frame: FrameId::new(&base_frame),
        odom_frame: FrameId::new(&odom_frame),
        truth_child: FrameId::new(&truth_child),
        max_catch_up_steps: physics_hz as usize, // at most one second per tick
        steps: 0,
        lagging: 0,
    });

    let _cmd_sub = if use_cmd_vel {
        let n = Arc::clone(&node);
        Some(node.create_subscription::<geometry_msgs::msg::Twist, _>(
            "cmd_vel".qos(qos::command()),
            {
                let shared = shared.clone();
                move |msg: geometry_msgs::msg::Twist| {
                    let mut guard = shared.lock();
                    let s: &mut Sim = &mut guard;
                    match twist_from_ros(&msg) {
                        Ok(t) => s.sim.set_command_override(Some(t)),
                        Err(e) => log_warn!(
                            n.logger().throttle(Duration::from_secs(5)),
                            "ignoring cmd_vel: {e}"
                        ),
                    }
                }
            },
        )?)
    } else {
        None
    };

    let n = Arc::clone(&node);
    let _timer = node.create_timer_repeating(Duration::from_millis(10), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let s: &mut Sim = &mut guard;
            {
                match now(&n) {
                    Ok(t) => s.advance_to(&n, t),
                    Err(e) => log_error!(n.logger(), "clock error: {e}"),
                }
                log_info!(
                    n.logger().throttle(Duration::from_secs(10)),
                    "sim t={:.1} s ({} steps, {} scans), true pose x={:.3} y={:.3} th={:.3}, lag events {}",
                    s.sim.stamp() - s.start,
                    s.steps,
                    s.scans,
                    s.sim.pose().x(),
                    s.sim.pose().y(),
                    s.sim.pose().theta(),
                    s.lagging
                );
            }
        }
    })?;

    log_info!(
        node.logger(),
        "sim_node up: seed={seed}, world={world_name}, physics {physics_hz} Hz, joint_states {wheel_hz} Hz, imu {imu_hz} Hz, scan {} ({lidar_beams} beams, frame {laser_frame}), slip={slip}, radius_err={radius_err}, gyro nd={gyro_nd} b0={gyro_b0}, cmd_vel={use_cmd_vel}",
        if lidar_enabled { format!("{lidar_hz} Hz") } else { "off".to_string() }
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "sim_node shutting down");
    drop(p);
    result
}
