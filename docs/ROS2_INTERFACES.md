# ROS 2 Interfaces

Standard message types only; no custom interface package exists yet. Custom messages will
be introduced only where no standard type fits (pose-graph edges, landmark observations,
frontier candidates) and will live in a single `robotics_interfaces` package.

## 1. Topics

### Phase 1 (implemented)

| Topic | Type | Direction | Node | QoS preset |
|---|---|---|---|---|
| `chatter` | `std_msgs/String` | pub / sub | `qos_talker` / `qos_listener` | `command` or `sensor_data` (parameter `reliable`) |
| `cmd_vel` | `geometry_msgs/Twist` | sub | `fake_wheel_encoders` | `command` |
| `joint_states` | `sensor_msgs/JointState` | pub / sub | `fake_wheel_encoders` / `wheel_odometry_node` | `sensor_data` |
| `odom` | `nav_msgs/Odometry` | pub | `wheel_odometry_node` | `odometry` |
| `/tf` | `tf2_msgs/TFMessage` | pub | `wheel_odometry_node` (`odom → base_link`) | `tf` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | pub | `wheel_odometry_node` | `diagnostics` |

### Phase 2 (implemented; launch `estimator_sim.launch.xml`)

| Topic | Type | Direction | Node | QoS preset | Rate |
|---|---|---|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | pub / sub | `sim_node` / `wheel_odometry_node` | `sensor_data` | 50 Hz |
| `imu/data` | `sensor_msgs/Imu` | pub / sub | `sim_node` / `state_estimator_node` | `sensor_data` | 100 Hz |
| `scan` | `sensor_msgs/LaserScan` (frame `laser`, 360 beams, REP 117 ranges) | pub | `sim_node` (consumers: Phase 3+) | `sensor_data` | 10 Hz |
| `wheel/odometry` | `nav_msgs/Odometry` (`odom` → `base_link`) | pub / sub | `wheel_odometry_node` (remapped from `odom`) / `state_estimator_node`, `estimator_evaluator` | `odometry` | 50 Hz |
| `odom` | `nav_msgs/Odometry` (`odom` → `base_link`) | pub | `state_estimator_node` | `odometry` | ≤ 50 Hz (`max_publish_rate_hz`) |
| `state_estimate` | `geometry_msgs/PoseWithCovarianceStamped` (`odom`) | pub | `state_estimator_node` | `odometry` | same stamps as `odom` |
| `ground_truth/odom` | `nav_msgs/Odometry` (`odom` → `base_link_ground_truth`) | pub / sub | `sim_node` / `estimator_evaluator` (evaluation only) | `odometry` | 100 Hz |
| `/tf` | `tf2_msgs/TFMessage` | pub | `state_estimator_node` (`odom → base_link`) | `tf` | with `odom` |
| `/tf_static` | `tf2_msgs/TFMessage` | pub | `sim_node` (`base_link → imu_link`, `base_link → laser`, one message) | `tf_static` | once |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | pub | `wheel_odometry_node`, `state_estimator_node` (health, counters, mean NIS), `estimator_evaluator` (RMSE) | `diagnostics` | 1 Hz / 0.2 Hz |
| `cmd_vel` | `geometry_msgs/Twist` | sub | `sim_node` when `use_cmd_vel:=true` | `command` | – |

### Planned (phases 3–9)

| Topic | Type | Producer → consumer |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | `mapping_node` / SLAM → localization, planner, explorer (latched) |
| `/pose`, `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `localization_node` |
| `/particle_cloud` | `geometry_msgs/PoseArray` | `localization_node` |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz → `localization_node` |
| `/slam/pose`, `/slam/path`, `/slam/graph` | `PoseWithCovarianceStamped`, `nav_msgs/Path`, `visualization_msgs/MarkerArray` | SLAM |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz / explorer → planner |
| `/plan` | `nav_msgs/Path` | `planner_node` → `controller_node` |
| `/esdf` | `sensor_msgs/Image`, `32FC1` (signed distance in metres, one float per cell, row 0 = grid row `j = 0`), paired by `header.stamp` with the `/map` it was computed from, whose grid geometry it shares | mapping → planner/controller; encoding decision D12 of `docs/phase-4-mapping/implementation.md` section 5.2 |
| `/cmd_vel` | `geometry_msgs/Twist` (`TwistStamped` when the base supports it) | `controller_node` → base |
| `/explore/frontiers` | `visualization_msgs/MarkerArray` | `explorer_node` |

## 2. Services (planned)

Short, synchronous operations only: `~/reset_odometry` (`std_srvs/Trigger`),
`~/set_initial_pose`, `~/save_map` / `~/load_map`, `~/fit_sensor_model`.

## 3. Actions (planned)

Long-running behaviours: `NavigateToPose` (Nav2-compatible `nav2_msgs/action/NavigateToPose`),
`FollowPath`, `Explore` (custom, feedback = coverage/entropy), `Relocalize`. rclrs 0.8 ships
action servers and clients.

## 4. Parameters

All configuration is declared parameters with defaults, descriptions and ranges; nothing
is hard-coded. Naming: snake_case, SI units, dotted groups (`noise.*`, `lidar.*`,
`frames.*`, `topics.*`). Phase 1 parameters are listed in each node's module docs
(`qos_talker.rs`, `qos_listener.rs`, `fake_wheel_encoders.rs`, `wheel_odometry_node.rs`) and
in `robotics_bringup/config/wheel_odometry.yaml`; Phase 2 parameters (`sim_node`,
`wheel_odometry_node`, `state_estimator_node`, `estimator_evaluator`) in
`robotics_bringup/config/state_estimation.yaml`, `docs/SIMULATION.md` § 5 and
`docs/STATE_ESTIMATION.md` § 3.

Runtime changes: parameters read once at startup are declared `read_only`; others are
read on use (`prefix` in the talker is live). Range violations and read-only writes are
rejected by rclrs's parameter service, which `ros2 param set` reports.

## 5. QoS decisions

| Preset | Reliability | Durability | History | Why |
|---|---|---|---|---|
| `sensor_data` | BEST_EFFORT | VOLATILE | KEEP_LAST 5 | a fresh LiDAR/IMU/joint sample beats a retransmitted stale one; matches `rmw_qos_profile_sensor_data` used by drivers (`ros2 topic echo /scan` needs `--qos-reliability best_effort`) |
| `odometry` | RELIABLE | VOLATILE | KEEP_LAST 10 | estimators must not silently miss integration steps |
| `map_latched` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST 1 | a planner or RViz started after the mapper still gets the latest map |
| `command` | RELIABLE | VOLATILE | KEEP_LAST 10 | dropped commands are safety-relevant |
| `tf` | RELIABLE | VOLATILE | KEEP_LAST 100 | identical to `tf2_ros::TransformBroadcaster` |
| `tf_static` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST 100 | identical to `tf2_ros::StaticTransformBroadcaster` |
| `diagnostics` | RELIABLE | VOLATILE | KEEP_LAST 10 | low rate, must arrive |

Defaults are never used implicitly: every publisher/subscription passes a preset via
`IntoPrimitiveOptions::qos`.

## 6. TF tree

```text
map ──(localization | SLAM)──► odom ──(wheel/state estimator)──► base_link ──(static)──► laser, imu_link, …
```

| Edge | Owner | Rate | Notes |
|---|---|---|---|
| `odom → base_link` | `state_estimator_node` (phase 2 stack) / `wheel_odometry_node` (phase 1 demo only; `publish_tf:=false` in phase 2) | ≤ 50 Hz | continuous, drifts |
| `map → odom` | `localization_node` **or** SLAM node | on correction | discontinuous; exactly one owner per launch (phase 3+) |
| `base_link → imu_link`, `base_link → laser` | `sim_node` in simulation (one `/tf_static` message; mount from `lidar.mount_*`); a static publisher / `robot_state_publisher` on a robot | once | `/tf_static`, transient local |

Consumers read TF through `robotics_ros_bridge::TfListener` (`docs/TF_BUFFER.md`). The
Phase 2 integration test verifies that each child frame has exactly one publisher.

## 7. Interoperability

The stack speaks the same interfaces as Nav2, slam_toolbox, robot_localization, RViz and
rosbag2. Any subsystem can be swapped: run Nav2's planner on our `/map` and `/amcl_pose`,
or our MCL under Nav2's controller. `wheel_odometry_node` is a drop-in replacement for a
`diff_drive_controller` odometry output when a base only exposes joint states.
