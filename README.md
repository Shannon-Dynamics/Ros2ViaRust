# ros2-rust-robotics

A modular robotics autonomy framework for **ROS 2 Jazzy**, written in **Rust**, whose
algorithms come from the book *Probabilistic Robotics via Rust*. Pure Rust libraries do
the estimation, mapping, planning and control; thin `rclrs` nodes connect them to
standard ROS 2 interfaces.

```text
Probabilistic Robotics via Rust  ─►  crates/robotics_*  ─►  robotics_ros_bridge  ─►  rclrs nodes  ─►  DDS  ─►  robot / sim / Nav2 / RViz
```

Status: **Phase 2 (state estimation, sensor models, and simulation foundation) complete** —
see [docs/ROADMAP.md](docs/ROADMAP.md) for phases 3–9 (localization, mapping, SLAM,
planning, control, exploration, full stack).

The same work is also written up as problem-based material, **ROS via Rust**: each part
opens with a concrete problem a TurtleBot3 has in a Webots simulation, derives the theory
that problem needs, then points at the Rust that solves it, and closes with a benchmark
against the Python and C++ stacks doing the same job. Start at
[docs/README.md](docs/README.md).

| Document | Content |
|---|---|
| [docs/README.md](docs/README.md) | "ROS via Rust": problem-based material, one part per phase |
| [sim/webots/README.md](sim/webots/README.md) | Webots TurtleBot3 simulation: what is Rust, what is not, how to run it |
| [benchmarks/README.md](benchmarks/README.md) | benchmark methodology and result files |
| [docs/AUDIT.md](docs/AUDIT.md) | audit of the reference repository (snapshot before Phase 2): what exists, what is Rust vs TypeScript vs prose, risks |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | layers, crates, packages, threading, TF ownership |
| [docs/SOURCE_MAPPING.md](docs/SOURCE_MAPPING.md) | chapter → crate/node mapping, conventions kept, deviations, worked examples as tests |
| [docs/ROS2_INTERFACES.md](docs/ROS2_INTERFACES.md) | topics, services, actions, parameters, QoS decisions, TF tree |
| [docs/STATE_ESTIMATION.md](docs/STATE_ESTIMATION.md) | the ESKF wheel + gyro estimator, validation (RMSE, NEES, NIS), known limitations |
| [docs/SIMULATION.md](docs/SIMULATION.md) | the deterministic Rusty simulator: world, LiDAR, RNG discipline, `sim_node` |
| [docs/TF_BUFFER.md](docs/TF_BUFFER.md) | the Rust transform buffer and TF listener (rclrs has no tf2) |
| [docs/BUILD.md](docs/BUILD.md) | environment, underlay, build, test scripts, troubleshooting |
| [docs/ROADMAP.md](docs/ROADMAP.md) | phased plan |

## What is in Phase 3 (in progress)

Localization against the Webots TurtleBot3 instead of the Rusty simulator, both filters
this phase set out to compare:

* `crates/robotics_localization` — ROS-free: the landmark map read from the simulation's own
  ground truth file (`map`), range and bearing observations of the round pillars extracted
  from a laser scan (`pillars`), and EKF localization on SE(2) with maximum likelihood
  association and a χ² gate (`ekf`), and particle filter localization on the same map and
  the same observations with low-variance resampling and a random-injection recovery
  strategy (`mcl`). Only the representation of the belief differs between the two, which is
  what makes the comparison mean anything.
* `ros2_ws/src/robotics_localization_bridge` — `localization_node`: `/scan` + `/odom` +
  `/imu` → `/localization/pose` in the `map` frame, plus `/diagnostics`; it owns the
  `map → odom` correction (REP 105) and never subscribes to ground truth. `mcl_node` is the
  particle filter alongside it, publishing `/mcl/pose` and no TF at all, so that running
  both in one launch cannot produce two owners of the same edge. One
  `localization_evaluator` per filter grades it against `/ground_truth/pose` and writes one
  CSV row per sample.
* `sim/webots/` — the Phase 3 world (6 by 6 m room, four pillars), the launch file that
  starts Webots and the official `webots_ros2_turtlebot` driver, the ground truth and
  teleport supervisor, a movie recorder, and the run script that captures a whole run as
  files.

Not built yet: Augmented MCL's `w_fast`/`w_slow` injection and KLD adaptive sizing, which
`mcl.rs` justifies simplifying rather than quietly omitting; dominant-cluster pose
extraction; global localization from a genuinely uniform prior, as opposed to recovery from
a kidnapping; `/particle_cloud`, `/amcl_pose` and `/initialpose`; and every benchmark row of
`benchmarks/phase-3-localization.csv`. See
[docs/phase-3-localization/](docs/phase-3-localization/) and
[docs/ROADMAP.md](docs/ROADMAP.md).

Phase 3 runs are kept as raw output under `evidence/phase-3-correctness/`, one directory per
run, and the simulation's own runs under `sim/webots/evidence/`. Numbers are not copied out
of them into prose, here or anywhere else.

What that evidence currently supports is still narrower than "Phase 3 works", and the
verdict files rather than this paragraph are the record. On a 21-waypoint route long enough
for odometry to drift, both EKF and MCL beat the dead reckoning they correct. On a teleport
part way through that same route, MCL recovers and the EKF structurally cannot, which is the
kidnapped robot question this phase exists to answer. Read the final normal evidence in
`evidence/phase-3-correctness/2026-10-10T144153-final-global-reset/` and kidnap evidence in
`evidence/phase-3-correctness/2026-10-10T143418-mcl-global-reset/`, and
[docs/phase-3-localization/implementation.md](docs/phase-3-localization/implementation.md)
section 5, which keeps both defects this phase found and what was done about each.

## What is in Phase 2

* `crates/robotics_filters` — `InformationFilter` (canonical form), `RtsSmoother`,
  NEES/NIS with χ² bounds (`consistency`), generic `Ekf` / `Ukf` over `Manifold`, the SE(2)
  error-state `EskfSe2` and the asynchronous wheel + gyro `OdomImuFusion`; `ParticleSet`
  with log weights, low-variance and multinomial resampling, ESS; KLD-adaptive sizing
  (`kld`) with pluggable binning (`GridBinning`, `Se2Binning`, `BinFn`).
* `crates/robotics_models::sensor` — gyro model, beam mixture with EM intrinsics
  (Table 6.2), likelihood field over an exact Felzenszwalb–Huttenlocher EDT, range–bearing
  landmark model (pose Jacobian, Table 6.5 pose sampler), likelihood tempering `p^β`.
* `crates/robotics_sim` — deterministic Rusty simulator: exact-arc kinematics, slip,
  calibration error, quantized encoders, gyro with bias walk, a segment `World` with exact
  ray casting and a 360-beam 2-D LiDAR; one seeded RNG stream per noise source.
* `crates/robotics_core` — additionally normal/χ² quantiles, `LaserScan::validate`, and the
  ROS-independent planar `TransformBuffer`.
* `ros2_ws/src/robotics_ros_bridge` — `TfListener` (`/tf` + `/tf_static` → buffer, TF
  authority conflict detection), planar TF and `PoseWithCovarianceStamped` converters,
  stricter `LaserScan` validation.
* `ros2_ws/src/robotics_sim_bridge` — `sim_node` (`/joint_states`, `/imu/data`, **`/scan`**,
  ground truth, static `base_link → {imu_link, laser}`), `estimator_evaluator`.
* `ros2_ws/src/robotics_estimation` — `state_estimator_node`: `/wheel/odometry` +
  `/imu/data` → `/odom`, **`/state_estimate`**, TF `odom → base_link`, `/diagnostics`
  (with live mean NIS).
* `ros2_ws/src/robotics_integration` — `phase2_stack_test`, a Rust-first `launch_testing`
  equivalent: spawns the launch, checks 56 properties of the live graph, Ctrl-C's it and
  requires a clean exit.
* `ros2_ws/src/robotics_bringup` — `estimator_sim.launch.xml` + `config/state_estimation.yaml`.

185 tests (unit, property, numerical regression, offline pipeline), including the book's
worked examples for the information filter, RTS smoother, KLD table and beam-model EM
(`docs/SOURCE_MAPPING.md`).

## What is in Phase 1

* `crates/robotics_core` — `Se2` (exp/log/adjoint, ⊞/⊟), `Manifold`, `Gaussian` /
  `Canonical`, seeded RNG, `Timestamp`, `FrameId`, internal sensor/state types.
* `crates/robotics_models` — differential-drive kinematics, `WheelOdometry` with
  covariance compounding, Thrun velocity and odometry motion models (sampler + density).
* `crates/robotics_filters` — `BayesFilter` trait, `HistogramFilter1D`, `KalmanFilter`
  (Joseph form) with evidence and NIS.
* `ros2_ws/src/robotics_ros_bridge` — message ⇄ core converters, QoS presets, TF
  broadcaster, diagnostics, SIGINT/SIGTERM shutdown.
* `ros2_ws/src/robotics_examples` — `qos_talker`, `qos_listener` (QoS, parameters,
  timers, workers), `fake_wheel_encoders` (a stand-in base publishing `joint_states`).
* `ros2_ws/src/robotics_estimation` — `wheel_odometry_node`: `joint_states` → `/odom`,
  TF `odom → base_link`, `/diagnostics`.
* `ros2_ws/src/robotics_bringup` — XML launch files and YAML parameters.

## Quick start (WSL2 Ubuntu 24.04, ROS 2 Jazzy)

```bash
# 0. prerequisites (once)
sudo apt install -y git libclang-dev python3-pip python3-vcstool
pip install --break-system-packages colcon-cargo colcon-ros-cargo
cargo install cargo-ament-build

# 1. interface underlay with Rust bindings (once, ~5 min)
scripts/setup_interfaces_underlay.sh

# 2. environment (every terminal)
source scripts/env.sh

# 3. pure Rust libraries — no ROS needed
cargo test

# 4. ROS 2 packages
cargo build --workspace                  # fast check
cd ros2_ws && colcon build && source install/setup.bash && cd ..

# 5. run
ros2 launch robotics_bringup estimator_sim.launch.xml       # Phase 2: sim + wheel odometry + ESKF
ros2 launch robotics_bringup wheel_odometry_demo.launch.xml # Phase 1
ros2 launch robotics_bringup pubsub_demo.launch.xml         # Phase 1

# 6. verify
bash scripts/gates.sh                      # fmt, clippy -D warnings, cargo test, colcon build
bash scripts/integration_test_phase2.sh    # Rust integration test of the Phase 2 stack
bash scripts/verify_phase2_all.sh          # the complete Phase 2 verification sequence
```

On this machine the git repository lives on Windows
(`D:\Shannon\ROS2 via RUST\ros2-rust-robotics`) and builds run in the WSL mirror
`~/ros2-rust-robotics` refreshed by `scripts/sync_to_wsl.sh`; see `docs/BUILD.md`.

## Phase 2 evidence (recorded 2026-09-27, WSL2 Ubuntu 24.04.2, ROS 2 Jazzy, rclrs 0.8.0, CycloneDDS, cargo 1.97.1)

Final verification sequence (`scripts/verify_phase2_all.sh`, run in order):

```text
1 fmt            PASS      cargo fmt --all --check
2 clippy         PASS      cargo clippy --workspace --all-targets --all-features -- -D warnings
3 test           PASS      TOTAL: 185 passed, 0 failed
4 underlay       PASS      all Rust interface crates present in ~/ros2_rust_interfaces_ws/install
5 colcon         PASS      Summary: 6 packages finished
6 environment    PASS
7-16 runtime     PASS      scripts/verify_phase2_runtime.sh (output below)
17 integration   PASS      56 checks, 56 passed, 0 failed
overall: PASS
```

`cargo test --workspace` per test binary:

```text
robotics_core        38 passed      robotics_filters     67 passed
robotics_models      36 passed      robotics_sim         21 passed
robotics_ros_bridge  14 passed      estimation_pipeline   2 passed   (offline ROS-pipeline replica)
robotics_integration  4 passed      lidar_regression      3 passed
```

colcon build (`scripts/gates.sh` run that first built `robotics_integration`; the final
sequence rebuilt incrementally: `Summary: 6 packages finished [2.68s]`):

```text
Finished <<< robotics_ros_bridge [1.61s]
Finished <<< robotics_examples [5.58s]
Finished <<< robotics_sim_bridge [5.83s]
Finished <<< robotics_estimation [6.04s]
Finished <<< robotics_bringup [2.84s]
Finished <<< robotics_integration [20.6s]
Summary: 6 packages finished [31.5s]
```

Nodes and topics (`ros2 launch robotics_bringup estimator_sim.launch.xml`):

```text
$ ros2 node list
/estimator_evaluator
/sim_node
/state_estimator_node
/wheel_odometry_node
$ ros2 topic list -t
/diagnostics [diagnostic_msgs/msg/DiagnosticArray]
/ground_truth/odom [nav_msgs/msg/Odometry]
/imu/data [sensor_msgs/msg/Imu]
/joint_states [sensor_msgs/msg/JointState]
/odom [nav_msgs/msg/Odometry]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/scan [sensor_msgs/msg/LaserScan]
/state_estimate [geometry_msgs/msg/PoseWithCovarianceStamped]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/wheel/odometry [nav_msgs/msg/Odometry]
```

Rates:

```text
$ ros2 topic hz /scan
average rate: 10.000   min: 0.099s max: 0.101s std dev: 0.00018s window: 67
$ ros2 topic hz /imu/data
average rate: 100.000  min: 0.008s max: 0.012s std dev: 0.00099s window: 100
$ ros2 topic hz /joint_states
average rate: 49.994   min: 0.018s max: 0.022s std dev: 0.00131s window: 100
$ ros2 topic hz /wheel/odometry
average rate: 49.998   min: 0.018s max: 0.023s std dev: 0.00131s window: 100
$ ros2 topic hz /odom
average rate: 50.001   min: 0.020s max: 0.021s std dev: 0.00019s window: 100
$ ros2 topic hz /state_estimate
average rate: 50.000   min: 0.019s max: 0.021s std dev: 0.00018s window: 100
$ ros2 topic hz /ground_truth/odom
average rate: 99.994   min: 0.007s max: 0.013s std dev: 0.00100s window: 100
```

QoS (`ros2 topic info -v`, publisher side; subscribers match):

```text
$ ros2 topic info -v /scan
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Node name: sim_node   Endpoint type: PUBLISHER   Reliability: BEST_EFFORT   History (Depth): KEEP_LAST (5)   Durability: VOLATILE
$ ros2 topic info -v /odom
Type: nav_msgs/msg/Odometry
Publisher count: 1
Node name: state_estimator_node   Endpoint type: PUBLISHER   Reliability: RELIABLE   History (Depth): KEEP_LAST (10)   Durability: VOLATILE
Subscription count: 1
Node name: estimator_evaluator    Endpoint type: SUBSCRIPTION   Reliability: RELIABLE   History (Depth): KEEP_LAST (10)   Durability: VOLATILE
$ ros2 topic info -v /state_estimate   → state_estimator_node, RELIABLE, KEEP_LAST (10), VOLATILE
$ ros2 topic info -v /imu/data         → sim_node, BEST_EFFORT, KEEP_LAST (5), VOLATILE; subscriber state_estimator_node (same)
$ ros2 topic info -v /tf               → Publisher count: 1, state_estimator_node, RELIABLE, KEEP_LAST (100), VOLATILE
$ ros2 topic info -v /tf_static        → Publisher count: 1, sim_node, RELIABLE, KEEP_LAST (100), TRANSIENT_LOCAL
```

Scan content:

```text
$ ros2 topic echo /scan --once --qos-reliability best_effort --no-arr
header: {stamp: {sec: 1790475114, nanosec: 577610779}, frame_id: laser}
angle_min: -3.1415927410125732   angle_max: 3.1241393089294434   angle_increment: 0.01745329238474369
time_increment: 0.0   scan_time: 0.10000000149011612   range_min: 0.11999999731779099   range_max: 8.0
ranges: '<sequence type: float, length: 360>'
$ ros2 topic echo /scan --once --qos-reliability best_effort --field ranges      (first values)
array('f', [7.38616943359375, 7.327226161956787, 7.221809387207031, 7.108884334564209, 7.182159423828125, 7.346911430358887, 7.554990291595459, inf, inf, inf, 7.935398101806641, ...
```

(`inf` = no return within 8 m, REP 117.)

Parameters:

```text
$ ros2 param list /state_estimator_node
  diagnostics_period_s  frames.base_link  frames.odom  imu_noise.use_message_covariance  imu_noise.yaw_rate_sigma
  imu_timeout  initial_covariance.sigma_bias  initial_covariance.sigma_speed  initial_covariance.sigma_theta
  initial_covariance.sigma_xy  max_dt  max_measurement_delay  max_publish_rate_hz
  process_noise.accel_noise_density  process_noise.gyro_bias_random_walk  process_noise.gyro_noise_density
  process_noise.slip_diffusion  publish_tf  topics.imu  topics.output_odom  topics.state_estimate
  topics.wheel_odom  use_sim_time  wheel_noise.bias_update_max_speed  wheel_noise.speed_sigma
  wheel_noise.wheel_only_yaw_var_per_s  wheel_noise.yaw_rate_per_speed  wheel_noise.yaw_rate_per_turn
  wheel_noise.yaw_rate_sigma  wheel_timeout
$ ros2 param get /sim_node lidar.rate_hz      Integer value is: 10
$ ros2 param get /sim_node lidar.beams        Integer value is: 360
$ ros2 param get /sim_node lidar.mount_x      Double value is: 0.05
$ ros2 param get /sim_node world              String value is: rusty_arena
$ ros2 param get /wheel_odometry_node publish_tf    Boolean value is: False
$ ros2 param set /state_estimator_node max_dt 1.0
Setting parameter failed: Parameter is read only
```

TF:

```text
$ ros2 run tf2_ros tf2_echo odom base_link
At time 1790475133.927610779
- Translation: [4.309, 0.504, 0.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.957]
$ ros2 run tf2_ros tf2_echo base_link laser
At time 0.0
- Translation: [0.050, 0.000, 0.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]
$ ros2 run tf2_ros tf2_echo odom laser
At time 1790475146.347610779
- Translation: [3.192, 0.429, 0.000]
- Rotation: in RPY (radian) [0.000, -0.000, 1.471]
```

Estimator (live `/diagnostics` and `estimator_evaluator`, seed 1, ≈190 s of driving;
offline table for 5 seeds in `docs/STATE_ESTIMATION.md` § 4):

```text
state_estimator_node: fusion  RUNNING  imu_accepted 11100  wheel_accepted 5530  published 5550  rejected_invalid 0
  nis_mean_speed_1dof: 0.022 (n=4101)   nis_mean_wheel_gyro_2dof: 0.918 (n=1429)
[estimator_evaluator]: vs ground truth | raw wheel: 9483 samples, pos RMSE 0.775 m, yaw RMSE 0.2706 rad
                                       | fused:     9499 samples, pos RMSE 0.159 m, yaw RMSE 0.0514 rad
fault injection (IMU with zero stamp, IMU with NaN yaw rate) → rejected_invalid 2,
  last_error 'imu: non-finite value in angular_velocity'; node keeps RUNNING
```

Offline replay (`cargo run --release -p robotics_sim --example estimator_accuracy`, 180 s,
seeds 1–5): fused position RMSE 0.108–0.317 m vs raw 0.687–0.923 m, fused yaw RMSE
0.035–0.148 rad vs raw 0.226–0.312 rad, fused pose NEES 0.38–2.29 (expected 3; raw
145–291), NIS 0.90–0.98 (2 DoF).

Clean shutdown (Ctrl-C = SIGINT to the launch process group):

```text
launch exit code: 0 after .258799489 s
[state_estimator_node-3]: process has finished cleanly [pid 2442]
[estimator_evaluator-4]: process has finished cleanly [pid 2443]
[sim_node-1]: process has finished cleanly [pid 2440]
[wheel_odometry_node-2]: process has finished cleanly [pid 2441]
remaining robotics processes (expect none): none
```

Integration test (`scripts/integration_test_phase2.sh`, excerpt of 56 checks):

```text
[PASS] startup: nodes, data and TF available                after 1.0 s (timeout 40 s)
[PASS] /scan: single publisher sim_node                     publishers ["sim_node"]
[PASS] /scan: publisher QoS BestEffort/Volatile/KEEP_LAST 5 ok
[PASS] /tf: single publisher state_estimator_node           publishers ["state_estimator_node"]
[PASS] /tf_static: publisher QoS Reliable/TransientLocal/KEEP_LAST 100 ok
[PASS] /scan: rate 10 Hz ±15 %                              9.97 Hz (100 msgs / 10.0 s)
[PASS] /imu/data: rate 100 Hz ±15 %                         99.97 Hz (1003 msgs / 10.0 s)
[PASS] /odom: rate 50 Hz ±15 %                              50.03 Hz (502 msgs / 10.0 s)
[PASS] /scan: 360 beams in frame 'laser', >150 valid returns 360 beams, frame 'laser', min valid 217, malformed 0
[PASS] /scan stamps on the simulator clock (|scan − truth| < 0.5 s) 0.050 s
[PASS] /odom matches ground truth by stamp; position RMSE < 0.5 m 502 matched, 0 unmatched, RMSE 0.0064 m
[PASS] TF base_link -> laser is the static mount (0.05, 0, 0) Ok(Se2 { x: 0.05, y: 0.0, theta: 0.0 })
[PASS] TF odom -> laser chain = (odom->base_link)(base_link->laser) Ok(Se2 { x: 2.4467…, y: 0.0466…, theta: 0.3434… })
[PASS] TF: every frame has exactly one publisher (no authority conflict) conflicts []; base_link 1 pub, laser 1 pub, imu_link 1 pub; rejected 0
[PASS] shutdown: ros2 launch exits with code 0 after SIGINT Some(ExitStatus(unix_wait_status(0))) after 0.40 s
[PASS] shutdown: every node finished cleanly, no SIGTERM/SIGKILL escalation LaunchSummary { started: 4, finished_cleanly: 4, died: 0, escalations: 0 }
[PASS] shutdown: no orphaned node processes                 none
56 checks, 56 passed, 0 failed
```

## Phase 1 evidence (recorded 2026-09-26, WSL2 Ubuntu 24.04, ROS 2 Jazzy, rclrs 0.8.0, CycloneDDS)

Toolchain and environment detection:

```text
cargo 1.97.1 (c980f4866 2026-06-30)
ROS_DISTRO=jazzy RMW=rmw_cyclonedds_cpp
```

Library and bridge tests (`cargo test --workspace`):

```text
robotics_core        test result: ok. 23 passed; 0 failed
robotics_filters     test result: ok.  8 passed; 0 failed
robotics_models      test result: ok. 13 passed; 0 failed
robotics_ros_bridge  test result: ok.  8 passed; 0 failed
```

colcon build:

```text
Starting >>> robotics_ros_bridge      Finished <<< robotics_ros_bridge [18.2s]
Starting >>> robotics_estimation      Finished <<< robotics_estimation [44.5s]
Starting >>> robotics_examples        Finished <<< robotics_examples [44.5s]
Starting >>> robotics_bringup         Finished <<< robotics_bringup [15.0s]
Summary: 4 packages finished [1min 25s]
$ ros2 pkg executables robotics_examples
robotics_examples fake_wheel_encoders
robotics_examples qos_listener
robotics_examples qos_talker
$ ros2 pkg executables robotics_estimation
robotics_estimation wheel_odometry_node
```

Rust publisher → DDS → Rust subscriber, inspected with the standard CLI:

```text
$ ros2 run robotics_examples qos_talker --ros-args -p period_ms:=200 -p prefix:=phase1 &
$ ros2 run robotics_examples qos_listener --ros-args -p report_period_s:=3 &
$ ros2 node list
/qos_listener
/qos_talker
$ ros2 topic info -v /chatter
Type: std_msgs/msg/String
Publisher count: 1      Node name: qos_talker    QoS: Reliability: RELIABLE  History (Depth): KEEP_LAST (10)  Durability: VOLATILE
Subscription count: 1   Node name: qos_listener  QoS: Reliability: RELIABLE  History (Depth): KEEP_LAST (10)  Durability: VOLATILE
[qos_talker]:   published #22: "phase1 22" (1 subscriber(s))
[qos_listener]: #22 heard "phase1 22"
```

Parameters (declared with defaults, descriptions, ranges and read-only flags):

```text
$ ros2 param list /qos_talker
  period_ms  prefix  reliable  use_sim_time
$ ros2 param get /qos_talker prefix
String value is: phase1
$ ros2 param set /qos_talker prefix bonjour
Set parameter successful
$ ros2 param describe /qos_talker period_ms
Parameter name: period_ms  Type: integer  Description: Publish period in milliseconds (read at startup)
  Constraints: Min value: 10  Max value: 10000
$ ros2 param set /qos_talker period_ms 5
Setting parameter failed: Parameter value is out of range
$ ros2 param set /qos_talker reliable false
Setting parameter failed: Parameter is read only
```

Wheel odometry pipeline (`ros2 launch robotics_bringup wheel_odometry_demo.launch.xml`):

```text
$ ros2 topic list
/chatter /cmd_vel /diagnostics /joint_states /odom /parameter_events /rosout /tf
$ ros2 topic hz /joint_states
average rate: 50.001   min: 0.020s max: 0.020s std dev: 0.00009s window: 20
$ ros2 topic info -v /odom
Type: nav_msgs/msg/Odometry  Node name: wheel_odometry_node  Reliability: RELIABLE  History: KEEP_LAST (10)  Durability: VOLATILE
$ ros2 topic echo /odom --once
header: {frame_id: odom}  child_frame_id: base_link
pose.pose.position: {x: 0.2635577122716873, y: 1.1390151503452988}  orientation: {z: 0.9742583291687335, w: 0.2254344872581561}
pose.covariance[0]=9.04e-05 [7]=5.55e-05 [35]=1.16e-04 (x, y, yaw slots; cross terms at [1],[5],[11],[30],[31])
twist.twist: linear.x: 0.2998216061443526  angular.z: 0.49970267690737147   (commanded 0.30 m/s, 0.50 rad/s)
$ ros2 run tf2_ros tf2_echo odom base_link
At time 1790431812.795093720
- Translation: [-0.237, 1.151, 0.000]
- Rotation: in RPY (radian) [0.000, -0.000, -2.736]
$ ros2 topic echo /diagnostics --once
status: - name: 'wheel_odometry_node: odometry'  message: integrating
          values: input_rate_hz: '50.0'  received: '1250'  published: '1249'  dropped: '0'  x: '-0.046'  y: '0.002'  theta: '-0.076'
$ ros2 param get /wheel_odometry_node wheel_separation
Double value is: 0.16
[wheel_odometry_node]: odom pose x=-0.016 y=0.000 th=-0.026 | v=0.300 w=0.500 | sigma_xy=0.0116
```

The last log line is the circle test: at `v = 0.3 m/s`, `ω = 0.5 rad/s` the robot returns
to the origin every `2π/0.5 ≈ 12.6 s`, and dead reckoning from the synthetic encoders does.

Command path (`/cmd_vel` → fake base → encoders → odometry) and clean shutdown:

```text
$ ros2 topic echo /odom --once --field twist.twist
linear:  {x: 0.30028854726506377}   angular: {z: 0.5004809121084297}
$ ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -0.3}}"
[fake_wheel_encoders]: new command v=0.500 omega=-0.300
$ ros2 topic echo /odom --once --field twist.twist
linear:  {x: 0.5002766279614221}    angular: {z: -0.30016597677687906}

$ kill -INT <qos_talker> <qos_listener>          # Ctrl-C
[qos_talker]:   qos_talker shutting down          → exit code 0
[qos_listener]: qos_listener shutting down        → exit code 0
$ kill -INT <ros2 launch wheel_odometry_demo>    # Ctrl-C on the launch
[fake_wheel_encoders-1]: fake_wheel_encoders shutting down
[wheel_odometry_node-2]: wheel_odometry_node shutting down
[INFO] [fake_wheel_encoders-1]: process has finished cleanly [pid 17985]
[INFO] [wheel_odometry_node-2]: process has finished cleanly [pid 17986]
launch exit code: 0
```

Static analysis: `cargo clippy --workspace --all-targets -- -D warnings` passes;
`cargo fmt --all --check` passes.

## Ecosystem limitations found (rclrs 0.8.0) and how the framework handles them

* **No signal handler**: a spinning executor ignores SIGINT; `robotics_ros_bridge::shutdown`
  installs one (`ctrlc` → `ExecutorCommands::halt_spinning`) and every node calls it.
* **Message crates** come from an interface underlay rebuilt with `rosidl_generator_rs`;
  rclrs needs `rcl_interfaces` and `rosgraph_msgs` in it or it does not compile.
* **No tf2**: the framework has its own planar transform buffer and `TfListener`
  (`docs/TF_BUFFER.md`).
* **No lifecycle nodes, no composition; `colcon test` does not run Rust tests** — the
  integration test is a Rust binary driven by `scripts/integration_test_phase2.sh`; each
  `ament_cargo` package recompiles its dependencies.
* **Workers + CycloneDDS**: worker wait-set threads stalled on DDS graph changes; nodes use
  node-level callbacks and resume after spurious `rcl_wait` timeouts
  (`docs/STATE_ESTIMATION.md` § 6).
* `MessageInfo::publication_sequence_number` is `u64::MAX` on CycloneDDS (unsupported);
  `publisher_gid` works and is used for TF authority checks.

## License

Apache-2.0 (see `LICENSE`, `NOTICE`). The reference book repository carries no license;
nothing from it is copied, algorithms are re-implemented from the mathematics and cited by
chapter.
