# Architecture

`ros2-rust-robotics` turns the algorithms of *Probabilistic Robotics via Rust* into a
modular ROS 2 autonomy framework whose runtime language is Rust. The one rule that shapes
everything else: **algorithms never see ROS**.

```text
 Reference book (design docs + Rust listings + TypeScript port)
        │   understand, re-derive, re-implement (docs/SOURCE_MAPPING.md)
        ▼
 Pure Rust robotics libraries            crates/robotics_*        no ROS, no threads,
        │                                                          deterministic when seeded
        ▼
 ROS adapters                            ros2_ws/src/robotics_ros_bridge
        │   message ⇄ core type converters, QoS presets, TF, diagnostics
        ▼
 ROS 2 Rust nodes (rclrs 0.8)            ros2_ws/src/robotics_*   thin: params → adapter →
        │                                                          library call → adapter → publish
        ▼
 DDS (CycloneDDS / Fast DDS via rmw)     standard sensor_msgs / nav_msgs / geometry_msgs / tf2
        │
        ▼
 Robot / simulator / RViz / Nav2 / rosbag2
```

## 1. Layers

### 1.1 Pure Rust libraries (`crates/`)

| Crate | Owns | Book chapters | Phase |
|---|---|---|---|
| `robotics_core` | SE(2) geometry (`Se2`, `Manifold`), Gaussians (moments + canonical), normal/χ² quantiles, seeded RNG, `Timestamp`, `FrameId`, the internal data model (`LaserScan`, `PointCloud2`, `Odometry2`, `ImuSample`, `ImuMeasurement`, `Twist2`, `Cov3`, `Stamped<T>`), the planar `TransformBuffer` (`tf`) | 2, 3, 4, 6 | 1–2 ✅ |
| `robotics_models` | differential-drive kinematics + wheel odometry, velocity/odometry motion models (`MotionModel` trait); sensor models: gyro, beam mixture + EM, likelihood field over an exact EDT, range–bearing landmark model, tempering | 4, 9, 10 | 1–2 ✅ |
| `robotics_filters` | `BayesFilter`, `HistogramFilter1D`, `KalmanFilter`, `InformationFilter`, `RtsSmoother`, NEES/NIS (`consistency`), `Ekf`/`Ukf` over `Manifold`, `EskfSe2`, `OdomImuFusion`, `ParticleSet` (low-variance/multinomial, ESS), KLD sizing (`kld`) | 5–8 | 1–2 ✅ |
| `robotics_localization` | EKF landmark localization, MCL / Augmented MCL, grid localization | 11, 12 | 3 |
| `robotics_mapping` | log-odds occupancy grid, inverse sensor model, entropy, map-accuracy evaluation, signed ESDF over the exact EDT. 2-D TSDF and quadtree (Ch. 19) are *not* Phase 4: a Phase 5+ concern | 13, 19 (ESDF) | 4 |
| `robotics_slam` | ICP/NDT, voxel map, pose graph + factor-graph optimizer (GN/LM, robust kernels), EKF-SLAM, FastSLAM/RBPF, RustSLAM-2D pipeline | 14–17 | 5 |
| `robotics_planning` | lattice A*/Dijkstra, PRM, RRT/RRT*, Dubins, MDP/POMDP solvers | 20–22 | 6 |
| `robotics_control` | MPPI, DWA, path tracking | 23 | 7 |
| `robotics_navigation` | frontier detection, information gain, utility, stop rules, supervisor state machine, safety margin | 24, 26 | 8–9 |
| `robotics_sim` | segment `World` + exact ray cast, Rusty robot (slip, calibration error), encoders, gyro, 2-D LiDAR, per-source RNG streams, bit-exact replay (`docs/SIMULATION.md`) | 4 | 2 ✅ |

Rules for a library crate:

* Depends on `robotics_core` (and `nalgebra`); never on `rclrs`.
* Every public fallible operation returns `Result<_, Error>`; no `unwrap()` outside tests.
* Every stochastic routine takes `&mut impl Rng`; no thread-local RNG.
* Every algorithm carries a `Reference: … Chapter N` doc comment.
* Numeric behaviour pinned by tests that reproduce the book's worked examples.

### 1.2 ROS adapter (`robotics_ros_bridge`)

The only crate where `ros_env::*_msgs` and `robotics_core` types meet.

* `convert`: `pose_to_ros / pose_from_ros`, `odometry_*`, `laser_scan_*` (REP 117 ranges
  kept, metadata validated), `imu_from_ros`, `imu_measurement_*` (strict planar IMU),
  `transform_stamped_*` (planar TF), `pose_with_covariance_stamped_to_ros`, `cov3_*`
  (planar `(x, y, θ)` ⇄ 6×6 slots `(0, 1, 5)`), `time_*`. Malformed messages become
  `ConvertError`, never panics.
* `qos`: one preset per data semantic (sensor, odometry, map-latched, command, tf, tf_static,
  diagnostics). See `docs/ROS2_INTERFACES.md` for the rationale.
* `tf`: `TfBroadcaster` on `/tf` and `/tf_static`, and `TfListener` feeding a
  `robotics_core::TransformBuffer` with TF-authority conflict detection (rclrs has no tf2
  binding; `docs/TF_BUFFER.md`). `transform_stamped_from_ros` rejects non-planar rotations.
* `shared` / `shutdown`: `Shared<T>` node state for node-level callbacks, SIGINT/SIGTERM
  handling and a spin loop that survives spurious `rcl_wait` timeouts (§2).
* `params`: `ParamStore` — read-only parameters with ranges and descriptions.
* `diagnostics`: `DiagnosticsPublisher` + `StatusBuilder` for `/diagnostics`.

### 1.3 Nodes (`ros2_ws/src/robotics_*`)

A node is a `main()` that: declares parameters → builds the library object → creates
publishers with explicit QoS → registers node-level callbacks whose mutable state is a
`Shared<T>` → converts messages at the boundary → publishes results, TF and diagnostics.
No algorithmic code lives in a node. Packages are grouped by subsystem, not by algorithm:

| Package | Type | Nodes (phases 1–2) | Planned |
|---|---|---|---|
| `robotics_ros_bridge` | ament_cargo lib | – (converters, QoS, TF broadcaster + listener, diagnostics) | – |
| `robotics_examples` | ament_cargo bins | `qos_talker`, `qos_listener`, `fake_wheel_encoders` | – |
| `robotics_estimation` | ament_cargo bins | `wheel_odometry_node`, `state_estimator_node` ✅ | – |
| `robotics_sim_bridge` | ament_cargo bins | `sim_node` (`/scan`, `/joint_states`, `/imu/data`, ground truth, static sensor TF), `estimator_evaluator` ✅ | – |
| `robotics_integration` | ament_cargo lib + bins | `phase2_stack_test` (Rust `launch_testing` equivalent) ✅ | one test per phase |
| `robotics_bringup` | ament_cmake | launch XML + YAML params (`pubsub_demo`, `wheel_odometry_demo`, `estimator_sim`) | per-phase launch files |
| `robotics_localization` | ament_cargo | – | `localization_node` (phase 3) |
| `robotics_mapping_ros` | ament_cargo | – | `ground_truth_tf_adapter` (phase 4, simulation only), `mapping_node`, `mapping_evaluator` (phase 4). Named `_ros` because a Cargo workspace cannot hold the crate `robotics_mapping` and a package of the same name |
| `robotics_slam` | ament_cargo | – | `scan_matcher_node`, `pose_graph_node` (phase 5) |
| `robotics_planning` | ament_cargo | – | `planner_node` (phase 6) |
| `robotics_control` | ament_cargo | – | `controller_node` (phase 7) |
| `robotics_navigation` | ament_cargo | – | `explorer_node`, `supervisor_node` (phase 8–9) |

## 2. Threading and state

rclrs 0.8 offers two ways to hold mutable node state: `Worker`s (each with its own
wait-set thread) and node-level callbacks. With CycloneDDS, processes whose wait sets run
on several threads were observed to stall for seconds whenever another DDS participant
(e.g. a `ros2` CLI) joined or left; the same code built from node-level primitives did
not (`docs/STATE_ESTIMATION.md` § 6). The framework therefore uses **node-level
callbacks** on the single executor thread, with the node's mutable state (filter,
integrator, counters) in one `robotics_ros_bridge::shared::Shared<T>` (an
`Arc<Mutex<T>>` that recovers from poisoning). Callbacks are serialized by the executor,
so the lock is uncontended. `robotics_ros_bridge::shutdown::spin` resumes spinning after
spurious `RCL_RET_TIMEOUT`s and returns on SIGINT/SIGTERM, so Ctrl-C ends every node with
exit code 0. Long computations (planning, optimization) will run on dedicated threads
with bounded channels (phase 5+).

## 3. Time and frames

* All internal time is `Timestamp` (ns). Nodes stamp outputs with the *input* stamp when
  it is set (`use_header_stamp`), else the node clock; `use_sim_time` works through rclrs's
  node clock.
* Frames are `FrameId` values; names are parameters, defaults follow REP 105:
  `map → odom → base_link → <sensor>`.
* TF ownership: `odom → base_link` = the state estimator (the wheel odometry node only
  when it runs alone, Phase 1 demo); `map → odom` = localization or SLAM, never both at
  once (bringup selects one). In simulation, Phase 4 adds a third possible owner of
  `map → odom`, `ground_truth_tf_adapter`, which publishes it from the simulator's ground
  truth; it is exclusive with `localization_node` under the same rule. Sensors → `base_link` are static: in simulation `sim_node`
  publishes `base_link → {imu_link, laser}` in one `/tf_static` message; on a robot the
  bringup's static publisher / `robot_state_publisher` does. Consumers look transforms up
  through `TfListener` (`docs/TF_BUFFER.md`).

## 4. Build system

* One Cargo workspace at the repo root. `default-members` = pure crates, so `cargo test`
  works on any machine without ROS. `cargo build --workspace` (or `colcon build` in
  `ros2_ws`) builds the ROS packages and needs a sourced ROS 2 Jazzy environment plus the
  interface underlay (`scripts/setup_interfaces_underlay.sh`).
* `colcon-ros-cargo` builds each `ament_cargo` package via `cargo ament-build`, installing
  binaries so `ros2 run` / `ros2 launch` find them.
* Messages: standard ROS interface packages are used unmodified. Their Rust bindings come
  from `rosidl_generator_rs` in the underlay and reach the code via `ros_env::<pkg>::msg::*`.

## 5. Determinism and testing

Library crates are tested with worked-example regressions, property tests (`proptest`)
and statistical checks on seeded RNG streams. ROS packages have unit tests for converters
and QoS (run with `cargo test --workspace` in a sourced env) and are verified end to end
in two ways: with the ROS CLI (`scripts/verify_phase2_runtime.sh`, evidence in
`README.md`) and with Rust-first integration tests (`robotics_integration`): a binary
spawns the launch file in its own process group, asserts on the live graph (nodes, topic
types, publisher QoS, rates, message content, stamps, TF through `TfListener`, TF
authority, estimator accuracy), sends SIGINT and requires every process to finish
cleanly (`scripts/integration_test_phase2.sh`). `colcon test` does not run Rust tests, so
these are scripts rather than `colcon test` targets.

## 6. Portability

Nothing in `crates/` uses `unsafe`, threads or platform APIs, so the libraries build for
x86_64 industrial PCs, aarch64 (Jetson, robot onboard computers) and `wasm32`. Robot
specifics (Unitree, AMR, humanoid) enter only through adapter nodes that map their driver
topics onto the standard interfaces the framework consumes.
