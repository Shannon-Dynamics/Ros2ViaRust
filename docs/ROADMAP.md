# Roadmap

Each phase ends with: library tests green, ROS node running under `ros2 launch`, CLI
evidence recorded in `README.md`, and `SOURCE_MAPPING.md` updated. Phases are ordered by
dependency, not by chapter number.

Problem-based write-up of these phases, one part each: [README.md](README.md).

## Phase 1 — Foundation ✅

* Cargo + colcon workspace; interface underlay with Rust bindings; `scripts/env.sh`.
* `robotics_core`: `Se2` (exp/log/adjoint/⊞/⊟), `Manifold`, `Gaussian`/`Canonical`,
  `Timestamp`, `FrameId`, internal data model.
* `robotics_models`: diff-drive kinematics, `WheelOdometry` with covariance compounding,
  velocity and odometry motion models (sampler + density).
* `robotics_filters`: `BayesFilter`, `HistogramFilter1D`, `KalmanFilter` (Joseph form).
* `robotics_ros_bridge`: converters, QoS presets, `TfBroadcaster`, diagnostics.
* Nodes: `qos_talker`, `qos_listener`, `fake_wheel_encoders`, `wheel_odometry_node`;
  launch files and YAML config.

## Phase 2 — State estimation, sensor models, simulation ✅ (closed 2026-09-27)

* ✅ `robotics_filters`: `Ekf`/`Ukf` generic over `Manifold`, error-state EKF on `Se2`
  (Ch. 7); `InformationFilter` + `RtsSmoother` (Ch. 6); `ParticleSet` with log weights,
  low-variance and multinomial resampling, ESS; KLD-adaptive sizing with pluggable
  binning (`kld`, Ch. 8); NEES/NIS with χ² bounds (`consistency`).
* ✅ `robotics_models::sensor`: beam model with EM intrinsics (Table 6.2), likelihood
  field over an exact EDT, range–bearing landmark model with pose Jacobian and pose
  sampler, likelihood tempering incl. ESS-targeted β (Ch. 10).
* ✅ `robotics_sim`: segment `World` with exact ray casting, Rusty kinematics, slip,
  encoders, gyro, 2-D LiDAR, per-source RNG streams, bit-exact replay (Ch. 4); `sim_node`
  publishes `/scan`, `/joint_states`, `/imu/data`, ground truth, static
  `base_link → {imu_link, laser}`.
* ✅ `state_estimator_node`: fuses `/wheel/odometry` + `/imu/data` with the ESKF; publishes
  `/odom`, `/state_estimate`, `odom → base_link` (sole owner), mean NIS on `/diagnostics`.
* ✅ TF buffer (`robotics_core::tf`) and `TfListener` in the bridge (rclrs has none), with
  TF-authority conflict detection; Rust-first `launch_testing` equivalent
  (`robotics_integration/phase2_stack_test`, 56 checks incl. SIGINT shutdown).

Exit evidence: `scripts/verify_phase2_all.sh` (fmt, clippy `-D warnings`, `cargo test`,
underlay, `colcon build`, runtime CLI checks, integration test), results in `README.md`
§ Evidence and `docs/STATE_ESTIMATION.md` § 4. Nothing from the Phase 2 contract was
deferred.

## Phase 3 — Localization (in progress)

Built and running: the EKF half, and the infrastructure around it.

* `crates/robotics_localization` (`map`, `pillars`, `ekf`): EKF localization on SE(2) with
  maximum likelihood association and a chi-square gate.
* `localization_node` and `localization_evaluator` in
  `ros2_ws/src/robotics_localization_bridge`; the node publishes `/localization/pose` and,
  since 2026-10-10, owns the `map → odom` correction (REP 105), so the TF chain
  `map → odom → base_link → {LDS-01, imu_link}` is complete.
* One bringup for the whole phase,
  `ros2_ws/src/robotics_bringup/launch/phase3_localization.launch.py`, which includes the
  Webots world launch and starts the localizer, the evaluator, the route driver and the
  odometry noise injector behind one set of arguments.
* The integration test, `scripts/verify_phase3_localization.sh`: 71 checks over the
  installed binaries, the run guards, bringup, topics and types, publication, the whole TF
  tree, the node's own diagnostics and parameters, the route driver and a clean shutdown.
  All 71 pass; the run is `evidence/phase-3-localization-runtime/2026-10-10/`.
* The measurement harness: the Webots world with its pillars, the ground truth and teleport
  supervisor, and `route_driver`, which drives a waypoint route closed loop on ground truth
  so that the geometry of a long route is a property of the route file rather than of how
  much the wheels slipped. Runs are kept as raw output under
  `evidence/phase-3-correctness/` and judged by `scripts/phase3_verdict.py`, never by a
  number typed into prose.

Not built yet: MCL and Augmented MCL, and with them global localization and kidnap
recovery; grid localization as a debug mode; `/particle_cloud`, `/amcl_pose` and
`/initialpose`; the kidnap scenario has a harness but has never been run; and every
benchmark row in `benchmarks/phase-3-localization.csv` is still empty, so no Rust against
rclpy against Nav2 comparison exists yet.

Criterion P1 of `docs/phase-3-localization/problem.md` is met on the two parts a run can
decide, on the odometry the simulator publishes with nothing injected: the filter's position
RMSE is below the dead reckoning it was given, and its error is flat over distance while dead
reckoning's grows. The NEES part of the same row is not evaluated, because it needs the full
covariance of each estimate and the evaluator CSV carries only `sigma_xy`. Read the verdict
file of the run rather than this paragraph; `docs/phase-3-localization/benchmark.md` section 6
lists the runs, and `theory.md` section 6 explains why the route had to be lengthened to 51 m
before the comparison meant anything. Details and the open decisions are in
`docs/phase-3-localization/implementation.md`.

Inputs already available from Phase 2: `/scan` with a static `base_link → laser`, `/odom`
and `odom → base_link`, the Rust `TfListener`, `World::raycast` as a `z*` source, beam /
likelihood-field / landmark models, tempering, `ParticleSet` + KLD sizing with
`Se2Binning`, `EskfSe2::update_position` / `update_heading` for absolute corrections.

* `robotics_localization`: EKF localization with ML association + χ² gating (Ch. 11),
  MCL and Augmented MCL with `w_fast/w_slow` injection, KLD adaptation, dominant-cluster
  estimate (Ch. 12), grid localization as a debug mode.
* `localization_node`: `/scan` + `/odom` + `/map` (+ `/initialpose`) → `/pose`,
  `/particle_cloud`, `map → odom`. Nav2-compatible outputs (`/amcl_pose`).

## Phase 4 — Mapping

* `robotics_mapping`: log-odds `OccupancyGrid` with hand-crafted inverse model and
  Bresenham/supercover integration, entropy (Ch. 13); exact EDT/ESDF and 2-D TSDF (Ch. 19).
* `mapping_node`: `/scan` + `map → base_link` → `/map` (latched), `/map_entropy`.

## Phase 5 — SLAM

* `robotics_slam`: point-to-point / point-to-plane ICP with voxel-hash correspondence and
  KISS-style adaptive threshold, NDT, ICP information matrix (Ch. 16); factor-graph
  optimizer (GN/LM, robust kernels, sparse via `faer`, Ch. 15); pose graph with loop
  candidates, ICP verification and χ² gate; EKF-SLAM (Ch. 14) and grid RBPF (Ch. 17) as
  alternative back ends.
* `scan_matcher_node` (LiDAR odometry) and `pose_graph_node` (keyframes, loop closure,
  optimization, submap-rendered `/map`, `map → odom`). No faked loop closure: until the
  detector and verifier are implemented, the node reports `loop_closure: unsupported`.

## Phase 6 — Planning

* `robotics_planning`: lattice A*/Dijkstra with octile heuristic, PRM, RRT/RRT* with
  `Steer` trait, Dubins words, C-space with ESDF clearance (Ch. 20); MDP value/policy
  iteration and POMDP (α-vectors, PBVI, POMCP) as decision layer (Ch. 21–22).
* `planner_node`: `/map` (+ `/esdf`) + `/goal_pose` → `/plan`; `NavigateToPose` action.

## Phase 7 — Control

* `robotics_control`: pure-pursuit baseline, DWA, MPPI with information-theoretic
  weights, cross term, Savitzky–Golay smoothing, ESDF obstacle cost (Ch. 23).
* `controller_node`: `/plan` + `/odom` + `/esdf` → `/cmd_vel`; safety envelope (max
  speeds, minimum clearance, stop on stale inputs) enforced before optimization.

## Phase 8 — Exploration

* `robotics_navigation`: frontier detection/clustering, ray-cast information gain,
  navigation cost field, utility scoring, stop rules (Ch. 24).
* `explorer_node` with an `Explore` action (feedback: coverage, entropy, gain rate).

## Phase 9 — Complete autonomy stack

* Supervisor state machine (`Explore | Navigate | Relocalize | Recover | Done`),
  dual-EMA kidnap detector, NIS divergence gate, watchdogs, chance-constrained safety
  margin (Ch. 26); mission manager; bringup for sim and for a real diff-drive base.
* Benchmarks (`criterion`): PF update, ICP, map update, planner, MPPI rollouts.

## Cross-cutting backlog

* Lifecycle nodes: rclrs 0.8 has no lifecycle support; nodes expose `~/reset` services
  and rely on launch-level supervision until upstream adds it.
* Composition: rclrs has no component container; one process per node for now.
* Custom interfaces (`robotics_interfaces`) once phase 5 needs graph/landmark messages.
* TF: planar buffer only; a 3-D buffer when a 3-D consumer appears (`docs/TF_BUFFER.md`).
* Integration tests are binaries run by `scripts/integration_test_*.sh`: `colcon test`
  does not run Rust tests.
