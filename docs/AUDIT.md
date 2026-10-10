# Audit of the reference repository

> **Snapshot.** This audit was written on 2026-09-26, before Phase 2 was implemented;
> statements such as "phase 2" or "planned" below describe the plan at that time. Current
> status: `docs/ROADMAP.md` (Phases 1–2 complete) and `docs/SOURCE_MAPPING.md`.

Date: 2026-09-26. Scope: `D:\Shannon\ROS2 via RUST\ProbabilisticRoboticRust-main\ProbabilisticRoboticRust-main`
(read-only; every file class listed below was opened, not inferred from names).

## A. Repository understanding

* An interactive **web book**, *Probabilistic Robotics via Rust* (Shannon Dynamics): Next.js
  16 static site in `web/` (26 MDX chapters, ~99 widgets), a LaTeX print edition in
  `pr-latex/` generated from the MDX, and 26 per-chapter design docs `Chapter-NN.md` plus
  `TOC.md` (the contract: notation, crate stack, chapter plan) and `CLAUDE.md`.
* **There is no Rust project.** `find` returns zero `.rs`/`Cargo.toml` files. The Rust the
  book "teaches" appears only as ~140 fenced listings in the MDX (~8 800 lines), extracted
  for this audit into scratch files. Many listings reference helpers that are never shown
  (`pr_core::geom::se2::SE2`, `associate`, `rasterize`, …) and are not compilable as
  printed. The design docs describe a planned `pr_*` cargo workspace that was never
  created.
* The **runnable implementation is TypeScript** (`web/lib/`, ~31 000 lines, 90 files),
  organised as `prob/ geom/ sim/ models/ filters/ localize/ mapping/ slam/ optim/ vision/
  plan/ decision/ pomdp/ control/ explore/ learn/ capstone/`, with numerical invariants in
  `web/lib/__checks__.ts` (32 checks) and per-chapter check files (ch15, 18, 20, 21, 23,
  25, capstone). These invariants are the most valuable artefact: hand-computed numbers
  that pin every algorithm.
* A stray `log/` directory (colcon logs from 2026-09-26) sits in the reference root; it was
  left untouched.
* **No license file** and no license statement anywhere. The repository must be treated as
  all-rights-reserved reference material; the framework re-implements algorithms from the
  mathematics and cites chapters, copying nothing.

## B. Algorithm inventory (candidates for reusable Rust modules)

Geometry/probability: SE(2) exp/log/adjoint/⊞/⊟, screw interpolation, Gaussian
moments/canonical/conditioning/entropy, seeded sampling.
Models: diff-drive kinematics & encoder odometry, velocity/odometry/tangent motion models,
map-conditioned motion, beam model + EM, likelihood field (exact EDT), landmark model,
tempering.
Filters: Bayes trait, histogram filter, KF (Joseph), information filter, RTS, EKF/UKF on
manifolds, ESKF SE(2), particle set + low-variance/multinomial resampling, KLD sizing,
NEES/NIS.
Localization: EKF localization + ML association + gating, MHT, grid localization, MCL,
Augmented MCL, novelty filtering, dominant-cluster estimate.
Mapping: log-odds occupancy grid with Bresenham, inverse models (hand-crafted, learned),
MAP mapping, multi-sensor fusion, entropy, quadtree, TSDF, ESDF/EDT, marching squares.
SLAM: EKF-SLAM with provisional landmarks, factor graph (GN/LM, kernels, Schur,
orderings), ICP point-to-point/point-to-plane, NDT, voxel-hash map, KISS adaptive
threshold, pose graph + loop closure + χ² gating, RustSLAM-2D pipeline, FastSLAM 1.0/2.0,
grid RBPF, pinhole/BA/IMU preintegration/sliding window.
Planning/decision: C-space, lattice A*/Dijkstra/wavefront/brushfire, potential fields,
PRM, RRT/RRT*, Dubins/Reeds–Shepp-lite, MDP VI/PI/prioritised sweeping, POMDP exact
VI/PBVI/POMCP/QMDP, AMDP coastal navigation.
Control: MPPI (information-theoretic weights, cross term, SG smoothing), DWA.
Exploration: frontier detection/clustering, ray-cast info gain, nav cost field, graph
log-det, utility, stop rules, active localization.
Learning: calibration/ECE/PIT, differentiable 1-D KF, soft resampling, temperature fit.
Capstone: task bus, supervisor FSM, dual-EMA kidnap detector, χ² gates, watchdog,
chance-constrained safety margin, relocalization MCL.

## C. Existing Rust inventory (honest status)

| Status | Chapters / items |
|---|---|
| Complete listings (compilable modulo elided helpers) | 1, 2, 3, 5 (one uncompilable field), 6 (KF/IF/RTS), 7 (EKF, UT), 8, 9 (models), 10 (beam/EM/landmark), 11 (EKF loc), 12 (MCL/AMCL step), 13 (grid, inverse model), 14 (EKF-SLAM core), 15 (factors, faer LM), 16 (voxel map, SVD align, ICP, pose graph), 17 (landmark EKF, FastSLAM, RBPF), 18 (pinhole, BA step, preint), 19 (TSDF, ESDF), 20 (C-space, A*, PRM, RRT*), 21 (MDP), 22 (POMDP model, QMDP, PBVI), 23 (MPPI), 24 (frontier, gain, utility), 25 (calibration, diff-KF, soft resample), 26 (bus, supervisor, margin) |
| Fragments / elided bodies | Ch. 4 World & `Lidar::scan`, Ch. 7 UKF filter struct & ESKF correct, Ch. 9 `NoiseKind`, Ch. 10 `rasterize`, Ch. 12 grid localizer, Ch. 13 learned/MAP mapping, Ch. 15 sparse helpers, Ch. 16 `Slam2d`/`Submap`/loop detection, Ch. 18 MSCKF/marginalization, Ch. 20 Dubins bodies/hybrid A*, Ch. 22 POMCP helpers/AMDP, Ch. 23 DWA, Ch. 24 `NavField` paths/active localization, Ch. 25 candle beam net, Ch. 26 task implementations & `run_mission` |
| Prose only | map-conditioned motion, map correlation model, KLD-MCL, DESPOT, colored-noise MPPI, iLQR, predicted-loop-closure augmentation, diffusion policy, Cartographer-style branch-and-bound, descriptor loop detection, `tf`/ROS integration |
| TypeScript-only implementations | ~all of the above have complete TS versions; DWA, quadtree, marching squares, prioritised sweeping, POMCP tree, AMDP, active localization, capstone tasks exist **only** in TS |

Naming hazard: TS uses control-theory `F,B,H,Q,R`; the Rust listings and the framework use
Thrun's `A,B,C,R(motion),Q(measurement)`.

## D. ROS 2 mapping

See `docs/SOURCE_MAPPING.md` (chapter → crate) and `docs/ROS2_INTERFACES.md`
(topics, QoS, TF ownership). Summary of nodes: `wheel_odometry_node` (Ch. 4; owns
`odom→base_link`), `state_estimator_node` (Ch. 6–7; `/imu/data` + `/wheel/odometry` →
`/odom`, `/state_estimate`), `localization_node` (Ch. 11–12; `/scan`, `/odom`, `/map` →
`/pose`, `/particle_cloud`, owns `map→odom`), `mapping_node` (Ch. 13/19; `/scan` + TF →
`/map`), `scan_matcher_node` + `pose_graph_node` (Ch. 15–16; `/scan`, `/odom` →
`/slam/pose`, `/slam/path`, `/slam/graph`, `/map`, owns `map→odom` instead of
localization), `planner_node` (Ch. 20; `/map`, `/goal_pose` → `/plan`; `NavigateToPose`
action), `controller_node` (Ch. 23; `/plan`, `/odom`, `/esdf` → `/cmd_vel`),
`explorer_node` (Ch. 24; `Explore` action), `supervisor_node` (Ch. 26; mode, chaos
services), `sim_node` (Ch. 4 world → `/scan`, `/joint_states`, ground truth).
Parameters: noise α's, σ's, frame names, topic names, map resolution, particle counts,
gates, cost weights; all declared with ranges/descriptions.

## E. Dependency audit

Existing (book's verified stack, TOC §2): nalgebra 0.35, faer 0.24, rand 0.9 +
rand_distr 0.6, statrs 0.19, sophus, parry2d 0.30, petgraph 0.8, pathfinding 4.15,
rayon, candle/burn, adskalman (tests), factrs 0.3, egui/eframe (widgets, not needed).

Chosen for phase 1 (all ROS-independent except the last two):

| Crate | Version | Why | Production | Portability |
|---|---|---|---|---|
| nalgebra | 0.35 | fixed-size vectors/matrices, Cholesky | mature | no_std-capable, WASM ok |
| rand | 0.10 (rand_distr 0.6 requires it; `SmallRng` no longer feature-gated) | seeded RNG | mature | ok |
| rand_distr | 0.6 | Normal/StandardNormal (Ziggurat) | mature | ok |
| thiserror | 2 | error enums | mature | ok |
| approx, proptest | dev | tests | – | – |
| rclrs | 0.8.0 | ROS 2 client library | evolving, "no stability guarantees" | Linux (bindgen + libclang), macOS/Windows CI exist |
| ros-env | 0.3.0 | message crates from AMENT_PREFIX_PATH | new in 0.8 | textual include: long builds with many crates |

Recommended later: faer (sparse Cholesky for pose graphs), parry2d (ray cast/collision),
petgraph (graphs, union-find), rayon (rollouts/particles, native only), criterion
(benchmarks). Avoid: candle/burn until Ch. 25 is actually needed; sophus only for 3-D.

## F. Architecture proposal

See `docs/ARCHITECTURE.md`. Workspace tree (phase 1 realised, later crates listed in the
roadmap):

```text
ros2-rust-robotics/
├── Cargo.toml                 # one workspace; default-members = pure crates
├── README.md  LICENSE  NOTICE
├── docs/  ARCHITECTURE.md  SOURCE_MAPPING.md  ROS2_INTERFACES.md  ROADMAP.md  BUILD.md  AUDIT.md
├── scripts/ env.sh  setup_interfaces_underlay.sh  interfaces.repos  sync_to_wsl.sh
├── crates/
│   ├── robotics_core/         # geometry, probability, time, frames, data model
│   ├── robotics_models/       # diff-drive, motion models (sensor models: phase 2)
│   ├── robotics_filters/      # BayesFilter, histogram, KF (EKF/UKF/PF: phase 2)
│   └── (robotics_localization, robotics_mapping, robotics_slam, robotics_planning,
│        robotics_control, robotics_navigation, robotics_sim — later phases)
└── ros2_ws/src/
    ├── robotics_ros_bridge/   # converters, QoS, TF, diagnostics (lib)
    ├── robotics_examples/     # qos_talker, qos_listener, fake_wheel_encoders
    ├── robotics_estimation/   # wheel_odometry_node (+ state_estimator_node: phase 2)
    └── robotics_bringup/      # launch XML + YAML
```

Fewer packages than the prompt's sketch: `robotics_math`/`robotics_geometry` are folded
into `robotics_core` (they would be micro-crates); nodes are grouped by subsystem.

## G. Implementation roadmap

`docs/ROADMAP.md`: 1 foundation ✅ → 2 state estimation → 3 localization → 4 mapping →
5 SLAM → 6 planning → 7 control → 8 exploration → 9 full stack + benchmarks.

## H. Risks

* **ros2-rust maturity**: rclrs 0.8.0 (released 2026-09-23) states "no stability
  guarantees"; the 0.8 API (executor/worker/`ros-env`) differs from every older tutorial,
  including the tutorial shipped in its own `docs/` (still shows 0.4-era `Node::new`,
  `QOS_PROFILE_DEFAULT` args). Mitigation: nodes are thin; API churn touches only the
  bridge and `main()` functions.
* **Message bindings**: `ros-env` requires an interface underlay rebuilt with
  `rosidl_generator_rs`; rclrs itself needs `rcl_interfaces` + `rosgraph_msgs` in it
  (verified: 48 compile errors otherwise). Interface crates generated by an older
  generator that already sit in `/opt/ros/jazzy` (bond, nav2_msgs, slam_toolbox, …, with
  `rosidl_runtime_rs = "0.6"`) are also picked up; they compiled in this session but are a
  latent breakage source.
* **Missing ROS features in rclrs**: no tf2 listener/buffer (broadcasting is trivial;
  a buffer must be written in Rust — done in Phase 2, `docs/TF_BUFFER.md`), no lifecycle nodes, no composition/component
  containers, `colcon test` not supported for Rust packages, each `ament_cargo` package
  rebuilds its dependencies (slow colcon builds).
* **Jazzy**: verified working with `rmw_cyclonedds_cpp`; Fast DDS untested here.
* **Performance**: particle filters, ICP and MPPI rollouts are the hot paths; the book
  uses rayon natively. Benchmarks are scheduled for phase 9 but the APIs already avoid
  per-cycle allocation where it matters (`Se2` is `Copy`, scans are borrowed).
* **Unsafe/FFI**: none in framework code (`#![forbid(unsafe_code)]`); FFI is confined to
  rclrs.
* **Dependencies**: rand 0.9→0.10 churn already bit once; pin minor versions in
  `Cargo.lock` (committed).
* **Algorithms described but not implemented in Rust**: see §C. Everything the roadmap
  promises has at least a complete TS reference or a complete listing; items that have
  neither (DESPOT, hybrid A*, Cartographer matcher, learned beam net) are marked
  unsupported until implemented, never faked.
