# Phase 3, implementation: the Rust side

> **State.** The first half of Phase 3 exists and runs: EKF position tracking against the
> pillar map, in [`crates/robotics_localization`](../../crates/robotics_localization/)
> (`map`, `pillars`, `ekf`) and wired to the topics by
> [`localization_node`](../../ros2_ws/src/robotics_localization_bridge/src/localization_node.rs)
> in `ros2_ws/src/robotics_localization_bridge`, graded against the simulator's ground
> truth by `localization_evaluator` next to it. Runs and their raw output live in
> [`../../evidence/phase-3-correctness/`](../../evidence/phase-3-correctness/).
>
> The second half does not exist yet: Monte Carlo localization, and with it global
> localization and kidnap recovery (`lib.rs` of the crate says so too), plus the grid
> localization debugging mode, the `robotics_bringup` launch and parameter files, and the
> integration test. Sections 2 and 3 below are still the plan, and the paths they name are
> the planned ones, not always the ones that were built. See [ROADMAP.md](../ROADMAP.md)
> for the phase contract.

## 1. Already in the repository

The estimation pieces [theory.md](theory.md) needs are written and tested; Phase 3 composes
them rather than starting over.

| Piece | Where | State |
|---|---|---|
| Bayes filter trait | [`bayes.rs`](../../crates/robotics_filters/src/bayes.rs) | implemented |
| EKF, generic over the manifold | [`ekf.rs`](../../crates/robotics_filters/src/ekf.rs) | implemented |
| Error-state EKF on SE(2), absolute position and heading updates | [`eskf_se2.rs`](../../crates/robotics_filters/src/eskf_se2.rs) | implemented |
| Particle set, log weights, low variance and multinomial resampling, ESS | [`particle.rs`](../../crates/robotics_filters/src/particle.rs) | implemented |
| KLD adaptive sample size, `Se2Binning` | [`kld.rs`](../../crates/robotics_filters/src/kld.rs) | implemented |
| NEES and NIS with chi-square bounds | [`consistency.rs`](../../crates/robotics_filters/src/consistency.rs) | implemented |
| Odometry and velocity motion models, sampler and density | [`motion/`](../../crates/robotics_models/src/motion/) | implemented |
| Beam model, likelihood field over an exact EDT, range and bearing landmarks, tempering | [`sensor/`](../../crates/robotics_models/src/sensor/) | implemented |
| SE(2) manifold, TF buffer with authority conflict detection | [`se2.rs`](../../crates/robotics_core/src/se2.rs), [`tf.rs`](../../crates/robotics_core/src/tf.rs) | implemented |
| Message conversion, QoS presets, TF broadcaster, diagnostics, shutdown | [`robotics_ros_bridge`](../../ros2_ws/src/robotics_ros_bridge/) | implemented |

## 2. What Phase 3 adds

**TODO**, planned shape only, to be replaced by a walkthrough of the merged code:

* `crates/robotics_localization`, a ROS-free crate: EKF localization with maximum
  likelihood association and a chi-square gate (**built**), MCL and augmented MCL with KLD
  sizing and dominant-cluster pose extraction, grid localization kept as a debugging mode
  (**not built**).
* The thin rclrs node that wires it to the topics in [ros-graph.md](ros-graph.md)
  (**built**, as
  [`ros2_ws/src/robotics_localization_bridge/src/localization_node.rs`](../../ros2_ws/src/robotics_localization_bridge/src/localization_node.rs),
  not as the `robotics_localization` package this section originally planned; the crate name
  is taken by the ROS-free crate above).
* A map source. The four pillars of
  [the world](../../sim/webots/worlds/turtlebot3_localization_room.wbt) are available as
  landmarks today ([landmarks.yaml](../../sim/webots/config/landmarks.yaml)); an occupancy
  grid for the beam and likelihood field models arrives with Phase 4, so until then the
  grid-based path is tested against a hand-built map of the same room.
* Launch and parameter files in `robotics_bringup`, plus the Webots bringup that runs the
  simulation and the node together.
* Integration test in the style of
  [`phase2_stack_test`](../../ros2_ws/src/robotics_integration/src/phase2_stack_test.rs):
  topics, frames, rates, TF ownership and clean shutdown asserted from a Rust binary.

## 3. Open questions to settle in code

**TODO**, each of these is a decision the implementation has to make and defend with a
measurement, not with an opinion:

1. **Settled for the EKF path: a landmark extractor.** Does the lidar feed a landmark
   extractor (cluster the scan, fit pillar centres) or go straight into a likelihood field
   over an occupancy grid? The first matches the sparse room and the existing landmark
   model; the second generalizes to Phase 4 and beyond. The built EKF takes range and
   bearing observations from
   [`pillars.rs`](../../crates/robotics_localization/src/pillars.rs); the likelihood field
   path stays open for the MCL half, which needs the Phase 4 occupancy grid anyway.
2. How many beams of the 360 are used per update, and with which tempering exponent, to
   keep the effective sample size healthy.
3. **Settled: a separate estimator, and it publishes no TF at all.** Whether the ESKF of
   Phase 2 absorbs the absolute correction directly (`update_position` and `update_heading`
   already exist) or the localization filter stays a separate estimator that only publishes
   `map -> odom`. `localization_node` is separate and publishes
   `geometry_msgs/PoseWithCovarianceStamped` in the `map` frame; it deliberately does not
   publish `map -> odom`, because nothing in Phase 3 consumes that edge yet and the node
   header argues an unverified TF edge is worse than none. That leaves the edge for the
   stack that first needs it, and [ros-graph.md](ros-graph.md) still describes it as owned
   here.
4. What the node does when the chi-square gate rejects everything for several seconds:
   report degraded on `/diagnostics`, trigger injection, or both.
5. Particle count floor and ceiling, and what gets published on `/particle_cloud` at what
   rate, so visualization does not dominate the measured cost.

## 4. How this part will be written

Same pattern as the rest of the material: every claim points at a file and a line that can
be read, every number comes from a run recorded in [benchmark.md](benchmark.md), and
anything that does not work yet says so in the text instead of being quietly omitted.
