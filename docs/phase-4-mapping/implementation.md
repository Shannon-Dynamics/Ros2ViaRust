# Phase 4, implementation: the Rust side

> **Status: plan only, nothing written in code.** No crate, package, script or launch file
> named here exists yet. Where this file says "exists", it points at something that is
> already in the repository and has been looked at; everything else is a plan, and where a
> fact still has to be checked against the simulator or the code it says so. The decisions
> recorded in section 5 were confirmed by the project owner on 2026-10-10.

## 1. Already in the repository

| What | Where | Used for |
|---|---|---|
| exact Euclidean distance transform over an occupancy mask, `DistanceGrid::from_occupancy` | [`robotics_models::sensor::likelihood_field`](../../crates/robotics_models/src/sensor/likelihood_field.rs) | the unsigned half of the ESDF; not to be duplicated |
| segment `World` with exact `raycast` and a seeded 2-D lidar | [`robotics_sim`](../../crates/robotics_sim/src/) | unit-test worlds for the grid; the exact visibility test that decides which ground-truth surfaces were observable |
| planar TF buffer and the Rust `TfListener` | `robotics_core::tf`, [`robotics_ros_bridge`](../../ros2_ws/src/robotics_ros_bridge/) | `map -> base_link` lookup at the scan stamp |
| QoS preset for a latched map, converters, diagnostics | `robotics_ros_bridge::{qos, convert, diagnostics}` | `/map` publication, node health |
| `/ground_truth/pose` at 50 Hz and the world's geometry | [`evaluation_supervisor.py`](../../sim/webots/scripts/evaluation_supervisor.py), the `.wbt` and `landmarks.yaml` | the input of the ground-truth TF adapter, the trajectory of the evaluator, and the ground-truth surfaces |
| `route_driver`, closed loop on ground truth, and `route_long.yaml` | `robotics_localization_bridge`, [`sim/webots/config/`](../../sim/webots/config/) | driving the robot through the room |
| evidence layout, `EVIDENCE_ROOT`, verdict-as-a-file convention | `scripts/evidence_root.sh`, `scripts/phase3_verdict.py` | the same discipline for Phase 4 |

## 2. What Phase 4 adds

Ordered so that each stage can be tested before the next exists. The last column says
whether the stage can be done before Phase 3 closes.

| Id | Task | Where | Before Phase 3 closes? |
|---|---|---|---|
| T1 | `OccupancyGrid`: dimensions, origin, resolution, log-odds per cell, clamping, world-to-cell and cell-to-world, three-state read-out (free, occupied, unknown) against thresholds | `crates/robotics_mapping` | yes |
| T2 | hand-crafted inverse sensor model and ray integration (Bresenham, then a supercover variant so that no cell on a diagonal is skipped), with a documented choice for no-return beams | `crates/robotics_mapping` | yes |
| T3 | entropy of the grid, total and over decided cells, with the property that an unobserved cell contributes exactly one bit | `crates/robotics_mapping` | yes |
| T4 | the map-accuracy evaluator: ground-truth surface set from the world, observable subset by exact ray cast, and the metrics of [benchmark.md](benchmark.md) section 2 | `crates/robotics_mapping` (module `eval`), plus an example binary that reads a grid and a world and writes a metrics file | yes |
| T5 | signed ESDF: outside distance from the existing exact EDT, inside distance from the same transform on the complement, so a planner can tell how deep it is inside an obstacle | `crates/robotics_mapping` | yes |
| T6 | `ground_truth_tf_adapter` (section 3) | `ros2_ws/src/robotics_mapping_ros` | yes, simulation only |
| T7 | `mapping_node` (TF in, `/map` out, no pose-source parameter) and `mapping_evaluator` | `ros2_ws/src/robotics_mapping_ros` | yes |
| T8 | harness: `scripts/run_phase4_mapping.sh`, `scripts/phase4_verdict.py` with every tolerance fixed in the file before the first evidence run, `robotics_bringup/launch/phase4_mapping.launch.py`, `scripts/verify_phase4_mapping.sh` | `scripts/`, `ros2_ws/` | the scripts yes; **running them waits** for Webots to be free (section 6) |
| T9 | informative references (non-blocking): an rclpy port of the same update, and `nav2_costmap_2d` if it can be run headless ([benchmark.md](benchmark.md) section 3) | `scripts/`, `ros2_ws/` | yes; nothing waits on them |
| T10 | **estimated-pose integration (M5):** the `map -> odom` owner for each estimator, with MCL's missing TF resolved one of the two ways of [problem.md](problem.md) section 6, and the M5 runs | `ros2_ws/`, possibly `mcl_node` | **no, waits for Phase 3** |

2-D TSDF and quadtree (Ch. 19) are **not** in this table: they were removed from Phase 4
on 2026-10-10 and are a Phase 5+ concern (ROADMAP, Phase 5). Nothing in Phase 4 consumes
them, and the signed ESDF of T5 does not need them.

**Data structure.** An occupancy grid of log-odds, one `f32` per cell in a flat row-major
`Vec`, origin and resolution as fields, and an `Option`-returning accessor so that a cell
outside the grid is not confused with an unknown cell. A quadtree or a hash of chunks pays
off when the map is large and mostly empty; this room is 6 by 6 m, which at 5 cm per cell
is a small dense array, and Phase 8 will explore larger spaces by growing a grid, not by
changing its representation. Log-odds rather than probabilities because the update is then
an addition and cannot overflow into 0 or 1.

**The inverse sensor model.** Hand-crafted: a hit adds `l_occ` to the cell containing the
return, every cell the beam crosses before it adds `l_free`, clamped to a band so a cell can
change its mind. The numbers are parameters, set once on the development run and then
frozen; they are not tuned on the evaluation runs. Resolution is a parameter too: 5 cm is
the proposal, because it is the usual size for this robot and equal to the wall thickness.
Neither number is chosen by this document.

## 3. Planned interface

Planned, to be confirmed against the implementation.

### 3.1 `mapping_node`

| Direction | Topic | Type | Note |
|---|---|---|---|
| in | `scan` | `sensor_msgs/LaserScan` | sensor QoS; frame `LDS-01`, so the transform to `base_link` is read from TF and the name is not assumed |
| in | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `map -> base_link`, looked up at the scan stamp, and `base_link -> LDS-01`. **The only source of the pose** |
| out | `map` | `nav_msgs/OccupancyGrid` | latched; the three states as 0, 100 and -1 |
| out | `map_entropy` | `std_msgs/Float64` | throttled |
| out | `esdf` | `nav_msgs/OccupancyGrid` or a custom message | the encoding is undecided, see `ROS2_INTERFACES.md` |
| out | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | scans integrated, scans skipped and why (no transform at that stamp, stale), update time |
| service | `~/reset` | `std_srvs/Empty` | rclrs 0.8 has no lifecycle nodes, the cross-cutting backlog's workaround |

`mapping_node` publishes no TF, owns no TF edge, and has **no pose-source parameter** and no
subscription to `/ground_truth/pose`. A scan with no transform at its stamp is skipped and
counted, never integrated at a guessed pose.

### 3.2 `ground_truth_tf_adapter` (simulation only)

| Direction | Topic | Type | Note |
|---|---|---|---|
| in | `ground_truth/pose` | `geometry_msgs/PoseStamped` | frame `map`, 50 Hz from the evaluation supervisor |
| in | `/tf` | `tf2_msgs/TFMessage` | `odom -> base_link` from `diff_drive_controller`, looked up at the ground-truth stamp |
| out | `/tf` | `tf2_msgs/TFMessage` | **`map -> odom` only**, `T_map_base * inverse(T_odom_base)` |
| out | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | that it is a ground-truth source, samples published, samples dropped for want of `odom -> base_link` |

It must be the only owner of `map -> odom` in its run, which the TF authority conflict
detection of `TfListener` is there to catch if `localization_node` is started by mistake.
The name says what it is, deliberately: a launch file that starts it for a real robot is a
bug.

### 3.3 `mapping_evaluator`

Reads the grid from `/map`, the trajectory from `/ground_truth/pose`, and the world
geometry from the same files the world is built from, and writes the metrics of
[benchmark.md](benchmark.md) section 2 to the run directory. The mapper never sees the world
file. Like `localization_evaluator` in Phase 3, it is allowed to read ground truth.

## 4. What waits for Phase 3 and what does not

| Part of Phase 4 | Needs Phase 3 closed? | Why |
|---|---|---|
| T1 to T5: grid, inverse model, entropy, evaluator, ESDF | no | pure library, tested on `robotics_sim` worlds and on the Phase 3 room's geometry |
| T6 to T9: adapter, node, evaluator, harness, informative references | no, except for running Webots | the pose comes from the adapter, which names itself as ground truth |
| M1, M2, M3, M4 evidence | no, except for running Webots | none of these claims uses an estimated pose |
| T10, M5: the real estimator's `map -> odom`, and the choice of estimator | **yes** | `localization_node` (EKF) owns `map -> odom` today; `mcl_node` publishes `/mcl/pose` and no TF, so for MCL either `mcl_node` gains a TF publisher (a Phase 3 code change that also changes check F11) or an MCL-to-TF adapter is added. Undecided, and a Phase 3 outcome, not something to guess |
| the P1 verdict as the bound on pose error behind M5 | yes | M5 is reported next to it and means nothing without it |
| Phase 3 localizing against `/map` | not part of this plan | not required to close either phase |

Nothing in T1 to T9 reads an estimator's output while Phase 3 is open, so no hidden
dependency on Phase 3 results exists in the first stages: the one input they take from it is
the Webots world, unchanged.

## 5. Decisions, and what is still open

Confirmed by the project owner on 2026-10-10:

| Id | Decision |
|---|---|
| D1 | The claims M1 to M5 of [problem.md](problem.md) section 4 as written, including three evaluation runs for M1. |
| D2 | The pose reaches the mapper through TF only. A separate `ground_truth_tf_adapter` provides `map -> odom` in simulation; `mapping_node` has no pose-source flag. (Implementation note: the adapter publishes `map -> odom` and not `map -> base_link`, because `base_link` already has the parent `odom`.) |
| D3 | Forward dependency for M5: `mcl_node` publishes no TF, so M5 needs a TF publisher in `mcl_node` or an adapter for MCL. Recorded in [problem.md](problem.md) section 6, undecided. |
| D4 | 2-D TSDF and quadtree (Ch. 19) are removed from Phase 4 and are a Phase 5+ concern. |
| D5 | Names: crate `robotics_mapping`; ROS package `robotics_mapping_ros` (not `robotics_mapping`, which would collide with the crate in the Cargo workspace), holding `ground_truth_tf_adapter`, `mapping_node` and `mapping_evaluator`. `ARCHITECTURE.md` and `SOURCE_MAPPING.md` are updated to match. |
| D6 | A second, more complex world is the backlog item *M1-extended*, after the Phase 3 trials. |
| D7 | The analytic geometry of the world is the main baseline. The rclpy port and `nav2_costmap_2d` are additional information and do not block. |

Still open, to be settled before they are coded:

1. **MCL's route to `map -> odom`** (D3): which of the two.
2. **No-return beams.** Whether a beam without a return clears free space up to the range
   limit, or is dropped. Both are used in practice and differ near the sensor limit.
3. **Scan skew.** At 5 Hz a sweep takes 0.2 s. Whether Webots stamps and simulates a scan as
   one instant or as a rotation during motion has not been checked; if it is a rotation, a
   robot at driving speed moves a large part of a cell during one sweep. To be measured on a
   recorded scan, not assumed either way.
4. **Lidar height against the world.** The walls are 0.5 m high and the pillars 0.6 m; that
   the lidar plane is below both is expected from the model and has not been verified here.
5. **The ESDF message encoding.**

## 6. Evidence, and how this part will be run

* Output goes to `evidence/phase-4-mapping/<run>/`, with `summary.yaml` naming the owner of
  `map -> odom`, and a verdict file written by `scripts/phase4_verdict.py` and never typed.
  Tolerances live in that script and are committed before the first evidence run, so no
  metric or threshold is chosen after seeing a result.
* **A run is copied to the Windows checkout the moment it finishes**, before anything else,
  because runs execute in the WSL mirror and `scripts/sync_to_wsl.sh` uses `rsync
  --delete`. This has destroyed evidence before ([../phase-3-localization/implementation.md](../phase-3-localization/implementation.md)
  section 5.3), and the first Phase 4 run is exactly as exposed.
* No Phase 4 run uses Webots, and nothing in Phase 4 that needs the simulator is run, until
  the Phase 3 trials that are using it have finished: they share the simulator, its port and
  the CPU, and the benchmark rules ask for nothing else running.
* Library tests use `robotics_sim` and the closed-form geometry of the room, not Webots, so
  T1 to T5 never wait for the simulator.

## 7. What this plan deliberately does not do

It does not edit `robotics_core`, `robotics_filters`, `robotics_models` or `robotics_sim`:
the ESDF reuses `DistanceGrid` as it is. It does not add a workspace member, a package or
a launch file, since a new member changes `Cargo.lock` and a build, and this plan was
written while Phase 3 trials were running from the same workspace. It does not extend
Phase 3: the remaining Phase 3 items stay in [../ROADMAP.md](../ROADMAP.md) as they were,
and the `mcl_node` TF question of D3 is recorded and not acted on.
