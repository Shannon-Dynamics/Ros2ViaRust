# Phase 4, implementation: the Rust side

> **Status: decisions taken, code in progress.** Phase 3 closed on 2026-10-10 and the
> open questions that could be settled without writing the mapper were settled the same day
> (section 5.2): three from data recorded in the simulator, two by design. What is built is
> listed in section 8 as it lands; anything this file calls a plan and section 8 does not list
> does not exist yet. The decisions of section 5.1 were confirmed by the project owner on
> 2026-10-10.

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

### 5.1 Confirmed by the project owner on 2026-10-10

| Id | Decision |
|---|---|
| D1 | The claims M1 to M5 of [problem.md](problem.md) section 4 as written, including three evaluation runs for M1. |
| D2 | The pose reaches the mapper through TF only. A separate `ground_truth_tf_adapter` provides `map -> odom` in simulation; `mapping_node` has no pose-source flag. (Implementation note: the adapter publishes `map -> odom` and not `map -> base_link`, because `base_link` already has the parent `odom`.) |
| D3 | Forward dependency for M5: `mcl_node` publishes no TF, so M5 needs a TF publisher in `mcl_node` or an adapter for MCL. Recorded in [problem.md](problem.md) section 6, undecided. |
| D4 | 2-D TSDF and quadtree (Ch. 19) are removed from Phase 4 and are a Phase 5+ concern. |
| D5 | Names: crate `robotics_mapping`; ROS package `robotics_mapping_ros` (not `robotics_mapping`, which would collide with the crate in the Cargo workspace), holding `ground_truth_tf_adapter`, `mapping_node` and `mapping_evaluator`. `ARCHITECTURE.md` and `SOURCE_MAPPING.md` are updated to match. |
| D6 | A second, more complex world is the backlog item *M1-extended*, after the Phase 3 trials. |
| D7 | The analytic geometry of the world is the main baseline. The rclpy port and `nav2_costmap_2d` are additional information and do not block. |

### 5.2 Questions settled on 2026-10-11, before any mapper code

Decided on the project owner's instruction to decide and document them, so open to the
owner's revision (unlike 5.1, they were not separately confirmed). The scan recording they
rely on was made on 2026-10-10.

| Id | Decision |
|---|---|
| D8 | The lidar plane (0.173 m) is below the walls (0.5 m) and the pillars (0.6 m): item 1. |
| D9 | A Webots scan is one instant at its header stamp; the mapper does not deskew: item 2. |
| D10 | A beam with no return clears free space to `range_max` by default, `drop` selectable: item 3. |
| D11 | A cell is updated at most once per scan, occupied over free: item 4. |
| D12 | The ESDF is a `sensor_msgs/Image` `32FC1` paired by stamp with `/map`: item 5. |


The first four were open at the end of 5.1. Items 1 to 3 are answers from the simulator's
own files and from a recorded run; the numbers are in
`evidence/phase-4-mapping/2026-10-10-scan-skew-probe/skew-report.yaml`, written by
[`scripts/phase4_scan_skew_analysis.py`](../../scripts/phase4_scan_skew_analysis.py), and are
not repeated here.

1. **Lidar height against the world (D8): the scan plane cuts every obstacle.** Read
   statically from the world and the two protos Webots caches
   (`TurtleBot3Burger.proto`, `RobotisLds01.proto`), not from a run: the lidar sits at
   `0.153 + 0.02 = 0.173` m above the floor, with one layer and no tilt, and the walls
   (`RectangleArena`, `wallHeight 0.5`, boxes centred at half height) and pillars
   (`Cylinder height 0.6`, centred at 0.3 m) both span it. Nothing in the room can pass over
   or under the scan plane, so a 2-D grid is an honest representation of this world. That is
   a property of this world and says nothing about furniture of other heights.
2. **Scan skew (D9): one scan is one instant, to within about one beam.** Recorded with the
   robot still, driving straight at the route driver's 0.18 m/s, and turning in place at its
   0.8 rad/s plus a 2.0 rad/s stress turn, against ground truth and the analytic room. Two
   models of when the 360 beams were measured were fitted per scan (one instant; or beams
   spread over a span `s`), and the method was first checked on synthetic scans with a
   known span (0 s and 0.2 s, both recovered). Findings: the published
   `time_increment` and `scan_time` are both `0`, so the driver asserts nothing about
   sweep timing; the seam between the last and first beam, which point the same way, shows no
   step at driving speed to within its standard error, where a 0.2 s sweep would show one
   of about 36 mm; and in the turns the fitted span equals about one beam of heading, not
   the nine and twenty-three beams a 0.2 s sweep would give, and it shrinks as the turn gets
   faster, so it is not a sweep time. **Decision:** the mapper treats every scan as one
   instant at its header stamp, looks the pose up at that stamp, and does no deskewing. The
   skew that remains at route speed is far below one 5 cm cell. **Scope:** this is a
   statement about the Webots LDS-01 model. The real LDS-01 rotates during the 0.2 s and a
   robot at speed would smear a scan; nothing here covers that, and a real-robot mapper
   would need the deskew this one omits. Not all phases of the recording could be used: the
   three stopping phases are not evidence (two have no valid seam beams, one has a residual
   several times the noise floor, so the robot had not settled), and the straight-line fit of
   the span is poorly conditioned, which is why the seam test, not that fit, carries the
   translation result.
3. **Beams without a return (D10): clear free space up to the range limit.** In this
   simulator a beam with nothing in range is published as `inf`; never as NaN, never as
   `range_max`. Against the analytic room, the beams that came back `inf` are exactly the
   beams whose true range exceeds 3.5 m (all but two of the standing-still ones, and those two
   at 3.483 m, inside the range noise), and no beam that came back finite has a true range
   above 3.5 m. So here `inf` is a real statement, "nothing within `range_max`", and
   discarding those beams would throw away about 30 per cent of every scan (the corners of
   the room are 4.24 m away) and leave the far part of the room unexplored for no reason.
   **Decision:** an `inf` beam, and a finite one at or beyond `range_max`, clears the cells
   along the beam up to `range_max` and marks nothing occupied; NaN and anything below
   `range_min` are dropped. A parameter selects `drop` instead, for a sensor whose no-return
   can also mean a surface that absorbed the beam; the default is `clear_to_max_range`, and the
   evidence above is why. Consequence of the published angle grid (increment `-2 pi / 359`,
   so beams 0 and 359 point the same way): it is correct, it was checked against the
   alternative `2 pi / 360` and fits better, and it means one direction is sampled twice, which
   the update below makes harmless.
4. **Update rule (D11): each cell is updated at most once per scan.** The grid collects the
   cells a scan found free and the cells it found occupied, and applies `l_free` to the
   free ones not also occupied and `l_occ` to the occupied ones, once each. This is
   Table 9.1 of Probabilistic Robotics (one update per cell in the perceptual field of the
   scan, not one per beam), it stops the cells near the sensor, which hundreds of beams cross,
   from saturating on a single scan, and it makes the result independent of beam order exactly
   rather than approximately. A cell that one beam passes through and another ends in is
   occupied: the hit is the stronger statement.
5. **ESDF message (D12): `sensor_msgs/Image`, encoding `32FC1`, paired with `/map`.** One
   `float32` per cell, in metres, row-major with row 0 at the grid's `j = 0` (the
   `nav_msgs/OccupancyGrid` order, which is the opposite of an image's top-down order and is
   stated here so that nobody flips it), positive in free space, negative inside an obstacle,
   zero level at the cell faces (the signed convention of section 8.2), saturating at the
   truncation distance, and `inf` where no obstacle exists. `header.frame_id` is the map's
   and `header.stamp` is the stamp of the `/map` message it was computed from; its grid
   geometry (width, height, resolution, origin) is **by contract that of `/map`**, so a
   consumer checks the two stamps and sizes and reads the geometry from `/map`. Rejected:
   `nav_msgs/OccupancyGrid` with the distance squeezed into `int8` (the type's semantics are 0
   to 100 and -1, every viewer and every consumer that subscribes by type would read the
   distances as probabilities, and `-1` is a legal signed distance); a `PointCloud2` (sparse
   or organized, lossless but with the geometry implicit in point spacing); and a custom
   message, which is what the established mappers ship (`nvblox_msgs/DistanceMapSlice`,
   `grid_map_msgs/GridMap`) and is the right end state, but would add an interface package
   and its Rust bindings to the underlay for a topic that nothing consumes before Phase 6.
   **Revisit** when Phase 6 chooses how a planner reads it; the stamp-pairing contract is the
   price of using a standard type until then.
6. **MCL's route to `map -> odom` (D3): still undecided.** It does not block T1 to T9, and
   nothing in this part reads an estimator's pose. It must be decided before T10 and the M5
   runs, and the two options are the ones of [problem.md](problem.md) section 6. Recorded here
   so that it is not forgotten: **T10 and M5 do not start until D3 is decided.**

### 5.3 Still open

* D3 above, and with it T10 and M5.
* The occupancy thresholds, the clamps, `l_occ`, `l_free` and the resolution are *defaults*
  in the library, chosen by reasoning in section 8.1 and fixed on the development run
  (section 6); they are not tuned on an evaluation run.

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
