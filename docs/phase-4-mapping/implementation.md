# Phase 4, implementation: the Rust side

> **Status (2026-10-11): T1 to T8 built, one development phase run, no evaluation runs.** The
> library, the three nodes, the launch file, the run script, the verdict script and the
> integration test exist and are listed in section 8. Three development runs on `route_long.yaml`
> have been made (`evidence/phase-4-mapping/2026-10-11-development-*`); the last has the mapper's
> parameters frozen and **misses the pre-registered M1 tolerances on one check of seven** (false
> occupied cells 2.6 per cent against 2). The three evaluation runs of claim M1 have not been made,
> because their routes have not been written; M5 waits for decision D3. Anything section 8 does not
> list does not exist. The decisions of section 5.1 were confirmed by the project owner on
> 2026-10-10; those of 5.2 were decided on the owner's instruction to decide and document them.

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

The "before Phase 3 closes" column is historical: Phase 3 closed on 2026-10-10 and what is built is in section 8.

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

### 5.2 Questions settled on 2026-10-11: items 1 to 6 before any mapper code, items 7 to 10 after the first runs

Decided on the project owner's instruction to decide and document them, so open to the
owner's revision (unlike 5.1, they were not separately confirmed). The scan recording they
rely on was made on 2026-10-10.

| Id | Decision |
|---|---|
| D8 | The lidar plane (0.173 m) is below the walls (0.5 m) and the pillars (0.6 m): item 1. |
| D9 | A Webots scan is one instant, not spread over its period, so the mapper does not deskew; but that instant is **not** its header stamp: item 2. |
| D10 | A beam with no return clears free space to `range_max` by default, `drop` selectable: item 3. |
| D11 | A cell is updated at most once per scan, occupied over free: item 4. |
| D12 | The ESDF is a `sensor_msgs/Image` `32FC1` paired by stamp with `/map`: item 5. |
| D13 | Because the stamp lags the measurement by up to a scan period, the mapper skips a scan when the sensor moved more than 5 cm or turned more than 0.015 rad across the window `[stamp - 0.2 s, stamp]`: item 7. |
| D14 | A no-return beam clears free space up to `range_max - 0.10 m`, not to `range_max`: item 8. |
| D15 | Scans are integrated at the pose `0.06 s` before their stamp (`scan_time_offset_s`, the Webots lidar's mean lag), not at the stamp: item 10. |
| D16 | A cell is occupied after four hits, not one (`p_occupied` 0.95, was 0.65): item 10. |
| D17 | A scan taken from where the last integrated one was taken is skipped (`motion_filter`, 2 cm or 0.02 rad): item 10. |


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
2. **Scan timing (D9, corrected by D13): a scan is one instant, but not the instant of its
   stamp.** Recorded with the robot still, driving straight at the route driver's 0.18 m/s,
   and turning in place at its 0.8 rad/s plus a 2.0 rad/s stress turn, against ground truth and
   the analytic room. Two things were measured, and the first report on this recording stated
   only the first.

   *Is a sweep spread over time? No.* Two models of when the 360 beams were measured were
   fitted per scan (one instant; or beams spread over a span `s`), and the method was first
   checked on synthetic scans with a known span (0 s and 0.2 s, both recovered). The published
   `time_increment` and `scan_time` are both `0`, so the driver asserts nothing; the seam between
   the last and first beam, which point the same way, shows no step at driving speed to within
   its standard error, where a 0.2 s sweep would show one of about 36 mm; and in the turns the
   fitted span equals about one beam of heading, not the nine and twenty-three beams a 0.2 s
   sweep would give, and it shrinks as the turn gets faster, so it is not a sweep time. So each
   scan is one instant and needs no deskewing.

   *Is that instant the header stamp? No.* The same fits carry a constant offset `d` between the
   scan stamp and the ground-truth clock, which the first report left out. In the turning
   phases, where a rotation pins it to a few milliseconds, the instants the beams were measured
   at lie on a regular **0.2 s lattice** (resultant length of their phase 0.995, phase standard
   deviation 3 ms; no other trial period from 0.19 to 0.22 s comes close) while the stamps do not
   (0.17). The stamp **lags the measurement by up to about 0.18 s, 0.06 s on average**, in a
   sawtooth: the driver publishes every 0.2 or 0.22 s on its own clock, the lidar measures every
   0.2 s on its, and the lag grows by about 0.02 s per scan until a measurement is skipped.
   The numbers are in `skew-report.yaml` (`stamp_lag_turning_phases`).

   Scope: both statements are about the Webots LDS-01 model and its driver. The real LDS-01
   rotates during the 0.2 s and a robot at speed would smear a scan; a real-robot mapper would
   need the deskew this one omits, and would not have this stamp lag if its driver stamped at
   measurement. Not all phases of the recording could be used: the three stopping phases are not
   evidence (two have no valid seam beams, one has a residual several times the noise floor, so
   the robot had not settled), and the straight-line fit of the span is poorly conditioned, which
   is why the seam test, not that fit, carries the translation result.

   **How this was found.** Not from the recording but from the first mapping run
   (`evidence/phase-4-mapping/2026-10-11-harness-smoke`), which mapped a 0.5 m triangle and
   failed M1 badly: cells called occupied up to 27 cm from any surface. That run integrated every
   scan at the pose TF gives for its stamp, which was the decision this item first recorded, and
   the offset `d` was already in the recording's report. It is a correction of an earlier
   statement of mine, and that run is kept to show it.
3. **Beams without a return (D10): clear free space up to the range limit.** In this
   simulator a beam with nothing in range is published as `inf`; never as NaN, never as
   `range_max`. Against the analytic room, the beams that came back `inf` are exactly the
   beams whose true range exceeds 3.5 m (all but two of the standing-still ones, and those two
   at 3.483 m, inside the range noise), and no beam that came back finite has a true range
   above 3.5 m. So here `inf` is a real statement, "nothing within `range_max`", and
   discarding those beams would throw away about 30 per cent of every scan (the corners of
   the room are 4.24 m away) and leave the far part of the room unexplored for no reason.
   **Decision:** an `inf` beam, and a finite one at or beyond `range_max`, clears the cells
   along the beam up to `range_max` less a margin (item 8) and marks nothing occupied; NaN and
   anything below `range_min` are dropped. A parameter selects `drop` instead, for a sensor whose no-return
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
   zero level at the cell faces (the signed convention of section 8.2), and saturating at the
   truncation distance (which is also what a map with no obstacle at all reports). `header.frame_id` is the map's
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

7. **The motion gate (D13).** The lag cannot be removed by the mapper: the measurement instant
   is only known to lie in `[stamp - 0.2 s, stamp]`, and a constant correction would leave the
   sawtooth's other 0.1 s. What can be bounded is the damage. If the sensor's pose barely
   changes across that window, the exact instant does not matter; if it changes a lot, the scan
   would be written into the map at a pose that may be wrong by the whole change. So the
   mapper looks the pose up at the stamp and at the start of the window and **skips the scan**
   when the sensor moved more than `motion_gate.max_translation_m` or turned more than
   `motion_gate.max_yaw_rad`. The error of an integrated scan is then at most that change.
   Defaults, from the geometry and not from a run: 0.05 m (the robot's 0.2 m/s limit moves 4 cm
   in 0.2 s, so straight driving always passes, with a worst-case position error of 4 cm and a
   typical one of a third of that, since the mean lag is 0.06 s) and 0.015 rad (5 cm at the
   lidar's 3.5 m limit, one cell: a turn faster than 0.075 rad/s is skipped, the route's
   0.8 rad/s turns and its heading corrections with it). Scans skipped for motion are counted
   and written to the latency file with the change that caused it; a run that skips more than
   half its scans is not valid (`scripts/phase4_verdict.py`). The cost is real: no scan is
   integrated while the robot turns. The alternatives were rejected: a constant offset (leaves
   up to 0.1 s), a lattice estimator that restamps scans from their stamps alone (the phase is
   not identifiable from the stamps without ground truth or an assumed minimum lag), and fixing
   the driver (it is used as published).

   Whether 4 cm of residual pose error during travel is small enough is what the development
   run measures; the tolerances of the verdict were committed before any run and were not
   changed for it.

8. **The clear margin (D14).** Item 3 as first written cleared right up to `range_max`, and
   the second Webots run showed what that does: wall cells at the edge of the lidar's reach
   were called free (`evidence/phase-4-mapping/2026-10-11-smoke-with-motion-gate`, 22 per cent
   of the observable surface samples in free cells, against 4 per cent of what the exact grid
   holds). The mechanism, found by tracing which beams crossed one such cell: a beam whose wall
   is a few centimetres beyond 3.5 m returns `inf`, and drawn from a pose that is a few
   centimetres off (the motion gate admits up to 5 cm and 0.015 rad, 5 cm at the limit) its
   free segment crosses the wall. The sensor is not at fault: in the standing-still recording
   `inf` appears only above 3.5 m, to within the range noise. Clearing stops **0.10 m short of
   `range_max`** (`model.clear_margin_m`, `DEFAULT_CLEAR_MARGIN`): the 3-sigma range noise at the
   limit (3 x 0.0043 x 3.5 m = 4.5 cm) plus the pose error the gate admits, rounded to two
   cells. The price is a 10 cm band at the limit that stays unknown instead of free.
9. **Two faults in the harness found by the same runs, fixed and kept in the record.** The
   `record` launch argument collided with the world launch file's own (launch configurations
   are global), which stopped the first attempt before it started. And the evaluator read the
   map's resolution from a `float32`, so 0.05 arrived as 0.0500000007 and a wall on a lattice
   line sat 2e-6 cells off it at the far edge, which made the closed-cell rule of `eval.rs`
   miss it: false-free samples were over-counted by an order of magnitude (it was 22 per cent
   where the exact grid says 4). The evaluator now restores the resolution to seven decimals,
   and the rule's tolerance is a ten-thousandth of a cell.

10. **Three changes made after the first development run, and why each is not a tuning of the
    result (D15 to D17).** The run `2026-10-11-development-route-long` failed M1 on five of seven
    checks (false occupied 13 per cent, 95th percentile of the surface error 1.5 cells, maximum
    9.5 cells, false free 6.6 per cent). Its recording was analysed before anything was changed,
    and replaying it offline with other read-out parameters reproduced its metrics exactly
    (904 occupied cells, 119 false), which makes the offline numbers below comparable to it. What
    the analysis found: of 904 occupied cells, 117 lie farther than a cell from any surface and
    115 of those are in the row of cells just outside a wall, mostly with a **single** hit. The
    hits that land there are 636 of 332 635 (0.19 per cent), taken while driving straight at
    0.18 m/s with a yaw rate near zero, on every wall, from every part of the route, in scans
    whose stamp lag is spread uniformly over its range. That is the translation timing error
    along the heading, up to 0.18 s x 0.18 m/s = 3.2 cm, on top of the range noise, and nothing
    ever crosses a cell outside a wall to clear it. Three changes, each aimed at one mechanism:

    * **D15, the mean stamp lag.** The lag was measured on the scan-skew recording
      (0.061 s on average), not on a mapping run, and it is a property of the driver. The mapper
      takes `scan_time_offset_s` (default 0: nothing is assumed of a driver it was not told
      about) and the Phase 4 launch sets it to -0.06. It halves the worst-case translation
      error (the lag then lies in [-0.08, 0.12] s) and leaves the motion gate as the bound for
      what is left.
    * **D16, more hits to occupy (first written as two, now four).** The occupied threshold
      first went from probability 0.65 to 0.75 (two hits). Replaying the first run's recording
      offline with only that change, the false-occupied cells fell from 119 to 59 and the
      surface error did not move. The second development run was meant to test it and did not:
      the default was not changed in the node (a text replacement that silently matched nothing),
      so that run had one-hit occupancy, which its recording then let me sweep offline (table in
      its `NOTE-what-this-run-found.md`): 121 false cells at one hit, 67 at two, 35 at three,
      20 at four (0.95), 15 at four-plus (0.96, where completeness falls below 0.95). The third run
      uses 0.95, the largest threshold of that grid that keeps completeness at the verdict's
      0.95. That is a choice made on a development recording after seeing it, and it leaves false
      occupied (2.8 per cent in the replay) above the verdict's 2 per cent: what is left is a row
      of cells outside the wall, at saturation, behind and beside the pillars, from grazing returns.
    * **D17, no repeats from a standing pose.** The log-odds update treats every scan as
      independent, and scans from one pose are not (the first smoke runs saturated a lidar
      "flying pixel", a depth-edge artefact of the rendered lidar, at a pillar's silhouette that
      way). `motion_filter` integrates a scan only if the sensor moved 2 cm or turned 0.02 rad
      since the last integrated scan; a robot at 0.18 m/s moves 3.6 cm per scan and is never
      affected. Scans it skips are counted and written out.

    These are decisions of a development run, which is what that run is for ("fixed on the
    development run, not tuned on the evaluation runs", section 2); the tolerances of the verdict
    were committed before any run and are not touched. They are made on one run, so whether they
    generalize is what the evaluation runs are for. If the second development run still fails,
    that failure is the result.

### 5.3 Still open

* **Flying pixels.** The Webots lidar returns ranges between a foreground and a background
  surface at depth edges, such as the silhouette of a pillar (the 2.75 m return between a pillar
  at 2.31 m and the wall behind it at 3.23 m, `evidence/phase-4-mapping/2026-10-11-smoke-with-margin`,
  `NOTE-what-this-run-found.md`). Repeats from a standing pose are now filtered (D17) and one hit
  no longer makes a cell occupied (D16), which together keep an isolated flying pixel from
  becoming an obstacle; a pixel that is hit twice from two different poses still would. A range-jump
  filter was not added, because it would also drop thin or grazing real surfaces. What is left
  shows in `surface_error_max_cells`, which is why that is a maximum and is reported next to the
  95th percentile.
* D3 above, and with it T10 and M5.
* The occupancy thresholds, the clamps, `l_occ`, `l_free` and the resolution are the defaults of
  `robotics_mapping::occupancy::OccupancyParams`, each with its reason in that item's
  documentation. Chosen by reasoning before any run, except the occupied threshold, which the
  development runs changed (item 10). They are frozen at the third development run's values.
* The beam-index dependence of the hits that land outside the walls (8.4), not explained. The
  evaluation runs show it is route-dependent (0 of about 3500 behind-hits on two routes, 25.5 per
  cent on the diamond route), so it is not a fixed property of those beams. The read-only analysis in
  `evidence/phase-4-mapping/2026-10-11-evaluation-group/NOTE-root-cause-analysis.md` finds it is an angular error
  of the beams near the two ends of the index range (about -14 to -23 mrad for beams 0 to 9, about +17 mrad for
  350 to 359) that shows only where a route meets the walls obliquely; a table fitted without a run predicts that
  run's rate. Where the error comes from (driver angle grid, sensor, TF) is not determined.
* Completeness below 0.95 on all three evaluation routes (0.926, 0.907, 0.886), not broken down.
* The verdict script's `same_launch_args` group check compares strings containing the route, so it
  is false for any three different routes; not edited, since the script is committed.

## 6. Evidence, and how this part will be run

* Output goes to `evidence/phase-4-mapping/<run>/`, with `summary.yaml` naming the owner of
  `map -> odom`, and a verdict file written by `scripts/phase4_verdict.py` and never typed.
  Tolerances live in that script and are committed before the first evidence run, so no
  metric or threshold is chosen after seeing a result.
* **A run is copied to the Windows checkout the moment it finishes**, before anything else,
  because runs execute in the WSL mirror and `scripts/sync_to_wsl.sh` uses `rsync
  --delete`. This has destroyed evidence before ([../phase-3-localization/implementation.md](../phase-3-localization/implementation.md)
  section 5.3), and the first Phase 4 run is exactly as exposed.
* (Written before Phase 3 closed, kept for the reason it gives.) No Phase 4 run uses Webots, and nothing in Phase 4 that needs the simulator is run, until
  the Phase 3 trials that are using it have finished: they share the simulator, its port and
  the CPU, and the benchmark rules ask for nothing else running.
* Library tests use `robotics_sim` and the closed-form geometry of the room, not Webots, so
  T1 to T5 never wait for the simulator.

## 7. What the first plan deliberately did not do (as written on 2026-10-10; superseded where section 8 says)

It does not edit `robotics_core`, `robotics_filters`, `robotics_models` or `robotics_sim`:
the ESDF reuses `DistanceGrid` as it is. It does not add a workspace member, a package or
a launch file, since a new member changes `Cargo.lock` and a build, and this plan was
written while Phase 3 trials were running from the same workspace. It does not extend
Phase 3: the remaining Phase 3 items stay in [../ROADMAP.md](../ROADMAP.md) as they were,
and the `mcl_node` TF question of D3 is recorded and not acted on.

## 8. What is built, and what the runs say

### 8.1 Status of the tasks

| Id | Task | State |
|---|---|---|
| T1 | `OccupancyGrid`, log-odds, three-state read-out, text form | built, `robotics_mapping::occupancy`, 65 unit tests across the crate |
| T2 | inverse sensor model, ray integration, no-return policy | built, `robotics_mapping::{integrate, raycast}`; also `MotionGate` and `MotionFilter` (`gate`), added after the first runs (D13, D17) |
| T3 | entropy | built, `robotics_mapping::entropy` |
| T4 | map-accuracy evaluator and its example binaries | built, `robotics_mapping::eval`, examples `map_metrics` and `replay_scans` |
| T5 | signed ESDF over the existing exact EDT | built, `robotics_mapping::esdf`; published as a `32FC1` image (D12); no consumer yet |
| T6 | `ground_truth_tf_adapter` | built, `robotics_mapping_ros` |
| T7 | `mapping_node`, `mapping_evaluator` | built, `robotics_mapping_ros`; the `~/reset` service is **not** built (no `std_srvs` Rust bindings in the interface underlay) |
| T8 | harness | built: `scripts/run_phase4_mapping.sh`, `scripts/phase4_verdict.py`, `robotics_bringup/launch/phase4_mapping.launch.py`, `scripts/verify_phase4_mapping.sh`. `benchmarks/phase-4-mapping.csv` is **not** written: it is for the evaluation runs, which now exist (8.3) but have not been written into it |
| T9 | informative references (rclpy port, `nav2_costmap_2d`) | **not built**; non-blocking |
| T10 | estimated-pose integration (M5) | **not started**; waits for D3 |

Integration tests on a synthetic copy of the Phase 3 room (`robotics_mapping/tests/room_mapping.rs`):
determinism and text round trip (M2 in the library), accuracy against the analytic geometry,
nothing written outside the walls, entropy falling, the ESDF against the analytic distance. The
`scripts/gates.sh` gates `fmt`, `clippy`, `test` and `colcon` pass on this work; its `docs` gate
fails on a Phase 3 item that predates it (the saved
`phase-3-global-localization/2026-10-10-ambiguity/wrong-claims-yaw.yaml` counts 5 wrong claims
for the final Webots run and the evidence it is generated from yields 8).

### 8.2 The ESDF convention

The signed field is positive in free space and negative inside an obstacle, with its zero level at
the **cell faces**, not the cell centres: a free cell beside an occupied one reads `+r/2` and the
occupied one `-r/2`. It is `d - r/2` outside and `-(d - r/2)` inside, where `d` is the exact
centre-to-centre distance of `robotics_models::DistanceGrid` to the nearest occupied cell (outside) or
to the nearest cell that is not occupied (inside). Unknown cells count as free space for the
distance; saturation is at the truncation distance (`esdf.truncation_m`, 2 m by default). It is
checked against the brute-force definition on random grids.

### 8.3 The evidence so far

Everything is in `evidence/phase-4-mapping/`; the verdict files are written by the verdict script,
and each run has a `NOTE-what-this-run-found.md` where something needs saying.

| Run | What it is | Result |
|---|---|---|
| `2026-10-10-scan-skew-probe` | a recording with the robot still, driving and turning, against ground truth | the findings of 5.2 items 1 to 3 |
| `2026-10-11-harness-smoke`, `-smoke-with-motion-gate`, `-smoke-with-margin` | three runs of the harness on a 0.5 m triangle | not results: they found the stamp lag, the need for the motion gate and the clear margin, and two harness faults |
| `2026-10-11-development-route-long` | development run 1 | M2, M4 pass; M1 fails on five checks |
| `2026-10-11-development-2-route-long` | development run 2 (D16 was not in effect, see its note) | M2, M4 pass; M1 fails on three checks |
| `2026-10-11-development-3-route-long` | development run 3, parameters frozen | M2, M4 pass; M1 fails on one check (false occupied) |
| `2026-10-11-evaluation-1-cross` | evaluation run 1, `route_eval_cross.yaml` | M2, M4 pass; M1 fails on one check (completeness) |
| `2026-10-11-evaluation-2-wall-lobes` | evaluation run 2, `route_eval_wall_lobes.yaml` | M2, M4 pass; M1 fails on two checks (completeness, false free) |
| `2026-10-11-evaluation-3-diamond-slow` | evaluation run 3, `route_eval_diamond.yaml`, 0.3 rad/s turns | M2, M4 pass; M1 fails on three checks (surface error p95, false occupied, completeness) |
| `2026-10-11-evaluation-group` | the group verdict, the overshoot analysis, and the cross-run note | see its `NOTE-what-the-evaluation-found.md` |
| `phase-4-mapping-runtime/*` | the integration test | see 8.4 |

**What the development runs establish and what they do not.** They establish that the mapper is
deterministic on real simulator data (M2), cheap (M4), and that with ground-truth pose it maps the
room's walls and pillars to within a cell on average, completely and with almost no free-space
errors, and that what separates it from the pre-registered tolerance is a small set of cells in the
row just outside the walls. They do not establish M1: the tolerances are for the evaluation runs, on
routes that played no part in choosing the parameters. The parameters were chosen over three runs
on one route, so the development run's own result is optimistic by construction, and the evaluation
runs (below) bear that out: M1 is met on none of the three, each missing different checks. M5 and the claim that mapping works with an estimator's pose are untouched.

### 8.4 Limitations known now

* The lidar's stamp lags its measurement by up to a scan period (5.2 item 2); the mapper skips
  the scans where that matters (D13) and corrects the mean (D15). A real driver that stamps at
  measurement does not need either; both are parameters.
* About a fifth of the scans are skipped for motion on this route, essentially all of them in the
  turns in place; the map is built from the straight legs.
* The row of cells just outside the walls is the remaining false occupied: hits that land more than
  4.5 cm past a face at a rate that depends on the beam's direction in the sensor frame (3 times in
  1000 over all beams, 2 in 100 for the beams straight behind the robot). Not explained (see
  `2026-10-11-development-3-route-long/NOTE-what-this-run-found.md`).
* Flying pixels at depth edges (5.3), the mount of the lidar read from TF and not checked against
  the proto, and the ground-truth reader `mapping_evaluator`'s position list being a 2 cm-spaced
  subsample of the route.
* The grid's origin puts the walls on lattice lines, so a wall is mapped two cells thick in places.
  The metrics are written to be indifferent to that (closed-cell rule), the occupancy is not.
