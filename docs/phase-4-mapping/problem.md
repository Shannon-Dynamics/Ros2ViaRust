# Phase 4, problem: what does the room look like?

> **Status (2026-10-11): the plan below was built; see
> [implementation.md](implementation.md) section 8 for what exists and what the runs found.**
> The text of this file is the plan as written and confirmed on 2026-10-10 and is not edited
> to match what happened, except where a note says so. There are no numbers in this file; the
> results are in verdict files under `evidence/phase-4-mapping/`, and the first runs found things
> the plan did not know (the lidar stamp lags its measurement, section 5.2 of the implementation
> file), which is why "M1 shows the mapper is implemented correctly" needed more than a mapper.

## 1. The situation

The TurtleBot3 Burger of Phase 3 drives around the same 6 by 6 metre room
([../../sim/webots/worlds/turtlebot3_localization_room.wbt](../../sim/webots/worlds/turtlebot3_localization_room.wbt)),
and for this part it is told where it is. At every instant it has

* a pose `x_t = (x, y, theta)` of `base_link` in the `map` frame, available as the TF chain
  `map -> odom -> base_link` and, for now, derived from the simulator's ground truth
  (section 6 says how, and what that does and does not allow);
* 360 range readings per revolution of the LDS-01 lidar at 5 Hz, with a range limit of
  3.5 m and a noise that is small against that limit.

What Phase 3 gave the robot about the room was a list: four pillar positions and the room
outline in [`landmarks.yaml`](../../sim/webots/config/landmarks.yaml). That is enough to
say where the robot is, and not enough for anything that has to move through the room. A
planner needs to know which places are free, which are occupied, and which have never been
looked at; a list of four pillars says nothing about the space between them.

## 2. Why a scan is not a map

A scan is the answer to one question asked from one place at one instant: how far to the
first thing along each of 360 directions? Three things stand between that and a map.

1. **A scan describes the sensor, not the world.** The ranges are in the lidar frame. To
   say anything about the room they have to be moved through the pose the robot had at the
   moment of that scan, so an error in the pose is an error in the map, one for one.
2. **A beam says two things, and they are not equally strong.** A beam that returns at
   range `r` says the space between the sensor and `r` was empty and that something was at
   `r`. It says nothing at all about what is behind `r`. Cells behind the hit are *unknown*,
   not free, and a map that cannot tell the two apart is one a planner will drive into.
3. **Evidence has to accumulate.** One scan is noisy, a second scan from another pose
   disagrees a little, and a moving object leaves a hit in a cell that a later beam then
   passes through. The map has to keep a belief per cell and revise it, not overwrite it.

## 3. Problem statement

Let `m = {m_i}` be the occupancy of every cell of a grid laid over the room, `m_i in {free,
occupied}`, `x_1:t` the poses and `z_1:t` the scans. This is *mapping with known poses*:
the poses are inputs, not unknowns. The robot must maintain

```text
bel(m) = p(m | z_1:t, x_1:t)  ~  prod_i p(m_i | z_1:t, x_1:t)
```

recursively, one scan at a time and at the scan rate. The product form is the standard
simplification (each cell independent given the poses) and is an assumption, not a fact:
two neighbouring cells of one wall are not independent, and the map ignores that. It is
what makes the update cheap enough to do on every scan.

The output is more than the grid. A grid of beliefs becomes three things the rest of the
stack consumes:

* an occupancy map with three states (free, occupied, unknown), published as
  `nav_msgs/OccupancyGrid` on `/map`;
* a scalar summary of how much is still unknown, the grid's entropy;
* a distance field over the occupied cells, the ESDF, which Phase 6 plans in and Phase 7
  keeps clear of obstacles with.

## 4. What counts as solved, and what kind of claim that is

This section is a decision, and it is made here so that it is not made implicitly later. It
needs the project owner's confirmation before work goes past the pure library.

**The claim of Phase 4 is not a rate.** Phase 3 needed repeated trials because its question
had a random yes-or-no outcome: a particle search either found the robot or it did not,
and one seed says almost nothing about the probability. Mapping has no such event. With
the poses given, the grid is a deterministic function of the scan stream; the only
randomness is the lidar noise, about 15 mm of range (`0.0043` of `maxRange 3.5` in the
Webots model), which is a fraction of one 5 cm cell, and it averages out in a log-odds
update. There is no "mapped" and "failed to map" to take a proportion of. So the
questions below are answered with a run, or with a test, or with a distribution over the
scans of a run, and none of them is answered with a success rate, a Wilson interval, or a
trial set.

| Claim | Kind | Evidence | What is reported |
|---|---|---|---|
| **M1** the map is accurate against the analytic ground truth, given the ground-truth pose | representative run | one development run to fix the parameters, then three evaluation runs on routes that played no part in choosing them | every run on its own row, with the metrics of [benchmark.md](benchmark.md) section 2, and their spread across runs. No pass *rate*. |
| **M2** the mapper is deterministic and agrees with an independent implementation | deterministic test | one recorded scan and pose stream, replayed | the Rust grid is bit-identical on a second replay: this is the blocking part, pass or fail, not a probability. Agreement with an independent (rclpy) implementation within a stated tolerance, cell by cell, is reported when that port exists and **does not block** (decision of 2026-10-10). |
| **M3** the grid obeys its own rules | unit and property tests | `cargo test` | log-odds are clamped; an update order that does not change which cells a scan touches does not change the result; each ray traversal does what it is defined to do (the supercover visits exactly the cells the segment crosses, checked against an independent oracle; the Bresenham line, the mapper's traversal since 2026-10-11, has `max(\|dx\|, \|dy\|) + 1` cells with both ends and unit steps); the beam layout and the evaluator's incidence limit and lookup rule behave as specified (`implementation.md` section 5.4); unknown stays unknown until observed |
| **M4** an update is cheap enough to run on every scan | distribution over scans | every scan of a run | median, 95th and 99th percentile of the update latency, with the accuracy metrics of the same run next to it (benchmarks rule 6) |
| **M5** *(waits for Phase 3)* the map is as accurate when the pose is the real estimator's | representative run | one run per estimator that provides `map -> odom` (forward dependency in section 6) | the M1 metrics, next to the pose error of that run. The claim is "with this pose track, the map is this good", not "mapping with the estimator works". |

**Why a few runs for M1 and not one.** The benchmark rules ask for a fixed set of seeds
so that the spread is visible, and one run cannot show it. Three (confirmed) is enough to
see whether the metrics move by a fraction of a cell or by several, and few enough that
nobody mistakes the table for a statistic. If the three differ by more than a cell, that
is a finding to investigate and not a reason to add runs until the average improves.

**What is explicitly not claimed.**

* No statement of the form "mapping succeeds N per cent of the time". It would need a
  trial set and a defined failure event, and the plan has neither.
* No claim about robustness to pose error as a probability. A curve of map error against
  pose error would need injected pose noise, and Phase 3 found (and deleted) a synthetic
  odometry noise injector that was wrong; it is not reintroduced here. M5 uses the pose
  error that the real estimator actually has.
* No claim that M1 shows mapping works on a real robot. With ground-truth pose the only
  errors left are the lidar noise and the discretization, so M1 shows that the *mapper is
  implemented correctly*, which is worth knowing and is weaker than the claim a reader
  might hear. The pose-induced blur, the part that matters in practice, is invisible to M1
  by construction and appears first in M5.
* The world is the Phase 3 room: four pillars, four walls, no clutter, no corridors, no
  occlusion to speak of. It is an easy world to map, which is what makes it a fair first
  test of correctness and an unfair test of capability. A second, more complex world is
  the backlog item *M1-extended* in [../ROADMAP.md](../ROADMAP.md), deferred until the
  Phase 3 trials are finished; it is not part of Phase 4's claims.

**Ground truth for M1.** The world is analytic, so no second mapper is needed to say what
the map should be: the room is an inner square with its faces at +/- 3.0 m and four
cylinders of radius 0.15 m, in the world file and, duplicated by hand, in `landmarks.yaml`.
The lidar sees *surfaces*, so the ground truth is the set of surfaces a beam can hit, not
a filled raster of the wall volume (wall thickness 0.05 m equals the proposed cell size, so
a raster would make the metrics depend on how the wall straddles the grid). Surfaces the
robot never had a line of sight to, for instance the far side of a pillar from every pose
on the route, are removed by an exact ray cast over the trajectory before scoring, so that
the map is not charged for what it could not have seen; the fraction removed is reported.

## 5. Why this is the next step

Phase 3 answers where the robot is and leaves the world as a list of four points. Mapping is
the first time the robot writes something down about the world itself, and every later phase
reads it: the planner searches it (Phase 6), the controller keeps clear of its ESDF
(Phase 7), the explorer looks for the boundary between its free and unknown cells
(Phase 8), and SLAM (Phase 5) is this same mapper with the poses turned into unknowns.
That last point is why Phase 4 is groundwork: it has no dramatic result of its own, and
Phase 5 inherits every defect it ships with.

## 6. The dependency on Phase 3, stated and not hidden

Mapping with known poses needs a pose. Phase 3 is not closed, so there is no pose estimate
that this part is allowed to call finished. The decision (confirmed 2026-10-10) is that
**the mapper has no pose-source switch.** `mapping_node` reads `map -> base_link` from TF
through the Rust `TfListener` and nothing else, so the same binary runs in every setting and
cannot quietly use a different pose in one of them. What differs is *who publishes
`map -> odom`*:

* In simulation, before Phase 3 closes, a separate node, `ground_truth_tf_adapter`, does:
  it reads `/ground_truth/pose` and the `odom -> base_link` transform and publishes
  `map -> odom = T_map_base * inverse(T_odom_base)`. The adapter has to publish `map -> odom`
  and not `map -> base_link`, because `base_link` already has a parent, `odom`, from
  `diff_drive_controller`, and a TF frame has exactly one. The chain
  `map -> odom -> base_link` then reproduces the ground-truth pose exactly at each stamp.
* `map -> odom` has one owner (REP 105 and the TF authority rule of
  [../TF_BUFFER.md](../TF_BUFFER.md)), so the adapter and `localization_node` never run
  together, and the Phase 4 launch starts one of them, never both.
* The adapter lives in `robotics_mapping_ros`, is simulation only, and is kept out of
  every launch file for a real robot. It is a *reader* of `/ground_truth/pose`, which
  Phase 3 reserved to `route_driver` and `localization_evaluator`; the rule is amended to
  name it and `mapping_evaluator` (which needs the trajectory to decide which surfaces were
  observable), and `mapping_node` is not among the readers.
* **Provenance.** Since the mapper has no flag to record, the run records it: the harness
  writes the owner of `map -> odom` (`ground_truth_tf_adapter` or `localization_node`) into
  `summary.yaml`, the adapter publishes its own `/diagnostics`, and the verdict script
  refuses to label a run M1 unless the adapter owned the edge. A run with the adapter is
  never called a result of the localization stack.

**Forward dependency of M5.** Mapping with the real estimator's pose needs one of those
estimators to own `map -> odom`. `localization_node` (EKF) already does. `mcl_node`
publishes `/mcl/pose` and **no TF**, so for MCL there are two ways, undecided:

1. add a `map -> odom` publisher to `mcl_node`. This is a change to Phase 3 code, and it
   contradicts check F11 of `scripts/verify_phase3_localization.sh` (which asserts that
   `mcl_node` claims no TF edge), so F11 changes with it; or
2. an adapter in `robotics_mapping_ros` that turns `/mcl/pose` into `map -> odom` the
   way the ground-truth adapter turns ground truth, which leaves `mcl_node` and F11 alone.

Either way only one publisher owns the edge in a given run, so M5 is one run per estimator,
never the two at once, and it cannot be started until Phase 3 has closed and said which
estimator is the one to map with.

Phase 3 does not consume `/map`, and Phase 4 does not feed it. Localizing against the grid
(beam or likelihood-field localization) is possible later and is not part of this plan or of
closing Phase 3.

Continue with [implementation.md](implementation.md), then [benchmark.md](benchmark.md).
