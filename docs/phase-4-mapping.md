# Phase 4: mapping

The robot knows where it is; now it has to write down what it has seen, turning a stream of
lidar scans taken from known poses into an occupancy grid and a distance field the rest of
the stack can plan in.

> **Status (2026-10-11): built and measured on development runs; the evaluation runs of M1 have
> not been made.** The pure crate `robotics_mapping`, the ROS package `robotics_mapping_ros`
> (`ground_truth_tf_adapter`, `mapping_node`, `mapping_evaluator`), the launch file, the run
> script, the verdict script and the integration test exist; see section 8 of
> [implementation](phase-4-mapping/implementation.md) for exactly what, and
> `evidence/phase-4-mapping/` for the runs. With ground-truth pose the mapper is deterministic and
> cheap, and on the third development run it meets six of the seven pre-registered accuracy
> checks of M1 and misses the seventh (false occupied cells, 2.6 per cent against 2). That is a
> development result: the verdicts that count are the evaluation runs, on routes that played no
> part in choosing the parameters, and they are not made. M5 (the estimator's pose) waits for the
> open decision D3.

## Where the detail is

| Document | Content |
|---|---|
| [problem](phase-4-mapping/problem.md) | the situation, why a scan is not a map, the problem statement, **what kind of claim Phase 4 makes**, and the dependency on Phase 3 |
| [implementation](phase-4-mapping/implementation.md) | tasks T1 to T10, the planned interfaces, what waits for Phase 3, the decision record and the open questions |
| [benchmark](phase-4-mapping/benchmark.md) | the metrics, the baseline and what it is and is not compared with |

Scope and exit criteria in the roadmap: [ROADMAP.md, Phase 4](ROADMAP.md). Theory and ROS
graph documents are written when there is something built to describe, as in Phase 3.

## The design in one page

**The problem.** Mapping with known poses: maintain
`bel(m) = p(m | z_1:t, x_1:t)` over a grid, one scan at a time, with each cell independent
given the poses. The output is a three-state occupancy map on `/map`, the grid's entropy,
and a signed ESDF.

**The claims, and that none of them is a rate.** With the poses given, the map is a
deterministic function of the scan stream, and the lidar noise is a fraction of one cell, so
there is no yes-or-no event to take a proportion of.

| Claim | Kind | Evidence |
|---|---|---|
| M1 the map is accurate against the analytic geometry of the world, given the ground-truth pose | representative run | one development run, then three evaluation runs, each reported on its own with the spread; no rate, no interval |
| M2 the mapper is deterministic | deterministic test | bit-identical replay (blocking); agreement with an rclpy port is informative |
| M3 the grid obeys its own rules | unit and property tests | `cargo test` |
| M4 an update is cheap enough for every scan | distribution over scans | median, 95th and 99th percentile, next to the accuracy of the same run |
| M5 the map is as accurate under the real estimator's pose | representative run | **waits for Phase 3** |

What is not claimed: any success rate, robustness to pose error as a probability, or that M1
shows mapping works on a real robot (with ground-truth pose it shows the mapper is
implemented correctly; pose-induced blur first appears in M5).

**The pose, without a flag.** `mapping_node` reads `map -> base_link` from TF and nothing
else. In simulation a separate `ground_truth_tf_adapter` provides the chain by publishing
`map -> odom` from the simulator's ground truth (not `map -> base_link`: `base_link` already
has the parent `odom`). It is exclusive with `localization_node`, since `map -> odom` has one
owner, and a run records who owned it. The mapper is not a reader of `/ground_truth/pose`;
the adapter and `mapping_evaluator` are.

**Forward dependency for M5.** `mcl_node` publishes `/mcl/pose` and no TF. Mapping with
MCL's pose needs either a `map -> odom` publisher in `mcl_node` (a Phase 3 code change that
also changes check F11) or an MCL-to-TF adapter. Undecided; a Phase 3 matter either way.

**Baseline.** The analytic geometry of the world, scored by surface error, completeness,
false-occupied and false-free cells, coverage, entropy as a progress trace (never alone) and
update latency. The rclpy port and `nav2_costmap_2d` are additional information and block
nothing; `slam_toolbox` is excluded because it estimates its own pose.

**Out of scope.** The 2-D TSDF and the quadtree (Ch. 19) are a Phase 5+ concern. A second,
more complex world is the backlog item M1-extended, after the Phase 3 trials.

**Packages.** Crate `crates/robotics_mapping`; ROS package `ros2_ws/src/robotics_mapping_ros`
with `ground_truth_tf_adapter`, `mapping_node` and `mapping_evaluator`.

## What could start before Phase 3 closed (it has since closed)

The pure crate (grid, inverse model, entropy, accuracy evaluator, ESDF), the adapter, the
node, the harness and the verdict script, all on ground-truth pose. Running any of it in
Webots waits until the Phase 3 trials have finished. What cannot start is the
estimated-pose integration (M5), which needs Phase 3 to have closed and to have said which
estimator to map with.
