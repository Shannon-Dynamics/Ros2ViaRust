# Phase 3, benchmark: Rust against the reference stacks

> **TODO: no measurement has been taken yet.** This file describes the comparison that
> closes Phase 3 and nothing else. There are no numbers in it, and there will be none in
> it later either: results are stored as data in
> [`../../benchmarks/phase-3-localization.csv`](../../benchmarks/phase-3-localization.csv)
> and this text will only interpret them. Methodology and file format are in
> [../../benchmarks/README.md](../../benchmarks/README.md).

## 1. What is compared

The same localization job, in the same Webots world, on the same recorded robot
trajectory, done three ways:

| Entry | Implementation | Language |
|---|---|---|
| `rust_amcl` | `robotics_localization` through `localization_node` | Rust |
| `nav2_amcl` | `nav2_amcl`, the standard ROS 2 localization node | C++ |
| `rclpy_amcl` | a direct rclpy port of the same algorithm, used as the slow reference and as a correctness oracle | Python |

The Python entry is not there to lose a speed contest. It is there to show the cost of the
same algorithm expressed three ways, and to catch the case where the fast implementations
agree with each other and are both wrong.

## 2. What is measured

**TODO**, to be fixed before the first run so that no metric is chosen after seeing the
results:

| Metric | Definition | Why |
|---|---|---|
| update latency | wall clock time of one filter cycle, from measurement in to pose out, reported as median, 95th and 99th percentile, never as a mean alone | the question is the tail, not the average |
| CPU time per second of simulated time | process CPU over a fixed run | what actually competes with the rest of the stack on a small board |
| memory | peak resident set size | particle counts in the thousands are not free |
| absolute trajectory error | estimate against simulator ground truth, mean and 95th percentile | an implementation that is fast and wrong is not faster |
| consistency | NEES inside the chi-square band, fraction of the run | see [problem.md](problem.md), section 4 |
| convergence | time and distance from a uniform prior to a single mode (P2) | the global localization half of the problem |
| recovery | time to recover after a teleport | the augmented MCL mechanism, measured instead of assumed |
| particle count | the KLD adapted count over time | explains the latency curve rather than just reporting it |

Each metric is recorded per seed, and every run is repeated over a fixed set of seeds, so
that the spread is visible instead of a single lucky run.

## 3. Conditions that keep it fair

* Same world file, same robot, same driver, same `/scan`, `/odom` and `/imu` streams: the
  simulation is started once from
  [`localization_world.launch.py`](../../sim/webots/launch/localization_world.launch.py)
  and only the node under test is swapped.
* Same trajectory, replayed rather than driven live, so that all three entries see the
  identical measurement sequence. **TODO**: record a bag and decide whether the three
  entries consume it offline or run against the live simulation one at a time.
* Same algorithm parameters where the implementations allow it: particle count bounds,
  KLD epsilon and z, resampling trigger, laser model and beam subsampling, motion noise.
  Any parameter that cannot be matched is listed as a difference, not quietly left alone.
* Webots in `realtime` mode and `--no-rendering` for the measured runs, since rendering
  competes for the same CPU.
* Host, kernel, ROS distribution, Webots version, rustc version, RMW implementation and
  CPU governor recorded with every row.
* Nothing else running: no RViz, no recording, no visualization topics at a rate that
  matters.

## 4. Known difficulty, resolved: where ground truth comes from

**Settled: a supervisor controller.** Ground truth has to come out of Webots, and the
Phase 2 stack solved it with its own simulator
([`sim_node`](../../ros2_ws/src/robotics_sim_bridge/src/sim_node.rs) publishes
`ground_truth/odom`), which has no equivalent here. The two options were a supervisor
controller reading the true pose out of the scene tree, or a GPS device in the world. The
first was chosen, because the same supervisor is also what teleports the robot for the
kidnap scenario, and because it adds no device to the robot that the estimator could
accidentally subscribe to.

It is [`sim/webots/scripts/evaluation_supervisor.py`](../../sim/webots/scripts/evaluation_supervisor.py),
an extern Webots supervisor attached to the devices-free `evaluation_supervisor` robot of
the world. The launch starts it on `ground_truth:=true`; it publishes
`/ground_truth/pose` at `ground_truth_rate_hz` (default 50) and teleports the robot when a
pose arrives on `/kidnap/teleport`. `localization_node` never subscribes to it, which is
what keeps the kidnap experiment honest; only `localization_evaluator` does.

## 5. Output

One row per entry per seed in
[`../../benchmarks/phase-3-localization.csv`](../../benchmarks/phase-3-localization.csv),
written by the benchmark script, never by hand. This file then says what the numbers mean
and what surprised us. That file is still only a header: no benchmark row exists yet, so
there is nothing here to interpret.

## 6. Correctness runs, which are not benchmarks

The comparison above has not started. What does exist is the correctness side of the same
apparatus: runs of the Rust entry alone against simulator ground truth, which answer
whether the filter works before anything asks how fast it is. They use the same world, the
same ground truth source and the same route, so a benchmark row later can be read next to
them.

One run is one directory under
[`../../evidence/phase-3-correctness/`](../../evidence/phase-3-correctness/), produced by
[`../../scripts/run_phase3_correctness.sh`](../../scripts/run_phase3_correctness.sh). Each
holds the environment it ran in and the SHA-256 of the world, the map and the route; the
per-sample rows as `<scenario>-run.csv`; the summary the evaluator wrote at shutdown as
`<scenario>-run.json`; the log of every process; and the verdict on criterion P1 as
`<scenario>-verdict.yaml`, computed from the first two by
[`../../scripts/phase3_verdict.py`](../../scripts/phase3_verdict.py).

Read the verdict file for the numbers. They are deliberately not repeated here: a figure
copied into prose is a figure that stops matching the run it came from, and this phase
already has one example of documentation that outlived its measurement. The runs so far,
newest first:

| Run | Route | What it is for |
|---|---|---|
| [`2026-10-10-long-route`](../../evidence/phase-3-correctness/2026-10-10-long-route) | `route_long.yaml`, 51 m, 21 sharp corners | the current answer on P1, position tracking, no kidnap |
| [`2026-10-06-after-fix`](../../evidence/phase-3-correctness/2026-10-06-after-fix) | a 0.9 m square, 3.1 m of path | the run that showed the route, not the filter, was the problem; why is in [theory.md](theory.md) section 6 |
| [`2026-10-06-nodes-killed`](../../evidence/phase-3-correctness/2026-10-06-nodes-killed) | the same square | a run whose nodes were killed before the evaluator could write its summary: a CSV with no JSON |
| [`2026-10-06`](../../evidence/phase-3-correctness/2026-10-06) | the same square | 588 rows grading the filter against a robot that never moved; kept as the reason the guards in [`../../scripts/phase3_guards.sh`](../../scripts/phase3_guards.sh) exist. See its `INVALID.md` |

Two parts of the P1 row of [problem.md](problem.md) are decided by these runs and one is
not. `strictly_better_than_odometry` and `bounded_over_distance` are in every verdict file.
The NEES part is not: it needs the full 3 by 3 covariance of each estimate and the CSV
carries only `sigma_xy`, so the verdict file reports `nees_in_95_band: not_evaluated`
rather than guessing. Closing that is a change to
[`localization_evaluator`](../../ros2_ws/src/robotics_localization_bridge/src/localization_evaluator.rs),
not to the filter.

The kidnap scenario has a harness and has never been run. There is no evidence directory
for it, which is the only reason this file does not discuss P2.
