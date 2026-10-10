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
| `rust_amcl` | `robotics_localization` through `localization_node` (EKF) and `mcl_node` (MCL) | Rust |
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

Two scenarios exist, and both are driven by the same script over the same route with the
same parameters. `normal` just drives it. `kidnap` drives it and teleports the robot once,
part way through, through the Webots supervisor and without telling either estimator. When a
direct comparison is the point, the script runs both estimators in the *same* Webots session
(`ESTIMATORS="ekf mcl"`), so the two are graded against one ground truth trajectory and, for
`kidnap`, against one teleport at one simulated instant, rather than against two separate
runs that merely share a configured route file.

One run is one directory under
[`../../evidence/phase-3-correctness/`](../../evidence/phase-3-correctness/), produced by
[`../../scripts/run_phase3_correctness.sh`](../../scripts/run_phase3_correctness.sh). Each
holds the environment it ran in and the SHA-256 of the world, the map and the route; the
per-sample rows as `<scenario>-run.csv`; the summary the evaluator wrote at shutdown as
`<scenario>-run.json`; the log of every process; and the verdict on criterion P1 as
`<scenario>-verdict.yaml`, computed from the first two by
[`../../scripts/phase3_verdict.py`](../../scripts/phase3_verdict.py). A kidnap run is graded
instead by [`../../scripts/phase3_kidnap_verdict.py`](../../scripts/phase3_kidnap_verdict.py),
because P1 and P2 ask different questions and need different numbers; that script's docstring
defines exactly what "recovered" means, declares the three constants it depends on, and reports
them in the output file next to the verdict they produced. The EKF's files are unsuffixed and
the MCL's carry a `-mcl` suffix, except for the kidnap verdicts, which are `-ekf` and `-mcl`
because that scenario had no earlier naming to stay compatible with.

Read the verdict file for the numbers. They are deliberately not repeated here: a figure
copied into prose is a figure that stops matching the run it came from, and this phase
already has one example of documentation that outlived its measurement. The runs so far,
newest first:

| Run | Route | What it is for |
|---|---|---|
| [`2026-10-10T144153-final-global-reset`](../../evidence/phase-3-correctness/2026-10-10T144153-final-global-reset) | `route_long.yaml`, 51.540 m, 21 waypoints | final P1 evidence: EKF and MCL both pass in one normal session, with zero MCL recovery injections |
| [`2026-10-10T143418-mcl-global-reset`](../../evidence/phase-3-correctness/2026-10-10T143418-mcl-global-reset) | the same route and a teleport after 120 s | final P2 evidence: MCL recovers after one full global reset; EKF does not |
| [`2026-10-10T142643-final-default`](../../evidence/phase-3-correctness/2026-10-10T142643-final-default) | the same route | retained failed attempt: a five-scan trigger caused seven false injections and failed P1 |
| [`2026-10-10T141910-mcl-6000-persist5`](../../evidence/phase-3-correctness/2026-10-10T141910-mcl-6000-persist5) | the same kidnap | five-scan trigger recovers in 14.120 s, retained because the next normal run shows why it is not the final setting |
| [`2026-10-10T141137-final-mcl`](../../evidence/phase-3-correctness/2026-10-10T141137-final-mcl) | the same kidnap | retained failed attempt: a 20-scan trigger replacing 20 per cent of the cloud did not recover |
| [`2026-10-10T140033-mcl-6000-persist20`](../../evidence/phase-3-correctness/2026-10-10T140033-mcl-6000-persist20) | the same normal route | calibration evidence: 6000 particles and a 20-scan trigger pass P1 with no injection |
| [`2026-10-10T135330-mcl-3000-persist20`](../../evidence/phase-3-correctness/2026-10-10T135330-mcl-3000-persist20) | the same normal route | calibration evidence: no injection, but 3000 particles narrowly fail to beat odometry |
| [`2026-10-10T134509-mcl-median-built`](../../evidence/phase-3-correctness/2026-10-10T134509-mcl-median-built) | the same normal route | first run of the median trigger from the correct build; 300 particles still fail P1 |
| [`2026-10-10T133711-mcl-median`](../../evidence/phase-3-correctness/2026-10-10T133711-mcl-median) | the same normal route | invalid for the median change: it used the stale executable in the root install space and exposed the two-install-space mistake |
| [`2026-10-10T133620-mcl-median`](../../evidence/phase-3-correctness/2026-10-10T133620-mcl-median) | the same normal route | interrupted by a WSL restart, retained with its partial logs and data |
| [`2026-10-10-mcl-gated`](../../evidence/phase-3-correctness/2026-10-10-mcl-gated) | `route_long.yaml`, 21 sharp corners | historical evidence after the per-observation gate; MCL still fails P1 and exposes the recovery-trigger defect fixed by the timestamped runs above |
| [`2026-10-10-mcl-comparison`](../../evidence/phase-3-correctness/2026-10-10-mcl-comparison) | the same route | MCL before it had a per-observation gate, kept as the measurement that found that defect; [implementation.md](implementation.md) section 5.1 |
| [`2026-10-10-long-route`](../../evidence/phase-3-correctness/2026-10-10-long-route) | `route_long.yaml`, 21 sharp corners | the EKF alone on P1, the run that first answered it, no kidnap |
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

P2 now has runs, which is what the `kidnap-verdict-*.yaml` files of
[`2026-10-10T143418-mcl-global-reset`](../../evidence/phase-3-correctness/2026-10-10T143418-mcl-global-reset) are. Read
them rather than this paragraph. What they decide is that the kidnapped robot question has
an answer in this framework and that the answer differs between the two filters by
construction rather than by tuning: one representation can recover and the other cannot, on
the same sensor, the same map and the same teleport. What they do not decide is the cost
side of the comparison at the top of this file, which still has no row.

The final normal verdicts give the directly comparable P1 measurements below. RMSE, P95 and
maximum are position errors against simulator ground truth. Growth is the mean position
error in the last quarter of the run divided by the mean in the first quarter.

| Estimator | RMSE (m) | P95 (m) | Maximum (m) | Growth factor | P1 |
|---|---:|---:|---:|---:|---|
| EKF | 0.008945 | 0.015877 | 0.026104 | 1.167 | PASS |
| MCL | 0.013649 | 0.021430 | 0.121113 | 1.185 | PASS |

The kidnap verdict requires position error below 0.15 m and absolute yaw error below 0.15
rad for 10 consecutive estimator samples. The MCL recovery window begins 6.920 s after the
teleport and its final position error is 0.004491 m. EKF never meets that window and ends at
2.874910 m. These values are generated from the retained CSV files by
`scripts/phase3_kidnap_verdict.py`.
