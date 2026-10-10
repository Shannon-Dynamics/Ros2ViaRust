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
rad for 10 consecutive estimator samples. In that single older run the MCL recovery window
begins 6.920 s after the teleport and its final position error is 0.004491 m; that is one run
and an outlier against the twenty Webots trials of section 7.5, whose fastest recovery is
15.480 s. EKF never meets that window and ends at
2.874910 m. These values are generated from the retained CSV files by
`scripts/phase3_kidnap_verdict.py`.

## 7. Global localization from a uniform prior

The problem is [problem.md](problem.md) section 6, the filter and its convergence criterion
are [theory.md](theory.md) section 8, and what was built and what was tried is
[implementation.md](implementation.md) section 6. This section is the measurements.

**Every number in this section is generated.** The tables below are written by
[`scripts/phase3_global_tables.py`](../../scripts/phase3_global_tables.py) from the
`summary.yaml`, `trials.csv` and per-trial verdict files under
[`../../evidence/phase-3-global-localization/`](../../evidence/phase-3-global-localization/),
spliced between the markers, and the `docs` gate of
[`../../scripts/gates.sh`](../../scripts/gates.sh) fails if they drift from the evidence. Do
not edit them by hand; add an experiment and regenerate.

### 7.1 Three experiments, and what each can and cannot say

| Experiment | What it is | What it cannot say |
|---|---|---|
| model level | the real `McLocalizer` against a simulated robot following the same exploration route, with simulated detections: ideal, noisy, no false or missed ones, exact odometry | anything about the real detector, the laser geometry or the simulator's odometry |
| end to end in Webots | the real `mcl_node`, detector, laser and odometry, graded against the supervisor's ground truth | how a result varies over many seeds: it is a few dozen trials, not hundreds |
| kidnap, model level | the existing tracking filter's kidnap recovery, unchanged, as a measurement | anything about a configuration other than the one that ships (the 48000 row is a labelled what-if) |

The model-level experiment is cheap, so it carries the statistics; the Webots one carries the
realism. Where they disagree, the Webots one is the answer and the disagreement is the
finding, and there is one, in 7.4.

### 7.2 Results

<!-- global-localization-tables:begin -->

**Model-level, 40 development start poses (starts seed 1000).** The real `McLocalizer`, a simulated robot and simulated detections (`crates/robotics_localization/examples/global_trials.rs`). These are the poses the explained-fraction criterion was chosen on, so they are optimistic for it.

| global particles | explained fraction | trials | success | wrong | lost | no claim | success rate | Wilson 95% | median time (s) | median distance (m) | median RMSE after (m) | correct at end of route |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 6,000 | 0.5 | 40 | 12 | 24 | 0 | 4 | 30% | 18 to 45% | 7.4 | 1.14 | 0.0247 | 12 |
| 12,000 | 0.5 | 40 | 18 | 22 | 0 | 0 | 45% | 31 to 60% | 3.4 | 0.43 | 0.0246 | 18 |
| 24,000 | 0.5 | 40 | 27 | 13 | 0 | 0 | 68% | 52 to 80% | 3.6 | 0.54 | 0.0226 | 28 |
| 48,000 | 0.5 | 40 | 32 | 8 | 0 | 0 | 80% | 65 to 90% | 3.4 | 0.54 | 0.0212 | 32 |
| 96,000 | 0.5 | 40 | 36 | 4 | 0 | 0 | 90% | 77 to 96% | 3.0 | 0.47 | 0.0186 | 36 |
| 192,000 | 0.5 | 40 | 39 | 1 | 0 | 0 | 98% | 87 to 100% | 2.8 | 0.47 | 0.0118 | 39 |
| 384,000 | 0.5 | 40 | 39 | 1 | 0 | 0 | 98% | 87 to 100% | 2.8 | 0.47 | 0.0110 | 40 |
| 6,000 | 0.51 | 40 | 12 | 18 | 0 | 10 | 30% | 18 to 45% | 8.0 | 1.14 | 0.0233 | 12 |
| 12,000 | 0.51 | 40 | 18 | 17 | 0 | 5 | 45% | 31 to 60% | 4.2 | 0.44 | 0.0228 | 18 |
| 24,000 | 0.51 | 40 | 27 | 11 | 0 | 2 | 68% | 52 to 80% | 3.8 | 0.54 | 0.0213 | 27 |
| 48,000 | 0.51 | 40 | 32 | 8 | 0 | 0 | 80% | 65 to 90% | 3.6 | 0.54 | 0.0189 | 32 |
| 96,000 | 0.51 | 40 | 36 | 4 | 0 | 0 | 90% | 77 to 96% | 3.2 | 0.50 | 0.0181 | 36 |
| 6,000 | 1.0 | 40 | 12 | 6 | 0 | 22 | 30% | 18 to 45% | 9.2 | 1.32 | 0.0146 | 12 |
| 12,000 | 1.0 | 40 | 17 | 8 | 0 | 15 | 42% | 29 to 58% | 7.8 | 1.08 | 0.0151 | 18 |
| 24,000 | 1.0 | 40 | 27 | 6 | 0 | 7 | 68% | 52 to 80% | 7.4 | 1.14 | 0.0140 | 27 |
| 48,000 | 1.0 | 40 | 33 | 5 | 0 | 2 | 82% | 68 to 91% | 6.0 | 0.76 | 0.0168 | 33 |
| 96,000 | 1.0 | 40 | 36 | 4 | 0 | 0 | 90% | 77 to 96% | 4.4 | 0.79 | 0.0133 | 36 |
| 192,000 | 1.0 | 40 | 39 | 1 | 0 | 0 | 98% | 87 to 100% | 3.8 | 0.59 | 0.0117 | 39 |
| 384,000 | 1.0 | 40 | 40 | 0 | 0 | 0 | 100% | 91 to 100% | 3.6 | 0.50 | 0.0103 | 40 |

**Model-level, 100 fresh start poses (starts seed 2000).** The same experiment on poses that played no part in any choice.

| global particles | explained fraction | trials | success | wrong | lost | no claim | success rate | Wilson 95% | median time (s) | median distance (m) | median RMSE after (m) | correct at end of route |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 6,000 | 0.5 | 100 | 29 | 60 | 0 | 11 | 29% | 21 to 39% | 4.2 | 0.72 | 0.0309 | 30 |
| 48,000 | 0.5 | 100 | 66 | 34 | 0 | 0 | 66% | 56 to 75% | 3.2 | 0.50 | 0.0167 | 67 |
| 96,000 | 0.5 | 100 | 81 | 17 | 0 | 2 | 81% | 72 to 87% | 3.4 | 0.54 | 0.0139 | 83 |
| 6,000 | 1.0 | 100 | 29 | 25 | 0 | 46 | 29% | 21 to 39% | 9.0 | 1.28 | 0.0156 | 29 |
| 48,000 | 1.0 | 100 | 66 | 26 | 0 | 8 | 66% | 56 to 75% | 6.0 | 0.97 | 0.0125 | 67 |
| 96,000 | 1.0 | 100 | 82 | 15 | 0 | 3 | 82% | 73 to 88% | 5.6 | 0.97 | 0.0120 | 84 |

**Cost of one scan, single process, nothing else running.**

_Not measured yet._

**End to end in Webots, final: each configuration over three filter seeds, pooled** (real detector, real laser, real odometry, ground truth from the supervisor; the same 20 start poses for every seed). The claim-based column is what the filter said and was right about (`SUCCESS`); the ground-truth column is where the estimate actually was at the end of the route, whatever was claimed. Each total is followed by the range over the seeds.

_Not run yet._

**The final Webots runs, one row per run.**

_Not run yet._

**The final Webots runs, one row per start pose**, counted over the three seeds of each configuration (`ok` claimed and right, `WRONG` a wrong claim, `LOST` a right claim lost again, `none` no claim; `end` is how many of the seeds ended inside both tolerances, by ground truth).

_Not run yet._

**The first Webots batch, seed 1, before the harness fixes**, kept for comparison: its diagnostics columns cover only the trials whose one `/diagnostics` message happened to be mcl_node's, and its injections cannot be graded because the log line had no stamp. Claims, errors and the ground-truth column do not depend on either.

| global particles | build | seed | explained fraction | trials | success | wrong | lost | no claim | invalid | success rate | Wilson 95% | median time (s) | median distance (m) | median RMSE after (m) | median of per-trial max ms/scan | trials with > 4 detections per scan, of those with a reading | trials where the kidnap trigger fired | injections: total (false, estimate already right) | correct at end of route (ground truth) | success, 10 random starts | success, 10 ambiguity-targeted starts | wrong claims, random / targeted | no claim, random / targeted |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 6,000 | debug | 1 | 0.5 | 20 | 6 | 13 | 1 | 0 | 0 | 30% | 15 to 52% | 8.3 | 0.99 | 0.0296 | 208 | 11 of 11 | 8 | 13 (ungraded: log has no stamp) | 11 | 5 of 10 | 1 of 10 | 4 / 9 | 0 / 0 |
| 48,000 | release | 1 | 1.0 | 20 | 9 | 0 | 1 | 10 | 0 | 45% | 26 to 66% | 9.4 | 1.08 | 0.0175 | 99 | 8 of 8 | 1 | 2 (ungraded: log has no stamp) | 10 | 6 of 10 | 3 of 10 | 0 / 0 | 3 / 7 |
| 96,000 | release | 1 | 1.0 | 20 | 11 | 0 | 1 | 8 | 0 | 55% | 34 to 74% | 8.1 | 0.64 | 0.0178 | 238 | 5 of 6 | 2 | 4 (ungraded: log has no stamp) | 12 | 8 of 10 | 3 of 10 | 0 / 0 | 1 / 7 |

<!-- global-localization-tables:end -->

Reading the tables:

* **Wrong, lost, no claim** are the three failures of [problem.md](problem.md) section 6. A
  filter that is confident and wrong is the one a robot would act on.
* **Explained fraction** is `global.convergence_min_explained_fraction`: 0.5 is the
  upper-median rule, 1.0 requires the cloud's mean to explain every detection of the scan.
* The **development** poses are the ones the explained fraction was chosen on; the **fresh**
  poses played no part in any choice. Compare the two to see how much choosing on a set
  flatters it.
* **Time** and **distance** are from the first scan to the claim, among the successes only,
  and distance is the ground-truth path length. They include the delay a criterion judged on
  spread suffers at turns (theory.md section 8).
* Timings in the model-level sweeps were taken with several processes sharing the machine and
  are inflated; the **single-process** table is the one to read for cost.

### 7.3 What the results say

The statements here are about the shape of the tables and are meant to be checked against
them, not read as numbers.

1. **Particle count is the lever.** At every step up in the global particle count the success
   rate goes up, at both criteria, on both start sets. 6000, the steady-state count, is not
   enough; this is particle deprivation (theory.md section 8) and not a bug.
2. **The strict criterion trades a confident wrong claim for no claim.** The same trials
   succeed at every count; the wrong claims fall and the no-claim count rises by about the
   same amount. It does not find the pose more often. It makes the filter safer to trust,
   and it is not a substitute for particles.
3. **The wrong claims are rotated twins.** In the model-level trials the yaw error at a wrong
   claim is, almost every time, close to half a turn or a quarter turn (`err_yaw_at_conv_rad`
   in `trials.csv`), which is the map's near rotational symmetry
   ([`map-symmetry.yaml`](../../evidence/phase-3-global-localization/2026-10-10-ambiguity/map-symmetry.yaml)).
4. **The counts that find the pose best are not real time.** The single-process cost table
   gives the cost per scan against the 200 ms scan period.

### 7.4 Where Webots and the model disagree

See the per-start table and the detector columns above. The ideal detector of the model-level
experiment never reports a detection that is not a pillar. The real one does, near walls, and
the tracking filter's kidnap trigger, which a converged search hands over to, reads a scan
with more spurious detections than genuine ones as a kidnapping and wipes out a correct pose.
That is `LOST_AFTER_CONVERGENCE`, which the model never produces and Webots does. It is a
property of the inherited trigger meeting the real detector and not of the search, and it is
not changed here (the kidnap tuning is out of scope), but it bounds what any search from
the middle of the room's edge can promise.

### 7.5 The kidnap tuning, measured

The tracking filter's kidnap recovery, exactly as it ships, is the model-level kidnap
experiment, and it is also a global localization from a uniform reset (implementation.md
section 6.4). Its recovery rate over many kidnaps is in the file below, with the what-if at a
larger count next to it.

<!-- kidnap-model-tables:begin -->

| configuration | steady-state particles | persistence (scans) | reset fraction | kidnaps | recovered | recovery rate | Wilson 95% | median recovery time (s) |
|---|---|---|---|---|---|---|---|---|
| as shipped | 6,000 | 20 | 1 | 102 | 40 | 39% | 30 to 49% | 5.2 |
| what-if: particle count only | 48,000 | 20 | 1 | 36 | 29 | 81% | 65 to 90% | 5.2 |

<!-- kidnap-model-tables:end -->

**Webots kidnap trials, a separate result that is not part of the generated table above.**
At one fixed kidnap geometry (seeds 1001–1020), MCL recovered 20/20. On the model sample that varies the kidnap geometry (n=102), the recovery rate is ≈39%/81% (depending on the condition). Both results measure the same `mcl.rs` code but cannot be compared directly, because the kidnap coverage tested differs.

The conditions behind the two model figures, so that the sentence above is not read as one sample:
39% is the as-shipped configuration (6000 particles, n=102); 81% is the labelled what-if at 48000
particles (n=36), both from the table above.

The 20 Webots trials are
[`2026-10-10T2026-c56f55d-verdicts`](../../evidence/phase-3-correctness/2026-10-10T2026-c56f55d-verdicts),
twenty complete `run_phase3_correctness.sh kidnap` sessions, each with one teleport at the same
target and the same time (120 s) and a different `MCL_SEED`. The figures below are copied from
that directory's `kidnap-trials-summary.yaml`, which
[`../../scripts/phase3_kidnap_trials_summary.py`](../../scripts/phase3_kidnap_trials_summary.py)
wrote from each trial's own verdict files:

| Estimator | Recovered | Rate | Wilson 95% | Recovery time min / median / max (s) |
|---|---:|---:|---|---|
| MCL | 20 of 20 | 1.0000 | 0.8389 to 1.0000 | 15.480 / 47.070 / 206.580 |
| EKF | 0 of 20 | 0.0000 | 0.0000 to 0.1611 | not applicable |

The interval describes the particle filter's randomness for this one kidnap, not the variety
of possible kidnaps (the trials are independent only in seed and simulator timing). The
earlier single recovery time of 6.92 s is not representative: no trial of the twenty recovered
in under 15.480 s, and the slowest took 206.580 s (see the time-bound limitation in
[ROADMAP.md](../ROADMAP.md)).

### 7.6 The runs

| Run | What it is for |
|---|---|
| `2026-10-10-smoke-6000-first-look` | the first look at 6000 particles, ten trials, by hand; why the sweep exists |
| `2026-10-10-sweep-particles-default-criterion` | 6000 to 96000 particles, upper-median criterion, 40 development poses |
| `2026-10-10-sweep-particles-large-default-criterion` | 192000 and 384000, same |
| `2026-10-10-sweep-particles-explained-0.51`, `...-explained-1.0` | the stricter criterion, 6000 to 96000, same poses |
| `2026-10-10-sweep-particles-large-explained-1.0` | 192000 and 384000, strict |
| `2026-10-10-confirm-fresh-starts-explained-0.5`, `...-1.0` | 100 poses that played no part in choosing anything |
| `2026-10-10-ambiguity` | the static ambiguity ranking and the map symmetry |
| `2026-10-10-starts` | every start pose set used, as files |
| `2026-10-10-webots-*` | end to end in Webots, three configurations, the same 20 poses |
| `2026-10-10-webots-smoke-1-trial` | the one-trial smoke test of the Webots runner, kept |
| `2026-10-10-kidnap-model-*` | the existing kidnap recovery as a model-level measurement |
| `phase-3-correctness/2026-10-10T2026-c56f55d-verdicts` | twenty real Webots kidnap sessions, one fixed teleport, seeds 1001 to 1020; numbers in section 7.5 |
| `phase-3-localization-runtime/*global-localization*` | the integration test, including one run that hung and was invalid, with its note |
