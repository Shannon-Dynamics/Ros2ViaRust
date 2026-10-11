# Phase 4, benchmark: how the map is judged

> **Status (2026-10-11): what is measured and the tolerances are fixed and in use;
> `benchmarks/phase-4-mapping.csv` is not written yet.** The metrics of section 2 are computed
> by `robotics_mapping::eval` (graded by `scripts/phase4_verdict.py`, whose tolerances were
> committed before the first run, with two evaluator rules and their pins added at Stage C, `implementation.md` 5.4),
> and four development runs and six evaluation runs exist (`evidence/phase-4-mapping/2026-10-11-development-*`,
> `-evaluation-*`; the first three evaluation runs were graded by the evaluator as it was and are burned); their numbers
> are in their `map-metrics.yaml` and `phase4-verdict.yaml`, not here. The CSV is for the evaluation runs,
> which are made but not yet written into it. The rclpy port and `nav2_costmap_2d` of section 3 are not built (non-blocking).
> This file contains no numbers and will contain none; the rules are those of
> [../../benchmarks/README.md](../../benchmarks/README.md).

## 1. What is evaluated, and against what

The claims are M1 to M5 of [problem.md](problem.md) section 4. Only M1, M4 and M5 are
measured on a map; M2 and M3 are tests and have no benchmark row. M1 is one development run
and three evaluation runs, confirmed on 2026-10-10.

The yardstick for accuracy is **the analytic geometry of the world**, not another
implementation. The room is known exactly (inner faces at +/- 3.0 m, four cylinders of
radius 0.15 m), so "what the map should be" needs no reference stack to define it, and
every implementation, ours or a reference, can be graded against the same thing.

## 2. What is measured

Definitions are fixed here; the tolerance `tau` and the occupancy thresholds are fixed in
`scripts/phase4_verdict.py` before the first evidence run. The proposal for `tau` is one
cell width, the least a discretized wall can be asked to be accurate to.

Let `S` be the ground-truth surfaces, sampled finely, and `S_obs` the subset of them that
an exact ray cast shows to be visible from at least one pose of the run's trajectory.

| Metric | Definition | Why |
|---|---|---|
| surface error | for every cell the map calls occupied, the distance from its centre to the nearest point of `S`; mean, 95th percentile, and maximum, in metres and in cells | accuracy of what the map claims is there |
| completeness | the fraction of `S_obs` with a map-occupied cell within `tau` | the map is not missing what it saw |
| false-occupied | the number and fraction of cells farther than `tau` from every point of `S` that the map calls occupied | phantom obstacles, which block a planner |
| false-free | the number and fraction of `S_obs` samples whose cell the map calls free | the dangerous direction: a planner drives through this. Reported apart from completeness, which also counts *unknown* |
| coverage | the fraction of cells inside the lidar's reach of the trajectory that the map has decided (free or occupied) | how much of what could be known is known |
| observable fraction | `|S_obs| / |S|` | so that a map is never charged for a surface it could not see, and the reader can see how much was set aside |
| entropy trace | grid entropy over simulated time, total and over decided cells | progress, and where the map stopped learning. **Never reported alone:** a confidently wrong map has low entropy |
| update latency | wall time of one scan integration, from scan in to grid updated; median, 95th and 99th percentile over every scan of the run | M4; the tail, not the mean |
| CPU time per simulated second | process CPU over the run | what competes with the rest of the stack |
| memory | peak resident set size, and the grid's own size | |
| owner of `map -> odom`, and pose error | which node published the edge (`ground_truth_tf_adapter` or an estimator), and for an estimator the pose error of the same run | a result without it cannot be read (M5) |

Every row carries the accuracy metrics next to the latency of the same run, since a fast
mapper that is wrong is not a result.

## 3. What it is compared with, and why

**The baseline, and the only blocking yardstick, is the analytic geometry of the world**
(decision of 2026-10-10). Everything below is additional information: it is run if it can
be, reported if it is, and no Phase 4 verdict waits for it.

| Entry | Role | Compared on | Notes |
|---|---|---|---|
| `rust_mapping` | the subject, graded against the analytic geometry | everything in section 2 | blocking |
| `rclpy_mapping`, a direct Python port of the same log-odds update | informative, **non-blocking** | cell-by-cell agreement with the Rust grid on the M2 recording; update latency | a second opinion on correctness and the slow reference for cost. If the fast implementation and the port agree and both are wrong, the analytic baseline is what catches it |
| `nav2_costmap_2d`, the obstacle layer | informative, **non-blocking** | surface error, completeness, false-occupied, false-free, latency | a costmap with marking and clearing, not a Bayesian log-odds grid: a *different algorithm with a comparable output*, reported as that. Whether it can be run headless on the same scan stream without the rest of Nav2 has not been checked; if not, the entry is dropped and the file says so |
| `slam_toolbox` | excluded | none | it estimates its own pose, so a difference from our map would mix mapping error with SLAM error, the confound Phase 4 exists to avoid. It belongs to the Phase 5 comparison |

**Why there is no head-to-head against "Nav2's mapping stack".** Nav2 does not ship a
mapper that takes poses as given: the maps it uses come from `slam_toolbox` (a SLAM system)
or from a map file served by `nav2_map_server`, and its own `costmap_2d` is a planner-facing
obstacle map. The honest evidence for Phase 4 is therefore the internal one against ground
truth.

## 4. Conditions that keep it fair

* Same world file, robot, driver, `/scan` stream and pose stream for every entry. The
  entries consume a *recorded* scan and pose stream offline wherever they can, so that
  they see the same measurements in the same order; the recording is the evidence for M2.
  The recorded pose stream is the TF chain the mapper saw, whoever owned `map -> odom`, and
  the owner is written into every row.
* Same resolution, same clamping and same sensor-model parameters where an implementation
  has them; every one that cannot be matched is listed as a difference in the row.
* Measured runs headless (`--no-rendering`), nothing else running, host, kernel, ROS
  distribution, Webots version, rustc version, RMW and governor recorded per row.
* The development run that fixes the parameters is never one of the evaluation runs.

## 5. Output

`benchmarks/phase-4-mapping.csv`, written by the Phase 4 harness script, in the long
format of the benchmark README (one row per entry, seed, configuration and metric). Per-cell
traces, such as the entropy over time, are better as JSON with the same field names, which
that README allows. Nothing in this directory is edited by hand.
