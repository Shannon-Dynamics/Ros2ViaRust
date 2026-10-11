# Phase 5, benchmark: how SLAM is judged

> **Status (2026-10-11): the metrics and the file format are fixed here, the tolerances are in
> `scripts/phase5_verdict.py`, and no evidence run exists.** `benchmarks/phase-5-slam.csv` is not
> written. This file contains no numbers and will contain none; the rules are those of
> [../../benchmarks/README.md](../../benchmarks/README.md). The claims are S1 to S8 of
> [problem.md](problem.md) section 4.

## 1. What is evaluated, and against what

**The only blocking yardstick is ground truth**: the simulator's pose of `base_link`, exact in
Rusty, and `/ground_truth/pose` in Webots (with the time caveat of problem.md 4.2). Not
odometry, not another SLAM system, and not the map: a map cannot grade a trajectory when the map
was written by the trajectory.

Three pieces of data, kept apart in every row:

| Data | Ground truth | Stamps | Used for |
|---|---|---|---|
| **Rusty, seeded** | exact | exact | S1a, S1b, S2, S3 (partly), S4, S5, S6, S7. The place where the drift can be made large and the loops are known. Model level: **not comparable with a Webots number** (Phase 3 made the same rule for its model-level kidnap rate). |
| **Recorded Webots scans** (`integrated-scans.csv` of the Phase 4 runs, read only) | ground-truth pose at the stamp, used only for registrations passing the motion gate | lag up to a scan period | S1a, S2, S4, S5, S6 on a recorded stream, replayed offline |
| **Live Webots** | `/ground_truth/pose` | as above | the evaluation runs, after the nodes exist. Not in this pass. |

## 2. What is measured

Definitions are fixed here. The tolerances are fixed in `scripts/phase5_verdict.py` before the first
evidence run, each with its derivation next to it, and the file's git history is the proof.

| Metric | Definition | Claim |
|---|---|---|
| registration error | for a registered pair `(i, j)`: translation error `‖t(Z⊖Z_gt)‖` and rotation error `|θ(Z⊖Z_gt)|` of the estimated relative pose `Z` against the ground-truth relative pose `Z_gt = X_gt,i⁻¹ X_gt,j`; per run the median, 95th percentile and maximum, and the count graded out of the count made | S1a |
| convergence trials | over a fixed seeded set of initial offsets, the count ending within a stated error of the truth, per offset range, with the Wilson interval | S1b |
| registration NEES | `νᵀ Λ ν` for `ν = Z ⊖ Z_gt` and `Λ` the reported information, its mean over the run, against the χ²(3) bounds for that many samples; CONSISTENT, OVERCONFIDENT or CONSERVATIVE | S2 |
| ATE | after aligning the estimated keyframe trajectory to ground truth by the SE(2) transform that fixes the first graded keyframe (not a best-fit alignment, which would hide drift), the RMS and the maximum of the keyframe position error | S4 |
| drift | the final graded keyframe's position error divided by the path length travelled to it, in m/m, and likewise the heading error per metre | S4 |
| graded fraction | graded keyframes (or registrations) over made, so that a result on a subset says so | S1a, S4 |
| loop outcomes | counts of accepted-and-correct, accepted-and-wrong and rejected, the last by `Reason`. **Wrong** is a property of the accepted *measurement*: its error against the ground-truth relative pose exceeds 0.05 m or 0.03 rad (the harness writes the limits it used and the verdict script refuses any others). It is not "the keyframes were far apart": a loop between keyframes 1.7 m apart with an accurate measurement is a good constraint, and an early version of this plan labelled by distance and would have called it false. A candidate is a **true loop**, for the recall only, when the ground-truth distance between its keyframes is under 1.5 m (the two views share a wall) and the path between them is at least 6 m | S5 |
| safety count | accepted-and-wrong; **pre-registered to be zero**, and the verdict says what zero supports (the rule of three, `3/n` as an upper bound on the wrong-accept rate at 95 per cent confidence, for `n` accepted candidates). Recall counts only the loops the detector *proposed*; a loop it never proposed is invisible to this file | S5 |
| replay | the final optimized poses and the loop outcomes of a second run of the whole pipeline on the same recorded stream, compared bit for bit | S6 |
| latency | wall time of one registration and of one optimization; median, 95th and 99th percentile over the run, next to the accuracy of the same run | S6 |
| back-end error | EKF-SLAM: NEES of the pose over time; both back ends: ATE against ground truth on the same synthetic run | S7 |

Every row carries the accuracy metrics next to the latency of the same run (benchmarks rule 6).

### 2.1 The metrics file

`slam-metrics.yaml`, written by the run harness (the `slam_synthetic_run` example for
synthetic data), is the only input of the verdict script. Its schema is part of the contract and
is in the header of `scripts/phase5_verdict.py`, with a refusal for any missing or unrecognized key,
so a run cannot be graded on a file the harness did not produce.

## 3. What it is compared with, and why

| Entry | Role | Compared on | Notes |
|---|---|---|---|
| ground truth | the blocking yardstick | everything in section 2 | |
| `lidar_odometry` (this crate's front end alone) | the baseline for S4 | ATE and drift against the same ground truth | the graph is only an improvement where it beats this on the same run, and the comparison is only meaningful after S5 holds |
| `slam_toolbox` (installed in `/opt/ros/jazzy`) | informative, **non-blocking** | ATE against ground truth on the same recorded stream | **excluded from Phase 4 because it estimates its own pose, and that is exactly the Phase 5 comparison.** It is a different algorithm family (Karto scan matching, Ceres pose graph) whose parameters would have to be matched one by one (benchmarks rule 3); every difference is listed in the row. Not built in this pass |
| `nav2_amcl`-style odometry | not used | | Webots' odometry is calibrated to the world and is not a baseline (problem.md 4.1) |

**Why no head-to-head against a SLAM system for the blocking claims.** A difference from
`slam_toolbox` mixes front-end, back-end, loop-closure and parameter differences, and the first
question to answer is whether this implementation is *correct* (against ground truth), not
whether it is better than a mature one. If both are measured against the same ground truth on the
same stream, the comparison is information and blocks nothing.

## 4. Conditions that keep it fair

* Same recorded stream for every entry, same ground-truth source and the same graded subset.
* The development run that fixes the parameters is never one of the evaluation runs, and the
  evaluation runs do not tune anything.
* Parameters that cannot be matched between entries are listed as differences in the row.
* Measured runs headless, nothing else running; host, kernel, ROS distribution, Webots version,
  rustc version, RMW and governor recorded per row (benchmarks rule 7).
* Rusty runs are seeded, and the seed is in the row; a result that held for one seed is shown on
  the others before it is claimed (the webbook found this three times).

## 5. Output

`benchmarks/phase-5-slam.csv` in the long format of the benchmark README, written by the harness;
per-keyframe traces (position error over path length, the loop-candidate table) as JSON with the
same field names. Nothing in this directory is edited by hand.
