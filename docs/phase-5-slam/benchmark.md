# Phase 5, benchmark: how SLAM is judged

> **Status (2026-10-11): the metrics and the file format are fixed here, the tolerances are in
> `scripts/phase5_verdict.py`, and the first evidence runs have been made (section 6 is their protocol; the results are in `evidence/phase-5-slam/NOTE-first-evidence-runs.md` and `benchmarks/phase-5-slam.csv`).** `benchmarks/phase-5-slam.csv` is not
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
| **Live Webots** | `/ground_truth/pose` | as above | the nodes against a running driver (`source:=live`): one run, section 7.1; S4 and S5 are not graded on it. |

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

## 6. The first evidence runs: the protocol, fixed before any of them was made

Written on 2026-10-11, after the harness and `scripts/phase5_verdict.py` and before the first run
of either harness into `evidence/phase-5-slam/`. Where a number is a choice it is a choice made
here, in advance. What the runs then showed is in `implementation.md` section 8.5, not here.

**The runs.** Everything is run by `scripts/run_phase5_slam.sh`, which refuses to overwrite a run.

| Run | Data | Route | Role | Seed |
|---|---|---|---|---|
| `synthetic-1-dev-route-long` | Rusty, exact | `route_long.yaml`, 1 lap, turns at 0.8 rad/s | development | 1 |
| `synthetic-2-eval-cross` | Rusty, exact | `route_eval_cross.yaml`, 1 lap, 0.8 rad/s | evaluation | 2 |
| `synthetic-3-eval-diamond` | Rusty, exact | `route_eval_diamond.yaml`, 1 lap, **0.3 rad/s** (the rate Phase 4 drove it at) | evaluation | 3 |
| `synthetic-4-eval-rectangle` | Rusty, exact | `route_slam_eval_rectangle.yaml`, 3 laps, 0.8 rad/s | evaluation | 4 |
| `recorded-1-dev-route-long`, `recorded-2-eval-cross`, `recorded-3-eval-wall-lobes`, `recorded-4-eval-diamond` | the four Phase 4 recordings of those routes, read only | see below | as named | 0 |

The routes were chosen by `scripts/phase5_route_revisits.py` (implementation.md section 9, item 1), not by any
run. Only the development run chooses anything, and the only thing it is allowed to choose is
`info_inflation` (below). A bug a run exposes is fixed and said so; a parameter is not retuned to move a number.

**S1a.** Every consecutive pair of scans is registered by point-to-plane ICP from **zero motion**
with a 0.3 m correspondence threshold; a pair is *graded* when the registration settled (`IcpResult::settled`),
and graded over made is reported. The error is that of the estimated relative pose against the ground-truth
relative pose. No registration is retried or hand-picked.

**S1a on Webots.** Four recordings are read, none changed: the development run `2026-10-11-development-3-route-long`
(role development) and the three evaluation recordings of Phase 4 (cross, wall lobes, diamond; role
evaluation). A pair is two consecutive lines of `integrated-scans.csv` whose stamps differ by at most 0.5 s. The
poses in the file are the mapper's ground truth, already gated by its motion filter (`implementation.md`
section 8.2 of this phase says what that does and does not give). The run grades against the `webots_gated`
tolerances of the verdict script. These runs have no pipeline sections: the file has no turning scans.

**S1b.** One scan pair of the run (scans `n/3` and `n/3 + 5`), 100 seeded trials per offset range, ranges
(0.05 m, 0.05 rad), (0.10 m, 0.10 rad), (0.20 m, 0.15 rad), (0.30 m, 0.25 rad): a translation drawn uniformly over
the disc and a rotation uniformly over the interval, added to the ground truth as the initial guess. **Success**
is: the registration settled and ended within 0.02 m and 0.01 rad of the ground truth.

**S2 and the freezing of `info_inflation`.** The mean NEES of the development run's S1a registrations is
measured with `info_inflation = 1.0`. The rule, fixed now: if it lies above the upper end of the 95 per cent
interval of the chi-squared(3) mean for that many samples (OVERCONFIDENT), `info_inflation` becomes
`mean NEES / 3` rounded to two significant digits; if it is inside the interval or below it, `info_inflation`
stays 1.0 (a conservative matrix is not made more confident). The value is then written into the defaults of
`IcpConfig`, the front end and the loop verifier, and **the three evaluation runs use it unchanged**. S2 is then
reported on every run as measured, and a value that does not hold on the evaluation routes is a finding.

**S4 and S5.** On each synthetic run the graph is built from LiDAR-odometry edges whose information is the compounded
ICP information divided by the frozen `info_inflation` (a fixed placeholder information was used in the first
smoke of the harness and is not used here). The primary runs carry **no** injected error: the drift is the
scan matcher's own. Because natural drift in this room may be small enough that loops change little, each of the four
routes is also run once with `--yaw-bias 0.004` (rad per keyframe, a systematic yaw error injected into the edges
the graph is handed, declared in the file and in the verdict), named `...-injected`; those runs are model-level
evidence for the loop machinery and say nothing about the scan matcher's drift. Seeds are those of the table.

**S6.** The recorded `scans.csv` is the input; the pipeline is run twice and compared bit for bit, with the S1a
pass run a second time and compared too. Wall times are of the release build in the WSL mirror, on the machine
written in `environment.txt`.

**S7.** Both back ends run on every synthetic run, fed the ground-truth increments of consecutive scans with declared
noise: body-frame standard deviations `0.02 d + 0.001` (along), `0.01 d + 0.001` (across), `0.03 |phi| + 0.005 d + 0.002`
(heading) for an increment of translation `d` and rotation `phi`, plus a systematic yaw error of 0.005 rad per
metre. These numbers are chosen, not measured, and are not tuned to either back end. EKF-SLAM also gets the pillar
detections of `robotics_localization::pillars` on the same scans, with the filter's default noise. The RBPF has 30
particles and its default parameters. No ranking is claimed.

### 6.1 What differs from the protocol above, and why (written after the runs)

Nothing above was edited after the runs. These are the departures, each visible in the evidence:

* **Run names.** The graded set is `v2-*`, not the names of the table: the first batch (`batch-1-before-front-end-fix/`)
  exposed a front-end bug on `route_eval_cross.yaml` (implementation.md 8.3 item 10), the fix changes the front end,
  and every run that used it was repeated with the fixed code. The first batch's S1a and S2 do not depend on the fix and
  are where `info_inflation` was measured; its S4 to S7 are superseded.
* **Injected runs.** The protocol said the injected variants would run on the graph's usual edge information. They did,
  in the first batch, and every loop was rejected (the compounded ICP information knows nothing of an injected systematic
  error, so the gate rightly refused). The injected variants of the graded set hand the graph the information the injected
  error warrants (3 cm and 0.01 rad per edge, as in `pose_graph_it.rs`). The natural runs are unchanged.
* **S1b.** Two offset ranges were added, (0.5 m, 0.4 rad) and (1.0 m, 0.8 rad), after the development run showed 100 of
  100 at the largest pre-registered range. They are reported, never graded.
* **S1a on Webots.** The protocol said the Webots runs have no pipeline sections; the verdict script's schema was extended,
  before the runs, to allow that. Its tolerances were not touched; `scripts/phase5_verdict.py --print-tolerances` prints the
  same constants before and after.
* **Held to the protocol.** S2 on the evaluation runs was reported as measured and not used to change `info_inflation`.

## 7. Live-recording S4 and S5 (Stage C of the closure): the protocol, fixed before the runs

Written on 2026-10-14, after the two recordings existed and were confirmed to contain turning scans
(`evidence/phase-5-slam/2026-10-14-webots-rectangle-recording*/recording-turns.txt`: 13 turns, 128 turning scans of
about 886, median turn rate 0.800 rad/s) and **before** the harness was run on either, and before any S4 or S5 number
for Webots data existed. Nothing in section 6 or in `scripts/phase5_verdict.py` is changed by this section; a
tolerance that has to move to make a Webots result pass is a finding to report, not a number to edit.

**The data.** Two recordings of `route_slam_eval_rectangle.yaml`, three laps, driven by `route_driver` at its default
0.8 rad/s in Webots R2025a with the official driver (`scripts/run_phase5_record.sh`). The mapper node that recorded
them had its motion gate and motion filter off, so every scan the driver published is in the file, with the pose
columns of the Phase 4 schema (the ground truth at `stamp - 0.06 s`). The second recording also has the dense 50 Hz
ground-truth track (the first lost it to a harness bug and is kept as it is). **Both recordings are graded, as role
`evaluation`, and both are reported; neither is chosen over the other.** The only thing that distinguishes them is the
physics of one more Webots run.

**What is graded, with which flags.**

* `slam_recorded_run --pipeline --emulate-motion-gate`: S1a over the pairs the Phase 4 mapper's gate would have
  admitted (the rule that makes a Webots S1a mean what it meant in section 6); S2 on the same registrations; S4 and S5
  on every scan of the stream with the same `pipeline` function the synthetic runs use, so that the numbers are
  comparable row by row; S6 with the pipeline run twice. `info_inflation` is the frozen 1.5, unchanged.
* `phase5_verdict.py` as it is. The data are `webots_recorded` and `webots_gated`, so the loose `webots_gated`
  tolerances apply (graph ATE RMS 0.10 m, final error 0.20 m); the S5 safety count is zero wrong constraints, the
  same limits (0.05 m, 0.03 rad) as everywhere.

**The one thing about these data that is not like the synthetic ones, and how it is measured instead of assumed.** The
recorded pose of a scan is the ground truth at `stamp - 0.06 s`, a mean lag. The lidar measures on a 0.2 s lattice
and its stamp lags the measurement by -0.02 to 0.18 s, so at 0.8 rad/s the recorded heading of a scan taken in a turn
is off by up to about 0.1 rad: more than S5's 0.03 rad. A constraint that is right could be labelled wrong, and a wrong
one right, for that reason alone, and S4 compares positions against the same poses. This is checked before anything
is concluded from S4 or S5:

1. `slam_recorded_run --turning-report` registers the pairs the gate would have refused and writes how far the
   registration is from the recorded relative pose there, outside the metrics file, not graded. The registration
   reads no ground truth, so on turning pairs the disagreement is the recorded pose's error.
2. `scripts/phase5_gt_instants.py` estimates the lattice phase from the data, as the phase at which the ground truth at
   the lattice instants agrees best with the registration's own rotation over the turning pairs, and compares it with
   the probe's 0.0603 s. It writes a corrected copy of each recording (the same scans, the pose columns replaced by the
   dense ground truth at those instants); the original is never touched. The second recording only, which has the
   dense track.
3. **Rule, fixed here:** if the recorded poses' rotation error over the turning pairs (their p95) exceeds S5's 0.03 rad,
   the corrected recording is graded as well, with the identical flags, and **both** results are reported next to each
   other and the text says which of them the claim rests on and why. If it does not exceed 0.03 rad, the corrected copy
   is reported as a measurement of the lag and not graded. In neither case is one result dropped because it looks
   worse.

The corrected recording is `ground_truth: webots_gated` in the verdict only because the schema has two values; the
stamp-quantization that the name stands for is what it removes. This is said wherever its numbers are used.

**What is compared, and what "agrees" means.** Next to the Webots numbers, in one table: `v2-synthetic-4-eval-rectangle`
(natural drift, the same route driven in Rusty) and `v2-synthetic-4-eval-rectangle-injected`. The synthetic prediction
being tested (implementation.md 8.5) is: the scan matcher's natural drift is small (about 3 mm ATE RMS in Rusty), so
the graph is not better than the chain and S4's pre-registered ratio check fails on natural runs while every absolute
check passes; S5 finds no wrong constraint among the loop candidates, which are almost all true loops. A Webots result
**agrees** with that if the chain's drift is again small and the graph is not better than it; it **disagrees** if the
front end drifts enough that loop closure visibly reduces the error (real beam-angle bias, lidar noise and slip make
that possible) or if any constraint is wrong. Both are reported as they come out. S8 stays unclaimable whatever they are.

### 7.1 Results of Stage C and the first live run (2026-10-15, written after the runs)

Nothing in section 7 above was edited. The protocol was followed with the flags it names, in the order it names.
`scripts/phase5_verdict.py` is byte-identical for every run in the table (sha256 `579b561e...`, in each run's
`environment.txt`), so no tolerance moved. `info_inflation` is the frozen 1.5.

**Runs.** `evidence/phase-5-slam/v2-recorded-5a-eval-rectangle-recording`, `...-5b-eval-rectangle-recording-2` (the two
recordings as recorded) and `...-5c-eval-rectangle-recording-2-corrected` (rule 3 of section 7: the recording whose
pose columns come from `scripts/phase5_gt_instants.py`, made because the turning-pair rotation error of the recorded
poses is above 0.03 rad, see below). The synthetic columns are `v2-synthetic-4-eval-rectangle` and
`v2-synthetic-4c-eval-rectangle-injected`, the same route driven in Rusty.

| | Rusty natural | Rusty injected (0.004 rad/kf) | Webots 5a (rec. 1) | Webots 5b (rec. 2) | Webots 5c (rec. 2, corrected) |
|---|---|---|---|---|---|
| S4 graph ATE RMS (limit 0.05 / **0.10** m on Webots) | 0.0090 m | 0.0098 m | 0.0136 m | 0.0157 m | 0.0149 m |
| S4 graph final error (0.1 / **0.2** m) | 0.0024 m | 0.0001 m | 0.0255 m | 0.0222 m | 0.0218 m |
| S4 odometry (chain) ATE RMS | 0.0035 m | 0.2425 m (injected) | 0.0228 m | 0.0217 m | 0.0210 m |
| S4 graph / chain ATE ratio (limit 1.05) | **2.53 (fails)** | 0.040 | 0.60 | 0.72 | 0.71 |
| S4 chain drift (limit 0.01 m/m) | 0.00008 | 0.0163 (not graded) | 0.00023 | 0.00124 | 0.00123 |
| **S4 verdict** | **FAIL** (ratio only) | PASS | **PASS** | **PASS** | **PASS** |
| S5 candidates / accepted | 182 / 181 | 185 / 185 | 182 / 147 | 180 / 157 | 180 / 157 |
| S5 accepted wrong (limit 0) | 0 | 0 | **54** | **63** | **0** |
| **S5 verdict** | PASS | PASS | **FAIL** | **FAIL** | **PASS** |
| S1a (gated pairs) | PASS | PASS | PASS | **FAIL** (`rotation_max` 0.0633 rad > 0.03) | PASS |

**Where Webots agrees and disagrees with the Rusty prediction (implementation.md 8.5).**

* **S4 disagrees.** The prediction was that the scan matcher barely drifts, so the graph cannot beat the chain and the
  ratio check fails on a natural run. In Webots the chain's own ATE RMS is 2.1 to 2.3 cm, six times Rusty's 0.35 cm,
  and loop closure brings it to 1.4 to 1.6 cm: the ratio is 0.60 to 0.72 and the check **passes**. Every absolute
  check passes in all three. Why the chain drifts more here was not tested (lidar noise, the beam layout and wheel slip are candidates). This is the opposite of the synthetic finding, and it is the Webots number that the claim rests on.
* **S5 disagrees on the recordings as recorded, and agrees on the corrected one.** With the recorded poses, 54 and 63
  accepted constraints are labelled wrong (error against the recorded relative pose above 0.05 m or 0.03 rad):
  S5 fails twice. With the corrected poses, the same recording has 0 wrong of 157. The verifier made the same
  decisions in both: candidates 180 and accepted 157 in 5b and 5c, only the labels differ.

**Why the labels differ, measured, and what that does not prove.** `--turning-report` registers the 128 pairs the motion
gate refuses and compares the registration, which reads no ground truth, with the recorded relative pose: rotation
error median 0.016 rad, **p95 0.140 rad (rec. 1) and 0.137 rad (rec. 2)**, against S5's 0.03 rad: the recorded pose
of a scan taken in a turn is wrong by more than the claim's tolerance (translation p95 0.7 to 0.9 cm), so a correct
loop constraint can be labelled wrong. `scripts/phase5_gt_instants.py` estimated the lidar's lattice phase from the
dense ground-truth track of recording 2 (`evidence/phase-5-slam/2026-10-15-gt-instants-recording-2/`): the best phase
is **0.198 s**, with the turning-pair rotation disagreement at rms 0.0035 rad, against 0.041 rad for the recorded
poses and 0.057 rad at the 0.0603 s phase of the Phase 4 probe. That the probe's phase and this one differ is **not
explained** (the probe was one run; the two are not the same number modulo the 0.2 s period). Against the corrected
poses the turning-pair translation error is p95 0.5 cm and S1a's translation p95 on gated pairs falls from 3.2 cm to
0.25 cm. Limits of this: (1) the phase was selected by agreement with the scan matcher's own rotations, so a
rotation bias common to the matcher and the labels would be absorbed, not exposed; the translation agreement
improved too and was not part of the fit, which supports the correction but does not prove it; (2) only recording 2
has the dense track, so **recording 1 cannot be corrected and its S5 failure stands as measured**; (3) rule 3 of
section 7 says which result a claim rests on, and this section says it plainly: S4 rests on all three runs, S5's
zero-wrong count holds only on the corrected recording 2 (0 of 157, which bounds the wrong-accept rate below about 1.9 per
cent of accepted at 95 per cent, in this world), and fails on both recordings as recorded.

**First live run of `source:=live` (`evidence/phase-5-slam/2026-10-15-live-source-live-1`).** Webots R2025a with the
official driver, the nodes from `phase5_slam.launch.py source:=live loop_closure:=true`, `scripts/run_phase5_live.sh`,
the same route and three laps; no replayer and no `ground_truth_tf_adapter` ran (`node_list.txt`). `scan_matcher_node`
received 880 scans and made 884 registrations in the capture file, `pose_graph_node` made 83 keyframes, 182 loop
candidates of which 154 were accepted, and published `slam/pose`, `slam/path` (last path: one pose per keyframe),
`/map` (54 received by the probe, 240 by 240 cells of 0.05 m in frame `map`) and `map -> odom` (882 samples seen).
`pose_graph_node` was the only publisher of `map -> odom`. The map the node wrote at its end equals a render from
files alone (final poses and recorded scans) byte for byte. The Stage B checker (`phase5_stage_b_check.py`, written
for a replay) passes 20 of 25 on this directory and fails 5:
* three are the end of the run: the probe is stopped before the SLAM nodes (the run script's order), so it missed the
  last `/map` (54 of 55 published) and the last two `map -> odom` (882 of 884), and the node's own `map_odom_dropped`
  is 0; this is the explanation from the order of the shutdown and the counts, **not** re-verified by a run with the
  order changed;
* two compare `map -> odom` with an expectation that assumes stationary odometry (the checker says so in its source),
  which holds for a replay and not for a driving robot. What a live run can be held to is that `map -> odom` is the
  constant `odom` to `map` offset up to drift and loop closure: it stays within 0.12 m and 0.10 rad of (0.03 m, 0, 0),
  the offset the geometry gives (the map frame is the first keyframe's sensor pose, the sensor is 0.03 m behind
  `base_link`), with a median step of 4.5 mm and a largest of 213 mm at loop closures; the last sample is
  (0.035, -0.021, 0.018 rad).

**S2 (reported, never graded).** Mean NEES of the gated registrations: 22.3 (5a) and 25.9 (5b), OVERCONFIDENT, and 1.10 (5c), below the
chi-squared(3) interval [2.8, 3.2], i.e. CONSERVATIVE. The overconfidence that set `info_inflation` = 1.5 on the Webots data
is therefore at least partly the ground truth's stamp error, not only the matrix; `info_inflation` was **not** touched and
no conclusion about its value is drawn from one corrected recording.

A rough check outside the protocol, not graded: the 83 keyframes of the live run against the ground-truth sensor poses
recorded with its scans (same lag caveat as above) have translation error rms 1.6 cm, max 4.1 cm, final 1.1 cm. S4 and S5
were **not graded on the live run itself**: its `integrated-scans.csv` has the same stamp-lag problem as the recordings, and
a corrected copy needs the dense track and a lattice estimate that were not made for it.

### 7.2 The lattice-phase discrepancy explained, and the protocol of the fresh live run (written 2026-10-15, before that run)

Written after the existing data were re-analysed and **before** the fresh run it ends with was made. Nothing in 7 or
7.1 is edited; where this section disagrees with 7.1 it is a correction of 7.1 and says so.

**What the two numbers are.** Both are the phase of the Webots lidar's 0.2 s measurement lattice, in simulation time, on the
clock of the `/ground_truth/pose` stamps (modulo 0.2 s; a phase of 0.198 s is -0.002 s). Neither is wall-clock and neither
includes queueing or dispatch: both are read from message stamps.

* **0.0603 s** (`evidence/phase-4-mapping/2026-10-10-scan-skew-probe`, `phase4_scan_skew_analysis.py`): for each turning scan, fit the
  constant offset `d` between the stamp and the instant the beams were measured, by matching the 360 ranges to the exact
  room seen from the dense ground-truth track; take the phase of `stamp + d` modulo 0.2 s. One run, 53 turning scans, the
  probe's own recorder driving the robot.
* **0.198 s** (`phase5_gt_instants.py`, `2026-10-15-gt-instants-recording-2`): choose the phase (and the edge of the lag window) at
  which the dense track at the lattice instants best reproduces **the scan matcher's rotation between consecutive scans**,
  over turning pairs. A different quantity from a different run (recording 2 of 2026-10-14), estimated from SLAM output.

**What the discrepancy was.** I ran the probe's own per-scan fit (its `fit_scan`, imported unchanged) on the turning scans of
recording 2 and of the first live run, which use ranges and the exact room and read no SLAM output. Result:

| run | method | phase of the measurement lattice | resultant length R | turning scans |
|---|---|---|---|---|
| probe, 2026-10-10 | per-scan range fit | **0.0603 s** | 0.995 | 53 |
| recording 2, 2026-10-14 | per-scan range fit | **0.0016 s** (first third 0.0017, last third 0.0015) | 0.991 | 111 |
| recording 2, 2026-10-14 | matcher agreement (`phase5_gt_instants.py`) | 0.198 s = **-0.002 s** | | 122 pairs |
| first live run, 2026-10-15 | per-scan range fit | **0.0017 s** (first third 0.0017, last third 0.0012) | 0.973 | 110 |

(`scripts/phase5_lattice_from_ranges.py` reproduces the range-fit rows with its own, stricter protocol: 0.0011 s and 0.0012 s on the
calibration turns, 0.0018 s on the others, for both runs.) So:

1. **The two measurements did not disagree about one thing.** 0.0603 s is the phase of the probe's run, and 0.198 s the phase of recording 2;
   the same method (range fit) on recording 2 gives 0.0016 s, 3 ms from the matcher-based 0.198 s (-0.002 s), inside the width
   of the minimum, which is flat to about 10 ms (`phase5_gt_instants.py`'s report: rotation disagreement 0.0035 rad at 0.198, 0.0075 at 0.02).
   Within a run the phase is stable to a few milliseconds (sd 4 ms over three laps; first and last third differ by 0.2 to 0.5 ms).
2. **7.1 was wrong to treat 0.0603 s as a reference the recording should agree with.** It is the phase of one run, and
   `docs/phase-4-mapping/implementation.md` 5.2 item 2 (and a note of mine that quoted "sim time 0.0603 + 0.2k") stated it as if it were a property of
   the driver. The lag distribution *is* a property of the driver and agrees across all three runs (mean 0.061, 0.0645 and 0.071 s; range about
   -0.02 to 0.17 s, apart from one outlier fit at -0.115 s in the live run); the phase of the lattice against the ground-truth clock is not. **Why the probe's run differs from the two later ones is not known.** The world, the
   launch file and the supervisor are unchanged between them (same `world_sha256`; no commit touches `sim/webots` between the probe and the
   recordings). A candidate, **untested**: the lattice starts when the lidar is enabled, which happens when the extern controller connects,
   and the number of 20 ms steps simulated before that can differ from run to run (0.0603 is three steps, 0.0016 is none). The fresh run adds one
   more value; three runs agreeing or not would not settle the mechanism, and it is not claimed.
3. **Neither measurement was a units, clock or dispatch mismatch**, which is what a factor of three suggests. The 3x is `0.0603 / 0.198`,
   and the numbers are on a circle (0.198 is 0.002 short of the period), so a ratio of them means nothing; the distance on the circle is 0.062 s.

**Was the correction applied in 7.1 principled?** In kind, yes; in how it was fitted and what it was compared with, it was not safe, and 7.1 did not
earn the right to call it a result.

* The stamp-lag error in the pose labels is real and independent of any fit: with the recorded labels (the track at `stamp - 0.06 s`), a
  turning scan is explained by the exact room worse than a straight one (range residual median 63 mm, p95 76 mm, against 15 mm and 20 mm,
  recording 2, `phase5_lattice_from_ranges.py`), and with the labels at the lattice instants it is not (16 and 22 mm on turns not used to estimate the phase).
  That is a measurement of the labels against the room, not against the matcher.
* The phase the correction used was fitted on the matcher's own rotations, so a rotation bias shared by matcher and labels would have been
  absorbed. It was not (the range fit agrees to 3 ms and reads nothing from SLAM), but 7.1 could not know that, and its comparison with 0.0603 s,
  which looked like a disagreement, was a comparison between runs.
* **The correction did not change what the verifier decided, and that is as it should be, not a flaw.** Loop acceptance reads scans and relative
  poses from odometry and never the ground truth (the same 157 of 180 accepted in 5b and 5c). Only the labels, which are the ground truth the
  claim is graded against, changed. A relabelling that moved the verifier's decisions would be the suspicious one. What does remain true is that S5's
  "zero wrong constraints" on recording 2 was graded against labels that had been chosen after the fact, on the one recording that could be relabelled.

**Protocol of the fresh run.** One new live run of `source:=live loop_closure:=true` (`scripts/run_phase5_live.sh`, the rectangle route, three laps,
with the dense track, the scans and the ground truth recorded by the harness) named `2026-10-15-live-source-live-2`. Graded exactly as in section 7 (the
flags of `slam_recorded_run`, `phase5_verdict.py` unchanged, `info_inflation` 1.5), on **two** copies of the same recording, both reported:

* **R, recorded labels:** `integrated-scans.csv` as the harness wrote it (the pose columns are the track at `stamp - 0.06 s`).
* **L, lattice labels:** a copy whose pose columns are the dense track at the measurement instants, from `scripts/phase5_lattice_from_ranges.py`: phase and lag
  window **estimated on the first three turns of the run only, from the scans' ranges against the exact room, reading no SLAM output**, and
  **validated on the other ten**, with the thresholds in that script's docstring (phase within 5 ms, R at least 0.95, 95 per cent of lags in the window). **If
  the validation fails the L copy is not graded**, and the report says so. No threshold is changed after the run.

Which of R and L a claim rests on, said now: **L**, because the S5 definition grades a constraint against the sensor's true relative pose and R's label of a scan taken in a turn
is wrong by more than S5's own tolerance (7.1: 0.14 rad p95; the range residual above is the independent evidence). R is reported because S5 on R is what the original
protocol graded, and it is expected to fail for the label's sake, not the verifier's: **if R fails and L passes, the text will say exactly that and will not call it a clean pass of R.** If
L fails, the number is the result. The run's S5 on the **live nodes' own** accepted loops (`node/loops.csv`) is compared with the offline replay of the same scans, to check that
the recorded-stream grading is of what the nodes did.

**Stage B recheck.** `scripts/phase5_stage_b_check.py` is rerun on live-1 unchanged, and on live-2 whose shutdown order was changed (`run_phase5_live.sh`: probe stops after the SLAM nodes),
which tests the explanation of the three end-of-run failures by removing the cause.

### 7.3 Results of the fresh live run (2026-10-15, written after the run; the protocol is 7.2, unchanged)

`evidence/phase-5-slam/2026-10-15-live-source-live-2` (`scripts/run_phase5_live.sh`, rectangle route, three laps, 894 scans, 13 turns, dense track,
clean shutdown, `webots_bin_left: 0`), graded offline on two copies of its recording as 7.2 fixed:
`v2-recorded-6a-live-2-recorded-labels` (R) and `v2-recorded-6b-live-2-lattice-labels` (L; the labels and their validation are in
`2026-10-15-live-2-labels/`). `scripts/phase5_verdict.py` is the same file as in 7.1 (sha256 `579b561e...`); `info_inflation` 1.5.

**The lattice estimate and its validation (it passed; no threshold was changed).** Phase 0.0014 s from the first three turns (R = 0.9925, sd 3.9 ms), 0.0018 s on the
other ten (0.4 ms apart, R = 0.9903), 1 of 83 validation lags outside the calibration window. Against the exact room, the range residual of the turning scans not
used for the estimate is median 53 mm (p95 75 mm) with the recorded labels and 17 mm (p95 22 mm) with the lattice labels; straight scans: 14 and 13 mm.
**So the label of a scan taken in a turn is wrong on the recorded pose and not on the lattice one, by a measurement that reads no SLAM output.**

| | R: recorded labels | L: lattice labels |
|---|---|---|
| S5 candidates / accepted (offline replay) | 182 / 164 | 182 / 164 |
| **S5 accepted wrong (limit 0)** | **54** | **0** |
| **S5 verdict** | **FAIL** | **PASS** |
| S4 graph ATE RMS / chain ATE RMS (ratio, limit 1.05) | 0.0156 / 0.0266 m (0.59) | 0.0132 / 0.0242 m (0.54) |
| S4 verdict | PASS (the verdict file also notes that S5 failed) | PASS |
| S1a (gated pairs; translation p95) | FAIL (0.032 m) | PASS (0.0024 m) |
| S5 on the **live nodes' own** accepted loops (`node/loops.csv`, 181 candidates, 156 accepted), `scripts/phase5_live_loops_label.py` | 50 wrong | **0 wrong** (worst accepted error 25.8 mm and 0.0142 rad, limits 0.05 and 0.03) |

The verifier decided the same thing in R and L (identical counts). The offline replay of the recorded stream and the live nodes are not the same run of the
algorithm (182 candidates / 164 accepted against 181 / 156; the first live run had 182 / 154): both are graded and both pass on L.

**What this does and does not establish.**

* **It is not a clean pass of the criterion as it was first written.** S5 graded on the labels the protocol originally used (R) fails, as it did on both earlier recordings, with 54 wrong; on the live
  nodes' own loops 50. What passes is S5 graded on labels that are the ground truth at the instant the scan was measured (L). The case that R's failure is the label's and not the verifier's rests
  on the independent range residual above and on the verifier's decisions not depending on labels; it does not rest on S5 having passed. Whether that makes S5 met is the owner's call, as it was in 7.1.
* **The correction is no longer fitted to the outcome.** Phase and window come from ranges, from the first three turns, with a held-out validation fixed in advance. That removes the circularity of 7.1; it does not make L the same thing as R.
* **The mechanism of the phase is still unknown.** Four runs: the Phase 4 probe 0.0603 s; recording 2, the first live run and this one 0.001 to 0.002 s. The three Phase 5 runs agree to under a millisecond (0.0011, 0.0012 and 0.0014 s on their calibration turns), so the lattice phase was **not**
  random across them; why the probe's run differs is untested (the "enabled three steps later" candidate of 7.2 stays a candidate). A fifth run with a different startup could show a different phase, and the validation exists for that case.
* **The pass is about the candidates this run produced** (164 accepted, 18 rejected, all for `gate_exceeded`; 0 wrong in 164 supports a wrong-accept rate below about 1.8 per cent at 95 per cent in this world). The loop candidates designed to be rejected are in the tests.
* S8 stays not claimable. Phase 5 is **not** marked closed by this section.

**Stage B recheck (the five failures of live-1).** `phase5_stage_b_check.py` rerun unchanged on live-1: **the same five fail (20 of 25)**. On live-2, whose probe stops after the SLAM nodes: **22 of 25, three fail.**
* The two `/map` count failures (54 received of 55) are **gone**: the end-of-run explanation was right for them. The probe now receives the last `/map`.
* `map -> odom` for every registered scan, and "none dropped": these two still fail in live-2, **with a different cause**. The probe now receives every message the node published (881 of 881), but the node published 881 for 882 registrations and counts one dropped:
  the very first scan (stamp 7.08 s), whose `odom -> base_link` lookup at `stamp - 0.06 s` falls before the start of the TF history (`last_error: lookup ... outside its history [7.06, 7.56]`). That is a startup race of the node, counted in its
  diagnostics, not harness noise; in live-1 it did not happen (dropped 0), so it depends on the run. Live-1's "882 of 884" was the probe's shutdown order (the node had published all 884), as explained; live-2's drop is a second, different finding and is not fixed.
* The last `map -> odom` against the recomputed estimate fails in both and **is the checker's assumption** (stationary odometry, as its own source says): live-2's last sample is (0.022, -0.007, 0.008 rad), the constant offset the geometry gives (0.03, 0, 0) to within 0.01 m and 0.01 rad, while the check expects the sensor's SLAM pose (1.14, -0.91).
* S1a on R fails on `rotation_max` (0.049 rad against 0.03) only; on L it passes.
