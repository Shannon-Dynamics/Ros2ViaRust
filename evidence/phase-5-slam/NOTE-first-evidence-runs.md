# Phase 5: what the first evidence runs show

Written 2026-10-11 from the run directories next to this file. Protocol: `docs/phase-5-slam/benchmark.md`
section 6, written before the first run. Verdicts: `phase5-verdict.yaml` in each run directory, from
`scripts/phase5_verdict.py`, whose tolerances were written before any of these runs and were **not changed
afterwards** (its schema was extended once, before the runs, to say what a Webots recording may leave out).
The numbers below are generated from the metrics files by `scripts/phase5_evidence_tables.py`, which also
checks this file (`--check`); they are not typed.

The verdict script that graded the runs is the working-tree version whose SHA-256 is in each run's `environment.txt`
(and `phase5-verdict.yaml`). Against the committed script (`2b04005`) it differs in its schema and in the graders that
handle absent sections and S7; `python3 -I scripts/phase5_verdict.py --print-tolerances` prints the same constants for both,
and no line between its `SCHEMA` and its statistics section changed.

**S8 is not claimable and nothing here says otherwise.** No node exists, nothing here read or wrote a
`/map`, and the pipeline has no dependency on `robotics_mapping`.

## What was run, and what superseded what

| Directory | What |
|---|---|
| `v2-synthetic-{1,2,3,4}-*` | **the graded set**: the development run (`route_long.yaml`) and three evaluation runs (`route_eval_cross.yaml`, `route_eval_diamond.yaml` at 0.3 rad/s, `route_slam_eval_rectangle.yaml`, three laps), natural drift, Rusty with exact ground truth |
| `v2-synthetic-{1c,2c,3c,4c}-*-injected` | the same four routes with a yaw error of 0.004 rad per keyframe injected into the edges the graph is handed, and the edge information that error warrants (3 cm and 0.01 rad). Model level: evidence for the loop machinery, none for the scan matcher's drift |
| `v2-recorded-{1,2,3,4}-*` | the Webots half of S1a (and S2, S6) on the four Phase 4 recordings, read only |
| `batch-1-before-front-end-fix/` | the first batch, kept because it is evidence of two things (below). Its S1a and S2 are valid and are where `info_inflation` came from; its S4 to S7 were produced by a front end with a bug and are superseded |
| `recorded-gt-step-check.txt` | `scripts/phase5_recorded_gt_steps.py` on the four recordings, the check behind the Webots caveat |

**A bug the runs found, and its fix.** The first batch crashed on `route_eval_cross.yaml` (twice, natural and
injected: see `batch-1-before-front-end-fix/synthetic-2-eval-cross/run.log`). The front end tracks a long
straight to millimetres, which shrinks its adaptive correspondence threshold to a few centimetres; the first
turning scan then needed several times that, ICP crawled and hit its iteration limit, and a failed
registration did not feed back into the threshold, so no later step could do better and the pose diverged
by metres. `route_long.yaml` begins with a turn and hid it. The fix (`odometry.rs`): a registration that does
not settle at the adaptive threshold is tried once more at the no-knowledge threshold, from the same guess. A regression test reproduces
the failure on a straight-then-turn route and fails without the fix (`front_end_it.rs`,
`a_sharp_turn_after_a_long_straight_is_tracked`). Nothing else in the core was changed; no parameter was
retuned. Every run below used the fixed front end, and none lost a registration to prediction.

**Four things changed between the batches, and why.** The front-end fix (above). `info_inflation`: frozen at
1.5 by the rule written in advance, from the mean NEES of `batch-1-before-front-end-fix/synthetic-1-dev-route-long`
(1.0 in force, 1828 registrations; the front-end fix does not touch registrations of scan pairs, so it
stands). The two largest offset ranges of S1b (0.5 m / 0.4 rad and 1.0 m / 0.8 rad) were added after the
first run showed 100 of 100 at the largest pre-registered range, to find where the basin ends; they are
reported and never graded. And the injected variants: the first batch handed the graph the compounded ICP
information, which knows nothing of an injected systematic error, and every loop was rejected in all
three injected runs that completed (almost all by the gate, a few as `not_converged` or `degenerate`), as a graph
that is overconfident by construction should. That tests
nothing about the loop machinery, so the injected variants now declare the information their error warrants.

## The results

<!-- BEGIN GENERATED TABLES (scripts/phase5_evidence_tables.py) -->
**S1a, synthetic (Rusty, exact ground truth)**

| run | made | graded | translation error, median / p95 / max (mm) | rotation error, median / p95 / max (mrad) | verdict |
|---|---|---|---|---|---|
| synthetic-1-dev-route-long | 1828 | 1828 | 2.5 / 5.3 / 9.8 | 0.8 / 2.5 / 4.2 | PASS |
| synthetic-2-eval-cross | 567 | 567 | 2.4 / 5.3 / 9.2 | 0.8 / 2.6 / 4.3 | PASS |
| synthetic-3-eval-diamond | 813 | 813 | 2.4 / 5.4 / 8.7 | 0.9 / 2.4 / 4.4 | PASS |
| synthetic-4-eval-rectangle | 843 | 843 | 2.5 / 5.4 / 7.6 | 0.9 / 2.6 / 4.7 | PASS |

**S1a, Webots recordings (ground truth gated, timing-limited; see the note)**

| run | made | graded | translation error, median / p95 / max (mm) | rotation error, median / p95 / max (mrad) | verdict |
|---|---|---|---|---|---|
| recorded-1-dev-route-long | 1290 | 1290 | 3.3 / 31.6 / 34.7 | 0.6 / 2.0 / 4.2 | PASS |
| recorded-2-eval-cross | 445 | 445 | 2.6 / 31.9 / 34.9 | 0.6 / 1.9 / 3.0 | PASS |
| recorded-3-eval-wall-lobes | 441 | 441 | 3.3 / 30.9 / 35.1 | 0.7 / 2.4 / 3.8 | PASS |
| recorded-4-eval-diamond | 517 | 517 | 3.1 / 30.9 / 32.6 | 0.7 / 1.9 / 2.6 | PASS |

**S2** (`info_inflation` 1.5, frozen from the development measurement; reported, never pass or fail)

| run | samples | mean NEES | 95% interval of the chi-squared(3) mean | label |
|---|---|---|---|---|
| synthetic-1-dev-route-long | 1828 | 3.022 | [2.889, 3.113] | CONSISTENT |
| synthetic-2-eval-cross | 567 | 3.113 | [2.802, 3.205] | CONSISTENT |
| synthetic-3-eval-diamond | 813 | 3.201 | [2.834, 3.171] | OVERCONFIDENT |
| synthetic-4-eval-rectangle | 843 | 3.142 | [2.837, 3.168] | CONSISTENT |
| recorded-1-dev-route-long | 1290 | 28.846 | [2.868, 3.135] | OVERCONFIDENT |
| recorded-2-eval-cross | 445 | 23.497 | [2.777, 3.232] | OVERCONFIDENT |
| recorded-3-eval-wall-lobes | 441 | 21.508 | [2.776, 3.233] | OVERCONFIDENT |
| recorded-4-eval-diamond | 517 | 22.855 | [2.793, 3.215] | OVERCONFIDENT |

**S1b** (model level; one scan pair per run; success = settled and within 0.02 m and 0.01 rad)

| run | initial offset (up to) | successes | Wilson 95% |  |
|---|---|---|---|---|
| synthetic-1-dev-route-long | 0.05 m / 0.05 rad | 100/100 | [0.963, 1.000] | graded |
| synthetic-1-dev-route-long | 0.1 m / 0.1 rad | 100/100 | [0.963, 1.000] | graded |
| synthetic-1-dev-route-long | 0.2 m / 0.15 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-1-dev-route-long | 0.3 m / 0.25 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-1-dev-route-long | 0.5 m / 0.4 rad | 99/100 | [0.946, 0.998] | reported |
| synthetic-1-dev-route-long | 1 m / 0.8 rad | 44/100 | [0.347, 0.538] | reported |
| synthetic-2-eval-cross | 0.05 m / 0.05 rad | 100/100 | [0.963, 1.000] | graded |
| synthetic-2-eval-cross | 0.1 m / 0.1 rad | 100/100 | [0.963, 1.000] | graded |
| synthetic-2-eval-cross | 0.2 m / 0.15 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-2-eval-cross | 0.3 m / 0.25 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-2-eval-cross | 0.5 m / 0.4 rad | 99/100 | [0.946, 0.998] | reported |
| synthetic-2-eval-cross | 1 m / 0.8 rad | 41/100 | [0.319, 0.508] | reported |
| synthetic-3-eval-diamond | 0.05 m / 0.05 rad | 100/100 | [0.963, 1.000] | graded |
| synthetic-3-eval-diamond | 0.1 m / 0.1 rad | 100/100 | [0.963, 1.000] | graded |
| synthetic-3-eval-diamond | 0.2 m / 0.15 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-3-eval-diamond | 0.3 m / 0.25 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-3-eval-diamond | 0.5 m / 0.4 rad | 99/100 | [0.946, 0.998] | reported |
| synthetic-3-eval-diamond | 1 m / 0.8 rad | 40/100 | [0.309, 0.498] | reported |
| synthetic-4-eval-rectangle | 0.05 m / 0.05 rad | 100/100 | [0.963, 1.000] | graded |
| synthetic-4-eval-rectangle | 0.1 m / 0.1 rad | 100/100 | [0.963, 1.000] | graded |
| synthetic-4-eval-rectangle | 0.2 m / 0.15 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-4-eval-rectangle | 0.3 m / 0.25 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-4-eval-rectangle | 0.5 m / 0.4 rad | 100/100 | [0.963, 1.000] | reported |
| synthetic-4-eval-rectangle | 1 m / 0.8 rad | 40/100 | [0.309, 0.498] | reported |

**S4, natural drift (nothing injected)**

| run | keyframes | path (m) | front end alone: ATE rms / max / final (mm) | optimized graph: ATE rms / max / final (mm) | drift, front end / graph (m per m) | graph ATE over front-end ATE | verdict |
|---|---|---|---|---|---|---|---|
| synthetic-1-dev-route-long | 192 | 53.1 | 3.4 / 8.2 / 4.3 | 10.0 / 18.0 / 2.9 | 8.04e-05 / 5.41e-05 | 2.93 | FAIL |
| synthetic-2-eval-cross | 55 | 17.9 | 3.3 / 7.4 / 7.4 | 6.6 / 14.3 / 10.9 | 4.15e-04 / 6.12e-04 | 1.99 | FAIL |
| synthetic-3-eval-diamond | 70 | 20.6 | 3.3 / 8.3 / 1.7 | 8.5 / 15.6 / 2.0 | 8.31e-05 / 9.90e-05 | 2.60 | FAIL |
| synthetic-4-eval-rectangle | 84 | 25.4 | 3.5 / 6.2 / 2.0 | 9.0 / 17.4 / 2.4 | 8.00e-05 / 9.34e-05 | 2.53 | FAIL |

**S4, with a yaw error injected into the edges (model level: says nothing about the scan matcher's drift)**

| run | keyframes | path (m) | front end alone: ATE rms / max / final (mm) | optimized graph: ATE rms / max / final (mm) | drift, front end / graph (m per m) | graph ATE over front-end ATE | verdict |
|---|---|---|---|---|---|---|---|
| synthetic-1c-dev-route-long-injected | 192 | 53.1 | 592.6 / 1279.9 / 67.6 | 11.0 / 20.6 / 4.9 | 1.27e-03 / 9.14e-05 | 0.02 | PASS |
| synthetic-2c-eval-cross-injected | 55 | 17.9 | 229.9 / 608.9 / 608.9 | 7.1 / 15.9 / 10.2 | 3.41e-02 / 5.71e-04 | 0.03 | PASS |
| synthetic-3c-eval-diamond-injected | 70 | 20.6 | 245.5 / 444.5 / 370.0 | 8.4 / 14.9 / 3.4 | 1.80e-02 / 1.63e-04 | 0.03 | PASS |
| synthetic-4c-eval-rectangle-injected | 84 | 25.4 | 242.5 / 451.7 / 414.2 | 9.8 / 19.3 / 0.1 | 1.63e-02 / 3.25e-06 | 0.04 | PASS |

**S5** (every candidate the detector proposed, labelled by ground truth; wrong = measurement error over 0.05 m or 0.03 rad)

| run | candidates | accepted, correct | accepted, **wrong** | rejected, by reason | true loops accepted / proposed | keyframes with a true partner that got a loop edge | precision, Wilson 95% | rule of three (wrong-accept rate below) |
|---|---|---|---|---|---|---|---|---|
| synthetic-1-dev-route-long | 484 | 481 | 0 | gate_exceeded 2, ambiguous 1 | 480/483 | 161/161 | [0.9921, 1.0000] | 0.0062 |
| synthetic-2-eval-cross | 40 | 36 | 0 | ambiguous 4 | 33/37 | 14/14 | [0.9036, 1.0000] | 0.0833 |
| synthetic-3-eval-diamond | 113 | 113 | 0 | none | 112/112 | 41/41 | [0.9671, 1.0000] | 0.0265 |
| synthetic-4-eval-rectangle | 182 | 181 | 0 | gate_exceeded 1 | 181/182 | 62/62 | [0.9792, 1.0000] | 0.0166 |
| synthetic-1c-dev-route-long-injected | 486 | 485 | 0 | ambiguous 1 | 482/483 | 161/161 | [0.9921, 1.0000] | 0.0062 |
| synthetic-2c-eval-cross-injected | 48 | 43 | 0 | ambiguous 5 | 33/37 | 14/14 | [0.9180, 1.0000] | 0.0698 |
| synthetic-3c-eval-diamond-injected | 114 | 114 | 0 | none | 112/112 | 41/41 | [0.9674, 1.0000] | 0.0263 |
| synthetic-4c-eval-rectangle-injected | 185 | 185 | 0 | none | 182/182 | 62/62 | [0.9797, 1.0000] | 0.0162 |

**S6** (release build, WSL2 host in each run's `environment.txt`; registrations are scan-to-map steps for synthetic runs and pair registrations for recordings)

| run | replay | registration ms, median / p95 / p99 | optimizations: count, s median / p95 / p99 | S1a translation p95 of the same run (mm) | verdict |
|---|---|---|---|---|---|
| synthetic-1-dev-route-long | identical | 3.21 / 5.77 / 7.62 | 162: 0.193 / 0.355 / 0.418 | 5.3 | PASS |
| synthetic-1c-dev-route-long-injected | identical | 3.33 / 6.06 / 8.19 | 162: 0.190 / 0.379 / 0.449 | 5.3 | PASS |
| synthetic-2-eval-cross | identical | 2.95 / 5.50 / 7.21 | 16: 0.103 / 0.136 / 0.136 | 5.3 | PASS |
| synthetic-2c-eval-cross-injected | identical | 2.86 / 5.27 / 7.07 | 18: 0.093 / 0.134 / 0.134 | 5.3 | PASS |
| synthetic-3-eval-diamond | identical | 2.58 / 4.68 / 5.72 | 41: 0.136 / 0.165 / 0.170 | 5.4 | PASS |
| synthetic-3c-eval-diamond-injected | identical | 2.57 / 4.67 / 5.67 | 41: 0.138 / 0.166 / 0.208 | 5.4 | PASS |
| synthetic-4-eval-rectangle | identical | 3.37 / 6.25 / 8.32 | 62: 0.125 / 0.169 / 0.196 | 5.4 | PASS |
| synthetic-4c-eval-rectangle-injected | identical | 3.23 / 6.00 / 7.72 | 62: 0.134 / 0.161 / 0.181 | 5.4 | PASS |
| recorded-1-dev-route-long | identical | 0.70 / 1.16 / 5.55 | none | 31.6 | PASS |
| recorded-2-eval-cross | identical | 0.66 / 0.96 / 4.06 | none | 31.9 | PASS |
| recorded-3-eval-wall-lobes | identical | 0.56 / 1.04 / 4.60 | none | 30.9 | PASS |
| recorded-4-eval-diamond | identical | 0.73 / 1.10 / 5.73 | none | 30.9 | PASS |

**S7** (informative, no ranking: synthetic odometry with the declared noise and a systematic yaw error of 0.005 rad per metre)

| run | dead reckoning of that odometry: ATE rms / final (m) | EKF-SLAM: ATE rms / max (m) | EKF-SLAM mean pose NEES | share of steps within the 97.5% bound | EKF-SLAM landmarks | grid RBPF (30 particles): ATE rms / max (m) | RBPF resamples, mean N_eff before |
|---|---|---|---|---|---|---|---|
| synthetic-1-dev-route-long | 0.315 / 0.259 | 0.016 / 0.037 | 3.45 | 0.95 | 46 on 4 pillars; worst 2.41 m | 1.582 / 3.388 | 573, 17.3 |
| synthetic-1c-dev-route-long-injected | 0.315 / 0.259 | 0.016 / 0.037 | 3.45 | 0.95 | 46 on 4 pillars; worst 2.41 m | 1.582 / 3.388 | 573, 17.3 |
| synthetic-2-eval-cross | 0.162 / 0.223 | 0.024 / 0.048 | 6.54 | 0.74 | 44 on 4 pillars; worst 2.40 m | 0.288 / 0.624 | 142, 18.5 |
| synthetic-2c-eval-cross-injected | 0.162 / 0.223 | 0.024 / 0.048 | 6.54 | 0.74 | 44 on 4 pillars; worst 2.40 m | 0.288 / 0.624 | 142, 18.5 |
| synthetic-3-eval-diamond | 0.127 / 0.170 | 0.020 / 0.035 | 4.82 | 0.90 | 32 on 4 pillars; worst 2.30 m | 1.145 / 1.881 | 321, 15.6 |
| synthetic-3c-eval-diamond-injected | 0.127 / 0.170 | 0.020 / 0.035 | 4.82 | 0.90 | 32 on 4 pillars; worst 2.30 m | 1.145 / 1.881 | 321, 15.6 |
| synthetic-4-eval-rectangle | 0.122 / 0.339 | 0.013 / 0.029 | 2.50 | 0.99 | 27 on 4 pillars; worst 2.53 m | 2.421 / 3.727 | 248, 18.0 |
| synthetic-4c-eval-rectangle-injected | 0.122 / 0.339 | 0.013 / 0.029 | 2.50 | 0.99 | 27 on 4 pillars; worst 2.53 m | 2.421 / 3.727 | 248, 18.0 |

<!-- END GENERATED TABLES -->

## What each claim shows, and its scope

**S1a.** On exact ground truth the scan matcher's relative-pose error over every consecutive pair is millimetres
(translation p95 about 5 mm, worst about 10 mm; rotation p95 about 2.5 mrad) on all four routes, all registrations
settled. **The Webots half cannot say that.** Its translation p95 is about 31 mm on all four recordings and its
maximum about 35 mm, against 5 mm for the same matcher on synthetic scans; the rotation errors match the
synthetic ones. `recorded-gt-step-check.txt` shows why the translation is the ground truth's: the recorded
ground-truth step between consecutive scans is exactly 36.0 mm or 39.6 mm (0.18 m/s times a stamp spacing of
0.20 s or 0.22 s), a quantization of one scan period of travel, and the maximum error is one such step. The Webots lidar measures on a regular 0.2 s lattice and
stamps do not follow it (Phase 4, implementation 5.2 item 2), so on pairs where a lattice step was skipped
the recorded relative translation is wrong by about a step. That mechanism is the Phase 4 finding applied
here and is consistent with the numbers; it was not separately tested. The consequence is the one the design
predicted: the Webots recordings bound the matcher's error above by the ground truth's (about 3.5 cm and a few
milliradians) and cannot resolve it below that. Scans were already motion-gated by Phase 4's mapper; no
occupancy-grid value was read.

**S1b.** Model level, one scan pair per run, four runs. From offsets up to 0.3 m and 0.25 rad ICP settled within
2 cm and 0.01 rad in 100 of 100 trials on every run; at 0.5 m and 0.4 rad in 99 or 100 of 100; at 1.0 m and 0.8 rad
in 40 to 44 of 100. So the basin ends between those two ranges on this pair. This is a statement about
ICP on scans of this room, not about Webots, and one pair per run is a small sample of geometries.

**S2.** With `info_inflation` 1.0 the development registrations were OVERCONFIDENT (mean NEES 4.53 against 3). After
the freeze the mean NEES is within the 95% interval of the chi-squared mean on the development run and on two of
the three evaluation routes, and marginally above it on the diamond route (3.20 against an upper end of 3.17).
So the factor 1.5 measured on one route holds on two others and is marginally short on a third. The interval
treats consecutive scans as independent, which they are not, so it is optimistic. **On the Webots recordings the
mean NEES is 21 to 29 and says nothing about the matcher's information**: the ground-truth error that dominates
S1a is ten times the matcher's claimed standard deviation, so the NEES is a measurement of the ground truth.

**S4.** *Natural drift:* the front end alone drifts by well under a centimetre over 18 to 53 m on every route
(about 3 mm ATE RMS), so there is almost nothing for a loop closure to repair, and the optimized graph is **worse
than the chain it started from** on all four routes (graph ATE 1.99 to 2.93 times the front end's). The
pre-registered check `graph_ate_rms_over_odometry` (at most 1.05) fails on all four, and S4 is FAIL on all four;
every absolute check passes (graph ATE 6.6 to 10.0 mm against 50 mm). The likely cause is that loop-closure
measurements accurate to a few millimetres are added in the hundreds to a chain that is accurate to fewer, and
that the overlapping edges of neighbouring keyframes are treated as independent when their errors are correlated;
neither was tested. In the terms of problem.md section 4, this is what the world allows: Rusty's lidar noise is a few millimetres after
averaging and its kinematics are exact, so the scan matcher has nothing to accumulate. *With injected error*: the
front end alone is at 0.23 to 0.59 m ATE RMS and the graph with accepted loops at 7 to 11 mm, S4 PASS on all four.
That shows the optimizer and the verified loops repair a systematic yaw error they were told about. It does not show
the scan matcher drifts, and it was run with edge information chosen for the error.

**S5.** Across the eight runs, 1638 loop constraints were accepted and none was wrong (error over 0.05 m or
0.03 rad against ground truth); per run the rule of three bounds the wrong-accept rate below the column of the table. **This
is weaker than it sounds.** Nearly every candidate the detector proposed was a true loop in this small, fully
visible room (for example 483 of 484 on the development run), so the data contains almost no false candidates for
the verifier to have failed on; the candidates designed to be rejected are in `pose_graph_it.rs`, not here. What
the verifier did reject were true loops: `ambiguous` on the cross route (four natural, five injected; two crossing
lines are symmetric under a half turn, which is a guess at why, not tested) and on the development route (one each),
and a few `gate_exceeded` on the development and rectangle routes.
Recall of what the detector proposed is high, and every keyframe that had a true-loop partner by ground truth ended up
with an accepted loop edge on every run. Loops the detector never proposed are not counted.

**S6.** The whole pipeline (front end, graph, loop verification), run twice on the recorded scan file, is bit-identical on all
eight synthetic runs, and so is the S1a pass; on the four Webots recordings what was replayed is the registration pass only. A registration step of the front end takes a few milliseconds at the 99th percentile in a release
build; an optimization with loops takes tenths of a second at the 99th percentile, on a graph of up to 192
keyframes and a few hundred loop edges, which is the sparse `faer` path (the dense path was not timed).

**S7.** On synthetic odometry with a declared systematic yaw error, EKF-SLAM (with pillar detections of the same
scans) has a lower ATE than dead reckoning of that odometry on all four routes, and the grid RBPF with 30 particles
and no scan-matching proposal has a **higher** ATE than dead reckoning on all four. EKF-SLAM holds more landmarks than there are pillars (27 to 46 for
four) because the Phase 3 detector reports some wall clusters as pillars and the filter has no landmark
retirement (DS14); a scratch check, not saved, found that even with exact odometry it holds one landmark on each
pillar plus the false ones. The RBPF tracks when the odometry is exact (the same scratch check, not saved) and
fails under a systematic yaw error, as a proposal without scan matching can. EKF-SLAM's mean pose NEES is from 2.5
to 6.5 over the four routes. These say which settings these two back ends cope with, and nothing about which is better.

## What these runs do not show

* Anything about the scan matcher's drift or a loop closure's value **on Webots**: the recordings have no turning
  scans, so the front end and the graph were not run on them, and no new Webots run was made.
* Anything about false loop closures in a world with real aliasing: this room is small and everything in it
  overlaps, and the one symmetric case (the cross route) produced rejections of true loops, not acceptances of false ones.
* Anything about `/map`, S8, the two nodes, or any ranking of back ends.
* That the tolerances are right. Every tolerance was fixed in advance and every check that failed, failed against
  it; whether a 1.05 ratio is the right bar for a graph on a chain that has no drift is a question for the
  project owner, not something these runs settle.
