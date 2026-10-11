# Round two: development run 4 and three fresh evaluation runs, M1 met on each

Development run 4 (`2026-10-11-development-4-route-long`) and three evaluation runs on routes written for this round
(`route_eval_oblique.yaml`, `route_eval_pillar_arcs.yaml`, `route_eval_perimeter.yaml`), all on **one mapper binary**
(`mapping_node` sha256 `274d6540ab91…`) with the parameters frozen before the development run, and no tuning on any
of them. Each run is a row of its own; there is no pass rate, no interval, and none should be made from three rows.
Every run is valid (scans lost to missing transforms 2 to 4; scans skipped for motion 16 to 27 per cent of those seen,
bound 50), M2 passes (the replay equals the saved grid byte for byte) and M4 passes (p99 update 1.28 to 1.35 ms, debug
build). Verdicts: each run's `phase4-verdict.yaml`; the group: `group-verdict.console` next to this note.

## What was frozen, and where each run recorded it

Beam layout `pixel_centres`; traversal Bresenham; upper clamp `model.p_max` 0.99330715 (`l_max` 5.0); the evaluator's 60 degree
incidence limit and closed-cell completeness lookup; everything else as development run 3 (`implementation.md` 5.4).
`summary.yaml` of each run holds them (`launch_arguments`: `tracer=bresenham`, `beam_layout=pixel_centres`, `p_max=0.99330715`;
`mapper_l_max: 5`; `mapper_beam_layout: pixel_centres`), and the development run's was checked against them by the chain
script before the first evaluation run started. `source_git_dirty: true` in every run: the work is not committed.

## The rows (from each run's `phase4-verdict.yaml`; limits in brackets)

| Check | development 4, route_long | 4 oblique | 5 pillar-arcs | 6 perimeter |
|---|---|---|---|---|
| surface error p95, cells (<= 1.0) | 0.50 | 0.50 | 0.50 | 0.50 |
| surface error max, cells (<= 1.5) | 1.50 | 1.50 | 0.88 | 0.88 |
| surface error mean, cells (reported) | 0.4999 | 0.4981 | 0.4970 | 0.4975 |
| false occupied fraction (<= 0.02) | 0.0021 (2 cells) | 0.0011 (1 cell) | 0 | 0 |
| completeness (>= 0.95) | 0.9843 | 0.9821 | 0.9764 | 0.9629 |
| false free fraction (<= 0.05) | 0.0138 | 0.0234 | 0.0280 | 0.0331 |
| coverage (>= 0.9) | 0.9976 | 0.9952 | 0.9949 | 0.9939 |
| observable fraction (>= 0.5) | 0.941 | 0.967 | 0.976 | 0.979 |
| **M1: checks passed of 7** | **7, PASS** | **7, PASS** | **7, PASS** | **7, PASS** |
| M2, M4 | pass, pass | pass, pass | pass, pass | pass, pass |
| scans integrated / skipped for motion | 1308 / 313 | 504 / 126 | 482 / 178 | 572 / 106 |
| path driven | 51.5 m | 19.6 m | 18.9 m | 22.3 m |

Spread of the mean surface error over the three evaluation runs: 0.0011 cells (limit 1.0). The tightest margins: completeness
0.9629 on the perimeter route (limit 0.95), false-free 3.3% on the same (limit 5%), and the maximum surface error, which is **exactly on its
limit (1.5 cells) in the development run and in the oblique run**, as it was in development run 3; it passes (the script compares to
1e-9) and it has no room.

## What this does and does not show

* **The routes did their job as stress.** The oblique route meets all four walls at 45 degrees off the normal at 0.45 m and
  drives away from each with that wall behind it, the geometry that exposed the beam-angle error (round one's route 3 had 57
  false-occupied cells at 7.9%; here: 1 cell, 0.1%). The pillar-arc route looks at each pillar from many directions, which is
  the geometry that eroded pillar cells (round one's completeness 0.886 to 0.926; here 0.976). Both were written to stress those
  things, not to avoid them.
* **The metric is not the one round one failed.** Between the rounds the evaluator's rules changed, on the project owner's
  decision: observability only within 60 degrees of a surface normal, and a symmetric closed-cell lookup for completeness
  (`implementation.md` 5.4 items 2 and 3). The round-two passes are against that metric. For the record, the frozen mapper
  parameters graded by the **old** evaluator on round one's streams give completeness 0.9623, **0.9456**, 0.9565 and 0.9716 for
  `route_long`, cross, wall-lobes and diamond (`evaluation-group/root-cause/output/29-pillar_clamp_sweep.txt`, the `bresenham
  l_max=5.0` rows): the cross stream would still have missed 0.95 under the old metric, so part of what separates round two
  from round one is the metric and not the mapper. The two cannot be separated further than that table does.
* **The parameters were chosen on the four burned runs** (`implementation.md` 5.4 item 4). These three routes had no part in the
  choice, which is what makes them a test, and the mapper passed it on each. Three routes in one world with one sensor are not a
  rate and not a claim about another room.
* **Not claimed:** anything about an estimated pose (M5, waits for decision D3), a more complex world (M1-extended), real
  hardware, or a changing environment (the larger clamp trades slower forgetting for stability, and a static room cannot price it).
  With ground-truth pose the only errors left are the lidar noise and the discretization, so M1 shows that the mapper is
  implemented correctly, as `problem.md` says.

## Open, not resolved by this round

* The beam-angle residual at the two ends of the index range (about -9 and +10 mrad after the correction): unexplained, and
  in no run does it produce a false-occupied cell (`evaluation-group/root-cause/output/25-stageB_residual.txt`; round two has
  3 false-occupied cells in 4 runs).
* The clamp is a trade-off with no plateau in the sweep; 5.0 is the rule's smallest value under the evaluator now in force.
* `same_launch_args: false` in the group verdict, because the launch string contains `route=` (a flaw of the committed verdict script,
  not edited); everything else of the launch string is identical across the runs.
* `benchmarks/phase-4-mapping.csv` is not written. M5, T9 (the rclpy port and `nav2_costmap_2d` references) and M1-extended
  are not started.
