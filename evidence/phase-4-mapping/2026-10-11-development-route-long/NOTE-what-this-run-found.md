# First development run: M2 and M4 pass, M1 fails, and why

`route_long.yaml` (51.5 m, 21 turns in place), role `development`, `ground_truth_tf_adapter`
owning `map -> odom`, the mapper with the motion gate and the clear margin (D13, D14) and
the first defaults: occupied after one hit, scans integrated at their stamp. 1749 scans
arrived: 1437 integrated, 310 skipped for motion, 2 lost to missing transforms.

**M2 passes**: replaying the 1437 recorded scans reproduces `final-grid.txt` byte for byte.
**M4 passes**: the 99th percentile of the update is 2.1 ms in a debug build.
**M1 fails on five of seven checks** (`phase4-verdict.yaml`): false occupied 13.2 per cent,
surface error 95th percentile 1.5 cells and maximum 9.5, false free 6.6 per cent; completeness
(0.965) and coverage (0.998) pass. `source_git_dirty: true` in `summary.yaml` is the untracked
`crates/robotics_wasm/` of the webbook, which is not part of this work (`source-status.txt`).

The analysis that followed (offline, from this directory's own files; the numbers and the
three changes it led to are in `docs/phase-4-mapping/implementation.md` section 5.2 item 10):
115 of the 117 far-from-surface occupied cells are in the row just outside a wall, mostly
one hit each; the 636 hits that land there (0.19 per cent of 332 635) were taken driving
straight, on every wall; replaying with the occupied threshold at two hits (offline, same
scans) halves the false-occupied cells and leaves the surface error as it is.

The offline replay of this recording with this run's own parameters reproduces its metrics
exactly (904 occupied, 119 false, completeness 0.9653, false free 0.0657), so `replay_scans`
followed by `map_metrics` is a faithful stand-in for the node and the evaluator, for read-out
parameters. It is not one for anything that changes which scans are integrated or at what pose.

This run is kept as it was produced. It is the development run's first attempt, not the
frozen one: the second is `../2026-10-11-development-2-route-long/`.
