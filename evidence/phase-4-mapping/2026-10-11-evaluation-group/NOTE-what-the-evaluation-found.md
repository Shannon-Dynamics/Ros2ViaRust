# The three M1 evaluation runs: M1 FAILS on every one, on different checks

Three runs on routes written after the parameters were frozen (`route_eval_cross.yaml`,
`route_eval_wall_lobes.yaml`, `route_eval_diamond.yaml`, in `sim/webots/config/`), the mapper and
every parameter as frozen at development run 3, no tuning in between. All three runs are valid
(`valid: true`: scans lost to missing transforms 4, 2 and 4; the motion-skip fraction, 68 of 523,
98 of 554 and 207 of 738, is under the bound of 0.5 in each). Each run's own verdict is its
`phase4-verdict.yaml`; this note adds nothing that is not in those files or in
`analysis-wall-overshoot.txt` next to it. Per `problem.md` section 4 each run is a row on its own;
there is no pass rate here and none should be made from it.

## The rows (M1, from each run's `phase4-verdict.yaml`)

| Check (limit) | 1 cross | 2 wall-lobes | 3 diamond-slow | development 3 (route_long) |
|---|---|---|---|---|
| surface error p95, cells (<= 1.0) | 0.5 | 0.5 | **1.5 FAIL** | 0.5 |
| surface error max, cells (<= 1.5) | 0.879 | 0.879 | 1.5 (on the limit) | 1.5 |
| surface error mean, cells (reported) | 0.498 | 0.497 | 0.576 | 0.524 |
| false occupied fraction (<= 0.02) | 0.0 (0 cells) | 0.0 (0 cells) | **0.0789 FAIL** (57 cells) | 0.0261 FAIL (18 cells) |
| completeness (>= 0.95) | **0.9259 FAIL** | **0.9072 FAIL** | **0.8856 FAIL** | 0.9537 |
| false free fraction (<= 0.05) | 0.0486 | **0.0624 FAIL** | 0.0419 | 0.0459 |
| coverage (>= 0.9) | 0.9926 | 0.9840 | 0.9869 | 0.9855 |
| observable fraction (>= 0.5) | 0.9849 | 0.9921 | 0.9622 | 0.9640 |
| M1 checks that pass, of 7 | 6 | 5 | 4 | 6 |
| M2 (replay bit-identical) | PASS | PASS | PASS | PASS |
| M4 update latency p99 (<= 0.1 s) | 2.13 ms | 1.78 ms | 2.77 ms | 1.89 ms |
| scans integrated / skipped for motion | 451 / 68 | 454 / 98 | 527 / 207 | 1312 / 311 |
| route driven | 17.44 m, 0.8 rad/s | 17.60 m, 0.8 rad/s | 20.46 m, 0.3 rad/s | 51.52 m, 0.8 rad/s |

Spread across the three evaluation runs: the mean surface error spreads by 0.078 cells (the
verdict's group check, limit 1.0 cell, `group-verdict.console`). The other metrics spread much more
than that: surface error p95 by 1.0 cell, false occupied from 0 to 7.9 per cent, completeness
0.886 to 0.926 (the three values are all below 0.95), false free 4.2 to 6.2 per cent. By
`problem.md`'s own rule, differences of a cell or more are a finding to investigate and not a reason
to add runs, which is what the open findings below are.

## Statement on `false_occupied_fraction`

It is **under 2 per cent on routes 1 and 2 (0 cells each) and over it on route 3 (7.9 per cent, 57
cells)**. It did not clear 2 per cent on all three, and it did not fail on all three.

## What is the same and what is not, against development run 3

Identical in all four runs: every threshold and weight in `summary.yaml` (occupied 2.944439, free
-0.61903924, `l_occ` 0.84729785, `l_free`, clamps, resolution 0.05), the launch arguments apart from
the route, the world file hash, the verdict script hash. **Not identical, and said so here: the
mapper executable.** Development run 3 ran
`mapping_node` `be4a581c...` built at commit `11451a9`; the three evaluation runs ran `fc649eca...`
built at `d89d1bd`. The only source difference in `crates/` and `ros2_ws/` between those commits is
`ros2_ws/src/robotics_mapping_ros/src/mapping_node.rs`, where the four optional `output.*` parameters
accept `none` for "off" (the note of development run 3: the old binary died when they were not
given). With all four given, as in all four runs, nothing differs, but that is a reading of the
diff and not a bit-identical binary. The three evaluation runs are the same binary as each other
(`same_mapper_sha: true`).

`same_launch_args: false` in `group-verdict.console` is the verdict script comparing launch strings
that contain `route=`, which is different in every run by design; with the route removed the three
strings are identical (checked). The verdict script was not edited (its hash is the committed one).

`source_git_dirty: true` in each run: the routes and the change to `run_phase4_mapping.sh` below were
not committed when the runs were made. The route files are in each run's `route_sha256`.

## Harness change made for these runs

`scripts/run_phase4_mapping.sh` accepts `DRIVE_ANGULAR` (rad/s, default unset: the driver's 0.8) and
writes the value it read back from the running `route_driver` into `summary.yaml`
(`route_driver_angular_rad_s`: 0.8, 0.8, 0.3). Run 3 was made with `DRIVE_ANGULAR=0.3`. Nothing about
the mapper changed.

## Open findings (not chased, parameters stay frozen)

**1. Beam-0-to-14 overshoot is route-dependent and, on route 3, large.** The wall hits that land more
than 4.5 cm past a face (`analysis-wall-overshoot.txt`), for beams 0 to 14 (straight behind the
robot), against all other beams:

| Run | beams 0-14 | other beams |
|---|---|---|
| development 3, route_long | 230 / 10325 = 2.2 % | 354 / 250345 = 0.14 % |
| evaluation 1, cross | 0 / 3625 = 0 % | 119 / 94943 = 0.13 % |
| evaluation 2, wall-lobes | 0 / 3387 = 0 % | 88 / 89248 = 0.10 % |
| evaluation 3, diamond-slow | **861 / 3379 = 25.5 %** | 257 / 101248 = 0.25 % |

The route-dependence settles one thing: it is not a fixed property of those beams in the sensor frame,
since two routes show none of it with about 3500 behind-hits each. What it does not settle is the
cause. Route 3 differs from routes 1 and 2 in several ways at once (the walls are seen only from
1.3 m or farther, the heading is 45 degrees off the room's axes, the turns are at 0.3 rad/s, the
route is driven in two laps), and `route_long` shares the far-wall and oblique-heading part with it
and not the slow turns; with three routes these cannot be told apart, and no cause is claimed. In
run 3, by range, 2.5 per cent of all hits at 3.0 to 3.5 m and 0.9 per cent at 2 to 3 m overshoot,
against 0.1 to 0.3 per cent in routes 1 and 2, so range alone does not explain it either. The
turn-rate reading in particular is not tested by this route: the mapper's motion gate rejects scans
turned faster than 0.075 rad/s, and 0.3 is above that, so no scan taken during a turn at either rate
is in the map. A test of turn rate on the scans themselves would need turns below 0.075 rad/s, which
nobody has checked the base can do.

**2. Completeness is below 0.95 on all three evaluation routes (0.926, 0.907, 0.886).** Development
run 3 had 0.954 and its note called the margin thin. This was not looked into further here; the
check is the fraction of observable surface samples with an occupied cell within a cell, and which
samples are missed has not been broken down by wall, pillar or range.

**3. A flaw in the verdict script's group check** (above, `same_launch_args`), which cannot be true
for three different routes.

**4. `route_long.yaml`'s header overstates its clearance.** It says the smallest clearance is over
0.6 m, from waypoints only; measured over the legs, the gap (centre-to-leg distance minus the pillar
radius 0.15 and the footprint 0.11) is 0.39 m at landmark_a and 0.40 m at landmark_d. The runs on it
completed, so nothing is invalid; the header is wrong. Not edited here.

## What this does and does not say

M1 is closed for Phase 4 in the sense that the three evaluation runs exist and are reported. It is
**not passed**: the pre-registered tolerances were not met on any of the three, on different checks
in each, and this is the result. With ground-truth pose it says about the mapper only that it is
deterministic (M2) and cheap (M4) on all of them, that the surface error is about half a cell on
average on all of them, and that the checks it misses depend on the route in ways not yet explained.
