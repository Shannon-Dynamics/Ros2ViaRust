# Phase 3 correctness runs

One directory per run of [`scripts/run_phase3_correctness.sh`](../../scripts/run_phase3_correctness.sh),
holding the raw output of that run and nothing else: no number here is retyped from a file,
and no file here was edited after the run wrote it. The documentation points at these
directories instead of quoting them, so that a claim can always be checked against the file
it came from.

Since 2026-10-10 every run also writes `<scenario>-verdict.yaml`, the decision on criterion
P1 of [`../../docs/phase-3-localization/problem.md`](../../docs/phase-3-localization/problem.md),
computed from the run's own CSV and JSON by
[`scripts/phase3_verdict.py`](../../scripts/phase3_verdict.py). Read that file for the
numbers of a run. The two older runs predate it; the script works on them as well, because
it only reads files they already contain.

| Run | Route | Scenario | Verdict |
|---|---|---|---|
| [`2026-10-10-long-route`](2026-10-10-long-route/) | `route_long.yaml`, 51 m, 21 sharp corners | normal | **The current answer on P1.** See `normal-verdict.yaml`. |
| [`2026-10-06-after-fix`](2026-10-06-after-fix/) | a 0.9 m square, 3.1 m of path | normal | Valid and complete, and **the wrong question asked correctly**: see below. |
| [`2026-10-06-nodes-killed`](2026-10-06-nodes-killed/) | the same square | normal | Valid motion, **no summary**. The route was driven and the CSV is complete, but the SIGINT of the run script never reached either Rust node, both were killed 30 s later, and `localization_evaluator` writes its summary only on a clean shutdown: there is no `normal-run.json`. The SIGKILL also only hit the `ros2 run` wrappers, leaving two orphaned node processes behind. |
| [`2026-10-06`](2026-10-06/) | the same square | normal | **Invalid, do not use.** The robot never moved: the ros2_control hardware interface never loaded, so `/cmd_vel` was accepted by nothing while every topic kept publishing and the evaluator kept writing rows. Post mortem in [`2026-10-06/INVALID.md`](2026-10-06/INVALID.md). |

Two further runs of the same route were made on 2026-10-10 and are not here. They were
written in the WSL build mirror and never copied to the Windows checkout, and the next
`scripts/sync_to_wsl.sh` deleted them, because that sync is `rsync --delete` from Windows
and removes whatever exists only in the mirror. The run above is a third run, made after the
same defects those two had found were fixed, so nothing was learned twice. The warning that
would have prevented it is now in the header of
[`../../scripts/sync_to_wsl.sh`](../../scripts/sync_to_wsl.sh), and the run wrapper copies a
finished run to the Windows checkout before anything else happens.

## What the 2026-10-10 run settles

Criterion P1 has two decidable parts and both are in `normal-verdict.yaml`:
`strictly_better_than_odometry`, the filter's position RMSE against the RMSE of the dead
reckoning it was given, and `bounded_over_distance`, whether its error in the last quarter
of the run is worse than in the first. Both are satisfied, on the odometry the simulator
publishes, with nothing injected: the verdict file carries `odometry_noise: off`.

What the verdict file also shows, and what the earlier route could not, is the shape of the
two error curves rather than only their sizes. The filter's `estimate_growth_factor` stays
near one while the odometry's `odometry_growth_factor` is several times that: the filter's
error is flat over distance and dead reckoning's grows with it, which is the actual claim
of [`../../docs/phase-3-localization/problem.md`](../../docs/phase-3-localization/problem.md)
section 2. A single RMSE ratio would have been a weaker result even if it had come out the
same way.

Three things are still open and no run here speaks to them:

* **The NEES part of P1 is not evaluated.** It needs the full 3 by 3 covariance of each
  estimate and the CSV carries only `sigma_xy`, so the verdict file says
  `nees_in_95_band: not_evaluated` instead of guessing. Closing it is a change to
  `localization_evaluator`, not to the filter.
* **The `kidnap` scenario has never been run.** The harness for it is in the run script and
  the teleport works, but there is no evidence directory, so P2 has no evidence of any kind.
* **The MCL half that P2 needs is not implemented.** See
  [`../../docs/phase-3-localization/implementation.md`](../../docs/phase-3-localization/implementation.md).

## Why the 2026-10-06 square is kept

It is a valid run of an invalid question, and that is worth more here than deleting it. The
route is a 0.9 m square driven in about 40 s, 3.1 m of path, and over that distance the
Webots differential drive reports odometry accurate to about two millimetres, so there was
almost nothing for an absolute correction to fix while the filter's own measurement noise
was fully present. The filter came out several times worse than the baseline, and the
correct reading of that was not "the EKF is worse than odometry" but "this route cannot
tell". Running `scripts/phase3_verdict.py` on it still prints `verdict: FAIL`, which is the
honest verdict for the measurement that was taken.

Why that odometry is as good as it is, and why the answer was to lengthen the route rather
than to degrade the robot, is in
[`../../docs/phase-3-localization/theory.md`](../../docs/phase-3-localization/theory.md)
section 6.
