# Phase 3 correctness runs

One directory per run of [`scripts/run_phase3_correctness.sh`](../../scripts/run_phase3_correctness.sh),
holding the raw output of that run and nothing else: no number here is retyped from a file,
and no file here was edited after the run wrote it. The documentation points at these
directories instead of quoting them, so that a claim can always be checked against the file
it came from.

| Run | Scenario | Verdict |
|---|---|---|
| [`2026-10-06`](2026-10-06/) | normal | **Invalid, do not use.** The robot never moved: the ros2_control hardware interface never loaded, so `/cmd_vel` was accepted by nothing while every topic kept publishing and the evaluator kept writing rows. Post mortem in [`2026-10-06/INVALID.md`](2026-10-06/INVALID.md). |
| [`2026-10-06-nodes-killed`](2026-10-06-nodes-killed/) | normal | Valid motion, **no summary**. The route was driven and the CSV is complete, but the SIGINT of the run script never reached either Rust node, both were killed 30 s later, and `localization_evaluator` writes its summary only on a clean shutdown: there is no `normal-run.json`. The SIGKILL also only hit the `ros2 run` wrappers, leaving two orphaned node processes behind. |
| [`2026-10-06-after-fix`](2026-10-06-after-fix/) | normal | **The usable run.** Both controllers active before the route started, the robot drove the full square, every process exited by itself with exit code 0 and nothing was left running, and `normal-run.json` was written. |

## What the usable run says, and what it does not

Read `2026-10-06-after-fix/normal-run.json` next to its `normal-environment.txt`. Two things
are worth knowing before anyone quotes it:

* **The EKF is not yet better than the odometry it corrects.** On this route
  `odom_only_rmse_position_m` is several times *lower* than the filter's
  `rmse_position_m`. The P1 criterion in
  [`../../docs/phase-3-localization/problem.md`](../../docs/phase-3-localization/problem.md)
  is "bounded over distance, and strictly better than the drift of `/odom` on the same
  run", and on this evidence it is not met.
* **That is a property of the route, not yet a verdict on the filter.** The route is a
  0.9 m square driven in about 40 s, and the Webots differential drive reports almost
  perfect odometry over that distance, so there is nearly nothing for an absolute
  correction to fix, while the filter's own measurement noise is fully present. A fair
  test of P1 needs a drive long enough for `/odom` to actually drift, or odometry degraded
  on purpose. Until one of those is run, neither "the EKF works" nor "the EKF is worse than
  odometry" is supported.

The `kidnap` scenario has not been run at all yet, so P2 (global localization and recovery)
has no evidence of any kind. The MCL half that P2 needs is not implemented, see
[`../../docs/phase-3-localization/implementation.md`](../../docs/phase-3-localization/implementation.md).
