# Phase 3 correctness runs

One directory per run of [`scripts/run_phase3_correctness.sh`](../../scripts/run_phase3_correctness.sh),
holding the raw output of that run and nothing else: no number here is retyped from a file,
and no file here was edited after the run wrote it. The documentation points at these
directories instead of quoting them, so that a claim can always be checked against the file
it came from.

Since 2026-10-10 every run also writes a verdict file, computed from the run's own CSV and
JSON, which is what to read for the numbers of a run. Which file depends on the scenario and
the estimator:

| Scenario | Estimator | File | Written by |
|---|---|---|---|
| `normal` | EKF | `normal-verdict.yaml` | [`scripts/phase3_verdict.py`](../../scripts/phase3_verdict.py) |
| `normal` | MCL | `normal-verdict-mcl.yaml` | the same |
| `kidnap` | EKF | `kidnap-verdict-ekf.yaml` | [`scripts/phase3_kidnap_verdict.py`](../../scripts/phase3_kidnap_verdict.py) |
| `kidnap` | MCL | `kidnap-verdict-mcl.yaml` | the same |

The `normal` files decide criterion P1 of
[`../../docs/phase-3-localization/problem.md`](../../docs/phase-3-localization/problem.md),
bounded tracking error. The `kidnap` files decide the P2 recovery row, which is a different
question and so has a different script and its own definition of what "recovered" means;
that definition and the three constants behind it are in the script's docstring and are
repeated into every file it writes. The oldest runs predate all of this; the P1 script works
on them anyway, because it only reads files they already contain.

A run that names two estimators drives one Webots session for both, so a `normal` or
`kidnap` directory holding both an EKF and an MCL verdict is one trajectory and, for
`kidnap`, one teleport at one simulated instant, graded twice. That is deliberate: it is the
only way the comparison is between two filters rather than between two runs.

| Run | Route | Scenario | Verdict |
|---|---|---|---|
| [`2026-10-10T144153-final-global-reset`](2026-10-10T144153-final-global-reset/) | `route_long.yaml`, 51.540 m, 21 waypoints | normal, both estimators | **Current P1 evidence.** EKF and MCL both pass; MCL records zero recovery injections. |
| [`2026-10-10T143418-mcl-global-reset`](2026-10-10T143418-mcl-global-reset/) | the same route, teleport after 120 s | kidnap, both estimators | **Current P2 evidence.** MCL recovers after one full global reset; EKF does not. |
| [`2026-10-10T142643-final-default`](2026-10-10T142643-final-default/) | the same route | normal, both estimators | Retained failed attempt: five-scan persistence caused seven false injections and MCL failed P1. |
| [`2026-10-10T141910-mcl-6000-persist5`](2026-10-10T141910-mcl-6000-persist5/) | the same route | kidnap, both estimators | Five-scan persistence recovers in 14.120 s; the following normal run shows its false-injection failure. |
| [`2026-10-10T141137-final-mcl`](2026-10-10T141137-final-mcl/) | the same route | kidnap, both estimators | Retained failed attempt: 20-scan persistence with a 20 per cent injection did not recover. |
| [`2026-10-10T133711-mcl-median`](2026-10-10T133711-mcl-median/) | the same route | normal, both estimators | Invalid for the new algorithm because it used a stale executable from the root install space. Retained to document the build-path error. |
| [`2026-10-10T133620-mcl-median`](2026-10-10T133620-mcl-median/) | the same route | normal, both estimators | Interrupted by a WSL restart. Partial output is retained rather than promoted to a verdict. |
| [`2026-10-10-mcl-gated`](2026-10-10-mcl-gated/) | `route_long.yaml`, 21 sharp corners | normal and kidnap, both estimators | Historical evidence after the per-observation gate: EKF passes P1, MCL still fails P1, and this run exposed the recovery-trigger defect fixed by the timestamped runs above. |
| [`2026-10-10-mcl-comparison`](2026-10-10-mcl-comparison/) | the same route | normal, both estimators | **MCL before it had a per-observation gate**, kept as the measurement that found that defect rather than deleted. Its `normal-verdict-mcl.yaml` is route-scale error next to the EKF's `normal-verdict.yaml` from the same session. Analysis in [`../../docs/phase-3-localization/implementation.md`](../../docs/phase-3-localization/implementation.md) section 5.1. |
| [`2026-10-10-long-route`](2026-10-10-long-route/) | `route_long.yaml`, 21 sharp corners | normal, EKF only | The run that first answered P1, before MCL existed. See `normal-verdict.yaml`. |
| [`2026-10-06-after-fix`](2026-10-06-after-fix/) | a 0.9 m square, 3.1 m of path | normal | Valid and complete, and **the wrong question asked correctly**: see below. |
| [`2026-10-06-nodes-killed`](2026-10-06-nodes-killed/) | the same square | normal | Valid motion, **no summary**. The route was driven and the CSV is complete, but the SIGINT of the run script never reached either Rust node, both were killed 30 s later, and `localization_evaluator` writes its summary only on a clean shutdown: there is no `normal-run.json`. The SIGKILL also only hit the `ros2 run` wrappers, leaving two orphaned node processes behind. |
| [`2026-10-06`](2026-10-06/) | the same square | normal | **Invalid, do not use.** The robot never moved: the ros2_control hardware interface never loaded, so `/cmd_vel` was accepted by nothing while every topic kept publishing and the evaluator kept writing rows. Post mortem in [`2026-10-06/INVALID.md`](2026-10-06/INVALID.md). |

Several runs of the same route are missing from this directory, and the reason is worth
keeping. They were written in the WSL build mirror and never copied to the Windows checkout,
and the next `scripts/sync_to_wsl.sh` deleted them, because that sync is `rsync --delete`
from Windows and removes whatever exists only in the mirror. It ran at the start of every
run, so the window was open constantly. That destroyed a 6270-row evaluation, a partial
integration-test log, and a set of MCL measurement runs.

A warning in the script header was the first attempt and was not enough, because it depended
on someone remembering a manual copy at the right moment. What replaced it does not:

1. [`../../scripts/evidence_root.sh`](../../scripts/evidence_root.sh) resolves
   `EVIDENCE_ROOT` to this directory's parent in the Windows checkout whenever it is
   mounted, and the run scripts write their output there as it is produced. There is no
   longer a moment at which the only copy is in the mirror.
2. `sync_to_wsl.sh` excludes `evidence/` and `benchmarks/` from the rsync, so `--delete`
   cannot reach run output even when a run does write into the mirror.

A consequence of (2): the mirror's `evidence/` no longer mirrors this directory. The Windows
checkout is the only authoritative copy.

## What the final 2026-10-10 runs settle

Criterion P1 has two decidable parts and both are in the final normal verdict files:
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
* **Uniform-prior global localization is not evaluated.** The kidnapped scenario is now
  measured, but `mcl_node` still starts from the known pose and nothing drives `new_global`.
* **Cross-language performance is not evaluated.** The correctness runs contain only the
  Rust implementation; the Python and C++ benchmark is a separate task.

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
