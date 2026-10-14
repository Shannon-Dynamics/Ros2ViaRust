# The first replay run, kept

Made by `scripts/verify_phase5_slam_nodes.sh` with the nodes as first built, in a debug colcon install and with
every stream replayed at 20 Hz without lockstep. 50 checks passed and 5 failed, and the failures were findings:

* `replay-recorded-1-dev-route-long`: the scan matcher did not stop on SIGINT (launch escalated to SIGKILL after 75 s)
  and the in-process reference panicked on `SingularSystem("increment information")`. Causes, found afterwards:
  a settled registration with no line features reported a zero information matrix, which cannot be inverted into a
  covariance (library fix: the configured fallback information, `StepResult::information_fallback`); and a front end
  that has lost track (this file has no turning scans) costs seconds per step in a debug build, so five queued scans
  outlast launch's SIGINT timeout (node fix: drop the backlog on SIGINT; test fix: a release install and a lockstep
  replayer).
* `replay-recorded-3-eval-wall-lobes`: one scan of 454 was dropped by the best-effort topic because lost-track steps
  take longer than the 50 ms replay period (test fix: lockstep).
* `replay-synthetic-1-dev-route-long` and `replay-synthetic-4-eval-rectangle`: check E1 compared the float64 reference's
  keyframe count with the node's, but the node (float32 scan metadata) makes one keyframe more on those streams than
  the graded evidence run, which is what the reference must match (checker fix).

Nothing in this directory was edited; it was moved whole from `../2026-10-11`. The run that supersedes it is `../2026-10-11`.
