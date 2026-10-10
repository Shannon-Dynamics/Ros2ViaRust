# Derived files regenerated; raw files untouched

`verdict.yaml` of each trial and this run's `summary.yaml` were written again after the run,
by the final `scripts/phase3_global_verdict.py`, from the raw files of the run
(`mcl_node.log`, `run.csv`, `run.json`, `diagnostics.yaml`), which were not touched. The
verdict script gained a count of the trials that have a detector reading.

This run's `diagnostics.yaml` is one message taken with `ros2 topic echo --once`, which is
whichever node published first and is often not `mcl_node`. So the detector and timing
readings exist for only some trials; `trials_with_a_diagnostics_reading` in `summary.yaml`
says how many. The claims, the timestamps and the errors do not depend on it. The replicate
runs named `...-seed101-...` capture a few seconds of the stream and have the readings for
every trial.

## Second regeneration, 2026-10-10 evening

`verdict.yaml` and `summary.yaml` were written once more, from the same untouched raw files,
by the final `scripts/phase3_global_verdict.py`, which added `correct_at_end_of_route` (ground
truth at the last sample, whatever was claimed) and the grading of kidnap injections. This
run's `mcl_node.log` predates the `KIDNAP_INJECTION stamp_s=` form of the injection line, so
its injections are counted and reported as `ungraded_log_has_no_stamp`, not graded.
