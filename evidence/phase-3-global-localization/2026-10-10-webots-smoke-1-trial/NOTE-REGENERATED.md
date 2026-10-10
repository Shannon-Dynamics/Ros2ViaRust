# Derived files regenerated; raw files untouched

## Second regeneration, 2026-10-10 evening

`verdict.yaml` and `summary.yaml` were written once more, from the same untouched raw files,
by the final `scripts/phase3_global_verdict.py`, which added `correct_at_end_of_route` (ground
truth at the last sample, whatever was claimed) and the grading of kidnap injections. This
run's `mcl_node.log` predates the `KIDNAP_INJECTION stamp_s=` form of the injection line, so
its injections are counted and reported as `ungraded_log_has_no_stamp`, not graded.
