# The first batch, kept

Made with the front end before the fix of `docs/phase-5-slam/implementation.md` section 8.3 item 10 (a registration
that did not settle at the adaptive threshold was not retried at a wider one), and before the injected runs handed
the graph the information their error warrants. See `../NOTE-first-evidence-runs.md`.

* `synthetic-2-eval-cross` and `synthetic-2c-eval-cross-injected` have no metrics: the harness panicked
  (`SingularSystem("increment information")` in `run.log`) because the front end had lost track at the first turn
  after a long straight and then reported a zero information matrix.
* `synthetic-1-dev-route-long` is the development measurement of S2 with `info_inflation` 1.0 (mean NEES 4.53 over 1828
  registrations), from which 1.5 was frozen by the rule of `docs/phase-5-slam/benchmark.md` section 6. Its S1a and S2 do
  not depend on the fix and stand; its S4 to S7 are superseded by `../v2-synthetic-1-dev-route-long`.
* The other directories are the first batch's other routes, natural and injected: superseded by the `v2-` runs.

Nothing here was edited after it was written; it was moved here, whole, from `..`.
