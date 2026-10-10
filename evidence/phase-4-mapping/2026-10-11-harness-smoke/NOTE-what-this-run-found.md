# The first Webots mapping run was a harness smoke test, and it failed M1 for a reason worth keeping

Run `2026-10-11-harness-smoke`: role `development`, route `route_smoke.yaml` (a triangle 0.5 m
across, 192 scans), the mapper as first built, with no motion gate. It was made to check that
the harness works end to end, and it does: provenance was recorded from the running graph
(`ground_truth_tf_adapter` owned `map -> odom`, `mapping_node` did not subscribe to ground
truth), M2 passed (the replay of the 189 integrated scans reproduces the grid byte for byte)
and M4 passed (99th percentile of the update 2.7 ms in a debug build).

**M1 failed, and badly** (see `phase4-verdict.yaml`): 230 of 772 occupied cells farther than one
cell from every surface, surface error up to 5.5 cells, a quarter of the observable surface
samples inside a cell called free. This is a smoke run on a 0.5 m triangle and not an
evaluation, but the size of the error is not a small-sample effect.

What it exposed, found by looking at where the wrong cells were and then at the scan-skew
recording `../2026-10-10-scan-skew-probe/`: the header stamp of a Webots `/scan` is **not** the
instant its beams were measured. The measurement instants lie on a regular 0.2 s lattice (phase
standard deviation 3 ms) and the stamps do not; the stamp lags the measurement by 0 to about
0.18 s, 0.06 s on average, in a sawtooth. Integrating a scan at the pose TF gives for its stamp
therefore uses a pose that is wrong by the motion of up to 0.18 s: harmless standing still,
a few centimetres driving straight at 0.18 m/s, and tens of centimetres at the far wall while the
robot turns in place at 0.8 rad/s, which this route does three times. The first report on that
recording said a scan is one instant "at its header stamp"; that was incomplete, and
`docs/phase-4-mapping/implementation.md` section 5.2 says what is true.

This run is kept as it was produced, with its verdict, as the evidence of that finding. The
mapper afterwards gained a motion gate (decision D13); the run that tests it is
`../2026-10-11-smoke-with-motion-gate/`. The verdict script was edited once after this run, to
count scans skipped for motion apart from scans lost to missing transforms; the edit and its
reason are in a comment in `scripts/phase4_verdict.py`, and no accuracy tolerance changed.
