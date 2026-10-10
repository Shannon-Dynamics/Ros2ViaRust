# Third harness smoke run: the clear margin and the evaluator fix, and a sensor artifact

Same 0.5 m triangle route as `../2026-10-11-harness-smoke/`, with the motion gate, the 0.10 m
clear margin (D14) and the evaluator reading the resolution to seven decimals.

`phase4-verdict.yaml` says `valid: false` and grades nothing: 4 of 169 scans were lost to
missing transforms (all in the first 1.1 s of the run, before the transform tree is complete),
which is 2.4 per cent of a 169-scan run and above the 2 per cent validity bound. That bound is
meant for the 1500-scan routes; on this short smoke route it is a property of the route and not
of the mapper. Nothing was changed for it. The metrics in `map-metrics.yaml` are read here
only to understand the mapper, not as a result: surface error mean 0.51 cells and 95th
percentile 0.5, false occupied 0.5 per cent, false free 3.9 per cent (the exact-grid
estimate was 4), completeness 0.976, coverage 0.982.

**One number is not like the others: the maximum surface error is 5.5 cells.** It is one cell,
hit about sixty times by one beam of the final stationary scans (the robot sits still for the
last 14 s), 0.28 m from the nearest pillar. Comparing that scan beam by beam with the
analytic room (`scripts/phase4_scan_skew_analysis.py` helpers) shows five beams of 360 that
disagree with the analytic range by more than 5 cm, and all five are at a pillar's silhouette:
measured ranges between the pillar's (2.31 m) and the wall behind it (3.23 m), 2.75 m for this
one. That is the usual depth-edge artifact of a range sensor whose pixel straddles a foreground
and a background surface (a "mixed pixel" or "flying pixel"); the Webots lidar is rendered, so
it has it too. It is not a mapping fault: the mapper writes a hit where it is told one is. A
stationary sensor sees the same flying pixel every scan, so sixty identical scans saturate its
cell as occupied, which is a second, separate weakness: repeated scans from one pose are not
independent measurements and the update treats them as if they were.

Neither is handled in this phase. They are recorded in
`docs/phase-4-mapping/implementation.md` section 5.3 and will show in `surface_error_max_cells`
of the runs that follow; the tolerances of `scripts/phase4_verdict.py` were committed before
any run and are not changed to absorb them.
