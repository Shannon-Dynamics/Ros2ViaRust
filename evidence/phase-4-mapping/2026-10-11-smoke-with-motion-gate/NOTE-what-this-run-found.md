# Second harness smoke run: the motion gate, and what it did not fix

Same 0.5 m triangle route as `../2026-10-11-harness-smoke/`, with the motion gate added
(`mapping_node` skips a scan when the sensor moved more than 5 cm or turned more than 0.015 rad
across the window `[stamp - 0.2 s, stamp]`; decision D13). 27 of 192 scans were skipped for
motion, 2 were lost to missing transforms, 163 were integrated.

It fixed what it was made for: the surface error of the occupied cells fell from a 95th
percentile of 3.5 cells to 0.5 and the false-occupied fraction from 30 to 0.9 per cent.

It did not fix `false_free_fraction` (22 per cent here), and the reason is not the mapper: the
evaluator read the grid's resolution from a `float32` (0.05 arrived as 0.0500000007), which
moved a wall lying on a lattice line 2e-6 cells off it and made the closed-cell rule of
`eval.rs` miss it. The exact grid in `final-grid.txt` gives about 4 per cent when graded by hand.
Fixed before `../2026-10-11-smoke-with-margin/`. Its other finding, beams that cross wall cells
at the edge of the lidar's reach, is decision D14 (the clear margin).

The verdict file here is the verdict script's output at the time and is kept unedited. The
`M1: FAIL` in it is real for the mapper and the evaluator as they were, not for the mapper as it
is now.
