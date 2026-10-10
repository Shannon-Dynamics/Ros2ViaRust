# Third development run: M1 misses on one check of seven; parameters frozen here

`route_long.yaml`, role `development`, `ground_truth_tf_adapter` owning `map -> odom`, with every
change of `docs/phase-4-mapping/implementation.md` 5.2 items 7 to 10 in effect this time, and the
thresholds the run used read back into `summary.yaml` (occupied 2.944439 = probability 0.95,
free -0.619, hit weight 0.847, mapper and evaluator agree). 1312 scans integrated, 311 skipped
for motion, 127 skipped as stationary repeats, 4 lost to missing transforms.

`phase4-verdict.yaml`: **M1 FAIL, on `false_occupied_fraction` only**: 18 of 689 occupied cells,
2.6 per cent against the verdict's 2 per cent. The other six checks pass: surface error 95th
percentile 0.5 cells and maximum 1.5 cells, completeness 0.954 (the margin over 0.95 is small),
false free 4.6 per cent, coverage 0.985, observable fraction 0.964. M2 passes (1312 scans
replayed, the grid reproduced byte for byte) and M4 passes (99th percentile of the update 1.9 ms,
debug build). The verdict script's tolerances are those committed before any run; the only edits
since are the 1e-9 slack on comparisons and the threshold-agreement guard, neither of which
changes this verdict (it did not need the slack: the maximum equals 1.5 to within 4e-15).

**What the 18 cells are.** All 18 are exactly 1.5 cells from the nearest surface, the row of cells
just outside a wall (on the north wall at x 1.9 to 2.7, on the west wall at y 1.6 to 2.2, and a
few elsewhere); 10 of them are at saturation. The hits that land in them have a median range of
3.15 m and an incidence (10th to 90th percentile) of 37 to 62 degrees from the wall normal, none above 70, so the
grazing-incidence explanation suggested by the offline replay of the second run is **not**
supported. What the whole run does show is that the rate of wall hits landing more than 4.5 cm past
the wall face depends strongly on the beam index: 2.0 per cent of the wall hits of beams 0 to 14
(straight behind the robot), 1.0 per cent of beams 60 to 74, 0.46 per cent of beams 45 to 59, 0.37
of 255 to 269, and under 0.1 per cent for most of the others. A cause tied to the beam's direction
in the sensor frame points at the sensor model or at the pose along the heading and has not been
chased further here. It is recorded in implementation.md 5.3 as open.

**Not a result of the mapper alone.** M1 here is with ground-truth pose; the map is as accurate as
the pose at the measurement instant, which the driver's stamp does not give (D13 to D15).

**Parameters are frozen at these values.** Whatever evaluation runs follow use them unchanged. The
mapper's executable was rebuilt after this run (the optional `output.*` parameters accept `none`;
before, a mapper started without them died, which `scripts/verify_phase4_mapping.sh` found);
with all four given, as here, nothing differs.
