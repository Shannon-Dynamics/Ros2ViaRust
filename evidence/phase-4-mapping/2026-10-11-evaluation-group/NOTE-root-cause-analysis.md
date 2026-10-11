# Root cause of the M1 failures: neither lead as stated, and three separate mechanisms

Read-only analysis of the four runs that exist on disk (`development-3-route-long`, `evaluation-1-cross`,
`evaluation-2-wall-lobes`, `evaluation-3-diamond-slow`). No Webots run, no parameter change, no route
change, nothing under `crates/`, `ros2_ws/` or `sim/` touched. Everything below is produced by
`root-cause/scripts/run_all.sh` (about 5 minutes); each claim cites the file in `root-cause/output/` that
prints it. Companion to `NOTE-what-the-evaluation-found.md`.

## Result, numbers first

| Failing check | Lead it was assigned to | What the data says |
|---|---|---|
| completeness, routes 1, 2, 3 and (thin) dev 3 | Lead 1, exposure | **Not exposure.** 196/203, 255/256, 304/306 and 124/124 missed samples sit next to cells that were marked occupied by 4 or more scans. It is three other things: pillar surface seen only edge-on, evidence cancelled by free updates on well-viewed pillars, and (route 3) the wall displaced outward (below) |
| false occupied, route 3 (57 cells) and dev 3 (18 cells) | Lead 2, velocity | **Not velocity.** The cells come from a fixed angular error of particular beams. A table of that error fitted *without* the run it predicts reproduces the held-out overshoot rate of beams 0 to 14: 24.3% predicted against 25.7% observed on route 3, and 0.4 and 0.6% against 0% on routes 1 and 2 |
| surface error p95, route 3 | Lead 2 | Same cause as the false-occupied cells: the displaced cells are 1.5 cells from the surface |
| false free, route 2 (6.2%) | neither | Not analysed here beyond the counts in `02-validate2.txt`; no claim |
| every evaluation check | the evaluator itself | Two properties of the metric change what a miss means (end of section 3) |

No single bug explains the four runs. Completeness fails on routes 1 and 2 with zero false-occupied cells, and the
cause there (pillar geometry and metric definition) has nothing to do with the cause of route 3's false-occupied cells
(beam angle), exactly as the prompt allowed for.

## 0. Method, and how it was checked

* A Python port of the mapper's integration (`integrate.rs`, `Supercover` of `raycast.rs`, `add_at` and `classify`
  of `occupancy.rs`) was run on each run's `integrated-scans.csv`. It reproduces each run's `final-grid.txt` with
  **0 differing cells and a maximum difference of 0.0** in all four (`01-validate.txt`).
* A port of the evaluator (`eval.rs`: surface samples at 0.01 m, visibility from every ground-truth position, the
  nearest-cell distance lookup, the closed-cell false-free rule) reproduces each run's `map-metrics.yaml`:
  observable counts 2680, 2738, 2758, 2675, completeness to ten digits, false-free counts 123, 133, 172, 112
  (`02-validate2.txt`). Everything per sample below is therefore the evaluator's own judgement.

Three premises of the prompt were checked and differ from the files:

1. **`l_occ = 2.944439` is not a per-hit weight.** It is the occupied *threshold* (probability 0.95). A hit adds
   `l_occ = 0.84729785`, a pass-through adds `l_free = -0.4054651`, the cell is clamped to [-1.99, 3.48], and a cell
   receives **one** update per scan, occupied if any beam ended in it, otherwise free if any crossed it
   (`integrate.rs`, `summary.yaml` of each run). Four uncancelled occupied updates (3.39) cross the threshold; three (2.54) do not.
2. **`scan_time_offset_s` defaults to 0.0** in `mapping_node.rs` (lines 545 to 552); the -0.06 s is passed by
   `run_phase4_mapping.sh` and recorded in each run's `launch_arguments`.
3. **Angular velocity of the scans that are in the map is not what differs between the runs.** See Lead 2.

## 1. Lead 1: exposure and integration count

**Measured.** For every sample the evaluator calls observable and does not find (203, 256, 306 on the evaluation
routes, 124 on `route_long`): how many integrated scans can see it (line of sight, within 3.5 m, on the lit side), and
the largest number of scans that marked its lookup cell or a 4-neighbour occupied (`04-lead1.txt`).

| Run | missed | seen by no integrated scan | marked occupied by 1 to 3 scans | marked occupied by 4 or more, still below threshold |
|---|---|---|---|---|
| route_long (dev 3) | 124 | 0 | 0 | 124 |
| cross | 203 | 3 | 4 | 196 |
| wall-lobes | 256 | 1 | 0 | 255 |
| diamond-slow | 306 | 2 | 0 | 304 |

Median number of integrated scans able to see a missed sample: 273, 95, 136, 201; for found samples: 611, 221, 177, 239.
Median of the largest occupied-update count on a neighbouring cell: 194, 52, 69, 83. So missed samples are somewhat
less exposed than found ones, but by tens to hundreds of updates, not by the one or two the lead proposed. **Lead 1
as stated is falsified for 879 of the 889 misses** (the other 10: 4 marked by one to three scans, 6 seen by no
integrated scan).

**What it is instead** (`05-lead1_table.txt`, `07-pillars.txt`, `08-qstats.txt`, `09-nearmiss.txt`, `06-profile.txt`):

| Run | missed | pillar, best view more than 37 degrees off the normal | pillar, well viewed (cos >= 0.8) | wall | unseen |
|---|---|---|---|---|---|
| route_long | 124 | 104 | 10 | 10 | 0 |
| cross | 203 | 88 | 77 | 35 | 3 |
| wall-lobes | 256 | 76 | 109 | 70 | 1 |
| diamond-slow | 306 | 67 | 22 | 215 | 2 |

* **Pillar samples that no scan viewed within 37 degrees of their normal are mostly missed:** 104 of 111 on route_long
  (94%), 88 of 107 on cross (82%), 76 of 105 on wall-lobes (72%), 67 of 106 on diamond-slow (63%); by view-angle bin
  between 39 and 100% (`07-pillars.txt`). Pillar samples with a view within 37 degrees are missed 6, 34, 43 and 13% of
  the time. The evaluator calls a sample observable if any ground-truth position has a line of sight to it at any
  incidence above zero, so the limb of every pillar counts. On route_long almost every miss is of this kind (104 of
  124); on the evaluation routes it is 88 of 203, 76 of 256 and 67 of 306. For scale only, and not a metric: with those
  samples left out of the count, completeness would be 0.992, 0.956, 0.932 and 0.907 (`05-lead1_table.txt`, last column).
* **Well-viewed pillar samples that are missed (10, 77, 109, 22):** the nearest occupied cell centre is typically
  0.1 to 0.28 m from the lookup cell, so the cells are absent, not a cell off (`09-nearmiss.txt`). Over **all** missed
  pillar samples (limb ones included; `08-qstats.txt` does not split by view angle) the neighbouring cell with the
  highest occupied share q (occupied updates over occupied plus free) has median q 0.33, 0.51 and 0.35 on the three
  evaluation routes (found pillar samples: 0.90, 0.74, 0.80), against a break-even of
  `l_free / (l_occ + l_free) = 0.324`: the evidence is being cancelled, not missing (the missed cells have a median of
  68, 88 and 59 occupied updates against 67, 64 and 103 free ones). *Why* those rim cells are freed (beams skimming a
  pillar's limb, pillar centres on lattice corners, the 5 cm cell against a 0.15 m radius) was **not determined**.
* **Walls on routes 1 and 2 and on route_long (35, 70 and 10 samples):** 35 of 35, 60 of 70 and 10 of 10 have the
  nearest occupied centre at the **diagonal, 0.0707 m, against a tolerance of 0.05 m** (`09-nearmiss.txt`): one-cell
  holes in a thin wall, missed by 2 cm of centre distance. The missed wall cells have q 0.79 to 0.91, none below break-even.
* **Walls on route 3 (215 samples) are the displaced wall of section 2:** wall W has 14 positions with only the
  cell one step outside the outer wall cell occupied and 8 with nothing, wall S 14 and 6 (`06-profile.txt`, patterns
  `...|..X` and `...|...`); at 5 samples per cell that is 110 and 100 missed samples, which are the counts of owners
  `w3` and `w0` in `04-lead1.txt`.

**Two properties of the evaluator found on the way** (`03-tiebreak.txt`): the nearest-cell lookup rounds the lattice
line the wall sits on upward, so a wall sample is judged on the **outer** cell for the east and north walls and on
the **inner** cell for the west and south walls. A wall displaced outward by one cell is therefore counted found on
two walls and missed on the other two, which is part of why route 3 loses 210 wall samples on W and S, 5 on N and none on E.
And "observable" ignores incidence (above). Neither was changed; both are properties of the pre-registered metric.

**Verdict on Lead 1:** explains none of the misses as stated (an exposure shortfall). Route shape matters, but through
the directions the pillars were seen from, not through scan count.

## 2. Lead 2: velocity dependence of the false-occupied cells

**Measured**, per run, from `update-latency.csv` (the gate's own recorded yaw and translation over the 0.2 s window
before each scan's stamp, which exists for every scan) joined to `integrated-scans.csv` by stamp (`10-lead2_basic.txt`,
`11-lead2.txt`). Finite differences of `sensor_theta` between consecutive integrated scans give the same: 517 of 526
route-3 pairs are 0.25 s or less apart (median 0.0000, p95 0.0108, max 0.0302 rad/s), the other 9 span 5.3 to 12.8 s
across skipped turns and would report an angular velocity for time the robot was not in the map (`17-omega_geom.txt`).

* **The scans that are in the map do not differ in angular velocity or speed between runs.** Integrated scans have
  median |omega| 0.0000 rad/s in all four runs, 93 to 100% below 0.01, maximum 0.071, 0.004, 0.074, 0.075 (the gate
  admits up to 0.075), and speed 0.180 m/s (median and p95) in all four. The 0.8 and 0.3 rad/s exist only in scans the
  gate rejected (maximum over all scans 0.802 and 0.302).
* **Route_long, within the run:** the 93 scans that made the 18 false-occupied cells have the same |omega| as the
  other 1219 (98% below 0.01 in both, Mann-Whitney p = 0.112); per-scan behind-beam overshoot against |omega|:
  Spearman -0.015 (p = 0.69, 783 scans). **No relation.**
* **Route 3, within the run:** the 214 scans behind the 57 cells are all at low omega (100% below 0.01, against 88%
  of the others, p = 0.0013), and the per-scan behind-beam overshoot rate falls with omega: 0.344 for omega below
  0.005 (209 scans), 0.045 for 0.005 to 0.02 (43 scans), 0.051 above (13 scans); Spearman -0.579 (p = 4.5e-25). This is
  the direction the prompt expected. It does not support a velocity cause, for two reasons the data shows:
  routes 1 and 2 have the same omega near 0 and the same 0.18 m/s and **0.000 overshoot in every scan** (`11-lead2.txt`);
  and a model with no velocity term at all predicts most of the same pattern from geometry, 27.5% against 15.0 and
  13.4% for the same three groups (observed 32.8, 4.5, 5.1%; `17-omega_geom.txt`). What remains (4.5 and 5.1% observed
  against about 14% predicted, 56 scans) is **unexplained**.
* **Scan-time offset, tested directly** (`16-timeoffset.txt`): a timing error moves the pose along the heading by
  velocity times the error, which has its own signature in the range residuals. Fitted with the angular term below
  removed, the shift is 1.2, 1.0, 1.1 and 0.1 mm (95% bootstrap intervals within about 2 mm), equivalent to at most
  12 ms of the -0.06 s setting, and 0.1 mm on the slow route. Only magnitudes are interpreted; the sign convention
  of the fit was not verified against the pose lookup. **No sign of a wrong offset on any run, route 3 included.**

**What explains the false-occupied cells instead:** the range residual of every wall beam against the true room,
ray-cast from the mapper's own pose (`12-residual.txt`), fitted as `residual = range * tan(incidence) * eps`, an
angular error eps of the beam (`13-warp.txt`, `14-warp2.txt`, pooled over the four runs):

* eps is about **-14 to -23 mrad for beams 0 to 9**, relaxing to about -2 by beam 19, a few mrad (mostly negative) from
  beam 20 to 140, within about 4 mrad of zero from 140 to 259, and about **+17 mrad for beams 350 to 359**. It is the same shape in
  each of the four runs (`13-warp.txt`, group means of 15 beams: -15, -6, -11, -15 mrad for beams 0 to 14 and +16, +3,
  +12, +17 for 345 to 359; route 1 sees only near-normal incidence there, so its values are the least constrained).
* Because a beam's range error is `range * tan(incidence) * eps`, the same eps costs nothing at normal incidence
  and about 3 m x 1 x 0.017 = 5 cm at 45 degrees. Routes 1 and 2 drive along the axes, so the walls behind the
  robot are at near-normal incidence; route 3 drives at 45 degrees to them. The error has opposite signs at the two
  ends of the index range, so on one side it puts hits beyond the wall (the false-occupied cells) and on the other it
  puts them inside (invisible to the metric): that is the "beams 0 to 14 but not 345 to 359" asymmetry of the
  development note.
* **Held-out prediction** (`15-predict.txt`): eps fitted per 5 beams on three runs, then used to predict the fourth
  run's rate of wall hits more than 4.5 cm past the face, with 15.5 mm of range noise:

| Held-out run | beams 0 to 14 observed | predicted | other beams observed | predicted |
|---|---|---|---|---|
| route_long | 2.34% | 3.54% | 0.14% | 0.39% |
| cross | 0.00% | 0.40% | 0.13% | 0.34% |
| wall-lobes | 0.00% | 0.57% | 0.09% | 0.28% |
| diamond-slow | 25.71% | 24.28% | 0.25% | 0.37% |

The model has no velocity, no route and no turn rate in it. It over-predicts the other beams slightly (the noise is
assumed a little high).

**Verdict on Lead 2:** velocity does not explain the false-occupied cells or the 0.3 rad/s run's difference; a
route-independent angular error of the beams near the ends of the index range does, with the route entering only
through the incidence at which it shows the walls. 85% of the beams that marked route 3's false-occupied cells are
beams 0 to 14 (489 of 578, `11-lead2.txt`).

**What this does not establish.** The error is a property of the *reported beam angles against the measured ranges*,
read through the mapper's pose. A pose error cannot be fully excluded as a contributor, but a pose error would not
depend on the beam index, and the table does. Its source (the driver's published angle grid, `-2 pi/359` over 360
beams, against the sensor's real spacing; the rendering of the lidar in Webots; the sensor frame in TF) is **not
determined**. For scale only: a true spacing of `2 pi/360` would put beam i `i * 4.86e-5` rad off the published grid,
which is +17 mrad at beam 359 and fits the last group; it does not fit the -23 mrad at beam 0, and it is 3 to 10 mrad
away from the group means in the middle (`14-warp2.txt`, "line" column). It is a candidate, not a finding.

## 3. Which check belongs to which mechanism

| Mechanism | Evidence | Checks it moves |
|---|---|---|
| Beam-index angular error (Lead 2, redefined) | `15-predict.txt` | false occupied (dev 3, route 3), surface error p95 (route 3), the displaced wall behind route 3's W and S misses |
| Pillar surface seen edge-on, plus the evaluator's observability definition | `07-pillars.txt`, `05-lead1_table.txt` | completeness on all four runs (104, 88, 76 and 67 misses) |
| Evidence cancelled on well-viewed pillar cells, cause open | `08-qstats.txt`, `09-nearmiss.txt` | completeness on routes 1 and 2 (77 and 109 misses) |
| One-cell wall holes against a 5 cm tolerance on a 5 cm lattice | `09-nearmiss.txt` | completeness on routes 1 and 2 (35 and 60 misses) |
| Wall-lookup tie-break (E and N outer cell, W and S inner cell) | `03-tiebreak.txt` | how many of route 3's displaced wall samples count as missed |
| Time offset, velocity, exposure | `16`, `11`, `04` | none found |

The false-free failure of route 2 (6.2%) is not assigned.

## 4. Recommendation for the next development pass

1. **First, the angular error, as a sensor-model question and not an occupancy parameter.** It is read-only to settle
   whether it is in the reported grid: compare the angle grid the driver publishes with the LDS-01's declared
   resolution and field of view (the proto is fetched from a URL by the world file and is not installed here, so this
   needs a look at that file and at the driver's lidar plugin), and fit eps per beam at single-beam resolution.
   If confirmed, correct the beam angles in the mapper's input and **replay the four recorded scan streams**
   (`replay_scans` takes the same files; no Webots run) to see what false occupied, surface error p95 and the displaced
   wall do. Those four runs are then development data, which they already have to be after this analysis, so the next
   evaluation routes have to be new ones.
2. **Decide the metric questions before any new evaluation run, and record them as decisions, not as edits to a
   graded run:** whether a sample is observable only within an incidence limit, and whether a wall sample's lookup
   should be indifferent to which side of the lattice line the wall is on. Together they decide 88 to 104 of the pillar
   misses and the W/S against E/N asymmetry. The tolerances of `phase4_verdict.py` were committed before the runs and
   have not been touched here.
3. **The well-viewed pillar misses are the open item.** A read-only replay with the grid origin shifted by half a
   cell would say whether the pillars' placement on lattice corners matters; the cause of the freed rim cells is not
   known.

Neither lead alone is conclusive for completeness, and the two settle different checks: Lead 2's real cause (the
angular error) is conclusive for false occupied, supported by a held-out prediction; Lead 1's real cause is a mixture,
with the pillar-geometry part measured and the well-viewed-pillar part still open.
