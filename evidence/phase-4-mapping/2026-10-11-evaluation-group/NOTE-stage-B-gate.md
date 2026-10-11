# Stage B gate: the route-1 regression, the end residual, and the incidence curve

Read-only, same method as `NOTE-root-cause-analysis.md` (Python port that reproduces the mapper and the evaluator;
`root-cause/scripts/`, outputs `root-cause/output/21` to `25`). No code in `crates/`, `ros2_ws/` or `scripts/` changed.
"H1" is the pixel-centre beam layout of `root-cause/stageA-sources/README.md`. All maps below use the frozen parameters.

## 1. Route 1's regression is in false-free, not false-occupied

`false_occupied` on route 1 is 0 before and after. `false_free_fraction` goes 4.86% (133 samples) to 5.59% (153), limit 5%.
(`21-stageB_diag_route1.txt`, `20-stageA_preview.txt`.)

* **It was marginal before.** The limit allows 136.9 samples; 133 leaves 3.9 samples of headroom.
* **All of it is pillar samples**, before and after. 124 are false-free in both maps, 29 became false-free, 9 stopped.
  The 29 touch 6 cells (pillars A, B and C).
* **Interior beams, not the end residual, by intervention** (`22-stageB_variants_route1.txt`):

| Correction applied to | false-free | completeness |
|---|---|---|
| none (published) | 133 (4.86%) | 0.9259 |
| all beams (H1) | 153 (5.59%) | 0.9098 |
| interior beams 15-344 only | 153 (5.59%) | 0.9098 |
| end beams 0-14 and 345-359 only | 133 (4.86%) | 0.9259 |

  Over the 6 cells the marks of end-zone beams are 0 before and 4 occupied and 3 free after; interior beams give
  751 to 752 occupied and 508 to 538 free updates (`21`). The cells had an occupied share of about 0.6 and went from
  median log-odds +0.45 to -1.89.
* **The metric is that sensitive.** Shifting every published beam by a uniform 2 mrad (0.6 cm at 3 m) gives
  124, 134, 135 and 129 false-free samples for -2, -1, +1 and +2 mrad (`22`): a spread of 11 samples with no direction,
  nearly three times the headroom. Marginal pillar rim cells flip with centimetre shifts.
* **The completeness fall on route 1 (0.9259 to 0.9098) is walls, not pillars**: pillars found 170 to 171 of 338,
  walls 2365 to 2320 of 2400; 50 samples of the south wall stop being found, whose positions with no occupied cell
  at all rise from 10 to 16 (`24-stageB_walls.txt`).

## 2. The end residual is not implicated in any remaining failure

After H1, the false-occupied cells are 4 on route_long and 0 on the three evaluation routes (`25-stageB_residual.txt`).
The hits that ended in the 4: 19, all from interior beams. Before H1: 91% (route_long, 148 of 163) and 100%
(route 3, 577 of 578) came from the end zone. The residual is real (`18-stageA_h1.txt`: about -9 and +10 mrad
remaining at beams 0 to 9 and 350 to 359) and would still put about 3.5% of route 3's beam 0 to 14 hits past the wall
(in-sample estimate, same file), but on these four runs it makes no false-occupied cell and no route-1 change.

## 3. What the correction does overall (`20`, `24`)

| Run | false occupied | surface p95 (cells) | completeness | false-free |
|---|---|---|---|---|
| route_long | 18 to 4 cells (2.61 to 0.55%) | 0.5 to 0.5 | 0.9537 to 0.9586 | 4.59 to 4.63% |
| cross | 0 to 0 | 0.5 to 0.5 | 0.9259 to 0.9098 | 4.86 to 5.59% |
| wall-lobes | 0 to 0 | 0.5 to 0.5 | 0.9072 to 0.8930 | 6.24 to 6.02% |
| diamond-slow | 57 to 0 cells (7.89 to 0%) | 1.5 to 0.5 | 0.8856 to 0.9320 | 4.19 to 3.59% |

The displaced outer-wall patterns of route 3 (`..X`, `.XX`) disappear (`24`); walls found 2185 to 2325 of 2400.
Completeness and false-free move both ways by 0.005 to 0.05 and 0.2 to 0.7 points, which is the scale of section 1's sensitivity.

## 4. Incidence curve (`23-stageB_curve.txt`)

Pillar samples, found fraction against the best angle between the surface normal and the direction to any ground-truth
trajectory position that sees the sample (the quantity an evaluator rule could compute), published beam angles
(H1 in brackets where it differs by more than 0.03):

| Angle | four runs pooled | the three evaluation routes | route_long |
|---|---|---|---|
| 0-4 | 466/578 = 0.81 | 354/466 = 0.76 | 112/112 = 1.00 |
| 5-14 | 49/74 = 0.66 | 32/57 = 0.56 | 17/17 = 1.00 |
| 15-24 | 44/79 = 0.56 | 25/58 = 0.43 | 19/21 = 0.90 |
| 25-34 | 32/74 = 0.43 | 21/56 = 0.38 | 11/18 = 0.61 |
| 35-44 | 30/81 = 0.37 (0.32) | 26/61 = 0.43 (0.36) | 4/20 = 0.20 |
| 45-59 | 39/115 = 0.34 (0.27) | 36/87 = 0.41 (0.32) | 3/28 = 0.11 |
| 60-74 | 24/125 = 0.19 | 24/93 = 0.26 | 0/32 = 0.00 |
| 75-89 | 8/125 = 0.06 | 8/93 = 0.09 | 0/32 = 0.00 |

Walls for contrast: 0.97 found at 0 to 14 degrees, 0.95 at 15 to 29, 104 of 104 at 30 to 44 (no wall sample has a best angle above 44).

**Shape.** Pooled and on the evaluation routes the curve declines without a bend: a plateau near 0.4 from 15 to 59 degrees,
then steps down at 60 and at 75. Route_long alone has a sharp edge between 25 and 40 degrees (1.0 to 0.2), which is where
">37" came from. The two sets disagree by 0.4 at the same angle (5-14: 1.00 against 0.56), so incidence alone does not decide
whether a pillar sample is found; the route does (the open evidence-cancellation item).

Completeness if samples above a limit were not counted (information only, not a choice criterion):

| limit | route_long | cross | wall-lobes | diamond-slow |
|---|---|---|---|---|
| 60 degrees, published | 0.977 | 0.945 | 0.923 | 0.902 |
| 60 degrees, H1 | 0.982 | 0.928 | 0.908 | 0.952 |
| none, H1 | 0.959 | 0.910 | 0.893 | 0.932 |

No limit in the table brings routes 1 and 2 to 0.95 after the correction.
