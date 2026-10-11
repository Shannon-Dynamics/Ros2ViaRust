# Pillar evidence cancellation: a capped diagnostic pass

Question: at the same viewing angle (5 to 14 degrees from the normal) `route_long` finds 1.00 of its pillar samples and the
evaluation routes 0.56, so something specific to the route cancels evidence that incidence does not explain.
Method as in `NOTE-stage-B-gate.md`: the Python port of the mapper and the evaluator (validated bit for bit,
`root-cause/output/01` and `02`), beam angles corrected by the `pixel_centres` layout, frozen parameters; the candidate fixes
were then run through the **Rust** `replay_scans.rs` and `map_metrics.rs`. No Webots run. Scripts `root-cause/scripts/pillar_*.py`,
`pillar_*.sh`; outputs `root-cause/output/27a`, `27b`, `28`, `29`.

## Result in one paragraph

No evaluator quirk and no scan-count difference explains it. A well-viewed pillar cell is built up by near-normal views and then
**eroded late in the run by grazing views, which mark it free at about the weight a hit marks it occupied**, and the occupied
state sits only 1.3 free updates below the upper clamp, so the last few scans of a route decide what the final map says.
Raising the upper clamp (`model.p_max`) restores the cells on all three evaluation routes, and a Bresenham traversal adds a
little more; both are replay-verified. The effect has no plateau up to the largest clamp tried, so the value is a trade-off to be
decided, not a number the data picks.

## What was tested and what each test says

**C1, evaluator-side counting: ruled out** (`27a`). Samples that a trajectory position sees within 15 degrees but no integrated scan
does: 0 of 129 (`route_long`), 0 of 187 (cross), 1 of 211 (wall-lobes), 2 of 125 (diamond).

**C2, scan density and distance: not the separator** (`27a`). For well-viewed samples (an integrated scan within 15 degrees),
found against missed:

| Run | scans within 15 deg | share of views past 60 deg | median range of the near-normal views |
|---|---|---|---|
| route_long, found (129; none missed) | 273 | 0.11 | 2.15 m |
| cross, found / missed (141 / 46) | 50 / 24 | 0.18 / 0.45 | 1.92 / 1.31 m |
| wall-lobes, found / missed (119 / 91) | 44 / 45 | 0.28 / 0.25 | 2.04 / 1.86 m |
| diamond, found / missed (118 / 5) | 138 / 13 | 0.19 / 0.40 | 2.26 / 1.10 m |

On wall-lobes found and missed samples have the same number of near-normal scans and the same share of grazing views; so no
single one of these separates them on every route.

**C3, per-scan outcome of the sample's best cell** (`27a`): the same for found and missed samples. A near-normal view (0 to 15
degrees) marks the cell occupied in 0.87 to 0.92 of scans either way (1.00 for the five diamond misses); a view past 60 degrees marks it occupied in 0.38 to 0.48
and **free in 0.20 to 0.46** (found) or 0.21 to 0.41 (missed). What differs between samples is only how many of each kind of view
they get, and when.

**State history, the finding** (`27b`): the best cell of every missed well-viewed sample **was occupied at some point (100%)**, for
0.84, 0.72 and 0.83 of its updates (cross, wall-lobes, diamond; found: 0.90, 0.70, 0.97), and the median number of updates since it
was last occupied is **8, 31 and 7** (found: 0). They were built and then lost. Grading the same run a few scans earlier gives
different completeness: cross 0.910 at the end against 0.935 twenty scans earlier (well-viewed pillar samples found 141 then 151 of
187, 163 at forty), wall-lobes 0.893 against 0.907 and 0.916; `route_long` and diamond move by under 0.01.

**The threshold geometry that makes this possible**: occupied needs log-odds 2.944, the upper clamp is 3.476, a free update is
-0.405, so a saturated occupied cell is not occupied after **two** free updates, but is called free only after eleven
(`occupancy.rs` doc table). The state "occupied" is much easier to lose than "free" is to reach.

## Candidate fixes, replay-verified (Rust, `pixel_centres`; `28`, `29`)

Completeness (checks passed of the seven M1 checks) with the unchanged evaluator, tracer and `l_max` (`model.p_max` = 0.97 is 3.476):

| tracer, `l_max` (`p_max`) | route_long | cross | wall-lobes | diamond |
|---|---|---|---|---|
| supercover, 3.476 (0.970), as run | 0.9586 (7) | 0.9098 (5) | 0.8930 (5) | 0.9320 (6) |
| supercover, 5.0 (0.993) | 0.9586 (7) | 0.9430 (6) | 0.9402 (5) | 0.9705 (7) |
| bresenham, 3.476 | 0.9593 (7) | 0.9142 (6) | 0.8905 (5) | 0.9622 (7) |
| bresenham, 5.0 (0.993) | 0.9623 (7) | 0.9456 (6) | 0.9565 (7) | 0.9716 (7) |
| bresenham, 6.0 (0.9975) | 0.9646 (7) | 0.9554 (7) | 0.9666 (7) | 0.9791 (7) |
| bresenham, 8.0 (0.9997) | 0.9728 (7) | 0.9620 (7) | 0.9739 (7) | 0.9836 (7) |
| supercover, 8.0 (0.9997) | 0.9657 (7) | 0.9529 (7) | 0.9619 (7) | 0.9764 (7) |

False-occupied stays 4, 0, 0, 0 cells and surface error p95 stays 0.5 cells in every row; false-free falls with the clamp
(cross 5.59% to 3.69%, wall-lobes 6.02% to 3.37%). Completeness rises monotonically with the clamp over the whole range tried,
for both tracers, and does not flatten.

## What this does not establish

* **The route dependence is described, not proven as the only cause.** The mix of late grazing views and the replay effect of the
  clamp and the tracer are measured; that grazing views are the thing that erodes the cells is the reading that fits C3 and the
  state history, and the clamp experiment is the intervention supporting it. Corner clipping by the supercover traversal was
  not isolated; the tracer effect is smaller and its mechanism is a candidate.
* **The value is not decided by the data.** More clamp is better throughout the tested range, and what it costs (slower
  forgetting in a world that changes) is not tested here: the room is static, so this world cannot show it.
* **The four runs are development data.** The routes were burned by the analysis; the choice made from them is validated, or not,
  only by fresh evaluation routes.
* Changing the tracer default touches the wording of claim M3 (the supercover property is a property of that module, which would no
  longer be the mapper's default).

## Recommendation for the Stage D freeze (superseded: the value was re-selected at `l_max` 5.0 under the Stage C evaluator, `implementation.md` 5.4 item 4)

Fix `model.p_max` and `model.tracer` before the development run and write the rule down first, since `route_long` cannot tell
these values apart (7 of 7 in every row). One rule that follows from this table: the smallest tested clamp for which all four
recorded streams pass all seven checks, which is **bresenham with `l_max` 6.0 (`model.p_max` 0.9975)**; supercover would
need 8.0. That is a choice made on burned data and should be labelled so.
