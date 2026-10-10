# Second development run: D15 and D17 were in effect, D16 was not

The run was made to test three changes (D15 the mean stamp lag, D16 two hits, D17 no repeats
from a standing pose; `docs/phase-4-mapping/implementation.md` section 5.2 item 10). **Only D15
and D17 were in effect.** The default of `model.p_occupied` was meant to go from 0.65 to 0.75
and the text replacement that did it silently matched nothing, because `rustfmt` had split the
line it was looking for. The header of `final-grid.txt` shows it (`occupied_threshold 0.619`,
which is probability 0.65); the commit that introduced D16 said it was in effect, and that
statement was wrong. Found by reading that header after the run, not before it. The harness now
writes the thresholds the run used into `summary.yaml`, read back from the files the run wrote,
and the verdict refuses a run whose mapper and evaluator disagree about them.

What the run does show, with one hit making a cell occupied (1309 scans integrated, 313
skipped for motion, 126 skipped as stationary repeats, 2 lost to missing transforms):

* false free fell to 4.3 per cent (it was 6.6) and passes; completeness and coverage pass;
* false occupied did **not** fall (12.8 per cent, 121 cells, against 13.2 per cent), nor did
  the 95th percentile (1.5 cells) or the maximum (9.5): the mean stamp lag (D15) was not the
  cause of the cells outside the walls. A regression of the wall hits' displacement on the
  heading finds no along-heading or lateral offset left (-0.6 mm and +0.9 mm); what remains is
  a spread (sd 14.6 mm about the wall plane, 0.2 per cent of 241 662 hits beyond 45 mm), the tail
  of the range noise and the residual timing error, falling on a row of cells outside the wall
  that no beam ever crosses to clear.
* replaying this recording offline with the occupied threshold at other values (same scans,
  `replay_scans` then `map_metrics`; the replay at this run's own 0.65 reproduces its metrics):

  | occupied threshold (probability, hits needed) | occupied cells | false occupied | 95th percentile of the surface error | completeness |
  |---|---|---|---|---|
  | 0.75, two | 851 | 67 (7.9 %) | 1.5 cells | 0.962 |
  | 0.85, three | 803 | 35 (4.4 %) | 0.71 cells | 0.960 |
  | 0.90, three | 771 | 32 (4.2 %) | 0.71 cells | 0.959 |
  | 0.95, four | 716 | 20 (2.8 %) | 0.5 cells | 0.959 |
  | 0.96, four | 654 | 15 (2.3 %) | 0.5 cells | 0.938 |

  The maximum surface error is 1.5 cells at every one of them (9.5 at one hit: the cells
  far from any surface are single-hit cells), and the 20 cells left at 0.95 are all in the row
  outside the walls, at saturation, along wall stretches behind and beside the pillars, which
  is a systematic effect of grazing returns and not noise.

The next run uses 0.95. That choice was made after seeing this table, by a rule stated for the
record: the largest value of this grid that keeps completeness at the verdict's 0.95 or more. It
is a choice on a development recording, which is what that run is for; it is not a pre-registered
one, and it leaves false occupied above the verdict's 2 per cent. The run is
`../2026-10-11-development-3-route-long/`.
