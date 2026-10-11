"""Stage B gate: after the correction (H1) is the unexplained end-zone residual (beams 0-14 and 345-359) implicated in any remaining false-occupied cell?
For every cell that is occupied and farther than one cell from a true surface in the corrected replay: which beams ended in it."""
import sys, os, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
def fix(s):
    n = len(s["ranges"]); fov = -(n - 1) * s["da"]; s2 = dict(s); s2["da"] = -fov / n; s2["a0"] = s["a0"] - 0.5 * fov / n; return s2
for run in RUNS:
    d = EV + "2026-10-11-" + run; par = R.read_params(d); scans = R.read_scans(d)
    for name, sc in (("published", scans), ("H1", [fix(s) for s in scans])):
        o = R.replay(sc, par); occ = o["L"] >= np.float32(par["occupied_threshold"])
        fo = {j * R.W + i for j, i in zip(*np.nonzero(occ)) if R.true_dist(*R.cell_centre(i, j)) > 0.05 + 1e-9}
        by = collections.Counter(); scans_by = collections.defaultdict(set)
        for si, bi, z, ex, ey, cell in o["hits"]:
            if cell in fo:
                zone = "end (0-14, 345-359)" if (bi < 15 or bi >= 345) else "interior (15-344)"; by[zone] += 1; scans_by[zone].add((si, cell))
        tot = sum(by.values())
        print(f"{run:27s} {name:9s} false-occupied cells {len(fo):3d}; beam hits that ended in them: {tot:5d}" + "".join(f"  {k}: {v} ({v/max(1,tot)*100:.0f}%)" for k, v in sorted(by.items())), flush=True)
