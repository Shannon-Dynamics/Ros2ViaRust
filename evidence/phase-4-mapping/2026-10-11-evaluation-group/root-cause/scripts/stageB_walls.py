"""Stage B gate: where does the completeness change from the beam-angle correction (H1) come from, by sample owner, and how does the wall's cross-section change?
Python port, frozen parameters, unchanged evaluator port. Pattern: three cells further inside | [inner cell, outer cell, outer+1 cell] counted from the true face."""
import sys, os, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
def fix(s):
    n = len(s["ranges"]); fov = -(n - 1) * s["da"]; s2 = dict(s); s2["da"] = -fov / n; s2["a0"] = s["a0"] - 0.5 * fov / n; return s2
def cells(w, m):
    if w == "E": return [(126 + k, m) for k in range(6)]
    if w == "W": return [(13 - k, m) for k in range(6)]
    if w == "N": return [(m, 126 + k) for k in range(6)]
    return [(m, 13 - k) for k in range(6)]
for run in RUNS:
    d = EV + "2026-10-11-" + run; par = R.read_params(d); scans = R.read_scans(d); traj = R.read_traj(d)
    G = {}
    for name, sc in (("published", scans), ("H1", [fix(s) for s in scans])):
        L = R.replay(sc, par)["L"]; G[name] = (L, R.grade(L, par, traj))
    S = G["published"][1]["samples"]; obs = G["published"][1]["observable"]
    own = np.array([s[4][0] + str(s[4][1]) for s in S]); kind = np.array([s[4][0] for s in S])
    print(f"\n== {run}")
    for name in ("published", "H1"):
        g = G[name][1]; occ = g["occ"]
        wall_occ = sum(1 for j, i in zip(*np.nonzero(occ)) if min(abs(3 - abs(R.cell_centre(i, j)[0])), abs(3 - abs(R.cell_centre(i, j)[1]))) < 0.2 and R.true_dist(*R.cell_centre(i, j)) < 0.2 and min(np.hypot(R.cell_centre(i, j)[0] - px, R.cell_centre(i, j)[1] - py) for px, py in R.PILLARS) > 0.4)
        print(f"  {name:9s} occupied cells {int(occ.sum()):4d} (wall cells {wall_occ:4d}); found: walls {int(g['found'][obs & (kind=='w')].sum())}/{int((obs & (kind=='w')).sum())}, pillars {int(g['found'][obs & (kind=='p')].sum())}/{int((obs & (kind=='p')).sum())}; completeness {g['completeness']:.4f}")
    fb, fa = G["published"][1]["found"], G["H1"][1]["found"]
    lost = obs & fb & ~fa; gained = obs & ~fb & fa
    print(f"  samples that stopped being found: {int(lost.sum())} {dict(collections.Counter(own[lost]))}; started being found: {int(gained.sum())} {dict(collections.Counter(own[gained]))}")
    for name in ("published", "H1"):
        occ = G[name][1]["occ"]; line = []
        for w in "ENWS":
            pat = collections.Counter()
            for m in range(14, 126):
                s = "".join("X" if occ[j, i] else "." for i, j in cells(w, m)); pat[s[3:]] += 1
            line.append(f"{w}: " + " ".join(f"{k}:{v}" for k, v in pat.most_common(4)))
        print(f"  wall cross-section [inner, outer, outer+1], {name:9s} " + " | ".join(line))
