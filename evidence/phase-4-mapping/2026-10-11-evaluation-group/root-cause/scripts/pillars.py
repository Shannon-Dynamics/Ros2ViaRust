import sys, pickle, math, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
BINS = [(-1, 0.2), (0.2, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 1.01)]
for run in RUNS:
    scans, par, out = pickle.load(open(f"replay_{run}.pkl","rb")); g = pickle.load(open(f"grade_{run}.pkl","rb"))
    S = g["samples"]; spos = np.array([[s["x"], s["y"]] for s in scans])
    vis = R.visibility(S, spos)
    tot = np.zeros(len(BINS), int); miss = np.zeros(len(BINS), int)
    for n, s in enumerate(S):
        if s[4][0] != "p" or not g["observable"][n]: continue
        if not vis[n].any():
            continue
        v = spos[vis[n]]
        dx, dy = v[:, 0] - s[0], v[:, 1] - s[1]
        cosb = float(np.max((s[2] * dx + s[3] * dy) / np.hypot(dx, dy)))   # best (most head-on) view from any integrated pose
        for b, (lo, hi) in enumerate(BINS):
            if lo <= cosb < hi:
                tot[b] += 1; miss[b] += (not g["found"][n]); break
    print(f"\n{run}: pillar samples (observable, seen by >=1 integrated scan) by BEST cos(incidence) over all integrated scans that see them")
    for b, (lo, hi) in enumerate(BINS):
        print(f"   best cos in [{lo:.1f},{min(hi,1.0):.1f}): {tot[b]:4d} samples, missed {miss[b]:4d} = {miss[b]/max(1,tot[b]):.2f}")
