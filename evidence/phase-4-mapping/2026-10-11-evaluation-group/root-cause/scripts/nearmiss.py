import sys, pickle, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
for run in RUNS:
    scans, par, out = pickle.load(open(f"replay_{run}.pkl","rb")); g = pickle.load(open(f"grade_{run}.pkl","rb"))
    S = g["samples"]; spos = np.array([[s["x"], s["y"]] for s in scans]); vis = R.visibility(S, spos); D = g["D"]
    cnt = {"pillar>=0.8": collections.Counter(), "wall": collections.Counter()}
    for n, s in enumerate(S):
        if not g["observable"][n] or g["found"][n] or not vis[n].any(): continue
        v = spos[vis[n]]; dx, dy = v[:,0]-s[0], v[:,1]-s[1]
        cosb = float(np.max((s[2]*dx + s[3]*dy)/np.hypot(dx, dy)))
        key = "wall" if s[4][0] == "w" else ("pillar>=0.8" if cosb >= 0.8 else None)
        if key is None: continue
        i, j = R.nearest_cell(s[0], s[1]); d = D[j, i]
        cnt[key]["0.0707 (diagonal)" if abs(d-0.0707107) < 1e-3 else "0.1 (two cells)" if abs(d-0.1) < 1e-3 else f"other {d:.3f}" if d < 1 else "no occupied cell near (>1 m)"] += 1
    print(run.split("2026-10-11-")[-1]); [print(f"   {k:12s} nearest occupied centre from the lookup cell:", dict(c)) for k, c in cnt.items()]
