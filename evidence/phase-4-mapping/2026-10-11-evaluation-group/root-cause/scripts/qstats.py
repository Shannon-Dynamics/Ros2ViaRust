import sys, pickle, math, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
NB = [(0,0),(1,0),(-1,0),(0,1),(0,-1)]
BE = 0.405465/(0.847298+0.405465)
print("break-even occupied share q* = l_free/(l_occ+l_free) = %.3f" % BE)
for run in RUNS:
    scans, par, out = pickle.load(open(f"replay_{run}.pkl","rb")); g = pickle.load(open(f"grade_{run}.pkl","rb"))
    S = g["samples"]; spos = np.array([[s["x"], s["y"]] for s in scans]); vis = R.visibility(S, spos)
    n_occ, n_free = out["n_occ"], out["n_free"]
    res = {"wall": [], "pillar": []}
    for n, s in enumerate(S):
        if not g["observable"][n] or not vis[n].any(): continue
        i, j = R.nearest_cell(s[0], s[1])
        nbr = [(i+di, j+dj) for di, dj in NB if 0 <= i+di < R.W and 0 <= j+dj < R.H]
        # the neighbourhood cell with the largest occupied share q = n_occ / (n_occ + n_free)
        best = max(((int(n_occ[b,a]) / max(1, int(n_occ[b,a]) + int(n_free[b,a])), int(n_occ[b,a]), int(n_free[b,a])) for a, b in nbr))
        v = spos[vis[n]]; dx, dy = v[:,0]-s[0], v[:,1]-s[1]; d = np.hypot(dx, dy); cosb = (s[2]*dx + s[3]*dy)/d
        good = cosb >= 0.8
        res["pillar" if s[4][0] == "p" else "wall"].append((g["found"][n], best[0], best[1], best[2], good.sum(), d[good].min() if good.any() else np.nan, np.median(d[good]) if good.any() else np.nan))
    print("\n==", run)
    for kind in ("wall", "pillar"):
        a = np.array(res[kind], dtype=float)
        f, m = a[a[:,0] == 1], a[a[:,0] == 0]
        def line(t, name):
            if len(t) == 0: print(f"   {kind:6s} {name}: none"); return
            print(f"   {kind:6s} {name}: n={len(t):4d}  best-neighbour occupied share q: median {np.median(t[:,1]):.2f}, share below break-even {np.mean(t[:,1] < BE):.2f}; marks of that cell: occupied median {np.median(t[:,2]):.0f}, freed median {np.median(t[:,3]):.0f}")
        line(f, "found "); line(m, "missed")
    # pillar samples with a good (cos>=0.8) view: missed share by median distance of the good views
    a = np.array(res["pillar"], dtype=float); a = a[a[:,4] > 0]
    print("   pillar samples with >=1 good view (cos>=0.8): missed share by MEDIAN distance of their good views")
    for lo, hi in [(0,1.0),(1.0,1.5),(1.5,2.0),(2.0,2.5),(2.5,3.0),(3.0,3.6)]:
        t = a[(a[:,6] >= lo) & (a[:,6] < hi)]
        if len(t): print(f"      {lo:.1f}-{hi:.1f} m: {len(t):4d} samples, missed {np.mean(t[:,0]==0):.2f}")
