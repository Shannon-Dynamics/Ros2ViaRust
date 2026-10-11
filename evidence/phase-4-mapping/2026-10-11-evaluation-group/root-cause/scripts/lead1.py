import sys, pickle, math, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
NB = [(0,0),(1,0),(-1,0),(0,1),(0,-1)]
for run in RUNS:
    scans, par, out = pickle.load(open(f"replay_{run}.pkl","rb")); g = pickle.load(open(f"grade_{run}.pkl","rb"))
    S = g["samples"]; thr = float(par["occupied_threshold"])
    spos = np.array([[s["x"], s["y"]] for s in scans])
    visS = R.visibility(S, spos)          # visibility from the poses scans were INTEGRATED at
    L, n_occ, n_free = out["L"], out["n_occ"], out["n_free"]
    cats = collections.Counter(); byowner = collections.Counter(); rows = []
    for n, s in enumerate(S):
        if not g["observable"][n] or g["found"][n]: continue
        i, j = R.nearest_cell(s[0], s[1])
        nbr = [(i+di, j+dj) for di, dj in NB if 0 <= i+di < R.W and 0 <= j+dj < R.H]
        Lmax = max(float(L[b, a]) for a, b in nbr)
        occ_max = max(int(n_occ[b, a]) for a, b in nbr); free_sum = max(int(n_free[b, a]) for a, b in nbr)
        nvis = int(visS[n].sum()); nvis_traj = int(g["vis"][n].sum())
        if nvis == 0: c = "A: visible only from poses no scan was integrated at"
        elif occ_max == 0: c = "B: seen by integrated scans, 0 scans marked it/neighbour occupied"
        elif occ_max < 4: c = "C: marked occupied by 1-3 scans (fewer than the 4 needed)"
        else: c = "D: marked occupied by >=4 scans, net log-odds still below threshold"
        cats[c] += 1; byowner[s[4][0] + str(s[4][1])] += 1
        rows.append((s, nvis, nvis_traj, occ_max, free_sum, Lmax))
    n_obs = g["n_obs"]; miss = sum(cats.values())
    print(f"\n== {run}: observable {n_obs}, missed {miss} ({miss/n_obs:.4f}); integrated scans {len(scans)}")
    for c, v in sorted(cats.items()): print(f"   {v:4d}  {c}")
    print("   missed by owner:", dict(sorted(byowner.items())))
    if rows:
        arr = lambda k: np.array([r[k] for r in rows])
        print("   per missed sample: scans able to see it (integrated poses) min/median/max %d/%d/%d; trajectory positions that see it median %d; max scans that marked a 5-cell neighbourhood cell occupied: median %d max %d; best final log-odds in the neighbourhood: median %.2f max %.2f (threshold %.2f)" % (
            arr(1).min(), np.median(arr(1)), arr(1).max(), np.median(arr(2)), np.median(arr(3)), arr(3).max(), np.median(arr(5)), arr(5).max(), thr))
    # comparison population: found observable samples
    fo = [n for n in range(len(S)) if g["observable"][n] and g["found"][n]]
    nv = np.array([visS[n].sum() for n in fo])
    print("   found samples: scans able to see them median %d, p5 %d" % (np.median(nv), np.percentile(nv, 5)))
    pickle.dump(rows, open(f"missed_{run}.pkl","wb"))
