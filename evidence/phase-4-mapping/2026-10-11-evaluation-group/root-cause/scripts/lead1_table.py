import sys, pickle, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
print("Missed completeness samples, by what kind of sample and how well any integrated scan viewed it")
print("(best cos = cosine between the sample's surface normal and the direction to the best-placed integrated scan pose that sees it)\n")
hdr = ["run", "observable", "missed", "unseen by any scan", "pillar, best cos<0.8", "pillar, best cos>=0.8", "wall, best cos<0.5", "wall, best cos>=0.5", "completeness", "if cos<0.8 pillar samples were not counted (diagnostic only)"]
rows = []
for run in RUNS:
    scans, par, out = pickle.load(open(f"replay_{run}.pkl","rb")); g = pickle.load(open(f"grade_{run}.pkl","rb"))
    S = g["samples"]; spos = np.array([[s["x"], s["y"]] for s in scans]); vis = R.visibility(S, spos)
    c = collections.Counter(); excl = 0
    for n, s in enumerate(S):
        if not g["observable"][n]: continue
        if not vis[n].any():
            if not g["found"][n]: c["unseen"] += 1
            continue
        v = spos[vis[n]]; dx, dy = v[:,0]-s[0], v[:,1]-s[1]
        cosb = float(np.max((s[2]*dx + s[3]*dy)/np.hypot(dx, dy)))
        if s[4][0] == "p" and cosb < 0.8:
            excl += 1
            if not g["found"][n]: c["p_lo"] += 1
        elif not g["found"][n]:
            if s[4][0] == "p": c["p_hi"] += 1
            else: c["w_lo" if cosb < 0.5 else "w_hi"] += 1
    miss = int((g["observable"] & ~g["found"]).sum())
    comp2 = (g["found"][g["observable"]].sum() - (excl - c["p_lo"])) / (g["n_obs"] - excl)
    rows.append([run, g["n_obs"], miss, c["unseen"], c["p_lo"], c["p_hi"], c["w_lo"], c["w_hi"], f"{g['completeness']:.4f}", f"{comp2:.4f}"])
w = [max(len(str(r[k])) for r in rows + [hdr]) for k in range(len(hdr))]
for r in [hdr] + rows: print("  ".join(str(x).ljust(w[k]) for k, x in enumerate(r)))
