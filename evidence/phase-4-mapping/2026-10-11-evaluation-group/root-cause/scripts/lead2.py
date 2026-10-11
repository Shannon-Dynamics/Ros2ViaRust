import sys, csv, pickle, math, collections, numpy as np
from scipy import stats
sys.path.insert(0, ".")
import rc_lib as R
import os; EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
TAU = 0.05
for run in RUNS:
    scans, par, out = pickle.load(open(f"replay_{run}.pkl","rb"))
    rows = {round(float(r["stamp_s"]), 6): r for r in csv.DictReader(open(EV + "2026-10-11-" + run + "/update-latency.csv")) if r["outcome"] == "integrated"}
    om = np.array([float(rows[round(s["stamp"], 6)]["window_yaw_rad"]) / 0.2 for s in scans])
    thr = np.float32(par["occupied_threshold"]); occ = out["L"] >= thr
    # false-occupied cells: occupied, centre farther than tau from the nearest true surface
    fo = {(i, j) for j in range(R.H) for i in range(R.W) if occ[j, i] and R.true_dist(*R.cell_centre(i, j)) > TAU + 1e-9}
    fo_idx = {j * R.W + i for i, j in fo}
    # per-hit overshoot
    per = collections.defaultdict(lambda: {"b_n": 0, "b_k": 0, "b_e": [], "o_n": 0, "o_k": 0})
    contrib = collections.Counter(); contrib_beam = collections.Counter()
    for (si, bi, z, ex, ey, cell) in out["hits"]:
        e = max(abs(ex) - 3.0, abs(ey) - 3.0)
        if cell in fo_idx:
            contrib[si] += 1; contrib_beam["behind(0-14)" if bi < 15 else "other"] += 1
        if e < -0.15: continue
        p = per[si]; g = "b" if bi < 15 else "o"
        p[g + "_n"] += 1; p[g + "_k"] += (e > 0.045)
        if g == "b": p["b_e"].append(e)
    print(f"\n== {run}: false-occupied cells {len(fo)}; scans that marked any of them occupied: {len(contrib)} of {len(scans)}; beams that did: {dict(contrib_beam)}")
    # 1. scans responsible vs all scans: angular velocity
    cs = np.array(sorted(contrib))
    if len(cs):
        print(f"   |omega| (rad/s) of responsible scans: median {np.median(om[cs]):.4f} p90 {np.percentile(om[cs],90):.4f} max {om[cs].max():.4f}; all integrated scans: median {np.median(om):.4f} p90 {np.percentile(om,90):.4f} max {om.max():.4f}")
        flag = np.zeros(len(scans), bool); flag[cs] = True
        u = stats.mannwhitneyu(om[flag], om[~flag]); print(f"   Mann-Whitney responsible vs the rest: U={u.statistic:.0f}, p={u.pvalue:.3g}; share of responsible scans with omega<0.01: {np.mean(om[flag]<0.01):.2f}, of the rest: {np.mean(om[~flag]<0.01):.2f}")
    # 2. per-scan behind-beam overshoot against omega (scans with >=5 behind wall hits)
    idx = [si for si, p in per.items() if p["b_n"] >= 5]
    if len(idx) > 10:
        rate = np.array([per[si]["b_k"] / per[si]["b_n"] for si in idx]); me = np.array([np.median(per[si]["b_e"]) for si in idx]); w = om[idx]
        r1 = stats.spearmanr(w, rate); r2 = stats.spearmanr(w, me)
        print(f"   per-scan behind-beam (0-14) wall hits, {len(idx)} scans with >=5: overshoot rate mean {rate.mean():.3f} (scans with any overshoot: {np.mean(rate>0):.2f}); Spearman(omega, rate) = {r1.statistic:+.3f} (p={r1.pvalue:.2g}); Spearman(omega, median signed overshoot) = {r2.statistic:+.3f} (p={r2.pvalue:.2g})")
        # rate by omega bin
        for lo, hi in [(0, 0.005), (0.005, 0.02), (0.02, 0.08)]:
            m = (w >= lo) & (w < hi)
            if m.sum(): print(f"      omega in [{lo},{hi}): {m.sum():4d} scans, mean behind-beam overshoot rate {rate[m].mean():.3f}")
    pickle.dump(dict(per=dict(per), om=om, contrib=contrib, fo=fo), open(f"lead2_{run}.pkl", "wb"))
