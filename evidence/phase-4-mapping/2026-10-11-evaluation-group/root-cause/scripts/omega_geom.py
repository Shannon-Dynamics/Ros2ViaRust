import pickle, math, numpy as np
from math import erf, sqrt
d = pickle.load(open("warp.pkl", "rb")); runs = list(d)
Phi = np.vectorize(lambda z: 0.5 * (1 + erf(z / sqrt(2)))); BIN = 5
def eps_table(train):
    B = np.concatenate([d[r][0] for r in train]); T = np.concatenate([d[r][1] for r in train])
    RES = np.concatenate([d[r][2] for r in train]); AL = np.concatenate([d[r][3] for r in train])
    m = np.abs(np.degrees(AL)) > 10; x = T * np.tan(AL); tab = np.zeros(72)
    for g in range(72):
        s = m & (B // BIN == g); tab[g] = (RES[s] * x[s]).sum() / (x[s] ** 2).sum() if s.sum() >= 80 else 0.0
    return tab
r = "evaluation-3-diamond-slow"
tab = eps_table([q for q in runs if q != r]); B, T, RES, AL, SI = d[r]
om = pickle.load(open("lead2_evaluation-3-diamond-slow.pkl", "rb"))["om"]
e_pred = T * np.sin(AL) * tab[B // BIN]; p_pred = 1 - Phi((0.045 - e_pred) / 0.0155); obs = (RES * np.cos(AL)) > 0.045
sel = B < 15
print("diamond run, beams 0-14 wall hits: observed and model-predicted overshoot rate by the scan's |omega| (the model has no velocity term)")
for lo, hi in [(0, 0.005), (0.005, 0.02), (0.02, 0.08)]:
    m = sel & (om[SI] >= lo) & (om[SI] < hi)
    print(f"   omega in [{lo},{hi}): {m.sum():5d} hits in {len(np.unique(SI[m])):3d} scans: observed {obs[m].mean()*100:5.1f}%  predicted by beam-angle model {p_pred[m].mean()*100:5.1f}%")
# finite-difference vs the mapper's own window value, on pairs of consecutive integrated scans
import csv
import os; EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/2026-10-11-evaluation-3-diamond-slow/"
scans, par, out = pickle.load(open("replay_evaluation-3-diamond-slow.pkl", "rb"))
st = np.array([s["stamp"] for s in scans]); th = np.unwrap(np.array([s["th"] for s in scans]))
dt = np.diff(st); fd = np.abs(np.diff(th)) / dt
ok = dt <= 0.25
print(f"finite difference |dtheta|/dt between consecutive integrated scans: {ok.sum()} of {len(dt)} pairs have dt <= 0.25 s; for those, median {np.median(fd[ok]):.4f}, p95 {np.percentile(fd[ok],95):.4f}, max {fd[ok].max():.4f} rad/s; the {(~ok).sum()} other pairs span {dt[~ok].min():.2f} to {dt[~ok].max():.1f} s (across skipped turns), where it would report up to {fd[~ok].max():.2f} rad/s over time the robot was not integrated")
