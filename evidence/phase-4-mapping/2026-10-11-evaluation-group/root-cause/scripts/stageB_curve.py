"""Stage B gate, question 2: found-fraction against incidence angle, pillar samples (and walls for contrast).
The angle of a sample is the best (smallest) angle between its surface normal and the direction to any ground-truth trajectory position that sees it
(line of sight, within 3.5 m, lit side): the quantity an evaluator rule could compute. Maps: published beam angles, and beam angles corrected by H1 (pixel centres).
Python port, frozen parameters, unchanged evaluator port."""
import sys, os, math, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
def fix(s):
    n = len(s["ranges"]); fov = -(n - 1) * s["da"]; s2 = dict(s); s2["da"] = -fov / n; s2["a0"] = s["a0"] - 0.5 * fov / n; return s2
BINS = list(range(0, 90, 5))
res = {}
for run in RUNS:
    d = EV + "2026-10-11-" + run; par = R.read_params(d); scans = R.read_scans(d); traj = R.read_traj(d)
    S = R.make_samples(0.01); vis = R.visibility(S, traj)
    ang = np.full(len(S), np.nan)
    for n, s in enumerate(S):
        if vis[n].any():
            v = traj[vis[n]]; dx, dy = v[:, 0] - s[0], v[:, 1] - s[1]
            ang[n] = math.degrees(math.acos(min(1.0, float(np.max((s[2] * dx + s[3] * dy) / np.hypot(dx, dy))))))
    out = {}
    for name, sc in (("published", scans), ("H1", [fix(s) for s in scans])):
        L = R.replay(sc, par)["L"]; g = R.grade(L, par, traj); out[name] = g["found"].copy(); out["comp_" + name] = g["completeness"]
    res[run] = dict(ang=ang, owner=[s[4][0] for s in S], found=out)
    print("replayed", run, flush=True)
def curve(maps, kind, runs):
    rows = []
    for lo in BINS:
        n = f = 0
        for r in runs:
            a = res[r]["ang"]; own = np.array(res[r]["owner"]); fo = res[r]["found"][maps]
            m = (own == kind) & (a >= lo) & (a < lo + 5); n += int(m.sum()); f += int(fo[m].sum())
        rows.append((lo, n, f))
    return rows
print("\nPILLAR samples: found fraction by best viewing angle (degrees from the surface normal), 5-degree bins; n and found counts are summed over the runs listed")
for title, runs in (("all four runs pooled (the same pillars appear in each run, so the runs are not independent)", RUNS), ("the three evaluation routes", RUNS[1:]), ("route_long only", RUNS[:1])):
    pub, h1 = curve("published", "p", runs), curve("H1", "p", runs)
    print(f"\n  {title}\n  {'angle':>9s} {'n':>5s} {'found, published':>17s} {'found, H1':>11s}")
    for (lo, n, f), (_, _, f2) in zip(pub, h1):
        if n: print(f"  {lo:3d}-{lo+4:<4d}  {n:5d} {f:6d} = {f/n:5.2f}   {f2:5d} = {f2/n:5.2f}")
print("\nPer run, pillar samples, found fraction by 15-degree band (published / H1):")
print(f"  {'run':27s} " + " ".join(f"{lo:2d}-{lo+14:<2d}".rjust(16) for lo in range(0, 90, 15)))
for r in RUNS:
    cells = []
    for lo in range(0, 90, 15):
        a = res[r]["ang"]; own = np.array(res[r]["owner"]); m = (own == "p") & (a >= lo) & (a < lo + 15)
        cells.append((f"{res[r]['found']['published'][m].sum()}/{m.sum()} {res[r]['found']['H1'][m].sum()}/{m.sum()}" if m.sum() else "-").rjust(16))
    print(f"  {r:27s} " + " ".join(cells))
print("\nWALL samples for contrast (pooled, published): found fraction by best viewing angle, 15-degree bands")
for lo in range(0, 90, 15):
    n = f = 0
    for r in RUNS:
        a = res[r]["ang"]; own = np.array(res[r]["owner"]); m = (own == "w") & (a >= lo) & (a < lo + 15); n += int(m.sum()); f += int(res[r]["found"]["published"][m].sum())
    print(f"  {lo:2d}-{lo+14:<2d}: {f}/{n}" + (f" = {f/n:.3f}" if n else ""))
print("\nCompleteness if samples whose best angle exceeds a limit were not counted as observable (INFORMATION ONLY: a limit chosen to make a run pass would be tuning the metric)")
print(f"  {'limit':>6s} " + " ".join(f"{r.split('2026-10-11-')[-1][:14]:>22s}" for r in RUNS))
for lim in (30, 40, 50, 60, 70, 80, 90):
    cells = []
    for r in RUNS:
        a = res[r]["ang"]; obs = ~np.isnan(a) & (a <= lim)
        cells.append(f"{res[r]['found']['published'][obs].sum()/obs.sum():.4f} / {res[r]['found']['H1'][obs].sum()/obs.sum():.4f}".rjust(22))
    print(f"  {lim:5d}  " + " ".join(cells))
print("  (each cell: published / H1)")
