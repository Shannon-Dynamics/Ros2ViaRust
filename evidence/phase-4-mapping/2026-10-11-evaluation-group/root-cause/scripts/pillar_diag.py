"""Capped diagnostic pass on the open item: why does the route, not the incidence angle, decide whether a well-viewed pillar sample is found?
(route_long 1.00 found at 5-14 degrees, the evaluation routes 0.56.)  Python port, frozen parameters, unchanged evaluator port, beam angles corrected by H1
(the layout the next runs will use).  Candidates tested:
  C1 evaluator-side: 'well viewed' by a trajectory position at which no scan was integrated (robot turning, gated, stationary repeats)
  C2 scan density / view distance near the pillar
  C3 cancellation by views of the same cell from other directions (per-scan outcome of the cell, by the angle the scan saw the sample at)
"""
import sys, os, math, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
NB = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]
def fix(s):
    n = len(s["ranges"]); fov = -(n - 1) * s["da"]; s2 = dict(s); s2["da"] = -fov / n; s2["a0"] = s["a0"] - 0.5 * fov / n; return s2

def replay_log(scans, par, tracked, clear_margin=0.10):
    """H1-corrected replay that also logs, for every scan, the mark (0 none, 1 free, 2 occupied) of the tracked cells."""
    l_occ, l_free = np.float32(par["l_occ"]), np.float32(par["l_free"]); l_min, l_max = np.float32(par["l_min"]), np.float32(par["l_max"])
    L = np.zeros(R.W * R.H, np.float32); pos = {c: k for k, c in enumerate(tracked)}; log = np.zeros((len(scans), len(tracked)), np.int8)
    for si, s in enumerate(scans):
        marks = {}
        for bi, raw in enumerate(s["ranges"]):
            z = float(raw); th = s["th"] + s["a0"] + bi * s["da"]; c, sn = math.cos(th), math.sin(th)
            if math.isnan(z) or z <= 0.0 or (math.isfinite(z) and z < s["rmin"]): continue
            if math.isfinite(z) and z < s["rmax"]: ex, ey, hit = s["x"] + c * z, s["y"] + sn * z, True
            else:
                reach = s["rmax"] - clear_margin
                if not reach > s["rmin"]: continue
                ex, ey, hit = s["x"] + c * reach, s["y"] + sn * reach, False
            cells = list(R.supercover(s["x"], s["y"], ex, ey))
            for k, (i, j) in enumerate(cells):
                if not (0 <= i < R.W and 0 <= j < R.H): break
                idx = j * R.W + i
                if k == len(cells) - 1:
                    if hit: marks[idx] = 2
                elif marks.get(idx, 0) < 1: marks[idx] = 1
        for idx, m in marks.items():
            L[idx] = np.clip(np.float32(L[idx] + (l_occ if m == 2 else l_free)), l_min, l_max)
            c = (idx % R.W, idx // R.W)
            if c in pos: log[si, pos[c]] = m
    return L.reshape(R.H, R.W), log

def angle_to(samples_xyz, positions):
    """matrix [n_samples, n_positions] of the angle (deg) between the sample's normal and the direction to the position, NaN where not visible"""
    return None

R_ = {}
for run in RUNS:
    d = EV + "2026-10-11-" + run; par = R.read_params(d); scans = R.read_scans(d); traj = R.read_traj(d)
    S = R.make_samples(0.01); pidx = [n for n, s in enumerate(S) if s[4][0] == "p"]
    Sp = [S[n] for n in pidx]
    tracked = sorted({(i, j) for s in Sp for (a, b) in [R.nearest_cell(s[0], s[1])] for di, dj in NB for (i, j) in [(a + di, b + dj)] if 0 <= i < R.W and 0 <= j < R.H})
    L, log = replay_log([fix(s) for s in scans], par, tracked)
    g = R.grade(L, par, traj); found = g["found"][pidx]; obs = g["observable"][pidx]
    spos = np.array([[s["x"], s["y"]] for s in scans]); vis_s = R.visibility(Sp, spos); vis_t = R.visibility(Sp, traj)
    def ang_mat(vis, pos):
        A = np.full(vis.shape, np.nan)
        for n, s in enumerate(Sp):
            m = vis[n]
            if m.any():
                v = pos[m]; dx, dy = v[:, 0] - s[0], v[:, 1] - s[1]; dd = np.hypot(dx, dy)
                A[n, m] = np.degrees(np.arccos(np.clip((s[2] * dx + s[3] * dy) / dd, -1, 1)))
        return A
    A_s, A_t = ang_mat(vis_s, spos), ang_mat(vis_t, traj)
    D_s = np.full(vis_s.shape, np.nan)
    for n, s in enumerate(Sp):
        m = vis_s[n]
        if m.any(): D_s[n, m] = np.hypot(spos[m][:, 0] - s[0], spos[m][:, 1] - s[1])
    R_[run] = dict(Sp=Sp, found=found, obs=obs, A_s=A_s, A_t=A_t, D_s=D_s, log=log, tracked=tracked, ntraj=len(traj), nscan=len(scans), vis_s=vis_s)
    print("replayed", run, flush=True)

print("\nC1  EVALUATOR-SIDE COUNTING: pillar samples that a trajectory position sees within 15 degrees of the normal ('well viewed' by the evaluator)")
print(f"{'run':27s} {'well viewed (traj)':>18s} {'of those: found':>16s} {'with NO integrated scan within 15 deg':>38s} {'found among those':>18s} {'found among the rest':>21s}")
for run in RUNS:
    r = R_[run]; wt = np.nanmin(np.where(np.isnan(r["A_t"]), 999, r["A_t"]), axis=1) <= 15; ws = np.nanmin(np.where(np.isnan(r["A_s"]), 999, r["A_s"]), axis=1) <= 15
    sel = wt & r["obs"]; no_scan = sel & ~ws; rest = sel & ws
    f = lambda m: f"{int(r['found'][m].sum())}/{int(m.sum())}" if m.sum() else "-"
    print(f"{run:27s} {int(sel.sum()):18d} {f(sel):>16s} {int(no_scan.sum()):38d} {f(no_scan):>18s} {f(rest):>21s}")

print("\nC2  SCAN DENSITY AND DISTANCE for the samples that ARE seen by an integrated scan within 15 degrees: found against missed")
print(f"{'run':27s} {'':8s} {'n':>4s} {'scans <=15deg':>14s} {'scans all views':>16s} {'share >60deg':>13s} {'median range <=15deg (m)':>26s}")
for run in RUNS:
    r = R_[run]; ws = np.nanmin(np.where(np.isnan(r["A_s"]), 999, r["A_s"]), axis=1) <= 15
    for name, m in (("found", ws & r["obs"] & r["found"]), ("missed", ws & r["obs"] & ~r["found"])):
        if not m.sum(): print(f"{run:27s} {name:8s}    0"); continue
        n15 = [int((r["A_s"][n] <= 15).sum()) for n in np.nonzero(m)[0]]; nall = [int((~np.isnan(r["A_s"][n])).sum()) for n in np.nonzero(m)[0]]
        graz = [float((r["A_s"][n] > 60).sum() / max(1, (~np.isnan(r["A_s"][n])).sum())) for n in np.nonzero(m)[0]]
        rng = [float(np.nanmedian(r["D_s"][n][r["A_s"][n] <= 15])) for n in np.nonzero(m)[0]]
        print(f"{run:27s} {name:8s} {int(m.sum()):4d} {np.median(n15):14.0f} {np.median(nall):16.0f} {np.median(graz):13.2f} {np.median(rng):26.2f}")

print("\nC3  PER-SCAN OUTCOME OF THE SAMPLE'S BEST CELL (the lookup cell or 4-neighbour with the most occupied marks), by the angle at which that scan sees the sample")
print("    share of scans that saw the sample in which the cell was marked OCCUPIED / FREE / left untouched; well-viewed samples (integrated scan within 15 deg), found vs missed")
bins = [(0, 15), (15, 30), (30, 60), (60, 90)]
for run in RUNS:
    r = R_[run]; ws = np.nanmin(np.where(np.isnan(r["A_s"]), 999, r["A_s"]), axis=1) <= 15; pos = {c: k for k, c in enumerate(r["tracked"])}
    print(f"  {run}")
    for name, m in (("found", ws & r["obs"] & r["found"]), ("missed", ws & r["obs"] & ~r["found"])):
        cnt = {b: np.zeros(3) for b in bins}
        for n in np.nonzero(m)[0]:
            s = r["Sp"][n]; a, b = R.nearest_cell(s[0], s[1]); cells = [(a + di, b + dj) for di, dj in NB if (a + di, b + dj) in pos]
            best = max(cells, key=lambda c: int((r["log"][:, pos[c]] == 2).sum()))
            for si in np.nonzero(~np.isnan(r["A_s"][n]))[0]:
                ang = r["A_s"][n, si]
                for bb in bins:
                    if bb[0] <= ang < bb[1]: cnt[bb][r["log"][si, pos[best]] if r["log"][si, pos[best]] in (0, 1, 2) else 0] += 1; break
        if m.sum():
            print(f"    {name:7s} n={int(m.sum()):3d}  " + "  ".join(f"{b[0]}-{b[1]}deg: occ {cnt[b][2]/max(1,cnt[b].sum()):.2f} free {cnt[b][1]/max(1,cnt[b].sum()):.2f} none {cnt[b][0]/max(1,cnt[b].sum()):.2f} (n={int(cnt[b].sum())})" for b in bins))
