"""Second half of the capped pillar diagnostic: is the final state of a well-viewed pillar cell decided by the last few scans (path dependence near the clamp)?
(a) grade each H1-corrected map at snapshots taken d scans before the end of the run; (b) trace the state history of the best cell of every well-viewed pillar sample.
Python port, frozen parameters, unchanged evaluator port."""
import sys, os, math, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
NB = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]
DS = [0, 10, 20, 40, 80, 160]
def fix(s):
    n = len(s["ranges"]); fov = -(n - 1) * s["da"]; s2 = dict(s); s2["da"] = -fov / n; s2["a0"] = s["a0"] - 0.5 * fov / n; return s2
def replay_snap(scans, par, tracked, snaps, clear_margin=0.10):
    l_occ, l_free = np.float32(par["l_occ"]), np.float32(par["l_free"]); l_min, l_max = np.float32(par["l_min"]), np.float32(par["l_max"])
    L = np.zeros(R.W * R.H, np.float32); pos = {c: k for k, c in enumerate(tracked)}; log = np.zeros((len(scans), len(tracked)), np.int8); out = {}
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
        if (len(scans) - 1 - si) in snaps: out[len(scans) - 1 - si] = L.reshape(R.H, R.W).copy()
    return L.reshape(R.H, R.W), log, out
print("(a) the same run graded d scans before its end (H1-corrected beams): completeness / false-free %, whole map; and the well-viewed pillar samples (a trajectory position within 15 deg) found")
hdr = " ".join(f"d={d:<3d}".rjust(22) for d in DS)
print(f"{'run':27s} {hdr}")
summ = []
for run in RUNS:
    d0 = EV + "2026-10-11-" + run; par = R.read_params(d0); scans = [fix(s) for s in R.read_scans(d0)]; traj = R.read_traj(d0)
    S = R.make_samples(0.01); vis = R.visibility(S, traj)
    ang = np.array([np.nan if not vis[n].any() else math.degrees(math.acos(min(1.0, float(np.max((S[n][2] * (traj[vis[n]][:, 0] - S[n][0]) + S[n][3] * (traj[vis[n]][:, 1] - S[n][1])) / np.hypot(traj[vis[n]][:, 0] - S[n][0], traj[vis[n]][:, 1] - S[n][1])))))) for n in range(len(S))])
    pill = np.array([s[4][0] == "p" for s in S]); wv = pill & ~np.isnan(ang) & (ang <= 15)
    pidx = np.nonzero(pill)[0]
    tracked = sorted({(i, j) for n in pidx for (a, b) in [R.nearest_cell(S[n][0], S[n][1])] for di, dj in NB for (i, j) in [(a + di, b + dj)] if 0 <= i < R.W and 0 <= j < R.H})
    L, log, snaps = replay_snap(scans, par, tracked, set(DS), 0.10)
    cells = []; wvfound = []
    for dd in DS:
        g = R.grade(snaps[dd], par, traj); cells.append(f"{g['completeness']:.4f}/{g['n_false_free']/g['n_obs']*100:.2f}".rjust(22)); wvfound.append(f"{int(g['found'][wv & g['observable']].sum())}/{int((wv & g['observable']).sum())}".rjust(22))
    print(f"{run:27s} " + " ".join(cells)); print(f"{'  well-viewed pillar found':27s} " + " ".join(wvfound), flush=True)
    summ.append((run, S, wv, tracked, log, par, scans, L, R.grade(L, par, traj)))
print("\n(b) state history of the best cell (lookup cell or 4-neighbour with the most occupied marks) of each well-viewed pillar sample; occupied = log-odds >= the occupied threshold")
print(f"{'run':27s} {'':7s} {'n':>4s} {'ever occ.':>9s} {'share of updates occupied':>26s} {'state changes':>14s} {'updates since last occupied':>28s}")
l_occ0 = None
for run, S, wv, tracked, log, par, scans, L, g in summ:
    pos = {c: k for k, c in enumerate(tracked)}; lo, lf, lmin, lmax = (np.float32(par[k]) for k in ("l_occ", "l_free", "l_min", "l_max")); thr = np.float32(par["occupied_threshold"])
    rows = {"found": [], "missed": []}
    for n in np.nonzero(wv & g["observable"])[0]:
        a, b = R.nearest_cell(S[n][0], S[n][1]); cs = [(a + di, b + dj) for di, dj in NB if (a + di, b + dj) in pos]
        best = max(cs, key=lambda c: int((log[:, pos[c]] == 2).sum())); m = log[:, pos[best]]
        l = np.float32(0); st = []
        for x in m:
            if x == 2: l = np.clip(np.float32(l + lo), lmin, lmax)
            elif x == 1: l = np.clip(np.float32(l + lf), lmin, lmax)
            if x: st.append(bool(l >= thr))
        st = np.array(st)
        if len(st) == 0: continue
        changes = int((st[1:] != st[:-1]).sum()); ever = bool(st.any()); share = float(st[np.argmax(st):].mean()) if ever else 0.0
        since = (len(st) - 1 - int(np.nonzero(st)[0].max())) if ever else len(st)
        rows["found" if g["found"][n] else "missed"].append((ever, share, changes, since))
    for k, v in rows.items():
        if not v: print(f"{run:27s} {k:7s}    0"); continue
        a = np.array(v, float); print(f"{run:27s} {k:7s} {len(v):4d} {a[:,0].mean():9.2f} {np.median(a[:,1]):26.2f} {np.median(a[:,2]):14.0f} {np.median(a[:,3]):28.0f}")
