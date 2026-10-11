"""Stage B gate, question 1, by intervention: replay with the beam-angle correction applied to different beam zones, or with a uniform angular shift,
and report false-free and completeness (Python port, frozen parameters, unchanged evaluator port). END = beams 0-14 and 345-359, INTERIOR = 15-344."""
import sys, os, math, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUN = sys.argv[1] if len(sys.argv) > 1 else "evaluation-1-cross"
FOVC = lambda s: -(len(s["ranges"]) - 1) * s["da"]
def h1(j, n, fov): return j * (fov / (n - 1) - fov / n) - 0.5 * fov / n          # true - published (rad)
def variant(scans, zone=None, shift=0.0):
    out = []
    for s in scans:
        n = len(s["ranges"]); fov = FOVC(s); s2 = dict(s)
        if zone is not None or shift != 0.0:
            corr = np.array([h1(j, n, fov) if (zone == "all" or (zone == "end" and (j < 15 or j >= 345)) or (zone == "interior" and 15 <= j < 345)) else 0.0 for j in range(n)]) if zone else np.zeros(n)
            s2["ang_corr"] = corr + shift
        out.append(s2)
    return out
def replay_corr(scans, par):
    # same as R.replay, but each beam angle gets its own correction (s["ang_corr"][j]); published angle otherwise
    l_occ, l_free = np.float32(par["l_occ"]), np.float32(par["l_free"]); l_min, l_max = np.float32(par["l_min"]), np.float32(par["l_max"]); L = np.zeros(R.W * R.H, np.float32)
    for s in scans:
        corr = s.get("ang_corr"); marks = {}
        for bi, raw in enumerate(s["ranges"]):
            z = float(raw); th = s["th"] + s["a0"] + bi * s["da"] + (0.0 if corr is None else corr[bi]); c, sn = math.cos(th), math.sin(th)
            if math.isnan(z) or z <= 0.0 or (math.isfinite(z) and z < s["rmin"]): continue
            if math.isfinite(z) and z < s["rmax"]: ex, ey, hit = s["x"] + c * z, s["y"] + sn * z, True
            else:
                reach = s["rmax"] - 0.10
                if not reach > s["rmin"]: continue
                ex, ey, hit = s["x"] + c * reach, s["y"] + sn * reach, False
            cells = list(R.supercover(s["x"], s["y"], ex, ey))
            for k, (i, j) in enumerate(cells):
                if not (0 <= i < R.W and 0 <= j < R.H): break
                idx = j * R.W + i
                if k == len(cells) - 1:
                    if hit: marks[idx] = 2
                elif marks.get(idx, 0) < 1: marks[idx] = 1
        for idx, m in marks.items(): L[idx] = np.clip(np.float32(L[idx] + (l_occ if m == 2 else l_free)), l_min, l_max)
    return L.reshape(R.H, R.W)
d = EV + "2026-10-11-" + RUN; par = R.read_params(d); scans = R.read_scans(d); traj = R.read_traj(d)
print(f"{RUN}: false-free limit 5%, completeness limit 0.95")
print(f"{'variant':44s} {'false-free':>14s} {'complete':>9s} {'false occ':>9s}")
for name, kw in (("published angles", {}), ("H1 on all beams", dict(zone="all")), ("H1 on INTERIOR beams only (15-344)", dict(zone="interior")), ("H1 on END beams only (0-14, 345-359)", dict(zone="end")),
                 ("published + uniform -2 mrad", dict(shift=-0.002)), ("published + uniform -1 mrad", dict(shift=-0.001)), ("published + uniform +1 mrad", dict(shift=0.001)), ("published + uniform +2 mrad", dict(shift=0.002))):
    L = replay_corr(variant(scans, **kw), par); g = R.grade(L, par, traj); occ = g["occ"]
    fo = sum(1 for j, i in zip(*np.nonzero(occ)) if R.true_dist(*R.cell_centre(i, j)) > 0.05 + 1e-9)
    print(f"{name:44s} {g['n_false_free']:5d} ({g['n_false_free']/g['n_obs']*100:5.2f}%) {g['completeness']:9.4f} {fo:9d}", flush=True)
