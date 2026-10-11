import sys, pickle, math, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
OUT = [0.0, math.pi, math.pi / 2, -math.pi / 2]            # outward normal angle of walls x=+3, x=-3, y=+3, y=-3
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def collect(run):
    scans, par, out = pickle.load(open(f"replay_{run}.pkl", "rb"))
    rows = []
    for si, s in enumerate(scans):
        n = len(s["ranges"]); idx = np.arange(n); th = s["th"] + s["a0"] + idx * s["da"]
        c, sn = np.cos(th), np.sin(th)
        with np.errstate(divide="ignore", invalid="ignore"):
            tw = np.stack([(3 - s["x"]) / c, (-3 - s["x"]) / c, (3 - s["y"]) / sn, (-3 - s["y"]) / sn])
        tw = np.where(tw > 0, tw, np.inf); k = np.argmin(tw, axis=0); t = tw[k, idx]
        hx, hy = s["x"] + t * c, s["y"] + t * sn
        # reject rays that hit a pillar first (measured much shorter) or leave the room via a corner region
        pil = np.zeros(n, bool)
        for px, py in R.PILLARS:
            bx, by = s["x"] - px, s["y"] - py; b = bx * c + by * sn; cc = bx * bx + by * by - R.R_P ** 2; disc = b * b - cc
            tp = -b - np.sqrt(np.where(disc >= 0, disc, 0)); pil |= (disc >= 0) & (tp > 0) & (tp < t + 0.2)
        alpha = np.array([wrap(th[i] - OUT[k[i]]) for i in range(n)])
        z = s["ranges"].astype(float)
        ok = np.isfinite(z) & (z > s["rmin"]) & (z < s["rmax"]) & np.isfinite(t) & (t < 3.4) & ~pil & (np.abs(z - t) < 0.2)
        rows.append((idx[ok], t[ok], z[ok] - t[ok], alpha[ok], np.full(ok.sum(), si)))
    return [np.concatenate(c) for c in zip(*rows)]
print("angular error eps (mrad) per group of 15 beams, fitted as  residual = range * tan(alpha) * eps  (alpha: signed incidence from the wall's outward normal), wall beams with |alpha| > 10 deg and |residual| < 0.2 m")
print("a constant mount yaw error would give the same eps in every group; a seam effect would not\n")
res_all = {}
for run in RUNS:
    B, T, RES, AL, SI = collect(run)
    res_all[run] = (B, T, RES, AL, SI)
    m = np.abs(np.degrees(AL)) > 10
    x = T * np.tan(AL)
    line = []
    for g in range(24):
        sel = m & (B >= 15 * g) & (B < 15 * (g + 1))
        if sel.sum() < 150: line.append("   --"); continue
        eps = (RES[sel] * x[sel]).sum() / (x[sel] ** 2).sum()
        line.append(f"{eps*1000:+5.0f}")
    print(f"{run.split('2026-10-11-')[-1][:24]:24s} " + " ".join(line))
print("\n" + " " * 25 + " ".join(f"{15*g:>5d}" for g in range(24)) + "   <- first beam index of the group (0 = straight behind, 180 = straight ahead)")
pickle.dump(res_all, open("warp.pkl", "wb"))
