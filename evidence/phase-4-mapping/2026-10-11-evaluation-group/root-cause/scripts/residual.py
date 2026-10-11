import sys, pickle, math, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]

def raycast_true(x0, y0, th):
    """range to the nearest true surface along each direction th (array) from (x0,y0), plus the surface normal angle-incidence cosine and kind."""
    c, s = np.cos(th), np.sin(th)
    best = np.full(th.shape, np.inf); cosinc = np.zeros(th.shape); kind = np.full(th.shape, -1)
    with np.errstate(divide="ignore", invalid="ignore"):
        for k, (axis, val, nrm) in enumerate([(0, 3.0, 1), (0, -3.0, 1), (1, 3.0, 1), (1, -3.0, 1)]):
            d = c if axis == 0 else s
            t = ((val - x0) / c) if axis == 0 else ((val - y0) / s)
            ok = (t > 0) & np.isfinite(t)
            upd = ok & (t < best)
            best = np.where(upd, t, best); cosinc = np.where(upd, np.abs(d), cosinc); kind = np.where(upd, k, kind)
        for k, (px, py) in enumerate(R.PILLARS):
            bx, by = x0 - px, y0 - py
            b = bx * c + by * s; cc = bx * bx + by * by - R.R_P ** 2
            disc = b * b - cc
            ok = disc >= 0
            t = -b - np.sqrt(np.where(ok, disc, 0))
            ok = ok & (t > 0)
            upd = ok & (t < best)
            hx, hy = x0 + t * c - px, y0 + t * s - py
            ci = np.abs((hx * c + hy * s) / R.R_P)
            best = np.where(upd, t, best); cosinc = np.where(upd, ci, cosinc); kind = np.where(upd, 10 + k, kind)
    return best, cosinc, kind

for run in RUNS:
    scans, par, out = pickle.load(open(f"replay_{run}.pkl", "rb"))
    B, Z, T, CI, K = [], [], [], [], []
    for s in scans:
        th = s["th"] + s["a0"] + np.arange(len(s["ranges"])) * s["da"]
        zt, ci, kd = raycast_true(s["x"], s["y"], th)
        z = s["ranges"].astype(float)
        ok = np.isfinite(z) & (z < s["rmax"]) & (z > s["rmin"]) & (zt < 3.4)
        B.append(np.nonzero(ok)[0]); Z.append(z[ok]); T.append(zt[ok]); CI.append(ci[ok]); K.append(kd[ok])
    B, Z, T, CI, K = map(np.concatenate, (B, Z, T, CI, K))
    res = Z - T
    inc = np.degrees(np.arccos(np.clip(CI, 0, 1)))
    walls = K < 4
    print(f"\n== {run}: {len(res)} beams with a finite return and a true range < 3.4 m; wall beams {walls.sum()}; overall residual (measured - true): mean {res.mean()*1000:+.1f} mm, sd {res.std()*1000:.1f} mm")
    sel = walls
    print("   wall beams: residual mean (mm) / sd (mm) / n, by beam group (rows) and incidence from the wall normal (columns)")
    cols = [(0, 20), (20, 35), (35, 50), (50, 90)]
    print("   group            " + "  ".join(f"inc {a}-{b} deg".rjust(24) for a, b in cols))
    for name, lo, hi in [("beams 0-14", 0, 15), ("beams 15-44", 15, 45), ("beams 45-89", 45, 90), ("beams 90-269", 90, 270), ("beams 270-344", 270, 345), ("beams 345-359", 345, 360)]:
        cells = []
        for a, b in cols:
            m = sel & (B >= lo) & (B < hi) & (inc >= a) & (inc < b)
            cells.append((f"{res[m].mean()*1000:+6.1f} / {res[m].std()*1000:5.1f} / {m.sum():5d}" if m.sum() >= 20 else "         -").rjust(24))
        print(f"   {name:15s}  " + "  ".join(cells))
    pickle.dump(dict(B=B, res=res, inc=inc, K=K, Z=Z, T=T), open(f"resid_{run}.pkl", "wb"))
