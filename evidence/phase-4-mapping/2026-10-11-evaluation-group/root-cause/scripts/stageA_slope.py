"""Stage A: straight-line fit of the beam-angle error over beams 20-339 (the ends are treated separately), with a bootstrap over scans,
against the zero-parameter values derived from the Webots shader and the driver: slope fov/(N-1) - fov/N per beam, intercept -0.5*fov/N."""
import sys, os, math, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
FOV, N = 6.28318, 360
OUT = [0.0, math.pi, math.pi / 2, -math.pi / 2]
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
NUM, DEN = [], []
for run in RUNS:
    for s in R.read_scans(EV + "2026-10-11-" + run):
        n = len(s["ranges"]); idx = np.arange(n); th = s["th"] + s["a0"] + idx * s["da"]
        c, sn = np.cos(th), np.sin(th)
        with np.errstate(divide="ignore", invalid="ignore"):
            tw = np.stack([(3 - s["x"]) / c, (-3 - s["x"]) / c, (3 - s["y"]) / sn, (-3 - s["y"]) / sn])
        tw = np.where(tw > 0, tw, np.inf); k = np.argmin(tw, axis=0); t = tw[k, idx]
        pil = np.zeros(n, bool)
        for px, py in R.PILLARS:
            bx, by = s["x"] - px, s["y"] - py; b = bx * c + by * sn; cc = bx * bx + by * by - R.R_P ** 2; disc = b * b - cc
            tp = -b - np.sqrt(np.where(disc >= 0, disc, 0)); pil |= (disc >= 0) & (tp > 0) & (tp < t + 0.2)
        al = np.array([wrap(th[i] - OUT[k[i]]) for i in range(n)]); z = s["ranges"].astype(float)
        ok = np.isfinite(z) & (z > s["rmin"]) & (z < s["rmax"]) & np.isfinite(t) & (t < 3.4) & ~pil & (np.abs(z - t) < 0.2) & (np.abs(np.degrees(al)) > 10)
        x = t * np.tan(al); res = z - t
        num = np.bincount(idx[ok] // 5, weights=res[ok] * x[ok], minlength=72); den = np.bincount(idx[ok] // 5, weights=x[ok] ** 2, minlength=72)
        NUM.append(num); DEN.append(den)
NUM, DEN = np.array(NUM), np.array(DEN)
g = np.arange(4, 68); jc = g * 5 + 2.0
def line(num, den):
    e = num[g] / den[g]; w = den[g]
    A = np.stack([np.ones_like(jc), jc], 1); W = np.diag(w)
    a, b = np.linalg.solve(A.T @ W @ A, A.T @ W @ e); return a, b
a, b = line(NUM.sum(0), DEN.sum(0))
rng = np.random.default_rng(1); est = []
for _ in range(400):
    pick = rng.integers(0, len(NUM), len(NUM)); est.append(line(NUM[pick].sum(0), DEN[pick].sum(0)))
est = np.array(est); lo, hi = np.percentile(est, [2.5, 97.5], axis=0)
print(f"straight line eps(j) = a + b*j over beams 20-339 ({len(NUM)} scans, bootstrap over scans)")
print(f"  intercept a = {a*1e3:+.1f} mrad   95% {lo[0]*1e3:+.1f} .. {hi[0]*1e3:+.1f}     source-derived: {-0.5*FOV/N*1e3:+.1f}")
print(f"  slope     b = {b*1e6:+.1f} urad/beam 95% {lo[1]*1e6:+.1f} .. {hi[1]*1e6:+.1f}     source-derived: {(FOV/(N-1)-FOV/N)*1e6:+.1f}")
print(f"  fitted value at beam 0 / 179 / 359 (extrapolated): {a*1e3:+.1f} / {(a+b*179)*1e3:+.1f} / {(a+b*359)*1e3:+.1f} mrad;  source-derived: {-8.7:+.1f} / -0.0 / {8.7:+.1f}")
# the ends against the interior line and against the source-derived table
def grp(lo_, hi_, a_=None, b_=None):
    n_, d_ = NUM[:, lo_ // 5:hi_ // 5].sum(), DEN[:, lo_ // 5:hi_ // 5].sum(); return n_ / d_
H1 = lambda j: j * (FOV / (N - 1) - FOV / N) - 0.5 * FOV / N
print("\nthe two ends: fitted eps of beams 0-9 and 350-359 against the source-derived table and against the interior line (mrad), with scan-bootstrap 95% intervals")
for (l, h) in ((0, 10), (350, 360)):
    bs = []
    for _ in range(400):
        pick = rng.integers(0, len(NUM), len(NUM)); bs.append(NUM[pick][:, l // 5:h // 5].sum() / DEN[pick][:, l // 5:h // 5].sum())
    jm = (l + h - 1) / 2
    print(f"  beams {l:3d}-{h-1:<3d}: fitted {grp(l, h)*1e3:+6.1f}  (95% {np.percentile(bs,2.5)*1e3:+6.1f} .. {np.percentile(bs,97.5)*1e3:+6.1f})   source-derived {H1(jm)*1e3:+6.1f}   interior line {(a+b*jm)*1e3:+6.1f}")
