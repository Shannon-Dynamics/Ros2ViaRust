"""Stage A: does the Webots cylindrical-projection ray layout explain the fitted beam-angle error?
Source facts (see ../stageA-sources/README.md):
  Webots merge_spherical.frag line 158:  angle(column j) = (0.5 - (j + 0.5) / N) * fovX   (N = horizontalResolution = 360)
  webots_ros2_driver Ros2Lidar.cpp:      published angle(j) = fov/2 - j * fov / (N - 1)
  RobotisLds01.proto:                    horizontalResolution 360, fieldOfView 6.28318
H1 (no fitted parameter): true_angle(j) - published_angle(j) = j * (fov/(N-1) - fov/N) - 0.5 * fov/N
"""
import sys, math, os, numpy as np
from math import erf, sqrt
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
FOV, N = 6.28318, 360
OUT = [0.0, math.pi, math.pi / 2, -math.pi / 2]
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def h1(j): return j * (FOV / (N - 1) - FOV / N) - 0.5 * FOV / N          # true - published (rad)
def spacing_only(j): return j * (FOV / (N - 1) - FOV / N)
def half_pixel_only(j): return j * 0.0 - 0.5 * FOV / N
Phi = np.vectorize(lambda z: 0.5 * (1 + erf(z / sqrt(2))))

def collect(run):
    scans = R.read_scans(EV + "2026-10-11-" + run)
    rows = []
    for s in scans:
        n = len(s["ranges"]); idx = np.arange(n); th = s["th"] + s["a0"] + idx * s["da"]
        c, sn = np.cos(th), np.sin(th)
        with np.errstate(divide="ignore", invalid="ignore"):
            tw = np.stack([(3 - s["x"]) / c, (-3 - s["x"]) / c, (3 - s["y"]) / sn, (-3 - s["y"]) / sn])
        tw = np.where(tw > 0, tw, np.inf); k = np.argmin(tw, axis=0); t = tw[k, idx]
        pil = np.zeros(n, bool)
        for px, py in R.PILLARS:
            bx, by = s["x"] - px, s["y"] - py; b = bx * c + by * sn; cc = bx * bx + by * by - R.R_P ** 2; disc = b * b - cc
            tp = -b - np.sqrt(np.where(disc >= 0, disc, 0)); pil |= (disc >= 0) & (tp > 0) & (tp < t + 0.2)
        alpha = np.array([wrap(th[i] - OUT[k[i]]) for i in range(n)])
        z = s["ranges"].astype(float)
        ok = np.isfinite(z) & (z > s["rmin"]) & (z < s["rmax"]) & np.isfinite(t) & (t < 3.4) & ~pil & (np.abs(z - t) < 0.2)
        rows.append((idx[ok], t[ok], z[ok] - t[ok], alpha[ok]))
    return [np.concatenate(c) for c in zip(*rows)]

data = {r: collect(r) for r in RUNS}
def fit(B, T, RES, AL, sel_beams, tab=None):
    m = (np.abs(np.degrees(AL)) > 10) & sel_beams
    x = T * np.tan(AL); res = RES if tab is None else RES - x * tab[B]
    return (res[m] * x[m]).sum() / (x[m] ** 2).sum() if m.sum() >= 80 else float("nan"), int(m.sum())
B = np.concatenate([data[r][0] for r in RUNS]); T = np.concatenate([data[r][1] for r in RUNS])
RES = np.concatenate([data[r][2] for r in RUNS]); AL = np.concatenate([data[r][3] for r in RUNS])
j = np.arange(N); tabs = {"H1 (both terms)": h1(j), "spacing term only": spacing_only(j), "half-pixel term only": half_pixel_only(j)}
print("H1 at the ends: beam 0 %+.1f mrad, beam 179 %+.1f, beam 359 %+.1f (published grid: beam 0 at +180.0 deg, beam 359 at -180.0 deg; Webots: +179.5 deg ... -179.5 deg)\n" % (h1(0)*1e3, h1(179)*1e3, h1(359)*1e3))
print("pooled over the four runs, 10-beam groups: fitted eps from the range residuals, and the three candidate tables (mrad); n = wall beams used")
print("group         fitted     H1  spacing  half-px      n")
rows = []
for g in range(0, 36):
    lo, hi = g * 10, g * 10 + 10
    e, n = fit(B, T, RES, AL, (B >= lo) & (B < hi))
    mid = np.arange(lo, hi)
    rows.append((lo, e, h1(mid).mean() * 1e3 / 1e3, spacing_only(mid).mean(), half_pixel_only(mid).mean(), n))
    print(f"{lo:4d}-{hi-1:<4d}   {e*1e3:+7.1f} {h1(mid).mean()*1e3:+7.1f} {spacing_only(mid).mean()*1e3:+7.1f} {half_pixel_only(mid).mean()*1e3:+7.1f} {n:7d}")
def rms(sel_groups, col):
    d = [(r[1] - r[col]) for r in rows if r[0] in sel_groups and not math.isnan(r[1])]
    return 1e3 * math.sqrt(sum(x * x for x in d) / len(d)), len(d)
interior = set(range(40, 330, 10)); ends = set(list(range(0, 40, 10)) + list(range(330, 360, 10)))
print("\nRMS difference between the fitted eps and each candidate, over 10-beam groups (mrad):")
for name, col in (("H1 (both terms)", 2), ("spacing term only", 3), ("half-pixel term only", 4)):
    ri, ni = rms(interior, col); re_, ne = rms(ends, col)
    print(f"   {name:22s} interior beams 40-329: {ri:5.1f} ({ni} groups)   beams 0-39 and 330-359: {re_:5.1f} ({ne} groups)")
r0 = float(np.sqrt(np.mean([(r[1] * 1e3) ** 2 for r in rows if r[0] in interior and not math.isnan(r[1])])))
print(f"   (no correction at all     interior {r0:5.1f})")
print("\nWHAT IS LEFT after correcting the beam angles by H1 (eps refitted on the residual), 5-beam groups at the two ends (mrad):")
tab = np.zeros(N); tab[:] = h1(j)
for lo in list(range(0, 30, 5)) + list(range(335, 360, 5)):
    e, n = fit(B, T, RES, AL, (B >= lo) & (B < lo + 5), tab=tab)
    print(f"   beams {lo:3d}-{lo+4:<3d} remaining {e*1e3:+6.1f}  (before correction {fit(B, T, RES, AL, (B >= lo) & (B < lo + 5))[0]*1e3:+6.1f})  n={n}")
print("\nrate of wall hits more than 4.5 cm past the face, beams 0-14 and the others; tables have NO term fitted on the run (H1: no fitted term at all)")
print(f"{'run':27s} {'0-14 obs':>9s} {'H1 pred':>8s} {'corrected: left':>16s}   {'others obs':>10s} {'H1 pred':>8s} {'corrected: left':>16s}")
SIG = 0.0155
for r in RUNS:
    b, t, res, al = data[r]; cosa = np.cos(al); obs = (res * cosa) > 0.045
    # prediction if the beams are NOT corrected: the error H1 says they have
    p_h1 = 1 - Phi((0.045 - t * np.sin(al) * h1(b)) / SIG)
    # if the beams ARE corrected by H1: what H1 says is removed; what remains is the fitted-minus-H1 table of this pool (in-sample, shown for scale)
    rem = np.zeros(N)
    for lo in range(0, N, 5):
        e, n = fit(B, T, RES, AL, (B >= lo) & (B < lo + 5), tab=tab); rem[lo:lo + 5] = 0.0 if math.isnan(e) else e
    p_left = 1 - Phi((0.045 - t * np.sin(al) * rem[b]) / SIG)
    sel = b < 15
    print(f"{r:27s} {obs[sel].mean()*100:8.2f}% {p_h1[sel].mean()*100:7.2f}% {p_left[sel].mean()*100:15.2f}%   {obs[~sel].mean()*100:9.2f}% {p_h1[~sel].mean()*100:7.2f}% {p_left[~sel].mean()*100:15.2f}%")
