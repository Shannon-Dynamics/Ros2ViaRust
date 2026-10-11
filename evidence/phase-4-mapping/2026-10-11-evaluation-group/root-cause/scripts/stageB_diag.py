"""Stage B gate, question 1: why did route 1's false_free rise from 4.86% to 5.59% when the beam angles are corrected (H1)?
Read-only: Python port (validated bit for bit, 01-validate.txt), frozen parameters, unchanged evaluator port.
Beam zones: END = beams 0-14 and 345-359 (where the fitted angular error left a residual after H1), INTERIOR = beams 15-344."""
import sys, os, math, collections, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUN = sys.argv[1] if len(sys.argv) > 1 else "evaluation-1-cross"
END = lambda b: b < 15 or b >= 345

def fix(s):
    n = len(s["ranges"]); fov = -(n - 1) * s["da"]; s2 = dict(s); s2["da"] = -fov / n; s2["a0"] = s["a0"] - 0.5 * fov / n; return s2

def replay_tracked(scans, par, clear_margin=0.10):
    l_occ, l_free = np.float32(par["l_occ"]), np.float32(par["l_free"]); l_min, l_max = np.float32(par["l_min"]), np.float32(par["l_max"])
    L = np.zeros(R.W * R.H, np.float32)
    C = {k: np.zeros(R.W * R.H, np.int32) for k in ("occ", "occ_end", "occ_int_only", "free", "free_end", "free_int_only")}
    for s in scans:
        occ_f, free_f = {}, {}          # idx -> bit 1 = an END beam, bit 2 = an INTERIOR beam
        for bi, raw in enumerate(s["ranges"]):
            z = float(raw); th = s["th"] + s["a0"] + bi * s["da"]; c, sn = math.cos(th), math.sin(th)
            if math.isnan(z) or z <= 0.0 or (math.isfinite(z) and z < s["rmin"]): continue
            if math.isfinite(z) and z < s["rmax"]:
                ex, ey, hit = s["x"] + c * z, s["y"] + sn * z, True
            else:
                reach = s["rmax"] - clear_margin
                if not reach > s["rmin"]: continue
                ex, ey, hit = s["x"] + c * reach, s["y"] + sn * reach, False
            bit = 1 if END(bi) else 2
            cells = list(R.supercover(s["x"], s["y"], ex, ey))
            for k, (i, j) in enumerate(cells):
                if not (0 <= i < R.W and 0 <= j < R.H): break
                idx = j * R.W + i
                if k == len(cells) - 1:
                    if hit: occ_f[idx] = occ_f.get(idx, 0) | bit
                else:
                    free_f[idx] = free_f.get(idx, 0) | bit
        for idx in set(occ_f) | set(free_f):
            if idx in occ_f:
                L[idx] = np.clip(np.float32(L[idx] + l_occ), l_min, l_max); f = occ_f[idx]
                C["occ"][idx] += 1; C["occ_end"][idx] += bool(f & 1); C["occ_int_only"][idx] += (f == 2)
            else:
                L[idx] = np.clip(np.float32(L[idx] + l_free), l_min, l_max); f = free_f[idx]
                C["free"][idx] += 1; C["free_end"][idx] += bool(f & 1); C["free_int_only"][idx] += (f == 2)
    return L.reshape(R.H, R.W), {k: v.reshape(R.H, R.W) for k, v in C.items()}

d = EV + "2026-10-11-" + RUN; par = R.read_params(d); scans = R.read_scans(d); traj = R.read_traj(d)
Lb, Cb = replay_tracked(scans, par); La, Ca = replay_tracked([fix(s) for s in scans], par)
assert (Lb == R.read_grid(d)).all(), "published replay does not reproduce the saved grid"
gb, ga = R.grade(Lb, par, traj), R.grade(La, par, traj)
S = gb["samples"]; ff_b, ff_a = gb["false_free"], ga["false_free"]
print(f"{RUN}: observable {gb['n_obs']}; false-free samples before {ff_b.sum()} ({ff_b.sum()/gb['n_obs']*100:.2f}%), after {ff_a.sum()} ({ff_a.sum()/ga['n_obs']*100:.2f}%); limit 5% = {0.05*gb['n_obs']:.1f} samples, so headroom before: {0.05*gb['n_obs']-ff_b.sum():.1f} samples")
new = np.nonzero(ff_a & ~ff_b)[0]; gone = np.nonzero(ff_b & ~ff_a)[0]; both = int((ff_a & ff_b).sum())
print(f"samples that are false-free in both: {both}; became false-free: {len(new)}; stopped being false-free: {len(gone)}\n")
def where(idx):
    c = collections.Counter(S[n][4][0] + str(S[n][4][1]) for n in idx); return dict(sorted(c.items()))
print("by owner (w0 S, w1 E, w2 N, w3 W; p0..p3 pillars A..D):\n  false-free before:", where(np.nonzero(ff_b)[0]), "\n  false-free after: ", where(np.nonzero(ff_a)[0]), "\n  became:", where(new), "\n  stopped:", where(gone))
def agg(idx, C):
    cells = sorted({c for n in idx for c in R.closed_cells(S[n][0], S[n][1])})
    return len(cells), {k: int(sum(C[k][j, i] for i, j in cells)) for k in C}
print("\ncells touching the samples that BECAME false-free (all their closed cells are Free after the correction):")
nc, b = agg(new, Cb); _, a = agg(new, Ca)
print(f"  {nc} cells; marks summed over those cells, before -> after (scans in which the cell was marked):")
for k, lab in (("occ", "occupied, any beam"), ("occ_end", "occupied, with >=1 END-zone beam"), ("occ_int_only", "occupied, interior beams only"), ("free", "free, any beam"), ("free_end", "free, with >=1 END-zone beam crossing"), ("free_int_only", "free, interior beams only")):
    print(f"    {lab:42s} {b[k]:6d} -> {a[k]:6d}   ({a[k]-b[k]:+d})")
print("  final log-odds of those cells, median before -> after: %.2f -> %.2f; share occupied-or-unknown before %.2f" % (
    np.median([Lb[j, i] for i, j in sorted({c for n in new for c in R.closed_cells(S[n][0], S[n][1])})]), np.median([La[j, i] for i, j in sorted({c for n in new for c in R.closed_cells(S[n][0], S[n][1])})]),
    np.mean([Lb[j, i] > np.float32(par['free_threshold']) for i, j in sorted({c for n in new for c in R.closed_cells(S[n][0], S[n][1])})])))
print("\nwhere on the surface (position along the wall in m / angle round the pillar in degrees) the new false-free samples are:")
pos = collections.defaultdict(list)
for n in new:
    s = S[n]
    if s[4][0] == "w": pos["w" + str(s[4][1])].append(round(s[0] if s[4][1] in (0, 2) else s[1], 2))
    else: cx, cy = R.PILLARS[s[4][1]]; pos["p" + str(s[4][1])].append(round(math.degrees(math.atan2(s[1] - cy, s[0] - cx)) % 360))
for k, v in sorted(pos.items()): print(f"  {k}: n={len(v)}  range {min(v)} .. {max(v)}  sorted sample: {sorted(v)[:: max(1, len(v)//12)]}")
# the same accounting for the samples that are false-free in BOTH, and the totals over every cell of the map
print("\nwhole map, marks summed over every cell, before -> after:")
for k in Cb: print(f"    {k:14s} {int(Cb[k].sum()):8d} -> {int(Ca[k].sum()):8d}")
print("occupied cells (state) before -> after:", int((Lb >= np.float32(par['occupied_threshold'])).sum()), "->", int((La >= np.float32(par['occupied_threshold'])).sum()))
