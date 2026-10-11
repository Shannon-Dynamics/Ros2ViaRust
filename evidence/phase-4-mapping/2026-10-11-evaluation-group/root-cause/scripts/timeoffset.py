import pickle, math, numpy as np
d = pickle.load(open("warp.pkl", "rb")); runs = list(d)
BIN = 5; V = 0.18
OUT = [0.0, math.pi, math.pi / 2, -math.pi / 2]
def eps_table(train):
    B = np.concatenate([d[r][0] for r in train]); T = np.concatenate([d[r][1] for r in train])
    RES = np.concatenate([d[r][2] for r in train]); AL = np.concatenate([d[r][3] for r in train])
    m = np.abs(np.degrees(AL)) > 10; x = T * np.tan(AL); tab = np.zeros(72)
    for g in range(72):
        s = m & (B // BIN == g)
        tab[g] = (RES[s] * x[s]).sum() / (x[s] ** 2).sum() if s.sum() >= 80 else 0.0
    return tab
rng = np.random.default_rng(0)
print("scan-time-offset check: pose displacement along the heading, delta (mm), fitted from wall range residuals after removing the beam-angle term;")
print("equivalent time error = delta / 0.18 m/s. Only the magnitude is interpreted; the sign convention of delta was not verified against the pose lookup\n")
for r in runs:
    scans, par, out = pickle.load(open(f"replay_{r}.pkl", "rb"))
    B, T, RES, AL, SI = d[r]
    tab = eps_table([q for q in runs if q != r])
    res = RES - T * np.tan(AL) * tab[B // BIN]
    h = np.array([scans[i]["th"] for i in SI]); phi = AL * 0
    # wall outward normal angle: recover from alpha = wrap(theta_beam - phi); theta_beam = h + bearing; use geometry instead: g = cos(h - phi)/cos(alpha)
    s0 = scans[0]; bearing = s0["a0"] + B * s0["da"]
    th_beam = h + bearing; phi = th_beam - AL
    g = np.cos(h - phi) / np.cos(AL)
    m = (np.abs(np.degrees(AL)) < 60) & (np.abs(res) < 0.15)
    delta = -(res[m] * g[m]).sum() / (g[m] ** 2).sum()
    # bootstrap over scans
    sid = SI[m]; u = np.unique(sid); est = []
    for _ in range(200):
        pick = rng.choice(u, len(u)); w = np.bincount(sid, minlength=len(scans))[pick]
        # weights by scan: use sums per scan
    per_num = np.bincount(sid, weights=-(res[m] * g[m]), minlength=len(scans)); per_den = np.bincount(sid, weights=g[m] ** 2, minlength=len(scans))
    for _ in range(300):
        pick = rng.choice(u, len(u)); est.append(per_num[pick].sum() / per_den[pick].sum())
    lo, hi = np.percentile(est, [2.5, 97.5])
    print(f"{r.split('2026-10-11-')[-1]:27s} delta {delta*1000:+6.1f} mm  (95% scan bootstrap {lo*1000:+6.1f} .. {hi*1000:+6.1f})  = {delta/V*1000:+6.0f} ms  ({lo/V*1000:+.0f} .. {hi/V*1000:+.0f} ms); wall beams used {int(m.sum())}")
