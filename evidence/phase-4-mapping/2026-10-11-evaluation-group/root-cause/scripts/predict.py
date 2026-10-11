import pickle, math, numpy as np
from math import erf, sqrt
d = pickle.load(open("warp.pkl", "rb")); runs = list(d)
Phi = np.vectorize(lambda z: 0.5 * (1 + erf(z / sqrt(2))))
SIGMA = 0.0155          # lidar range noise in the wall-normal direction (m), about 0.0043 of 3.5 m
BIN = 5
def eps_table(train):
    B = np.concatenate([d[r][0] for r in train]); T = np.concatenate([d[r][1] for r in train])
    RES = np.concatenate([d[r][2] for r in train]); AL = np.concatenate([d[r][3] for r in train])
    m = np.abs(np.degrees(AL)) > 10; x = T * np.tan(AL)
    tab = np.zeros(360 // BIN)
    for g in range(360 // BIN):
        s = m & (B // BIN == g)
        tab[g] = (RES[s] * x[s]).sum() / (x[s] ** 2).sum() if s.sum() >= 80 else 0.0
    return tab
print("beams 0-14 and all other beams: wall hits more than 4.5 cm past the face; the angular-error table is fitted WITHOUT the run it predicts\n")
print(f"{'held-out run':27s} {'beams 0-14 observed':>20s} {'predicted':>10s}   {'other beams observed':>21s} {'predicted':>10s}")
for r in runs:
    tab = eps_table([q for q in runs if q != r])
    B, T, RES, AL, SI = d[r]
    cosa = np.cos(AL); e_obs = RES * cosa                      # hit position error along the wall normal (outward positive), wall beams
    hit_in_wall = (T * cosa < 1e9)
    eps = tab[B // BIN]
    e_pred = T * np.sin(AL) * eps                                # predicted mean shift along the outward normal
    # observed overshoot is measured in the same way as before: hit point more than 4.5 cm past the face
    face_gap = 0.0                                              # truth: hit exactly at the face
    over_obs = (RES * cosa) > 0.045
    over_pred = 1 - Phi((0.045 - e_pred) / (SIGMA))
    out = []
    for name, sel in (("behind", B < 15), ("other", B >= 15)):
        out += [over_obs[sel].mean(), over_pred[sel].mean(), int(sel.sum())]
    print(f"{r.split('2026-10-11-')[-1]:27s} {out[0]*100:19.2f}% {out[1]*100:9.2f}%   {out[3]*100:20.2f}% {out[4]*100:9.2f}%")
