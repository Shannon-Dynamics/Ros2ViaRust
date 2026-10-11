"""Stage A PREVIEW, not the Stage B result: replay each run's recorded scans with the beam angles corrected by H1
(angle(j) = angle_min - (j + 0.5) * (angle_min - angle_max) / N, i.e. Webots' pixel-centre layout) in the Python port that
reproduces the mapper bit for bit (01-validate.txt), and grade with the evaluator port (02-validate2.txt).
Parameters, thresholds and routes are the frozen ones. Stage B repeats this with replay_scans.rs."""
import sys, os, math, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
def metrics(L, par, traj):
    g = R.grade(L, par, traj); occ = g["occ"]
    d = np.array([R.true_dist(*R.cell_centre(i, j)) for j, i in zip(*np.nonzero(occ))]) / R.RES
    fo = int((d > 1.0 + 1e-9).sum()); p95 = float(np.sort(d)[max(1, math.ceil(0.95 * len(d))) - 1]); mx = float(d.max())
    return dict(occupied=int(occ.sum()), false_occupied=fo, fo_frac=fo / max(1, int(occ.sum())), p95=p95, max=mx, completeness=g["completeness"], false_free=g["n_false_free"] / g["n_obs"])
print(f"{'run':27s} {'':9s} {'occupied':>8s} {'false occ':>9s} {'fraction':>8s} {'p95 cells':>9s} {'max cells':>9s} {'complete':>8s} {'false free':>10s}")
for run in RUNS:
    d = EV + "2026-10-11-" + run; par = R.read_params(d); scans = R.read_scans(d); traj = R.read_traj(d)
    rows = {}
    for name, fix in (("published", False), ("H1", True)):
        sc = []
        for s in scans:
            s2 = dict(s)
            if fix:
                n = len(s["ranges"]); fov = -(n - 1) * s["da"]                  # published: da = -fov/(N-1), a0 = +fov/2
                s2["da"] = -fov / n; s2["a0"] = s["a0"] - 0.5 * fov / n
            sc.append(s2)
        out = R.replay(sc, par); rows[name] = metrics(out["L"], par, traj)
    for name in ("published", "H1"):
        m = rows[name]
        print(f"{run:27s} {name:9s} {m['occupied']:8d} {m['false_occupied']:9d} {m['fo_frac']*100:7.2f}% {m['p95']:9.2f} {m['max']:9.2f} {m['completeness']:8.4f} {m['false_free']*100:9.2f}%", flush=True)
