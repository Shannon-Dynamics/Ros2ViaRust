import sys, pickle, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
import os; EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
def kv(path):
    d = {}
    for l in open(path):
        if ":" in l and not l.startswith(" "):
            k, v = l.split(":", 1); d[k.strip()] = v.strip()
    return d
for run in ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]:
    full = "2026-10-11-" + run
    scans, par, out = pickle.load(open(f"replay_{run}.pkl", "rb"))
    traj = R.read_traj(EV + full)
    g = R.grade(out["L"], par, traj)
    m = kv(EV + full + "/map-metrics.yaml")
    print(run, "| observable mine/saved:", g["n_obs"], m["samples_observable"], "| completeness mine/saved: %.10f %.10f" % (g["completeness"], float(m["completeness"])),
          "| false_free mine/saved:", g["n_false_free"], m["false_free_samples"], "| traj rows", len(traj), "positions_kept", m["positions_kept"], flush=True)
    pickle.dump(g, open(f"grade_{run}.pkl", "wb"))
