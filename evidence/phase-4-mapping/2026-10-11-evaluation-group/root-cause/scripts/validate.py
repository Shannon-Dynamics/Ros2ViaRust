import sys, time, pickle, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
import os; EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["2026-10-11-development-3-route-long","2026-10-11-evaluation-1-cross","2026-10-11-evaluation-2-wall-lobes","2026-10-11-evaluation-3-diamond-slow"]
for run in RUNS:
    t = time.time()
    d = EV + run
    par = R.read_params(d); scans = R.read_scans(d)
    out = R.replay(scans, par)
    saved = R.read_grid(d)
    diff = np.abs(out["L"] - saved)
    occ_thr = np.float32(par["occupied_threshold"])
    print(run.split("2026-10-11-")[1], "scans", len(scans), "max |L - saved| =", float(diff.max()),
          "cells differing:", int((diff > 0).sum()), "occupied mine/saved:", int((out["L"] >= occ_thr).sum()), int((saved >= occ_thr).sum()),
          "%.0fs" % (time.time() - t), flush=True)
    pickle.dump((scans, par, out), open(f"replay_{run.split('2026-10-11-')[1]}.pkl", "wb"))
