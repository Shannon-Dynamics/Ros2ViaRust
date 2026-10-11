import sys, csv, pickle, numpy as np
sys.path.insert(0, ".")
import rc_lib as R
import os; EV = os.environ["RC_EVIDENCE"].rstrip("/") + "/"
RUNS = ["development-3-route-long","evaluation-1-cross","evaluation-2-wall-lobes","evaluation-3-diamond-slow"]
WIN = 0.2
for run in RUNS:
    rows = list(csv.DictReader(open(EV + "2026-10-11-" + run + "/update-latency.csv")))
    integ = [r for r in rows if r["outcome"] == "integrated"]
    w = np.array([float(r["window_yaw_rad"]) / WIN for r in integ]); v = np.array([float(r["window_translation_m"]) / WIN for r in integ])
    st = np.array([float(r["stamp_s"]) for r in integ])
    gaps = np.diff(st)
    all_w = np.array([float(r["window_yaw_rad"]) / WIN for r in rows if r["outcome"] in ("integrated", "skipped_motion") and r["window_yaw_rad"]])
    print(f"{run}: integrated {len(integ)}; |omega| over the 0.2 s window (rad/s): median {np.median(w):.4f} p95 {np.percentile(w,95):.4f} max {w.max():.4f}; "
          f"speed (m/s): median {np.median(v):.3f} p95 {np.percentile(v,95):.3f} max {v.max():.3f}; "
          f"share of integrated scans with omega < 0.01: {np.mean(w<0.01):.2f}, 0.01-0.04: {np.mean((w>=0.01)&(w<0.04)):.2f}, >=0.04: {np.mean(w>=0.04):.2f}")
    print(f"      gaps between consecutive integrated scans: median {np.median(gaps):.2f} s, share > 0.45 s: {np.mean(gaps>0.45):.2f}, max {gaps.max():.1f} s; "
          f"omega over ALL scans seen by the gate (integrated+motion-skipped): p95 {np.percentile(all_w,95):.3f}, max {all_w.max():.3f}")
