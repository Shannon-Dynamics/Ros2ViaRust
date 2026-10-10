#!/usr/bin/env python3
"""Check that scripts/phase4_scan_skew_analysis.py can tell an instant sweep from a swept one.

    python3 -I scripts/phase4_scan_skew_selftest.py

Builds two synthetic recordings of the same motion, one whose beams are all measured at the
scan stamp and one whose beams are spread over 0.2 s, runs the real analysis on both, and
exits non-zero unless the fitted sweep span is near 0 for the first and near 0.2 for the
second. It exists so that a Webots answer of "instant" is known to be a measurement and not a
property of a method that would say "instant" about anything.
"""
import csv
import importlib.util
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("skew", os.path.join(HERE, "phase4_scan_skew_analysis.py"))
skew = importlib.util.module_from_spec(spec)
spec.loader.exec_module(skew)

MOUNT = (-0.03, 0.0, 0.0)
N = 360
INC = 2 * math.pi / N


def pose(t):
    """Still for 8 s, then straight at 0.18 m/s for 8 s, then turning at 0.8 rad/s."""
    if t < 8:
        return -1.0, -0.5, 0.3
    if t < 16:
        return -1.0 + 0.18 * (t - 8) * math.cos(0.3), -0.5 + 0.18 * (t - 8) * math.sin(0.3), 0.3
    return -1.0 + 1.44 * math.cos(0.3), -0.5 + 1.44 * math.sin(0.3), 0.3 + 0.8 * (t - 16)


def make(run, span, seed):
    rng = np.random.default_rng(seed)
    os.makedirs(run, exist_ok=True)
    with open(os.path.join(run, "gt.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp_s", "x", "y", "qz", "qw"])
        for k in range(int(26 / 0.02)):
            t = 0.02 * k
            x, y, yaw = pose(t)
            w.writerow([f"{t:.6f}", x, y, math.sin(yaw / 2), math.cos(yaw / 2)])
    with open(os.path.join(run, "phases.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["phase", "linear", "angular", "start_sim_s", "duration_s"])
        w.writerow(["still", 0, 0, 0.0, 8.0])
        w.writerow(["straight", 0.18, 0, 8.0, 8.0])
        w.writerow(["turn", 0, 0.8, 16.0, 8.0])
    with open(os.path.join(run, "scans.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp_s", "frame_id", "angle_min", "angle_max", "angle_increment",
                    "time_increment", "scan_time", "range_min", "range_max", "n"]
                   + [f"r{i}" for i in range(N)])
        for k in range(int(24 / 0.2)):
            t = 0.5 + 0.2 * k
            idx = np.arange(N)
            tb = t + 0.04 + span * idx / (N - 1)   # a constant 0.04 s stamp offset as well
            xs = np.array([pose(u)[0] for u in tb]); ys = np.array([pose(u)[1] for u in tb])
            yaws = np.array([pose(u)[2] for u in tb])
            ox = xs + MOUNT[0] * np.cos(yaws)
            oy = ys + MOUNT[0] * np.sin(yaws)
            z = skew.raycast(ox, oy, yaws + idx * INC) + rng.normal(0, 0.0043 * 3.5 / 3, N)
            z = np.where(z > 3.5, np.inf, z)
            w.writerow([f"{t:.6f}", "LDS-01", 0.0, 2 * math.pi - INC, INC, 0.0, 0.2, 0.12, 3.5, N]
                       + [repr(float(v)) for v in z])


def span_of(run, phase):
    sys.argv = ["x", run]
    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        skew.main()
    import yaml
    rep = yaml.safe_load(open(os.path.join(run, "skew-report.yaml")))
    return rep["phases"][phase]["swept_model_span_s"]["median"], rep


def main():
    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        for span in (0.0, 0.2):
            run = os.path.join(tmp, f"span{span}")
            make(run, span, 7)
            for phase in ("straight", "turn"):
                s, rep = span_of(run, phase)
                near = abs(s - span) < 0.03
                print(f"true span {span:.2f} s, phase {phase:8s}: fitted median span {s:+.3f} s "
                      f"{'ok' if near else 'WRONG'}")
                ok &= near
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
