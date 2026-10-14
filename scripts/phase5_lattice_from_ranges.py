#!/usr/bin/env python3
"""When was each scan of a Webots recording measured? Answered from the scan's own ranges, not from SLAM output.

    python3 -I scripts/phase5_lattice_from_ranges.py --scans integrated-scans.csv --gt ground-truth.csv \\
        --report report.yaml [--write-labelled labelled.csv]

Why this exists (docs/phase-5-slam/benchmark.md 7.2). scripts/phase5_gt_instants.py estimated the lidar's lattice
phase by agreement with the scan matcher's own rotations, and its 0.198 s disagreed with the 0.0603 s of the Phase 4
probe. Two things were wrong with that, and this script removes both. (1) The fit read SLAM output, so a pose label
graded against the SLAM output was fitted to it. This script reads only the scans' ranges, the dense ground-truth
track and the analytic room (the same world, wall faces at +/- 3.0 m and four pillars of radius 0.15 m, that
scripts/phase4_scan_skew_analysis.py uses; its `fit_scan` is imported unchanged). (2) It compared one run's phase
with another run's: the phase of the 0.2 s measurement lattice against the ground-truth clock is a property of a RUN,
not of the driver (benchmark.md 7.2), so it is estimated for the run that is graded, from that run's data.

The protocol, fixed before the run it was first used on (docs/phase-5-slam/benchmark.md 7.2):

  1. Turning scans: scans whose sensor heading rate in the dense track, at stamp - 0.06 s, is at least 0.6 rad/s
     (the route driver turns at 0.8). Turns are runs of such scans separated by less than 1 s.
  2. CALIBRATION = the first three turns of the run. For each such scan, the probe's per-scan fit gives the offset d
     between the stamp and the measurement instant (instant = stamp + d; lag = -d); a fit with rms above 0.03 m is
     dropped. The lattice phase phi is the circular mean of (stamp + d) modulo 0.2 s, and the window of lags is
     [lag_min, lag_max] of the calibration scans.
  3. VALIDATION = every later turn, not used by step 2: the same fits must give a phase within 0.005 s of phi with
     resultant length at least 0.95, and at least 95 per cent of the lags inside the window up to 0.01 s (a single
     per-scan fit can land far off: the development recording of the first live run has one at -0.115 s, which a
     rule of "every lag" would have turned into a failure of the whole run; this was relaxed on that recording and
     BEFORE the run it is first used to grade). If any fails the labels are NOT written (exit code 3) and the run is
     graded on the recorded labels only.
  4. LABELS: every scan whose instant the dense track covers gets the track at t = phi + 0.2 m, m the integer for
     which stamp - t lies in [lo, lo + 0.2), lo centring the calibration lags in that window; the sensor pose is the
     base pose composed with the lidar mount (read from the first recorded scan, the robot at rest at the origin).
     A scan the track does not cover keeps its recorded pose, and they are counted.
  5. LABEL QUALITY, which does not use SLAM output either: for every scan, the rms difference between its ranges and
     the exact ranges of the analytic room seen from the labelled pose (valid beams only), for the recorded labels
     (stamp - 0.06 s) and for the new ones, reported for turning and straight scans, calibration and validation
     separately. A label is better if the room explains the scan better.

Reads files only; imports no ROS. The run is not tuned: no threshold above is changed after the data are seen, and a
failed validation is a result.
"""
import argparse
import bisect
import importlib.util
import math
import os
import statistics
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("skew", os.path.join(HERE, "phase4_scan_skew_analysis.py"))
skew = importlib.util.module_from_spec(spec)
spec.loader.exec_module(skew)

PERIOD = 0.2
TURN_RATE = 0.6          # rad/s
CALIBRATION_TURNS = 3
MAX_FIT_RMS = 0.03       # m
PHASE_TOLERANCE = 0.005  # s
MIN_RESULTANT = 0.95
MIN_IN_WINDOW = 0.95     # fraction of the validation lags
WINDOW_SLACK = 0.01      # s
RECORDED_OFFSET = -0.06  # s: the pose columns of a recording are the track at stamp + this


def read_scans(path):
    out = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        f = line.rstrip("\n").split(",")
        n = int(f[8])
        out.append({"t": float(f[0]), "pose": (float(f[1]), float(f[2]), float(f[3])), "angle_min": float(f[4]),
                    "inc": float(f[5]), "range_min": float(f[6]), "range_max": float(f[7]),
                    "ranges": np.array([float(v) for v in f[9:9 + n]]), "line": line})
    return out


def read_gt(path):
    rows = [[float(v) for v in l.split(",")] for l in open(path) if not l.startswith("#") and l.strip()]
    a = np.array(rows)
    return skew.Track(a[:, 0], a[:, 1], a[:, 2], a[:, 3])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def circular(times, period=PERIOD):
    a = 2 * np.pi * np.mod(times, period) / period
    r = float(abs(np.mean(np.exp(1j * a))))
    mean = float((math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a))) % (2 * math.pi)) * period / (2 * math.pi))
    dev = (np.mod(times, period) - mean + period / 2) % period - period / 2
    return r, mean, float(dev.std())


def circ_diff(a, b, period=PERIOD):
    return (a - b + period / 2) % period - period / 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scans", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--report", required=True)
    ap.add_argument("--write-labelled")
    a = ap.parse_args()

    scans = read_scans(a.scans)
    track = read_gt(a.gt)
    t_lo, t_hi = float(track.t[0]), float(track.t[-1])
    mount = (scans[0]["pose"][0], scans[0]["pose"][1], scans[0]["pose"][2])  # robot at rest at the origin
    print(f"# {len(scans)} scans; dense track {t_lo:.2f} to {t_hi:.2f} s; lidar mount read from the first scan: {mount}")

    def heading_rate(when):
        h = track.at(np.array([when - 0.1, when + 0.1]))[2]
        return float((h[1] - h[0]) / 0.2)

    turning = [i for i, s in enumerate(scans)
               if t_lo + 1.0 < s["t"] < t_hi - 1.0 and abs(heading_rate(s["t"] + RECORDED_OFFSET)) >= TURN_RATE]
    turns, cur = [], []
    for i in turning:
        if cur and scans[i]["t"] - scans[cur[-1]]["t"] >= 1.0:
            turns.append(cur)
            cur = []
        cur.append(i)
    if cur:
        turns.append(cur)
    print(f"# {len(turning)} turning scans in {len(turns)} turns")
    if len(turns) <= CALIBRATION_TURNS:
        print("FAIL: not enough turns to hold any out", file=sys.stderr)
        return 2
    calib_idx = [i for t in turns[:CALIBRATION_TURNS] for i in t]
    valid_idx = [i for t in turns[CALIBRATION_TURNS:] for i in t]

    def fits(idx):
        out = []
        for i in idx:
            f = skew.fit_scan(track, scans[i], mount, with_s=False)
            if f and f["rms"] <= MAX_FIT_RMS:
                out.append((i, scans[i]["t"], f["d"], f["rms"]))
        return out

    cf, vf = fits(calib_idx), fits(valid_idx)
    print(f"# per-scan fits kept: calibration {len(cf)} of {len(calib_idx)}, validation {len(vf)} of {len(valid_idx)}")
    ct = np.array([s + d for _, s, d, _ in cf])
    R, phi, sd = circular(ct)
    lags = np.array([-d for _, _, d, _ in cf])
    lag_min, lag_max = float(lags.min()), float(lags.max())
    if lag_max - lag_min >= PERIOD:
        print("FAIL: the calibration lags span a whole period", file=sys.stderr)
        return 2
    lo = lag_min - (PERIOD - (lag_max - lag_min)) / 2
    vt = np.array([s + d for _, s, d, _ in vf])
    vR, vphi, vsd = circular(vt)
    v_lags = np.array([-d for _, _, d, _ in vf])
    dphi = circ_diff(vphi, phi)
    out_of_window = int(np.sum((v_lags < lag_min - WINDOW_SLACK) | (v_lags > lag_max + WINDOW_SLACK)))
    ok_phase = abs(dphi) <= PHASE_TOLERANCE
    ok_R = vR >= MIN_RESULTANT and R >= MIN_RESULTANT
    ok_window = len(vf) > 0 and 1.0 - out_of_window / len(vf) >= MIN_IN_WINDOW
    valid = ok_phase and ok_R and ok_window
    print(f"calibration: phase {phi:.4f} s, resultant {R:.4f}, sd {sd * 1000:.1f} ms, lags [{lag_min:+.3f}, {lag_max:+.3f}] s, "
          f"window lower edge {lo:+.4f} s")
    print(f"validation: phase {vphi:.4f} s (differs {dphi * 1000:+.1f} ms), resultant {vR:.4f}, sd {vsd * 1000:.1f} ms, "
          f"{out_of_window} of {len(vf)} lags outside the calibration window")
    print(f"validation {'PASSED' if valid else 'FAILED'} (phase within {PHASE_TOLERANCE * 1000:.0f} ms: {ok_phase}, "
          f"resultant >= {MIN_RESULTANT}: {ok_R}, lags inside the window >= {MIN_IN_WINDOW:.0%}: {ok_window})")

    def instant(stamp):
        m = math.floor((stamp - lo - phi) / PERIOD)
        return phi + PERIOD * m

    def sensor_pose(when):
        if not (t_lo <= when <= t_hi):
            return None
        b = track.at(np.array([when]))
        return compose((float(b[0][0]), float(b[1][0]), float(b[2][0])), mount)

    def residual(s, pose):
        sc = dict(s)
        valid_b = skew.valid_beams(sc, track, (0, 0, 0))
        ox, oy, th = pose
        ang = s["angle_min"] + np.arange(len(s["ranges"])) * s["inc"]
        pred = skew.raycast(np.full(ang.shape, ox), np.full(ang.shape, oy), th + ang)
        r = (s["ranges"] - pred)[valid_b]
        if r.size < 40:
            return None
        return float(np.sqrt(np.mean(np.minimum(r * r, skew.TRUNC ** 2))))

    calib_set, valid_set = set(calib_idx), set(valid_idx)
    quality = {}
    uncovered = 0
    for i, s in enumerate(scans):
        rec = sensor_pose(s["t"] + RECORDED_OFFSET)
        new = sensor_pose(instant(s["t"]))
        if rec is None or new is None:
            uncovered += 1
            continue
        group = "calibration_turning" if i in calib_set else "validation_turning" if i in valid_set else \
            "turning_not_fitted" if i in set(turning) else "straight"
        ra, rb = residual(s, rec), residual(s, new)
        if ra is None or rb is None:
            continue
        quality.setdefault(group, []).append((ra, rb))
    lines = []
    for g, v in sorted(quality.items()):
        ra = np.array([x[0] for x in v])
        rb = np.array([x[1] for x in v])
        lines.append(f"{g}: {len(v)} scans; range residual vs the room, median / p95 (mm): recorded labels "
                     f"{np.median(ra) * 1000:.1f} / {np.percentile(ra, 95) * 1000:.1f}, instant labels "
                     f"{np.median(rb) * 1000:.1f} / {np.percentile(rb, 95) * 1000:.1f}")
    for l in lines:
        print(l)
    print(f"scans the dense track does not cover (keep their recorded pose): {uncovered}")

    with open(a.report, "w") as f:
        f.write("# Written by scripts/phase5_lattice_from_ranges.py; reads scans' ranges, the dense track and the analytic room only.\n")
        f.write(f"scans: {len(scans)}\nturning_scans: {len(turning)}\nturns: {len(turns)}\ncalibration_turns: {CALIBRATION_TURNS}\n")
        f.write(f"calibration_fits: {len(cf)}\ncalibration_phase_s: {phi:.5f}\ncalibration_resultant: {R:.5f}\n"
                f"calibration_phase_sd_s: {sd:.5f}\nlag_min_s: {lag_min:.5f}\nlag_max_s: {lag_max:.5f}\nwindow_lower_edge_s: {lo:.5f}\n")
        f.write(f"validation_fits: {len(vf)}\nvalidation_phase_s: {vphi:.5f}\nvalidation_phase_difference_s: {dphi:.5f}\n"
                f"validation_resultant: {vR:.5f}\nvalidation_lags_outside_window: {out_of_window}\n")
        f.write(f"validation_passed: {str(valid).lower()}\nuncovered_scans: {uncovered}\n")
        f.write("range_residual_vs_room:\n")
        for l in lines:
            f.write(f"  - \"{l}\"\n")
    if not valid:
        return 3
    if a.write_labelled:
        n_new = 0
        with open(a.write_labelled, "w") as f:
            f.write(f"# the scans of {a.scans}, unchanged; the pose columns are the dense track {a.gt} at the lattice instants of phase "
                    f"{phi:.5f} s, lag window [{lo:+.4f}, {lo + PERIOD:+.4f}), estimated from the ranges of the first {CALIBRATION_TURNS} turns "
                    f"and validated on the others (scripts/phase5_lattice_from_ranges.py)\n")
            f.write("# stamp_s,sensor_x,sensor_y,sensor_theta,angle_min,angle_increment,range_min,range_max,n,ranges...\n")
            for s in scans:
                p = sensor_pose(instant(s["t"]))
                tail = s["line"].split(",", 4)[4]
                if p is None:
                    p = s["pose"]
                else:
                    n_new += 1
                f.write(f"{s['t']:.9f},{p[0]:.9f},{p[1]:.9f},{p[2]:.9f},{tail}")
        print(f"# wrote {a.write_labelled}: {n_new} of {len(scans)} poses replaced")
    return 0


if __name__ == "__main__":
    sys.exit(main())
