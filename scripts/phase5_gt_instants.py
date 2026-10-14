#!/usr/bin/env python3
"""How good is the recorded ground-truth pose of a scan taken while the robot turns?

    python3 -I scripts/phase5_gt_instants.py --scans integrated-scans.csv --gt ground-truth.csv \\
        --registrations node/registrations.csv [--out report.yaml] [--write-corrected corrected.csv]

Inputs: a recording of every scan (scripts/run_phase5_record.sh; the pose columns of each scan are the
ground truth at `stamp - 0.06` s, the offset Phase 4's mapper uses), the DENSE ground-truth track of the same
run (`ground-truth.csv`, 50 Hz), and the registrations the scan matcher made of the same scans
(`registrations.csv` of a replay; they come from the scans alone and read no ground truth).

The question. The Webots lidar measures on a regular 0.2 s lattice in simulation time and its stamps do not
follow it: they lag the measurement by -0.02 to 0.18 s (docs/phase-4-mapping/implementation.md 5.2 item 2,
which also recorded the lattice phase, 0.0603 s, from a probe). Phase 4 used the mean lag, -0.06 s, because a
mapper that skips scans while moving is insensitive to it. A SLAM claim graded on turning scans is not: at
0.8 rad/s the heading differs by 0.8 x (lag error) rad, up to about 0.1 rad, against S5's 0.03 rad limit for a
wrong loop constraint.

What this does, in order, with nothing tuned to the answer it hopes for:
  1. For every lattice phase phi in [0, 0.2) s (1 ms steps) and every lower edge L0 of the lag window in
     [-0.06, 0.02] s (5 ms steps), put each scan at the instant t = phi + 0.2 m, m the integer that makes
     `stamp - t` fall in [L0, L0 + 0.2): the phase says where the lattice is, and the window edge says which
     lattice instant a stamp belongs to (a stamp near the edge of a fixed window is assigned to the wrong
     instant by a small error in either, which the first version of this script showed). Read the dense
     ground truth at those instants (linear in position, shortest arc in heading), and compare the relative
     heading between consecutive scans with the heading change the scan matcher found. The comparison is
     over the TURNING pairs (ground-truth heading step above 0.05 rad), where the instants matter.
  2. Report the (phase, edge) with the smallest rms disagreement, and for comparison the disagreement at the
     probe's phase 0.0603 s with its window [-0.02, 0.18) and for the recorded poses (stamp - 0.06 s).
     A phase that disagrees with the probe's is a finding, not an error: the probe was one run.
  3. With --write-corrected, write a copy of the recording whose pose columns are the ground truth at the
     lattice instants of the BEST phase (sensor pose = base pose composed with the lidar mount, read from the
     recording). The scans are copied unchanged. This file is a different recording of the same run: the
     original is never touched, and what is graded on which is said wherever a number is given.

Reads files only; imports no ROS.
"""
import argparse
import bisect
import csv
import math
import statistics
import sys

LAG_LO, LAG_HI = -0.02, 0.18
PERIOD = 0.2
PROBE_PHASE = 0.0603
TURNING_STEP_RAD = 0.05


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def read_scans(path):
    out = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        f = line.rstrip("\n").split(",")
        out.append((float(f[0]), float(f[1]), float(f[2]), float(f[3]), line))
    return out


def read_gt(path):
    t, x, y, yaw = [], [], [], []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        f = line.split(",")
        t.append(float(f[0]))
        x.append(float(f[1]))
        y.append(float(f[2]))
        yaw.append(float(f[3]))
    for i in range(1, len(yaw)):  # unwrap
        yaw[i] = yaw[i - 1] + wrap(yaw[i] - yaw[i - 1])
    return t, x, y, yaw


class Track:
    def __init__(self, t, x, y, yaw):
        self.t, self.x, self.y, self.yaw = t, x, y, yaw

    def at(self, when):
        i = bisect.bisect_right(self.t, when)
        if i <= 0 or i >= len(self.t):
            return None
        a = (when - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return (self.x[i - 1] + a * (self.x[i] - self.x[i - 1]),
                self.y[i - 1] + a * (self.y[i] - self.y[i - 1]),
                self.yaw[i - 1] + a * (self.yaw[i] - self.yaw[i - 1]))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), s * a[0] - c * a[1], wrap(-a[2]))


def between(a, b):
    return compose(inverse(a), b)


def lattice_instant(stamp, phi, lo=LAG_LO):
    # t = phi + PERIOD*m with stamp - t in [lo, lo + PERIOD): m = floor((stamp - lo - phi)/PERIOD)
    m = math.floor((stamp - lo - phi) / PERIOD)
    return phi + PERIOD * m, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scans", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--registrations", required=True)
    ap.add_argument("--out")
    ap.add_argument("--write-corrected")
    a = ap.parse_args()

    scans = read_scans(a.scans)
    track = Track(*read_gt(a.gt))
    regs = {}
    for r in csv.DictReader(open(a.registrations)):
        if r["status"] in ("Tracked", "TrackedWithFallback", "Tracked "):
            regs[int(r["stamp_ns"])] = (float(r["dx"]), float(r["dy"]), float(r["dtheta"]))
    # the lidar's mount in the base frame, from the first scan (the robot is at rest at the base pose)
    first = track.at(scans[0][0] - 0.06)
    base0 = first if first else (0.0, 0.0, 0.0)
    mount = between(base0, (scans[0][1], scans[0][2], scans[0][3]))
    print(f"# lidar mount in base_link, read from the first recorded scan: x {mount[0]:.4f} y {mount[1]:.4f} yaw {mount[2]:.5f}")

    pairs = []  # (stamp_prev, stamp, (dx,dy,dth)_icp)
    for p, q in zip(scans, scans[1:]):
        key = round(q[0] * 1e9)
        if 0 < q[0] - p[0] <= 0.5 and key in regs:
            pairs.append((p[0], q[0], regs[key]))
    print(f"# {len(scans)} scans, {len(pairs)} consecutive pairs with a tracked registration")

    def sensor_at(when):
        b = track.at(when)
        return None if b is None else compose(b, mount)

    def residuals(instant_of):
        rot, trans, n_turn = [], [], 0
        for s0, s1, icp in pairs:
            t0, t1 = instant_of(s0), instant_of(s1)
            p0, p1 = sensor_at(t0), sensor_at(t1)
            if p0 is None or p1 is None or t1 <= t0 + 1e-9:
                continue
            gt = between(p0, p1)
            if abs(gt[2]) <= TURNING_STEP_RAD:
                continue
            n_turn += 1
            rot.append(wrap(icp[2] - gt[2]))
            trans.append(math.hypot(icp[0] - gt[0], icp[1] - gt[1]))
        return rot, trans, n_turn

    def rms(x):
        return math.sqrt(sum(v * v for v in x) / len(x)) if x else float("nan")

    curve = []  # (phi, rms rotation, rms translation, turning pairs, lo)
    for k in range(200):
        phi = k * 0.001
        for j in range(17):
            lo = -0.06 + 0.005 * j
            rot, trans, n = residuals(lambda s, phi=phi, lo=lo: lattice_instant(s, phi, lo)[0])
            curve.append((phi, rms(rot), rms(trans), n, lo))
    best = min((c for c in curve if c[3] > 10), key=lambda c: c[1])
    best_phi, best_lo = best[0], best[4]
    at_probe = residuals(lambda s: lattice_instant(s, PROBE_PHASE, LAG_LO)[0])
    recorded = residuals(lambda s: s - 0.06)
    def line(label, rot, trans, n):
        if not rot:
            return f"{label}: no turning pairs"
        ar = sorted(abs(v) for v in rot)
        return (f"{label}: {n} turning pairs, rotation disagreement rms {rms(rot):.4f} rad, median |.| {statistics.median(ar):.4f}, "
                f"p95 {ar[int(0.95 * (len(ar) - 1))]:.4f}, max {ar[-1]:.4f}; translation rms {rms(trans) * 1000:.1f} mm")
    print(line(f"best lattice phase {best_phi:.3f} s, window [{best_lo:+.3f}, {best_lo + PERIOD:+.3f})",
               *residuals(lambda s: lattice_instant(s, best_phi, best_lo)[0])))
    print(line("probe lattice phase 0.0603 s", *at_probe))
    print(line("recorded poses (stamp - 0.06 s)", *recorded))
    worst = max(curve, key=lambda c: c[1] if c[3] > 10 else -1)
    near = sorted((c for c in curve if c[3] > 10), key=lambda c: c[1])[:6]
    print(f"# disagreement over the search: best {best[1]:.4f} rad, worst {worst[1]:.4f} rad; the six best (phase s, window edge s, rad): "
          + ", ".join(f"({c[0]:.3f}, {c[4]:+.3f}, {c[1]:.4f})" for c in near))
    lags = sorted(s - lattice_instant(s, best_phi, best_lo)[0] for s, _, _, _, _ in scans)
    print(f"# stamp lag behind the best lattice instant: min {lags[0]:.3f} / median {lags[len(lags) // 2]:.3f} / max {lags[-1]:.3f} s "
          f"(the probe of 2026-10-10: -0.02 to 0.18, mean 0.061)")

    if a.out:
        with open(a.out, "w") as f:
            f.write(f"scans: {len(scans)}\npairs_with_registration: {len(pairs)}\n")
            f.write(f"best_phase_s: {best_phi:.4f}\nbest_window_lower_edge_s: {best_lo:.4f}\nbest_rotation_rms_rad: {best[1]:.5f}\nbest_translation_rms_m: {best[2]:.5f}\nturning_pairs: {best[3]}\n")
            f.write(f"probe_phase_s: {PROBE_PHASE}\nprobe_phase_rotation_rms_rad: {rms(at_probe[0]):.5f}\n")
            f.write(f"recorded_rotation_rms_rad: {rms(recorded[0]):.5f}\nrecorded_rotation_p95_rad: {sorted(abs(v) for v in recorded[0])[int(0.95 * (len(recorded[0]) - 1))]:.5f}\n")
            f.write("best_rotation_rms_rad_by_phase_ms:  # minimum over the window edge, for each phase\n")
            for k in range(200):
                row = [c for c in curve if abs(c[0] - k * 0.001) < 1e-9 and c[3] > 10]
                if row:
                    f.write(f"  {k}: {min(c[1] for c in row):.5f}\n")
    if a.write_corrected:
        n_ok = 0
        with open(a.write_corrected, "w") as f:
            f.write(f"# the scans of {a.scans}, copied unchanged; the pose columns are the ground truth of {a.gt} at the lattice instants of phase {best_phi:.4f} s, lag window [{best_lo:+.3f}, {best_lo + PERIOD:+.3f}) (scripts/phase5_gt_instants.py)\n")
            f.write("# stamp_s,sensor_x,sensor_y,sensor_theta,angle_min,angle_increment,range_min,range_max,n,ranges...\n")
            for stamp, x, y, th, line_ in scans:
                p = sensor_at(lattice_instant(stamp, best_phi, best_lo)[0])
                tail = line_.split(",", 4)[4]
                if p is None:
                    p = (x, y, th)
                else:
                    n_ok += 1
                f.write(f"{stamp:.9f},{p[0]:.9f},{p[1]:.9f},{p[2]:.9f},{tail}")
        print(f"# wrote {a.write_corrected}: {n_ok} of {len(scans)} poses replaced")
    return 0


if __name__ == "__main__":
    sys.exit(main())
