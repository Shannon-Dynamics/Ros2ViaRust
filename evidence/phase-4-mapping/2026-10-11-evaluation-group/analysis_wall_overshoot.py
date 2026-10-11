#!/usr/bin/env python3
"""Where the wall hits that land outside the wall come from, per run. Read-only analysis of the
scans a run integrated (integrated-scans.csv); it changes nothing and grades nothing.

    python3 -I analysis_wall_overshoot.py <phase-4-mapping evidence dir> > analysis-wall-overshoot.txt

A hit is a finite range below 3.5 m. It is a wall hit if it lies at most 0.15 m inside a wall face
(a hit well inside the room is a pillar or a mid-air return). It overshoots if it lies more than
4.5 cm past the face (|coordinate| > 3.045 m). The sensor pose and angles are the ones the mapper
used, taken from the scan record: beam i points at sensor_theta + angle_min + i * angle_increment.
Beams 0 to 14 point straight behind the robot (angle_min is +pi).
"""
import collections, math, os, sys

RUNS = [
    "2026-10-11-development-3-route-long",
    "2026-10-11-evaluation-1-cross",
    "2026-10-11-evaluation-2-wall-lobes",
    "2026-10-11-evaluation-3-diamond-slow",
]
BINS = [(0, 1), (1, 2), (2, 3), (3, 3.5)]


def scans(path):
    for line in open(path):
        if line.startswith("#") or line.startswith("stamp"):
            continue
        p = line.strip().split(",")
        if len(p) < 10:
            continue
        sx, sy, st, a0, da = map(float, p[1:6])
        n = int(p[8])
        yield sx, sy, st, a0, da, [float(v) for v in p[9:9 + n]]


def main(root):
    print("wall hits more than 4.5 cm past the wall face, by range and by beam group")
    for run in RUNS:
        tot, bad = collections.Counter(), collections.Counter()
        gtot, gbad = collections.Counter(), collections.Counter()
        for sx, sy, st, a0, da, rs in scans(os.path.join(root, run, "integrated-scans.csv")):
            for i, r in enumerate(rs):
                if not math.isfinite(r) or r >= 3.5:
                    continue
                a = st + a0 + i * da
                x, y = sx + r * math.cos(a), sy + r * math.sin(a)
                ex = max(abs(x) - 3.0, abs(y) - 3.0)
                if ex < -0.15:
                    continue
                for k, (lo, hi) in enumerate(BINS):
                    if lo <= r < hi:
                        tot[k] += 1
                        bad[k] += ex > 0.045
                g = 0 if i < 15 else 1
                gtot[g] += 1
                gbad[g] += ex > 0.045
        ranges = " ".join(f"{BINS[k][0]}-{BINS[k][1]}m {bad[k]}/{tot[k]}={bad[k] / max(1, tot[k]):.4f}" for k in range(4))
        print(f"{run}\n  by range: {ranges}\n  beams 0-14: {gbad[0]}/{gtot[0]}={gbad[0] / max(1, gtot[0]):.4f}"
              f"   other beams: {gbad[1]}/{gtot[1]}={gbad[1] / max(1, gtot[1]):.4f}")


if __name__ == "__main__":
    main(sys.argv[1])
