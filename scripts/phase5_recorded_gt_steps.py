#!/usr/bin/env python3
"""How long are the ground-truth steps between the consecutive recorded scans of a Phase 4 run?

    python3 -I scripts/phase5_recorded_gt_steps.py integrated-scans.csv [...]

Why it exists: the Webots half of claim S1a grades a registration against the ground-truth relative pose
of two consecutive recorded scans, and the translation error comes out near 3 cm on every route where a
synthetic run gives 5 mm. The Webots lidar measures on a 0.2 s lattice and its stamps do not follow it
(docs/phase-4-mapping/implementation.md section 5.2, item 2), and the recorded pose of a scan is the
ground truth at its stamp less a constant offset. This prints what that does to the ground truth itself:
the length of each consecutive ground-truth step, and the stamp spacing it came from. Reads only the
recording; no scan is registered.
"""
import collections
import math
import sys


def main(paths):
    for path in paths:
        rows = []
        for line in open(path):
            if line.startswith("#"):
                continue
            f = line.split(",", 8)
            rows.append((float(f[0]), float(f[1]), float(f[2])))
        pairs = []
        for a, b in zip(rows, rows[1:]):
            dt = b[0] - a[0]
            if 0 < dt <= 0.5:
                pairs.append((round(dt, 2), math.hypot(b[1] - a[1], b[2] - a[2])))
        by = collections.defaultdict(list)
        for dt, s in pairs:
            by[dt].append(s)
        print(f"{path}")
        print(f"  consecutive pairs at most 0.5 s apart: {len(pairs)}")
        for dt in sorted(by):
            xs = sorted(by[dt])
            share = len(xs) / len(pairs)
            print(f"  stamp spacing {dt:.2f} s: {len(xs)} pairs ({share * 100:.1f} per cent), ground-truth step "
                  f"min {xs[0] * 1000:.1f} / median {xs[len(xs) // 2] * 1000:.1f} / max {xs[-1] * 1000:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]) if len(sys.argv) > 1 else print(__doc__))
