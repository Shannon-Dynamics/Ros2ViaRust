#!/usr/bin/env python3
"""How close is the landmark map to being rotationally symmetric?

    python3 -I scripts/phase3_map_symmetry.py <landmarks.yaml>

Writes YAML on stdout. For each rotation of 90, 180 and 270 degrees it finds the best centre
(a grid search over the square [-1, 1] m, 1 cm steps, plus the origin and the landmark
centroid reported separately) and the best one-to-one matching of the rotated landmarks onto
the originals, and reports the worst and the RMS distance a rotated landmark lands from the
landmark it was matched to. A rotation under which the map matches to within the filter's
measurement tolerance is a pose the filter cannot tell from the true one by landmark
positions alone; this is the evidence for why a global search can converge, confidently, onto
a pose rotated by that angle (docs/phase-3-localization/benchmark.md).

The tolerance to read the numbers against is the filter's association gate: with the default
0.05 m range sigma, an observation is "explained" within about 3 sigma = 0.15 m.
"""
import itertools
import math
import sys

import yaml


def rotate(p, c, angle):
    dx, dy = p[0] - c[0], p[1] - c[1]
    ca, sa = math.cos(angle), math.sin(angle)
    return (c[0] + ca * dx - sa * dy, c[1] + sa * dx + ca * dy)


def best_match(points, centre, angle):
    """Smallest worst-case distance over every one-to-one matching of rotated onto original."""
    rotated = [rotate(p, centre, angle) for p in points]
    best = None
    for perm in itertools.permutations(range(len(points))):
        d = [math.dist(rotated[i], points[j]) for i, j in enumerate(perm)]
        worst = max(d)
        if best is None or worst < best[0]:
            best = (worst, math.sqrt(sum(x * x for x in d) / len(d)), perm)
    return best


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <landmarks.yaml>", file=sys.stderr)
        return 2
    with open(sys.argv[1]) as handle:
        doc = yaml.safe_load(handle)
    marks = doc["landmarks"]
    names = [m["id"] for m in marks]
    pts = [(float(m["x"]), float(m["y"])) for m in marks]
    centroid = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))

    print(f"# Rotational symmetry of {sys.argv[1]}, by scripts/phase3_map_symmetry.py.")
    print("# worst_m is the largest distance a rotated landmark lands from the landmark it is")
    print("# matched to; compare it with the association gate (about 0.15 m at sigma 0.05 m).")
    print("landmarks:")
    for n, p in zip(names, pts):
        print(f"  {n}: [{p[0]}, {p[1]}]")
    print(f"landmark_centroid: [{centroid[0]:.4f}, {centroid[1]:.4f}]")
    print("rotations:")
    for deg in (90, 180, 270):
        angle = math.radians(deg)
        about_origin = best_match(pts, (0.0, 0.0), angle)
        about_centroid = best_match(pts, centroid, angle)
        best = None
        for ix in range(-100, 101):
            for iy in range(-100, 101):
                c = (ix / 100.0, iy / 100.0)
                m = best_match(pts, c, angle)
                if best is None or m[0] < best[0][0]:
                    best = (m, c)
        (worst, rms, perm), c = best
        print(f"  {deg}:")
        for label, (w, r, pm) in (("about_origin", about_origin),
                                  ("about_landmark_centroid", about_centroid)):
            print(f"    {label}: {{worst_m: {w:.3f}, rms_m: {r:.3f}}}")
        print(f"    best_centre: {{x: {c[0]:.2f}, y: {c[1]:.2f}, worst_m: {worst:.3f}, "
              f"rms_m: {rms:.3f}, matching: {[names[j] for j in perm]}}}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
