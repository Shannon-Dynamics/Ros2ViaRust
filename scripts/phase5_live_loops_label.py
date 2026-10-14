#!/usr/bin/env python3
"""S5 safety count on the loop constraints a LIVE run's pose_graph_node itself accepted.

    python3 -I scripts/phase5_live_loops_label.py NODE_DIR SCANS.csv [--label NAME]

NODE_DIR holds loops.csv and keyframes.csv written by the node (capture:=true); SCANS.csv is a recording whose pose
columns are the labels to grade against (the recorded ones, or those of scripts/phase5_lattice_from_ranges.py).
An accepted constraint is wrong if its measurement X_from^-1 X_to differs from the labelled relative pose of the
two keyframes' scans by more than 0.05 m or 0.03 rad (the limits of scripts/phase5_verdict.py, which this script does
not own: it only counts). The keyframe -> scan link is the stamp. Reads files only.
"""
import csv, math, sys

WRONG_M, WRONG_RAD = 0.05, 0.03

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def inv(a):
    c, s = math.cos(a[2]), math.sin(a[2]); return (-(c * a[0] + s * a[1]), s * a[0] - c * a[1], wrap(-a[2]))
def comp(a, b):
    c, s = math.cos(a[2]), math.sin(a[2]); return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))

def main():
    node, scans = sys.argv[1], sys.argv[2]
    pose = {}
    for l in open(scans):
        if l.startswith('#') or not l.strip(): continue
        f = l.split(',', 4)
        pose[round(float(f[0]) * 1e9)] = (float(f[1]), float(f[2]), float(f[3]))
    kf = {int(r['id']): int(r['stamp_ns']) for r in csv.DictReader(open(f'{node}/keyframes.csv'))}
    acc = wrong = rej = missing = 0
    worst = []
    for r in csv.DictReader(open(f'{node}/loops.csv')):
        if r['verdict'] != 'accepted':
            rej += 1; continue
        a, b = kf[int(r['from'])], kf[int(r['to'])]
        if a not in pose or b not in pose:
            missing += 1; continue
        truth = comp(inv(pose[a]), pose[b])
        meas = (float(r['meas_x']), float(r['meas_y']), float(r['meas_theta']))
        e = comp(inv(meas), truth)
        acc += 1
        d, t = math.hypot(e[0], e[1]), abs(e[2])
        worst.append((d, t))
        if not (d < WRONG_M and t < WRONG_RAD): wrong += 1
    print(f"accepted {acc} (no label for {missing}), rejected {rej}, accepted_wrong {wrong}")
    print(f"error of accepted constraints: max {max(w[0] for w in worst)*1000:.1f} mm, {max(w[1] for w in worst):.4f} rad; "
          f"median {sorted(w[0] for w in worst)[len(worst)//2]*1000:.1f} mm, {sorted(w[1] for w in worst)[len(worst)//2]:.4f} rad")

main()
