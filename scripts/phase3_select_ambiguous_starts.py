#!/usr/bin/env python3
"""Pick the start poses of the ambiguity-targeted global localization trials.

    python3 -I scripts/phase3_select_ambiguous_starts.py <ambiguity.csv> <global_trials binary> <landmarks.yaml> [count]

The rule is fixed here, in code, before any trial on the selected poses has been run:

1. rank every row of ambiguity.csv (written by `global_trials ambiguity`) by the number of
   separate pose-space regions that explain one static scan, most first, ties broken by the
   width of the widest region, widest first;
2. keep the first `count` (default 8) of them for which the exploration route generator finds
   a clear route, since a trial needs a route to be driven;
3. add the spawn pose (0, 0, 0), which sits at the centre of the map's near rotational
   symmetry (scripts/phase3_map_symmetry.py), and its 180 degree twin (0, 0, pi).

Writes the starts file (x,y,yaw) on stdout and the reasoning, per pose, on stderr.
"""
import csv
import math
import subprocess
import sys


def route_exists(binary, landmarks, x, y, yaw):
    run = subprocess.run(
        [binary, "route", "--map", landmarks, "--x", str(x), "--y", str(y), "--yaw", str(yaw)],
        capture_output=True,
    )
    return run.returncode == 0


def main():
    if len(sys.argv) not in (4, 5):
        print(__doc__, file=sys.stderr)
        return 2
    table, binary, landmarks = sys.argv[1:4]
    count = int(sys.argv[4]) if len(sys.argv) == 5 else 8
    rows = list(csv.DictReader(open(table)))
    rows.sort(key=lambda r: (-int(r["modes"]), -float(r["widest_mode_m"] or 0.0)))
    chosen, skipped = [], 0
    for r in rows:
        if len(chosen) == count:
            break
        x, y, yaw = float(r["x"]), float(r["y"]), float(r["yaw"])
        if route_exists(binary, landmarks, x, y, yaw):
            chosen.append((x, y, yaw, "ambiguity rank, modes=%s widest=%sm visible=%s" %
                           (r["modes"], r["widest_mode_m"], r["visible_pillars"])))
        else:
            skipped += 1
    chosen.append((0.0, 0.0, 0.0, "spawn pose, centre of the near symmetry"))
    chosen.append((0.0, 0.0, math.pi, "180 degree twin of the spawn pose"))
    print("x,y,yaw")
    for x, y, yaw, why in chosen:
        print(f"{x:.4f},{y:.4f},{yaw:.4f}")
        print(f"  ({x:6.2f}, {y:6.2f}, {yaw:6.3f})  {why}", file=sys.stderr)
    print(f"  skipped {skipped} higher-ranked starts with no clear route", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
