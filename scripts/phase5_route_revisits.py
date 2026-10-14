#!/usr/bin/env python3
"""How many true loops does a route contain? Geometry only: no run, no scan, no estimate.

    python3 -I scripts/phase5_route_revisits.py ROUTE.yaml [--laps N] [--angular RAD_S]

Drives the waypoints the way `route_driver` does (a turn in place, then a straight leg at
0.18 m/s, a scan every 0.2 s), places keyframes by the pose graph's own policy (0.5 m or 0.35 rad
since the last one), and counts the keyframe pairs that are a TRUE LOOP by the definition of
docs/phase-5-slam/benchmark.md section 2: at least 6 m of path apart and within 1.5 m of each
other. A route serves claims S4 and S5 if it has at least 20 such pairs (the fewest candidates
`scripts/phase5_verdict.py` will grade S5 on). This is the rule that decided which routes Phase 5
reuses (docs/phase-5-slam/implementation.md section 9, item 1).
"""
import math
import re
import sys

LINEAR, DT = 0.18, 0.2
KEYFRAME_TRANSLATION, KEYFRAME_ROTATION = 0.5, 0.35
MIN_PATH_GAP, MAX_DISTANCE, MIN_PAIRS = 6.0, 1.5, 20


def waypoints(path):
    pts = []
    for line in open(path):
        m = re.search(r"\{x:\s*(-?[\d.]+),\s*y:\s*(-?[\d.]+)\}", line)
        if m and not line.strip().startswith("#"):
            pts.append((float(m[1]), float(m[2])))
    return pts


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def trajectory(wps, angular):
    x = y = th = 0.0
    out = [(x, y, th)]
    for wx, wy in wps:
        target = math.atan2(wy - y, wx - x)
        d = wrap(target - th)
        n = max(0, math.ceil(abs(d) / (angular * DT) - 1e-9))
        for _ in range(n):
            th = wrap(th + d / n)
            out.append((x, y, th))
        n = max(0, math.ceil(math.hypot(wx - x, wy - y) / (LINEAR * DT) - 1e-9))
        for i in range(n):
            x += (wx - x) / (n - i)
            y += (wy - y) / (n - i)
            out.append((x, y, th))
        th = target
    return out


def keyframes(traj):
    kf, arcs, arc, last = [0], [0.0], 0.0, traj[0]
    for i in range(1, len(traj)):
        arc += math.hypot(traj[i][0] - traj[i - 1][0], traj[i][1] - traj[i - 1][1])
        if (math.hypot(traj[i][0] - last[0], traj[i][1] - last[1]) >= KEYFRAME_TRANSLATION
                or abs(wrap(traj[i][2] - last[2])) >= KEYFRAME_ROTATION):
            kf.append(i)
            arcs.append(arc)
            last = traj[i]
    return kf, arcs


def count(path, laps=1, angular=0.8):
    traj = trajectory(waypoints(path) * laps, angular)
    kf, arcs = keyframes(traj)
    pairs, with_partner = 0, set()
    for k in range(len(kf)):
        for j in range(k):
            if arcs[k] - arcs[j] >= MIN_PATH_GAP:
                a, b = traj[kf[j]], traj[kf[k]]
                if math.hypot(a[0] - b[0], a[1] - b[1]) < MAX_DISTANCE:
                    pairs += 1
                    with_partner.add(k)
    return {"scans": len(traj), "keyframes": len(kf), "path_m": arcs[-1],
            "true_loop_pairs": pairs, "keyframes_with_partner": len(with_partner)}


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    path, laps, angular = argv[0], 1, 0.8
    if "--laps" in argv:
        laps = int(argv[argv.index("--laps") + 1])
    if "--angular" in argv:
        angular = float(argv[argv.index("--angular") + 1])
    r = count(path, laps, angular)
    r["serves_s4_s5"] = r["true_loop_pairs"] >= MIN_PAIRS
    for k, v in r.items():
        print(f"{k}: {v:.1f}" if isinstance(v, float) else f"{k}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
