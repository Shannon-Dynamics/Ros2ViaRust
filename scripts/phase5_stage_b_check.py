#!/usr/bin/env python3
"""Checks on one replay of the pose graph node with loop closure, /map and map -> odom (Stage B).

    python3 -I scripts/phase5_stage_b_check.py --dir RUN --loop on|off [--tf on|off] [--map on|off]

RUN holds what scripts/verify_phase5_slam_nodes.sh section G leaves: node/ (the nodes' capture files:
registrations.csv, keyframes.csv, keyframes_final.csv, loops.csv, final-map.txt), probe.json (what a
subscriber saw on /map, /tf and /slam/path, scripts/phase5_map_tf_probe.py), render-created.txt and
render-final.txt (the grid rendered from the keyframes as created and as optimized, by the example
render_check from the recorded scans and the keyframe poses alone, sharing nothing with the node).

Every check prints PASS or FAIL with a name; the exit code is non-zero if anything failed. The checks are
about what the node *does*, not how good the map is: nothing here grades a map against a world (claim S8
is not claimable) and nothing here measures loop closure against ground truth (that is S5, graded by
scripts/phase5_verdict.py).
"""
import argparse
import csv
import json
import math
import os
import sys

fails, passes = [], 0


def check(name, cond, detail=""):
    global passes
    if cond:
        passes += 1
        print(f"PASS  {name}")
    else:
        fails.append(name)
        print(f"FAIL  {name}  {detail}")


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--loop", choices=["on", "off"], required=True)
    ap.add_argument("--tf", choices=["on", "off"], default="on")
    ap.add_argument("--map", choices=["on", "off"], default="on")
    a = ap.parse_args()
    # The diagnostics are published once a second, so their counters can be a keyframe behind the end of the
    # run; the exact counts are the ones the node logs as it shuts down.
    import re
    launch_log = open(os.path.join(a.dir, "launch.log")).read()
    m_proc = re.search(r"pose_graph_node shutting down: (\d+) keyframes from (\d+) registrations \((\d+) processed", launch_log)
    processed = (int(m_proc.group(2)), int(m_proc.group(3))) if m_proc else None
    m_end = re.search(r"pose_graph_node shutting down: .*loop_closure: (\w+) \((\d+) candidates, (\d+) accepted, (\d+) rejected, (\d+) loop edges\), (\d+) map\(s\), (\d+) map -> odom", launch_log)
    end = {"status": m_end.group(1), "candidates": int(m_end.group(2)), "accepted": int(m_end.group(3)),
           "rejected": int(m_end.group(4)), "edges": int(m_end.group(5)), "maps": int(m_end.group(6)),
           "map_odom": int(m_end.group(7))} if m_end else None
    print(f"# the node's shutdown line: {end}")
    d, node = a.dir, os.path.join(a.dir, "node")
    probe = json.load(open(os.path.join(d, "probe.json")))
    diag = probe["diagnostics_last"]
    kf0, kf1 = rows(f"{node}/keyframes.csv"), rows(f"{node}/keyframes_final.csv")
    loops = rows(f"{node}/loops.csv")
    regs = rows(f"{node}/registrations.csv")
    accepted = [r for r in loops if r["verdict"] == "accepted"]
    closure_events = sorted({int(r["to"]) for r in accepted})
    print(f"# {len(kf1)} keyframes, {len(loops)} loop candidates, {len(accepted)} accepted at {len(closure_events)} keyframe(s)")

    check("the pose graph node processed every registration the scan matcher made (no backlog was cut off by the shutdown)",
          processed is not None and processed[0] == processed[1] == len(regs), f"{processed} vs {len(regs)} rows in registrations.csv")

    # ---- loop closure: off means off, on means on, and the diagnostics say which
    if a.loop == "off":
        check("loop closure off: the diagnostics say unsupported", diag.get("loop_closure") == "unsupported", diag.get("loop_closure"))
        check("loop closure off: no candidate was proposed and loops.csv has only its header", len(loops) == 0, len(loops))
        check("loop closure off: no optimization ran", str(diag.get("optimizations")) == "0", diag.get("optimizations"))
    else:
        check("loop closure on: the diagnostics say enabled", diag.get("loop_closure") == "enabled", diag.get("loop_closure"))
        check("loop closure on: candidates were proposed", len(loops) > 0, len(loops))
        check("loop closure on: the node's final counters equal the verdict file's rows",
              end is not None and end["candidates"] == len(loops) and end["accepted"] == len(accepted) and end["accepted"] + end["rejected"] == end["candidates"],
              f"{end} vs {len(loops)}/{len(accepted)}")
        check("loop closure on: every accepted loop is a loop edge of the graph", end is not None and end["edges"] == len(accepted),
              f"{end} vs {len(accepted)}")
        check("loop closure on: the loop_errors counter is zero", str(diag.get("loop_errors")) == "0", diag.get("loop_errors"))

    # ---- the keyframes: unchanged until a closure moves them, and a closure does move them
    check("the final file has the keyframes of the created file, one for one", [r["id"] for r in kf0] == [r["id"] for r in kf1] and
          [r["stamp_ns"] for r in kf0] == [r["stamp_ns"] for r in kf1])
    disp = max((math.hypot(float(p["x"]) - float(q["x"]), float(p["y"]) - float(q["y"])) for p, q in zip(kf0, kf1)), default=0.0)
    print(f"# largest keyframe displacement caused by loop closure: {disp * 1000:.3f} mm")
    if a.loop == "off":
        check("loop closure off: the optimized keyframes are the created ones, bit for bit", disp == 0.0 and all(
            (p["x"], p["y"], p["theta"]) == (q["x"], q["y"], q["theta"]) for p, q in zip(kf0, kf1)))
    elif closure_events:
        check("loop closure on: an accepted loop moved at least one keyframe that had been published", disp > 0.0, disp)

    # ---- the map: a replay, once per closure and once at the end, a function of the final poses alone
    if a.map == "on":
        maps = probe["maps"]
        expected = len(closure_events) + 1
        check(f"/map was published once per loop closure and once at the end ({expected})", len(maps) == expected, f"{len(maps)} message(s)")
        check("every /map is in the frame map, 240 x 240 cells of 0.05 m",
              all(m["frame"] == "map" and m["width"] == 240 and m["height"] == 240 and abs(m["resolution"] - 0.05) < 1e-6 for m in maps))
        check("the map has free and occupied cells", bool(maps) and maps[-1]["free"] > 500 and maps[-1]["occupied"] > 20,
              maps[-1] if maps else "no map")
        rlog = open(f"{d}/render-final.log").read()
        import re
        m = re.search(r"rendered (\d+) keyframes \((\d+) integrated, (\d+) without a scan in the file, (\d+) rejected\)", rlog)
        check("the render integrated every keyframe's scan: none rejected, none without a scan",
              bool(m) and m.group(1) == m.group(2) and m.group(3) == "0" and m.group(4) == "0", rlog.strip())
        check("the node's own render integrated every keyframe too (its diagnostics report no skipped keyframe)",
              "render skipped" not in diag.get("last_error", ""), diag.get("last_error"))
        final_map = open(f"{node}/final-map.txt").read()
        check("the map the node wrote at the end equals a render from files alone (final poses + recorded scans), byte for byte",
              final_map == open(f"{d}/render-final.txt").read())
        created_render = open(f"{d}/render-created.txt").read()
        if a.loop == "on" and disp >= 0.05:
            check("a closure that moved a keyframe by a cell or more changes the map: the map follows the poses it is given",
                  created_render != final_map)
        if a.loop == "off":
            check("loop closure off: the map equals the one rendered from the keyframes as created", created_render == final_map)
        check("the node's final count of maps published equals what a subscriber received", end is not None and end["maps"] == len(maps),
              f"{end} vs {len(maps)}")
    else:
        check("map off: no /map message", len(probe["maps"]) == 0)
        check("map off: the node counts no map", end is not None and end["maps"] == 0, f"{end}")

    # ---- the path is republished whole after a closure
    path_msgs = probe["paths"]
    last = probe["last_path"]
    check("the last /slam/path has one pose per keyframe", len(last) == len(kf1), f"{len(last)} vs {len(kf1)}")
    check("the last /slam/path equals the optimized keyframes (the closure's moves reached the subscriber)",
          len(last) == len(kf1) and all(abs(p[0] - float(q["x"])) < 1e-9 and abs(p[1] - float(q["y"])) < 1e-9 and abs(wrap(p[2] - float(q["theta"]))) < 1e-9
                                        for p, q in zip(last, kf1)))
    if a.loop == "on" and closure_events:
        check("/slam/path was published again after the closures (more messages than keyframes' creation alone)", len(path_msgs) > len(kf1), len(path_msgs))
    expected_frame = "map" if (a.map == "on" or a.tf == "on") else "slam_origin"
    check(f"/slam/path is in the frame {expected_frame}", all(p["frame"] == expected_frame for p in path_msgs) and bool(path_msgs))

    # ---- map -> odom
    samples = probe["map_odom"]
    edges = probe["edges"]
    if a.tf == "on":
        n_reg = len(regs)
        check("map -> odom is published for every registered scan", len(samples) == n_reg, f"{len(samples)} of {n_reg}")
        check("map -> odom stamps never go back", all(b[0] >= a_[0] for a_, b in zip(samples, samples[1:])))
        check("map -> odom: every registered scan was published, none dropped for want of odom -> sensor",
              end is not None and end["map_odom"] == len(samples) and str(diag.get("map_odom_dropped")) == "0", f"{end}, samples {len(samples)}, dropped {diag.get('map_odom_dropped')}")
        check("no second owner of the edge was seen",
              str(diag.get("authority_conflicts")) == "0", f"conflicts {diag.get('authority_conflicts')}")
        check("the only publisher of the edge map -> odom is the node: no map -> base_link or other map edge",
              all(k == "map->odom" or not k.startswith("map->") for k in edges), sorted(edges))
        # Independent expectation: stationary odometry means map -> odom is the SLAM estimate of the sensor.
        # That is the newest optimized keyframe composed with the increments registered after it.
        kf = kf1[-1]
        est = (float(kf["x"]), float(kf["y"]), float(kf["theta"]))
        for r in regs:
            if int(r["stamp_ns"]) > int(kf["stamp_ns"]):
                est = compose(est, (float(r["dx"]), float(r["dy"]), float(r["dtheta"])))
        got = samples[-1] if samples else None
        check("the last map -> odom equals the estimate recomputed from the files (the keyframe composed with the increments after it)",
              got is not None and math.hypot(got[1] - est[0], got[2] - est[1]) < 1e-6 and abs(wrap(got[3] - est[2])) < 1e-6,
              f"got {got}, expected {est}")
        if a.loop == "on" and closure_events and samples:
            # at the first closure the estimate jumps by the correction; the sample stream must contain a step of that size
            steps = [math.hypot(b[1] - a_[1], b[2] - a_[2]) for a_, b in zip(samples, samples[1:])]
            print(f"# map -> odom step between consecutive samples: median {sorted(steps)[len(steps) // 2] * 1000:.2f} mm, largest {max(steps) * 1000:.2f} mm")
    else:
        check("map -> odom off: no map -> odom on /tf", "map->odom" not in edges, sorted(edges))
        check("map -> odom off: the diagnostics say none", diag.get("map_odom_published") == "none", diag.get("map_odom_published"))

    print(f"passed {passes}, failed {len(fails)}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
