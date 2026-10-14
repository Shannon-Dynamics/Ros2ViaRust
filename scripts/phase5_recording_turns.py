#!/usr/bin/env python3
"""Does a recording contain scans taken while the robot turns, and how many, how fast?

    python3 -I scripts/phase5_recording_turns.py integrated-scans.csv [...]
    python3 -I scripts/phase5_recording_turns.py --yaml OUT.yaml integrated-scans.csv

Reads only the pose columns of the recording (stamp_s, sensor_x, sensor_y, sensor_theta: the schema of
scripts/phase5_recorded_gt_steps.py). Nothing is registered, nothing is graded.

Definitions, fixed here and not tuned to a file:

  * a *turning scan* is one whose recorded sensor heading differs from the previous recorded scan's by
    more than 0.015 rad, when the two are at most 0.5 s apart. 0.015 rad is the mapper's own motion
    gate (`motion_gate.max_yaw_rad`, robotics_mapping::MotionGate), the rule that removed turning scans
    from every Phase 4 recording, so "no turning scan" means what it meant there. For the first scan of
    a recording the previous one does not exist and it is not counted.
  * a *turn* is a maximal run of consecutive turning scans (a gap of more than 0.5 s ends the run) whose
    heading change has one sign. Its angle is the signed sum of the wrapped heading steps.
  * the rate of a step is its wrapped heading change over the stamp spacing, in rad/s. The stamps of
    this lidar are not the measurement instants (docs/phase-4-mapping/implementation.md 5.2 item 2), so
    a single step's rate is only good to a few per cent of its spacing; the median over a turn is what is
    reported.
"""
import math
import statistics
import sys

TURN_STEP_RAD = 0.015
MAX_GAP_S = 0.5


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def read(path):
    rows = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        f = line.split(",", 4)
        rows.append((float(f[0]), float(f[1]), float(f[2]), float(f[3])))
    return rows


def analyse(rows):
    steps = []  # (stamp, dt, dtheta, ds)
    for a, b in zip(rows, rows[1:]):
        dt = b[0] - a[0]
        if 0 < dt <= MAX_GAP_S:
            steps.append((b[0], dt, wrap(b[3] - a[3]), math.hypot(b[1] - a[1], b[2] - a[2])))
    turning = [s for s in steps if abs(s[2]) > TURN_STEP_RAD]
    turns, cur = [], []
    last_stamp = None
    for s in steps:
        t, dt, dth, _ = s
        is_turn = abs(dth) > TURN_STEP_RAD
        contiguous = last_stamp is not None and (t - last_stamp) <= MAX_GAP_S + 1e-9
        if is_turn and cur and contiguous and (dth > 0) == (cur[-1][2] > 0):
            cur.append(s)
        else:
            if cur:
                turns.append(cur)
            cur = [s] if is_turn else []
        last_stamp = t
    if cur:
        turns.append(cur)
    out = {
        "scans": len(rows),
        "duration_s": rows[-1][0] - rows[0][0] if rows else 0.0,
        "steps_at_most_0p5s_apart": len(steps),
        "turning_scans": len(turning),
        "turning_scan_share": len(turning) / len(steps) if steps else 0.0,
        "turns": [],
    }
    for tr in turns:
        rates = [s[2] / s[1] for s in tr]
        out["turns"].append({
            "start_stamp_s": tr[0][0] - tr[0][1],
            "scans": len(tr),
            "angle_rad": sum(s[2] for s in tr),
            "median_rate_rad_s": statistics.median(rates),
            "max_abs_step_rad": max(abs(s[2]) for s in tr),
        })
    if turning:
        r = [abs(s[2] / s[1]) for s in turning]
        out["abs_rate_rad_s"] = {"min": min(r), "median": statistics.median(r), "max": max(r)}
    return out


def render(path, a):
    L = [f"{path}",
         f"  scans: {a['scans']}   duration: {a['duration_s']:.1f} s   consecutive pairs <= 0.5 s apart: {a['steps_at_most_0p5s_apart']}",
         f"  turning scans (heading step > {TURN_STEP_RAD} rad): {a['turning_scans']} ({a['turning_scan_share'] * 100:.1f} per cent of steps)"]
    if a["turning_scans"]:
        r = a["abs_rate_rad_s"]
        L.append(f"  |heading rate| over turning scans: min {r['min']:.3f} / median {r['median']:.3f} / max {r['max']:.3f} rad/s")
    L.append(f"  turns (runs of turning scans of one sign): {len(a['turns'])}")
    for i, t in enumerate(a["turns"], 1):
        L.append(f"    {i:2d}. from {t['start_stamp_s']:8.2f} s: {t['scans']:2d} scans, angle {math.degrees(t['angle_rad']):7.1f} deg, "
                 f"median rate {t['median_rate_rad_s']:+.3f} rad/s, largest step {t['max_abs_step_rad']:.3f} rad")
    if a["turns"]:
        angles = [math.degrees(abs(t["angle_rad"])) for t in a["turns"]]
        L.append(f"  turn angle (abs): min {min(angles):.1f} / median {statistics.median(angles):.1f} / max {max(angles):.1f} deg; "
                 f"total {sum(angles):.0f} deg")
    return "\n".join(L)


def to_yaml(path, a):
    L = [f"recording: {path}", f"scans: {a['scans']}", f"duration_s: {a['duration_s']:.3f}",
         f"turn_step_threshold_rad: {TURN_STEP_RAD}", f"turning_scans: {a['turning_scans']}",
         f"turning_scan_share: {a['turning_scan_share']:.4f}", f"turn_count: {len(a['turns'])}"]
    if a["turning_scans"]:
        r = a["abs_rate_rad_s"]
        L += [f"abs_rate_rad_s_min: {r['min']:.4f}", f"abs_rate_rad_s_median: {r['median']:.4f}",
              f"abs_rate_rad_s_max: {r['max']:.4f}"]
    L.append("turns:")
    for t in a["turns"]:
        L.append(f"  - {{start_stamp_s: {t['start_stamp_s']:.3f}, scans: {t['scans']}, angle_rad: {t['angle_rad']:.4f}, "
                 f"median_rate_rad_s: {t['median_rate_rad_s']:.4f}}}")
    return "\n".join(L) + "\n"


def main(argv):
    yaml_out = None
    if argv[:1] == ["--yaml"]:
        yaml_out, argv = argv[1], argv[2:]
    if not argv:
        print(__doc__)
        return 2
    for p in argv:
        a = analyse(read(p))
        print(render(p, a))
        if yaml_out:
            open(yaml_out, "w").write(to_yaml(p, a))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
