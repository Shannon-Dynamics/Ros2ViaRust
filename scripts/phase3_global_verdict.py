#!/usr/bin/env python3
"""Grade global localization trials from a uniform prior, against simulator ground truth.

    python3 -I scripts/phase3_global_verdict.py trial   <trial directory>
    python3 -I scripts/phase3_global_verdict.py summary <run directory>

Writes YAML on stdout. Called by scripts/run_phase3_global_trials.sh once per trial and once
for the run. Nothing is typed in: every number is computed from the files a trial wrote,
`mcl_node.log` (the filter's own claims, with their timestamps), `run.csv` (the evaluator's
estimate against ground truth, per sample) and `diagnostics.yaml`.

The definitions are those of crates/robotics_localization/examples/global_trials.rs, the
model-level experiment, so the two can be read side by side:

* convergence is **when the filter says so**: the GLOBAL_LOCALIZATION_CONVERGED line of
  mcl_node, whose own criterion is documented in crates/robotics_localization/src/global.rs.
  Whether it was right is decided here, from ground truth, never by the filter;
* success needs the position error below POSITION_TOLERANCE_M and the absolute yaw error
  below YAW_TOLERANCE_RAD (the thresholds of scripts/phase3_kidnap_verdict.py) both at the
  moment of the claim and at the end of the route;
* the four outcomes are SUCCESS, WRONG_CONVERGENCE (the claim was made and was wrong),
  LOST_AFTER_CONVERGENCE (right when made, wrong at the end), and NO_CONVERGENCE (no claim);
* a trial in which the robot did not move is INVALID, for the reason scripts/phase3_guards.sh
  exists: grading an estimate against a robot that never moved grades nothing.

Two things are graded next to the category, because the category alone hides them:

* `correct_at_end_of_route` is ground truth only (the last sample inside both tolerances),
  whatever the filter claimed. A wrong claim that the kidnap trigger later reset into the
  right answer is WRONG_CONVERGENCE and correct at the end; a right claim that the trigger
  reset into a wrong one is LOST_AFTER_CONVERGENCE and not. The summary counts both columns.
* every kidnap injection is graded by the estimate just before it fired: `false` when that
  estimate was already inside both tolerances (the trigger reset a correct belief, which is
  what spurious detections do to the upper-median gate), `true` when it was not. It needs
  the `KIDNAP_INJECTION stamp_s=` form of the log line; an older log has no stamp, and its
  injections are counted as `ungraded`.
"""

import csv
import glob
import math
import os
import re
import sys

import yaml

POSITION_TOLERANCE_M = 0.15
YAW_TOLERANCE_RAD = 0.15
MIN_DISPLACEMENT_M = 0.2

INJECTION = re.compile(r"KIDNAP_INJECTION stamp_s=([\d.]+)")
STARTED = re.compile(r"GLOBAL_LOCALIZATION_STARTED stamp_s=([\d.]+) particles=(\d+)")
CONVERGED = re.compile(
    r"GLOBAL_LOCALIZATION_CONVERGED stamp_s=([\d.]+) started_stamp_s=([\d.]+) "
    r"scored_updates=(\d+) travel_odom_m=([\d.]+) sigma_xy=([\d.]+) sigma_theta=([\d.]+) "
    r"particles_before=(\d+) particles_after=(\d+)"
)


def read_rows(path):
    rows = []
    with open(path) as handle:
        header = handle.readline().rstrip("\n").split(",")
        index = {name: i for i, name in enumerate(header)}
        for line in handle:
            parts = line.rstrip("\n").split(",")
            if len(parts) != len(header):
                continue
            rows.append({
                "stamp": float(parts[index["stamp_s"]]),
                "gx": float(parts[index["gt_x"]]),
                "gy": float(parts[index["gt_y"]]),
                "pos": float(parts[index["position_error_m"]]),
                "yaw": abs(float(parts[index["yaw_error_rad"]])),
            })
    return rows


def diagnostics_values(path):
    """The key/value pairs of the mcl_node status in a `ros2 topic echo --once /diagnostics`."""
    out = {}
    try:
        with open(path) as handle:
            for doc in yaml.safe_load_all(handle):
                for status in (doc or {}).get("status", []):
                    if "mcl_node" in status.get("name", ""):
                        out = {v["key"]: v["value"] for v in status.get("values", [])}
                        out["_message"] = status.get("message", "")
    except (OSError, yaml.YAMLError):
        pass
    return out


def analyse_trial(directory):
    r = {"directory": os.path.basename(directory.rstrip("/"))}
    start_txt = os.path.join(directory, "start.txt")
    if os.path.exists(start_txt):
        m = re.search(r"x=(\S+) y=(\S+) yaw=(\S+)", open(start_txt).read())
        if m:
            r["start"] = [float(m.group(1)), float(m.group(2)), float(m.group(3))]
    if os.path.exists(os.path.join(directory, "SKIPPED.txt")):
        r["category"] = "SKIPPED"
        return r

    log = open(os.path.join(directory, "mcl_node.log"), errors="replace").read()
    started = STARTED.search(log)
    converged = CONVERGED.search(log)
    r["injections"] = len(re.findall(r"injecting random particles", log))
    injection_stamps = [float(m.group(1)) for m in INJECTION.finditer(log)]
    csv_path = os.path.join(directory, "run.csv")
    rows = read_rows(csv_path) if os.path.exists(csv_path) else []
    if not started or not rows:
        r["category"] = "INVALID"
        r["reason"] = "the filter never started" if not started else "no evaluator rows"
        return r

    start_stamp = float(started.group(1))
    r["global_particles"] = int(started.group(2))
    displacement = max(math.hypot(x["gx"] - rows[0]["gx"], x["gy"] - rows[0]["gy"]) for x in rows)
    r["robot_displacement_m"] = displacement
    if displacement < MIN_DISPLACEMENT_M:
        r["category"] = "INVALID"
        r["reason"] = "the robot never moved"
        return r

    r["samples"] = len(rows)
    r["final_position_error_m"] = rows[-1]["pos"]
    r["final_yaw_error_rad"] = rows[-1]["yaw"]
    diag = diagnostics_values(os.path.join(directory, "diagnostics.yaml"))
    r["diagnostics_captured"] = bool(diag)
    for key in ("observe_ms_last", "observe_ms_max", "scans", "detections", "particles"):
        if key in diag:
            r[key] = float(diag[key]) if key.startswith("observe") else int(diag[key])

    def ok(pos, yaw):
        return pos < POSITION_TOLERANCE_M and yaw < YAW_TOLERANCE_RAD

    r["correct_at_end_of_route"] = ok(rows[-1]["pos"], rows[-1]["yaw"])
    false_injections = true_injections = 0
    for stamp in injection_stamps:
        before = [x for x in rows if x["stamp"] < stamp]
        if before and ok(before[-1]["pos"], before[-1]["yaw"]):
            false_injections += 1
        else:
            true_injections += 1
    r["injections_false"] = false_injections
    r["injections_true"] = true_injections
    r["injections_ungraded"] = r["injections"] - len(injection_stamps)

    if not converged:
        r["category"] = "NO_CONVERGENCE"
        return r
    conv_stamp = float(converged.group(1))
    r["convergence_time_s"] = conv_stamp - start_stamp
    r["convergence_scored_updates"] = int(converged.group(3))
    r["convergence_travel_odom_m"] = float(converged.group(4))
    r["convergence_sigma_xy_m"] = float(converged.group(5))
    r["convergence_sigma_theta_rad"] = float(converged.group(6))
    r["particles_before"] = int(converged.group(7))
    r["particles_after"] = int(converged.group(8))

    window = [x for x in rows if start_stamp <= x["stamp"] <= conv_stamp]
    path = sum(math.hypot(b["gx"] - a["gx"], b["gy"] - a["gy"]) for a, b in zip(window, window[1:]))
    r["convergence_travel_true_m"] = path
    at_claim = next((x for x in rows if x["stamp"] >= conv_stamp), rows[-1])
    r["position_error_at_convergence_m"] = at_claim["pos"]
    r["yaw_error_at_convergence_rad"] = at_claim["yaw"]
    after = [x for x in rows if x["stamp"] >= conv_stamp]
    r["rmse_position_after_convergence_m"] = math.sqrt(sum(x["pos"] ** 2 for x in after) / len(after))
    r["max_position_error_after_convergence_m"] = max(x["pos"] for x in after)
    if not ok(at_claim["pos"], at_claim["yaw"]):
        r["category"] = "WRONG_CONVERGENCE"
    elif not ok(rows[-1]["pos"], rows[-1]["yaw"]):
        r["category"] = "LOST_AFTER_CONVERGENCE"
    else:
        r["category"] = "SUCCESS"
    return r


def median(values):
    values = sorted(values)
    return values[len(values) // 2] if values else float("nan")


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def wilson(successes, n):
    if n == 0:
        return float("nan"), float("nan")
    z = 1.96
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def emit(name, value, indent):
    pad = "  " * indent
    if isinstance(value, float):
        print(f"{pad}{name}: {value:.4f}")
    elif isinstance(value, list):
        print(f"{pad}{name}: [{', '.join(f'{v:.3f}' if isinstance(v, float) else str(v) for v in value)}]")
    else:
        print(f"{pad}{name}: {value}")


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ("trial", "summary"):
        print(f"usage: {sys.argv[0]} trial <dir> | summary <run dir>", file=sys.stderr)
        return 2
    if sys.argv[1] == "trial":
        result = analyse_trial(sys.argv[2])
        print(f"# Global localization trial verdict for {sys.argv[2]}, by scripts/phase3_global_verdict.py.")
        print("trial:")
        for key, value in result.items():
            emit(key, value, 1)
        return 0

    run = sys.argv[2]
    trials = [analyse_trial(d) for d in sorted(glob.glob(os.path.join(run, "trial-*")))]
    env = {}
    try:
        for line in open(os.path.join(run, "environment.txt")):
            if ": " in line:
                k, v = line.split(": ", 1)
                env[k.strip()] = v.strip()
    except OSError:
        pass
    graded = [t for t in trials if t["category"] in
              ("SUCCESS", "WRONG_CONVERGENCE", "LOST_AFTER_CONVERGENCE", "NO_CONVERGENCE")]
    count = lambda c: sum(1 for t in graded if t["category"] == c)
    successes = [t for t in graded if t["category"] == "SUCCESS"]
    lo, hi = wilson(len(successes), len(graded))
    print(f"# Summary of {run}/trial-*, by scripts/phase3_global_verdict.py; no number is typed.")
    print("run:")
    for key in ("run", "date_utc", "build", "global_particles", "steady_state_particles",
                "convergence_min_explained_fraction", "seed_of_trial_0", "laps", "git_head",
                "mcl_node_sha256", "starts_sha256", "odometry_noise"):
        if key in env:
            emit(key, env[key], 1)
    print(f"  success_position_tolerance_m: {POSITION_TOLERANCE_M}")
    print(f"  success_yaw_tolerance_rad: {YAW_TOLERANCE_RAD}")
    print(f"trials_total: {len(trials)}")
    print(f"trials_graded: {len(graded)}")
    print(f"trials_skipped_no_route: {sum(1 for t in trials if t['category'] == 'SKIPPED')}")
    print(f"trials_invalid: {sum(1 for t in trials if t['category'] == 'INVALID')}")
    print("outcomes:")
    for c in ("SUCCESS", "WRONG_CONVERGENCE", "LOST_AFTER_CONVERGENCE", "NO_CONVERGENCE"):
        print(f"  {c}: {count(c)}")
    # The claim-based count above and the ground-truth one below are different questions:
    # what the filter said, and where it actually ended up. Both are always reported.
    correct_end = [t for t in graded if t.get("correct_at_end_of_route")]
    print("correct_at_end_of_route:")
    print(f"  total: {len(correct_end)}")
    for c in ("SUCCESS", "WRONG_CONVERGENCE", "LOST_AFTER_CONVERGENCE", "NO_CONVERGENCE"):
        print(f"  among_{c}: {sum(1 for t in correct_end if t['category'] == c)}")
    if graded:
        print(f"success_rate: {len(successes) / len(graded):.4f}")
        print(f"success_rate_wilson95: [{lo:.4f}, {hi:.4f}]")
    times = [t["convergence_time_s"] for t in successes]
    dists = [t["convergence_travel_true_m"] for t in successes]
    rmse = [t["rmse_position_after_convergence_m"] for t in successes]
    print("among_successes:")
    print(f"  convergence_time_s: {{mean: {mean(times):.2f}, median: {median(times):.2f}}}")
    print(f"  convergence_distance_m: {{mean: {mean(dists):.3f}, median: {median(dists):.3f}}}")
    print(f"  rmse_position_after_convergence_m: {{mean: {mean(rmse):.4f}, median: {median(rmse):.4f}}}")
    # The map has four pillars, so a trial that averages more than four detections per scan
    # must be reporting detections that are not pillars (a wall seen at close range). Fewer
    # than four proves nothing, so this is a lower bound on how often it happens.
    # Not every trial has a reading: an early version of the runner took one /diagnostics
    # message with `--once`, which is whichever node published first and often not mcl_node.
    # So the count of trials with a reading is reported next to every count taken from one.
    per_scan = [t["detections"] / t["scans"] for t in graded if t.get("scans")]
    print("detector:")
    print(f"  trials_with_a_diagnostics_reading: {len(per_scan)}")
    if per_scan:
        print(f"  detections_per_scan_median_over_those_trials: {median(per_scan):.2f}")
        print(f"  trials_averaging_more_than_4_detections_per_scan: {sum(1 for d in per_scan if d > 4.0)}")
    kidnap_after = [t for t in graded if t.get("injections", 0) > 0]
    print(f"trials_where_the_kidnap_trigger_fired: {len(kidnap_after)}")
    print("kidnap_injections:")
    print(f"  total: {sum(t.get('injections', 0) for t in graded)}")
    print(f"  false_estimate_was_already_correct: {sum(t.get('injections_false', 0) for t in graded)}")
    print(f"  true_estimate_was_wrong: {sum(t.get('injections_true', 0) for t in graded)}")
    print(f"  ungraded_log_has_no_stamp: {sum(t.get('injections_ungraded', 0) for t in graded)}")
    print(f"  trials_with_a_false_injection: {sum(1 for t in graded if t.get('injections_false', 0) > 0)}")
    obs = [t["observe_ms_max"] for t in graded if "observe_ms_max" in t]
    if obs:
        print("cost:")
        print(f"  observe_ms_max_over_trials: {max(obs):.2f}")
        print(f"  observe_ms_max_median_over_trials: {median(obs):.2f}")
    print("per_trial:")
    for t in trials:
        fields = [f"category: {t['category']}"]
        if "correct_at_end_of_route" in t:
            fields.append(f"correct_at_end_of_route: {t['correct_at_end_of_route']}")
        if t.get("injections"):
            fields.append(f"injections: {t['injections']}")
            if t.get("injections_false"):
                fields.append(f"injections_false: {t['injections_false']}")
        if "start" in t:
            fields.insert(0, f"start: [{', '.join(f'{v:.2f}' for v in t['start'])}]")
        for key, fmt in (("convergence_time_s", "{:.1f}"), ("convergence_travel_true_m", "{:.2f}"),
                         ("position_error_at_convergence_m", "{:.3f}"),
                         ("yaw_error_at_convergence_rad", "{:.3f}"),
                         ("final_position_error_m", "{:.3f}"), ("observe_ms_max", "{:.1f}")):
            if key in t:
                fields.append(f"{key}: {fmt.format(t[key])}")
        print(f"  {t['directory']}: {{{', '.join(fields)}}}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
