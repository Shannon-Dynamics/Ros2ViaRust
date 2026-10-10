#!/usr/bin/env python3
"""Turn one Phase 3 kidnap run into a recovery verdict, for one estimator.

    python3 -I scripts/phase3_kidnap_verdict.py <run.json> <run.csv> <environment.txt> <label>

Writes YAML on stdout. Called twice per kidnap run by scripts/run_phase3_correctness.sh,
once with label "ekf" and once with label "mcl" (whichever estimators that run included),
each against that estimator's own CSV and JSON. Nothing here recomputes what
localization_evaluator already wrote; the recovery time is the one new number, and it comes
from the CSV rows the evaluator produced, never typed in by hand.

This is not the P1 verdict of scripts/phase3_verdict.py. P1 asks whether tracking error
stays bounded and beats dead reckoning; this file asks the kidnapped robot question (Thrun
and Fox): after the belief is invalidated without warning, does the estimate find its way
back, and how long does that take. The two questions need different numbers, hence a
separate script rather than a mode flag on the other one.

## What "recovered" means here, exactly

The evaluator's own JSON already splits samples into before_kidnap and after_kidnap and
reports, for after_kidnap, the RMSE, the max error and the single best (smallest) position
error reached. None of those alone answer "did it recover": the best error could be one
lucky sample in an otherwise-lost filter, and the RMSE is dragged down by the understandably
bad samples right after the teleport even for a filter that does go on to recover.

So recovery is defined as: the position error drops below RECOVERY_POSITION_THRESHOLD_M,
the absolute yaw error drops below RECOVERY_YAW_THRESHOLD_RAD, and both *stay* below their
limits for RECOVERY_SUSTAINED_SAMPLES consecutive estimates. The first sample of the first
such run, timed from the kidnap event, is recovery_time_s.

* RECOVERY_POSITION_THRESHOLD_M = 0.15 m. Chosen as roughly nine times the before-kidnap P95 position
  error the EKF already reached on the same route without a kidnap
  (evidence/phase-3-correctness/2026-10-10-long-route/normal-verdict.yaml:
  p95_position_error_m = 0.0169 m): generous enough that ordinary tracking noise is not
  mistaken for a failure to recover, far tighter than the metres-scale error the estimate
  sits at immediately after a teleport to the opposite side of a 6 m room, so crossing it is
  a real claim about having found the right pose rather than merely stopped being "close" in
  a 6-metre room.
* RECOVERY_YAW_THRESHOLD_RAD = 0.15 rad. This is 8.6 degrees: loose enough for ordinary
  tracking noise, but tight enough to exclude a position coincidence with the robot facing
  the wrong direction. Position and yaw must pass in the same sustained window.
* RECOVERY_SUSTAINED_SAMPLES = 10. At the ~20 Hz the estimate is actually published (gated
  by max_publish_rate_hz, driven by 50 Hz odometry and 5 Hz scans), 10 consecutive samples
  is about half a second of staying close, long enough that a single sample passing through
  the threshold while the estimate is still moving past the true pose does not count as
  recovered. This is an independent, evaluator-side definition of recovery; it is not the
  same number as the sustained-low-fit streak that fires a particle injection inside
  robotics_localization::mcl, which counts scans, not pose-error samples, and exists to
  decide when to act rather than to judge whether recovery happened afterward.

Both constants are declared once, here, and are reported in the output file next to the
verdict they produced, so a reader never has to go hunting for what "recovered" meant.
"""

import json
import sys

RECOVERY_POSITION_THRESHOLD_M = 0.15
RECOVERY_YAW_THRESHOLD_RAD = 0.15
RECOVERY_SUSTAINED_SAMPLES = 10


def fmt(value, digits=6):
    if value is None:
        return "null"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def find_recovery(rows, kidnap_t):
    """First index i such that rows[i:i+N] are all after the kidnap and all below the
    threshold; returns (recovery_t_s, recovery_time_s) or (None, None)."""
    n = RECOVERY_SUSTAINED_SAMPLES
    for i in range(len(rows) - n + 1):
        window = rows[i : i + n]
        if all(
            r["after_kidnap"]
            and r["position_error_m"] < RECOVERY_POSITION_THRESHOLD_M
            and abs(r["yaw_error_rad"]) < RECOVERY_YAW_THRESHOLD_RAD
            for r in window
        ):
            t = window[0]["t_s"]
            return t, t - kidnap_t
    return None, None


def main():
    if len(sys.argv) != 5:
        print(f"usage: {sys.argv[0]} <run.json> <run.csv> <environment.txt> <label>",
              file=sys.stderr)
        return 2
    json_path, csv_path, env_path, label = sys.argv[1:5]

    try:
        with open(json_path) as handle:
            summary = json.load(handle)
    except (OSError, ValueError) as error:
        print(f"error: cannot read the run summary {json_path}: {error}")
        print("verdict: INCONCLUSIVE")
        return 1

    env = {}
    try:
        with open(env_path) as handle:
            for line in handle:
                if ": " in line:
                    key, value = line.split(": ", 1)
                    env[key.strip()] = value.strip()
    except OSError:
        pass

    rows = []
    try:
        with open(csv_path) as handle:
            header = handle.readline().rstrip("\n").split(",")
            index = {name: i for i, name in enumerate(header)}
            for line in handle:
                parts = line.rstrip("\n").split(",")
                if len(parts) != len(header):
                    continue
                rows.append({
                    "t_s": float(parts[index["t_s"]]),
                    "position_error_m": float(parts[index["position_error_m"]]),
                    "yaw_error_rad": float(parts[index["yaw_error_rad"]]),
                    "after_kidnap": parts[index["after_kidnap"]] == "1",
                })
    except (OSError, ValueError, KeyError) as error:
        print(f"error: cannot read the rows {csv_path}: {error}")
        print("verdict: INCONCLUSIVE")
        return 1

    kidnap = summary.get("kidnap")
    if kidnap is None:
        print(f"error: {json_path} has no kidnap event; this is not a kidnap run")
        print("verdict: INCONCLUSIVE")
        return 1
    kidnap_t = kidnap["seconds_into_run"]

    before = summary.get("before_kidnap", {})
    after = summary.get("after_kidnap", {})
    after_rows = [r for r in rows if r["after_kidnap"]]

    recovery_t, recovery_time_s = find_recovery(rows, kidnap_t)
    recovered = recovery_t is not None

    print("# Kidnap recovery verdict, derived from")
    print(f"# {json_path} and {csv_path} by scripts/phase3_kidnap_verdict.py.")
    print("# Not the P1 verdict of scripts/phase3_verdict.py; see this script's docstring")
    print("# for why recovery needs its own definition and its own file.")
    print("run:")
    print(f"  estimator: {label}")
    for key in ("scenario", "date_utc", "route_file", "route_sha256", "route_waypoints",
                "laps", "kidnap_after_s"):
        if key in env:
            print(f"  {key}: {env[key]}")
    print(f"  samples_total: {len(rows)}")
    print("kidnap_event:")
    print(f"  seconds_into_run: {fmt(kidnap_t)}")
    target = kidnap.get("target", {})
    print(f"  target_x: {fmt(target.get('x'))}")
    print(f"  target_y: {fmt(target.get('y'))}")
    print(f"  target_theta: {fmt(target.get('theta'))}")
    print("before_kidnap:")
    print(f"  samples: {before.get('samples', 0)}")
    print(f"  rmse_position_m: {fmt(before.get('rmse_position_m'))}")
    print(f"  final_position_error_m: {fmt(before.get('final_position_error_m'))}")
    print("after_kidnap:")
    print(f"  samples: {len(after_rows)}")
    print(f"  rmse_position_m: {fmt(after.get('rmse_position_m'))}")
    print(f"  max_position_error_m: {fmt(after.get('max_position_error_m'))}")
    print(f"  final_position_error_m: {fmt(after.get('final_position_error_m'))}")
    best_after = summary.get("after_kidnap", {}).get("best_position_error_after_kidnap")
    print(f"  best_position_error_m: {fmt(best_after)}")
    print("recovery:")
    print(f"  position_threshold_m: {RECOVERY_POSITION_THRESHOLD_M}")
    print(f"  yaw_threshold_rad: {RECOVERY_YAW_THRESHOLD_RAD}")
    print(f"  sustained_samples_required: {RECOVERY_SUSTAINED_SAMPLES}")
    print(f"  recovered: {str(recovered).lower()}")
    print(f"  recovered_at_seconds_into_run: {fmt(recovery_t)}")
    print(f"  recovery_time_s: {fmt(recovery_time_s)}")
    print(f"  verdict: {'RECOVERED' if recovered else 'DID_NOT_RECOVER'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
