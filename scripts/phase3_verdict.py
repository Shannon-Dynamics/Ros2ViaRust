#!/usr/bin/env python3
"""Turn one Phase 3 run into a verdict file.

    python3 -I scripts/phase3_verdict.py <run.json> <run.csv> <environment.txt>

Writes YAML on stdout. It is called by scripts/run_phase3_correctness.sh and reads only
files that run produced, so the verdict is derived from the measurement rather than typed
next to it. Nothing here recomputes the RMSE the evaluator already wrote; the percentiles
and the two half-run figures are the only new numbers, and they come from the CSV rows.

The criterion is P1 of docs/phase-3-localization/problem.md:

    absolute trajectory error of the estimate against ground truth, mean and 95th
    percentile, over a fixed drive: bounded over distance, and strictly better than the
    drift of /odom on the same run

Two of the three parts are decidable from these files:

  * strictly_better  the estimate RMSE is below the odometry RMSE of the same run.
  * bounded          the estimate error in the last quarter of the run is not materially
                     worse than in the first quarter, while the odometry error is compared
                     the same way for contrast. "Materially" is a factor of 1.5, which is
                     loose on purpose: this asks whether the error grows with distance, not
                     whether it is constant.

The NEES part of the criterion is not decided here. It needs the full 3x3 covariance of
each estimate and the CSV carries only sigma_xy, so this file reports
`nees_in_95_band: not_evaluated` rather than guessing.
"""

import json
import sys


def percentile(values, q):
    """Linear-interpolated percentile of an unsorted list."""
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    pos = (len(s) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (pos - lo)


def mean(values):
    return sum(values) / len(values) if values else None


def fmt(value, digits=6):
    if value is None:
        return "null"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def main():
    if len(sys.argv) != 4:
        print(f"usage: {sys.argv[0]} <run.json> <run.csv> <environment.txt>",
              file=sys.stderr)
        return 2
    json_path, csv_path, env_path = sys.argv[1:4]

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

    # Per-sample errors from the rows the evaluator wrote. Only the rows before a kidnap
    # are used: after a teleport the estimate is deliberately wrong and the P1 criterion is
    # about tracking, not recovery.
    est_err, odom_err, path = [], [], 0.0
    previous = None
    try:
        with open(csv_path) as handle:
            header = handle.readline().rstrip("\n").split(",")
            index = {name: i for i, name in enumerate(header)}
            for line in handle:
                row = line.rstrip("\n").split(",")
                if len(row) != len(header):
                    continue
                if row[index["after_kidnap"]] != "0":
                    continue
                est_err.append(float(row[index["position_error_m"]]))
                value = row[index["odom_position_error_m"]]
                if value.lower() not in ("nan", "-nan", ""):
                    odom_err.append(float(value))
                point = (float(row[index["gt_x"]]), float(row[index["gt_y"]]))
                if previous is not None:
                    path += ((point[0] - previous[0]) ** 2
                             + (point[1] - previous[1]) ** 2) ** 0.5
                previous = point
    except (OSError, ValueError, KeyError) as error:
        print(f"error: cannot read the rows {csv_path}: {error}")
        print("verdict: INCONCLUSIVE")
        return 1

    before = summary.get("before_kidnap", {})
    rmse_est = before.get("rmse_position_m")
    rmse_odom = before.get("odom_only_rmse_position_m")

    quarter = max(1, len(est_err) // 4)
    first_est, last_est = mean(est_err[:quarter]), mean(est_err[-quarter:])
    first_odom, last_odom = mean(odom_err[:quarter]), mean(odom_err[-quarter:])

    strictly_better = (rmse_est is not None and rmse_odom is not None
                       and rmse_est < rmse_odom)
    bounded = (first_est is not None and last_est is not None
               and last_est <= 1.5 * max(first_est, 1e-9))
    verdict = "PASS" if (strictly_better and bounded) else "FAIL"
    if rmse_est is None or rmse_odom is None:
        verdict = "INCONCLUSIVE"

    growth = (last_est / first_est) if first_est else None
    odom_growth = (last_odom / first_odom) if first_odom else None
    ratio = (rmse_est / rmse_odom) if rmse_odom else None

    print("# Verdict on criterion P1 of docs/phase-3-localization/problem.md, derived")
    print(f"# from {json_path} and {csv_path} by scripts/phase3_verdict.py.")
    print("run:")
    for key in ("scenario", "date_utc", "route_file", "route_sha256", "route_waypoints",
                "laps", "odometry_topic_under_test", "odometry_noise"):
        if key in env:
            print(f"  {key}: {env[key]}")
    print(f"  samples: {len(est_err)}")
    print(f"  ground_truth_path_length_m: {fmt(path, 3)}")
    print("estimate_vs_ground_truth:")
    print(f"  rmse_position_m: {fmt(rmse_est)}")
    print(f"  mean_position_error_m: {fmt(mean(est_err))}")
    print(f"  p95_position_error_m: {fmt(percentile(est_err, 0.95))}")
    print(f"  max_position_error_m: {fmt(before.get('max_position_error_m'))}")
    print(f"  rmse_yaw_rad: {fmt(before.get('rmse_yaw_rad'))}")
    print("odometry_vs_ground_truth:")
    print(f"  rmse_position_m: {fmt(rmse_odom)}")
    print(f"  mean_position_error_m: {fmt(mean(odom_err))}")
    print(f"  p95_position_error_m: {fmt(percentile(odom_err, 0.95))}")
    print(f"  max_position_error_m: {fmt(max(odom_err) if odom_err else None)}")
    print("growth_over_the_run:  # mean error of the last quarter over the first quarter")
    print(f"  estimate_first_quarter_m: {fmt(first_est)}")
    print(f"  estimate_last_quarter_m: {fmt(last_est)}")
    print(f"  estimate_growth_factor: {fmt(growth, 3)}")
    print(f"  odometry_first_quarter_m: {fmt(first_odom)}")
    print(f"  odometry_last_quarter_m: {fmt(last_odom)}")
    print(f"  odometry_growth_factor: {fmt(odom_growth, 3)}")
    print("criterion_p1:")
    print(f"  estimate_over_odometry_rmse_ratio: {fmt(ratio, 3)}")
    print(f"  strictly_better_than_odometry: {str(strictly_better).lower()}")
    print(f"  bounded_over_distance: {str(bounded).lower()}")
    print("  nees_in_95_band: not_evaluated  # needs the full covariance, see the docstring")
    print(f"  verdict: {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
