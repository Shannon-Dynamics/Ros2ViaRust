#!/usr/bin/env python3
"""Aggregate a directory of single-teleport Webots kidnap sessions into a recovery rate.

    python3 -I scripts/phase3_kidnap_trials_summary.py <group directory>

<group directory> holds trial-NN/ subdirectories, each one a complete
`scripts/run_phase3_correctness.sh kidnap` session (one teleport) run with a different
MCL_SEED. Writes, next to them:

  kidnap-trials.csv           one row per trial and estimator, from that trial's own verdict
                              file and CSV; nothing is recomputed from memory or typed in
  kidnap-trials-summary.yaml  recovery rate with a Wilson 95% interval per estimator

"Recovered" is exactly the verdict of scripts/phase3_kidnap_verdict.py: position error
< 0.15 m and absolute yaw error < 0.15 rad, both held for 10 consecutive samples, measured
against simulator ground truth after the teleport. This file adds no definition of its own.

A session that did not produce a verdict file (the simulation or a node failed) is counted
as `no_verdict`, never as "did not recover": a broken session says nothing about the filter.
The interval treats trials as independent Bernoulli draws. Here they differ only in the
particle filter's seed and in simulator timing, with one fixed teleport (same target, same
time), so the interval describes the filter's randomness for that kidnap and not the
variety of possible kidnaps.
"""
import csv
import glob
import math
import os
import sys

Z95 = 1.959964


def wilson(k, n, z=Z95):
    if n == 0:
        return None, None
    p = k / n
    d = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / d
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, centre - half), min(1.0, centre + half)


def read_verdict(path):
    out = {}
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line.startswith("#") or ": " not in line:
                continue
            key, value = line.split(": ", 1)
            out[key] = value
    return out


def read_tail(csv_path):
    last = None
    with open(csv_path) as handle:
        for row in csv.DictReader(handle):
            last = row
    return last


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    group = sys.argv[1]
    trials = sorted(glob.glob(os.path.join(group, "trial-*")))
    trials = [t for t in trials if os.path.isdir(t)]
    if not trials:
        print(f"no trial-* directories in {group}", file=sys.stderr)
        return 1

    rows = []
    for t in trials:
        name = os.path.basename(t)
        seed = ""
        env = os.path.join(t, "kidnap-environment.txt")
        if os.path.exists(env):
            for line in open(env):
                if line.startswith("mcl_seed:"):
                    seed = line.split(":", 1)[1].strip()
        for label, vfile, cfile in (
            ("ekf", "kidnap-verdict-ekf.yaml", "kidnap-run.csv"),
            ("mcl", "kidnap-verdict-mcl.yaml", "kidnap-run-mcl.csv"),
        ):
            vpath = os.path.join(t, vfile)
            cpath = os.path.join(t, cfile)
            if not (os.path.exists(vpath) and os.path.exists(cpath)):
                continue
            v = read_verdict(vpath)
            verdict = v.get("verdict", "no_verdict")
            if verdict not in ("RECOVERED", "DID_NOT_RECOVER"):
                verdict = "no_verdict"
            tail = read_tail(cpath)
            rows.append({
                "trial": name,
                "estimator": label,
                "mcl_seed": seed if label == "mcl" else "",
                "verdict": verdict,
                "recovery_time_s": v.get("recovery_time_s", "null"),
                "after_kidnap_max_position_error_m": v.get("max_position_error_m", "null"),
                "final_position_error_m": tail["position_error_m"] if tail else "",
                "final_abs_yaw_error_rad": (
                    f"{abs(float(tail['yaw_error_rad'])):.6f}" if tail else ""
                ),
            })

    fields = list(rows[0].keys())
    with open(os.path.join(group, "kidnap-trials.csv"), "w", newline="") as handle:
        w = csv.DictWriter(handle, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    lines = [
        "# Kidnap recovery over repeated real Webots teleports, derived from kidnap-trials.csv",
        "# by scripts/phase3_kidnap_trials_summary.py. Recovery is the verdict of",
        "# scripts/phase3_kidnap_verdict.py (< 0.15 m and < 0.15 rad, held 10 samples).",
        "# One fixed teleport per session; trials differ only in the MCL seed.",
        f"group: {os.path.basename(os.path.abspath(group))}",
        f"sessions_found: {len(trials)}",
    ]
    for label in ("ekf", "mcl"):
        rs = [r for r in rows if r["estimator"] == label]
        n = sum(r["verdict"] in ("RECOVERED", "DID_NOT_RECOVER") for r in rs)
        k = sum(r["verdict"] == "RECOVERED" for r in rs)
        none = sum(r["verdict"] == "no_verdict" for r in rs)
        lo, hi = wilson(k, n)
        times = sorted(float(r["recovery_time_s"]) for r in rs
                       if r["verdict"] == "RECOVERED" and r["recovery_time_s"] != "null")
        lines += [
            f"{label}:",
            f"  trials_with_verdict: {n}",
            f"  no_verdict_sessions: {none}",
            f"  recovered: {k}",
            f"  recovery_rate: {k / n:.4f}" if n else "  recovery_rate: null",
            f"  wilson95_low: {lo:.4f}" if lo is not None else "  wilson95_low: null",
            f"  wilson95_high: {hi:.4f}" if hi is not None else "  wilson95_high: null",
        ]
        if times:
            mid = len(times) // 2
            median = times[mid] if len(times) % 2 else (times[mid - 1] + times[mid]) / 2
            lines += [
                f"  recovery_time_min_s: {times[0]:.3f}",
                f"  recovery_time_median_s: {median:.3f}",
                f"  recovery_time_max_s: {times[-1]:.3f}",
            ]
    with open(os.path.join(group, "kidnap-trials-summary.yaml"), "w") as handle:
        handle.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
