#!/usr/bin/env python3
"""Write benchmarks/phase-5-slam.csv from the metrics files of the Phase 5 evidence runs.

    python3 -I scripts/phase5_benchmark_csv.py [evidence/phase-5-slam] [benchmarks/phase-5-slam.csv]

One row per run, per entry, per metric, in the long format of benchmarks/README.md section 3. Nothing
in the CSV is typed by hand and nothing is computed here beyond flattening: every value is a number
in some run's `slam-metrics.yaml`, and the rows are written in a fixed order, so running this twice
gives the same file. The environment columns come from the run's own `environment.txt`.

Entries: `rust_slam` (the scan matcher, the front end and the pose graph: S1a, S1b, S2, S4, S5, S6),
`rust_ekf_slam` and `rust_rbpf` (S7, on synthetic odometry), `dead_reckoning_synthetic` (the S7 odometry
alone). The comparison with other implementations that benchmarks/README.md describes (rclpy, `slam_toolbox`)
does not exist yet; those entries are absent, not empty.
"""
import csv
import os
import sys

import yaml

COLUMNS = ["phase", "entry", "language", "seed", "config", "metric", "value", "unit", "host", "cpu",
           "kernel", "ros_distro", "webots_version", "rustc_version", "rmw", "headless", "timestamp", "commit"]


def unit_of(metric):
    leaf = metric.split(".")[-1]
    for suffix, unit in (("_m_per_m", "m/m"), ("_rad_per_m", "rad/m"), ("_rad", "rad"), ("_m", "m"),
                         ("_s", "s"), ("_fraction", "fraction"), ("_per_m", "1/m")):
        if leaf.endswith(suffix):
            return unit
    if leaf in ("count", "made", "graded", "samples", "trials", "successes", "candidates", "landmarks",
                "particles", "resamples", "keyframes_made", "keyframes_graded", "provisional_left",
                "distinct_pillars", "nees_samples") or "registrations" in leaf:
        return "count"
    if "nees" in leaf or "n_eff" in leaf:
        return "dimensionless"
    return ""


def flatten(prefix, node, out):
    if isinstance(node, dict):
        for k in sorted(node):
            flatten(f"{prefix}.{k}" if prefix else str(k), node[k], out)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            if isinstance(v, dict):
                tag = f"offset{v.get('offset_m')}m_{v.get('offset_rad')}rad"
                flatten(f"{prefix}[{tag}]", {k: x for k, x in v.items() if k not in ("offset_m", "offset_rad")}, out)
            else:
                flatten(f"{prefix}[{i}]", v, out)
    elif isinstance(node, bool):
        out.append((prefix, int(node)))
    elif isinstance(node, (int, float)):
        out.append((prefix, node))


def read_env(path):
    env = {}
    if os.path.exists(path):
        for line in open(path):
            if ": " in line:
                k, v = line.rstrip("\n").split(": ", 1)
                env[k.strip()] = v.strip()
    return env


def entry_of(metric):
    if metric.startswith("s7.ekf_slam"):
        return "rust_ekf_slam"
    if metric.startswith("s7.rbpf"):
        return "rust_rbpf"
    if metric.startswith("s7.dead_reckoning"):
        return "dead_reckoning_synthetic"
    return "rust_slam"


def main(argv):
    root = argv[0] if argv else "evidence/phase-5-slam"
    out_path = argv[1] if len(argv) > 1 else "benchmarks/phase-5-slam.csv"
    rows = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        mp = os.path.join(d, "slam-metrics.yaml")
        if not os.path.exists(mp):
            continue
        m = yaml.safe_load(open(mp))
        env = read_env(os.path.join(d, "environment.txt"))
        run = m["run"]
        dirty = env.get("source_git_dirty_paths", "").strip()
        commit = env.get("source_git_commit", "unknown") + ("-dirty" if dirty else "")
        config = (f"{name}|{run['route']}|role={run['role']}|ground_truth={run['ground_truth']}"
                  f"|info_inflation={run.get('info_inflation', 'not recorded')}"
                  f"|injected_yaw_bias_rad_per_keyframe={run['injected_yaw_bias_rad_per_keyframe']}")
        synthetic = run["data_source"] == "synthetic"
        flat = []
        flatten("", {k: v for k, v in m.items() if k not in ("schema", "run")}, flat)
        for metric, value in flat:
            rows.append({
                "phase": 5, "entry": entry_of(metric), "language": "rust", "seed": run["seed"],
                "config": config, "metric": metric, "value": value, "unit": unit_of(metric),
                "host": env.get("host", ""), "cpu": env.get("cpu", ""), "kernel": env.get("kernel", ""),
                "ros_distro": "none", "webots_version": "none (synthetic)" if synthetic else "recorded by Phase 4",
                "rustc_version": env.get("rustc", ""), "rmw": "none",
                "headless": "not applicable (no simulator)" if synthetic else "recording",
                "timestamp": env.get("date", ""), "commit": commit,
            })
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} rows from {len({r['config'] for r in rows})} runs -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
