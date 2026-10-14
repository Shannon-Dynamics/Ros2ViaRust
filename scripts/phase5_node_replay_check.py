#!/usr/bin/env python3
"""Check one replay of the Phase 5 SLAM nodes against what the library does on the same stream.

    python3 -I scripts/phase5_node_replay_check.py --scans scans.csv --node DIR --reference DIR \
        [--metrics slam-metrics.yaml] [--out check.yaml]

`--node` holds what `scan_matcher_node` and `pose_graph_node` wrote (registrations.csv, keyframes.csv);
`--reference` holds what `slam_replay_reference` (the same library calls, no ROS) wrote on the same
recorded file. The checks, each printed as PASS or FAIL, exit code 1 if any fails:

  R1  the node published one registration for every scan the reference registered, with the same stamps
  R2  the registrations are the reference's, row for row (status, threshold, iterations, correspondences,
      rmse and the increment): the node adds nothing to and takes nothing from the front end
  K1  the pose graph made the same keyframes, at the same stamps
  K2  the keyframe poses agree with the reference to 1e-9 (m, rad): the only difference between the
      two paths is the information matrix's round trip through a message's covariance
  G   (with ground truth) the keyframe trajectory is graded against the scans file's ground-truth
      poses, which only this script reads: ATE RMS, maximum and final error, printed as measured
  E1  (with `--metrics` and `--f64-reference`) the in-process reference run WITHOUT the float32 rounding of
      the scan metadata reproduces the graded synthetic evidence run's front-end-alone trajectory error
      to 1e-9 m: the reference is the harness' pipeline
  I   (informational, never counted) how far the node's trajectory error is from the evidence run's: the
      only difference between the two is that a `sensor_msgs/LaserScan` carries `angle_min` and
      `angle_increment` as float32

The ground truth is in the scans file (columns 2 to 4) and is never on the ROS graph: the replayer ignores
those columns. Nothing here is a tolerance on how GOOD the front end is; that is the verdict script's job.
"""
import csv
import math
import sys

import yaml

POSE_TOL = 1e-9
EVIDENCE_TOL = 1e-9


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def read_gt(path):
    gt = {}
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        f = line.split(",", 8)
        gt[round(float(f[0]) * 1e9)] = (float(f[1]), float(f[2]), float(f[3]))
    return gt


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def between(a, b):
    """a^-1 * b for planar poses (x, y, theta)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


def main(argv):
    arg = lambda n: argv[argv.index(n) + 1] if n in argv else None
    scans, node, ref = arg("--scans"), arg("--node"), arg("--reference")
    if not (scans and node and ref):
        print(__doc__)
        return 2
    results, facts = [], {}

    def check(name, ok, detail=""):
        results.append((name, bool(ok), detail))
        print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))

    nreg, rreg = rows(f"{node}/registrations.csv"), rows(f"{ref}/registrations.csv")
    nkf, rkf = rows(f"{node}/keyframes.csv"), rows(f"{ref}/keyframes.csv")
    facts.update(registrations_node=len(nreg), registrations_reference=len(rreg),
                 keyframes_node=len(nkf), keyframes_reference=len(rkf))
    check("R1 one registration per registered scan, same stamps",
          [r["stamp_ns"] for r in nreg] == [r["stamp_ns"] for r in rreg] and len(nreg) > 0,
          f"{len(nreg)} node, {len(rreg)} reference")
    cols = ["status", "retried", "threshold_m", "iterations", "correspondences", "rmse_m", "degenerate", "dx", "dy", "dtheta"]
    first_bad = next((i for i, (a, b) in enumerate(zip(nreg, rreg)) if any(a[c] != b[c] for c in cols)), None)
    check("R2 registrations equal the reference, row for row", first_bad is None and len(nreg) == len(rreg),
          "identical" if first_bad is None else f"first difference at row {first_bad}: {dict((c, (nreg[first_bad][c], rreg[first_bad][c])) for c in cols if nreg[first_bad][c] != rreg[first_bad][c])}")
    statuses = {}
    for r in nreg:
        statuses[r["status"]] = statuses.get(r["status"], 0) + 1
    facts["status_counts"] = statuses
    lat = sorted(float(r["latency_us"]) for r in nreg if r.get("latency_us"))
    if lat:
        q = lambda p: lat[min(len(lat) - 1, int(p * len(lat)))]
        facts["registration_latency_us_p50_p95_p99_max"] = [q(0.5), q(0.95), q(0.99), lat[-1]]
    check("K1 same keyframes at the same stamps",
          [r["stamp_ns"] for r in nkf] == [r["stamp_ns"] for r in rkf] and len(nkf) > 0,
          f"{len(nkf)} node, {len(rkf)} reference")
    worst = 0.0
    if len(nkf) == len(rkf):
        for a, b in zip(nkf, rkf):
            d = between((float(a["x"]), float(a["y"]), float(a["theta"])), (float(b["x"]), float(b["y"]), float(b["theta"])))
            worst = max(worst, math.hypot(d[0], d[1]), abs(d[2]))
    facts["keyframe_pose_max_difference"] = worst
    check("K2 keyframe poses agree with the reference to 1e-9", len(nkf) == len(rkf) and worst <= POSE_TOL, f"max difference {worst:.3e}")

    gt = read_gt(scans)
    stamps = [int(r["stamp_ns"]) for r in nkf]
    if stamps and all(s in gt for s in stamps):
        origin = gt[stamps[0]]
        errs, last = [], None
        for r, s in zip(nkf, stamps):
            truth = between(origin, gt[s])
            est = (float(r["x"]), float(r["y"]), float(r["theta"]))
            errs.append(math.hypot(truth[0] - est[0], truth[1] - est[1]))
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        facts.update(ate_rms_m=rms, ate_max_m=max(errs), final_error_m=errs[-1])
        check("G keyframe trajectory against ground truth (measured, not graded)", True,
              f"ATE RMS {rms * 1000:.2f} mm, max {max(errs) * 1000:.2f} mm, final {errs[-1] * 1000:.2f} mm over {len(errs)} keyframes")
        metrics = arg("--metrics")
        if metrics:
            s4 = yaml.safe_load(open(metrics))["s4"]
            m, kf_made = s4["odometry"], s4["keyframes_made"]
            f64dir = arg("--f64-reference")
            if f64dir:
                rkf64 = rows(f"{f64dir}/keyframes.csv")
                e64 = []
                st64 = [int(r["stamp_ns"]) for r in rkf64]
                for r, s_ in zip(rkf64, st64):
                    t = between(gt[st64[0]], gt[s_])
                    e64.append(math.hypot(t[0] - float(r["x"]), t[1] - float(r["y"])))
                rms64 = math.sqrt(sum(e * e for e in e64) / len(e64))
                ok = len(rkf64) == kf_made and abs(rms64 - m["ate_rms_m"]) <= EVIDENCE_TOL \
                    and abs(max(e64) - m["ate_max_m"]) <= EVIDENCE_TOL and abs(e64[-1] - m["final_error_m"]) <= EVIDENCE_TOL
                check("E1 the float64 reference reproduces the graded evidence run's front end alone", ok,
                      f"reference ATE RMS {rms64 * 1000:.4f} mm and {len(rkf64)} keyframes, evidence {m['ate_rms_m'] * 1000:.4f} mm and {kf_made}")
            print(f"INFO  node vs evidence front end alone: ATE RMS {rms * 1000:.2f} vs {m['ate_rms_m'] * 1000:.2f} mm, "
                  f"max {max(errs) * 1000:.2f} vs {m['ate_max_m'] * 1000:.2f} mm, keyframes {len(nkf)} vs {kf_made} "
                  "(float32 scan metadata of the message: a keyframe threshold can flip)")
            facts.update(evidence_ate_rms_m=m["ate_rms_m"], evidence_ate_max_m=m["ate_max_m"])
    else:
        check("G ground truth available for every keyframe stamp", False, "stamps missing from the scans file")

    out = arg("--out")
    if out:
        with open(out, "w") as f:
            yaml.safe_dump({"facts": facts, "checks": [{"name": n, "pass": ok, "detail": d} for n, ok, d in results]}, f, sort_keys=False)
    failed = [n for n, ok, _ in results if not ok]
    print(f"{len(results) - len(failed)} of {len(results)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
