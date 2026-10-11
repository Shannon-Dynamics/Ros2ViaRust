#!/usr/bin/env python3
"""Self-test of scripts/phase5_verdict.py on fabricated metrics files.

    python3 -I scripts/phase5_verdict_selftest.py

The files are test FIXTURES written to a temporary directory and thrown away. They are not
evidence, they are never written under `evidence/`, and nothing here says anything about how the
SLAM code performs: the point is that the grader grades what it is given, refuses what it should
refuse, and fails each claim for the reason it should. Exits non-zero on the first failure.
"""
import copy
import importlib.util
import os
import sys
import tempfile

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("phase5_verdict", os.path.join(HERE, "phase5_verdict.py"))
pv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pv)


def stats(median, p95, p99, mx, count):
    return {"median": median, "p95": p95, "p99": p99, "max": mx, "count": count}


def nominal():
    return {
        "schema": "phase5-slam-metrics/1",
        "run": {
            "data_source": "synthetic", "ground_truth": "exact", "role": "development", "seed": 1,
            "route": "fixture", "injected_yaw_bias_rad_per_keyframe": 0.004,
            "robotics_mapping_dependency": False, "build_profile": "release",
        },
        "s1a": {
            "registrations_made": 290, "registrations_graded": 290,
            "translation_error_m": stats(0.002, 0.006, 0.01, 0.02, 290),
            "rotation_error_rad": stats(0.001, 0.003, 0.005, 0.01, 290),
        },
        "s2": {"dof": 3, "samples": 290, "mean_nees": 3.4},
        "s1b": {
            "success_error_m": 0.02, "success_error_rad": 0.01,
            "ranges": [
                {"offset_m": 0.05, "offset_rad": 0.05, "trials": 100, "successes": 100},
                {"offset_m": 0.10, "offset_rad": 0.10, "trials": 100, "successes": 98},
                {"offset_m": 0.30, "offset_rad": 0.25, "trials": 100, "successes": 40},
            ],
        },
        "s4": {
            "keyframes_made": 33, "keyframes_graded": 33, "path_length_m": 17.6,
            "odometry": {"ate_rms_m": 0.10, "ate_max_m": 0.2, "final_error_m": 0.2,
                         "drift_m_per_m": 0.011, "heading_drift_rad_per_m": 0.01},
            "graph": {"ate_rms_m": 0.01, "ate_max_m": 0.03, "final_error_m": 0.02,
                      "drift_m_per_m": 0.001, "heading_drift_rad_per_m": 0.001},
        },
        "s5": {
            "candidates": 56, "accepted_correct": 50, "accepted_wrong": 0, "true_loops_proposed": 40,
            "true_loops_accepted": 38, "wrong_constraint_error_m": 0.05, "wrong_constraint_error_rad": 0.03,
            "rejected_by_reason": {"no_overlap": 0, "not_converged": 1, "poor_fit": 2, "degenerate": 0,
                                   "gate_exceeded": 3, "ambiguous": 0},
        },
        "s6": {
            "replay_identical": True, "scan_period_s": 0.2,
            "registration_latency_s": stats(0.004, 0.008, 0.02, 0.05, 293),
            "optimization_latency_s": stats(0.05, 0.1, 0.2, 0.3, 20),
        },
    }


def grade(m):
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, "slam-metrics.yaml"), "w") as f:
            yaml.safe_dump(m, f)
        res, _ = pv.grade(d)
    return res


def expect(cond, what):
    if not cond:
        print(f"SELF-TEST FAILED: {what}")
        sys.exit(1)
    print(f"ok  {what}")


def status(res, cid):
    return res["claims"][cid]["status"]


def mutate(path, value):
    m = nominal()
    cur = m
    for k in path[:-1]:
        cur = cur[k]
    cur[path[-1]] = value
    return m


# --- the statistics the grader leans on ----------------------------------------------------
expect(abs(pv.chi2_quantile(3, 0.95) - 7.814727903251179) < 1e-6, "chi-squared(3) 95% quantile is 7.8147")
expect(abs(pv.chi2_quantile(1, 0.95) - 3.841458820694124) < 1e-6, "chi-squared(1) 95% quantile is 3.8415")
expect(abs(pv.chi2_quantile(10, 0.5) - 9.341817752645137) < 1e-6, "chi-squared(10) median is 9.3418")
lo, hi = pv.wilson(96, 100)
expect(abs(lo - 0.9016) < 5e-4, "Wilson lower bound of 96/100 is 0.9016")
expect(pv.wilson(95, 100)[0] < pv.S1B_WILSON_LOWER_MIN <= lo, "96 of 100 is the fewest that clears 0.90")

# --- the nominal fixture ---------------------------------------------------------------------
r = grade(nominal())
expect(r["valid"], "the nominal fixture is valid")
for cid in ("S1a", "S1b", "S4", "S5", "S6"):
    expect(status(r, cid) == "PASS", f"nominal fixture passes {cid}")
expect(status(r, "S2") == "REPORTED", "S2 is reported, never pass or fail")
expect(status(r, "S8") == "NOT_CLAIMABLE", "S8 is not claimable")
expect(any("OVERCONFIDENT" in n or "CONSISTENT" in n or "CONSERVATIVE" in n for n in r["claims"]["S2"]["notes"]),
       "S2 names a verdict word")

# --- each claim fails for its own reason ---------------------------------------------------------
expect(status(grade(mutate(("s1a", "translation_error_m", "p95"), 0.05)), "S1a") == "FAIL", "S1a fails on translation p95")
expect(status(grade(mutate(("s1a", "rotation_error_rad", "max"), 0.2)), "S1a") == "FAIL", "S1a fails on rotation max")
expect(status(grade(mutate(("s1a", "registrations_graded"), 200)), "S1a") == "FAIL", "S1a fails when too few registrations settled")
expect(status(grade(mutate(("s1a", "registrations_graded"), 10)), "S1a") == "NOT_GRADED", "S1a is not graded on too few registrations")
m = nominal(); m["s1b"]["ranges"][1]["successes"] = 90
expect(status(grade(m), "S1b") == "FAIL", "S1b fails when a graded range has a low Wilson bound")
m = nominal(); m["s1b"]["ranges"][2]["successes"] = 0
expect(status(grade(m), "S1b") == "PASS", "S1b does not grade offsets beyond the front end's need")
m = nominal(); m["s1b"]["ranges"][0]["trials"] = 50; m["s1b"]["ranges"][0]["successes"] = 50
expect(status(grade(m), "S1b") == "NOT_GRADED", "S1b is not graded on too few trials")
m = nominal(); m["run"]["data_source"] = "webots_recorded"; m["run"]["ground_truth"] = "webots_gated"
expect(status(grade(m), "S1b") == "NOT_GRADED", "S1b is never graded on Webots data")
expect(status(grade(mutate(("s4", "graph", "ate_rms_m"), 0.06)), "S4") == "FAIL", "S4 fails on graph ATE")
m = nominal(); m["s4"]["graph"]["ate_rms_m"] = 0.12
expect(status(grade(m), "S4") == "FAIL", "S4 fails when the graph is worse than the chain it started from")
m = nominal(); m["run"]["injected_yaw_bias_rad_per_keyframe"] = 0.0
expect(status(grade(m), "S4") == "FAIL", "S4 grades the natural drift when nothing was injected (0.011 > 0.01)")
m = nominal(); m["s5"]["accepted_wrong"] = 1; m["s5"]["accepted_correct"] = 49
expect(status(grade(m), "S5") == "FAIL", "S5 fails on a single wrong accepted constraint")
m = nominal(); m["s5"].update(candidates=10, accepted_correct=8, accepted_wrong=0)
m["s5"]["rejected_by_reason"] = {"no_overlap": 0, "not_converged": 0, "poor_fit": 1, "degenerate": 0, "gate_exceeded": 1, "ambiguous": 0}
expect(status(grade(m), "S5") == "NOT_GRADED", "S5 is not graded on too few candidates")
expect(status(grade(mutate(("s6", "replay_identical"), False)), "S6") == "FAIL", "S6 fails when the replay differs")
expect(status(grade(mutate(("s6", "registration_latency_s", "p99"), 0.2)), "S6") == "FAIL", "S6 fails on registration p99")
expect(status(grade(mutate(("s6", "optimization_latency_s", "p99"), 3.0)), "S6") == "FAIL", "S6 fails on optimization p99")

# --- what it refuses -------------------------------------------------------------------------------
m = nominal(); m["extra"] = 1
expect(not grade(m)["valid"], "an unknown top-level key is refused")
m = nominal(); m["s5"]["surprise"] = 1
expect(not grade(m)["valid"], "an unknown nested key is refused")
m = nominal(); del m["s6"]["replay_identical"]
expect(not grade(m)["valid"], "a missing key is refused")
m = nominal(); m["run"]["ground_truth"] = "webots_gated"
expect(not grade(m)["valid"], "synthetic data with gated ground truth is refused")
m = nominal(); m["run"]["robotics_mapping_dependency"] = True
expect(not grade(m)["valid"], "a run that depended on robotics_mapping is refused")
m = nominal(); m["s4"]["path_length_m"] = "1.7e1"
expect(not grade(m)["valid"], "a number that YAML read as a string is refused, not compared")
m = nominal(); m["s1a"]["translation_error_m"]["p95"] = float("nan")
expect(not grade(m)["valid"], "a NaN is refused")
m = nominal(); m["s4"]["path_length_m"] = 1.0
expect(not grade(m)["valid"], "a run that did not travel is refused")
m = nominal(); m["s5"]["candidates"] = 57
expect(not grade(m)["valid"], "s5 counts that do not add up are refused")
m = nominal(); m["s5"]["wrong_constraint_error_m"] = 0.2
expect(not grade(m)["valid"], "a harness that labelled wrong constraints with other limits is refused")
m = nominal(); m["schema"] = "phase5-slam-metrics/2"
expect(not grade(m)["valid"], "another schema version is refused")
with tempfile.TemporaryDirectory() as d:
    res, _ = pv.grade(d)
    expect(not res["valid"], "a directory with no metrics file is refused")

# --- the group check ------------------------------------------------------------------------------------
runs = []
for ate in (0.010, 0.012, 0.016):
    m = nominal(); m["run"]["role"] = "evaluation"; m["s4"]["graph"]["ate_rms_m"] = ate
    r = grade(m); r["name"] = f"r{ate}"; runs.append(r)
g = pv.group_check(runs)
expect(g is not None and g[1], "three evaluation runs within the spread pass the group check")
m = nominal(); m["run"]["role"] = "evaluation"; m["s4"]["graph"]["ate_rms_m"] = 0.07
r = grade(m); r["name"] = "far"; runs.append(r)
expect(not pv.group_check(runs)[1], "an evaluation run far from the others fails the group check")

print("all self-tests passed")
