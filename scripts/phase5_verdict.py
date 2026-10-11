#!/usr/bin/env python3
"""Turn a Phase 5 SLAM metrics file into a verdict file. Nothing in a verdict is typed by hand.

    python3 -I scripts/phase5_verdict.py <run directory> [<run directory> ...]
    python3 -I scripts/phase5_verdict.py --print-tolerances

Reads `slam-metrics.yaml` in every run directory (written by the run harness, for synthetic data
`cargo run --release -p robotics_slam --example slam_synthetic_run -- --out DIR`), writes
`phase5-verdict.yaml` next to it and prints it. Several directories are graded one by one and then
checked against each other (the three evaluation runs of claim S4).

EVERY TOLERANCE IS IN THIS FILE, AND THIS FILE WAS WRITTEN BEFORE ANY EVIDENCE RUN EXISTED. That is
the point of it: no threshold below was chosen after looking at a result of the harness it grades.
The git history of this file is the proof: a change to a number after an evidence run exists is a
visible edit, and the run it would have favoured is the one to distrust. Each number has its
derivation next to it, from the lidar's noise, the geometry of the room, or the design of the
claim; the integration tests of `crates/robotics_slam/tests/` printed model-level numbers while the
code was written, and those are a different data source and a different set of thresholds (the
tests' own), so nothing here was fitted to a run of the harness.

What is graded, from `docs/phase-5-slam/problem.md` section 4:

  S1a  scan-matching accuracy against the ground-truth relative pose: a distribution over the
       registrations of a run. Graded against tolerances that depend on how good the ground truth
       is (exact in Rusty; gated, with a motion bound, on Webots).
  S1b  ICP's basin of convergence: a rate on a seeded trial set, MODEL LEVEL ONLY. Graded for the
       offsets the front end actually has to absorb (Wilson lower bound), reported beyond them.
  S2   the honesty of the information matrix: REPORTED, never pass or fail.
  S4   trajectory accuracy of the graph against ground truth.
  S5   loop closure: pass or fail on the SAFETY count (accepted constraints that are wrong,
       pre-registered to be zero); everything else is reported.
  S6   determinism (bit-identical replay) and latency.

NOT graded here: S3 (the optimizer: `cargo test -p robotics_slam`), S7 (the back ends: informative,
no metrics file in this pass), and S8, which is not claimable: see problem.md section 4 and 5.

A run that is not valid gets no verdict at all, and the reason: a number computed on a broken run
is not a result.

THE METRICS FILE (schema phase5-slam-metrics/1). Every key below is required unless marked
optional, and ANY other key is refused, so a run cannot be graded on a file the harness did not
produce.

  schema: phase5-slam-metrics/1
  run: {data_source: synthetic|webots_recorded|webots_live, ground_truth: exact|webots_gated,
        role: development|evaluation, seed, route, injected_yaw_bias_rad_per_keyframe,
        robotics_mapping_dependency: false, build_profile: release|debug}
  s1a: {registrations_made, registrations_graded,
        translation_error_m: STATS, rotation_error_rad: STATS}
  s2:  {dof, samples, mean_nees}
  s1b: {success_error_m, success_error_rad,
        ranges: [{offset_m, offset_rad, trials, successes}, ...]}
  s4:  {keyframes_made, keyframes_graded, path_length_m, odometry: TRAJ, graph: TRAJ}
  s5:  {candidates, accepted_correct, accepted_wrong, true_loops_proposed, true_loops_accepted,
        wrong_constraint_error_m, wrong_constraint_error_rad, rejected_by_reason: {no_overlap,
        not_converged, poor_fit, degenerate, gate_exceeded, ambiguous}}
  s6:  {replay_identical, scan_period_s, registration_latency_s: STATS, optimization_latency_s: STATS}

  STATS = {median, p95, p99, max, count}
  TRAJ  = {ate_rms_m, ate_max_m, final_error_m, drift_m_per_m, heading_drift_rad_per_m}
"""
import hashlib
import math
import os
import sys

import yaml

# ----------------------------------------------------------------------------------------
# Tolerances.
# ----------------------------------------------------------------------------------------

SCHEMA = "phase5-slam-metrics/1"

# -- validity ----------------------------------------------------------------------------

# A run that covers less than this is not a trajectory: the route of the synthetic run is two laps
# of a 1.4 m circle (17.6 m) and route_long.yaml is 53 m.
MIN_PATH_M = 5.0

# Fewest registrations or keyframes for a distribution or a trajectory statistic to mean anything.
MIN_GRADED_COUNT = 50
MIN_GRADED_KEYFRAMES = 10

# -- S1a: registration error against the ground-truth relative pose ------------------------
#
# With EXACT ground truth the error is the matcher's. A registration uses a couple of hundred
# downsampled points (5 cm voxels) on walls with 15 mm range noise (0.0043 of 3.5 m, the Webots
# model's figure, used by the synthetic lidar too). One wall point fixes position across the wall
# to about 15 mm; averaging over a couple of hundred makes millimetres. The tolerances below are a
# deliberately generous multiple of that (the point spacing, 2 cm, for the 95th percentile and
# 5 cm for the worst case), because S1a is graded on every registration of a run including the
# ones at the room's corners, where fewer walls are in view; rotation is the same arithmetic over
# a lever arm of about 2 m: 15 mm / 2 m = 7.5 mrad per point, averaged down, bounded at 10 mrad
# (p95) and 30 mrad (max).
S1A_EXACT = {
    "trans_p95_m": 0.02,
    "trans_max_m": 0.05,
    "rot_p95_rad": 0.01,
    "rot_max_rad": 0.03,
}

# With WEBOTS ground truth the error includes the ground truth's own. The scan is stamped up to a
# scan period after it was measured (docs/phase-4-mapping/implementation.md 5.2 item 2), so a
# registration is graded only when both scans pass the mapper's motion gate (the sensor moved at
# most 5 cm and 0.015 rad across the window). That bounds each scan's ground-truth error by the
# gate, and a relative pose by twice it: 0.10 m and 0.03 rad worst case. The maximum is held to
# that bound; the 95th percentile to half of it, since both scans are rarely at the gate's edge.
S1A_WEBOTS_GATED = {
    "trans_p95_m": 0.05,
    "trans_max_m": 0.10,
    "rot_p95_rad": 0.015,
    "rot_max_rad": 0.03,
}

# Registrations that did not settle are not graded (they are counted: graded / made). A run in
# which more than this fraction of the registrations it made failed to settle was not tracking.
S1A_MIN_GRADED_FRACTION = 0.95

# -- S1b: basin of convergence --------------------------------------------------------------
#
# Graded for the offsets the front end has to absorb: its prediction error per scan is at most the
# robot's motion in one scan period (0.18 m/s * 0.2 s = 3.6 cm; 0.8 rad/s * 0.2 s = 0.16 rad when
# turning, which a constant-velocity prediction mostly removes). Offsets up to 0.10 m and
# 0.10 rad are graded; larger ones are reported, because the claim is "ICP recovers from what the
# front end gives it", not "ICP has a large basin". The bound is on the lower end of the 95 per
# cent Wilson interval: 96 successes in 100 trials give 0.90.
S1B_GRADED_MAX_OFFSET_M = 0.10
S1B_GRADED_MAX_OFFSET_RAD = 0.10
S1B_WILSON_LOWER_MIN = 0.90
S1B_MIN_TRIALS = 100
WILSON_Z = 1.959964  # two-sided 95 per cent

# -- S2: REPORTED, never graded ---------------------------------------------------------------
S2_ALPHA = 0.05  # two-sided interval on the mean NEES; the samples are correlated, so it is optimistic

# -- S4: trajectory ------------------------------------------------------------------------------
#
# The graph's absolute trajectory error. A cell is 5 cm and so is the point spacing; a trajectory
# whose RMS error exceeds a cell is a map that is blurred at the resolution the rest of the stack
# plans in. Webots ground truth is looser by the same factor as S1a.
S4_GRAPH_ATE_RMS_MAX_M = {"exact": 0.05, "webots_gated": 0.10}
S4_GRAPH_FINAL_ERROR_MAX_M = {"exact": 0.10, "webots_gated": 0.20}

# The front end's own drift, graded only when NO error was injected: 1 per cent of path is the
# usual order for scan-matching odometry (chosen; no number of this repository stands behind it
# yet, and that is why it is a tolerance to be revised on the development run, visibly, if it is
# wrong).
S4_ODOMETRY_DRIFT_MAX_M_PER_M = 0.01

# The graph with verified loop closures must not be worse than the chain it started from: correct
# constraints cannot raise the expected error. 5 per cent is the slack for a single run.
S4_GRAPH_NOT_WORSE_FACTOR = 1.05

# Group check across the evaluation runs: the graph ATE of runs that are supposed to agree. Several
# runs that differ by more than a cell are a finding to investigate, not a reason to add runs until
# the average improves (problem.md section 4, the same rule as Phase 4).
S4_SPREAD_MAX_M = 0.05

# -- S5: loop closure ----------------------------------------------------------------------------
#
# The only criterion that FAILS a run is the safety count: an accepted constraint that is wrong
# (its error against the ground-truth relative pose exceeds wrong_constraint_error_*; the harness
# writes the limits it used and this script refuses any other than these). A false loop closure
# bends the whole map; a missed one costs drift. Recall is therefore reported and never required.
S5_WRONG_CONSTRAINT_ERROR_M = 0.05  # the cell size
S5_WRONG_CONSTRAINT_ERROR_RAD = 0.03
# A verdict on the safety count from fewer candidates than this is not a statement about it.
S5_MIN_CANDIDATES = 20

# -- S6: determinism and latency -------------------------------------------------------------------
# One registration must leave at least half of a scan period to everything else (the Phase 4 rule
# for M4). One optimization runs once per keyframe, and keyframes are at least 0.5 m apart at the
# route's 0.18 m/s: about 2.8 s; the bound is a third of that.
S6_REGISTRATION_P99_FRACTION_OF_SCAN_PERIOD = 0.5
S6_OPTIMIZATION_P99_MAX_S = 1.0

# Comparisons are made to this much slack (a quotient in floating point can land a few parts in
# 10^15 on the wrong side of a limit it is meant to meet; see phase4_verdict.py).
EPSILON = 1e-9

# ----------------------------------------------------------------------------------------
# Statistics
# ----------------------------------------------------------------------------------------


def wilson(successes, n, z=WILSON_Z):
    """Wilson score interval for a binomial proportion."""
    if n <= 0:
        return (float("nan"), float("nan"))
    p = successes / n
    d = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / d
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, centre - half), min(1.0, centre + half))


def _gamma_p(a, x):
    """Regularized lower incomplete gamma function P(a, x)."""
    if x <= 0:
        return 0.0
    if x < a + 1:
        term = total = 1.0 / a
        ap = a
        for _ in range(1000):
            ap += 1
            term *= x / ap
            total += term
            if abs(term) < abs(total) * 1e-15:
                break
        return total * math.exp(-x + a * math.log(x) - math.lgamma(a))
    # continued fraction for Q(a, x)
    tiny = 1e-300
    b = x + 1 - a
    c = 1 / tiny
    d = 1 / b
    h = d
    for i in range(1, 1000):
        an = -i * (i - a)
        b += 2
        d = an * d + b
        d = tiny if abs(d) < tiny else d
        c = b + an / c
        c = tiny if abs(c) < tiny else c
        d = 1 / d
        delta = d * c
        h *= delta
        if abs(delta - 1) < 1e-15:
            break
    return 1 - math.exp(-x + a * math.log(x) - math.lgamma(a)) * h


def chi2_quantile(dof, p):
    """Quantile of the chi-squared distribution, by bisection on the regularized gamma."""
    lo, hi = 0.0, max(10.0, 10.0 * dof)
    while _gamma_p(dof / 2, hi / 2) < p:
        hi *= 2
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if _gamma_p(dof / 2, mid / 2) < p:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


# ----------------------------------------------------------------------------------------
# Schema
# ----------------------------------------------------------------------------------------

STATS = {"median", "p95", "p99", "max", "count"}
TRAJ = {"ate_rms_m", "ate_max_m", "final_error_m", "drift_m_per_m", "heading_drift_rad_per_m"}
REASONS = {"no_overlap", "not_converged", "poor_fit", "degenerate", "gate_exceeded", "ambiguous"}
SCHEMA_KEYS = {
    "schema": None,
    "run": {
        "data_source", "ground_truth", "role", "seed", "route",
        "injected_yaw_bias_rad_per_keyframe", "robotics_mapping_dependency", "build_profile",
    },
    "s1a": {"registrations_made", "registrations_graded", "translation_error_m", "rotation_error_rad"},
    "s2": {"dof", "samples", "mean_nees"},
    "s1b": {"success_error_m", "success_error_rad", "ranges"},
    "s4": {"keyframes_made", "keyframes_graded", "path_length_m", "odometry", "graph"},
    "s5": {
        "candidates", "accepted_correct", "accepted_wrong", "true_loops_proposed",
        "true_loops_accepted", "wrong_constraint_error_m", "wrong_constraint_error_rad",
        "rejected_by_reason",
    },
    "s6": {
        "replay_identical", "scan_period_s", "registration_latency_s", "optimization_latency_s",
    },
}
NESTED = {
    ("s1a", "translation_error_m"): STATS,
    ("s1a", "rotation_error_rad"): STATS,
    ("s4", "odometry"): TRAJ,
    ("s4", "graph"): TRAJ,
    ("s5", "rejected_by_reason"): REASONS,
    ("s6", "registration_latency_s"): STATS,
    ("s6", "optimization_latency_s"): STATS,
}
RANGE_KEYS = {"offset_m", "offset_rad", "trials", "successes"}


def schema_problems(m):
    """Every way `m` differs from the schema: missing keys, unknown keys, wrong types."""
    out = []
    if not isinstance(m, dict):
        return ["the metrics file is not a mapping"]
    for k in m:
        if k not in SCHEMA_KEYS:
            out.append(f"unknown top-level key: {k}")
    for sec, keys in SCHEMA_KEYS.items():
        if sec not in m:
            out.append(f"missing section: {sec}")
            continue
        if keys is None:
            continue
        if not isinstance(m[sec], dict):
            out.append(f"{sec} is not a mapping")
            continue
        for k in m[sec]:
            if k not in keys:
                out.append(f"unknown key: {sec}.{k}")
        for k in keys:
            if k not in m[sec]:
                out.append(f"missing key: {sec}.{k}")
    for (sec, k), keys in NESTED.items():
        v = m.get(sec, {}).get(k) if isinstance(m.get(sec), dict) else None
        if v is None:
            continue
        if not isinstance(v, dict):
            out.append(f"{sec}.{k} is not a mapping")
            continue
        for kk in v:
            if kk not in keys:
                out.append(f"unknown key: {sec}.{k}.{kk}")
        for kk in keys:
            if kk not in v:
                out.append(f"missing key: {sec}.{k}.{kk}")
    # Every leaf that is not a label must be a number (or, for one flag, a bool). PyYAML reads
    # `1.7e1` (no dot, no sign) as a string, and a grader that compares a string with a limit
    # must refuse, not crash.
    labels = {("run", k) for k in ("data_source", "ground_truth", "role", "route", "build_profile")}
    labels |= {("run", "robotics_mapping_dependency"), ("s6", "replay_identical")}

    def leaves(path, v):
        if isinstance(v, dict):
            for k, x in v.items():
                yield from leaves(path + (k,), x)
        elif isinstance(v, list):
            for i, x in enumerate(v):
                yield from leaves(path + (i,), x)
        else:
            yield path, v

    for path, v in leaves((), {k: x for k, x in m.items() if k != "schema"}):
        if path[:2] in labels:
            continue
        if isinstance(v, bool) or not isinstance(v, (int, float)) or (isinstance(v, float) and v != v):
            out.append(f"{'.'.join(map(str, path))} is not a number: {v!r}")
    ranges = m.get("s1b", {}).get("ranges") if isinstance(m.get("s1b"), dict) else None
    if ranges is not None:
        if not isinstance(ranges, list) or not ranges:
            out.append("s1b.ranges is not a non-empty list")
        else:
            for i, r in enumerate(ranges):
                if not isinstance(r, dict) or set(r) != RANGE_KEYS:
                    out.append(f"s1b.ranges[{i}] must have exactly the keys {sorted(RANGE_KEYS)}")
    return out


# ----------------------------------------------------------------------------------------
# Grading
# ----------------------------------------------------------------------------------------


def fmt(v):
    if isinstance(v, float):
        return repr(v)
    return v


class Check:
    def __init__(self, name, value, limit, op):
        self.name, self.value, self.limit, self.op = name, value, limit, op
        if value is None or (isinstance(value, float) and math.isnan(value)):
            self.ok = False
        elif op == "<=":
            self.ok = value <= limit + EPSILON
        elif op == ">=":
            self.ok = value >= limit - EPSILON
        elif op == "==":
            self.ok = value == limit
        else:
            raise ValueError(op)

    def line(self):
        return (f"      {self.name}: {{value: {fmt(self.value)!r}, limit: {fmt(self.limit)!r}, "
                f"op: '{self.op}', pass: {str(self.ok).lower()}}}")


def claim(status, checks=None, notes=None):
    return {"status": status, "checks": checks or [], "notes": notes or []}


def validity(m):
    """Reasons the run cannot be graded at all (an empty list means it can)."""
    r = list(schema_problems(m))
    if r:
        return r
    run = m["run"]
    if m["schema"] != SCHEMA:
        r.append(f"schema is {m['schema']!r}, this script grades {SCHEMA!r}")
    if run["data_source"] not in ("synthetic", "webots_recorded", "webots_live"):
        r.append(f"unknown data_source {run['data_source']!r}")
    if run["ground_truth"] not in ("exact", "webots_gated"):
        r.append(f"unknown ground_truth {run['ground_truth']!r}")
    elif (run["data_source"] == "synthetic") != (run["ground_truth"] == "exact"):
        r.append("ground_truth does not match data_source (synthetic is exact, Webots is gated)")
    if run["role"] not in ("development", "evaluation"):
        r.append(f"unknown role {run['role']!r}")
    if run["robotics_mapping_dependency"] is not False:
        r.append("the pipeline that produced this run depended on robotics_mapping: no S claim may be "
                 "graded on it while Phase 4's M1 is open (problem.md section 6)")
    if m["s4"]["path_length_m"] < MIN_PATH_M:
        r.append(f"the robot travelled {m['s4']['path_length_m']} m, needs {MIN_PATH_M} m")
    s5 = m["s5"]
    n_rej = sum(s5["rejected_by_reason"].values())
    if s5["accepted_correct"] + s5["accepted_wrong"] + n_rej != s5["candidates"]:
        r.append("s5 counts do not add up: accepted_correct + accepted_wrong + rejected != candidates")
    if s5["wrong_constraint_error_m"] != S5_WRONG_CONSTRAINT_ERROR_M or \
            s5["wrong_constraint_error_rad"] != S5_WRONG_CONSTRAINT_ERROR_RAD:
        r.append("the harness labelled wrong constraints with limits other than this script's")
    if m["s1a"]["registrations_graded"] > m["s1a"]["registrations_made"]:
        r.append("s1a graded more registrations than were made")
    for rg in m["s1b"]["ranges"]:
        if rg["successes"] > rg["trials"]:
            r.append("s1b has more successes than trials")
    return r


def grade_s1a(m):
    gt = m["run"]["ground_truth"]
    tol = S1A_EXACT if gt == "exact" else S1A_WEBOTS_GATED
    a = m["s1a"]
    made, graded = a["registrations_made"], a["registrations_graded"]
    frac = graded / made if made else 0.0
    if graded < MIN_GRADED_COUNT:
        return claim("NOT_GRADED", notes=[f"only {graded} registrations graded, needs {MIN_GRADED_COUNT}"])
    t, r = a["translation_error_m"], a["rotation_error_rad"]
    checks = [
        Check("graded_fraction", frac, S1A_MIN_GRADED_FRACTION, ">="),
        Check("translation_p95_m", t["p95"], tol["trans_p95_m"], "<="),
        Check("translation_max_m", t["max"], tol["trans_max_m"], "<="),
        Check("rotation_p95_rad", r["p95"], tol["rot_p95_rad"], "<="),
        Check("rotation_max_rad", r["max"], tol["rot_max_rad"], "<="),
    ]
    notes = [f"ground truth: {gt}; graded {graded} of {made} registrations made",
             f"translation median {t['median']!r} m, rotation median {r['median']!r} rad"]
    if gt == "webots_gated":
        notes.append("graded only where both scans pass the motion gate: a statement about that subset")
    return claim("PASS" if all(c.ok for c in checks) else "FAIL", checks, notes)


def grade_s1b(m):
    if m["run"]["data_source"] != "synthetic":
        return claim("NOT_GRADED", notes=["S1b is a model-level trial set; not graded on Webots data"])
    checks, notes = [], []
    for rg in m["s1b"]["ranges"]:
        lo, hi = wilson(rg["successes"], rg["trials"])
        label = f"offset {rg['offset_m']} m / {rg['offset_rad']} rad"
        graded = (rg["offset_m"] <= S1B_GRADED_MAX_OFFSET_M + EPSILON
                  and rg["offset_rad"] <= S1B_GRADED_MAX_OFFSET_RAD + EPSILON)
        text = f"{label}: {rg['successes']}/{rg['trials']}, Wilson 95% [{lo:.4f}, {hi:.4f}]"
        if graded:
            if rg["trials"] < S1B_MIN_TRIALS:
                return claim("NOT_GRADED", notes=[f"{label}: {rg['trials']} trials, needs {S1B_MIN_TRIALS}"])
            checks.append(Check(f"wilson_lower[{rg['offset_m']}m,{rg['offset_rad']}rad]", lo, S1B_WILSON_LOWER_MIN, ">="))
            notes.append(text + " (graded)")
        else:
            notes.append(text + " (reported, not graded)")
    if not checks:
        return claim("NOT_GRADED", notes=["no trial range within the graded offsets"] + notes)
    notes.append("model level: not comparable with a Webots number, and not extrapolated to one")
    return claim("PASS" if all(c.ok for c in checks) else "FAIL", checks, notes)


def grade_s2(m):
    s = m["s2"]
    n, dof, mean = s["samples"], s["dof"], s["mean_nees"]
    if n < MIN_GRADED_COUNT:
        return claim("NOT_GRADED", notes=[f"only {n} samples"])
    lo = chi2_quantile(dof * n, S2_ALPHA / 2) / n
    hi = chi2_quantile(dof * n, 1 - S2_ALPHA / 2) / n
    label = "OVERCONFIDENT" if mean > hi else "CONSERVATIVE" if mean < lo else "CONSISTENT"
    return claim("REPORTED", notes=[
        f"{label}: mean NEES {mean!r} against the chi-squared({dof}) mean interval [{lo:.4f}, {hi:.4f}] "
        f"for {n} samples",
        "never pass or fail: the information matrix is known to be overconfident (the beams are not "
        "independent), and info_inflation is set from the development run and frozen",
        "the interval treats the samples as independent; consecutive scans are not, so it is optimistic",
    ])


def grade_s4(m):
    run, s4, s5 = m["run"], m["s4"], m["s5"]
    gt = run["ground_truth"]
    if s4["keyframes_graded"] < MIN_GRADED_KEYFRAMES:
        return claim("NOT_GRADED", notes=[f"only {s4['keyframes_graded']} keyframes graded"])
    odo, graph = s4["odometry"], s4["graph"]
    injected = run["injected_yaw_bias_rad_per_keyframe"] != 0
    checks = [
        Check("graph_ate_rms_m", graph["ate_rms_m"], S4_GRAPH_ATE_RMS_MAX_M[gt], "<="),
        Check("graph_final_error_m", graph["final_error_m"], S4_GRAPH_FINAL_ERROR_MAX_M[gt], "<="),
        Check("graph_ate_rms_over_odometry", graph["ate_rms_m"] / max(odo["ate_rms_m"], 1e-12),
              S4_GRAPH_NOT_WORSE_FACTOR, "<="),
    ]
    notes = [f"graded {s4['keyframes_graded']} of {s4['keyframes_made']} keyframes, path {s4['path_length_m']!r} m",
             f"odometry ATE RMS {odo['ate_rms_m']!r} m, drift {odo['drift_m_per_m']!r} m/m"]
    if injected:
        notes.append("an error was INJECTED into the odometry edges "
                     f"({run['injected_yaw_bias_rad_per_keyframe']!r} rad per keyframe): the odometry "
                     "drift is not the scan matcher's and is not graded; this run is model-level "
                     "evidence for the loop machinery only")
    else:
        checks.append(Check("odometry_drift_m_per_m", odo["drift_m_per_m"], S4_ODOMETRY_DRIFT_MAX_M_PER_M, "<="))
    accepted = s5["accepted_correct"] + s5["accepted_wrong"]
    if accepted == 0:
        notes.append("no loop closure was accepted: the graph is the odometry chain and the comparison "
                     "with it is vacuous (problem.md section 4, S4)")
    if s5["accepted_wrong"] > 0:
        notes.append("a wrong constraint was accepted (S5 fails): the graph's error is not attributable "
                     "to the optimizer")
    return claim("PASS" if all(c.ok for c in checks) else "FAIL", checks, notes)


def grade_s5(m):
    s5 = m["s5"]
    accepted = s5["accepted_correct"] + s5["accepted_wrong"]
    prec_lo, prec_hi = wilson(s5["accepted_correct"], accepted)
    notes = [
        f"candidates {s5['candidates']}, accepted {accepted} "
        f"(correct {s5['accepted_correct']}, wrong {s5['accepted_wrong']}), "
        f"rejected by reason {dict(s5['rejected_by_reason'])}",
        f"candidate-level precision {s5['accepted_correct']}/{accepted}, Wilson 95% "
        f"[{prec_lo:.4f}, {prec_hi:.4f}]" if accepted else "no candidate was accepted",
        f"true loops proposed {s5['true_loops_proposed']}, accepted {s5['true_loops_accepted']}: recall "
        "of what the DETECTOR PROPOSED (loops it never proposed are not counted); reported, never required",
    ]
    if s5["candidates"] < S5_MIN_CANDIDATES:
        return claim("NOT_GRADED", notes=notes + [f"{s5['candidates']} candidates, needs {S5_MIN_CANDIDATES}"])
    if accepted > 0 and s5["accepted_wrong"] == 0:
        notes.append(f"zero wrong constraints in {accepted} accepted: this supports a wrong-accept rate "
                     f"below about {3 / accepted:.4f} of accepted candidates at 95 per cent (the rule of "
                     "three), in this world and no other")
    notes.append("a PASS here is a statement about the candidates this run produced; the candidates "
                 "designed to be rejected are in the unit and integration tests, not in this file")
    checks = [Check("accepted_wrong", s5["accepted_wrong"], 0, "==")]
    return claim("PASS" if all(c.ok for c in checks) else "FAIL", checks, notes)


def grade_s6(m):
    s6 = m["s6"]
    reg, opt = s6["registration_latency_s"], s6["optimization_latency_s"]
    checks = [
        Check("replay_identical", bool(s6["replay_identical"]), True, "=="),
        Check("registration_p99_s", reg["p99"], S6_REGISTRATION_P99_FRACTION_OF_SCAN_PERIOD * s6["scan_period_s"], "<="),
    ]
    notes = [f"build profile {m['run']['build_profile']}; registrations {int(reg['count'])}, "
             f"median {reg['median']!r} s, p95 {reg['p95']!r} s",
             f"S1a translation p95 of the same run: {m['s1a']['translation_error_m']['p95']!r} m (accuracy next to speed)"]
    if opt["count"] and opt["count"] > 0:
        checks.append(Check("optimization_p99_s", opt["p99"], S6_OPTIMIZATION_P99_MAX_S, "<="))
        notes.append(f"optimizations {int(opt['count'])}, median {opt['median']!r} s")
    else:
        notes.append("no optimization ran (no loop was accepted): its latency is not graded")
    return claim("PASS" if all(c.ok for c in checks) else "FAIL", checks, notes)


def grade(run_dir):
    """Grade one run directory; return (verdict dict, text lines)."""
    name = os.path.basename(os.path.abspath(run_dir))
    path = os.path.join(run_dir, "slam-metrics.yaml")
    header = [
        "# Written by scripts/phase5_verdict.py from slam-metrics.yaml in this directory.",
        "# Every tolerance lives in that script, which was written before the first evidence run.",
        f"run: {name}",
        f"verdict_script_sha256: {sha256(__file__)}",
    ]
    if not os.path.exists(path):
        return {"valid": False}, header + ["valid: false", "invalid_reasons:", "  - slam-metrics.yaml is missing"]
    header.append(f"metrics_sha256: {sha256(path)}")
    with open(path) as f:
        try:
            m = yaml.safe_load(f)
        except yaml.YAMLError as e:
            return {"valid": False}, header + ["valid: false", "invalid_reasons:", f"  - unreadable: {e}"]
    reasons = validity(m)
    if reasons:
        return {"valid": False}, header + ["valid: false", "invalid_reasons:"] + [f"  - {r}" for r in reasons]
    run = m["run"]
    claims = {
        "S1a": grade_s1a(m), "S1b": grade_s1b(m), "S2": grade_s2(m), "S4": grade_s4(m),
        "S5": grade_s5(m), "S6": grade_s6(m),
    }
    claims["S3"] = claim("NOT_GRADED", notes=["tests: cargo test -p robotics_slam"])
    claims["S7"] = claim("NOT_GRADED", notes=["informative, non-blocking; no metrics file in this pass"])
    claims["S8"] = claim("NOT_CLAIMABLE", notes=[
        "blocked on Phase 4's M1, a measured S5 on the evaluation runs, and the submap-rendering node"])
    lines = header + [
        "valid: true",
        f"data_source: {run['data_source']}",
        f"ground_truth: {run['ground_truth']}",
        f"role: {run['role']}",
        f"seed: {run['seed']}",
        f"route: {run['route']}",
        f"injected_yaw_bias_rad_per_keyframe: {run['injected_yaw_bias_rad_per_keyframe']!r}",
        f"build_profile: {run['build_profile']}",
        "claims:",
    ]
    for cid in ("S1a", "S1b", "S2", "S3", "S4", "S5", "S6", "S7", "S8"):
        c = claims[cid]
        lines.append(f"  {cid}:")
        lines.append(f"    status: {c['status']}")
        if c["checks"]:
            lines.append("    checks:")
            lines += [chk.line() for chk in c["checks"]]
        if c["notes"]:
            lines.append("    notes:")
            lines += [f"      - {yaml.safe_dump(n, default_flow_style=True, width=10**6).strip().rstrip('.').strip()}"
                      for n in c["notes"]]
    return {"valid": True, "claims": claims, "metrics": m, "name": name}, lines


def group_check(results):
    graded = [r for r in results if r.get("valid") and r["claims"]["S4"]["status"] in ("PASS", "FAIL")
              and r["metrics"]["run"]["role"] == "evaluation"]
    if len(graded) < 2:
        return None
    ates = [r["metrics"]["s4"]["graph"]["ate_rms_m"] for r in graded]
    spread = max(ates) - min(ates)
    ok = spread <= S4_SPREAD_MAX_M + EPSILON
    return [
        "group:",
        f"  runs: {[r['name'] for r in graded]}",
        f"  s4_graph_ate_rms_m: {ates!r}",
        f"  s4_spread_m: {{value: {spread!r}, limit: {S4_SPREAD_MAX_M!r}, pass: {str(ok).lower()}}}",
        "  note: a spread above the limit is a finding to investigate, not a reason to add runs",
    ], ok


def print_tolerances():
    names = sorted(n for n in globals() if n.startswith(("S1A_", "S1B_", "S2_", "S4_", "S5_", "S6_", "MIN_")))
    for n in names:
        print(f"{n} = {globals()[n]!r}")


def main(argv):
    if "--print-tolerances" in argv:
        print_tolerances()
        return 0
    dirs = [a for a in argv if not a.startswith("--")]
    if not dirs:
        print(__doc__)
        return 2
    results, rc = [], 0
    for d in dirs:
        res, lines = grade(d)
        results.append(res)
        text = "\n".join(lines) + "\n"
        if res.get("valid"):
            if any(c["status"] == "FAIL" for c in res["claims"].values()):
                rc = max(rc, 1)
        else:
            rc = max(rc, 2)
        with open(os.path.join(d, "phase5-verdict.yaml"), "w") as f:
            f.write(text)
        print(text)
    g = group_check(results)
    if g:
        lines, ok = g
        print("\n".join(lines))
        if not ok:
            rc = max(rc, 1)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
