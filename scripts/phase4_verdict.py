#!/usr/bin/env python3
"""Turn a Phase 4 mapping run into a verdict file. Nothing in a verdict is typed by hand.

    python3 -I scripts/phase4_verdict.py <run directory> [<run directory> ...]
    python3 -I scripts/phase4_verdict.py --print-tau-cells

Writes `phase4-verdict.yaml` into every run directory and prints it. Several directories are
graded one by one and then checked against each other (same mapper binary, same settings,
spread of the accuracy), which is what the three evaluation runs of claim M1 need.

EVERY TOLERANCE IS IN THIS FILE, AND THIS FILE WAS COMMITTED BEFORE THE FIRST EVIDENCE RUN.
That is the point of it: no threshold below was chosen after looking at a Webots result. The
git history of this file is the proof: a change to a number after an evidence run exists is a
visible edit, and the run it would have favoured is the one to distrust. Each number has a
derivation next to it, from the geometry of the grid and the sensor, from the design of the
metric, or from a synthetic run of the mapper on a copy of the room
(`crates/robotics_mapping/tests/room_mapping.rs`), which is a different data source from the
simulator runs graded here.

What is graded, from `docs/phase-4-mapping/problem.md` section 4:

  M1  the map is accurate against the analytic geometry, given the ground-truth pose. Graded
      only for a run in which `ground_truth_tf_adapter` owned `map -> odom`; any other owner
      is refused, not failed (that is claim M5, and it has no verdict yet: decision D3).
  M2  the mapper is deterministic: replaying the scans it integrated, with the poses it
      integrated them at, gives its final grid byte for byte.
  M4  an update is cheap enough to run on every scan: the 99th percentile of the update time,
      from the per-scan latency file, against the scan period, with the accuracy of the same
      run next to it.

M3 (the grid obeys its own rules) is `cargo test -p robotics_mapping`, not a run.

A run that is not valid, because the robot did not move, the mapper read ground truth, or most
scans were lost, gets no M1 verdict at all: a number computed on a broken run is not a result.
"""
import csv
import hashlib
import math
import os
import sys

# ----------------------------------------------------------------------------------------
# Tolerances. Cell units are cells of the run's own resolution (5 cm in the proposal).
# ----------------------------------------------------------------------------------------

# The tolerance of the metrics: a map-occupied cell farther than this from every surface is
# false occupied, and a surface sample farther than this from every occupied cell is missing.
# One cell width: the least a discretized wall can be asked to be accurate to (benchmark.md
# section 2). `mapping_evaluator` takes it as a parameter and this script refuses a run whose
# metrics carry a different one.
TAU_CELLS = 1.0

# The two rules of the evaluator (robotics_mapping::eval) that decide what the metrics mean, fixed here
# like tau and refused when a run's map-metrics.yaml says otherwise. ADDED ON 2026-10-11, at Stage C
# of the work that followed the three evaluation runs (docs/phase-4-mapping/implementation.md section
# 5.4), before the fourth development run and before any run graded with them; no tolerance below
# was changed. The runs made before that date were graded without these keys and keep the verdict
# files they have; this script no longer regrades them (they would be refused).
#   MAX_INCIDENCE_DEG: a surface sample is observable only if some pose sees it within this angle of
#     its normal. A judgment call, with the curve it rests on, in section 5.4.
#   COMPLETENESS_LOOKUP: a sample is found if an occupied cell is within tau of any cell whose closed
#     box contains it, so that the verdict does not depend on how a tie on a lattice line is rounded.
MAX_INCIDENCE_DEG = 60.0
COMPLETENESS_LOOKUP = "closed_cells"

# Surface error of the occupied cells, 95th percentile. A hit lands in the cell that contains
# the measured endpoint, whose centre is at most half a cell diagonal (0.71 cells) from that
# endpoint; the range noise of the LDS-01 model is 0.0043 of the range, at most 15 mm or 0.3
# of a 5 cm cell. A correct mapper therefore keeps the 95th percentile within one cell.
SURFACE_ERROR_P95_MAX_CELLS = 1.0

# Surface error, maximum. The two bounds above add to 1.0 cells for a flat wall; the extra
# half cell is for the curved pillars, where the nearest surface point to a cell centre is
# not the point the beam hit, and for the 20 ms of pose timing the scan-skew evidence
# (D9) leaves unresolved: at 0.18 m/s that is 4 mm.
SURFACE_ERROR_MAX_MAX_CELLS = 1.5

# False-occupied cells, as a fraction of the occupied cells: phantom obstacles, which block a
# planner. With a correct pose and a correct sensor model there should be none; two per cent
# is the slack for a hit that lands one cell off at a pillar edge. More than that means the
# mapper is writing obstacles where there are none.
FALSE_OCCUPIED_FRACTION_MAX = 0.02

# Completeness: the fraction of observable surface within tau of an occupied cell. A surface
# the beams cross at a grazing angle can fall between two beams (360 beams are 6 cm apart at
# 3.5 m, more than a cell), so 1.0 is not demanded; 0.95 says that at most one sample in
# twenty of what the robot could see is missing from the map.
COMPLETENESS_MIN = 0.95

# False-free surface samples, as a fraction of the observable ones: the dangerous direction,
# since a planner drives through a cell the map calls free. Flat walls must contribute none
# (a surface on a lattice line is not counted: see the closed-cell rule in eval.rs). What is
# left is the silhouette of the four cylinders, whose edge cells are mostly free space that
# grazing beams cross, and which a beam-by-beam update cannot call occupied. The pillars are
# about 14 per cent of the surface samples (4 x 0.94 m of the 24 m of wall plus 3.8 m of
# pillar), so five per cent is about a third of the pillar samples sitting in such a cell: a
# generous bound on silhouette cells, not a target.
FALSE_FREE_FRACTION_MAX = 0.05

# Coverage: the fraction of the cells in line of sight and range of the trajectory that the
# map decided (free or occupied). The route is built to look at the whole room; the cells
# that stay unknown are the ones no beam crossed, so a mapper that works decides nearly all.
COVERAGE_MIN = 0.90

# Not a quality bound, a sanity bound: a run whose observable fraction is below this did not
# see the room, whatever the map says about the little it saw.
OBSERVABLE_FRACTION_MIN = 0.5

# Update latency, 99th percentile, against the scan period (0.2 s at 5 Hz): the update must
# leave at least half of every period to everything else on the machine. The build is the
# debug profile colcon makes, so this is deliberately not tight.
SCAN_PERIOD_S = 0.2
UPDATE_LATENCY_P99_MAX_S = 0.5 * SCAN_PERIOD_S

# Validity: of the scans the mapper tried to integrate (those not skipped for motion, below), at
# least this fraction must have been integrated: a scan is lost when its transform never
# arrives. A map from a stream that lost more than two per cent of its scans to missing
# transforms is a map of a different experiment.
INTEGRATED_FRACTION_MIN = 0.98

# Validity: scans skipped *by design* because the sensor moved too much across the window the
# scan's stamp leaves open (`mapping_node` motion gate; docs/phase-4-mapping/implementation.md
# section 5.2 items 2 and 6). They are not losses and do not count against the fraction above,
# but a mapper that discarded more than half of what it was given did not map the route, so
# this bounds them. The route turns in place 21 times; a quarter to a third of its scans fall
# in those turns and in heading corrections, which is why half is the bound and not a tenth.
#
# A scan skipped as a stationary repeat (`motion_filter`, decision D17) is neither a loss nor a
# motion skip: it is redundancy removed by design, counted in neither fraction and reported.
#
# ADDED AFTER THE HARNESS SMOKE RUN (evidence/phase-4-mapping/2026-10-11-harness-smoke),
# which exposed that the lidar's stamp lags its measurement and led to the gate. It is a
# validity bound and not an accuracy tolerance, no development or evaluation run existed
# when it was added, and this paragraph is the record of that edit.
MOTION_SKIPPED_FRACTION_MAX = 0.5

# Validity: the robot must have moved. Every route is several metres long; this is the
# threshold of scripts/phase3_guards.sh for the same reason (the 2026-10-06 run in which the
# robot never moved and nothing said so).
MIN_TRAVEL_M = 0.2

# Group check: the mean surface error of runs that are supposed to agree. Several runs that
# differ by more than one cell are a finding to investigate (problem.md section 4), not a
# reason to add runs until the average improves.
SPREAD_MAX_CELLS = 1.0

# Comparisons are made to this much slack. Distances in cells on a lattice take values such as
# 1.0 and 1.5 exactly, and a quotient of metres computed in floating point lands a few parts
# in 10^15 on the wrong side of the limit it is meant to meet (a cell at exactly 1.5 cells
# measured 1.5000000000000036 and failed a limit of 1.5). Added after the first development
# run, which showed it; it moves no tolerance by more than a billionth.
EPSILON = 1e-9

# ----------------------------------------------------------------------------------------


def read_kv(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if not line or line.startswith("#") or ": " not in line:
                continue
            k, v = line.split(": ", 1)
            out[k.strip()] = v.strip()
    return out


def num(v):
    if v is None or v == "null":
        return None
    return float(v)


def percentile(sorted_vals, q):
    n = len(sorted_vals)
    rank = min(max(math.ceil(q * n), 1), n)
    return sorted_vals[rank - 1]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


class Check:
    def __init__(self, name, value, limit, op):
        self.name, self.value, self.limit, self.op = name, value, limit, op
        if value is None:
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
        v = "null" if self.value is None else repr(self.value)
        return f"    {self.name}: {{value: {v}, limit: {self.limit!r}, op: '{self.op}', pass: {str(self.ok).lower()}}}"


def grade(run):
    """Grade one run directory; return (text lines, facts used by the group check)."""
    name = os.path.basename(os.path.abspath(run))
    lines = [
        "# Written by scripts/phase4_verdict.py from the files in this directory.",
        "# Every tolerance lives in that script, which was committed before the first evidence run.",
        f"run: {name}",
        f"verdict_script_sha256: {sha256(__file__)}",
    ]
    facts = {"run": name}

    def need(fname):
        p = os.path.join(run, fname)
        return p if os.path.exists(p) else None

    summary_p = need("summary.yaml")
    if not summary_p:
        lines += ["valid: false", "invalid_reasons:", "  - summary.yaml is missing"]
        return lines, facts
    summary = read_kv(summary_p)
    lines.append(f"role: {summary.get('role', 'unknown')}")
    owner = summary.get("owner_of_map_odom", "unknown")
    lines.append(f"owner_of_map_odom: {owner}")
    lines.append(f"mapper_occupied_threshold: {summary.get('mapper_occupied_threshold')}")
    lines.append(f"mapper_free_threshold: {summary.get('mapper_free_threshold')}")
    lines.append(f"mapper_l_occ: {summary.get('mapper_l_occ')}")
    facts.update(
        mapper_sha=summary.get("mapping_node_executable_sha256", "unknown"),
        launch_args=summary.get("launch_arguments", "unknown"),
        role=summary.get("role", "unknown"),
        commit=summary.get("source_git_commit", "unknown"),
    )

    # ---- validity guards ----------------------------------------------------------------
    reasons = []
    metrics_p = need("map-metrics.yaml")
    metrics = read_kv(metrics_p) if metrics_p else {}
    if not metrics_p or "error" in metrics:
        reasons.append("map-metrics.yaml is missing or reports that nothing was graded")
    if summary.get("mapping_node_subscribes_ground_truth") != "false":
        reasons.append(
            "the mapper's subscriptions were not recorded as free of /ground_truth/pose "
            f"(mapping_node_subscribes_ground_truth: {summary.get('mapping_node_subscribes_ground_truth')})"
        )
    travel = None
    traj_p = need("trajectory.csv")
    if traj_p:
        with open(traj_p) as f:
            pts = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
        if pts:
            travel = max(math.hypot(x - pts[0][0], y - pts[0][1]) for x, y in pts)
    if travel is None or travel < MIN_TRAVEL_M:
        reasons.append(f"the robot did not move (largest displacement {travel}, needs {MIN_TRAVEL_M} m)")
    integrated = skipped = skipped_motion = skipped_stationary = 0
    lat = []
    lat_p = need("update-latency.csv")
    if lat_p:
        with open(lat_p) as f:
            for r in csv.DictReader(f):
                if r["outcome"] == "integrated":
                    integrated += 1
                    lat.append(float(r["latency_us"]) * 1e-6)
                elif r["outcome"] == "skipped_motion":
                    skipped_motion += 1
                elif r["outcome"] == "skipped_stationary":
                    skipped_stationary += 1
                else:
                    skipped += 1
    tried = integrated + skipped
    frac = integrated / tried if tried else None
    if frac is None or frac < INTEGRATED_FRACTION_MIN:
        reasons.append(f"too many scans were lost to missing transforms: {integrated} integrated of {tried}")
    seen = integrated + skipped + skipped_motion
    motion_frac = skipped_motion / seen if seen else None
    if motion_frac is None or motion_frac > MOTION_SKIPPED_FRACTION_MAX:
        reasons.append(f"too many scans were skipped for motion: {skipped_motion} of {seen}")
    if metrics and num(metrics.get("tau_cells")) != TAU_CELLS:
        reasons.append(f"the metrics used tau_cells {metrics.get('tau_cells')}, this script grades {TAU_CELLS}")
    if metrics and num(metrics.get("max_incidence_deg")) != MAX_INCIDENCE_DEG:
        reasons.append(f"the metrics used max_incidence_deg {metrics.get('max_incidence_deg')}, this script grades {MAX_INCIDENCE_DEG}")
    if metrics and metrics.get("completeness_lookup") != COMPLETENESS_LOOKUP:
        reasons.append(f"the metrics used completeness_lookup {metrics.get('completeness_lookup')}, this script grades {COMPLETENESS_LOOKUP}")
    # The mapper and the evaluator must read the same grid with the same thresholds, or the
    # states the evaluator graded are not the states the mapper held.
    mt, et = summary.get("mapper_occupied_threshold"), summary.get("evaluator_occupied_threshold")
    if mt is None or et is None or mt != et:
        reasons.append(f"the mapper and the evaluator used different occupied thresholds ({mt} vs {et})")
    valid = not reasons
    lines.append(f"valid: {str(valid).lower()}")
    if reasons:
        lines.append("invalid_reasons:")
        lines += [f"  - {r}" for r in reasons]
    lines += [
        f"scans_integrated: {integrated}",
        f"scans_lost_to_missing_transforms: {skipped}",
        f"scans_skipped_for_motion: {skipped_motion}",
        f"scans_skipped_as_stationary_repeats: {skipped_stationary}",
        f"largest_displacement_m: {travel}",
    ]

    # ---- M1 ------------------------------------------------------------------------------
    if owner != "ground_truth_tf_adapter":
        lines += [
            "M1: REFUSED",
            f"M1_note: map -> odom was owned by {owner!r}, not ground_truth_tf_adapter. This is a "
            "claim M5 run; this script has no M5 grading (decision D3 of "
            "docs/phase-4-mapping/implementation.md is open), so nothing is graded here.",
        ]
    elif not valid:
        lines += ["M1: NOT_GRADED", "M1_note: the run is not valid, see invalid_reasons."]
    else:
        res = num(metrics["resolution_m"])
        p95 = num(metrics.get("surface_error_p95_cells"))
        mx = num(metrics.get("surface_error_max_cells"))
        checks = [
            Check("surface_error_p95_cells", p95, SURFACE_ERROR_P95_MAX_CELLS, "<="),
            Check("surface_error_max_cells", mx, SURFACE_ERROR_MAX_MAX_CELLS, "<="),
            Check("false_occupied_fraction", num(metrics.get("false_occupied_fraction")), FALSE_OCCUPIED_FRACTION_MAX, "<="),
            Check("completeness", num(metrics.get("completeness")), COMPLETENESS_MIN, ">="),
            Check("false_free_fraction", num(metrics.get("false_free_fraction")), FALSE_FREE_FRACTION_MAX, "<="),
            Check("coverage", num(metrics.get("coverage")), COVERAGE_MIN, ">="),
            Check("observable_fraction", num(metrics.get("observable_fraction")), OBSERVABLE_FRACTION_MIN, ">="),
        ]
        ok = all(c.ok for c in checks)
        lines.append(f"M1: {'PASS' if ok else 'FAIL'}")
        lines.append("M1_checks:")
        lines += [c.line() for c in checks]
        lines.append("M1_reported_not_graded:")
        for k in ("surface_error_mean_cells", "surface_error_mean_m", "false_occupied_cells",
                  "false_free_samples", "samples_observable", "samples_total", "samples_outside_grid",
                  "occupied_cells", "free_cells", "unknown_cells", "reachable_cells"):
            lines.append(f"    {k}: {metrics.get(k)}")
        facts["surface_error_mean_cells"] = num(metrics.get("surface_error_mean_cells"))
        facts["resolution_m"] = res

    # ---- M2 ------------------------------------------------------------------------------
    fg, rg = need("final-grid.txt"), need("replay-grid.txt")
    if not (fg and rg):
        lines += ["M2: NOT_GRADED", "M2_note: final-grid.txt or replay-grid.txt is missing."]
    else:
        same = open(fg, "rb").read() == open(rg, "rb").read()
        replayed = summary.get("replay_scans")
        n_ok = replayed is not None and int(replayed) == integrated
        lines += [
            f"M2: {'PASS' if (same and n_ok) else 'FAIL'}",
            f"M2_final_grid_equals_replay_byte_for_byte: {str(same).lower()}",
            f"M2_replayed_scans_equal_integrated_scans: {str(n_ok).lower()}",
        ]

    # ---- M4 ------------------------------------------------------------------------------
    if len(lat) < 10:
        lines += ["M4: NOT_GRADED", "M4_note: fewer than ten integrated scans in update-latency.csv."]
    else:
        s = sorted(lat)
        p50, p95l, p99 = percentile(s, 0.5), percentile(s, 0.95), percentile(s, 0.99)
        c = Check("update_latency_p99_s", p99, UPDATE_LATENCY_P99_MAX_S, "<=")
        lines += [
            f"M4: {'PASS' if c.ok else 'FAIL'}",
            "M4_checks:",
            c.line(),
            "M4_reported_not_graded:",
            f"    update_latency_p50_s: {p50!r}",
            f"    update_latency_p95_s: {p95l!r}",
            f"    update_latency_max_s: {s[-1]!r}",
            f"    scans: {len(s)}",
            f"    scan_period_s: {SCAN_PERIOD_S!r}",
            "    build: debug (colcon default), not a release build",
        ]
    return lines, facts


def main():
    argv = sys.argv[1:]
    if argv == ["--print-tau-cells"]:
        print(TAU_CELLS)
        return 0
    if not argv or argv[0].startswith("-"):
        print(__doc__, file=sys.stderr)
        return 2
    all_facts = []
    for run in argv:
        lines, facts = grade(run)
        text = "\n".join(lines) + "\n"
        with open(os.path.join(run, "phase4-verdict.yaml"), "w") as f:
            f.write(text)
        print(text)
        all_facts.append(facts)
    if len(argv) > 1:
        ev = [f for f in all_facts if f.get("role") == "evaluation"]
        print("group:")
        print(f"  runs: {len(argv)}")
        print(f"  evaluation_runs: {len(ev)}")
        for key in ("mapper_sha", "launch_args", "commit"):
            vals = {f.get(key) for f in all_facts}
            print(f"  same_{key}: {str(len(vals) == 1).lower()}")
        means = [f["surface_error_mean_cells"] for f in ev if f.get("surface_error_mean_cells") is not None]
        if len(means) >= 2:
            spread = max(means) - min(means)
            print(f"  surface_error_mean_cells_spread: {spread!r}")
            print(f"  spread_within_{SPREAD_MAX_CELLS}_cells: {str(spread <= SPREAD_MAX_CELLS).lower()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
