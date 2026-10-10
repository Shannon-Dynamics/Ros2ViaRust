#!/usr/bin/env python3
"""What kind of pose is a wrong global localization claim?

    python3 -I scripts/phase3_wrong_claim_analysis.py <phase-3-global-localization directory>
    python3 -I scripts/phase3_wrong_claim_analysis.py <directory> --check <saved yaml>

Pools every WRONG_CONVERGENCE trial of every model-level run under the directory
(`*/p*/trials.csv`) and, separately, of every final end-to-end Webots run
(`*-webots-final-*/trial-*/verdict.yaml`), and reports how far the yaw of the wrong pose is from the truth: within a
tolerance of a half turn, of a quarter turn, or neither. A wrong claim that is a rotated copy
of the true pose is the symmetry of the map at work (scripts/phase3_map_symmetry.py); one that
is neither is something else. Writes YAML on stdout; `--check` instead compares the output
with a saved copy and exits non-zero if they differ, which is how the saved
`2026-10-10-ambiguity/wrong-claims-yaw.yaml` is kept honest (scripts/gates.sh, `docs`).

The yaw errors are absolute and wrapped to [0, pi] by the code that wrote them
(crates/robotics_localization/examples/global_trials.rs, scripts/phase3_global_verdict.py),
so a quarter turn either way is one number, pi / 2.
"""
import csv
import io
import glob
import math
import os
import sys

import yaml

TOLERANCE_RAD = 0.3


def classify(errors):
    half = sum(abs(e - math.pi) < TOLERANCE_RAD for e in errors)
    quarter = sum(abs(e - math.pi / 2) < TOLERANCE_RAD for e in errors)
    return half, quarter


def emit(out, name, errors, per_run):
    n = len(errors)
    half, quarter = classify(errors)
    print(f"{name}:", file=out)
    print(f"  wrong_claims_pooled: {n}", file=out)
    print(f"  within_tolerance_of_a_half_turn: {half}", file=out)
    print(f"  within_tolerance_of_a_quarter_turn: {quarter}", file=out)
    print(f"  neither: {n - half - quarter}", file=out)
    if n:
        print(f"  fraction_rotated_twins: {(half + quarter) / n:.4f}", file=out)
    print("  runs_contributing:", file=out)
    for run, count in per_run.items():
        print(f"    {run}: {count}", file=out)


def main():
    args = sys.argv[1:]
    check = None
    if len(args) == 3 and args[1] == "--check":
        check = args[2]
        args = args[:1]
    if len(args) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    root = args[0]
    errors, per_run = [], {}
    for path in sorted(glob.glob(os.path.join(root, "*", "p*", "trials.csv"))):
        run = os.path.basename(os.path.dirname(os.path.dirname(path)))
        found = [float(r["err_yaw_at_conv_rad"]) for r in csv.DictReader(open(path))
                 if r["category"] == "WRONG_CONVERGENCE"]
        errors += found
        per_run[run] = per_run.get(run, 0) + len(found)
    w_errors, w_per_run = [], {}
    for path in sorted(glob.glob(os.path.join(root, "*-webots-final-*", "trial-*", "verdict.yaml"))):
        run = os.path.basename(os.path.dirname(os.path.dirname(path)))
        t = (yaml.safe_load(open(path)) or {}).get("trial", {})
        w_per_run.setdefault(run, 0)
        if t.get("category") == "WRONG_CONVERGENCE":
            w_errors.append(float(t["yaw_error_at_convergence_rad"]))
            w_per_run[run] += 1

    out = io.StringIO()
    print("# By scripts/phase3_wrong_claim_analysis.py; no number here is typed.", file=out)
    print(f"tolerance_rad: {TOLERANCE_RAD}", file=out)
    emit(out, "model_level", errors, per_run)
    emit(out, "webots_final", w_errors, w_per_run)
    text = out.getvalue()
    if check is None:
        sys.stdout.write(text)
        return 0
    saved = open(check).read()
    if saved != text:
        print(f"{check} does not match the evidence; regenerate it with this script", file=sys.stderr)
        return 1
    print(f"{check}: matches the evidence")
    return 0


if __name__ == "__main__":
    sys.exit(main())
