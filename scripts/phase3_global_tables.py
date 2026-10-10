#!/usr/bin/env python3
"""Build the global localization results tables from the saved evidence, and keep the
documentation in step with it.

    python3 -I scripts/phase3_global_tables.py --print                  # show the tables
    python3 -I scripts/phase3_global_tables.py --write <doc.md>         # splice them into a doc
    python3 -I scripts/phase3_global_tables.py --check <doc.md> ...     # fail if a doc is stale

Why this exists. The documentation of this phase does not hold numbers somebody typed: a
figure copied into prose stops matching the run it came from. The global localization
results are a table of dozens of numbers, so the table is generated from the
`summary.yaml` and `trials.csv` files each experiment wrote, and spliced into the document
between

    <!-- global-localization-tables:begin -->
    <!-- global-localization-tables:end -->

A second pair, `kidnap-model-tables`, holds the model-level kidnap recovery measurement.

`--check` regenerates the tables and compares them with what the document holds, so a
document that was edited by hand, or an experiment that was added without the document being
updated, is an error rather than a quiet disagreement.

The evidence root is `$EVIDENCE_ROOT/phase-3-global-localization` (scripts/evidence_root.sh),
or `--evidence <dir>`.
"""

import argparse
import csv
import glob
import os
import re
import sys

import yaml

# Marker pairs: <!-- NAME:begin --> ... <!-- NAME:end -->, one generated block each.
BLOCKS = ("global-localization-tables", "kidnap-model-tables")

DEFAULT_ROOT = "/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/evidence"

# (heading, evidence run directory, one row per particle count found in it)
MODEL_SWEEPS = [
    ("sweep-particles-default-criterion", "0.5"),
    ("sweep-particles-large-default-criterion", "0.5"),
    ("sweep-particles-explained-0.51", "0.51"),
    ("sweep-particles-explained-1.0", "1.0"),
    ("sweep-particles-large-explained-1.0", "1.0"),
]
CONFIRM = [
    ("confirm-fresh-starts-explained-0.5", "0.5"),
    ("confirm-fresh-starts-explained-1.0", "1.0"),
]
# The first Webots batch, one seed per configuration, run before the harness fixes (its
# /diagnostics capture took whichever node published first, and its log has no injection
# stamps). Kept and shown, labelled as such; the final runs below are what the text reads.
WEBOTS_FIRST = [
    "webots-6000-debug-explained-0.5",
    "webots-48000-release-explained-1.0",
    "webots-96000-release-explained-1.0",
]
# The final batch: every configuration, three filter seeds each, the harness and node fixed.
# Directory names are <date>-webots-final-seed<S>-<particles>-<build>-explained-<fraction>.
WEBOTS_CONFIGS = [
    ("6000", "debug", "0.5"),
    ("48000", "release", "1.0"),
    ("96000", "release", "1.0"),
]
CATEGORIES = ("SUCCESS", "WRONG_CONVERGENCE", "LOST_AFTER_CONVERGENCE", "NO_CONVERGENCE")


def final_runs(root, config):
    particles, build, fraction = config
    runs = glob.glob(os.path.join(
        root, f"*-webots-final-seed*-{particles}-{build}-explained-{fraction}"))
    runs = [r for r in runs if load(os.path.join(r, "summary.yaml"))]
    return sorted(runs, key=lambda r: int(re.search(r"-seed(\d+)-", r).group(1)))


def all_final_runs(root):
    out = []
    for config in WEBOTS_CONFIGS:
        out += final_runs(root, config)
    return out


def load(path):
    try:
        with open(path) as handle:
            return yaml.safe_load(handle)
    except (OSError, yaml.YAMLError):
        return None


def pct(x):
    return f"{100.0 * x:.0f}%"


def ci(summary):
    lo, hi = summary["success_rate_wilson95"]
    return f"{100.0 * lo:.0f} to {100.0 * hi:.0f}%"


def find_runs(root, suffix):
    """Evidence directories named <date>-<suffix>."""
    return sorted(glob.glob(os.path.join(root, f"*-{suffix}")))


def model_rows(root, suffix, fraction):
    rows = []
    for run in find_runs(root, suffix):
        for sub in sorted(glob.glob(os.path.join(run, "p*")),
                          key=lambda p: int(os.path.basename(p)[1:])):
            s = load(os.path.join(sub, "summary.yaml"))
            if not s or "outcomes" not in s:
                continue
            o = s["outcomes"]
            ok = s.get("among_successes", {})
            end_ok = 0
            try:
                with open(os.path.join(sub, "trials.csv")) as handle:
                    for t in csv.DictReader(handle):
                        end_ok += (float(t["final_err_pos_m"]) < 0.15
                                   and float(t["final_err_yaw_rad"]) < 0.15)
            except OSError:
                end_ok = None
            rows.append((int(s["config"]["global_particles"]), fraction, s["trials"], o["SUCCESS"],
                         o["WRONG_CONVERGENCE"], o["LOST_AFTER_CONVERGENCE"], o["NO_CONVERGENCE"],
                         pct(s["success_rate"]), ci(s),
                         ok.get("convergence_time_s", {}).get("median"),
                         ok.get("convergence_distance_m", {}).get("median"),
                         ok.get("rmse_position_after_convergence_m", {}).get("median"), end_ok))
    rows.sort(key=lambda r: (r[1], r[0]))
    return rows


def md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    for r in rows:
        out.append("| " + " | ".join(r) + " |")
    return "\n".join(out)


def fmt(x, digits):
    return "n/a" if x is None or x != x else f"{x:.{digits}f}"


def model_table(root, sweeps):
    rows = []
    for suffix, fraction in sweeps:
        rows += model_rows(root, suffix, fraction)
    rows.sort(key=lambda r: (float(r[1]), r[0]))
    body = [[f"{n:,}", frac, str(trials), str(s), str(w), str(l), str(nc), rate, c,
             fmt(t, 1), fmt(d, 2), fmt(rm, 4), "n/a" if e is None else str(e)]
            for n, frac, trials, s, w, l, nc, rate, c, t, d, rm, e in rows]
    return md_table(["global particles", "explained fraction", "trials", "success", "wrong",
                     "lost", "no claim", "success rate", "Wilson 95%", "median time (s)",
                     "median distance (m)", "median RMSE after (m)", "correct at end of route"],
                    body)


def scan_cost_table(root):
    """The clean, single-process timing measurement, if it was made."""
    rows = []
    for run in find_runs(root, "cost-isolated"):
        for sub in sorted(glob.glob(os.path.join(run, "p*")),
                          key=lambda p: int(os.path.basename(p)[1:])):
            s = load(os.path.join(sub, "summary.yaml"))
            if not s or "cost" not in s:
                continue
            rows.append([f"{int(s['config']['global_particles']):,}", str(s["trials"]),
                         fmt(s["cost"]["mean_scan_ms"], 1), fmt(s["cost"]["max_scan_ms"], 1)])
    if not rows:
        return "_Not measured yet._"
    return md_table(["global particles", "trials", "mean ms per scan", "max ms per scan"], rows)


def webots_table(runs):
    rows = []
    for run in runs:
        if True:
            s = load(os.path.join(run, "summary.yaml"))
            if not s or "outcomes" not in s:
                continue
            o, r = s["outcomes"], s["run"]
            ok = s.get("among_successes", {})
            cost = s.get("cost", {})
            det = s.get("detector", {})
            end_ok = 0
            # The 20 start poses are the 10 random ones of the fresh set followed by the 10
            # chosen by the ambiguity ranking and the symmetry (the starts directory has both
            # as files), so the trial index says which kind a trial is.
            by_kind = {"random": [0, 0, 0, 0], "ambiguous": [0, 0, 0, 0]}  # n, success, wrong, none
            for tv in sorted(glob.glob(os.path.join(run, "trial-*", "verdict.yaml"))):
                v = load(tv)
                t = v["trial"] if v else {}
                if "final_position_error_m" in t and "final_yaw_error_rad" in t:
                    end_ok += (t["final_position_error_m"] < 0.15 and t["final_yaw_error_rad"] < 0.15)
                index = int(os.path.basename(os.path.dirname(tv)).split("-")[1])
                k = by_kind["random" if index < 10 else "ambiguous"]
                k[0] += 1
                k[1] += t.get("category") == "SUCCESS"
                k[2] += t.get("category") == "WRONG_CONVERGENCE"
                k[3] += t.get("category") == "NO_CONVERGENCE"
            rows.append([f"{int(r['global_particles']):,}", str(r["build"]), str(r["seed_of_trial_0"]),
                         str(r["convergence_min_explained_fraction"]), str(s["trials_graded"]),
                         str(o["SUCCESS"]), str(o["WRONG_CONVERGENCE"]),
                         str(o["LOST_AFTER_CONVERGENCE"]), str(o["NO_CONVERGENCE"]),
                         str(s["trials_invalid"]), pct(s["success_rate"]) if s["trials_graded"] else "n/a",
                         ci(s) if s["trials_graded"] else "n/a",
                         fmt(ok.get("convergence_time_s", {}).get("median"), 1),
                         fmt(ok.get("convergence_distance_m", {}).get("median"), 2),
                         fmt(ok.get("rmse_position_after_convergence_m", {}).get("median"), 4),
                         fmt(cost.get("observe_ms_max_median_over_trials"), 0),
                         (f"{det['trials_averaging_more_than_4_detections_per_scan']} of "
                          f"{det['trials_with_a_diagnostics_reading']}")
                         if "trials_with_a_diagnostics_reading" in det else "n/a",
                         str(s.get("trials_where_the_kidnap_trigger_fired", "n/a")),
                         injections_cell(s),
                         str(end_ok),
                         "{} of {}".format(by_kind["random"][1], by_kind["random"][0]),
                         "{} of {}".format(by_kind["ambiguous"][1], by_kind["ambiguous"][0]),
                         "{} / {}".format(by_kind["random"][2], by_kind["ambiguous"][2]),
                         "{} / {}".format(by_kind["random"][3], by_kind["ambiguous"][3])])
    if not rows:
        return "_Not run yet._"
    return md_table(["global particles", "build", "seed", "explained fraction", "trials", "success",
                     "wrong", "lost", "no claim", "invalid", "success rate", "Wilson 95%",
                     "median time (s)", "median distance (m)", "median RMSE after (m)",
                     "median of per-trial max ms/scan", "trials with > 4 detections per scan, of those with a reading",
                     "trials where the kidnap trigger fired",
                     "injections: total (false, estimate already right)",
                     "correct at end of route (ground truth)",
                     "success, 10 random starts", "success, 10 ambiguity-targeted starts",
                     "wrong claims, random / targeted", "no claim, random / targeted"], rows)


def injections_cell(s):
    k = s.get("kidnap_injections")
    if not k:
        return "n/a"
    if k.get("ungraded_log_has_no_stamp"):
        return f"{k['total']} (ungraded: log has no stamp)"
    return f"{k['total']} ({k['false_estimate_was_already_correct']})"


def span(values):
    return f"{min(values)} to {max(values)}" if values else "n/a"


def webots_final_summary(root):
    """One row per configuration, pooled over its seeds: what the filter claimed next to
    where it actually ended up, and how far the seeds spread."""
    rows = []
    for config in WEBOTS_CONFIGS:
        runs = final_runs(root, config)
        if not runs:
            continue
        sums = [load(os.path.join(r, "summary.yaml")) for r in runs]
        n = sum(s["trials_graded"] for s in sums)
        per = {c: [s["outcomes"][c] for s in sums] for c in CATEGORIES}
        end = [s["correct_at_end_of_route"]["total"] for s in sums]
        wrong_fixed = sum(s["correct_at_end_of_route"]["among_WRONG_CONVERGENCE"] for s in sums)
        none_right = sum(s["correct_at_end_of_route"]["among_NO_CONVERGENCE"] for s in sums)
        k = [s["kidnap_injections"] for s in sums]
        seeds = ", ".join(str(s["run"]["seed_of_trial_0"]) for s in sums)
        rows.append([f"{int(config[0]):,}", config[1], config[2], seeds, str(n),
                     f"{sum(per['SUCCESS'])} ({span(per['SUCCESS'])})",
                     f"{sum(end)} ({span(end)})",
                     f"{sum(per['WRONG_CONVERGENCE'])} ({span(per['WRONG_CONVERGENCE'])})",
                     f"{sum(per['LOST_AFTER_CONVERGENCE'])} ({span(per['LOST_AFTER_CONVERGENCE'])})",
                     f"{sum(per['NO_CONVERGENCE'])} ({span(per['NO_CONVERGENCE'])})",
                     str(wrong_fixed), str(none_right),
                     f"{sum(x['total'] for x in k)}",
                     f"{sum(x['false_estimate_was_already_correct'] for x in k)}",
                     f"{sum(x['trials_with_a_false_injection'] for x in k)}"])
    if not rows:
        return "_Not run yet._"
    return md_table(["global particles", "build", "explained fraction", "seeds of trial 0",
                     "trials", "claim-based: claimed and right (per-seed range)",
                     "correct at end of route, ground truth (per-seed range)",
                     "wrong claim", "lost after a right claim", "no claim",
                     "wrong claims correct at the end", "no-claim trials correct at the end",
                     "kidnap injections", "of which false (estimate already right)",
                     "trials with a false injection"], rows)


def webots_per_start_pooled(root):
    """One row per start pose, one column per configuration, counting over the seeds."""
    columns, starts = [], []
    for config in WEBOTS_CONFIGS:
        runs = final_runs(root, config)
        if not runs:
            continue
        col = {}
        for run in runs:
            for t in sorted(glob.glob(os.path.join(run, "trial-*"))):
                v = load(os.path.join(t, "verdict.yaml"))
                st = os.path.join(t, "start.txt")
                if not v or not os.path.exists(st):
                    continue
                m = re.search(r"x=(\S+) y=(\S+) yaw=(\S+)", open(st).read())
                key = tuple(round(float(g), 2) for g in m.groups())
                if key not in starts:
                    starts.append(key)
                tr = v["trial"]
                c = col.setdefault(key, {"ok": 0, "WRONG": 0, "LOST": 0, "none": 0, "end": 0, "n": 0})
                c["n"] += 1
                c["end"] += bool(tr.get("correct_at_end_of_route"))
                c[{"SUCCESS": "ok", "WRONG_CONVERGENCE": "WRONG", "NO_CONVERGENCE": "none",
                   "LOST_AFTER_CONVERGENCE": "LOST"}.get(tr["category"], "none")] += 0 if \
                    tr["category"] not in CATEGORIES else 1
        columns.append((f"{int(config[0]):,} {config[1]} f={config[2]}", col))
    if not columns:
        return "_Not run yet._"

    def cell(c):
        if not c:
            return ""
        parts = [f"{k} {c[k]}" for k in ("ok", "WRONG", "LOST", "none") if c[k]]
        return ", ".join(parts) + f"; end {c['end']}/{c['n']}"
    body = [[f"({x:.2f}, {y:.2f}, {yaw:.2f})"] + [cell(c.get(k)) for _, c in columns]
            for (x, y, yaw) in starts]
    return md_table(["start pose x, y, yaw"] + [h for h, _ in columns], body)


def webots_per_start(runs):
    """One row per start pose, one column per Webots run."""
    if not runs:
        return "_Not run yet._"
    starts = None
    columns = []
    for run in runs:
        env = {}
        for line in open(os.path.join(run, "environment.txt")):
            if ": " in line:
                k, v = line.split(": ", 1)
                env[k.strip()] = v.strip()
        col = {}
        for t in sorted(glob.glob(os.path.join(run, "trial-*"))):
            v = load(os.path.join(t, "verdict.yaml"))
            st = os.path.join(t, "start.txt")
            if not v or not os.path.exists(st):
                continue
            m = re.search(r"x=(\S+) y=(\S+) yaw=(\S+)", open(st).read())
            key = tuple(round(float(g), 2) for g in m.groups())
            tr = v["trial"]
            cat = {"SUCCESS": "ok", "WRONG_CONVERGENCE": "WRONG", "NO_CONVERGENCE": "none",
                   "LOST_AFTER_CONVERGENCE": "LOST", "INVALID": "invalid",
                   "SKIPPED": "skipped"}.get(tr["category"], tr["category"])
            if "convergence_time_s" in tr:
                cat += f" {tr['convergence_time_s']:.0f}s"
            col[key] = cat
        columns.append((f"{int(env['global_particles']):,} {env['build']} f={env['convergence_min_explained_fraction']}", col))
        starts = list(col.keys()) if starts is None else starts + [k for k in col if k not in starts]
    body = [[f"({x:.2f}, {y:.2f}, {yaw:.2f})"] + [c.get(k, "") for _, c in columns]
            for (x, y, yaw) in starts]
    return md_table(["start pose x, y, yaw"] + [h for h, _ in columns], body)


def kidnap_model_table(root):
    rows = []
    for run in sorted(glob.glob(os.path.join(root, "*-kidnap-model-*"))):
        what = "as shipped" if run.endswith("existing-tuning") else "what-if: particle count only"
        for sub in sorted(glob.glob(os.path.join(run, "p*")),
                          key=lambda p: int(os.path.basename(p)[1:])):
            s = load(os.path.join(sub, "summary.yaml"))
            if not s or "recovered" not in s:
                continue
            c = s["config"]
            lo, hi = s["recovery_rate_wilson95"]
            rows.append([what, f"{int(c['steady_state_particles']):,}",
                         str(c["recovery_sustained_updates"]), str(c["recovery_injection_fraction"]),
                         str(s["trials"]), str(s["recovered"]), pct(s["recovery_rate"]),
                         f"{100.0 * lo:.0f} to {100.0 * hi:.0f}%",
                         fmt(s["recovery_time_s"]["median"], 1)])
    if not rows:
        return "_Not measured yet._"
    return md_table(["configuration", "steady-state particles", "persistence (scans)",
                     "reset fraction", "kidnaps", "recovered", "recovery rate", "Wilson 95%",
                     "median recovery time (s)"], rows)


def generate_blocks(root):
    return {"global-localization-tables": generate(root),
            "kidnap-model-tables": kidnap_model_table(root) + "\n"}


def generate(root):
    parts = []
    parts.append("**Model-level, 40 development start poses (starts seed 1000).** The real "
                 "`McLocalizer`, a simulated robot and simulated detections "
                 "(`crates/robotics_localization/examples/global_trials.rs`). These are the poses "
                 "the explained-fraction criterion was chosen on, so they are optimistic for it.\n\n"
                 + model_table(root, MODEL_SWEEPS))
    parts.append("**Model-level, 100 fresh start poses (starts seed 2000).** The same experiment "
                 "on poses that played no part in any choice.\n\n" + model_table(root, CONFIRM))
    parts.append("**Cost of one scan, single process, nothing else running.**\n\n"
                 + scan_cost_table(root))
    parts.append("**End to end in Webots, final: each configuration over three filter seeds, "
                 "pooled** (real detector, real laser, real odometry, ground truth from the "
                 "supervisor; the same 20 start poses for every seed). The claim-based column "
                 "is what the filter said and was right about (`SUCCESS`); the ground-truth "
                 "column is where the estimate actually was at the end of the route, whatever "
                 "was claimed. Each total is followed by the range over the seeds.\n\n"
                 + webots_final_summary(root))
    parts.append("**The final Webots runs, one row per run.**\n\n"
                 + webots_table(all_final_runs(root)))
    parts.append("**The final Webots runs, one row per start pose**, counted over the three "
                 "seeds of each configuration (`ok` claimed and right, `WRONG` a wrong claim, "
                 "`LOST` a right claim lost again, `none` no claim; `end` is how many of the "
                 "seeds ended inside both tolerances, by ground truth).\n\n"
                 + webots_per_start_pooled(root))
    first = []
    for suffix in WEBOTS_FIRST:
        first += find_runs(root, suffix)
    parts.append("**The first Webots batch, seed 1, before the harness fixes**, kept for "
                 "comparison: its diagnostics columns cover only the trials whose one "
                 "`/diagnostics` message happened to be mcl_node's, and its injections cannot be "
                 "graded because the log line had no stamp. Claims, errors and the ground-truth "
                 "column do not depend on either.\n\n" + webots_table(first))
    return "\n\n".join(parts) + "\n"


def splice(text, blocks):
    """Replace each marked block that the document has. A document need not hold all of them,
    but it has to hold at least one, so that a mistyped marker is an error and not a no-op."""
    found = 0
    for name, block in blocks.items():
        begin, end = f"<!-- {name}:begin -->", f"<!-- {name}:end -->"
        if begin not in text:
            continue
        if end not in text:
            raise SystemExit(f"the document has {begin} but no {end}")
        found += 1
        head, rest = text.split(begin, 1)
        _, tail = rest.split(end, 1)
        text = f"{head}{begin}\n\n{block}\n{end}{tail}"
    if not found:
        raise SystemExit("the document has none of the marker pairs " + ", ".join(blocks))
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--evidence", default=None)
    ap.add_argument("--print", action="store_true")
    ap.add_argument("--write", metavar="DOC")
    ap.add_argument("--check", nargs="+", metavar="DOC")
    args = ap.parse_args()
    root = args.evidence or os.path.join(
        os.environ.get("EVIDENCE_ROOT") or DEFAULT_ROOT, "phase-3-global-localization")
    if not os.path.isdir(root):
        print(f"no evidence directory at {root}", file=sys.stderr)
        return 2
    blocks = generate_blocks(root)
    if args.print:
        for name, block in blocks.items():
            print(f"<!-- {name} -->\n{block}")
    if args.write:
        text = open(args.write, encoding="utf-8").read()
        open(args.write, "w", encoding="utf-8").write(splice(text, blocks))
        print(f"wrote the tables into {args.write}")
    if args.check:
        stale = []
        for doc in args.check:
            text = open(doc, encoding="utf-8").read()
            if splice(text, blocks) != text:
                stale.append(doc)
        if stale:
            print("stale (regenerate with --write): " + ", ".join(stale), file=sys.stderr)
            return 1
        print("the documentation matches the evidence")
    return 0


if __name__ == "__main__":
    sys.exit(main())
