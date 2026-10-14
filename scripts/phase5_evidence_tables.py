#!/usr/bin/env python3
"""The result tables of the Phase 5 evidence note, generated from the run directories.

    python3 -I scripts/phase5_evidence_tables.py                   # tables on stdout
    python3 -I scripts/phase5_evidence_tables.py --check NOTE.md   # exit 1 if NOTE.md disagrees

Reads `slam-metrics.yaml` (and the verdicts written next to it) of every directory under
`evidence/phase-5-slam/` that starts with `v2-`: the graded set. `batch-1-before-front-end-fix/` is the
superseded first batch and is not read. Every number printed is a number in a metrics file or computed
from them here (the Wilson bound, the chi-squared interval, a ratio); nothing is typed by hand, and
`--check` is how the note stays true: the block between the markers must equal what this prints.
"""
import importlib.util
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "evidence", "phase-5-slam")
BEGIN, END = "<!-- BEGIN GENERATED TABLES (scripts/phase5_evidence_tables.py) -->", "<!-- END GENERATED TABLES -->"

spec = importlib.util.spec_from_file_location("phase5_verdict", os.path.join(HERE, "phase5_verdict.py"))
pv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pv)


def runs():
    out = []
    for name in sorted(os.listdir(ROOT)):
        p = os.path.join(ROOT, name, "slam-metrics.yaml")
        if name.startswith("v2-") and os.path.exists(p):
            out.append((name, yaml.safe_load(open(p)), yaml.safe_load(open(os.path.join(ROOT, name, "phase5-verdict.yaml")))))
    return out


def short(name):
    return name[3:]


def mm(x):
    return f"{x * 1000:.1f}"


def table(header, rows):
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    lines += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(lines)


def build():
    R = runs()
    syn = [r for r in R if r[1]["run"]["data_source"] == "synthetic"]
    rec = [r for r in R if r[1]["run"]["data_source"] == "webots_recorded"]
    nat = [r for r in syn if r[1]["run"]["injected_yaw_bias_rad_per_keyframe"] == 0]
    inj = [r for r in syn if r[1]["run"]["injected_yaw_bias_rad_per_keyframe"] != 0]
    out = []

    def s1a_rows(rs):
        rows = []
        for n, m, v in rs:
            a = m["s1a"]
            t, r = a["translation_error_m"], a["rotation_error_rad"]
            rows.append([short(n), a["registrations_made"], a["registrations_graded"],
                         f"{mm(t['median'])} / {mm(t['p95'])} / {mm(t['max'])}",
                         f"{mm(r['median'])} / {mm(r['p95'])} / {mm(r['max'])}", v["claims"]["S1a"]["status"]])
        return rows
    hdr = ["run", "made", "graded", "translation error, median / p95 / max (mm)", "rotation error, median / p95 / max (mrad)", "verdict"]
    out += ["**S1a, synthetic (Rusty, exact ground truth)**", "", table(hdr, s1a_rows([r for r in nat])), ""]
    out += ["**S1a, Webots recordings (ground truth gated, timing-limited; see the note)**", "", table(hdr, s1a_rows(rec)), ""]

    rows = []
    for n, m, v in nat + rec:
        s = m["s2"]
        lo = pv.chi2_quantile(3 * s["samples"], pv.S2_ALPHA / 2) / s["samples"]
        hi = pv.chi2_quantile(3 * s["samples"], 1 - pv.S2_ALPHA / 2) / s["samples"]
        label = "OVERCONFIDENT" if s["mean_nees"] > hi else "CONSERVATIVE" if s["mean_nees"] < lo else "CONSISTENT"
        rows.append([short(n), s["samples"], f"{s['mean_nees']:.3f}", f"[{lo:.3f}, {hi:.3f}]", label])
    out += ["**S2** (`info_inflation` 1.5, frozen from the development measurement; reported, never pass or fail)", "",
            table(["run", "samples", "mean NEES", "95% interval of the chi-squared(3) mean", "label"], rows), ""]

    rows = []
    for n, m, v in nat:
        for rg in m["s1b"]["ranges"]:
            lo, hi = pv.wilson(rg["successes"], rg["trials"])
            graded = rg["offset_m"] <= pv.S1B_GRADED_MAX_OFFSET_M + 1e-9 and rg["offset_rad"] <= pv.S1B_GRADED_MAX_OFFSET_RAD + 1e-9
            rows.append([short(n), f"{rg['offset_m']} m / {rg['offset_rad']} rad", f"{rg['successes']}/{rg['trials']}", f"[{lo:.3f}, {hi:.3f}]", "graded" if graded else "reported"])
    out += ["**S1b** (model level; one scan pair per run; success = settled and within 0.02 m and 0.01 rad)", "",
            table(["run", "initial offset (up to)", "successes", "Wilson 95%", ""], rows), ""]

    def s4_rows(rs):
        rows = []
        for n, m, v in rs:
            s4 = m["s4"]
            o, g = s4["odometry"], s4["graph"]
            rows.append([short(n), s4["keyframes_made"], f"{s4['path_length_m']:.1f}",
                         f"{mm(o['ate_rms_m'])} / {mm(o['ate_max_m'])} / {mm(o['final_error_m'])}",
                         f"{mm(g['ate_rms_m'])} / {mm(g['ate_max_m'])} / {mm(g['final_error_m'])}",
                         f"{o['drift_m_per_m']:.2e} / {g['drift_m_per_m']:.2e}",
                         f"{g['ate_rms_m'] / o['ate_rms_m']:.2f}", v["claims"]["S4"]["status"]])
        return rows
    hdr = ["run", "keyframes", "path (m)", "front end alone: ATE rms / max / final (mm)", "optimized graph: ATE rms / max / final (mm)", "drift, front end / graph (m per m)", "graph ATE over front-end ATE", "verdict"]
    out += ["**S4, natural drift (nothing injected)**", "", table(hdr, s4_rows(nat)), ""]
    out += ["**S4, with a yaw error injected into the edges (model level: says nothing about the scan matcher's drift)**", "", table(hdr, s4_rows(inj)), ""]

    rows = []
    for n, m, v in nat + inj:
        s5 = m["s5"]
        acc = s5["accepted_correct"] + s5["accepted_wrong"]
        lo, hi = pv.wilson(s5["accepted_correct"], acc)
        rej = ", ".join(f"{k} {x}" for k, x in s5["rejected_by_reason"].items() if x) or "none"
        rows.append([short(n), s5["candidates"], s5["accepted_correct"], s5["accepted_wrong"], rej,
                     f"{s5['true_loops_accepted']}/{s5['true_loops_proposed']}",
                     f"{s5['keyframes_with_accepted_loop']}/{s5['keyframes_with_true_partner']}",
                     f"[{lo:.4f}, {hi:.4f}]", f"{3 / acc:.4f}" if acc else "n/a"])
    out += ["**S5** (every candidate the detector proposed, labelled by ground truth; wrong = measurement error over 0.05 m or 0.03 rad)", "",
            table(["run", "candidates", "accepted, correct", "accepted, **wrong**", "rejected, by reason", "true loops accepted / proposed", "keyframes with a true partner that got a loop edge", "precision, Wilson 95%", "rule of three (wrong-accept rate below)"], rows), ""]

    rows = []
    for n, m, v in syn + rec:
        s6, a = m["s6"], m["s1a"]
        rg, op = s6["registration_latency_s"], s6["optimization_latency_s"]
        rows.append([short(n), "identical" if s6["replay_identical"] else "**DIFFERENT**",
                     f"{rg['median'] * 1000:.2f} / {rg['p95'] * 1000:.2f} / {rg['p99'] * 1000:.2f}",
                     (f"{int(op['count'])}: {op['median']:.3f} / {op['p95']:.3f} / {op['p99']:.3f}" if op["count"] else "none"),
                     f"{mm(a['translation_error_m']['p95'])}", v["claims"]["S6"]["status"]])
    out += ["**S6** (release build, WSL2 host in each run's `environment.txt`; registrations are scan-to-map steps for synthetic runs and pair registrations for recordings)", "",
            table(["run", "replay", "registration ms, median / p95 / p99", "optimizations: count, s median / p95 / p99", "S1a translation p95 of the same run (mm)", "verdict"], rows), ""]

    rows = []
    for n, m, v in syn:
        s = m["s7"]
        d, e, r = s["dead_reckoning"], s["ekf_slam"], s["rbpf"]
        rows.append([short(n), f"{d['ate_rms_m']:.3f} / {d['final_error_m']:.3f}",
                     f"{e['ate_rms_m']:.3f} / {e['ate_max_m']:.3f}", f"{e['mean_nees']:.2f}", f"{e['nees_within_975_bound_fraction']:.2f}",
                     f"{e['landmarks']} on {e['distinct_pillars']} pillars; worst {e['landmark_error_max_m']:.2f} m",
                     f"{r['ate_rms_m']:.3f} / {r['ate_max_m']:.3f}", f"{r['resamples']}, {r['mean_n_eff_before']:.1f}"])
    out += ["**S7** (informative, no ranking: synthetic odometry with the declared noise and a systematic yaw error of 0.005 rad per metre)", "",
            table(["run", "dead reckoning of that odometry: ATE rms / final (m)", "EKF-SLAM: ATE rms / max (m)", "EKF-SLAM mean pose NEES", "share of steps within the 97.5% bound", "EKF-SLAM landmarks", "grid RBPF (30 particles): ATE rms / max (m)", "RBPF resamples, mean N_eff before"], rows), ""]
    return "\n".join(out)


def main(argv):
    block = build()
    if "--check" in argv:
        path = argv[argv.index("--check") + 1]
        text = open(path).read()
        if BEGIN not in text or END not in text:
            print(f"{path}: markers not found")
            return 1
        got = text.split(BEGIN, 1)[1].split(END, 1)[0].strip("\n")
        if got != block.strip("\n"):
            print(f"{path}: the generated block is stale; run scripts/phase5_evidence_tables.py and paste it between the markers")
            return 1
        print(f"{path}: generated tables are current")
        return 0
    print(BEGIN)
    print(block)
    print(END)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
