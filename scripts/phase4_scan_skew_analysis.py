#!/usr/bin/env python3
"""Decide, from recorded data, whether a Webots LDS-01 sweep is one instant or a rotation.

    python3 -I scripts/phase4_scan_skew_analysis.py <run directory>

<run directory> holds scans.csv, gt.csv and phases.csv written by
scripts/phase4_scan_skew_record.py. Writes skew-report.yaml next to them.

The method, and why it cannot be talked into an answer. The world is analytic (four inner
wall faces at +/- 3.0 m and four pillars of radius 0.15 m, from
sim/webots/config/landmarks.yaml), so for any lidar pose the exact range of every beam is
known. Two models of when the 360 beams of one /scan were measured are fitted to each
recorded scan, and the one that predicts the ranges better, by a margin the standing-still
phase makes meaningful, is the answer:

  instant   every beam is measured at ground-truth time  t_stamp + d
  swept     beam i is measured at                        t_stamp + d + s * i / (N - 1)

d is a free constant offset between the scan stamp and the ground truth clock; s is the
number of seconds the sweep spans, and it is free too. If a sweep is one instant, fitting s
gains nothing and its estimate is zero within noise; if it is a rotation during which the
robot moves, s is the sweep duration and the instant model leaves a residual that grows with
the robot's speed. The standing-still phase fixes the lidar mounting (x, y, yaw in the robot
frame) and shows what a perfect model's residual is, so the moving phases are judged
against measured noise and not against a number chosen here.

Nothing is assumed about the direction of the sweep: s may come out positive or negative.

Three further checks use the same recording and the same analytic world, and do not depend
on the fit above:

  seam         beams 359 and 0 point the same way, and are the first and last of the sweep. If
               the sweep were a rotation, their ranges would differ by what the robot moved
               between them; the test compares (z[359] - z[0]) with the instant model's
               prediction, scan by scan, and reports the mean and its standard error. A
               0.2 s sweep at 0.18 m/s would put that mean near 36 mm.
  angle grid   the stationary residual with the published angle increment (-2 pi / 359) and
               with the alternative -2 pi / 360, to say which grid the beams really lie on
  no return    for beams that came back `inf`, the exact range the analytic world predicts,
               to say what `inf` means in this simulator
  stamp lag    the constant offset `d` of the instant model, per scan, in the turning phases
               (where a rotation pins it to a few milliseconds): how far the header stamp lags
               the instant the beams were measured at, and whether those instants lie on a
               regular lattice (resultant length R of their phase modulo a trial period; 1 is
               perfectly periodic, 0 is uniform). Added after the first Webots mapping run
               showed that the stamp is not the measurement time.
"""
import csv
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

WALL = 3.0
PILLARS = [(1.8, 1.2), (-1.4, 2.0), (-2.0, -1.0), (0.9, -2.1)]
PILLAR_R = 0.15
TRUNC = 0.10  # m: residuals beyond this are edge effects, not timing, and are capped


def raycast(ox, oy, angles):
    """Exact range from (ox, oy) along each world-frame angle; inf if nothing is hit."""
    dx, dy = np.cos(angles), np.sin(angles)
    best = np.full(angles.shape, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for t in ((WALL - ox) / dx, (-WALL - ox) / dx, (WALL - oy) / dy, (-WALL - oy) / dy):
            t = np.where(t > 0, t, np.inf)
            best = np.minimum(best, t)
    for cx, cy in PILLARS:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - PILLAR_R ** 2
        disc = b * b - c
        t = np.where(disc >= 0, -b - np.sqrt(np.where(disc >= 0, disc, 0)), np.inf)
        t = np.where(t > 0, t, np.inf)
        best = np.minimum(best, t)
    return best


class Track:
    """Ground-truth pose as a function of time: linear in x, y and unwrapped yaw."""

    def __init__(self, t, x, y, yaw):
        self.t, self.x, self.y, self.yaw = t, x, y, np.unwrap(yaw)

    def at(self, tq):
        return (np.interp(tq, self.t, self.x), np.interp(tq, self.t, self.y),
                np.interp(tq, self.t, self.yaw))


def load(run):
    with open(os.path.join(run, "gt.csv")) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["stamp_s"]) for r in rows])
    x = np.array([float(r["x"]) for r in rows])
    y = np.array([float(r["y"]) for r in rows])
    yaw = np.array([2 * math.atan2(float(r["qz"]), float(r["qw"])) for r in rows])
    track = Track(t, x, y, yaw)
    with open(os.path.join(run, "phases.csv")) as f:
        phases = [(r["phase"], float(r["start_sim_s"]), float(r["duration_s"]),
                   float(r["linear"]), float(r["angular"])) for r in csv.DictReader(f)]
    scans = []
    with open(os.path.join(run, "scans.csv")) as f:
        for r in csv.DictReader(f):
            n = int(r["n"])
            scans.append({
                "t": float(r["stamp_s"]), "frame": r["frame_id"],
                "angle_min": float(r["angle_min"]), "inc": float(r["angle_increment"]),
                "time_increment": float(r["time_increment"]),
                "scan_time": float(r["scan_time"]),
                "range_min": float(r["range_min"]), "range_max": float(r["range_max"]),
                "ranges": np.array([float(r[f"r{i}"]) for i in range(n)]),
            })
    return track, phases, scans


def predict(track, scan, mount, d, s):
    """Predicted ranges of one scan under the (d, s) timing model and the lidar mounting."""
    n = len(scan["ranges"])
    idx = np.arange(n)
    ang = scan["angle_min"] + idx * scan["inc"]
    tb = scan["t"] + d + s * idx / (n - 1)
    x, y, yaw = track.at(tb)
    lx, ly, lyaw = mount
    ox = x + lx * np.cos(yaw) - ly * np.sin(yaw)
    oy = y + lx * np.sin(yaw) + ly * np.cos(yaw)
    return raycast(ox, oy, yaw + lyaw + ang)


def residual(track, scan, mount, d, s, valid):
    r = predict(track, scan, mount, d, s)
    return (scan["ranges"] - r)[valid]


def valid_beams(scan, track, mount):
    """Beams that returned inside the sensor limits, away from range discontinuities."""
    z = scan["ranges"]
    ok = np.isfinite(z) & (z >= scan["range_min"]) & (z <= scan["range_max"] - 0.05)
    # drop beams whose neighbours differ a lot: a pillar edge, where a tiny timing change
    # flips a beam between two surfaces and would dominate any least-squares fit
    jump = np.abs(np.diff(z, prepend=z[0])) > 0.25
    jump |= np.abs(np.diff(z, append=z[-1])) > 0.25
    return ok & ~jump


def cost(res):
    return float(np.sum(np.minimum(res * res, TRUNC ** 2)))


def fit_scan(track, scan, mount, with_s):
    valid = valid_beams(scan, track, mount)
    if valid.sum() < 40:
        return None
    best = (np.inf, 0.0, 0.0)
    for s in (np.arange(-0.5, 0.5001, 0.02) if with_s else [0.0]):
        for d in np.arange(-0.3, 0.3001, 0.01):
            c = cost(residual(track, scan, mount, d, s, valid))
            if c < best[0]:
                best = (c, d, s)
    _, d0, s0 = best
    for s in (np.arange(s0 - 0.02, s0 + 0.0201, 0.002) if with_s else [0.0]):
        for d in np.arange(d0 - 0.01, d0 + 0.0101, 0.001):
            c = cost(residual(track, scan, mount, d, s, valid))
            if c < best[0]:
                best = (c, d, s)
    c, d, s = best
    res = residual(track, scan, mount, d, s, valid)
    return {"d": d, "s": s, "rms": float(np.sqrt(np.mean(np.minimum(res * res, TRUNC ** 2)))),
            "beams": int(valid.sum())}


def fit_mount(track, scans):
    """Lidar x, y, yaw in the robot frame, from scans taken while the robot stands still."""
    valid = [valid_beams(sc, track, (0, 0, 0)) for sc in scans]

    def f(p):
        out = []
        for sc, v in zip(scans, valid):
            out.append(residual(track, sc, (p[0], p[1], p[2]), 0.0, 0.0, v))
        return np.concatenate(out)

    sol = least_squares(f, x0=[-0.03, 0.0, 0.0], loss="soft_l1", f_scale=0.02)
    return tuple(float(v) for v in sol.x)


def seam_test(track, scans, mount, t0, dur):
    """Mean of (z[last] - z[0]) minus the instant model's prediction, over a phase."""
    vals = []
    for sc in scans:
        if not (t0 + 1.0 <= sc["t"] <= t0 + dur - 0.3):
            continue
        f = fit_scan(track, sc, mount, with_s=False)
        if not f:
            continue
        z = sc["ranges"]
        p = predict(track, sc, mount, f["d"], 0.0)
        ok = lambda i: np.isfinite(z[i]) and np.isfinite(p[i]) and abs(z[i] - p[i]) < 0.1
        if ok(len(z) - 1) and ok(0):
            vals.append((z[-1] - z[0]) - (p[-1] - p[0]))
    a = np.array(vals)
    if a.size < 2:
        return {"n": int(a.size)}
    return {"n": int(a.size), "mean_m": float(a.mean()),
            "standard_error_m": float(a.std(ddof=1) / math.sqrt(a.size))}


def angle_grid_check(track, scans, mount, t0, dur):
    out = {}
    sel = [sc for sc in scans if t0 + 1.0 <= sc["t"] <= t0 + dur]
    for label, inc in (("published_minus_2pi_over_359", scans[0]["inc"]),
                       ("alternative_minus_2pi_over_360", -2 * math.pi / 360)):
        res = []
        for sc in sel:
            s2 = dict(sc)
            s2["inc"] = inc
            v = valid_beams(s2, track, mount)
            res.append(np.where(v, sc["ranges"] - predict(track, s2, mount, 0.0, 0.0), np.nan))
        a = np.array(res)
        out[label + "_rms_m"] = float(np.sqrt(np.nanmean(np.minimum(a ** 2, TRUNC ** 2))))
    return out


def no_return_check(track, scans, mount, t0, dur):
    inf_pred, fin_pred = [], []
    for sc in scans:
        if not (t0 + 1.0 <= sc["t"] <= t0 + dur):
            continue
        p = predict(track, sc, mount, 0.0, 0.0)
        inf = np.isinf(sc["ranges"])
        inf_pred += list(p[inf])
        fin_pred += list(p[~inf])
    ip, fp = np.array(inf_pred), np.array(fin_pred)
    rmax = scans[0]["range_max"]
    return {
        "inf_beams": int(ip.size), "finite_beams": int(fp.size),
        "inf_beams_min_predicted_range_m": float(ip.min()),
        "inf_beams_with_predicted_range_above_range_max": int((ip > rmax).sum()),
        "inf_beams_with_predicted_range_below_range_max": int((ip <= rmax).sum()),
        "finite_beams_max_predicted_range_m": float(fp.max()),
        "finite_beams_with_predicted_range_above_range_max": int((fp > rmax).sum()),
    }


def stamp_lag(track, scans, mount, phases):
    """Offset between each scan's stamp and its measurement instant, and its periodicity."""
    stamps, offsets = [], []
    for name, t0, dur, lin, ang in phases:
        if ang == 0.0:
            continue  # only turning phases pin d down tightly
        for sc in scans:
            if t0 + 1.0 <= sc["t"] <= t0 + dur - 0.3:
                f = fit_scan(track, sc, mount, with_s=False)
                if f:
                    stamps.append(sc["t"])
                    offsets.append(f["d"])
    s_, d_ = np.array(stamps), np.array(offsets)
    tm = s_ + d_  # the instant the beams were measured at, on the ground-truth clock
    out = {"scans": int(s_.size)}
    lag = -d_
    out["stamp_minus_measurement_s"] = {
        "mean": float(lag.mean()), "sd": float(lag.std()),
        "min": float(lag.min()), "max": float(lag.max())}

    def lattice(times, period):
        a = 2 * math.pi * np.mod(times, period) / period
        r = abs(np.mean(np.exp(1j * a)))
        mean = (math.atan2(np.mean(np.sin(a)), np.mean(np.cos(a))) % (2 * math.pi)) * period / (2 * math.pi)
        dev = (np.mod(times, period) - mean + period / 2) % period - period / 2
        return float(r), float(mean), float(dev.std())

    for period in (0.19, 0.2, 0.21, 0.22):
        r, mean, sd = lattice(tm, period)
        out[f"measurement_instants_period_{period}"] = {
            "resultant_length": r, "mean_phase_s": mean, "phase_sd_s": sd}
    r, _, _ = lattice(s_, 0.2)
    out["header_stamps_period_0.2"] = {"resultant_length": r}
    return out


def summarize(vals):
    a = np.array(vals)
    if a.size == 0:
        return {"n": 0}
    return {"n": int(a.size), "median": float(np.median(a)),
            "p25": float(np.percentile(a, 25)), "p75": float(np.percentile(a, 75)),
            "min": float(a.min()), "max": float(a.max())}


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    run = sys.argv[1]
    track, phases, scans = load(run)
    still = [p for p in phases if p[0] == "still"][0]
    still_scans = [sc for sc in scans if still[1] + 1.0 <= sc["t"] <= still[1] + still[2]]
    mount = fit_mount(track, still_scans)

    stamps = np.array([sc["t"] for sc in scans])
    periods = np.diff(stamps)
    nonfinite = {
        "inf": int(sum(np.isinf(sc["ranges"]).sum() for sc in scans)),
        "nan": int(sum(np.isnan(sc["ranges"]).sum() for sc in scans)),
        "above_range_max": int(sum(((sc["ranges"] > sc["range_max"]) &
                                    np.isfinite(sc["ranges"])).sum() for sc in scans)),
        "below_range_min": int(sum((sc["ranges"] < sc["range_min"]).sum() for sc in scans)),
        "beams_total": int(sum(len(sc["ranges"]) for sc in scans)),
    }
    out = {
        "scans": len(scans),
        "frame_id": scans[0]["frame"],
        "n_beams": len(scans[0]["ranges"]),
        "angle_min": scans[0]["angle_min"],
        "angle_increment": scans[0]["inc"],
        "time_increment_published": sorted({sc["time_increment"] for sc in scans}),
        "scan_time_published": sorted({sc["scan_time"] for sc in scans}),
        "stamp_period_s": summarize(periods),
        "no_return_beams": nonfinite,
        "lidar_mount_in_robot_frame_fitted_standing_still": {
            "x_m": mount[0], "y_m": mount[1], "yaw_rad": mount[2]},
        "angle_grid_standing_still": angle_grid_check(track, scans, mount, still[1], still[2]),
        "no_return_standing_still": no_return_check(track, scans, mount, still[1], still[2]),
        "stamp_lag_turning_phases": stamp_lag(track, scans, mount, phases),
        "phases": {},
    }
    for name, t0, dur, lin, ang in phases:
        sel = [sc for sc in scans if t0 + 1.0 <= sc["t"] <= t0 + dur - 0.3]
        inst = [fit_scan(track, sc, mount, with_s=False) for sc in sel]
        swept = [fit_scan(track, sc, mount, with_s=True) for sc in sel]
        inst = [r for r in inst if r]
        swept = [r for r in swept if r]
        out["phases"][name] = {
            "seam_test": seam_test(track, scans, mount, t0, dur),
            "commanded_linear_m_s": lin, "commanded_angular_rad_s": ang,
            "scans_used": len(swept),
            "instant_model_rms_m": summarize([r["rms"] for r in inst]),
            "instant_model_offset_d_s": summarize([r["d"] for r in inst]),
            "swept_model_rms_m": summarize([r["rms"] for r in swept]),
            "swept_model_span_s": summarize([r["s"] for r in swept]),
            "swept_model_offset_d_s": summarize([r["d"] for r in swept]),
        }

    def dump(d, ind=0):
        lines = []
        for k, v in d.items():
            pad = "  " * ind
            if isinstance(v, dict):
                lines.append(f"{pad}{k}:")
                lines += dump(v, ind + 1)
            elif isinstance(v, list):
                lines.append(f"{pad}{k}: {v}")
            elif isinstance(v, float):
                lines.append(f"{pad}{k}: {v:.6f}")
            else:
                lines.append(f"{pad}{k}: {v}")
        return lines

    text = ["# Written by scripts/phase4_scan_skew_analysis.py from scans.csv, gt.csv, phases.csv.",
            "# Nothing here was typed in. See that script's docstring for the two models."]
    text += dump(out)
    open(os.path.join(run, "skew-report.yaml"), "w").write("\n".join(text) + "\n")
    print("\n".join(text))
    return 0


if __name__ == "__main__":
    sys.exit(main())
