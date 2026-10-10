//! Experiments for global localization from a uniform prior (Phase 3, P2 from nothing).
//!
//! ```text
//! cargo run --release -p robotics_localization --example global_trials -- route     <flags>
//! cargo run --release -p robotics_localization --example global_trials -- ambiguity <flags>
//! cargo run --release -p robotics_localization --example global_trials -- trials    <flags>
//! ```
//!
//! Flags are `--name value`. Every subcommand takes `--map <landmarks.yaml>`.
//!
//! * `route --x X --y Y --yaw YAW` prints the exploration route for a start pose as a
//!   route file for `route_driver`. The Webots trials and the trials below use this one
//!   generator, so the two experiments drive the same route from the same start.
//! * `kidnap` runs the *existing* kidnap recovery of the tracking filter, unchanged, as a
//!   model-level experiment: a filter that knows its pose is carried to another pose
//!   without being told, and is graded on whether and how fast it finds itself again. It is
//!   here because that recovery is itself a global re-localization from a uniform reset, so
//!   what the global search experiments show about particle counts bears on it.
//! * `starts --n N --starts-seed S` prints the random start poses `trials` would use, as a
//!   starts file, so another experiment can use exactly the same ones.
//! * `ambiguity --out FILE` ranks start poses by how many separate poses explain one static
//!   scan, which is the static ambiguity of the map under this filter's own likelihood.
//! * `trials` runs the real [`McLocalizer`] (`new_uniform_prior`, `predict`, `observe`)
//!   against a simulated robot and simulated pillar detections, many times, and writes one
//!   CSV row per trial plus a summary. See the flags in [`TrialArgs`].
//!
//! What the simulated robot and detector are, and are not. The robot is a kinematic
//! unicycle that follows the route like `route_driver` does (turn in place at 0.8 rad/s,
//! then straight at 0.18 m/s, 0.06 m waypoint tolerance), with exact odometry, as the
//! Webots odometry has no noise model. The detector reports every pillar whose surface is
//! within 3.2 m (`PillarDetector::max_range`) and not hidden behind another pillar, with
//! Gaussian range and bearing noise. It has no false detections and no missed ones, no
//! partial occlusion and no beam geometry, which the real detector has; so these results
//! bound the *filter's* behaviour, and the Webots trials are what includes the detector.

use nalgebra::Vector2;
use rand::rngs::SmallRng;
use rand::{Rng, RngExt, SeedableRng};
use robotics_core::angle::{angle_diff, normalize_angle};
use robotics_core::Se2;
use robotics_localization::{
    sample_free_pose, GlobalLocalizationConfig, MapFile, McLocalizer, MclConfig,
};
use robotics_models::sensor::landmark::{LandmarkMap, LandmarkModel, RangeBearing};
use std::collections::HashMap;
use std::f64::consts::PI;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;
use std::time::Instant;

/// Success tolerances, the same numbers as `scripts/phase3_kidnap_verdict.py`.
const POSITION_TOLERANCE_M: f64 = 0.15;
const YAW_TOLERANCE_RAD: f64 = 0.15;
/// `PillarDetector::max_range`: the largest *surface* range a detection is made at.
const DETECTOR_MAX_RANGE_M: f64 = 3.2;
/// The filter's assumed range and bearing sigma (`MclConfig::default`).
const MODEL_SIGMA: f64 = 0.05;
/// Where the laser sits on the base, as the simulated robot reports it through TF.
const LASER_X: f64 = -0.03;
/// Side of the exploration square (m).
const ROUTE_SIDE_M: f64 = 1.2;
/// A route may not come closer to a pillar centre than this (m), nor leave this square.
const ROUTE_PILLAR_CLEARANCE_M: f64 = 0.5;
const ROUTE_ROOM_LIMIT_M: f64 = 2.6;
const ROBOT_FOOTPRINT_M: f64 = 0.11;

// ---------------------------------------------------------------- arguments

fn flags(args: &[String]) -> HashMap<String, String> {
    let mut out = HashMap::new();
    let mut i = 0;
    while i < args.len() {
        if let Some(name) = args[i].strip_prefix("--") {
            let value = args.get(i + 1).cloned().unwrap_or_default();
            out.insert(name.to_string(), value);
            i += 2;
        } else {
            i += 1;
        }
    }
    out
}

fn get<T: std::str::FromStr>(f: &HashMap<String, String>, name: &str, default: T) -> T {
    f.get(name).and_then(|v| v.parse().ok()).unwrap_or(default)
}

fn need(f: &HashMap<String, String>, name: &str) -> String {
    f.get(name)
        .cloned()
        .unwrap_or_else(|| panic!("missing required flag --{name}"))
}

// ---------------------------------------------------------------- the world

struct World {
    map: LandmarkMap,
    radius: f64,
    model: LandmarkModel,
    sensor_pose: Se2,
}

impl World {
    fn load(path: &str, sigma_range: f64, sigma_bearing: f64) -> Self {
        let file = MapFile::load(path).expect("readable landmark map");
        let sensor_pose = Se2::new(LASER_X, 0.0, 0.0);
        let model = LandmarkModel::new(sigma_range, sigma_bearing)
            .and_then(|m| m.with_sensor_pose(sensor_pose))
            .expect("landmark model");
        World {
            map: file.landmark_map(),
            radius: file.nominal_radius(),
            model,
            sensor_pose,
        }
    }

    /// Pillars the detector would report from `pose`, noise-free, as the filter's own
    /// range-bearing model predicts them. Hidden pillars (behind a nearer one) and pillars
    /// out of range are left out.
    fn visible(&self, pose: &Se2) -> Vec<RangeBearing> {
        let sensor = pose.compose(&self.sensor_pose).translation();
        let lms = self.map.landmarks();
        let mut out = Vec::new();
        for (i, l) in lms.iter().enumerate() {
            let to = l.position - sensor;
            let dist = to.norm();
            if dist - self.radius > DETECTOR_MAX_RANGE_M || dist < 0.12 {
                continue;
            }
            let hidden = lms.iter().enumerate().any(|(j, o)| {
                if i == j {
                    return false;
                }
                let along = (o.position - sensor).dot(&to) / dist;
                if along <= 0.0 || along >= dist {
                    return false;
                }
                let perp = ((o.position - sensor) - to * (along / dist)).norm();
                perp < self.radius
            });
            if hidden {
                continue;
            }
            if let Ok(z) = self.model.predict(pose, &l.position) {
                out.push(z);
            }
        }
        out
    }
}

/// Standard normal draw (Box-Muller); `rand` here has no normal distribution of its own.
fn normal<R: Rng + ?Sized>(rng: &mut R) -> f64 {
    let u1: f64 = rng.random::<f64>().max(1e-300);
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
}

// ---------------------------------------------------------------- the route

/// The exploration route from `start`: a square of side [`ROUTE_SIDE_M`] with one corner at
/// the start, driven round and closed back at the start. Of the eight squares that have a
/// corner there (four leg directions relative to the start heading, left or right turn),
/// the first one that stays inside the room and clear of every pillar is used, in the order
/// forward-left, forward-right, left-left, left-right, and so on. `None` if none is clear.
fn exploration_route(start: &Se2, map: &LandmarkMap) -> Option<Vec<Vector2<f64>>> {
    let p0 = start.translation();
    for quarter in 0..4 {
        for sign in [1.0, -1.0] {
            let a = start.theta() + quarter as f64 * PI / 2.0;
            let u = Vector2::new(a.cos(), a.sin());
            let v = Vector2::new(-u.y, u.x) * sign;
            let pts = [
                p0 + u * ROUTE_SIDE_M,
                p0 + u * ROUTE_SIDE_M + v * ROUTE_SIDE_M,
                p0 + v * ROUTE_SIDE_M,
                p0,
            ];
            let inside = pts
                .iter()
                .all(|p| p.x.abs() <= ROUTE_ROOM_LIMIT_M && p.y.abs() <= ROUTE_ROOM_LIMIT_M);
            let mut corners = vec![p0];
            corners.extend_from_slice(&pts);
            let clear = corners.windows(2).all(|seg| {
                map.landmarks().iter().all(|l| {
                    dist_to_segment(&l.position, &seg[0], &seg[1]) >= ROUTE_PILLAR_CLEARANCE_M
                })
            });
            if inside && clear {
                return Some(pts.to_vec());
            }
        }
    }
    None
}

fn dist_to_segment(p: &Vector2<f64>, a: &Vector2<f64>, b: &Vector2<f64>) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(&ab) / ab.norm_squared().max(1e-12)).clamp(0.0, 1.0);
    (p - (a + ab * t)).norm()
}

fn route_yaml(start: &Se2, points: &[Vector2<f64>], laps: usize) -> String {
    let mut s = String::new();
    let _ = writeln!(
        s,
        "# Exploration route for a global localization trial, written by\n\
         # crates/robotics_localization/examples/global_trials.rs (subcommand `route`).\n\
         # Start pose: x={:.4} y={:.4} yaw={:.4}. A square of side {ROUTE_SIDE_M} m with one\n\
         # corner at the start, closed back at the start.",
        start.x(),
        start.y(),
        start.theta()
    );
    let _ = writeln!(
        s,
        "route:\n  name: global-localization-exploration\n\nwaypoints:"
    );
    for _ in 0..laps {
        for p in points {
            let _ = writeln!(s, "  - {{x: {:.4}, y: {:.4}}}", p.x, p.y);
        }
    }
    s
}

// ---------------------------------------------------------------- the simulated robot

/// Kinematic robot following waypoints the way `route_driver` does.
struct Robot {
    truth: Se2,
    waypoints: Vec<Vector2<f64>>,
    next: usize,
    stopped_for: f64,
}

impl Robot {
    const LINEAR: f64 = 0.18;
    const ANGULAR: f64 = 0.8;
    const HEADING_TOLERANCE: f64 = 0.04;
    const POSITION_TOLERANCE: f64 = 0.06;
    const STOP_SECONDS: f64 = 3.0;

    fn step(&mut self, dt: f64) -> bool {
        let Some(&target) = self.waypoints.get(self.next) else {
            self.stopped_for += dt;
            return self.stopped_for < Self::STOP_SECONDS;
        };
        let to = target - self.truth.translation();
        if to.norm() < Self::POSITION_TOLERANCE {
            self.next += 1;
            return true;
        }
        let want = to.y.atan2(to.x);
        let err = angle_diff(want, self.truth.theta());
        if err.abs() > Self::HEADING_TOLERANCE {
            let turn = err.signum() * (Self::ANGULAR * dt).min(err.abs());
            self.truth = Se2::new(
                self.truth.x(),
                self.truth.y(),
                normalize_angle(self.truth.theta() + turn),
            );
        } else {
            let d = (Self::LINEAR * dt).min(to.norm());
            self.truth = Se2::new(
                self.truth.x() + d * self.truth.theta().cos(),
                self.truth.y() + d * self.truth.theta().sin(),
                self.truth.theta(),
            );
        }
        true
    }
}

// ---------------------------------------------------------------- one trial

#[derive(Debug, Default, Clone)]
struct TrialResult {
    start: (f64, f64, f64),
    seed: u64,
    category: &'static str,
    converged: bool,
    conv_time_s: f64,
    conv_scored_updates: u64,
    conv_travel_true_m: f64,
    conv_travel_odom_m: f64,
    err_pos_at_conv: f64,
    err_yaw_at_conv: f64,
    final_err_pos: f64,
    final_err_yaw: f64,
    rmse_pos_after_conv: f64,
    max_err_after_conv: f64,
    particles_before: usize,
    particles_after: usize,
    injections_after_conv: u64,
    scans: u64,
    mean_scan_ms: f64,
    max_scan_ms: f64,
}

struct TrialConfig {
    mcl: MclConfig,
    global: GlobalLocalizationConfig,
    sigma_range: f64,
    sigma_bearing: f64,
    laps: usize,
}

fn run_trial(world: &World, tc: &TrialConfig, start: Se2, seed: u64) -> Option<TrialResult> {
    let route = exploration_route(&start, &world.map)?;
    let mut waypoints = Vec::new();
    for _ in 0..tc.laps {
        waypoints.extend_from_slice(&route);
    }
    let mut rng = SmallRng::seed_from_u64(seed);
    let mut filter =
        McLocalizer::new_uniform_prior(&tc.mcl, &tc.global, world.map.clone(), &mut rng).ok()?;

    let dt = 0.02;
    let steps_per_scan = 10; // 50 Hz odometry, 5 Hz scans, as the simulator publishes them
    let mut robot = Robot {
        truth: start,
        waypoints,
        next: 0,
        stopped_for: 0.0,
    };
    let origin = start.inverse(); // odometry starts at the identity wherever the robot is
    let mut odom_prev = origin.compose(&robot.truth);
    let mut travel_true = 0.0;
    let mut prev_xy = robot.truth.translation();

    let mut r = TrialResult {
        start: (start.x(), start.y(), start.theta()),
        seed,
        ..Default::default()
    };
    let (mut after_sq, mut after_n) = (0.0, 0u64);
    let (mut scan_ms_sum, mut scan_ms_max) = (0.0, 0.0f64);
    let mut predict_ms_since_scan = 0.0;
    let mut step = 0u64;
    loop {
        if !robot.step(dt) {
            break;
        }
        step += 1;
        let odom = origin.compose(&robot.truth);
        let t0 = Instant::now();
        filter.predict(&odom_prev, &odom, &mut rng);
        predict_ms_since_scan += t0.elapsed().as_secs_f64() * 1e3;
        odom_prev = odom;
        travel_true += (robot.truth.translation() - prev_xy).norm();
        prev_xy = robot.truth.translation();

        if step % steps_per_scan != 0 {
            continue;
        }
        let z: Vec<RangeBearing> = world
            .visible(&robot.truth)
            .into_iter()
            .map(|z| RangeBearing {
                range: z.range + tc.sigma_range * normal(&mut rng),
                bearing: normalize_angle(z.bearing + tc.sigma_bearing * normal(&mut rng)),
            })
            .collect();
        let t1 = Instant::now();
        let report = filter.observe(&z, &mut rng);
        // The filter's whole cost per scan: the ten odometry predictions since the last
        // scan plus this scan's update.
        let ms = t1.elapsed().as_secs_f64() * 1e3 + predict_ms_since_scan;
        predict_ms_since_scan = 0.0;
        scan_ms_sum += ms;
        scan_ms_max = scan_ms_max.max(ms);
        r.scans += 1;

        let est = filter.pose();
        let e_pos = est.distance_to(&robot.truth);
        let e_yaw = angle_diff(est.theta(), robot.truth.theta()).abs();
        if report.converged_now {
            let conv = filter.global_status().and_then(|g| g.converged);
            r.converged = true;
            r.conv_time_s = step as f64 * dt;
            r.conv_travel_true_m = travel_true;
            r.err_pos_at_conv = e_pos;
            r.err_yaw_at_conv = e_yaw;
            if let Some(c) = conv {
                r.conv_scored_updates = c.scored_updates;
                r.conv_travel_odom_m = c.travel_m;
                r.particles_before = c.particles_before;
                r.particles_after = c.particles_after;
            }
        }
        if r.converged {
            after_sq += e_pos * e_pos;
            after_n += 1;
            r.max_err_after_conv = r.max_err_after_conv.max(e_pos);
        }
        r.final_err_pos = e_pos;
        r.final_err_yaw = e_yaw;
    }
    r.injections_after_conv = if r.converged {
        filter.stats().injections
    } else {
        0
    };
    r.rmse_pos_after_conv = if after_n > 0 {
        (after_sq / after_n as f64).sqrt()
    } else {
        f64::NAN
    };
    r.mean_scan_ms = if r.scans > 0 {
        scan_ms_sum / r.scans as f64
    } else {
        0.0
    };
    r.max_scan_ms = scan_ms_max;
    let ok = |p: f64, y: f64| p < POSITION_TOLERANCE_M && y < YAW_TOLERANCE_RAD;
    r.category = if !r.converged {
        "NO_CONVERGENCE"
    } else if !ok(r.err_pos_at_conv, r.err_yaw_at_conv) {
        "WRONG_CONVERGENCE"
    } else if !ok(r.final_err_pos, r.final_err_yaw) {
        "LOST_AFTER_CONVERGENCE"
    } else {
        "SUCCESS"
    };
    Some(r)
}

// ---------------------------------------------------------------- kidnap subcommand

#[derive(Debug, Default, Clone)]
struct KidnapResult {
    from: (f64, f64, f64),
    to: (f64, f64, f64),
    seed: u64,
    recovered: bool,
    recovery_time_s: f64,
    final_err_pos: f64,
    final_err_yaw: f64,
    injections: u64,
    before_rmse: f64,
}

/// One kidnapped-robot trial of the tracking filter exactly as it is configured: starts
/// knowing `from`, tracks for `before_s` seconds of the exploration route, is moved to `to`
/// without a word (odometry sees no motion), drives the exploration route from there, and is
/// graded as `scripts/phase3_kidnap_verdict.py` grades a kidnap: recovered once the error is
/// inside both tolerances for ten consecutive scans.
fn run_kidnap_trial(
    world: &World,
    mcl: &MclConfig,
    sigmas: (f64, f64),
    from: Se2,
    to: Se2,
    before_s: f64,
    seed: u64,
) -> Option<KidnapResult> {
    let route_a = exploration_route(&from, &world.map)?;
    let route_b = exploration_route(&to, &world.map)?;
    let mut rng = SmallRng::seed_from_u64(seed);
    let mut filter = McLocalizer::new_at(mcl, world.map.clone(), from, &mut rng).ok()?;
    let dt = 0.02;
    let mut robot = Robot {
        truth: from,
        waypoints: route_a,
        next: 0,
        stopped_for: 0.0,
    };
    let mut odom = Se2::IDENTITY;
    let mut odom_prev = odom;
    let mut prev_truth = robot.truth;
    let mut step = 0u64;
    let mut kidnapped_at: Option<f64> = None;
    let mut streak = 0;
    let mut r = KidnapResult {
        from: (from.x(), from.y(), from.theta()),
        to: (to.x(), to.y(), to.theta()),
        seed,
        ..Default::default()
    };
    let (mut before_sq, mut before_n) = (0.0, 0u64);
    loop {
        let t = step as f64 * dt;
        if kidnapped_at.is_none() && t >= before_s {
            robot = Robot {
                truth: to,
                waypoints: route_b.clone(),
                next: 0,
                stopped_for: 0.0,
            };
            prev_truth = to; // no motion is seen by the odometry
            kidnapped_at = Some(t);
        }
        if !robot.step(dt) {
            break;
        }
        step += 1;
        odom = odom.compose(&prev_truth.between(&robot.truth));
        prev_truth = robot.truth;
        filter.predict(&odom_prev, &odom, &mut rng);
        odom_prev = odom;
        if step % 10 != 0 {
            continue;
        }
        let z: Vec<RangeBearing> = world
            .visible(&robot.truth)
            .into_iter()
            .map(|z| RangeBearing {
                range: z.range + sigmas.0 * normal(&mut rng),
                bearing: normalize_angle(z.bearing + sigmas.1 * normal(&mut rng)),
            })
            .collect();
        filter.observe(&z, &mut rng);
        let est = filter.pose();
        let e_pos = est.distance_to(&robot.truth);
        let e_yaw = angle_diff(est.theta(), robot.truth.theta()).abs();
        r.final_err_pos = e_pos;
        r.final_err_yaw = e_yaw;
        match kidnapped_at {
            None => {
                before_sq += e_pos * e_pos;
                before_n += 1;
            }
            Some(k) => {
                if e_pos < POSITION_TOLERANCE_M && e_yaw < YAW_TOLERANCE_RAD {
                    streak += 1;
                    if streak == 10 && !r.recovered {
                        r.recovered = true;
                        r.recovery_time_s = step as f64 * dt - 9.0 * 0.2 - k;
                    }
                } else {
                    streak = 0;
                }
            }
        }
    }
    r.injections = filter.stats().injections;
    r.before_rmse = if before_n > 0 {
        (before_sq / before_n as f64).sqrt()
    } else {
        f64::NAN
    };
    Some(r)
}

fn kidnap(f: &HashMap<String, String>) {
    let sigma_range: f64 = get(f, "noise-range", 0.05);
    let sigma_bearing: f64 = get(f, "noise-bearing", 0.05);
    let world = World::load(&need(f, "map"), 0.05, 0.05);
    let mcl = MclConfig {
        num_particles: get(f, "steady", 6000),
        recovery_sustained_updates: get(f, "persist", 20),
        sensor_pose: world.sensor_pose,
        ..Default::default()
    };
    let n: usize = get(f, "trials", 40);
    let seed: u64 = get(f, "seed", 1);
    let starts_seed: u64 = get(f, "starts-seed", 3000);
    let before_s: f64 = get(f, "before", 12.0);
    let label = f.get("label").cloned().unwrap_or_else(|| "kidnap".into());
    let out = need(f, "out");
    fs::create_dir_all(&out).expect("output directory");

    // Pairs of start poses at least two metres apart, so the kidnap is a real one.
    let pool = random_starts(&world, 2 * n + 20, starts_seed);
    let mut rows = Vec::new();
    for i in 0..n {
        let (a, b) = (pool[2 * i], pool[2 * i + 1]);
        if a.distance_to(&b) < 2.0 {
            continue;
        }
        if let Some(r) = run_kidnap_trial(
            &world,
            &mcl,
            (sigma_range, sigma_bearing),
            a,
            b,
            before_s,
            seed + i as u64,
        ) {
            eprintln!(
                "[{label}] trial {i:3} ({:5.2},{:5.2}) -> ({:5.2},{:5.2}) {} recovery {:5.1}s final err {:.3}",
                r.from.0, r.from.1, r.to.0, r.to.1,
                if r.recovered { "RECOVERED    " } else { "NOT_RECOVERED" },
                r.recovery_time_s, r.final_err_pos
            );
            rows.push(r);
        }
    }
    let mut csv = String::from(
        "label,trial,from_x,from_y,from_yaw,to_x,to_y,to_yaw,seed,recovered,recovery_time_s,\
         final_err_pos_m,final_err_yaw_rad,injections,rmse_pos_before_kidnap_m\n",
    );
    for (i, r) in rows.iter().enumerate() {
        let _ = writeln!(
            csv,
            "{label},{i},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{},{:.2},{:.4},{:.4},{},{:.4}",
            r.from.0,
            r.from.1,
            r.from.2,
            r.to.0,
            r.to.1,
            r.to.2,
            r.seed,
            u8::from(r.recovered),
            r.recovery_time_s,
            r.final_err_pos,
            r.final_err_yaw,
            r.injections,
            r.before_rmse
        );
    }
    fs::write(Path::new(&out).join("kidnap_trials.csv"), csv).expect("write kidnap_trials.csv");
    let total = rows.len();
    let rec = rows.iter().filter(|r| r.recovered).count();
    let (lo, hi) = wilson(rec, total);
    let mut times: Vec<f64> = rows
        .iter()
        .filter(|r| r.recovered)
        .map(|r| r.recovery_time_s)
        .collect();
    let mut y = String::new();
    let _ = writeln!(y, "# Summary of {out}/kidnap_trials.csv, written by global_trials.rs; no number here is typed.");
    let _ = writeln!(y, "label: {label}");
    let _ = writeln!(y, "config:");
    let _ = writeln!(y, "  steady_state_particles: {}", mcl.num_particles);
    let _ = writeln!(
        y,
        "  recovery_sustained_updates: {}",
        mcl.recovery_sustained_updates
    );
    let _ = writeln!(
        y,
        "  recovery_injection_fraction: {}",
        mcl.recovery_injection_fraction
    );
    let _ = writeln!(y, "  tracked_before_kidnap_s: {before_s}");
    let _ = writeln!(y, "  detection_noise_range_m: {sigma_range}");
    let _ = writeln!(y, "  detection_noise_bearing_rad: {sigma_bearing}");
    let _ = writeln!(y, "  seed: {seed}");
    let _ = writeln!(y, "  starts_seed: {starts_seed}");
    let _ = writeln!(y, "trials: {total}");
    let _ = writeln!(y, "recovered: {rec}");
    let _ = writeln!(y, "recovery_rate: {:.4}", rec as f64 / total.max(1) as f64);
    let _ = writeln!(y, "recovery_rate_wilson95: [{lo:.4}, {hi:.4}]");
    let _ = writeln!(y, "recovery_time_s: {{median: {:.2}}}", median(&mut times));
    fs::write(Path::new(&out).join("summary.yaml"), y).expect("write summary.yaml");
    eprintln!("[{label}] {rec}/{total} recovered, summary in {out}/summary.yaml");
}

// ---------------------------------------------------------------- trials subcommand

fn read_starts(path: &str) -> Vec<Se2> {
    fs::read_to_string(path)
        .expect("readable starts file")
        .lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#') && !l.starts_with("x,"))
        .map(|l| {
            let v: Vec<f64> = l.split(',').filter_map(|t| t.trim().parse().ok()).collect();
            Se2::new(v[0], v[1], v[2])
        })
        .collect()
}

/// `n` start poses drawn uniformly over the free space whose exploration route is clear, in
/// the order the generator produces them, so that a prefix of a longer list is the shorter
/// list: the first ten of `--trials 100` are the ten of `--trials 10`.
fn random_starts(world: &World, n: usize, seed: u64) -> Vec<Se2> {
    let mut rng = SmallRng::seed_from_u64(seed);
    let mut v = Vec::new();
    while v.len() < n {
        let p = sample_free_pose(&world.map, 2.8, world.radius + ROBOT_FOOTPRINT_M, &mut rng)
            .expect("free pose");
        if exploration_route(&p, &world.map).is_some() {
            v.push(p);
        }
    }
    v
}

fn wilson(successes: usize, n: usize) -> (f64, f64) {
    if n == 0 {
        return (f64::NAN, f64::NAN);
    }
    let z = 1.96;
    let (n_f, p) = (n as f64, successes as f64 / n as f64);
    let denom = 1.0 + z * z / n_f;
    let centre = (p + z * z / (2.0 * n_f)) / denom;
    let half = z * (p * (1.0 - p) / n_f + z * z / (4.0 * n_f * n_f)).sqrt() / denom;
    ((centre - half).max(0.0), (centre + half).min(1.0))
}

fn median(v: &mut [f64]) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn trials(f: &HashMap<String, String>) {
    let sigma_range: f64 = get(f, "noise-range", 0.05);
    let sigma_bearing: f64 = get(f, "noise-bearing", 0.05);
    let world = World::load(&need(f, "map"), 0.05, 0.05);
    let global = GlobalLocalizationConfig {
        num_particles: get(f, "particles", 6000),
        landmark_keepout_m: world.radius + ROBOT_FOOTPRINT_M,
        convergence_sigma_xy_m: get(f, "sigma-xy", 0.05),
        convergence_sigma_theta_rad: get(f, "sigma-theta", 0.05),
        convergence_sustained_scans: get(f, "sustained", 10),
        convergence_min_explained_fraction: get(f, "explained-fraction", 0.5),
    };
    let mcl = MclConfig {
        num_particles: get(f, "steady", 6000),
        sensor_pose: world.sensor_pose,
        ..Default::default()
    };
    let tc = TrialConfig {
        mcl,
        global,
        sigma_range,
        sigma_bearing,
        laps: get(f, "laps", 1),
    };
    let seed: u64 = get(f, "seed", 1);
    let starts_seed: u64 = get(f, "starts-seed", 1000);
    let label = f.get("label").cloned().unwrap_or_else(|| "trials".into());
    let out = need(f, "out");
    fs::create_dir_all(&out).expect("output directory");

    let starts: Vec<Se2> = match f.get("starts").map(String::as_str) {
        None | Some("random") => random_starts(&world, get(f, "trials", 20), starts_seed),
        Some(path) => read_starts(path),
    };

    let mut rows = Vec::new();
    let mut skipped = 0;
    for (i, start) in starts.iter().enumerate() {
        match run_trial(&world, &tc, *start, seed + i as u64) {
            Some(r) => {
                eprintln!(
                    "[{label}] trial {:3} start ({:6.2},{:6.2},{:6.2}) -> {:22} conv {:6.1}s err {:.3}/{:.3}",
                    i, r.start.0, r.start.1, r.start.2, r.category, r.conv_time_s,
                    r.err_pos_at_conv, r.final_err_pos
                );
                rows.push(r);
            }
            None => skipped += 1,
        }
    }

    let mut csv = String::from(
        "label,trial,start_x,start_y,start_yaw,seed,category,converged,conv_time_s,\
         conv_scored_updates,conv_travel_true_m,conv_travel_odom_m,err_pos_at_conv_m,\
         err_yaw_at_conv_rad,final_err_pos_m,final_err_yaw_rad,rmse_pos_after_conv_m,\
         max_err_after_conv_m,particles_before,particles_after,injections_after_conv,scans,\
         mean_scan_ms,max_scan_ms\n",
    );
    for (i, r) in rows.iter().enumerate() {
        let _ = writeln!(
            csv,
            "{label},{i},{:.4},{:.4},{:.4},{},{},{},{:.2},{},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{},{},{},{:.3},{:.3}",
            r.start.0, r.start.1, r.start.2, r.seed, r.category, u8::from(r.converged),
            r.conv_time_s, r.conv_scored_updates, r.conv_travel_true_m, r.conv_travel_odom_m,
            r.err_pos_at_conv, r.err_yaw_at_conv, r.final_err_pos, r.final_err_yaw,
            r.rmse_pos_after_conv, r.max_err_after_conv, r.particles_before, r.particles_after,
            r.injections_after_conv, r.scans, r.mean_scan_ms, r.max_scan_ms
        );
    }
    fs::write(Path::new(&out).join("trials.csv"), csv).expect("write trials.csv");

    let n = rows.len();
    let count = |c: &str| rows.iter().filter(|r| r.category == c).count();
    let succ = count("SUCCESS");
    let (lo, hi) = wilson(succ, n);
    let mut t: Vec<f64> = rows
        .iter()
        .filter(|r| r.category == "SUCCESS")
        .map(|r| r.conv_time_s)
        .collect();
    let mut d: Vec<f64> = rows
        .iter()
        .filter(|r| r.category == "SUCCESS")
        .map(|r| r.conv_travel_true_m)
        .collect();
    let mut rm: Vec<f64> = rows
        .iter()
        .filter(|r| r.category == "SUCCESS")
        .map(|r| r.rmse_pos_after_conv)
        .collect();
    let mean = |v: &[f64]| {
        if v.is_empty() {
            f64::NAN
        } else {
            v.iter().sum::<f64>() / v.len() as f64
        }
    };
    let mut y = String::new();
    let _ = writeln!(
        y,
        "# Summary of {out}/trials.csv, written by global_trials.rs; no number here is typed."
    );
    let _ = writeln!(y, "label: {label}");
    let _ = writeln!(y, "config:");
    let _ = writeln!(y, "  global_particles: {}", tc.global.num_particles);
    let _ = writeln!(y, "  steady_state_particles: {}", tc.mcl.num_particles);
    let _ = writeln!(
        y,
        "  convergence_sigma_xy_m: {}",
        tc.global.convergence_sigma_xy_m
    );
    let _ = writeln!(
        y,
        "  convergence_sigma_theta_rad: {}",
        tc.global.convergence_sigma_theta_rad
    );
    let _ = writeln!(
        y,
        "  convergence_sustained_scans: {}",
        tc.global.convergence_sustained_scans
    );
    let _ = writeln!(
        y,
        "  convergence_min_explained_fraction: {}",
        tc.global.convergence_min_explained_fraction
    );
    let _ = writeln!(y, "  detection_noise_range_m: {sigma_range}");
    let _ = writeln!(y, "  detection_noise_bearing_rad: {sigma_bearing}");
    let _ = writeln!(y, "  laps: {}", tc.laps);
    let _ = writeln!(y, "  seed: {seed}");
    let _ = writeln!(y, "  starts_seed: {starts_seed}");
    let _ = writeln!(y, "  success_position_tolerance_m: {POSITION_TOLERANCE_M}");
    let _ = writeln!(y, "  success_yaw_tolerance_rad: {YAW_TOLERANCE_RAD}");
    let _ = writeln!(y, "trials: {n}");
    let _ = writeln!(y, "skipped_no_feasible_route: {skipped}");
    let _ = writeln!(y, "outcomes:");
    for c in [
        "SUCCESS",
        "WRONG_CONVERGENCE",
        "LOST_AFTER_CONVERGENCE",
        "NO_CONVERGENCE",
    ] {
        let _ = writeln!(y, "  {c}: {}", count(c));
    }
    let _ = writeln!(y, "success_rate: {:.4}", succ as f64 / n.max(1) as f64);
    let _ = writeln!(y, "success_rate_wilson95: [{lo:.4}, {hi:.4}]");
    let _ = writeln!(y, "among_successes:");
    let _ = writeln!(
        y,
        "  convergence_time_s: {{mean: {:.2}, median: {:.2}}}",
        mean(&t),
        median(&mut t)
    );
    let _ = writeln!(
        y,
        "  convergence_distance_m: {{mean: {:.3}, median: {:.3}}}",
        mean(&d),
        median(&mut d)
    );
    let _ = writeln!(
        y,
        "  rmse_position_after_convergence_m: {{mean: {:.4}, median: {:.4}}}",
        mean(&rm),
        median(&mut rm)
    );
    let _ = writeln!(y, "cost:");
    let _ = writeln!(
        y,
        "  mean_scan_ms: {:.3}",
        mean(&rows.iter().map(|r| r.mean_scan_ms).collect::<Vec<_>>())
    );
    let _ = writeln!(
        y,
        "  max_scan_ms: {:.3}",
        rows.iter().map(|r| r.max_scan_ms).fold(0.0, f64::max)
    );
    fs::write(Path::new(&out).join("summary.yaml"), y).expect("write summary.yaml");
    eprintln!("[{label}] {succ}/{n} success, summary in {out}/summary.yaml");
}

// ---------------------------------------------------------------- ambiguity subcommand

/// Union-find over grid cells.
struct Dsu(Vec<usize>);
impl Dsu {
    fn find(&mut self, i: usize) -> usize {
        let mut r = i;
        while self.0[r] != r {
            r = self.0[r];
        }
        let mut c = i;
        while self.0[c] != r {
            let n = self.0[c];
            self.0[c] = r;
            c = n;
        }
        r
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.0[ra] = rb;
        }
    }
}

/// How many separate regions of pose space explain every detection of one static noise-free
/// scan from `start`, under the filter's own gate; and how wide the widest region is.
fn static_modes(world: &World, start: &Se2, gate: f64) -> (usize, usize, f64) {
    let z = world.visible(start);
    if z.is_empty() {
        return (0, 0, f64::NAN);
    }
    let (res, half) = (0.1_f64, 2.8_f64);
    let nth = 72usize;
    let nxy = (2.0 * half / res).round() as usize + 1;
    let idx = |ix: usize, iy: usize, it: usize| (ix * nxy + iy) * nth + it;
    let mut ok = vec![false; nxy * nxy * nth];
    for ix in 0..nxy {
        for iy in 0..nxy {
            for it in 0..nth {
                let pose = Se2::new(
                    -half + ix as f64 * res,
                    -half + iy as f64 * res,
                    -PI + it as f64 * 2.0 * PI / nth as f64,
                );
                ok[idx(ix, iy, it)] = z.iter().all(|zi| {
                    world.map.landmarks().iter().any(|l| {
                        world
                            .model
                            .log_likelihood(&pose, &l.position, zi)
                            .map(|ll| ll >= gate)
                            .unwrap_or(false)
                    })
                });
            }
        }
    }
    let mut dsu = Dsu((0..ok.len()).collect());
    for ix in 0..nxy {
        for iy in 0..nxy {
            for it in 0..nth {
                if !ok[idx(ix, iy, it)] {
                    continue;
                }
                let here = idx(ix, iy, it);
                for (dx, dy, dt) in [(1, 0, 0), (0, 1, 0), (0, 0, 1)] {
                    let (jx, jy, jt) = (ix + dx, iy + dy, (it + dt) % nth);
                    if jx < nxy && jy < nxy && ok[idx(jx, jy, jt)] {
                        dsu.union(here, idx(jx, jy, jt));
                    }
                }
            }
        }
    }
    let mut comps: HashMap<usize, (f64, f64, f64, f64)> = HashMap::new();
    for ix in 0..nxy {
        for iy in 0..nxy {
            for it in 0..nth {
                if ok[idx(ix, iy, it)] {
                    let root = dsu.find(idx(ix, iy, it));
                    let (x, y) = (-half + ix as f64 * res, -half + iy as f64 * res);
                    let e = comps.entry(root).or_insert((x, x, y, y));
                    e.0 = e.0.min(x);
                    e.1 = e.1.max(x);
                    e.2 = e.2.min(y);
                    e.3 = e.3.max(y);
                }
            }
        }
    }
    let widest = comps
        .values()
        .map(|c| ((c.1 - c.0).powi(2) + (c.3 - c.2).powi(2)).sqrt())
        .fold(0.0, f64::max);
    (comps.len(), z.len(), widest)
}

fn ambiguity(f: &HashMap<String, String>) {
    let world = World::load(&need(f, "map"), 0.05, 0.05);
    let out = need(f, "out");
    // The filter's own gate: ln(peak) - chi2(2, 0.99)/2, see McLocalizer::fit_log_threshold,
    // for the default measurement sigmas of 0.05 m and 0.05 rad.
    let gate = -(MODEL_SIGMA * MODEL_SIGMA).ln() - (2.0 * PI).ln() - 0.5 * 9.21034;
    let step: f64 = get(f, "step", 0.5);
    let mut rows = String::from("x,y,yaw,visible_pillars,modes,widest_mode_m\n");
    let mut n = 0;
    let mut x = -2.5;
    while x <= 2.5 + 1e-9 {
        let mut y = -2.5;
        while y <= 2.5 + 1e-9 {
            for k in 0..4 {
                let yaw = normalize_angle(k as f64 * PI / 2.0);
                let p = Se2::new(x, y, yaw);
                let near = world.map.landmarks().iter().any(|l| {
                    (l.position.x - x).hypot(l.position.y - y) < world.radius + ROBOT_FOOTPRINT_M
                });
                if near {
                    continue;
                }
                let (modes, visible, widest) = static_modes(&world, &p, gate);
                let _ = writeln!(rows, "{x:.2},{y:.2},{yaw:.4},{visible},{modes},{widest:.3}");
                n += 1;
            }
            y += step;
        }
        x += step;
    }
    fs::write(&out, rows).expect("write ambiguity csv");
    eprintln!("ambiguity: {n} start poses ranked into {out}");
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(cmd) = args.first() else {
        eprintln!(
            "usage: global_trials <route|starts|ambiguity|trials> --map <landmarks.yaml> ..."
        );
        std::process::exit(2);
    };
    let f = flags(&args[1..]);
    match cmd.as_str() {
        "route" => {
            let world = World::load(&need(&f, "map"), 0.05, 0.05);
            let start = Se2::new(get(&f, "x", 0.0), get(&f, "y", 0.0), get(&f, "yaw", 0.0));
            match exploration_route(&start, &world.map) {
                Some(pts) => print!("{}", route_yaml(&start, &pts, get(&f, "laps", 1))),
                None => {
                    eprintln!("no clear exploration route from this start pose");
                    std::process::exit(3);
                }
            }
        }
        "starts" => {
            // Same generator as `trials --starts random`, printed as a starts file.
            let world = World::load(&need(&f, "map"), 0.05, 0.05);
            println!("x,y,yaw");
            for p in random_starts(&world, get(&f, "n", 20), get(&f, "starts-seed", 1000)) {
                println!("{:.4},{:.4},{:.4}", p.x(), p.y(), p.theta());
            }
        }
        "kidnap" => kidnap(&f),
        "ambiguity" => ambiguity(&f),
        "trials" => trials(&f),
        other => {
            eprintln!("unknown subcommand {other}");
            std::process::exit(2);
        }
    }
}
