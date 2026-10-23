//! Chapter 4 bindings: EKF localization (see web/design/Chapter-04.md).
//!
//! Everything that estimates is `robotics_localization` (`pillars::detect`,
//! `EkfLocalizer::{predict, observe}`) fed by `robotics_sim` (`World`, `Lidar`, `RustySim`)
//! and `robotics_models::WheelOdometry`, in the order and at the rates
//! `ros2_ws/src/robotics_localization_bridge/src/localization_node.rs` uses them: a
//! prediction for every 50 Hz wheel sample that moved, a detection and a correction for every
//! 5 Hz scan, the node's default `LocalizerConfig`, `OdometryMotion` and `PillarDetector`.
//!
//! The *world* is the repository's own: the four pillars of
//! `sim/webots/config/landmarks.yaml` and the route of `sim/webots/config/route_long.yaml`
//! are `include_str!`ed and parsed here (`MapFile::parse` is the node's parser), so the page
//! cannot drift from the files the robot reads.
//!
//! Three pieces of code in this file are **not** crate algorithms, and each says so where it
//! lives; none of them is code the robot runs:
//!
//! 1. [`Follower`], a restatement of the control law of `route_driver.rs` (turn in place at
//!    0.8 rad/s, drive at 0.18 m/s with a proportional heading term, same gains), because
//!    that node is an rclrs process and cannot be compiled to WebAssembly. It is the
//!    harness that makes the robot go somewhere, not the subject of the chapter.
//! 2. [`build_world`], which draws each round pillar as a regular 24-gon of the pillar's
//!    radius, because Rusty's `World` holds line segments only. The detector then sees what a
//!    ray caster sees of a pillar, with a facet error of under a millimetre (the checks assert
//!    it), not a perfect circle.
//! 3. The *kidnap*, which moves the simulated robot the way the Webots supervisor does: the
//!    simulator keeps integrating the wheels from where it was, and a rigid transform applied
//!    once to the truth carries the robot elsewhere without telling the estimator or the
//!    odometry.
//!
//! **What this is not.** It is Rusty, not Webots. Rusty's odometry is deliberately
//! uncalibrated (2 % wheel slip per step, a 0.2 % right-wheel radius error, the Chapter 2
//! values), so its dead reckoning drifts by metres; the Webots reference odometry was
//! calibrated upstream against this world's slip and drifts by centimetres
//! (`docs/phase-3-localization/theory.md` section 6). The chapter says which is which beside
//! every number.
//!
//! Every stochastic function takes `seed: u64`. As in Chapters 2 and 3, a seed names a
//! different stream on WebAssembly than on the robot (`ch02_seeded_rng_probe`), so the page's
//! draws are not the native runs; `cargo test` pins the native realization and
//! `web/checks/ch04.ts` the module's.

use robotics_core::nalgebra::{Matrix3, Vector2};
use robotics_core::{Manifold, Se2, Timestamp, Twist2};
use robotics_filters::consistency::{nees, ConsistencyStats};
use robotics_localization::{
    detect, EkfLocalizer, LocalizerConfig, MapFile, OdometryMotion, PillarDetector, UpdateReport,
};
use robotics_models::sensor::landmark::RangeBearing;
use robotics_models::{DiffDriveParams, OdometryNoise, WheelOdometry};
use robotics_sim::{Lidar, LidarConfig, RustyConfig, RustySim, Segment, Trajectory, World};
use std::f64::consts::PI;
use wasm_bindgen::prelude::*;

const LANDMARKS_YAML: &str = include_str!("../../../sim/webots/config/landmarks.yaml");
const ROUTE_YAML: &str = include_str!("../../../sim/webots/config/route_long.yaml");

/// Where the laser sits on the base (m), as the simulated robot reports it through TF and
/// as `crates/robotics_localization/examples/global_trials.rs` assumes.
const LASER_X: f64 = -0.03;
/// Inner wall faces of the room are at ±3.0 m (`landmarks.yaml`, `floor_size: [6.0, 6.0]`).
const ROOM_HALF: f64 = 3.0;
/// Sides of the polygon that stands in for a round pillar.
const PILLAR_SIDES: usize = 24;
/// The Webots LDS-01 as `docs/phase-3-localization/theory.md` section 6 describes it:
/// 360 beams, 0.12 m to 3.5 m, about 15 mm of range noise, 5 Hz.
const LDS_RANGE_MAX: f64 = 3.5;
const LDS_SIGMA: f64 = 0.015;
const SCAN_HZ: f64 = 5.0;
const PHYSICS_DT: f64 = 0.005;
/// Physics steps per wheel sample (50 Hz), per scan (5 Hz) and per control tick (20 Hz, the
/// `rate_hz` default of `route_driver`).
const WHEEL_EVERY: u64 = 4;
const SCAN_EVERY: u64 = 40;
const CONTROL_EVERY: u64 = 10;
/// The simulation ends here if the route has not (a stuck leg never ends a real run either).
const MAX_SECONDS: f64 = 900.0;

fn err<E: std::fmt::Display>(e: E) -> String {
    e.to_string()
}

fn map_file() -> Result<MapFile, String> {
    MapFile::parse(LANDMARKS_YAML).map_err(err)
}

/// The waypoints of `route_long.yaml`, in order. A deliberately small reader for the exact
/// shape of that file: `- {x: .., y: ..}` lines under `waypoints:`.
fn route_waypoints() -> Result<Vec<(f64, f64)>, String> {
    let mut out = Vec::new();
    let mut in_waypoints = false;
    for raw in ROUTE_YAML.lines() {
        let line = raw.split('#').next().unwrap_or("");
        if line.trim().is_empty() {
            continue;
        }
        if !line.starts_with(' ') {
            in_waypoints = line.trim() == "waypoints:";
            continue;
        }
        if !in_waypoints {
            continue;
        }
        let body = line
            .trim()
            .strip_prefix("- {")
            .and_then(|s| s.strip_suffix('}'))
            .ok_or_else(|| format!("route line not understood: {line:?}"))?;
        let (mut x, mut y) = (None, None);
        for pair in body.split(',') {
            let (k, v) = pair
                .split_once(':')
                .ok_or_else(|| format!("route pair not understood: {pair:?}"))?;
            let v: f64 = v.trim().parse().map_err(err)?;
            match k.trim() {
                "x" => x = Some(v),
                "y" => y = Some(v),
                other => return Err(format!("unknown route key {other:?}")),
            }
        }
        out.push((
            x.ok_or("waypoint without x")?,
            y.ok_or("waypoint without y")?,
        ));
    }
    if out.is_empty() {
        return Err("the route file lists no waypoints".into());
    }
    Ok(out)
}

/// **Not crate code** (see the module docs): the room and the pillars as Rusty segments.
fn build_world() -> Result<World, String> {
    let mf = map_file()?;
    let mut world = World::empty();
    world
        .add_rectangle(
            Vector2::new(-ROOM_HALF, -ROOM_HALF),
            Vector2::new(ROOM_HALF, ROOM_HALF),
        )
        .map_err(err)?;
    for l in &mf.landmarks {
        let ring: Vec<Vector2<f64>> = (0..PILLAR_SIDES)
            .map(|k| {
                let a = 2.0 * PI * k as f64 / PILLAR_SIDES as f64;
                Vector2::new(l.x + l.radius * a.cos(), l.y + l.radius * a.sin())
            })
            .collect();
        world.add_polyline(&ring, true).map_err(err)?;
    }
    Ok(world)
}

fn laser_mount() -> Se2 {
    Se2::new(LASER_X, 0.0, 0.0)
}

fn lds_config(range_sigma: f64) -> LidarConfig {
    LidarConfig {
        mount: laser_mount(),
        range_min: 0.12,
        range_max: LDS_RANGE_MAX,
        rate_hz: SCAN_HZ,
        range_sigma,
        dropout_probability: 0.0,
        ..LidarConfig::default()
    }
}

/// The SplitMix64 finalizer, used to derive the lidar's stream from the master seed so that
/// it is independent of the wheel-slip stream (as `RustySim` derives its own).
fn stream_seed(seed: u64, stream: u64) -> u64 {
    let mut z = seed ^ stream.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// The node's detector and filter configuration, with the laser mount of this world.
fn localizer_config(sigma_range: f64, sigma_bearing: f64) -> LocalizerConfig {
    LocalizerConfig {
        sigma_range,
        sigma_bearing,
        sensor_pose: laser_mount(),
        ..LocalizerConfig::default()
    }
}

// ---------------------------------------------------------------------------------------
// The map, the route, the defaults
// ---------------------------------------------------------------------------------------

/// The pillars of `landmarks.yaml` as flat `[x, y, radius]` triples, in file order (the
/// landmark id is the index).
#[wasm_bindgen]
pub fn ch04_landmarks() -> Result<Vec<f64>, String> {
    Ok(map_file()?
        .landmarks
        .iter()
        .flat_map(|l| [l.x, l.y, l.radius])
        .collect())
}

/// The waypoints of `route_long.yaml` as flat `[x, y]` pairs.
#[wasm_bindgen]
pub fn ch04_route() -> Result<Vec<f64>, String> {
    Ok(route_waypoints()?
        .iter()
        .flat_map(|&(x, y)| [x, y])
        .collect())
}

/// Half the side of the room (m).
#[wasm_bindgen]
pub fn ch04_room_half() -> f64 {
    ROOM_HALF
}

/// The pose the robot is spawned at, `[x, y, θ]`.
#[wasm_bindgen]
pub fn ch04_spawn_pose() -> Result<Vec<f64>, String> {
    let p = map_file()?.spawn_pose.ok_or("landmarks.yaml has no spawn pose")?;
    Ok(vec![p.x(), p.y(), p.theta()])
}

/// The defaults of the detector the node runs, `[max_range, cluster_gap, min_points,
/// max_chord, radius]`.
#[wasm_bindgen]
pub fn ch04_detector_defaults() -> Vec<f64> {
    let d = PillarDetector::default();
    vec![d.max_range, d.cluster_gap, d.min_points as f64, d.max_chord, d.radius]
}

/// The defaults of the filter the node runs, `[sigma_range, sigma_bearing, gate_probability,
/// initial_sigma_xy, initial_sigma_theta, gate]`, where `gate` is the χ²(2) quantile.
#[wasm_bindgen]
pub fn ch04_filter_defaults() -> Result<Vec<f64>, String> {
    let c = LocalizerConfig::default();
    let f = EkfLocalizer::new(&c, map_file()?.landmark_map(), Se2::IDENTITY).map_err(err)?;
    Ok(vec![
        c.sigma_range,
        c.sigma_bearing,
        c.gate_probability,
        c.initial_sigma_xy,
        c.initial_sigma_theta,
        f.gate(),
    ])
}

// ---------------------------------------------------------------------------------------
// One scan and the detector
// ---------------------------------------------------------------------------------------

/// One scan from a robot at a pose, and what `pillars::detect` made of it.
#[wasm_bindgen]
pub struct Ch04Scan {
    sensor: Vec<f64>,
    ranges: Vec<f64>,
    angle_min: f64,
    angle_increment: f64,
    detections: Vec<f64>,
}

/// Number of values per detection in [`Ch04Scan::detections`].
pub const DETECTION_STRIDE: usize = 9;

#[wasm_bindgen]
impl Ch04Scan {
    /// The laser's pose in the world, `[x, y, θ]`.
    #[wasm_bindgen(getter)]
    pub fn sensor(&self) -> Vec<f64> {
        self.sensor.clone()
    }
    /// One range per beam (m), `NaN` where the beam returned nothing.
    #[wasm_bindgen(getter)]
    pub fn ranges(&self) -> Vec<f64> {
        self.ranges.clone()
    }
    /// Bearing of beam 0 in the sensor frame (rad).
    #[wasm_bindgen(getter)]
    pub fn angle_min(&self) -> f64 {
        self.angle_min
    }
    /// Angle between beams (rad).
    #[wasm_bindgen(getter)]
    pub fn angle_increment(&self) -> f64 {
        self.angle_increment
    }
    /// Nine values per detection: `range`, `bearing` (to the pillar *centre*, sensor
    /// frame), `beams` in the cluster, `chord` (m), the centre's world `x` and `y`, the id of
    /// the nearest map landmark to that centre, its distance (m) and the true range from the
    /// sensor to that landmark's centre (m).
    #[wasm_bindgen(getter)]
    pub fn detections(&self) -> Vec<f64> {
        self.detections.clone()
    }
}

fn nearest_landmark(mf: &MapFile, p: &Vector2<f64>) -> (usize, f64) {
    mf.landmarks
        .iter()
        .enumerate()
        .map(|(i, l)| (i, (Vector2::new(l.x, l.y) - p).norm()))
        .fold((0, f64::INFINITY), |best, c| if c.1 < best.1 { c } else { best })
}

/// World position of a range-bearing observation made by a sensor at `sensor`.
fn world_point(sensor: &Se2, z: &RangeBearing) -> Vector2<f64> {
    sensor.act(&Vector2::new(z.range * z.bearing.cos(), z.range * z.bearing.sin()))
}

/// One scan of the room from the robot pose `(x, y, theta)` with range noise `range_sigma`,
/// run through the pillar detector exactly as the localization node runs it.
#[wasm_bindgen]
pub fn ch04_scan(x: f64, y: f64, theta: f64, range_sigma: f64, seed: u64) -> Result<Ch04Scan, String> {
    let mf = map_file()?;
    let world = build_world()?;
    let mut lidar = Lidar::new(lds_config(range_sigma), stream_seed(seed, 4)).map_err(err)?;
    let body = Se2::new(x, y, theta);
    let scan = lidar.scan(&world, &body, Timestamp::from_secs_f64(0.0));
    let sensor = lidar.sensor_pose(&body);
    let mut detections = Vec::new();
    for d in detect(&scan, &PillarDetector::default()) {
        let w = world_point(&sensor, &d.z);
        let (id, dist) = nearest_landmark(&mf, &w);
        let l = &mf.landmarks[id];
        let true_range = (Vector2::new(l.x, l.y) - sensor.translation()).norm();
        detections.extend_from_slice(&[
            d.z.range,
            d.z.bearing,
            d.beams as f64,
            d.chord,
            w.x,
            w.y,
            id as f64,
            dist,
            true_range,
        ]);
    }
    Ok(Ch04Scan {
        sensor: vec![sensor.x(), sensor.y(), sensor.theta()],
        ranges: scan
            .ranges
            .iter()
            .map(|&r| if r.is_finite() { r as f64 } else { f64::NAN })
            .collect(),
        angle_min: scan.angle_min,
        angle_increment: scan.angle_increment,
        detections,
    })
}

// ---------------------------------------------------------------------------------------
// Data association and the gate
// ---------------------------------------------------------------------------------------

/// What the filter does with one scan when its belief is wrong by a known amount.
#[wasm_bindgen]
pub struct Ch04Assoc {
    per_detection: Vec<f64>,
    before: Vec<f64>,
    after: Vec<f64>,
    gate: f64,
}

/// Number of values per detection in [`Ch04Assoc::per_detection`].
pub const ASSOC_STRIDE: usize = 6;

#[wasm_bindgen]
impl Ch04Assoc {
    /// Six values per detection, each judged *alone against the belief the scan starts
    /// from* (the real update is sequential, see [`Ch04Assoc::after`]): `accepted` (1 or 0),
    /// the landmark id it was associated with (−1 if rejected), its NIS (`NaN` if rejected),
    /// the id of the landmark it truly is (nearest to its world position from the *true*
    /// pose), and its world position under the *belief* `x`, `y` (where the filter thinks the
    /// pillar is).
    #[wasm_bindgen(getter)]
    pub fn per_detection(&self) -> Vec<f64> {
        self.per_detection.clone()
    }
    /// The belief before the scan, `[x, y, θ, position error (m), yaw error (rad)]`.
    #[wasm_bindgen(getter)]
    pub fn before(&self) -> Vec<f64> {
        self.before.clone()
    }
    /// The belief after `observe` took the whole scan, `[x, y, θ, position error (m), yaw
    /// error (rad), accepted, rejected, mean NIS (NaN if none)]`.
    #[wasm_bindgen(getter)]
    pub fn after(&self) -> Vec<f64> {
        self.after.clone()
    }
    /// The χ²(2) gate in use.
    #[wasm_bindgen(getter)]
    pub fn gate(&self) -> f64 {
        self.gate
    }
}

fn wrap(a: f64) -> f64 {
    robotics_core::angle::angle_diff(a, 0.0)
}

/// Scan from the true pose `(x, y, theta)` and hand it to an EKF whose belief is the truth
/// moved by `(dx, dy)` metres (world frame) and `dtheta` radians, with the stated
/// uncertainty. `gate_probability` is `LocalizerConfig::gate_probability`.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn ch04_associate(
    x: f64,
    y: f64,
    theta: f64,
    dx: f64,
    dy: f64,
    dtheta: f64,
    sigma_xy: f64,
    sigma_theta: f64,
    gate_probability: f64,
    seed: u64,
) -> Result<Ch04Assoc, String> {
    let mf = map_file()?;
    let truth = Se2::new(x, y, theta);
    let scan = ch04_scan(x, y, theta, LDS_SIGMA, seed)?;
    let belief = Se2::new(x + dx, y + dy, theta + dtheta);
    let cfg = LocalizerConfig {
        initial_sigma_xy: sigma_xy,
        initial_sigma_theta: sigma_theta,
        gate_probability,
        ..localizer_config(0.05, 0.05)
    };
    let loc = EkfLocalizer::new(&cfg, mf.landmark_map(), belief).map_err(err)?;
    let believed_sensor = belief.compose(&laser_mount());

    let mut obs = Vec::new();
    let mut per = Vec::new();
    for d in scan.detections.chunks(DETECTION_STRIDE) {
        let z = RangeBearing {
            range: d[0],
            bearing: d[1],
        };
        let mut one = loc.clone();
        let r = one.observe(&[z]);
        let (accepted, id, nis) = match r.associations.first() {
            Some(&(id, nis)) if r.accepted == 1 => (1.0, id as f64, nis),
            _ => (0.0, -1.0, f64::NAN),
        };
        let believed = world_point(&believed_sensor, &z);
        per.extend_from_slice(&[accepted, id, nis, d[6], believed.x, believed.y]);
        obs.push(z);
    }
    let pose_row = |p: &Se2| {
        vec![
            p.x(),
            p.y(),
            p.theta(),
            p.distance_to(&truth),
            wrap(p.theta() - truth.theta()),
        ]
    };
    let before = pose_row(&belief);
    let mut full = loc.clone();
    let report: UpdateReport = full.observe(&obs);
    let mut after = pose_row(&full.pose());
    after.extend_from_slice(&[
        report.accepted as f64,
        report.rejected as f64,
        report.mean_nis.unwrap_or(f64::NAN),
    ]);
    Ok(Ch04Assoc {
        per_detection: per,
        before,
        after,
        gate: loc.gate(),
    })
}

// ---------------------------------------------------------------------------------------
// The route follower (not crate code)
// ---------------------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Leg {
    Turn,
    Drive,
    Finished,
}

/// **Not crate code.** The control law of `route_driver.rs`, with its default gains: turn in
/// place until the heading error is under 0.04 rad, then drive at 0.18 m/s steering with a
/// proportional term (gain 1.2 1/s, capped at 0.3 rad/s) until within 0.06 m of the
/// waypoint; fall back to turning if the heading error exceeds 0.35 rad; give up on a leg
/// after 60 s. It reads the *true* pose, as the real driver reads `/ground_truth/pose`, and
/// it is the only part of the harness that does.
struct Follower {
    route: Vec<(f64, f64)>,
    index: usize,
    leg: Leg,
    leg_started: f64,
}

impl Follower {
    const LINEAR: f64 = 0.18;
    const ANGULAR: f64 = 0.8;
    const HEADING_GAIN: f64 = 1.2;
    const MAX_DRIVE_ANGULAR: f64 = 0.3;
    const POSITION_TOLERANCE: f64 = 0.06;
    const HEADING_TOLERANCE: f64 = 0.04;
    const MAX_HEADING_ERROR: f64 = 0.35;
    const LEG_TIMEOUT: f64 = 60.0;

    fn new(route: Vec<(f64, f64)>) -> Self {
        Self {
            route,
            index: 0,
            leg: Leg::Turn,
            leg_started: 0.0,
        }
    }

    fn advance(&mut self, t: f64) {
        self.index += 1;
        if self.index < self.route.len() {
            self.leg = Leg::Turn;
            self.leg_started = t;
        } else {
            self.index = self.route.len() - 1;
            self.leg = Leg::Finished;
        }
    }

    fn command(&mut self, pose: &Se2, t: f64) -> Twist2 {
        if self.leg == Leg::Finished {
            return Twist2::ZERO;
        }
        let (tx, ty) = self.route[self.index];
        let (dx, dy) = (tx - pose.x(), ty - pose.y());
        let distance = dx.hypot(dy);
        let heading_error = wrap(dy.atan2(dx) - pose.theta());
        if t - self.leg_started > Self::LEG_TIMEOUT {
            self.advance(t);
            return Twist2::ZERO;
        }
        match self.leg {
            Leg::Finished => Twist2::ZERO,
            Leg::Turn => {
                if heading_error.abs() <= Self::HEADING_TOLERANCE {
                    self.leg = Leg::Drive;
                    self.leg_started = t;
                    Twist2::ZERO
                } else {
                    Twist2::unicycle(0.0, Self::ANGULAR * heading_error.signum())
                }
            }
            Leg::Drive => {
                if distance <= Self::POSITION_TOLERANCE {
                    self.advance(t);
                    Twist2::ZERO
                } else if heading_error.abs() > Self::MAX_HEADING_ERROR {
                    self.leg = Leg::Turn;
                    self.leg_started = t;
                    Twist2::ZERO
                } else {
                    let w = (Self::HEADING_GAIN * heading_error)
                        .clamp(-Self::MAX_DRIVE_ANGULAR, Self::MAX_DRIVE_ANGULAR);
                    Twist2::unicycle(Self::LINEAR, w)
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------
// The session: Rusty drives the route, the EKF localizes
// ---------------------------------------------------------------------------------------

/// Names of the columns of one row (one row per 5 Hz scan), in order.
pub const COLUMNS: [&str; 31] = [
    "t",
    "distance",
    "truth_x",
    "truth_y",
    "truth_th",
    "odom_x",
    "odom_y",
    "odom_th",
    "ekf_x",
    "ekf_y",
    "ekf_th",
    "ekf_cov_xx",
    "ekf_cov_xy",
    "ekf_cov_yy",
    "ekf_sigma_th",
    "odom_err",
    "ekf_err",
    "odom_yaw_err",
    "ekf_yaw_err",
    "nees",
    "nees_mean",
    "nis_scan",
    "nis_mean",
    "detections",
    "accepted",
    "rejected",
    "inside_3sigma",
    "inside_3sigma_share",
    "odom_cov_xx",
    "odom_cov_xy",
    "odom_cov_yy",
];

/// Names of the entries of [`Ch04Session::summary`], in order.
pub const SUMMARY: [&str; 17] = [
    "scans",
    "seconds",
    "distance",
    "odom_rmse",
    "ekf_rmse",
    "odom_max",
    "odom_final",
    "ekf_final",
    "ekf_max",
    "ekf_yaw_rmse",
    "nees_mean",
    "nis_mean",
    "inside_3sigma_share",
    "accepted",
    "rejected",
    "kidnap_seconds",
    "finished",
];

/// A drive of the 51 m route in Rusty with the localization node's filter running on it,
/// advanced a few scans at a time so a widget can play it.
#[wasm_bindgen]
pub struct Ch04Session {
    sim: RustySim,
    odom: WheelOdometry,
    lidar: Lidar,
    world: World,
    loc: EkfLocalizer,
    detector: PillarDetector,
    motion: OdometryMotion,
    follower: Follower,
    correct: bool,
    kidnap_after: f64,
    kidnap_target: Se2,
    kidnap: Option<Se2>,
    kidnap_seconds: f64,
    prev_odom: Se2,
    last_sim_pose: Se2,
    steps: u64,
    distance: f64,
    scans: usize,
    rows: Vec<f64>,
    nees_stats: ConsistencyStats,
    nis_sum: f64,
    nis_n: u64,
    accepted: u64,
    rejected: u64,
    inside: u64,
    sq_odom: f64,
    sq_ekf: f64,
    sq_ekf_yaw: f64,
    max_odom: f64,
    max_ekf: f64,
}

#[wasm_bindgen]
impl Ch04Session {
    /// A new drive.
    ///
    /// * `slip_sigma`, `radius_error`: Rusty's per-step wheel slip and right-wheel radius
    ///   error (Chapter 2's defaults are 0.02 and 0.002).
    /// * `sigma_range`, `sigma_bearing`: the measurement noise the *filter assumes* (the
    ///   node's defaults are 0.05 and 0.05; the sensor's own noise is fixed).
    /// * `correct`: when `false` the filter predicts but never observes, which is the dead
    ///   reckoning with its own propagated covariance.
    /// * `kidnap_after`: simulated seconds after which the robot is carried to
    ///   `(kidnap_x, kidnap_y, kidnap_theta)` (negative: never).
    #[wasm_bindgen(constructor)]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        seed: u64,
        slip_sigma: f64,
        radius_error: f64,
        sigma_range: f64,
        sigma_bearing: f64,
        correct: bool,
        kidnap_after: f64,
        kidnap_x: f64,
        kidnap_y: f64,
        kidnap_theta: f64,
    ) -> Result<Ch04Session, String> {
        let mf = map_file()?;
        let cfg = RustyConfig {
            params: DiffDriveParams::RUSTY,
            physics_dt: PHYSICS_DT,
            wheel_rate_hz: 1.0 / (PHYSICS_DT * WHEEL_EVERY as f64),
            slip_sigma,
            right_radius_error: radius_error,
            seed,
            lidar: None,
            ..RustyConfig::default()
        };
        let idle = Trajectory::new(
            vec![Segment {
                duration: 1.0,
                twist: Twist2::ZERO,
            }],
            true,
        )
        .map_err(err)?;
        let sim = RustySim::with_world(cfg.clone(), idle, World::empty()).map_err(err)?;
        let mut odom = WheelOdometry::new(cfg.params, OdometryNoise::default()).map_err(err)?;
        let w0 = sim.initial_wheel_sample();
        odom.update_from_wheel_angles(w0.left_angle, w0.right_angle, w0.stamp);
        let spawn = mf.spawn_pose.unwrap_or(Se2::IDENTITY);
        let loc = EkfLocalizer::new(&localizer_config(sigma_range, sigma_bearing), mf.landmark_map(), spawn)
            .map_err(err)?;
        let lidar = Lidar::new(lds_config(LDS_SIGMA), stream_seed(seed, 4)).map_err(err)?;
        Ok(Ch04Session {
            sim,
            odom,
            lidar,
            world: build_world()?,
            loc,
            detector: PillarDetector::default(),
            motion: OdometryMotion::default(),
            follower: Follower::new(route_waypoints()?),
            correct,
            kidnap_after,
            kidnap_target: Se2::new(kidnap_x, kidnap_y, kidnap_theta),
            kidnap: None,
            kidnap_seconds: f64::NAN,
            prev_odom: Se2::IDENTITY,
            last_sim_pose: Se2::IDENTITY,
            steps: 0,
            distance: 0.0,
            scans: 0,
            rows: Vec::new(),
            nees_stats: ConsistencyStats::new(3).map_err(err)?,
            nis_sum: 0.0,
            nis_n: 0,
            accepted: 0,
            rejected: 0,
            inside: 0,
            sq_odom: 0.0,
            sq_ekf: 0.0,
            sq_ekf_yaw: 0.0,
            max_odom: 0.0,
            max_ekf: 0.0,
        })
    }

    /// Advance until `scans` more scans have been processed or the route is finished.
    /// Returns how many were processed.
    pub fn advance(&mut self, scans: u32) -> Result<u32, String> {
        let mut done = 0;
        while done < scans && !self.finished() {
            if self.physics_step()? {
                done += 1;
            }
        }
        Ok(done)
    }

    /// `true` once the route is finished (or the time limit is reached).
    pub fn finished(&self) -> bool {
        self.follower.leg == Leg::Finished || self.sim.time() >= MAX_SECONDS
    }

    /// Scans processed so far.
    pub fn scans(&self) -> u32 {
        self.scans as u32
    }

    /// Names of the columns of [`Ch04Session::rows`].
    pub fn columns() -> Vec<String> {
        COLUMNS.iter().map(|s| s.to_string()).collect()
    }

    /// Names of the entries of [`Ch04Session::summary`].
    pub fn summary_names() -> Vec<String> {
        SUMMARY.iter().map(|s| s.to_string()).collect()
    }

    /// Every row so far, row-major, `COLUMNS.len()` values per scan. Columns: `t` (s),
    /// `distance` (m driven), truth, raw odometry and EKF poses (`x, y, θ`), the EKF's
    /// world-frame position covariance (`xx, xy, yy`) and heading σ, the position and yaw
    /// error of odometry and of the EKF, this scan's NEES and its running mean, this scan's
    /// mean NIS (`NaN` if nothing was accepted) and its running mean, the detections, the
    /// observations accepted and rejected by the gate, whether the truth is inside the
    /// filter's own 3σ position ellipse (1 or 0) and the running share of scans for which it
    /// is, and the raw odometry's own position covariance (world frame).
    pub fn rows(&self) -> Vec<f64> {
        self.rows.clone()
    }

    /// The statistics of the run so far, in the order of `SUMMARY`.
    pub fn summary(&self) -> Vec<f64> {
        let n = self.scans.max(1) as f64;
        vec![
            self.scans as f64,
            self.sim.time(),
            self.distance,
            (self.sq_odom / n).sqrt(),
            (self.sq_ekf / n).sqrt(),
            self.max_odom,
            self.rows
                .chunks(COLUMNS.len())
                .last()
                .map(|r| r[15])
                .unwrap_or(f64::NAN),
            self.rows
                .chunks(COLUMNS.len())
                .last()
                .map(|r| r[16])
                .unwrap_or(f64::NAN),
            self.max_ekf,
            (self.sq_ekf_yaw / n).sqrt(),
            self.nees_stats.mean(),
            if self.nis_n > 0 {
                self.nis_sum / self.nis_n as f64
            } else {
                f64::NAN
            },
            self.inside as f64 / n,
            self.accepted as f64,
            self.rejected as f64,
            self.kidnap_seconds,
            if self.finished() { 1.0 } else { 0.0 },
        ]
    }
}

impl Ch04Session {
    /// The true pose: what the simulator integrated, carried by the kidnap if one happened.
    fn truth(&self) -> Se2 {
        match &self.kidnap {
            Some(k) => k.compose(&self.sim.pose()),
            None => self.sim.pose(),
        }
    }

    /// One physics step; `true` if it ended with a scan.
    fn physics_step(&mut self) -> Result<bool, String> {
        if self.steps % CONTROL_EVERY == 0 {
            let cmd = self.follower.command(&self.truth(), self.sim.time());
            self.sim.set_command_override(Some(cmd));
        }
        let s = self.sim.step();
        self.steps += 1;

        let p = self.sim.pose();
        self.distance += (p.translation() - self.last_sim_pose.translation()).norm();
        self.last_sim_pose = p;

        if self.kidnap.is_none() && self.kidnap_after >= 0.0 && self.sim.time() >= self.kidnap_after {
            self.kidnap = Some(self.kidnap_target.compose(&p.inverse()));
            self.kidnap_seconds = self.sim.time();
        }

        if let Some(w) = s.wheel {
            if self.odom.update_from_wheel_angles(w.left_angle, w.right_angle, w.stamp).is_some() {
                let now = self.odom.pose();
                let u = EkfLocalizer::relative_motion(&self.prev_odom, &now);
                self.prev_odom = now;
                // As the node does: a robot that did not move adds no noise.
                if u.translation().norm() >= 1e-9 || u.theta().abs() >= 1e-9 {
                    self.loc.predict(&self.motion, &u).map_err(err)?;
                }
            }
        }

        if self.steps % SCAN_EVERY != 0 {
            return Ok(false);
        }
        self.scan_update();
        Ok(true)
    }

    fn scan_update(&mut self) {
        let truth = self.truth();
        let stamp = self.sim.stamp();
        let scan = self.lidar.scan(&self.world, &truth, stamp);
        let detections = detect(&scan, &self.detector);
        let observations: Vec<RangeBearing> = detections.iter().map(|d| d.z).collect();
        let report = if self.correct {
            self.loc.observe(&observations)
        } else {
            UpdateReport {
                observed: observations.len(),
                ..Default::default()
            }
        };
        self.accepted += report.accepted as u64;
        self.rejected += report.rejected as u64;
        if let Some(m) = report.mean_nis {
            self.nis_sum += m * report.accepted as f64;
            self.nis_n += report.accepted as u64;
        }

        let est = self.loc.pose();
        let cov = self.loc.covariance();
        let odom = self.odom.pose();
        let odom_err = odom.distance_to(&truth);
        let ekf_err = est.distance_to(&truth);
        let nees_now = nees(&truth.boxminus(&est), &cov).unwrap_or(f64::NAN);
        if nees_now.is_finite() {
            let _ = self.nees_stats.push(nees_now);
        }
        // Position covariance in the world frame, to draw next to the world-frame poses.
        let r = est.rotation();
        let pos = r * cov.fixed_view::<2, 2>(0, 0).into_owned() * r.matrix().transpose();
        let inside = {
            // Is the true position inside the 3σ ellipse of the filter's own position
            // covariance? (Mahalanobis distance of the error, in the world frame.)
            let e = Vector2::new(truth.x() - est.x(), truth.y() - est.y());
            pos.try_inverse().map(|inv| (e.transpose() * inv * e)[(0, 0)] <= 9.0).unwrap_or(false)
        };
        self.inside += inside as u64;
        self.sq_odom += odom_err * odom_err;
        self.sq_ekf += ekf_err * ekf_err;
        let yaw_err_ekf = wrap(est.theta() - truth.theta());
        self.sq_ekf_yaw += yaw_err_ekf * yaw_err_ekf;
        self.max_odom = self.max_odom.max(odom_err);
        self.max_ekf = self.max_ekf.max(ekf_err);
        self.scans += 1;

        let ocov: Matrix3<f64> = self.odom.covariance();
        let ro = odom.rotation();
        let opos = ro * ocov.fixed_view::<2, 2>(0, 0).into_owned() * ro.matrix().transpose();
        let n = self.scans as f64;
        self.rows.extend_from_slice(&[
            self.sim.time(),
            self.distance,
            truth.x(),
            truth.y(),
            truth.theta(),
            odom.x(),
            odom.y(),
            odom.theta(),
            est.x(),
            est.y(),
            est.theta(),
            pos[(0, 0)],
            pos[(0, 1)],
            pos[(1, 1)],
            cov[(2, 2)].sqrt(),
            odom_err,
            ekf_err,
            wrap(odom.theta() - truth.theta()),
            yaw_err_ekf,
            nees_now,
            self.nees_stats.mean(),
            report.mean_nis.unwrap_or(f64::NAN),
            if self.nis_n > 0 { self.nis_sum / self.nis_n as f64 } else { f64::NAN },
            report.observed as f64,
            report.accepted as f64,
            report.rejected as f64,
            inside as u8 as f64,
            self.inside as f64 / n,
            opos[(0, 0)],
            opos[(0, 1)],
            opos[(1, 1)],
        ]);
    }
}

#[cfg(test)]
mod tests {
    //! `cargo test` runs these natively, so what they pin is the *repository's* realization
    //! of each seed (64-bit `SmallRng`). `web/checks/ch04.ts` pins the module's own, which is
    //! different noise for the same seed, and asserts the same qualitative claims over its own
    //! draws. Numbers with no random stream (the noise-free detector grid) must agree.
    use super::*;

    fn session(seed: u64, slip: f64, radius: f64, scale: f64, correct: bool, kidnap: f64) -> Ch04Session {
        Ch04Session::new(seed, slip, radius, 0.05 * scale, 0.05 * scale, correct, kidnap, -1.5, -1.5, 2.0).unwrap()
    }

    fn run(mut s: Ch04Session) -> (Vec<f64>, Vec<f64>) {
        while !s.finished() {
            s.advance(50).unwrap();
        }
        (s.summary(), s.rows())
    }

    fn get(v: &[f64], name: &str) -> f64 {
        v[SUMMARY.iter().position(|n| *n == name).unwrap()]
    }

    fn col(rows: &[f64], name: &str) -> Vec<f64> {
        let c = COLUMNS.iter().position(|n| *n == name).unwrap();
        rows.chunks(COLUMNS.len()).map(|r| r[c]).collect()
    }

    #[test]
    fn the_repository_files_parse() {
        assert_eq!(map_file().unwrap().landmarks.len(), 4);
        let route = route_waypoints().unwrap();
        assert_eq!(route.len(), 21);
        assert_eq!(route[0], (0.0, 1.8));
        assert_eq!(COLUMNS.len(), 31);
        assert_eq!(SUMMARY.len(), 17);
    }

    #[test]
    fn a_seed_replays_and_two_seeds_differ() {
        let a = run(session(3, 0.02, 0.002, 1.0, true, -1.0));
        let b = run(session(3, 0.02, 0.002, 1.0, true, -1.0));
        let c = run(session(4, 0.02, 0.002, 1.0, true, -1.0));
        assert_eq!(a.1, b.1, "same seed, same rows, bit for bit");
        assert_ne!(a.1, c.1);
    }

    /// The repository's native realization of seeds 1 to 8, the table the chapter's page
    /// does *not* reproduce (the module draws other noise for the same seeds).
    #[test]
    fn native_table_of_eight_seeds() {
        // seed: odometry RMSE (m), EKF RMSE (m), EKF max error (m), observations rejected
        let pinned: [(u64, f64, f64, f64, f64); 8] = [
            (1, 0.4179, 0.0079, 0.0147, 946.0),
            (2, 0.4480, 0.0078, 0.0146, 934.0),
            (3, 0.4735, 0.0076, 0.0142, 971.0),
            (4, 0.3884, 0.0079, 0.0157, 977.0),
            (5, 0.4607, 0.0076, 0.0141, 950.0),
            (6, 0.4266, 0.0077, 0.0149, 924.0),
            (7, 0.4373, 0.0076, 0.0139, 949.0),
            (8, 0.5068, 0.0078, 0.0143, 959.0),
        ];
        for (seed, odom, ekf, max, rejected) in pinned {
            let (v, _) = run(session(seed, 0.02, 0.002, 1.0, true, -1.0));
            let r4 = |x: f64| (x * 1e4).round() / 1e4;
            assert_eq!(r4(get(&v, "odom_rmse")), odom, "seed {seed}");
            assert_eq!(r4(get(&v, "ekf_rmse")), ekf, "seed {seed}");
            assert_eq!(r4(get(&v, "ekf_max")), max, "seed {seed}");
            assert_eq!(get(&v, "rejected"), rejected, "seed {seed}");
            // what must hold on every target
            assert!(get(&v, "odom_rmse") > 0.35 && get(&v, "ekf_rmse") < 0.01, "seed {seed}");
            assert_eq!(get(&v, "inside_3sigma_share"), 1.0, "seed {seed}");
            assert!((51.0..52.0).contains(&get(&v, "distance")), "seed {seed}");
            assert_eq!(get(&v, "finished"), 1.0);
        }
    }

    #[test]
    fn without_correction_the_filter_is_the_odometry() {
        let (v, rows) = run(session(1, 0.02, 0.002, 1.0, false, -1.0));
        for (e, o) in col(&rows, "ekf_x").iter().zip(col(&rows, "odom_x")) {
            assert!((e - o).abs() < 1e-9);
        }
        assert!((get(&v, "ekf_rmse") - get(&v, "odom_rmse")).abs() < 1e-9);
        assert_eq!(get(&v, "accepted") + get(&v, "rejected"), 0.0);
    }

    #[test]
    fn a_correction_is_not_free_perfect_odometry_beats_the_filter() {
        // No slip and no radius error: the wheels are exact, the filter only adds the
        // detector's noise and bias.
        for seed in 1..=4 {
            let (v, _) = run(session(seed, 0.0, 0.0, 1.0, true, -1.0));
            assert!(get(&v, "odom_rmse") < 1e-3, "seed {seed}: {}", get(&v, "odom_rmse"));
            assert!(get(&v, "ekf_rmse") > 5e-3, "seed {seed}");
            assert!(get(&v, "ekf_rmse") > 5.0 * get(&v, "odom_rmse"), "seed {seed}");
        }
    }

    #[test]
    fn an_overconfident_filter_throws_its_data_away_and_is_worse() {
        for seed in 1..=4 {
            let (honest, _) = run(session(seed, 0.02, 0.002, 1.0, true, -1.0));
            let (over, _) = run(session(seed, 0.02, 0.002, 0.1, true, -1.0));
            assert!(get(&over, "nees_mean") > 20.0, "seed {seed}");
            assert!(get(&over, "inside_3sigma_share") < 0.05, "seed {seed}");
            assert!(get(&over, "ekf_rmse") > 2.0 * get(&honest, "ekf_rmse"), "seed {seed}");
            assert!(get(&over, "rejected") > 3.0 * get(&honest, "rejected"), "seed {seed}");
        }
    }

    #[test]
    fn a_kidnapped_filter_does_not_recover() {
        for seed in 1..=4 {
            let (v, rows) = run(session(seed, 0.02, 0.002, 1.0, true, 120.0));
            assert_eq!(get(&v, "kidnap_seconds"), 120.0);
            let t = col(&rows, "t");
            let err = col(&rows, "ekf_err");
            let truth_x = col(&rows, "truth_x");
            let truth_y = col(&rows, "truth_y");
            // the first row at or after the carry has the robot at the target
            let k = t.iter().position(|&x| x >= 120.0).unwrap();
            assert!(((truth_x[k] + 1.5).powi(2) + (truth_y[k] + 1.5).powi(2)).sqrt() < 1e-9);
            // before it the filter was tracking, to centimetres
            assert!(err[..k].iter().all(|&e| e < 0.05), "seed {seed}");
            // after it the error is the length of the carry and stays about that: it never
            // comes back under 3 m in the rest of the route
            assert!(err[k] > 3.0, "seed {seed}");
            assert!(err[k..].iter().all(|&e| e > 3.0), "seed {seed}: {}", err[k..].iter().cloned().fold(f64::INFINITY, f64::min));
            assert!(get(&v, "ekf_final") > 3.0, "seed {seed}");
            // and it knows something is wrong: it throws most of what it sees away
            let rej: f64 = col(&rows, "rejected")[k..].iter().sum();
            let acc: f64 = col(&rows, "accepted")[k..].iter().sum();
            assert!(rej > 5.0 * acc, "seed {seed}: rejected {rej}, accepted {acc}");
        }
    }

    /// Geometry only (no noise stream), so these counts are the same natively and on
    /// WebAssembly; `web/checks/ch04.ts` asserts the same numbers.
    #[test]
    fn the_detector_on_a_grid_of_poses_without_noise() {
        let lm: Vec<(f64, f64)> = map_file().unwrap().landmarks.iter().map(|l| (l.x, l.y)).collect();
        let (mut scans, mut genuine, mut spurious) = (0, 0, 0);
        let mut bias = Vec::new();
        let mut worst: f64 = 0.0;
        for ix in (-26..=26).step_by(3) {
            for iy in (-26..=26).step_by(3) {
                for th in [0.0, 1.3, 2.6, -2.0] {
                    let (x, y) = (ix as f64 / 10.0, iy as f64 / 10.0);
                    if lm.iter().any(|&(lx, ly)| (lx - x).hypot(ly - y) < 0.45) {
                        continue;
                    }
                    let sc = ch04_scan(x, y, th, 0.0, 1).unwrap();
                    scans += 1;
                    for d in sc.detections().chunks(DETECTION_STRIDE) {
                        if d[7] < 0.1 {
                            genuine += 1;
                            worst = worst.max(d[7]);
                            bias.push(d[0] - d[8]);
                        } else {
                            spurious += 1;
                        }
                    }
                }
            }
        }
        assert_eq!((scans, genuine, spurious), (1176, 2823, 225));
        // Finding: the centroid of the visible arc lies behind the near surface, so
        // adding the radius overshoots the centre: the reported range is too long, every
        // time, by about 3 cm (median), and by at most 5.3 cm.
        bias.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!(bias[0] > 0.0, "noise-free, the reported range is never short");
        let median = bias[bias.len() / 2];
        assert!((0.025..0.030).contains(&median), "median bias {median}");
        assert!(*bias.last().unwrap() < 0.053);
        assert!(worst < 0.0503);
    }

    #[test]
    fn the_gate_has_a_capture_range_set_by_the_beliefs_own_uncertainty() {
        let poses = [(0.0, 0.0, 0.0), (0.3, -0.4, 2.0), (-1.0, 1.0, -1.0), (1.0, -1.0, 0.7)];
        for &(x, y, th) in &poses {
            for dir in [0.0f64, 1.5, 3.0, -1.5] {
                let at = |off: f64, sig: f64| {
                    ch04_associate(x, y, th, off * dir.cos(), off * dir.sin(), 0.0, sig, sig, 0.99, 7).unwrap()
                };
                // a belief that is right to 10 cm, with the node's 5 cm σ: everything seen
                // is accepted, and is the right pillar or a wall artefact the gate keeps out
                for off in [0.0, 0.05, 0.1] {
                    let a = at(off, 0.05);
                    for d in a.per_detection().chunks(ASSOC_STRIDE) {
                        if d[0] == 1.0 {
                            assert_eq!(d[1], d[3], "pose {x},{y} off {off}");
                        }
                    }
                    assert!(a.after()[3] <= a.before()[3].max(0.01), "pose {x},{y} off {off}: {:?}", a.after());
                }
                // 0.5 m off with that σ, at most one observation is accepted (one whose
                // range is unchanged by a sideways error) and the belief stays wrong
                let a = at(0.5, 0.05);
                assert!(a.after()[5] <= 1.0, "pose {x},{y}");
                assert!(a.after()[3] > 0.4, "pose {x},{y}: {:?}", a.after());
                // the same offset with a belief that admits 30 cm: at least two observations
                // are accepted, all of them the right pillar, and the error falls by more
                // than half (to 0.21 m at worst of these poses; usually under 5 cm)
                let a = at(0.5, 0.3);
                assert!(a.after()[5] >= 2.0 && a.after()[3] < 0.25, "pose {x},{y} {:?}", a.after());
                for d in a.per_detection().chunks(ASSOC_STRIDE) {
                    if d[0] == 1.0 {
                        assert_eq!(d[1], d[3], "pose {x},{y}");
                    }
                }
            }
        }
    }

    /// A belief that admits a lot (σ = 0.3 m) and is wrong by more than the pillars are
    /// apart from what they look like from nearby accepts a *wrong* pillar: the gate is
    /// satisfied, the association is not the truth, and the "correction" moves the belief
    /// away from it. Geometry plus a small range noise: the case is not an accident of one
    /// noise draw (asserted for several seeds).
    #[test]
    fn a_wide_belief_can_associate_the_wrong_pillar_and_be_made_worse() {
        for seed in 1..=6 {
            let a = ch04_associate(0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.3, 0.3, 0.99, seed).unwrap();
            let wrong = a
                .per_detection()
                .chunks(ASSOC_STRIDE)
                .filter(|d| d[0] == 1.0 && d[1] != d[3])
                .count();
            assert!(wrong >= 1, "seed {seed}");
            assert!(a.after()[3] > a.before()[3], "seed {seed}: {:?} vs {:?}", a.after(), a.before());
        }
    }
}
