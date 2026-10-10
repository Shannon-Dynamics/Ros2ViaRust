//! `route_driver`: drives the robot along a fixed waypoint route for an evaluation run.
//!
//! ```text
//!   geometry_msgs/PoseStamped (topics.ground_truth) ─► waypoint follower ─► geometry_msgs/TwistStamped (topics.cmd_vel)
//!                                                              │
//!                                                              └─► /diagnostics, and exit(0) when the route is finished
//! ```
//!
//! This node is part of the *measuring apparatus*, not of the stack under test. It is the
//! second node after `localization_evaluator` that is allowed to subscribe to ground
//! truth, and for the same reason: driving a route and grading an estimate are both jobs
//! of the harness. `localization_node` still never sees ground truth, which is what keeps
//! the evaluation meaningful.
//!
//! Why it exists at all, rather than a list of `ros2 topic pub` calls in the run script:
//! an open-loop route cannot be driven for minutes in this simulator. A commanded
//! `angular.z` is not the rate the robot turns at, because the wheels slip against the
//! floor: the 2026-10-06 square was commanded four 1.6 rad turns and ground truth recorded
//! about 1.38 rad each, a 13 degree error per corner. Over a route with twenty corners
//! that is several radians of accumulated heading error, and the robot ends up driving into
//! a pillar or a wall instead of along the route that was designed. Closing the loop on
//! ground truth makes the geometry of the route a property of the route file rather than of
//! how much the wheels happened to slip.
//!
//! Each leg is a turn in place followed by a straight drive, so the corners stay sharp:
//! that is the point of the route, because a sharp corner is where wheel odometry earns its
//! error. During the straight leg the heading is held with a small proportional term; if
//! the heading error grows past `drive.max_heading_error`, the follower drops back to
//! turning in place rather than driving an arc.

use rclrs::*;
use robotics_core::{FrameId, Se2, Timestamp};
use robotics_ros_bridge::convert::{header, now, pose_from_ros};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::geometry_msgs;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

/// A route as the file describes it.
struct RouteFile {
    name: String,
    waypoints: Vec<(f64, f64)>,
}

/// Read a route file.
///
/// Same deliberately small reader as `robotics_localization::MapFile`, for the same
/// reason: a waypoint quietly dropped from the route changes what was measured, so
/// anything the reader does not understand is an error rather than a skipped line.
///
/// ```text
/// route:
///   name: star-sawtooth-star
/// waypoints:
///   - {x: 0.0, y: 1.8}
/// ```
fn read_route(path: impl AsRef<Path>) -> Result<RouteFile, String> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let mut name = String::new();
    let mut waypoints = Vec::new();
    let mut section = String::new();
    for (i, raw) in text.lines().enumerate() {
        let line = i + 1;
        let content = raw.split('#').next().unwrap_or("");
        if content.trim().is_empty() {
            continue;
        }
        let indent = content.len() - content.trim_start().len();
        let trimmed = content.trim();
        if indent == 0 {
            section = trimmed
                .strip_suffix(':')
                .ok_or_else(|| format!("line {line}: expected a `section:` header"))?
                .to_string();
            continue;
        }
        match section.as_str() {
            "route" => {
                let (k, v) = trimmed
                    .split_once(':')
                    .ok_or_else(|| format!("line {line}: expected `key: value`"))?;
                if k.trim() == "name" {
                    name = v.trim().to_string();
                } else {
                    return Err(format!("line {line}: unknown route key {:?}", k.trim()));
                }
            }
            "waypoints" => {
                let body = trimmed
                    .strip_prefix("- {")
                    .and_then(|s| s.strip_suffix('}'))
                    .ok_or_else(|| format!("line {line}: expected `- {{x: .., y: ..}}`"))?;
                let mut x = None;
                let mut y = None;
                for pair in body.split(',') {
                    let (k, v) = pair
                        .trim()
                        .split_once(':')
                        .ok_or_else(|| format!("line {line}: expected `key: value`"))?;
                    let value = v.trim().parse::<f64>().map_err(|e| {
                        format!("line {line}: {:?} is not a number ({e})", v.trim())
                    })?;
                    match k.trim() {
                        "x" => x = Some(value),
                        "y" => y = Some(value),
                        other => return Err(format!("line {line}: unknown key {other:?}")),
                    }
                }
                match (x, y) {
                    (Some(x), Some(y)) => waypoints.push((x, y)),
                    _ => return Err(format!("line {line}: a waypoint needs both x and y")),
                }
            }
            other => return Err(format!("line {line}: unknown section {other:?}")),
        }
    }
    if waypoints.is_empty() {
        return Err("the route has no waypoints".to_string());
    }
    Ok(RouteFile { name, waypoints })
}

/// What the follower is doing on the current leg.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Leg {
    /// Turning in place to face the waypoint.
    Turn,
    /// Driving straight towards it.
    Drive,
    /// Every waypoint and every lap is done.
    Finished,
}

impl Leg {
    fn as_str(self) -> &'static str {
        match self {
            Leg::Turn => "turn",
            Leg::Drive => "drive",
            Leg::Finished => "finished",
        }
    }
}

/// Tuning of the follower.
#[derive(Clone, Copy)]
struct Gains {
    linear: f64,
    angular: f64,
    heading_gain: f64,
    max_drive_angular: f64,
    position_tolerance: f64,
    heading_tolerance: f64,
    max_heading_error: f64,
    leg_timeout: f64,
}

struct State {
    route: Vec<(f64, f64)>,
    route_name: String,
    laps: u32,
    gains: Gains,
    publisher: Publisher<geometry_msgs::msg::TwistStamped>,
    frame: FrameId,
    stamp_commands: bool,
    pose: Option<Se2>,
    index: usize,
    lap: u32,
    leg: Leg,
    leg_started: Option<Timestamp>,
    distance_travelled: f64,
    rotation_turned: f64,
    last_pose: Option<Se2>,
    corners: u64,
    timeouts: u64,
    ticks: u64,
    finished_at: Option<Timestamp>,
    last_error: String,
}

/// Wrap an angle into `(-pi, pi]`.
fn wrap(mut d: f64) -> f64 {
    while d > std::f64::consts::PI {
        d -= std::f64::consts::TAU;
    }
    while d <= -std::f64::consts::PI {
        d += std::f64::consts::TAU;
    }
    d
}

impl State {
    /// The waypoint being driven to. Saturating, because `index` is left at the last
    /// waypoint once the route is finished and the diagnostics timer still reports a
    /// target; indexing past the end panicked the timer thread at the end of the
    /// 2026-10-10 run, after the route itself had completed.
    fn target(&self) -> (f64, f64) {
        self.route[self.index.min(self.route.len() - 1)]
    }

    fn send(&self, node: &Node, stamp: Timestamp, linear: f64, angular: f64) {
        // A zero stamp by default, which is not laziness. This node runs on the system
        // clock while the simulator runs on /clock, so a stamp taken from here would be an
        // epoch time handed to a controller whose clock starts near zero, and
        // `diff_drive_controller` would discard every command as older than
        // `cmd_vel_timeout`. A zero stamp is the one case it handles explicitly: it
        // substitutes its own clock and warns once. That is also exactly what the
        // `ros2 topic pub` calls of the earlier runs did, so the commanded path is
        // unchanged from the runs this one is compared against.
        let stamp = if self.stamp_commands {
            stamp
        } else {
            Timestamp::from_sec_nanosec(0, 0)
        };
        let msg = geometry_msgs::msg::TwistStamped {
            header: header(stamp, &self.frame),
            twist: geometry_msgs::msg::Twist {
                linear: geometry_msgs::msg::Vector3 {
                    x: linear,
                    y: 0.0,
                    z: 0.0,
                },
                angular: geometry_msgs::msg::Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: angular,
                },
            },
        };
        if let Err(e) = self.publisher.publish(msg) {
            log_error!(
                node.logger().throttle(Duration::from_secs(5)),
                "cannot publish a velocity command: {e}"
            );
        }
    }

    /// Move on to the next waypoint, or finish the route.
    fn advance(&mut self, node: &Node, stamp: Timestamp) {
        self.index += 1;
        if self.index < self.route.len() {
            self.leg = Leg::Turn;
            self.leg_started = Some(stamp);
            return;
        }
        self.lap += 1;
        if self.lap >= self.laps {
            self.index = self.route.len() - 1;
            self.leg = Leg::Finished;
            self.finished_at = Some(stamp);
            log_info!(
                node.logger(),
                "route {:?} finished: {} lap(s), {} waypoints, {:.2} m driven, \
                 {:.2} rad turned, {} corner(s), {} leg timeout(s)",
                self.route_name,
                self.laps,
                self.route.len() as u32 * self.laps,
                self.distance_travelled,
                self.rotation_turned,
                self.corners,
                self.timeouts
            );
            return;
        }
        self.index = 0;
        self.leg = Leg::Turn;
        self.leg_started = Some(stamp);
        log_info!(
            node.logger(),
            "lap {} of {} starting ({:.2} m driven so far)",
            self.lap + 1,
            self.laps,
            self.distance_travelled
        );
    }

    fn step(&mut self, node: &Node, stamp: Timestamp) {
        self.ticks += 1;
        if self.leg == Leg::Finished {
            self.send(node, stamp, 0.0, 0.0);
            return;
        }
        let Some(pose) = self.pose else {
            // No ground truth yet: hold still rather than drive blind.
            self.send(node, stamp, 0.0, 0.0);
            return;
        };
        if let Some(previous) = self.last_pose {
            self.distance_travelled += (pose.translation() - previous.translation()).norm();
            self.rotation_turned += wrap(pose.theta() - previous.theta()).abs();
        }
        self.last_pose = Some(pose);
        if self.leg_started.is_none() {
            self.leg_started = Some(stamp);
        }

        let (tx, ty) = self.target();
        let dx = tx - pose.x();
        let dy = ty - pose.y();
        let distance = (dx * dx + dy * dy).sqrt();
        let heading_error = wrap(dy.atan2(dx) - pose.theta());
        let g = self.gains;

        // A leg that does not end is a stuck run. Give up on this waypoint, record it, and
        // carry on to the next one: a route that reports two timeouts is still a usable
        // measurement, a run that hangs forever is not.
        if let Some(started) = self.leg_started {
            if stamp - started > g.leg_timeout {
                self.timeouts += 1;
                self.last_error = format!(
                    "waypoint {} ({:.2}, {:.2}) not reached within {:.0} s on the {} leg",
                    self.index,
                    tx,
                    ty,
                    g.leg_timeout,
                    self.leg.as_str()
                );
                log_warn!(node.logger(), "{}", self.last_error);
                self.advance(node, stamp);
                self.send(node, stamp, 0.0, 0.0);
                return;
            }
        }

        match self.leg {
            Leg::Finished => unreachable!("handled above"),
            Leg::Turn => {
                if heading_error.abs() <= g.heading_tolerance {
                    self.corners += 1;
                    self.leg = Leg::Drive;
                    self.leg_started = Some(stamp);
                    self.send(node, stamp, 0.0, 0.0);
                    return;
                }
                self.send(node, stamp, 0.0, g.angular * heading_error.signum());
            }
            Leg::Drive => {
                if distance <= g.position_tolerance {
                    log_info!(
                        node.logger().throttle(Duration::from_secs(1)),
                        "waypoint {}/{} of lap {} reached at ({:.3}, {:.3})",
                        self.index + 1,
                        self.route.len(),
                        self.lap + 1,
                        pose.x(),
                        pose.y()
                    );
                    self.advance(node, stamp);
                    self.send(node, stamp, 0.0, 0.0);
                    return;
                }
                if heading_error.abs() > g.max_heading_error {
                    // Too far off course to correct while moving; stop and turn.
                    self.leg = Leg::Turn;
                    self.leg_started = Some(stamp);
                    self.send(node, stamp, 0.0, 0.0);
                    return;
                }
                let angular = (g.heading_gain * heading_error)
                    .clamp(-g.max_drive_angular, g.max_drive_angular);
                self.send(node, stamp, g.linear, angular);
            }
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let commands = Arc::clone(executor.commands());
    let node = executor.create_node("route_driver")?;
    let mut p = ParamStore::new();

    let gt_topic = p.string(
        &node,
        "topics.ground_truth",
        "ground_truth/pose",
        "Ground truth pose the follower closes the loop on (geometry_msgs/PoseStamped)",
    )?;
    let cmd_topic = p.string(
        &node,
        "topics.cmd_vel",
        "cmd_vel",
        "Velocity command output (geometry_msgs/TwistStamped)",
    )?;
    let frame = p.string(
        &node,
        "frames.base_link",
        "base_link",
        "Frame the velocity command is expressed in",
    )?;
    let route_file = p.string(
        &node,
        "route_file",
        "",
        "Path to the route file (sim/webots/config/route_*.yaml)",
    )?;
    let laps = p.i64(
        &node,
        "laps",
        1,
        1,
        100,
        "How many times to drive the route",
    )? as u32;
    let rate = p.f64(&node, "rate_hz", 20.0, 1.0, 200.0, "Command rate (Hz)")?;
    let gains = Gains {
        linear: p.f64(
            &node,
            "drive.linear",
            0.18,
            0.01,
            1.0,
            "Speed of a straight leg (m/s); the driver config caps linear.x at 0.2",
        )?,
        angular: p.f64(
            &node,
            "drive.angular",
            0.8,
            0.01,
            2.0,
            "Rate of a turn in place (rad/s)",
        )?,
        heading_gain: p.f64(
            &node,
            "drive.heading_gain",
            1.2,
            0.0,
            20.0,
            "Proportional heading term used while driving straight (1/s)",
        )?,
        max_drive_angular: p.f64(
            &node,
            "drive.max_angular",
            0.3,
            0.0,
            2.0,
            "Cap on the heading correction while driving, so a straight leg stays straight",
        )?,
        position_tolerance: p.f64(
            &node,
            "drive.position_tolerance",
            0.06,
            0.005,
            1.0,
            "A waypoint counts as reached inside this radius (m)",
        )?,
        heading_tolerance: p.f64(
            &node,
            "drive.heading_tolerance",
            0.04,
            0.001,
            1.0,
            "A turn in place ends inside this heading error (rad)",
        )?,
        max_heading_error: p.f64(
            &node,
            "drive.max_heading_error",
            0.35,
            0.01,
            3.0,
            "Heading error that sends the follower back to turning in place (rad)",
        )?,
        leg_timeout: p.f64(
            &node,
            "drive.leg_timeout",
            60.0,
            1.0,
            600.0,
            "Give up on a waypoint after this long and move to the next one (s)",
        )?,
    };
    let stamp_commands = p.bool(
        &node,
        "stamp_commands",
        false,
        "Stamp the velocity commands from this node's clock. Off by default: this node runs \
         on the system clock and the controller on /clock, so a stamp from here is stale on \
         arrival and a zero stamp makes the controller substitute its own",
    )?;
    let linger = p.f64(
        &node,
        "stop_seconds",
        3.0,
        0.0,
        60.0,
        "Keep publishing a zero command for this long after the route, so the base stops \
         before the node exits",
    )?;
    let diag_period = p.i64(
        &node,
        "diagnostics_period_s",
        1,
        1,
        60,
        "Diagnostics period (s)",
    )?;

    if route_file.is_empty() {
        log_error!(node.logger(), "route_file is required");
        return Err(RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        });
    }
    let route = read_route(&route_file).map_err(|e| {
        log_error!(node.logger(), "cannot read route {route_file}: {e}");
        RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        }
    })?;
    log_info!(
        node.logger(),
        "route {:?} from {}: {} waypoint(s) x {} lap(s)",
        route.name,
        route_file,
        route.waypoints.len(),
        laps
    );

    let publisher = node.create_publisher::<geometry_msgs::msg::TwistStamped>(
        cmd_topic.as_str().qos(qos::odometry()),
    )?;
    let diagnostics = DiagnosticsPublisher::new(&node)?;

    let shared = Shared::new(State {
        route: route.waypoints,
        route_name: route.name,
        laps,
        gains,
        publisher,
        frame: FrameId::new(&frame),
        stamp_commands,
        pose: None,
        index: 0,
        lap: 0,
        leg: Leg::Turn,
        leg_started: None,
        distance_travelled: 0.0,
        rotation_turned: 0.0,
        last_pose: None,
        corners: 0,
        timeouts: 0,
        ticks: 0,
        finished_at: None,
        last_error: String::new(),
    });

    let _gt_sub = node.create_subscription::<geometry_msgs::msg::PoseStamped, _>(
        gt_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |msg: geometry_msgs::msg::PoseStamped| {
                if let Ok(pose) = pose_from_ros(&msg.pose) {
                    shared.lock().pose = Some(pose);
                }
            }
        },
    )?;

    // The route is finished and the base has been told to stop for `stop_seconds`: halt the
    // executor so that the run script can simply `wait` for this process instead of
    // guessing how long the route takes.
    let done = Arc::new(AtomicBool::new(false));
    let n = Arc::clone(&node);
    let _control = node.create_timer_repeating(Duration::from_secs_f64(1.0 / rate), {
        let shared = shared.clone();
        let done = Arc::clone(&done);
        move || {
            let stamp = match now(&n) {
                Ok(t) => t,
                Err(e) => {
                    log_error!(
                        n.logger().throttle(Duration::from_secs(5)),
                        "clock error: {e}"
                    );
                    return;
                }
            };
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            st.step(&n, stamp);
            if let Some(at) = st.finished_at {
                if stamp - at >= linger && !done.swap(true, Ordering::SeqCst) {
                    log_info!(n.logger(), "route driver done, stopping");
                    commands.halt_spinning();
                }
            }
        }
    })?;

    let n = Arc::clone(&node);
    let _diag = node.create_timer_repeating(Duration::from_secs(diag_period as u64), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            let stamp = match now(&n) {
                Ok(t) => t,
                Err(_) => return,
            };
            let (level, message) = match (st.leg, st.pose) {
                (Leg::Finished, _) => (DiagnosticLevel::Ok, "FINISHED".to_string()),
                (_, None) => (
                    DiagnosticLevel::Warn,
                    "WAITING for ground truth".to_string(),
                ),
                _ => (DiagnosticLevel::Ok, "DRIVING".to_string()),
            };
            let (tx, ty) = st.target();
            let b = StatusBuilder::new("route_driver: waypoint follower", level, &message)
                .value("route", &st.route_name)
                .value("leg", st.leg.as_str())
                .value("lap", format!("{}/{}", st.lap + 1, st.laps))
                .value("waypoint", format!("{}/{}", st.index + 1, st.route.len()))
                .value("target_x", format!("{tx:.3}"))
                .value("target_y", format!("{ty:.3}"))
                .value(
                    "distance_travelled_m",
                    format!("{:.3}", st.distance_travelled),
                )
                .value("rotation_turned_rad", format!("{:.3}", st.rotation_turned))
                .value("corners", st.corners)
                .value("leg_timeouts", st.timeouts)
                .value("commands", st.ticks)
                .value("last_error", &st.last_error);
            if let Err(e) = diagnostics.publish(stamp, vec![b.build()]) {
                log_error!(n.logger(), "diagnostics publish failed: {e}");
            }
        }
    })?;

    log_info!(
        node.logger(),
        "route_driver up: {} -> {} at {:.0} Hz, {} parameters",
        gt_topic,
        cmd_topic,
        rate,
        p.len()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    {
        let guard = shared.lock();
        log_info!(
            node.logger(),
            "route_driver shutting down after {:.2} m and {:.2} rad on route {:?} \
             ({} corner(s), {} leg timeout(s))",
            guard.distance_travelled,
            guard.rotation_turned,
            guard.route_name,
            guard.corners,
            guard.timeouts
        );
    }
    drop(p);
    result
}
