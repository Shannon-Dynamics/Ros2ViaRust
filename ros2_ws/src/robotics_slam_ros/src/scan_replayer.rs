//! `scan_replayer` — publishes a recorded scan stream as `/scan`, for testing the SLAM nodes
//! without a simulator. Test harness only: it is not part of any real-robot launch.
//!
//! The input is a CSV in the format of Phase 4's `integrated-scans.csv` and of the `scans.csv`
//! the Phase 5 harness records: `stamp_s, sensor_x, sensor_y, sensor_theta, angle_min,
//! angle_increment, range_min, range_max, n, ranges...`. **The three pose columns are ignored:**
//! they are ground truth, and nothing on the ROS graph may read it from here. The scans are
//! published with the stamps in the file (not the wall clock), the sensor QoS a real driver uses,
//! at `rate_hz`, starting once `wait_subscribers` subscribers are connected. After the last scan
//! the node lingers `linger_s` and then exits with code 0, so a launch file can shut everything
//! else down when it ends (as `route_driver` does).
//!
//! **Lockstep** (`lockstep:=true`): the next scan is published only after the scan matcher's
//! registration of the previous one has come back on `topics.registration` (or `lockstep_timeout_s`
//! has passed, for a scan the matcher declined), and `rate_hz` is then only an upper bound. A
//! sensor does not wait for its consumer, and a consumer that cannot keep up loses scans on a
//! best-effort topic; lockstep is for tests that must see every scan whatever the build profile
//! or the machine, and says nothing about real time. Without it the replay is as lossy as a real
//! driver would be.
//!
//! **Stationary odometry** (`odom_tf:=identity`, off by default): before each scan the replayer
//! publishes `odom -> <frame_id>` as the identity, stamped where `pose_graph_node` looks it up
//! (`odom_tf_time_offset_s` after the scan's stamp). It exists so that the ownership of
//! `map -> odom` can be tested without a simulator and **without reading the ground-truth columns
//! the replayer ignores**: with the sensor "stationary" in `odom`, `map -> odom` has to equal the
//! SLAM estimate of the sensor pose in `map`, which a check can recompute from the node's own
//! files. It is test scaffolding and says nothing about any odometry; a real run takes
//! `odom -> base_link` from the driver.

use rclrs::*;
use robotics_core::{FrameId, LaserScan, Se2, Timestamp};
use robotics_ros_bridge::convert::laser_scan_to_ros;
use robotics_ros_bridge::convert::time_from_ros;
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::tf::TfBroadcaster;
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::{geometry_msgs, sensor_msgs};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn load(
    path: &str,
    frame: &FrameId,
    limit: usize,
) -> Result<Vec<sensor_msgs::msg::LaserScan>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let mut out = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        if limit > 0 && out.len() >= limit {
            break;
        }
        let f: Vec<&str> = line.split(',').collect();
        let bad = |what: &str| format!("{path}:{}: {what}", ln + 1);
        if f.len() < 9 {
            return Err(bad("too few fields"));
        }
        let num = |i: usize| {
            f[i].trim()
                .parse::<f64>()
                .map_err(|e| bad(&format!("field {i}: {e}")))
        };
        let n = num(8)? as usize;
        if f.len() != 9 + n {
            return Err(bad(&format!("{} ranges, the header says {n}", f.len() - 9)));
        }
        let mut ranges = Vec::with_capacity(n);
        for r in &f[9..] {
            ranges.push(r.trim().parse::<f32>().map_err(|e| bad(&e.to_string()))?);
        }
        let scan = LaserScan {
            stamp: Timestamp::from_secs_f64(num(0)?),
            frame: frame.clone(),
            angle_min: num(4)?,
            angle_increment: num(5)?,
            range_min: num(6)?,
            range_max: num(7)?,
            ranges,
            ..LaserScan::default()
        };
        out.push(laser_scan_to_ros(&scan));
    }
    if out.is_empty() {
        return Err(format!("{path}: no scans"));
    }
    Ok(out)
}

struct State {
    scans: Vec<sensor_msgs::msg::LaserScan>,
    next: usize,
    /// Lockstep: the newest registration stamp seen (ns), and when the last scan went out.
    registered_ns: i64,
    last_sent: Instant,
    started: bool,
    created: Instant,
    finished: Option<Instant>,
    publisher: Publisher<sensor_msgs::msg::LaserScan>,
    /// Stationary odometry: the broadcaster, the parent and child frames and the stamp offset.
    odom: Option<(TfBroadcaster, FrameId, FrameId, f64)>,
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("scan_replayer")?;
    let mut p = ParamStore::new();
    let file = p.string(
        &node,
        "file",
        "none",
        "Recorded scan stream (CSV, the Phase 4 integrated-scans format)",
    )?;
    let topic = p.string(
        &node,
        "topics.scan",
        "scan",
        "Where the scans are published (sensor QoS)",
    )?;
    let frame_id = p.string(
        &node,
        "frame_id",
        "LDS-01",
        "frame_id of the published scans (the Webots lidar's)",
    )?;
    let rate = p.f64(
        &node,
        "rate_hz",
        5.0,
        0.1,
        1000.0,
        "Scans per second (5 is real time for the Webots lidar)",
    )?;
    let wait_subscribers = p.i64(
        &node,
        "wait_subscribers",
        1,
        0,
        100,
        "Start when this many subscribers are connected",
    )? as usize;
    let wait_timeout = p.f64(
        &node,
        "wait_timeout_s",
        30.0,
        0.0,
        3600.0,
        "Start anyway after this long (s)",
    )?;
    let linger = p.f64(
        &node,
        "linger_s",
        3.0,
        0.0,
        600.0,
        "Stay up this long after the last scan (s)",
    )?;
    let lockstep = p.bool(&node, "lockstep", false, "Publish a scan only after the previous one's registration has come back (tests; says nothing about real time)")?;
    let registration_topic = p.string(
        &node,
        "topics.registration",
        "registration",
        "Lockstep: the scan matcher's registration topic",
    )?;
    let lockstep_timeout = p.f64(
        &node,
        "lockstep_timeout_s",
        5.0,
        0.1,
        3600.0,
        "Lockstep: give up waiting for a registration after this long (s)",
    )?;
    let odom_tf = p.string(
        &node,
        "odom_tf",
        "none",
        "none | identity: publish odom -> frame_id as the identity before each scan (test scaffolding, see the module documentation)",
    )?;
    let odom_frame = p.string(
        &node,
        "frames.odom",
        "odom",
        "Parent frame of the stationary odometry",
    )?;
    let odom_offset = p.f64(
        &node,
        "odom_tf_time_offset_s",
        -0.06,
        -1.0,
        0.0,
        "Seconds added to a scan's stamp to stamp the stationary odometry (where pose_graph_node looks it up)",
    )?;
    let max_scans = p.i64(
        &node,
        "max_scans",
        0,
        0,
        10_000_000,
        "Publish only the first this many scans (0: all)",
    )? as usize;
    if file == "none" {
        log_error!(node.logger(), "parameter file is required");
        return Err(RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        });
    }
    let scans = load(&file, &FrameId::new(&frame_id), max_scans).map_err(|e| {
        log_error!(node.logger(), "{e}");
        RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        }
    })?;
    if odom_tf != "none" && odom_tf != "identity" {
        log_error!(
            node.logger(),
            "odom_tf must be none or identity, not {odom_tf:?}"
        );
        return Err(RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        });
    }
    let total = scans.len();
    let shared = Shared::new(State {
        scans,
        next: 0,
        registered_ns: i64::MIN,
        last_sent: Instant::now(),
        started: false,
        created: Instant::now(),
        finished: None,
        publisher: node.create_publisher::<sensor_msgs::msg::LaserScan>(
            topic.as_str().qos(qos::sensor_data()),
        )?,
        odom: if odom_tf == "identity" {
            Some((
                TfBroadcaster::new(&node)?,
                FrameId::new(&odom_frame),
                FrameId::new(&frame_id),
                odom_offset,
            ))
        } else {
            None
        },
    });
    let _registration_sub = if lockstep {
        let shared = shared.clone();
        Some(
            node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
                registration_topic.as_str().qos(
                    rclrs::QoSProfile::default()
                        .reliable()
                        .volatile()
                        .keep_last(200),
                ),
                move |msg: geometry_msgs::msg::PoseWithCovarianceStamped| {
                    let mut guard = shared.lock();
                    let st: &mut State = &mut guard;
                    st.registered_ns = st
                        .registered_ns
                        .max(time_from_ros(&msg.header.stamp).as_nanos());
                },
            )?,
        )
    } else {
        None
    };
    let commands = Arc::clone(executor.commands());
    let n = Arc::clone(&node);
    let _timer = node.create_timer_repeating(Duration::from_secs_f64(1.0 / rate), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            if !st.started {
                let connected = st.publisher.get_subscription_count().unwrap_or(0);
                if connected >= wait_subscribers
                    || st.created.elapsed().as_secs_f64() >= wait_timeout
                {
                    if connected < wait_subscribers {
                        log_warn!(
                            n.logger(),
                            "starting with {connected} subscriber(s), wanted {wait_subscribers}"
                        );
                    }
                    log_info!(
                        n.logger(),
                        "replaying {total} scans at {rate} Hz, {connected} subscriber(s) connected"
                    );
                    st.started = true;
                } else {
                    return;
                }
            }
            if lockstep && st.next > 0 && st.next < st.scans.len() {
                let previous = time_from_ros(&st.scans[st.next - 1].header.stamp).as_nanos();
                if st.registered_ns < previous
                    && st.last_sent.elapsed().as_secs_f64() < lockstep_timeout
                {
                    return;
                }
            }
            if st.next < st.scans.len() {
                if let Some((tf, parent, child, offset)) = st.odom.as_ref() {
                    let at = time_from_ros(&st.scans[st.next].header.stamp).plus_secs_f64(*offset);
                    if let Err(e) = tf.send(at, parent, child, &Se2::IDENTITY) {
                        log_error!(n.logger(), "stationary odometry publish failed: {e}");
                    }
                }
                st.last_sent = Instant::now();
                if let Err(e) = st.publisher.publish(&st.scans[st.next]) {
                    log_error!(n.logger(), "publish failed: {e}");
                }
                st.next += 1;
                if st.next == st.scans.len() {
                    st.finished = Some(Instant::now());
                    log_info!(n.logger(), "replay finished: {} scans published", st.next);
                }
            } else if st
                .finished
                .is_some_and(|t| t.elapsed().as_secs_f64() >= linger)
            {
                commands.halt_spinning();
            }
        }
    })?;
    log_info!(
        node.logger(),
        "scan_replayer up: {file} -> {topic}, lockstep {lockstep}, stationary odometry {odom_tf}, {} parameters",
        p.len()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    drop(p);
    result
}
