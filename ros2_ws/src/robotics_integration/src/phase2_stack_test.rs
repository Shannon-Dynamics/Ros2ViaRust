//! `phase2_stack_test` — end-to-end test of the Phase 2 stack, driven from Rust.
//!
//! ```text
//!  1. spawn  `ros2 launch robotics_bringup estimator_sim.launch.xml` in its own process group
//!  2. observe the graph with an rclrs node: nodes, topics + types, publisher QoS,
//!     message rates, scan content, stamp consistency, TF (through TfListener),
//!     TF authority, estimator accuracy against ground truth
//!  3. SIGINT the launch process group (= Ctrl-C) and require: launch exits 0, every node
//!     "finished cleanly", no SIGTERM/SIGKILL escalation, no orphaned node processes
//!  4. print the check table; exit code 0 only if every check passed
//! ```
//!
//! Usage (from a sourced workspace, see `scripts/integration_test_phase2.sh`):
//!
//! ```text
//! ros2 run robotics_integration phase2_stack_test [--measure-secs 10] [--startup-timeout 40]
//!     [--log <launch log path>] [--attach] [-- <extra launch args>]
//! ```
//!
//! `--attach` skips steps 1 and 3 and tests an already running stack.

use rclrs::*;
use robotics_core::{FrameId, LookupTime, Manifold, Se2, Timestamp, TransformBuffer};
use robotics_integration::{rate_within, CheckReport, ExpectedQos, LaunchSummary, TopicCounter};
use robotics_ros_bridge::convert::{
    laser_scan_from_ros, odometry_from_ros, pose_from_ros, time_from_ros,
};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::{qos, TfListener};
use ros_env::{geometry_msgs, nav_msgs, sensor_msgs};
use std::collections::{BTreeMap, VecDeque};
use std::os::unix::process::CommandExt;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

const NODES: [&str; 4] = [
    "sim_node",
    "wheel_odometry_node",
    "state_estimator_node",
    "estimator_evaluator",
];

struct Opts {
    measure_secs: f64,
    startup_timeout: f64,
    log: String,
    attach: bool,
    launch_args: Vec<String>,
}

fn parse_args() -> Opts {
    let mut o = Opts {
        measure_secs: 10.0,
        startup_timeout: 40.0,
        log: format!(
            "{}/phase2_integration/launch.log",
            std::env::var("HOME").unwrap_or_else(|_| "/tmp".into())
        ),
        attach: false,
        launch_args: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--measure-secs" => {
                o.measure_secs = it.next().and_then(|v| v.parse().ok()).unwrap_or(10.0)
            }
            "--startup-timeout" => {
                o.startup_timeout = it.next().and_then(|v| v.parse().ok()).unwrap_or(40.0)
            }
            "--log" => o.log = it.next().unwrap_or(o.log),
            "--attach" => o.attach = true,
            "--" => o.launch_args.extend(it.by_ref()),
            // `ros2 run` appends --ros-args ...; stop parsing there.
            "--ros-args" => break,
            other => eprintln!("ignoring unknown argument {other}"),
        }
    }
    o
}

#[derive(Default)]
struct Observed {
    counters: BTreeMap<&'static str, TopicCounter>,
    scan_frame: String,
    scan_beams: usize,
    scan_min_valid: usize,
    scan_errors: u64,
    scan_stamp: Option<Timestamp>,
    truth: VecDeque<(Timestamp, Se2)>,
    truth_latest: Option<Timestamp>,
    odom_frames: Option<(String, String)>,
    estimate_frame: String,
    odom_errors: Vec<f64>,
    odom_unmatched: u64,
}

impl Observed {
    fn record(&mut self, topic: &'static str, stamp: Timestamp) {
        self.counters.entry(topic).or_default().record(stamp);
    }
}

fn spawn_launch(o: &Opts) -> std::io::Result<Child> {
    if let Some(dir) = std::path::Path::new(&o.log).parent() {
        std::fs::create_dir_all(dir)?;
    }
    let log = std::fs::File::create(&o.log)?;
    let mut cmd = Command::new("ros2");
    cmd.args(["launch", "robotics_bringup", "estimator_sim.launch.xml"])
        .args(&o.launch_args)
        .env("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
        .stdin(Stdio::null())
        .stdout(log.try_clone()?)
        .stderr(log)
        // Own process group, like a terminal job: SIGINT to the group = Ctrl-C.
        .process_group(0);
    cmd.spawn()
}

fn signal_group(pgid: u32, sig: &str) {
    let _ = Command::new("kill")
        .args([sig, "--", &format!("-{pgid}")])
        .status();
}

fn orphaned_nodes() -> Vec<String> {
    let out = Command::new("pgrep")
        .args(["-af", "lib/robotics_(sim_bridge|estimation|examples)/"])
        .output();
    match out {
        Ok(o) => String::from_utf8_lossy(&o.stdout)
            .lines()
            .map(str::to_owned)
            .collect(),
        Err(_) => Vec::new(),
    }
}

fn spin_for(executor: &mut Executor, secs: f64, stop: &std::sync::atomic::AtomicBool) {
    let until = Instant::now() + Duration::from_secs_f64(secs);
    while Instant::now() < until && !stop.load(Ordering::SeqCst) {
        let _ = executor.spin(SpinOptions::default().timeout(Duration::from_millis(50)));
    }
}

fn main() {
    let o = parse_args();
    let code = match run(&o) {
        Ok(true) => 0,
        Ok(false) => 1,
        Err(e) => {
            eprintln!("phase2_stack_test: {e}");
            2
        }
    };
    std::process::exit(code);
}

fn run(o: &Opts) -> Result<bool, Box<dyn std::error::Error>> {
    let mut report = CheckReport::default();
    let mut launch = if o.attach {
        None
    } else {
        let child = spawn_launch(o)?;
        println!(
            "launched estimator_sim.launch.xml (pgid {}), log: {}",
            child.id(),
            o.log
        );
        Some(child)
    };

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let stop = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("phase2_stack_test")?;
    let obs = Shared::new(Observed::default());

    // ---------------------------------------------------------------- subscriptions
    let _scan = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        "scan".qos(qos::sensor_data()),
        {
            let obs = obs.clone();
            move |m: sensor_msgs::msg::LaserScan| {
                let mut o = obs.lock();
                match laser_scan_from_ros(&m) {
                    Ok(s) => {
                        let valid = s.valid_beams().count();
                        o.scan_min_valid = if o.scan_beams == 0 {
                            valid
                        } else {
                            o.scan_min_valid.min(valid)
                        };
                        o.scan_beams = s.len();
                        o.scan_frame = s.frame.as_str().to_owned();
                        o.scan_stamp = Some(s.stamp);
                        o.record("/scan", s.stamp);
                    }
                    Err(_) => o.scan_errors += 1,
                }
            }
        },
    )?;
    let _imu = node.create_subscription::<sensor_msgs::msg::Imu, _>(
        "imu/data".qos(qos::sensor_data()),
        {
            let obs = obs.clone();
            move |m: sensor_msgs::msg::Imu| {
                obs.lock()
                    .record("/imu/data", time_from_ros(&m.header.stamp))
            }
        },
    )?;
    let _joints = node.create_subscription::<sensor_msgs::msg::JointState, _>(
        "joint_states".qos(qos::sensor_data()),
        {
            let obs = obs.clone();
            move |m: sensor_msgs::msg::JointState| {
                obs.lock()
                    .record("/joint_states", time_from_ros(&m.header.stamp))
            }
        },
    )?;
    let _wheel = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        "wheel/odometry".qos(qos::odometry()),
        {
            let obs = obs.clone();
            move |m: nav_msgs::msg::Odometry| {
                obs.lock()
                    .record("/wheel/odometry", time_from_ros(&m.header.stamp))
            }
        },
    )?;
    let _truth = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        "ground_truth/odom".qos(qos::odometry()),
        {
            let obs = obs.clone();
            move |m: nav_msgs::msg::Odometry| {
                if let Ok(t) = odometry_from_ros(&m) {
                    let mut o = obs.lock();
                    o.truth.push_back((t.stamp, t.pose));
                    while o.truth.len() > 4000 {
                        o.truth.pop_front();
                    }
                    o.truth_latest = Some(t.stamp);
                    o.record("/ground_truth/odom", t.stamp);
                }
            }
        },
    )?;
    let _odom =
        node.create_subscription::<nav_msgs::msg::Odometry, _>("odom".qos(qos::odometry()), {
            let obs = obs.clone();
            move |m: nav_msgs::msg::Odometry| {
                if let Ok(e) = odometry_from_ros(&m) {
                    let mut o = obs.lock();
                    o.odom_frames = Some((
                        e.frame.as_str().to_owned(),
                        e.child_frame.as_str().to_owned(),
                    ));
                    let truth = o
                        .truth
                        .iter()
                        .rev()
                        .find(|(s, _)| (*s - e.stamp).abs() < 1e-3)
                        .map(|(_, p)| *p);
                    match truth {
                        Some(t) => o.odom_errors.push(e.pose.distance_to(&t)),
                        None => o.odom_unmatched += 1,
                    }
                    o.record("/odom", e.stamp);
                }
            }
        })?;
    let _estimate = node.create_subscription::<geometry_msgs::msg::PoseWithCovarianceStamped, _>(
        "state_estimate".qos(qos::odometry()),
        {
            let obs = obs.clone();
            move |m: geometry_msgs::msg::PoseWithCovarianceStamped| {
                if pose_from_ros(&m.pose.pose).is_ok() {
                    let mut o = obs.lock();
                    o.estimate_frame = m.header.frame_id.clone();
                    o.record("/state_estimate", time_from_ros(&m.header.stamp));
                }
            }
        },
    )?;
    let buffer = Shared::new(TransformBuffer::default());
    let tf = TfListener::new(&node, buffer)?;

    // ---------------------------------------------------------------- startup
    let expected_topics: [(&str, &str); 8] = [
        ("/scan", "sensor_msgs/msg/LaserScan"),
        ("/imu/data", "sensor_msgs/msg/Imu"),
        ("/joint_states", "sensor_msgs/msg/JointState"),
        ("/wheel/odometry", "nav_msgs/msg/Odometry"),
        ("/odom", "nav_msgs/msg/Odometry"),
        (
            "/state_estimate",
            "geometry_msgs/msg/PoseWithCovarianceStamped",
        ),
        ("/tf", "tf2_msgs/msg/TFMessage"),
        ("/tf_static", "tf2_msgs/msg/TFMessage"),
    ];
    let data_topics = [
        "/scan",
        "/imu/data",
        "/joint_states",
        "/wheel/odometry",
        "/odom",
        "/state_estimate",
    ];
    let t0 = Instant::now();
    let mut ready = false;
    while t0.elapsed().as_secs_f64() < o.startup_timeout && !stop.load(Ordering::SeqCst) {
        spin_for(&mut executor, 0.5, &stop);
        let names: Vec<String> = node
            .get_node_names()
            .map(|v| v.into_iter().map(|n| n.name).collect())
            .unwrap_or_default();
        let nodes_up = NODES.iter().all(|n| names.iter().any(|m| m == n));
        let data_up = {
            let ob = obs.lock();
            data_topics
                .iter()
                .all(|t| ob.counters.get(t).is_some_and(|c| c.total > 0))
        };
        let tf_up = tf.can_transform("odom", "laser", LookupTime::Latest);
        if nodes_up && data_up && tf_up {
            ready = true;
            break;
        }
    }
    report.check(
        "startup: nodes, data and TF available",
        ready,
        format!(
            "after {:.1} s (timeout {:.0} s)",
            t0.elapsed().as_secs_f64(),
            o.startup_timeout
        ),
    );

    // ---------------------------------------------------------------- graph
    let names: Vec<(String, String)> = node
        .get_node_names()?
        .into_iter()
        .map(|n| (n.namespace, n.name))
        .collect();
    for n in NODES {
        let found = names.iter().filter(|(_, m)| m == n).count();
        report.check(
            format!("node /{n} is running exactly once"),
            found == 1,
            format!("{found} instance(s)"),
        );
    }
    let topics = node.get_topic_names_and_types()?;
    for (t, ty) in expected_topics {
        let types = topics.get(t).cloned().unwrap_or_default();
        report.check(
            format!("topic {t} has type {ty}"),
            types == vec![ty.to_string()],
            format!("{types:?}"),
        );
    }

    let qos_expect: [(&str, &str, rclrs::QoSProfile); 8] = [
        ("/scan", "sim_node", qos::sensor_data()),
        ("/imu/data", "sim_node", qos::sensor_data()),
        ("/joint_states", "sim_node", qos::sensor_data()),
        ("/wheel/odometry", "wheel_odometry_node", qos::odometry()),
        ("/odom", "state_estimator_node", qos::odometry()),
        ("/state_estimate", "state_estimator_node", qos::odometry()),
        ("/tf", "state_estimator_node", qos::tf()),
        ("/tf_static", "sim_node", qos::tf_static()),
    ];
    for (topic, owner, profile) in qos_expect {
        let pubs = node.get_publishers_info_by_topic(topic)?;
        let names: Vec<&str> = pubs.iter().map(|p| p.node_name.as_str()).collect();
        let owner_ok = names == vec![owner];
        report.check(
            format!("{topic}: single publisher {owner}"),
            owner_ok,
            format!("publishers {names:?}"),
        );
        let exp = ExpectedQos::from_profile(&profile);
        let qos_result = pubs
            .iter()
            .map(|p| exp.matches(&p.qos_profile))
            .collect::<Result<Vec<_>, _>>();
        report.check(
            format!(
                "{topic}: publisher QoS {:?}/{:?}/KEEP_LAST {}",
                exp.reliability, exp.durability, exp.depth
            ),
            !pubs.is_empty() && qos_result.is_ok(),
            qos_result.err().unwrap_or_else(|| "ok".into()),
        );
    }

    // ---------------------------------------------------------------- rates
    {
        let mut ob = obs.lock();
        for c in ob.counters.values_mut() {
            c.reset_window();
        }
        ob.odom_errors.clear();
    }
    let t_measure = Instant::now();
    spin_for(&mut executor, o.measure_secs, &stop);
    let window = t_measure.elapsed().as_secs_f64();
    {
        let ob = obs.lock();
        for (topic, nominal) in [
            ("/scan", 10.0),
            ("/imu/data", 100.0),
            ("/joint_states", 50.0),
            ("/wheel/odometry", 50.0),
            ("/odom", 50.0),
            ("/state_estimate", 50.0),
            ("/ground_truth/odom", 100.0),
        ] {
            let c = ob.counters.get(topic).cloned().unwrap_or_default();
            let hz = c.rate(window);
            report.check(
                format!("{topic}: rate {nominal} Hz ±15 %"),
                rate_within(hz, nominal, 0.15),
                format!("{hz:.2} Hz ({} msgs / {window:.1} s)", c.count),
            );
            report.check(
                format!("{topic}: header stamps strictly increasing"),
                c.count > 0 && c.non_monotonic == 0,
                format!("{} non-monotonic", c.non_monotonic),
            );
        }

        // ------------------------------------------------------------ content
        report.check(
            "/scan: 360 beams in frame 'laser', >150 valid returns",
            ob.scan_beams == 360
                && ob.scan_frame == "laser"
                && ob.scan_min_valid > 150
                && ob.scan_errors == 0,
            format!(
                "{} beams, frame '{}', min valid {}, malformed {}",
                ob.scan_beams, ob.scan_frame, ob.scan_min_valid, ob.scan_errors
            ),
        );
        let lag = match (ob.scan_stamp, ob.truth_latest) {
            (Some(s), Some(t)) => (t - s).abs(),
            _ => f64::INFINITY,
        };
        report.check(
            "/scan stamps on the simulator clock (|scan − truth| < 0.5 s)",
            lag < 0.5,
            format!("{lag:.3} s"),
        );
        report.check(
            "/odom frames odom -> base_link; /state_estimate in odom",
            ob.odom_frames == Some(("odom".into(), "base_link".into()))
                && ob.estimate_frame == "odom",
            format!("{:?}, '{}'", ob.odom_frames, ob.estimate_frame),
        );
        let n = ob.odom_errors.len();
        let rmse = (ob.odom_errors.iter().map(|e| e * e).sum::<f64>() / n.max(1) as f64).sqrt();
        report.check(
            "/odom matches ground truth by stamp; position RMSE < 0.5 m",
            n > 100 && rmse < 0.5,
            format!(
                "{n} matched, {} unmatched, RMSE {rmse:.4} m",
                ob.odom_unmatched
            ),
        );
    }

    // ---------------------------------------------------------------- TF
    let odom_base = tf.lookup_transform("odom", "base_link", LookupTime::Latest);
    report.check(
        "TF odom -> base_link resolves (latest)",
        odom_base.is_ok(),
        match &odom_base {
            Ok(t) => format!(
                "x={:.3} y={:.3} th={:.3} @ {:.3}",
                t.transform.x(),
                t.transform.y(),
                t.transform.theta(),
                t.stamp.as_secs_f64()
            ),
            Err(e) => e.to_string(),
        },
    );
    let base_laser = tf.lookup_transform("base_link", "laser", LookupTime::Latest);
    report.check(
        "TF base_link -> laser is the static mount (0.05, 0, 0)",
        base_laser
            .as_ref()
            .is_ok_and(|t| t.transform.boxminus(&Se2::new(0.05, 0.0, 0.0)).norm() < 1e-9),
        format!("{:?}", base_laser.as_ref().map(|t| t.transform)),
    );
    let chain = tf.lookup_transform("odom", "laser", LookupTime::Latest);
    let chain_ok = match (&chain, &odom_base, &base_laser) {
        (Ok(c), Ok(a), Ok(b)) if c.stamp == a.stamp => {
            c.transform
                .boxminus(&a.transform.compose(&b.transform))
                .norm()
                < 1e-9
        }
        (Ok(c), _, Ok(b)) => {
            // odom->base_link may have advanced between the two lookups; re-evaluate it at
            // the chain's stamp.
            tf.lookup_transform("odom", "base_link", LookupTime::At(c.stamp))
                .is_ok_and(|a| {
                    c.transform
                        .boxminus(&a.transform.compose(&b.transform))
                        .norm()
                        < 1e-9
                })
        }
        _ => false,
    };
    report.check(
        "TF odom -> laser chain = (odom->base_link)(base_link->laser)",
        chain_ok,
        format!("{:?}", chain.as_ref().map(|t| t.transform)),
    );
    let past = odom_base
        .as_ref()
        .map(|t| t.stamp.plus_secs_f64(-0.5))
        .map(|s| {
            tf.lookup_transform("odom", "base_link", LookupTime::At(s))
                .is_ok()
        })
        .unwrap_or(false);
    report.check(
        "TF interpolates odom -> base_link 0.5 s in the past",
        past,
        "",
    );
    let conflicts = tf.authority_conflicts();
    let stats = tf.stats();
    report.check(
        "TF: every frame has exactly one publisher (no authority conflict)",
        conflicts.is_empty()
            && stats.publisher_count("base_link") == 1
            && stats.publisher_count("laser") == 1,
        format!(
            "conflicts {conflicts:?}; base_link {} pub, laser {} pub, imu_link {} pub; rejected {}",
            stats.publisher_count("base_link"),
            stats.publisher_count("laser"),
            stats.publisher_count("imu_link"),
            stats.rejected
        ),
    );
    let frames: Vec<String> = tf
        .buffer()
        .lock()
        .frames()
        .iter()
        .map(FrameId::to_string)
        .collect();
    report.check(
        "TF tree is odom -> base_link -> {laser, imu_link}",
        frames == ["base_link", "imu_link", "laser", "odom"],
        format!("{frames:?}"),
    );

    // ---------------------------------------------------------------- shutdown
    drop(tf);
    if let Some(child) = launch.as_mut() {
        let pgid = child.id();
        println!("sending SIGINT to launch process group {pgid} (Ctrl-C)");
        signal_group(pgid, "-INT");
        let t_sig = Instant::now();
        let mut status = None;
        while t_sig.elapsed() < Duration::from_secs(20) {
            if let Some(s) = child.try_wait()? {
                status = Some(s);
                break;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        let elapsed = t_sig.elapsed().as_secs_f64();
        if status.is_none() {
            signal_group(pgid, "-KILL");
            let _ = child.wait();
        }
        report.check(
            "shutdown: ros2 launch exits with code 0 after SIGINT",
            status.is_some_and(|s| s.success()),
            format!("{status:?} after {elapsed:.2} s"),
        );
        let log = std::fs::read_to_string(&o.log).unwrap_or_default();
        let summary = LaunchSummary::parse(&log);
        report.check(
            "shutdown: every node finished cleanly, no SIGTERM/SIGKILL escalation",
            summary.clean() && summary.started == NODES.len(),
            format!("{summary:?}"),
        );
        let orphans = orphaned_nodes();
        report.check(
            "shutdown: no orphaned node processes",
            orphans.is_empty(),
            if orphans.is_empty() {
                "none".into()
            } else {
                orphans.join("; ")
            },
        );
    }

    print!("{}", report.render());
    Ok(report.passed())
}
