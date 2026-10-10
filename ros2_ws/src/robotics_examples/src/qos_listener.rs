//! `qos_listener` — subscriber with explicit QoS, message metadata and a rate report.
//!
//! Parameters
//! * `reliable` (bool, read-only, default true): must match `qos_talker`.
//! * `report_period_s` (int, default 5): how often the receive rate is logged.
//!
//! Subscribes to `std_msgs/String` on `chatter`.

use rclrs::*;
use robotics_ros_bridge::qos;
use robotics_ros_bridge::shared::Shared;
use ros_env::std_msgs;
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Default)]
struct Stats {
    received: u64,
    last: Option<String>,
    window_start: Option<Instant>,
    window_count: u64,
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("qos_listener")?;

    let reliable: ReadOnlyParameter<bool> = node
        .declare_parameter("reliable")
        .default(true)
        .description("true: RELIABLE (command QoS); false: BEST_EFFORT (sensor QoS)")
        .read_only()?;
    let report_period_s: ReadOnlyParameter<i64> = node
        .declare_parameter("report_period_s")
        .default(5)
        .range(ParameterRange {
            lower: Some(1),
            upper: Some(600),
            step: None,
        })
        .read_only()?;

    let profile = if reliable.get() {
        qos::command()
    } else {
        qos::sensor_data()
    };
    let shared = Shared::new(Stats::default());

    let node_for_sub = Arc::clone(&node);
    let _subscription =
        node.create_subscription::<std_msgs::msg::String, _>("chatter".qos(profile), {
            let shared = shared.clone();
            move |msg: std_msgs::msg::String, info: MessageInfo| {
                let mut guard = shared.lock();
                let stats: &mut Stats = &mut guard;
                {
                    stats.received += 1;
                    stats.window_count += 1;
                    stats.window_start.get_or_insert_with(Instant::now);
                    // MessageInfo carries the DDS timestamps; the sequence numbers are u64::MAX
                    // on RMW implementations that do not support them (CycloneDDS), so only the
                    // latency is reported.
                    let latency_ms = info
                        .source_timestamp
                        .and_then(|src| std::time::SystemTime::now().duration_since(src).ok())
                        .map(|d| d.as_secs_f64() * 1e3);
                    log_info!(
                        node_for_sub.logger(),
                        "#{} heard \"{}\" (latency {})",
                        stats.received,
                        msg.data,
                        latency_ms
                            .map(|ms| format!("{ms:.2} ms"))
                            .unwrap_or_else(|| "n/a".into())
                    );
                    stats.last = Some(msg.data);
                }
            }
        })?;

    let node_for_timer = Arc::clone(&node);
    let _report =
        node.create_timer_repeating(Duration::from_secs(report_period_s.get() as u64), {
            let shared = shared.clone();
            move || {
                let mut guard = shared.lock();
                let stats: &mut Stats = &mut guard;
                {
                    let hz = stats
                        .window_start
                        .map(|t0| stats.window_count as f64 / t0.elapsed().as_secs_f64().max(1e-9))
                        .unwrap_or(0.0);
                    log_info!(
                        node_for_timer.logger(),
                        "rate {hz:.2} Hz over the last window, {} total, last=\"{}\"",
                        stats.received,
                        stats.last.as_deref().unwrap_or("-")
                    );
                    stats.window_start = Some(Instant::now());
                    stats.window_count = 0;
                }
            }
        })?;

    log_info!(
        node.logger(),
        "qos_listener up: topic=chatter reliable={}",
        reliable.get()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "qos_listener shutting down");
    result
}
