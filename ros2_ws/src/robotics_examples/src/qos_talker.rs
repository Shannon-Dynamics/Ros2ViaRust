//! `qos_talker` — timer-driven publisher with explicit QoS and parameters.
//!
//! Parameters
//! * `period_ms` (int, 10..10000, default 500): publish period.
//! * `reliable` (bool, read-only, default true): RELIABLE (command preset) vs
//!   BEST_EFFORT (sensor-data preset). The listener must use the same setting or the
//!   endpoints will not match.
//! * `prefix` (string, default "hello"): message text prefix; changeable at runtime with
//!   `ros2 param set /qos_talker prefix bonjour`.
//!
//! Publishes `std_msgs/String` on `chatter`.

use rclrs::*;
use robotics_ros_bridge::qos;
use robotics_ros_bridge::shared::Shared;
use ros_env::std_msgs;
use std::sync::Arc;
use std::time::Duration;

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("qos_talker")?;

    let period_ms: MandatoryParameter<i64> = node
        .declare_parameter("period_ms")
        .default(500)
        .range(ParameterRange {
            lower: Some(10),
            upper: Some(10_000),
            step: None,
        })
        .description("Publish period in milliseconds (read at startup)")
        .mandatory()?;
    let reliable: ReadOnlyParameter<bool> = node
        .declare_parameter("reliable")
        .default(true)
        .description("true: RELIABLE/volatile (command QoS); false: BEST_EFFORT (sensor QoS)")
        .read_only()?;
    let prefix: MandatoryParameter<Arc<str>> = node
        .declare_parameter("prefix")
        .default("hello".into())
        .description("Text prefix of every message; may be changed at runtime")
        .mandatory()?;

    let profile = if reliable.get() {
        qos::command()
    } else {
        qos::sensor_data()
    };
    let publisher = node.create_publisher::<std_msgs::msg::String>("chatter".qos(profile))?;

    log_info!(
        node.logger(),
        "qos_talker up: topic=chatter period_ms={} reliable={} prefix={}",
        period_ms.get(),
        reliable.get(),
        prefix.get()
    );

    // Node-level callbacks over shared state (see robotics_ros_bridge::shared).
    let shared = Shared::new(0);
    let node_for_timer = Arc::clone(&node);
    let _timer = node.create_timer_repeating(Duration::from_millis(period_ms.get() as u64), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let count: &mut u64 = &mut guard;
            {
                *count += 1;
                let msg = std_msgs::msg::String {
                    data: format!("{} {}", prefix.get(), count),
                };
                match publisher.publish(&msg) {
                    Ok(()) => log_info!(
                        node_for_timer.logger().throttle(Duration::from_secs(2)),
                        "published #{count}: \"{}\" ({} subscriber(s))",
                        msg.data,
                        publisher.get_subscription_count().unwrap_or(0)
                    ),
                    Err(e) => log_error!(node_for_timer.logger(), "publish failed: {e}"),
                }
            }
        }
    })?;

    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "qos_talker shutting down");
    result
}
