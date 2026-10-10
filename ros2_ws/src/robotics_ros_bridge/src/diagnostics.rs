//! `diagnostic_msgs/DiagnosticArray` publishing with a small status builder.
//!
//! Nodes report health at ~1 Hz: update rate, dropped messages, convergence, timing.
//! Per-cycle logging is deliberately *not* done through this path.

use crate::convert::time_to_ros;
use crate::qos;
use rclrs::{IntoPrimitiveOptions, Node, Publisher, RclrsError};
use robotics_core::Timestamp;
use ros_env::{diagnostic_msgs, std_msgs};

/// Severity, mirroring `diagnostic_msgs/DiagnosticStatus` levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticLevel {
    /// Everything nominal.
    Ok,
    /// Degraded but operating.
    Warn,
    /// Not operating correctly.
    Error,
    /// No recent data.
    Stale,
}

impl DiagnosticLevel {
    fn code(self) -> u8 {
        match self {
            DiagnosticLevel::Ok => 0,
            DiagnosticLevel::Warn => 1,
            DiagnosticLevel::Error => 2,
            DiagnosticLevel::Stale => 3,
        }
    }
}

/// Builder for one `DiagnosticStatus`.
pub struct StatusBuilder {
    status: diagnostic_msgs::msg::DiagnosticStatus,
}

impl StatusBuilder {
    /// Start a status for component `name` (conventionally `node_name: subsystem`).
    pub fn new(name: &str, level: DiagnosticLevel, message: &str) -> Self {
        Self {
            status: diagnostic_msgs::msg::DiagnosticStatus {
                level: level.code(),
                name: name.to_owned(),
                message: message.to_owned(),
                hardware_id: String::new(),
                values: Vec::new(),
            },
        }
    }

    /// Set the hardware id field.
    pub fn hardware_id(mut self, id: &str) -> Self {
        self.status.hardware_id = id.to_owned();
        self
    }

    /// Append a key/value pair.
    pub fn value(mut self, key: &str, value: impl ToString) -> Self {
        self.status.values.push(diagnostic_msgs::msg::KeyValue {
            key: key.to_owned(),
            value: value.to_string(),
        });
        self
    }

    /// Finish.
    pub fn build(self) -> diagnostic_msgs::msg::DiagnosticStatus {
        self.status
    }
}

/// Publisher on `/diagnostics`.
pub struct DiagnosticsPublisher {
    publisher: Publisher<diagnostic_msgs::msg::DiagnosticArray>,
}

impl DiagnosticsPublisher {
    /// Create the publisher on `/diagnostics` with the diagnostics QoS.
    pub fn new(node: &Node) -> Result<Self, RclrsError> {
        let publisher = node.create_publisher::<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics".qos(qos::diagnostics()),
        )?;
        Ok(Self { publisher })
    }

    /// Publish a set of statuses stamped `stamp`.
    pub fn publish(
        &self,
        stamp: Timestamp,
        statuses: Vec<diagnostic_msgs::msg::DiagnosticStatus>,
    ) -> Result<(), RclrsError> {
        let msg = diagnostic_msgs::msg::DiagnosticArray {
            header: std_msgs::msg::Header {
                stamp: time_to_ros(stamp),
                frame_id: String::new(),
            },
            status: statuses,
        };
        self.publisher.publish(msg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builder_fills_fields() {
        let s = StatusBuilder::new("wheel_odometry: rate", DiagnosticLevel::Warn, "slow")
            .hardware_id("base")
            .value("hz", 12.5)
            .value("dropped", 3)
            .build();
        assert_eq!(s.level, 1);
        assert_eq!(s.name, "wheel_odometry: rate");
        assert_eq!(s.values.len(), 2);
        assert_eq!(s.values[0].key, "hz");
        assert_eq!(s.values[1].value, "3");
    }
}
