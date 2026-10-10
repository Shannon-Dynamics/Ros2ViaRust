//! `robotics_integration` — building blocks of the framework's Rust-first integration
//! tests (the equivalent of `launch_testing` for this Rust-only stack).
//!
//! The test binaries (`phase2_stack_test`) spawn a launch file in its own process group,
//! observe the running graph through an ordinary `rclrs` node, assert on what they see,
//! send SIGINT to the launch (Ctrl-C) and verify that every process exits cleanly. This
//! library holds the pure, unit-tested parts: the check report, QoS expectations, message
//! rate accounting, stamp monotonicity and launch-log parsing.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use rclrs::{QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy};
use robotics_core::Timestamp;
use std::time::Instant;

/// One named assertion.
#[derive(Debug, Clone, PartialEq)]
pub struct CheckResult {
    /// What was checked.
    pub name: String,
    /// Outcome.
    pub passed: bool,
    /// Measured values / reason.
    pub detail: String,
}

/// Ordered list of check results.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CheckReport {
    results: Vec<CheckResult>,
}

impl CheckReport {
    /// Record a check.
    pub fn check(&mut self, name: impl Into<String>, passed: bool, detail: impl Into<String>) {
        self.results.push(CheckResult {
            name: name.into(),
            passed,
            detail: detail.into(),
        });
    }
    /// All results.
    pub fn results(&self) -> &[CheckResult] {
        &self.results
    }
    /// Number of checks.
    pub fn len(&self) -> usize {
        self.results.len()
    }
    /// `true` without checks.
    pub fn is_empty(&self) -> bool {
        self.results.is_empty()
    }
    /// Number of failed checks.
    pub fn failures(&self) -> usize {
        self.results.iter().filter(|r| !r.passed).count()
    }
    /// `true` if at least one check ran and none failed.
    pub fn passed(&self) -> bool {
        !self.results.is_empty() && self.failures() == 0
    }
    /// Human-readable table.
    pub fn render(&self) -> String {
        let mut out = String::new();
        for r in &self.results {
            out.push_str(&format!(
                "[{}] {:<52} {}\n",
                if r.passed { "PASS" } else { "FAIL" },
                r.name,
                r.detail
            ));
        }
        out.push_str(&format!(
            "{} checks, {} passed, {} failed\n",
            self.len(),
            self.len() - self.failures(),
            self.failures()
        ));
        out
    }
}

/// Expected QoS of an endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExpectedQos {
    /// Reliability policy.
    pub reliability: QoSReliabilityPolicy,
    /// Durability policy.
    pub durability: QoSDurabilityPolicy,
    /// `KEEP_LAST` depth.
    pub depth: u32,
}

impl ExpectedQos {
    /// The expectation derived from one of the framework's presets.
    pub fn from_profile(p: &QoSProfile) -> Self {
        let depth = match p.history {
            QoSHistoryPolicy::KeepLast { depth } | QoSHistoryPolicy::SystemDefault { depth } => {
                depth
            }
            _ => 0,
        };
        Self {
            reliability: p.reliability,
            durability: p.durability,
            depth,
        }
    }

    /// Compare an endpoint's QoS with the expectation; `Err` explains the mismatch.
    pub fn matches(&self, actual: &QoSProfile) -> Result<(), String> {
        let mut problems = Vec::new();
        if actual.reliability != self.reliability {
            problems.push(format!(
                "reliability {:?} != {:?}",
                actual.reliability, self.reliability
            ));
        }
        if actual.durability != self.durability {
            problems.push(format!(
                "durability {:?} != {:?}",
                actual.durability, self.durability
            ));
        }
        match actual.history {
            QoSHistoryPolicy::KeepLast { depth } if depth == self.depth => {}
            other => problems.push(format!("history {other:?} != KeepLast({})", self.depth)),
        }
        if problems.is_empty() {
            Ok(())
        } else {
            Err(problems.join(", "))
        }
    }
}

/// Message counter for one topic over a measurement window.
#[derive(Debug, Clone, Default)]
pub struct TopicCounter {
    /// Messages received in the window.
    pub count: u64,
    /// Messages received since the start of the test.
    pub total: u64,
    /// Receipt time of the first message in the window.
    pub first: Option<Instant>,
    /// Receipt time of the latest message.
    pub last: Option<Instant>,
    /// Header stamp of the latest message.
    pub last_stamp: Option<Timestamp>,
    /// Messages whose header stamp did not increase.
    pub non_monotonic: u64,
}

impl TopicCounter {
    /// Record one message received now with header stamp `stamp`.
    pub fn record(&mut self, stamp: Timestamp) {
        self.record_at(Instant::now(), stamp);
    }

    /// Record one message received at `at`.
    pub fn record_at(&mut self, at: Instant, stamp: Timestamp) {
        if let Some(prev) = self.last_stamp {
            if stamp <= prev {
                self.non_monotonic += 1;
            }
        }
        self.count += 1;
        self.total += 1;
        self.first.get_or_insert(at);
        self.last = Some(at);
        self.last_stamp = Some(stamp);
    }

    /// Start a new measurement window (keeps totals and the last stamp).
    pub fn reset_window(&mut self) {
        self.count = 0;
        self.first = None;
        self.non_monotonic = 0;
    }

    /// Rate over `window_secs` seconds.
    pub fn rate(&self, window_secs: f64) -> f64 {
        if window_secs > 0.0 {
            self.count as f64 / window_secs
        } else {
            0.0
        }
    }
}

/// `|measured − nominal| ≤ rel_tol · nominal`.
pub fn rate_within(measured: f64, nominal: f64, rel_tol: f64) -> bool {
    (measured - nominal).abs() <= rel_tol * nominal
}

/// What `ros2 launch` reported about its processes.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LaunchSummary {
    /// `process started with pid` lines.
    pub started: usize,
    /// `process has finished cleanly` lines.
    pub finished_cleanly: usize,
    /// `process has died` lines (non-zero exit or signal).
    pub died: usize,
    /// `sending signal 'SIGTERM'` / `'SIGKILL'` escalations by launch.
    pub escalations: usize,
}

impl LaunchSummary {
    /// Parse the combined stdout/stderr of `ros2 launch`.
    pub fn parse(log: &str) -> Self {
        let mut s = Self::default();
        for line in log.lines() {
            if line.contains("process started with pid") {
                s.started += 1;
            } else if line.contains("process has finished cleanly") {
                s.finished_cleanly += 1;
            } else if line.contains("process has died") {
                s.died += 1;
            }
            if line.contains("sending signal 'SIGTERM'")
                || line.contains("sending signal 'SIGKILL'")
            {
                s.escalations += 1;
            }
        }
        s
    }

    /// Every started process finished cleanly without escalation.
    pub fn clean(&self) -> bool {
        self.started > 0
            && self.finished_cleanly == self.started
            && self.died == 0
            && self.escalations == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robotics_ros_bridge::qos;
    use std::time::Duration;

    #[test]
    fn report_counts_failures() {
        let mut r = CheckReport::default();
        assert!(!r.passed(), "an empty report is not a pass");
        r.check("a", true, "ok");
        assert!(r.passed());
        r.check("b", false, "boom");
        assert!(!r.passed());
        assert_eq!(r.failures(), 1);
        assert!(r.render().contains("[FAIL] b"));
        assert!(r.render().contains("2 checks, 1 passed, 1 failed"));
    }

    #[test]
    fn qos_expectations_follow_the_presets() {
        let sensor = ExpectedQos::from_profile(&qos::sensor_data());
        assert!(sensor.matches(&qos::sensor_data()).is_ok());
        assert!(sensor.matches(&qos::odometry()).is_err());
        let latched = ExpectedQos::from_profile(&qos::tf_static());
        let err = latched.matches(&qos::tf()).unwrap_err();
        assert!(err.contains("durability"), "{err}");
        assert_eq!(ExpectedQos::from_profile(&qos::tf()).depth, 100);
    }

    #[test]
    fn rates_and_monotonic_stamps() {
        let mut c = TopicCounter::default();
        let t0 = Instant::now();
        for i in 0..10 {
            c.record_at(
                t0 + Duration::from_millis(100 * i),
                Timestamp::from_secs_f64(i as f64),
            );
        }
        assert_eq!(c.non_monotonic, 0);
        assert!((c.rate(1.0) - 10.0).abs() < 1e-12);
        c.record_at(t0, Timestamp::from_secs_f64(3.0));
        assert_eq!(c.non_monotonic, 1);
        c.reset_window();
        assert_eq!((c.count, c.total, c.non_monotonic), (0, 11, 0));
        assert!(rate_within(9.5, 10.0, 0.1));
        assert!(!rate_within(8.5, 10.0, 0.1));
    }

    #[test]
    fn launch_log_parsing() {
        let log = "\
[INFO] [sim_node-1]: process started with pid [1]
[INFO] [state_estimator_node-2]: process started with pid [2]
[INFO] [sim_node-1]: process has finished cleanly [pid 1]
[INFO] [state_estimator_node-2]: process has finished cleanly [pid 2]
";
        let s = LaunchSummary::parse(log);
        assert_eq!(s.started, 2);
        assert!(s.clean());
        let bad = format!("{log}[ERROR] [x-3]: process started with pid [3]\n[ERROR] [x-3]: process has died [pid 3, exit code -9]\n");
        assert!(!LaunchSummary::parse(&bad).clean());
        let escalated = format!("{log}[INFO] [x]: sending signal 'SIGTERM' to process[x-3]\n");
        assert!(!LaunchSummary::parse(&escalated).clean());
    }
}
