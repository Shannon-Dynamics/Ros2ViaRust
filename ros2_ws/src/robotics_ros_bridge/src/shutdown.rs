//! Graceful shutdown on SIGINT / SIGTERM.
//!
//! rclrs 0.8 installs no signal handler (`Context::ok` documents that one is still to
//! come), so a spinning executor ignores Ctrl-C and `ros2 launch` has to escalate to
//! SIGTERM/SIGKILL. Every node in this framework calls [`install`] once so that the first
//! signal halts the executor, `spin` returns, and `main` can exit cleanly.

use rclrs::{Executor, RclrsError, SpinOptions};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Spin the executor until it is halted by a signal or fails.
///
/// * Errors reported *after* a shutdown signal (typically a `Timeout` from the wait set
///   being torn down) are expected and swallowed.
/// * **Spurious timeouts are resumed.** With CycloneDDS, `rcl_wait` occasionally returns
///   `RCL_RET_TIMEOUT` although no timeout was requested, typically when another DDS
///   participant (e.g. a `ros2` CLI) joins or leaves. rclrs 0.8 treats that as fatal for the
///   wait-set runner; rclcpp simply waits again. When every error of a returned spin is a
///   `Timeout` and no shutdown was requested, this function spins again (the number of
///   resumes is logged, rate-limited).
/// * Any other error is returned so a genuine failure still terminates the node.
///
/// Only primitives on the node's own wait set are protected by this loop, which is why the
/// framework's nodes do not use rclrs `Worker`s (see [`crate::shared`]).
pub fn spin(executor: &mut Executor, shutdown_flag: &AtomicBool) -> Result<(), RclrsError> {
    let mut resumes: u64 = 0;
    let mut last_report = std::time::Instant::now() - std::time::Duration::from_secs(60);
    loop {
        let errors = executor.spin(SpinOptions::default());
        if shutdown_flag.load(Ordering::SeqCst) {
            return Ok(());
        }
        if errors.is_empty() {
            return Ok(());
        }
        let all_timeouts = errors.iter().all(|e| {
            matches!(
                e,
                RclrsError::RclError {
                    code: rclrs::RclReturnCode::Timeout,
                    ..
                }
            )
        });
        if !all_timeouts {
            return Err(errors.into_iter().next().unwrap_or(RclrsError::RclError {
                code: rclrs::RclReturnCode::Error,
                msg: None,
            }));
        }
        resumes += 1;
        if last_report.elapsed() >= std::time::Duration::from_secs(10) {
            rclrs::log_warn!(
                "robotics_ros_bridge.spin",
                "rcl_wait returned a spurious timeout (DDS graph change); resumed spinning ({resumes} time(s) so far)"
            );
            last_report = std::time::Instant::now();
        }
    }
}

/// Install a process-wide SIGINT/SIGTERM handler that stops `executor`.
///
/// Returns a flag that becomes `true` once a signal has been received, for callbacks
/// that want to check it. Installing twice in one process is an error from the
/// underlying `ctrlc` crate and is reported, not ignored.
pub fn install(executor: &Executor) -> Result<Arc<AtomicBool>, RclrsError> {
    let flag = Arc::new(AtomicBool::new(false));
    let commands = Arc::clone(executor.commands());
    let flag_in_handler = Arc::clone(&flag);
    ctrlc::set_handler(move || {
        flag_in_handler.store(true, Ordering::SeqCst);
        commands.halt_spinning();
    })
    .map_err(|e| {
        rclrs::log_error!(
            "robotics_ros_bridge",
            "failed to install signal handler: {e}"
        );
        RclrsError::RclError {
            code: rclrs::RclReturnCode::Error,
            msg: None,
        }
    })?;
    Ok(flag)
}
