#!/bin/bash
# Run a command inside the WSL build mirror with the full environment.
#
# Refreshes ~/ros2-rust-robotics from the Windows checkout, sources scripts/env.sh, then
# runs the given command from the mirror root. Intended to be invoked from Windows as:
#
#   wsl.exe -d Ubuntu-24.04 -- bash "/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/scripts/wsl_run.sh" cargo test
#
# Output of the command is also appended to ~/ros2-rust-robotics-last-run.log.
set +u
bash "/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/scripts/sync_to_wsl.sh" > /dev/null || exit 1
cd "$HOME/ros2-rust-robotics" || exit 1
source scripts/env.sh > /dev/null 2>&1
"$@" 2>&1 | tee "$HOME/ros2-rust-robotics-last-run.log"
exit "${PIPESTATUS[0]}"
