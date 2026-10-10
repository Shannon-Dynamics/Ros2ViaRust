#!/bin/bash
# Rust-first integration test of the Phase 2 stack (the framework's launch_testing
# equivalent). Run from the WSL mirror root after `colcon build`:
#
#   bash scripts/integration_test_phase2.sh [--measure-secs N] [--startup-timeout N] [-- launch args]
#
# The test binary (ros2_ws/src/robotics_integration) spawns
# `ros2 launch robotics_bringup estimator_sim.launch.xml` in its own process group, checks
# nodes, topics, types, publisher QoS, rates, scan content, stamps, TF (through the Rust
# TfListener), TF authority and estimator accuracy, then sends SIGINT (Ctrl-C) to the
# launch and requires a clean exit of every process. Exit code 0 = all checks passed.
#
# Output: ~/phase2_integration/report.log (check table), ~/phase2_integration/launch.log.
set +u
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
OUT="$HOME/phase2_integration"
mkdir -p "$OUT"
source scripts/env.sh > /dev/null 2>&1
source ros2_ws/install/setup.bash
export RCUTILS_LOGGING_BUFFERED_STREAM=0
# Clean slate: no node processes from earlier runs.
pkill -f "lib/robotics_(sim_bridge|estimation|examples)/" 2>/dev/null
ros2 daemon stop > /dev/null 2>&1
ros2 run robotics_integration phase2_stack_test --log "$OUT/launch.log" "$@" 2>&1 | tee "$OUT/report.log"
rc=${PIPESTATUS[0]}
echo "phase2_stack_test exit code: $rc"
exit "$rc"
