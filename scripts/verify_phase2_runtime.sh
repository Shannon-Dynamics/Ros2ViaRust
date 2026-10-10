#!/bin/bash
# Phase 2 runtime verification with the standard ROS 2 CLI: launches the simulator
# (wheels, IMU, LiDAR) + wheel odometry + ESKF stack and records the evidence quoted in
# README.md. Run from the WSL mirror root after `colcon build`:
#
#   bash scripts/verify_phase2_runtime.sh [run_seconds] [launch args...]
#
# Output is also written to ~/phase2_runtime/verify.log (launch output: launch.log).
# The automated pass/fail version of these checks is scripts/integration_test_phase2.sh.
set +u
RUN_SECONDS="${1:-60}"
shift
LAUNCH_ARGS=("$@")
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
OUT="$HOME/phase2_runtime"          # not /tmp: WSL clears /tmp when the distro stops
mkdir -p "$OUT"
LOG="$OUT/verify.log"
LAUNCH_LOG="$OUT/launch.log"
: > "$LOG"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
# The ros2 CLI is Python: unbuffered stdout, and SIGINT (not SIGTERM) from `timeout`, so a
# low-rate topic's output is not lost in a block buffer when the CLI is stopped.
export PYTHONUNBUFFERED=1
alive() {  # liveness checkpoint: latest simulator and evaluator log lines
    echo "--- liveness @ $(date +%T): $(grep -c 'vs ground truth' "$LAUNCH_LOG") evaluator reports; last sim: $(grep 'sim t=' "$LAUNCH_LOG" | tail -1 | grep -oE 'sim t=[0-9.]+ s \([0-9]+ steps, [0-9]+ scans\)')"
}
exec > >(tee -a "$LOG") 2>&1
source scripts/env.sh > /dev/null 2>&1
source ros2_ws/install/setup.bash

# Clean slate: no stale nodes from earlier runs, fresh CLI daemon.
pkill -f "lib/robotics_(sim_bridge|estimation|examples)/" 2>/dev/null
ros2 daemon stop > /dev/null 2>&1; ros2 daemon start > /dev/null 2>&1

echo "=== $(date -Is)  $(lsb_release -ds)  ROS_DISTRO=$ROS_DISTRO  RMW=$RMW_IMPLEMENTATION ==="
echo "=== launch: ros2 launch robotics_bringup estimator_sim.launch.xml (own process group) ==="
setsid ros2 launch robotics_bringup estimator_sim.launch.xml "${LAUNCH_ARGS[@]}" > "$LAUNCH_LOG" 2>&1 &
LAUNCH=$!
sleep 10

alive
echo "=== ros2 node list ==="; ros2 node list
echo "=== ros2 topic list -t ==="; ros2 topic list -t
for t in /scan /odom /state_estimate /imu/data /wheel/odometry /joint_states /tf /tf_static; do
    echo "=== ros2 topic info -v $t ==="
    ros2 topic info -v "$t" | grep -E "^Type|Publisher count|Subscription count|Node name|Endpoint type|Reliability|Durability|History"
done
for t in /scan /imu/data /joint_states /wheel/odometry /odom /state_estimate /ground_truth/odom; do
    echo "=== ros2 topic hz $t ==="
    timeout -s INT 8 ros2 topic hz "$t" --window 100 2>&1 | grep -E "average rate|min:" | tail -2
done
alive
echo "=== ros2 topic echo /scan --once (metadata and first ranges) ==="
timeout 8 ros2 topic echo /scan --once --qos-reliability best_effort --no-arr > "$OUT/scan_once.txt" 2>&1
grep -vE "^(---)$" "$OUT/scan_once.txt" | head -20
timeout 8 ros2 topic echo /scan --once --qos-reliability best_effort --field ranges 2>/dev/null | head -1 | cut -c1-240
echo "=== ros2 topic echo /state_estimate --once ==="
timeout 8 ros2 topic echo /state_estimate --once --no-arr
echo "=== ros2 param list /state_estimator_node ==="; ros2 param list /state_estimator_node
echo "=== ros2 param list /sim_node (lidar.*, frames.*, world) ==="
ros2 param list /sim_node | grep -E "lidar|frames|world"
for prm in lidar.rate_hz lidar.beams lidar.range_max lidar.mount_x frames.laser world; do
    printf "%s: " "$prm"; ros2 param get /sim_node "$prm"
done
for prm in publish_tf topics.state_estimate max_dt; do
    printf "%s: " "$prm"; ros2 param get /state_estimator_node "$prm"
done
echo "=== ros2 param set /state_estimator_node max_dt 1.0 (read-only, expect rejection) ==="
ros2 param set /state_estimator_node max_dt 1.0
echo "=== ros2 param get /wheel_odometry_node publish_tf ==="; ros2 param get /wheel_odometry_node publish_tf
echo "=== tf2_echo odom base_link ==="
timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "At time|Translation|RPY \(radian\)" | head -3
echo "=== tf2_echo base_link laser ==="
timeout 6 ros2 run tf2_ros tf2_echo base_link laser 2>&1 | grep -E "At time|Translation|RPY \(radian\)" | head -3
echo "=== tf2_echo odom laser (chain) ==="
timeout 6 ros2 run tf2_ros tf2_echo odom laser 2>&1 | grep -E "At time|Translation|RPY \(radian\)" | head -3
echo "=== estimator diagnostics (RUNNING expected) ==="
timeout 8 ros2 topic echo /diagnostics 2>/dev/null | grep -A 40 "state_estimator_node: fusion" | grep -E "message:|key: (state|imu_accepted|wheel_accepted|published|rejected_invalid|rejected_timestamp|gyro_bias|sigma_theta|nis_mean_speed_1dof|nis_mean_wheel_gyro_2dof)$" -A1 | grep -vE "^--" | head -30

alive
echo "=== fault injection: IMU with zero stamp, then IMU with NaN yaw rate ==="
ros2 topic pub --once -w 0 /imu/data sensor_msgs/msg/Imu "{header: {frame_id: imu_link}, angular_velocity: {z: 0.1}}" > /dev/null
ros2 topic pub --once -w 0 /imu/data sensor_msgs/msg/Imu "{header: {stamp: now, frame_id: imu_link}, angular_velocity: {z: .nan}}" > /dev/null
sleep 2
timeout 8 ros2 topic echo /diagnostics 2>/dev/null | grep -A 40 "state_estimator_node: fusion" | grep -E "message:|key: (state|rejected_invalid|rejected_timestamp|last_error)$" -A1 | grep -vE "^--" | head -10
grep -E "rejected imu measurement" "$LAUNCH_LOG" | head -3

echo "=== running for ${RUN_SECONDS} s more to accumulate accuracy statistics ==="
sleep "$RUN_SECONDS"
grep -E "estimator_evaluator.*vs ground truth" "$LAUNCH_LOG" | tail -3
grep -E "estimator state" "$LAUNCH_LOG" | head -5
alive

echo "=== Ctrl-C (SIGINT to the launch process group) ==="
T0=$(date +%s.%N)
kill -INT -- -"$LAUNCH"
wait "$LAUNCH"; RC=$?
echo "launch exit code: $RC after $(echo "$(date +%s.%N) - $T0" | bc) s"
grep -E "shutting down|finished cleanly|process has died|sending signal" "$LAUNCH_LOG" | tail -10
echo "=== spurious rcl_wait timeouts resumed ==="
grep -E "spurious timeout" "$LAUNCH_LOG" | tail -5 || true
echo "=== remaining robotics processes (expect none) ==="
pgrep -af "lib/robotics_(sim_bridge|estimation|examples)/" || echo "none"
