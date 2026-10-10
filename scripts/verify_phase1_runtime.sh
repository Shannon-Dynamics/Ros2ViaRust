#!/bin/bash
# Phase 1 runtime regression: both Phase 1 launch files still work (rates, TF, cmd_vel,
# parameters, clean Ctrl-C) after later refactors. Run from the WSL mirror root after
# `colcon build`. Output also goes to ~/phase1_runtime/verify.log.
set +u
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
OUT="$HOME/phase1_runtime"; mkdir -p "$OUT"
LOG="$OUT/verify.log"; : > "$LOG"
exec > >(tee -a "$LOG") 2>&1
source scripts/env.sh > /dev/null 2>&1
source ros2_ws/install/setup.bash
export RCUTILS_LOGGING_BUFFERED_STREAM=0
pkill -f "lib/robotics_" 2>/dev/null

echo "=== pubsub_demo ==="
setsid ros2 launch robotics_bringup pubsub_demo.launch.xml period_ms:=100 > "$OUT/pubsub.log" 2>&1 &
LA=$!; sleep 6
ros2 node list
timeout 8 ros2 topic hz /chatter --window 30 2>&1 | grep -E "average rate" | tail -1
ros2 param set /qos_talker prefix bonjour
timeout 8 ros2 topic echo /chatter --once --field data
ros2 param set /qos_talker period_ms 5
kill -INT -- -"$LA"; wait "$LA"; echo "launch exit code: $?"
grep -E "heard|shutting down|finished cleanly|process has died" "$OUT/pubsub.log" | tail -5

echo "=== wheel_odometry_demo ==="
setsid ros2 launch robotics_bringup wheel_odometry_demo.launch.xml > "$OUT/odom.log" 2>&1 &
LA=$!; sleep 6
ros2 node list
timeout 8 ros2 topic hz /odom --window 50 2>&1 | grep -E "average rate" | tail -1
ros2 topic info -v /tf | grep -E "Publisher count|Node name"
timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "Translation" | head -1
timeout 10 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -0.3}}" > /dev/null
sleep 1
timeout 8 ros2 topic echo /odom --once --field twist.twist
kill -INT -- -"$LA"; wait "$LA"; echo "launch exit code: $?"
grep -E "new command|shutting down|finished cleanly|process has died" "$OUT/odom.log" | tail -5
echo "=== remaining robotics processes (expect none) ==="
pgrep -af "lib/robotics_" || echo "none"
