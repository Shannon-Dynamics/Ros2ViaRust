#!/usr/bin/env bash
# Phase 3 integration test: brings the localization stack up through the single bringup
# launch and checks it, one assertion at a time, with the standard ROS 2 CLI.
#
#   bash scripts/verify_phase3_localization.sh [output directory]
#
# The Phase 2 equivalent is a Rust binary (`robotics_integration/phase2_stack_test`, 56
# checks) plus scripts/verify_phase2_runtime.sh, which records the CLI evidence. This file
# is the Phase 3 counterpart of both: it follows the shape of verify_phase2_runtime.sh, and
# it also decides. Every check prints `PASS` or `FAIL` with a name, the counts are
# summarised at the end, and the exit code is non-zero if anything failed.
#
# Why bash and not a second Rust stack test: most of what Phase 3 adds is outside any one
# node. The TF chain it completes is assembled by four publishers, three of which are the
# official Webots driver and two upstream ROS nodes, and the two guards that protect a run
# live in scripts/phase3_guards.sh. A test of the thing as launched is therefore a test of
# a launch, a TF tree and two shell predicates, which is what this is.
#
# What it covers:
#   A  the pieces exist and are installed               (9 checks, no simulator)
#   B  the two run guards accept and reject correctly    (8 checks, no simulator)
#   C  the stack comes up through the bringup launch     (8 checks)
#   D  every expected topic exists with the right type  (12 checks)
#   E  the topics actually publish                       (5 checks)
#   F  the TF tree is complete, map -> odom -> base_link -> sensors  (10 checks)
#   G  localization_node (EKF) behaves and reports     (8 checks)
#   H  mcl_node (MCL) behaves and reports              (7 checks)
#   I  route_driver drives and the robot moves         (6 checks)
#   J  the stack shuts down and leaves nothing behind  (5 checks)
#
# Evidence is written to the output directory, which defaults to
# <EVIDENCE_ROOT>/phase-3-localization-runtime/<date>, never to /tmp: WSL clears /tmp when
# the distro stops and that has already cost one set of run logs. EVIDENCE_ROOT comes from
# scripts/evidence_root.sh and is the evidence/ directory of the Windows checkout whenever
# that checkout is mounted, so the output is not sitting in the WSL mirror where the next
# rsync --delete of scripts/sync_to_wsl.sh could remove it.

set -u
# Deliberately no `set -m` here, unlike run_phase3_correctness.sh. The only signal this
# script sends is the SIGINT of the shutdown section, and `setsid` already puts the launch in its own
# process group, so job control buys nothing. It costs something: with `set -m` the
# `timeout -s INT` of the topic-rate loop below reached this script too and killed it with
# exit 130 half way through section E, which is how the first run of this file ended
# (evidence/phase-3-localization-runtime/2026-10-10, 46 checks and then nothing).

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/phase3_guards.sh"

source "$SCRIPT_DIR/evidence_root.sh"   # EVIDENCE_ROOT, the Windows checkout when mounted
OUT=${1:-$EVIDENCE_ROOT/phase-3-localization-runtime/$(date +%Y-%m-%d)}
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
LOG="$OUT/verify.log"
LAUNCH_LOG="$OUT/launch.log"
: > "$LOG"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1
exec > >(tee -a "$LOG") 2>&1

MAP="$REPO/sim/webots/config/landmarks.yaml"
ROUTE="$REPO/sim/webots/config/route_smoke.yaml"
LONG_ROUTE="$REPO/sim/webots/config/route_long.yaml"
BIN=$REPO/ros2_ws/install/robotics_localization_bridge/lib/robotics_localization_bridge

PASS=0
FAIL=0
FAILED_NAMES=()
ok()   { PASS=$((PASS + 1)); printf 'PASS  %s\n' "$1"; }
bad()  { FAIL=$((FAIL + 1)); FAILED_NAMES+=("$1"); printf 'FAIL  %s\n' "$1"; }
check() {  # check <name> <command...>
    local name=$1; shift
    if "$@" > /dev/null 2>&1; then ok "$name"; else bad "$name"; fi
}
refute() {  # refute <name> <command...>  : passes when the command fails
    local name=$1; shift
    if "$@" > /dev/null 2>&1; then bad "$name"; else ok "$name"; fi
}
check_match() {  # check_match <name> <extended regex> <file>
    local name=$1 pattern=$2 file=$3
    if [ -s "$file" ] && grep -qE "$pattern" "$file"; then ok "$name"; else bad "$name"; fi
}
# Like check_match, but only within the few lines following a block-start pattern: /diagnostics
# now carries interleaved status entries from both localization_node and mcl_node (and from
# route_driver and controller_manager besides), and a plain grep for a generic field like
# "message: RUNNING" would pass on either node's line regardless of which one this check means
# to be about. DiagnosticStatus's field order (name, then message, see
# robotics_ros_bridge::diagnostics::StatusBuilder::new) puts the two adjacent, so one line of
# context after the name is enough.
check_block_match() {  # check_block_match <name> <block_start_regex> <contains_regex> <file> [lines_after]
    local name=$1 start=$2 contains=$3 file=$4 n=${5:-1}
    if [ -s "$file" ] && grep -A"$n" -E "$start" "$file" | grep -qE "$contains"; then
        ok "$name"
    else
        bad "$name"
    fi
}
section() { printf '\n===== %s =====\n' "$1"; }

printf '=== %s  ROS_DISTRO=%s  RMW=%s ===\n' \
    "$(date -Is)" "${ROS_DISTRO:-unset}" "${RMW_IMPLEMENTATION:-default}"
printf 'repository: %s\noutput:     %s\n' "$REPO" "$OUT"

# ----------------------------------------------------------------- A: the pieces exist
section "A: the pieces exist and are installed"
check "A1  the colcon install space exists" test -d "$REPO/ros2_ws/install"
for exe in localization_node mcl_node localization_evaluator route_driver; do
    check "A   $exe is installed and executable" test -x "$BIN/$exe"
done
check "A5  the bringup launch is installed" \
    test -f "$REPO/ros2_ws/install/robotics_bringup/share/robotics_bringup/launch/phase3_localization.launch.py"
check "A6  the world launch exists" \
    test -f "$REPO/sim/webots/launch/localization_world.launch.py"
check "A7  the landmark map is readable" test -r "$MAP"
check "A8  the smoke route is readable" test -r "$ROUTE"
if [ "$(grep -c '^ *- {' "$LONG_ROUTE")" -ge 10 ]; then
    ok "A9  the measurement route has at least 10 waypoints"
else
    bad "A9  the measurement route has at least 10 waypoints"
fi

# ----------------------------------------------------------------- B: the run guards
section "B: the run guards accept and reject correctly"
# These are the guards of scripts/phase3_guards.sh, which run_phase3_correctness.sh sources
# too, so this tests the code a real run is protected by rather than a copy of it.
HEALTHY='diffdrive_controller      diff_drive_controller/DiffDriveController  active
joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster  active'
HALF='diffdrive_controller      diff_drive_controller/DiffDriveController  unconfigured
joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster  active'
NO_BROADCASTER='diffdrive_controller   diff_drive_controller/DiffDriveController  active'
check  "B1  controllers_active_in accepts both active" controllers_active_in "$HEALTHY"
refute "B2  controllers_active_in rejects an unconfigured diffdrive_controller" \
    controllers_active_in "$HALF"
refute "B3  controllers_active_in rejects a missing joint_state_broadcaster" \
    controllers_active_in "$NO_BROADCASTER"
refute "B4  controllers_active_in rejects an empty listing" controllers_active_in ""

HEADER='stamp_s,t_s,gt_x,gt_y,gt_theta,est_x,est_y,est_theta,position_error_m,yaw_error_rad,odom_x,odom_y,odom_theta,odom_position_error_m,est_sigma_xy_m,after_kidnap'
{ echo "$HEADER"
  for i in 0 1 2 3 4 5 6 7 8 9; do
      printf '%s,%s,%s,0.0,0,0,0,0,0,0,0,0,0,0,0,0\n' "$i" "$i" "0.$i"
  done
} > "$OUT/fixture-moving.csv"
{ echo "$HEADER"
  for i in 0 1 2 3 4 5 6 7 8 9; do
      printf '%s,%s,0.001,0.0,0,0,0,0,0,0,0,0,0,0,0,0\n' "$i" "$i"
  done
} > "$OUT/fixture-stationary.csv"
check  "B5  ground_truth_moved accepts a run that moved" \
    ground_truth_moved "$OUT/fixture-moving.csv"
refute "B6  ground_truth_moved rejects a stationary run" \
    ground_truth_moved "$OUT/fixture-stationary.csv"
refute "B7  ground_truth_moved rejects a missing CSV" \
    ground_truth_moved "$OUT/fixture-does-not-exist.csv"
if [ "$(ground_truth_displacement "$OUT/fixture-moving.csv")" = "0.900" ]; then
    ok "B8  ground_truth_displacement measures the travel it should"
else
    bad "B8  ground_truth_displacement measures the travel it should"
fi

# ----------------------------------------------------------------- C: bringup
section "C: the stack comes up through the bringup launch"
if pgrep -x webots-bin > /dev/null; then
    bad "C0  no webots-bin was running before the test"
    echo "a webots-bin process is already holding the controller port; stop it and rerun"
    exit 1
fi
ok "C0  no webots-bin was running before the test"

wait_for() {  # wait_for <seconds> <command...>
    local limit=$1; shift
    for _ in $(seq 1 "$limit"); do
        if "$@" > /dev/null 2>&1; then return 0; fi
        sleep 1
    done
    return 1
}
has_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
has_node()  { ros2 node list  2>/dev/null | grep -qx "$1"; }

echo "--- ros2 launch robotics_bringup phase3_localization.launch.py (own process group) ---"
setsid ros2 launch robotics_bringup phase3_localization.launch.py \
    gui:=false mode:=realtime ground_truth:=true localize:=true mcl:=true drive:=false \
    > "$LAUNCH_LOG" 2>&1 &
LAUNCH=$!

# If this script dies before section J, the launch does not: `setsid` gave it its own
# session, so it outlives its parent and the surviving webots-bin keeps holding the
# controller port. The next run then aborts on "a webots-bin process is already running",
# which is what happened after the aborted first run of this file. The shutdown section still does the
# graceful shutdown and checks it; this trap is only for the paths that never reach it, and
# `trap - EXIT` there keeps the two from both firing.
cleanup() {
    local rc=$?
    echo "--- cleanup: the test is leaving before the shutdown section, stopping the launch ---"
    kill -INT -- -"$LAUNCH" 2>/dev/null
    sleep 5
    kill -9 -- -"$LAUNCH" 2>/dev/null
    pkill -f "webots-bin .*--port=1234" 2>/dev/null
    pkill -f "lib/robotics_localization_bridge/" 2>/dev/null
    exit "$rc"
}
trap cleanup EXIT INT TERM

check "C1  /clock appears"               wait_for 180 has_topic /clock
check "C2  /scan appears"                wait_for 120 has_topic /scan
check "C3  /ground_truth/pose appears"   wait_for 60  has_topic /ground_truth/pose
check "C4  both controllers become active" wait_for 90 controllers_active
check "C5  localization_node is in the node list" wait_for 60 has_node /localization_node
check "C6  mcl_node is in the node list" wait_for 60 has_node /mcl_node
check "C7  robot_state_publisher is in the node list" has_node /robot_state_publisher
check "C8  the Ros2Supervisor is in the node list" \
    bash -c 'ros2 node list 2>/dev/null | grep -q Ros2Supervisor'
ros2 node list    > "$OUT/node_list.txt"    2>&1
ros2 topic list -t > "$OUT/topic_list.txt"  2>&1
ros2 control list_controllers > "$OUT/controllers.txt" 2>&1

# ----------------------------------------------------------------- D: topics and types
section "D: every expected topic exists with the right type"
has_type() {  # has_type <topic> <type>
    ros2 topic list -t 2>/dev/null | grep -qx "$1 \[$2\]"
}
check "D1  /scan is a sensor_msgs/LaserScan" has_type /scan sensor_msgs/msg/LaserScan
check "D2  /odom is a nav_msgs/Odometry" has_type /odom nav_msgs/msg/Odometry
check "D3  /imu is a sensor_msgs/Imu" has_type /imu sensor_msgs/msg/Imu
check "D4  /joint_states is a sensor_msgs/JointState" \
    has_type /joint_states sensor_msgs/msg/JointState
check "D5  /clock is a rosgraph_msgs/Clock" has_type /clock rosgraph_msgs/msg/Clock
check "D6  /ground_truth/pose is a geometry_msgs/PoseStamped" \
    has_type /ground_truth/pose geometry_msgs/msg/PoseStamped
check "D7  /localization/pose is a geometry_msgs/PoseWithCovarianceStamped" \
    wait_for 60 has_type /localization/pose geometry_msgs/msg/PoseWithCovarianceStamped
check "D8  /mcl/pose is a geometry_msgs/PoseWithCovarianceStamped" \
    wait_for 60 has_type /mcl/pose geometry_msgs/msg/PoseWithCovarianceStamped
check "D9  /tf is a tf2_msgs/TFMessage" has_type /tf tf2_msgs/msg/TFMessage
check "D10 /tf_static is a tf2_msgs/TFMessage" has_type /tf_static tf2_msgs/msg/TFMessage
check "D11 /diagnostics is a diagnostic_msgs/DiagnosticArray" \
    has_type /diagnostics diagnostic_msgs/msg/DiagnosticArray
check "D12 /kidnap/teleport is offered by the supervisor" has_topic /kidnap/teleport
check "D13 /kidnap/events is offered by the supervisor" has_topic /kidnap/events

# ----------------------------------------------------------------- E: they publish
section "E: the topics actually publish"
publishes() {  # publishes <topic> [extra ros2 topic echo args...]
    local topic=$1; shift
    timeout 15 ros2 topic echo --once "$topic" "$@" > /dev/null 2>&1
}
check "E1  /scan publishes" publishes /scan --qos-reliability best_effort
check "E2  /odom publishes" publishes /odom
check "E3  /ground_truth/pose publishes" publishes /ground_truth/pose
check "E4  /localization/pose publishes" publishes /localization/pose
check "E5  /mcl/pose publishes" publishes /mcl/pose
check "E6  /tf publishes" publishes /tf
for t in /scan /odom /localization/pose /mcl/pose /ground_truth/pose; do
    printf -- '--- ros2 topic hz %s ---\n' "$t"
    timeout -s INT 8 ros2 topic hz "$t" --window 50 2>&1 | grep -E "average rate" | tail -1
done > "$OUT/topic_hz.txt" 2>&1

# ----------------------------------------------------------------- F: the TF tree
section "F: the TF tree is complete"
# map -> odom is the edge Phase 3 adds and localization_node is its only owner; everything
# below it belongs to the Webots driver and robot_state_publisher. Full TF authority
# conflict detection is the job of the Rust TfListener and is covered by the Phase 2 stack
# test; what is checked here is that each edge resolves and that the chains compose.
# Twenty seconds, not eight: tf2_echo has to fill its buffer and find a time it can
# interpolate at, and every stamp here is simulation time. A six-second timeout produced
# nothing but `Terminated` in the 2026-10-10 run and was read as a missing transform.
resolves() {  # resolves <parent> <child>
    timeout 20 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | grep -q "^At time"
}
check "F1  map -> odom resolves"              resolves map odom
check "F2  odom -> base_link resolves"        resolves odom base_link
check "F3  base_link -> LDS-01 resolves"      resolves base_link LDS-01
check "F4  base_link -> imu_link resolves"    resolves base_link imu_link
check "F5  base_link -> base_footprint resolves" resolves base_link base_footprint
check "F6  map -> base_link resolves through the chain" resolves map base_link
check "F7  map -> LDS-01 resolves through the chain"    resolves map LDS-01
check "F8  map -> imu_link resolves through the chain"  resolves map imu_link
# Several nodes publish on /tf, so `--once` returns whichever message arrived first and
# says nothing about the others: on 2026-10-10 it caught the driver's odom -> base_link and
# this check failed while F1 had already resolved map -> odom. Capture a few seconds of the
# stream instead and look for the edge in it.
timeout 8 ros2 topic echo /tf > "$OUT/tf_stream.yaml" 2>&1
check_match "F9  a transform with child_frame_id odom is broadcast" \
    "child_frame_id: *'?odom'?$" "$OUT/tf_stream.yaml"
ros2 topic info -v /tf > "$OUT/tf_publishers.txt" 2>&1
check_match "F10 localization_node is a publisher on /tf" \
    "localization_node" "$OUT/tf_publishers.txt"
# mcl_node subscribes to /tf (its TfListener looks up base_link -> laser), so a plain grep
# for its name over the whole `ros2 topic info -v` output would match the subscriber
# section even though it never publishes; isolate the publisher section first.
refute "F11 mcl_node is not a publisher on /tf (by design, see mcl_node.rs)" \
    bash -c "awk '/Publisher count:/,/Subscription count:/' '$OUT/tf_publishers.txt' | grep -q mcl_node"
{
    for pair in "map odom" "map base_link" "odom base_link" "base_link LDS-01" \
                "base_link imu_link" "map LDS-01"; do
        set -- $pair
        printf -- '--- tf2_echo %s %s ---\n' "$1" "$2"
        timeout 20 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 |
            grep -E "At time|Translation|Rotation: in RPY \(radian\)" | head -3
    done
} > "$OUT/tf.txt" 2>&1

# ----------------------------------------------------------------- G: the node under test
section "G: localization_node behaves and reports"
timeout 20 ros2 topic echo /diagnostics > "$OUT/diagnostics.yaml" 2>&1
check_match "G1  localization_node reports its status" \
    "localization_node: ekf" "$OUT/diagnostics.yaml"
check_block_match "G2  its status is RUNNING" \
    "name: 'localization_node: ekf'" "message: RUNNING" "$OUT/diagnostics.yaml"
check_match "G3  it reports map -> odom as published" \
    "value: published" "$OUT/diagnostics.yaml"
check_match "G4  it reports a non-zero tf_sent" \
    "key: tf_sent" "$OUT/diagnostics.yaml"
check_block_match "G5  the laser mount came from TF, not from parameters" \
    "key: sensor_pose_source" "value: tf" "$OUT/diagnostics.yaml"
{
    ros2 param list /localization_node
    for prm in publish_tf frames.map frames.odom topics.odom map_file; do
        printf '%s: ' "$prm"; ros2 param get /localization_node "$prm"
    done
    echo "--- param set publish_tf false, expected to be rejected (read-only) ---"
    ros2 param set /localization_node publish_tf false
} > "$OUT/params.txt" 2>&1
check_match "G6  publish_tf is true" "publish_tf.*[Bb]oolean value is: True" "$OUT/params.txt"
check_match "G7  topics.odom points at the odometry the simulator publishes" \
    "topics.odom:.*String value is: /?odom$" "$OUT/params.txt"
check_match "G8  the read-only parameters refuse to be set" \
    "Setting parameter failed|read-only|not set" "$OUT/params.txt"

# ----------------------------------------------------------------- H: mcl_node
section "H: mcl_node (MCL) behaves and reports"
timeout 20 ros2 topic echo /diagnostics > "$OUT/diagnostics_mcl.yaml" 2>&1
check_match "H1  mcl_node reports its status" \
    "mcl_node: particle filter" "$OUT/diagnostics_mcl.yaml"
check_block_match "H2  its status is RUNNING (no injection should have fired yet)" \
    "name: 'mcl_node: particle filter'" "message: RUNNING" "$OUT/diagnostics_mcl.yaml"
check_match "H3  it reports its particle count" "key: particles" "$OUT/diagnostics_mcl.yaml"
check_match "H4  it reports an effective sample size" "key: ess" "$OUT/diagnostics_mcl.yaml"
check_block_match "H5  the laser mount came from TF, not from parameters" \
    "key: sensor_pose_source" "value: tf" "$OUT/diagnostics_mcl.yaml"
{
    ros2 param list /mcl_node
    for prm in particles.count measurement.sigma_range recovery.sustained_updates topics.odom map_file; do
        printf '%s: ' "$prm"; ros2 param get /mcl_node "$prm"
    done
} > "$OUT/params_mcl.txt" 2>&1
check_match "H6  particles.count matches McLocalizer's documented default" \
    "particles.count:.*Integer value is: 6000" "$OUT/params_mcl.txt"
check_match "H7  topics.odom points at the odometry the simulator publishes" \
    "topics.odom:.*String value is: /?odom$" "$OUT/params_mcl.txt"

# ----------------------------------------------------------------- I: route_driver
section "I: route_driver drives and the robot moves"
GT_BEFORE=$(timeout 10 ros2 topic echo --once /ground_truth/pose --field pose.position.x 2>/dev/null | head -1)
ros2 run robotics_localization_bridge route_driver --ros-args \
    -p "route_file:=$ROUTE" -p laps:=1 > "$OUT/route_driver.log" 2>&1 &
ROUTE_PID=$!
check "I1  route_driver appears in the node list" wait_for 30 has_node /route_driver
check "I2  /cmd_vel is a geometry_msgs/TwistStamped" \
    wait_for 30 has_type /cmd_vel geometry_msgs/msg/TwistStamped
check "I3  /cmd_vel publishes" publishes /cmd_vel
timeout 20 ros2 topic echo /diagnostics > "$OUT/diagnostics_driving.yaml" 2>&1
check_match "I4  route_driver reports that it is driving" \
    "route_driver: waypoint follower" "$OUT/diagnostics_driving.yaml"
echo "--- waiting for the smoke route to finish ---"
wait "$ROUTE_PID"; ROUTE_RC=$?
echo "route_driver exit code: $ROUTE_RC"
if [ "$ROUTE_RC" = 0 ]; then
    ok "I5  route_driver exits 0 when the route is finished"
else
    bad "I5  route_driver exits 0 when the route is finished"
fi
check_match "I6  the robot reached the waypoints of the route" \
    "route .*finished|waypoint 3/3" "$OUT/route_driver.log"

# ----------------------------------------------------------------- J: shutdown
section "J: the stack shuts down and leaves nothing behind"
trap - EXIT INT TERM    # from here on the shutdown below is the one that runs
echo "--- SIGINT to the launch process group ---"
T0=$(date +%s.%N)
kill -INT -- -"$LAUNCH"
wait "$LAUNCH"; LAUNCH_RC=$?
echo "launch exit code: $LAUNCH_RC after $(awk "BEGIN {printf \"%.1f\", $(date +%s.%N) - $T0}") s"
if [ "$LAUNCH_RC" -le 1 ]; then
    ok "J1  the launch exits on SIGINT"
else
    bad "J1  the launch exits on SIGINT"
fi
sleep 3
refute "J2  no webots-bin is left holding the controller port" pgrep -x webots-bin
refute "J3  no localization or harness node is left running" \
    pgrep -f "lib/robotics_localization_bridge/"
check_match "J4  the launch log records how Webots was stopped" \
    "shutdown: webots (exited on SIGTERM|ignored SIGTERM)" "$LAUNCH_LOG"
check "J5  the evidence files were written" \
    bash -c "test -s '$OUT/node_list.txt' && test -s '$OUT/tf.txt' && test -s '$OUT/diagnostics.yaml'"

# ----------------------------------------------------------------- summary
section "summary"
printf 'checks: %d\npassed: %d\nfailed: %d\n' "$((PASS + FAIL))" "$PASS" "$FAIL"
if [ "$FAIL" -gt 0 ]; then
    printf 'failed checks:\n'
    printf '  %s\n' "${FAILED_NAMES[@]}"
fi
printf 'evidence: %s\n' "$OUT"
{
    printf 'date_utc: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'checks: %d\npassed: %d\nfailed: %d\n' "$((PASS + FAIL))" "$PASS" "$FAIL"
    printf 'verdict: %s\n' "$([ "$FAIL" -eq 0 ] && echo PASS || echo FAIL)"
    if [ "$FAIL" -gt 0 ]; then
        printf 'failed_checks:\n'
        printf '  - %s\n' "${FAILED_NAMES[@]}"
    fi
} > "$OUT/summary.yaml"
[ "$FAIL" -eq 0 ]
