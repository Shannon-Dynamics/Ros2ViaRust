#!/usr/bin/env bash
# One Phase 3 correctness run: EKF localization against simulator ground truth.
#
#   bash scripts/run_phase3_correctness.sh <normal|kidnap> <output directory>
#
# Both scenarios drive the identical route, a square, with the identical parameters. The
# only difference is that `kidnap` teleports the robot once, halfway through, through the
# Webots Supervisor and without telling the estimator. Everything the run produces is a
# file in the output directory; no number is printed here and copied anywhere by hand.
#
# What runs:
#   Webots + official webots_ros2_turtlebot driver     (simulation, not Rust)
#   evaluation_supervisor.py                           (ground truth + teleport, not Rust)
#   localization_node                                  (Rust, the thing under test)
#   localization_evaluator                             (Rust, grades it against ground truth)

set -u

# Job control, so that every SIGINT this script sends is a real one. A non-interactive shell
# starts a background job with SIGINT and SIGQUIT set to SIG_IGN, so that Ctrl-C on the
# terminal cannot reach it, and CPython does not install its own handler over an inherited
# SIG_IGN. Without `set -m` the `kill -INT` in `stop()` is a no-op for the launch and for
# both Rust nodes: each one is reported as "still alive 30 s after SIGINT" and then killed
# with SIGKILL, which is what the 2026-10-06 run recorded. That measured this script, not
# the nodes. Verified on a live `ros2 launch` process as `SigIgn: ...1001006`, SIGINT bit
# set. With job control each job gets its own process group and the default disposition.
set -m

SCENARIO=${1:?usage: run_phase3_correctness.sh <normal|kidnap> <output directory>}
OUT=${2:?usage: run_phase3_correctness.sh <normal|kidnap> <output directory>}
case "$SCENARIO" in
    normal|kidnap) ;;
    *) echo "scenario must be normal or kidnap" >&2; exit 2 ;;
esac

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
LAUNCH="$REPO/sim/webots/launch/localization_world.launch.py"
MAP="$REPO/sim/webots/config/landmarks.yaml"
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
CSV="$OUT/$SCENARIO-run.csv"
JSON="$OUT/$SCENARIO-run.json"

# The kidnap target: a free pose in the same room, far from where the route passes.
KIDNAP_X=-1.5
KIDNAP_Y=-1.5
KIDNAP_YAW=2.0          # rad; quaternion below is sin(yaw/2), cos(yaw/2)
KIDNAP_QZ=0.8414709848
KIDNAP_QW=0.5403023059

log() { echo "[$SCENARIO] $*" | tee -a "$OUT/$SCENARIO-run.log"; }

if pgrep -x webots-bin > /dev/null; then
    log "FAIL: a webots-bin process is already running, stop it first"
    exit 1
fi

{
    echo "scenario: $SCENARIO"
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)"
    echo "kernel: $(uname -r)"
    echo "os: $(. /etc/os-release && echo "$PRETTY_NAME")"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
    echo "ros_distro: ${ROS_DISTRO:-unset}"
    echo "rmw: ${RMW_IMPLEMENTATION:-default}"
    echo "webots: $(webots --version 2>&1 | head -1)"
    echo "rustc: $(rustc --version 2>&1)"
    echo "map_file: $MAP"
    echo "map_sha256: $(sha256sum "$MAP" | cut -d' ' -f1)"
    echo "world_sha256: $(sha256sum "$REPO/sim/webots/worlds/turtlebot3_localization_room.wbt" | cut -d' ' -f1)"
    if [ "$SCENARIO" = kidnap ]; then
        echo "kidnap_target: x=$KIDNAP_X y=$KIDNAP_Y yaw=$KIDNAP_YAW"
    fi
} > "$OUT/$SCENARIO-environment.txt"

log "starting the world with ground truth"
ros2 launch "$LAUNCH" gui:=false mode:=realtime ground_truth:=true \
    > "$OUT/$SCENARIO-launch.log" 2>&1 &
LAUNCH_PID=$!

wait_for() {  # wait_for <seconds> <description> <command...>
    local limit=$1 what=$2; shift 2
    for _ in $(seq 1 "$limit"); do
        if "$@" > /dev/null 2>&1; then return 0; fi
        sleep 1
    done
    log "FAIL: timed out after ${limit}s waiting for $what"
    return 1
}
has_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

wait_for 180 "/scan" has_topic /scan || { kill -INT "$LAUNCH_PID"; exit 1; }
wait_for 60 "/ground_truth/pose" has_topic /ground_truth/pose || { kill -INT "$LAUNCH_PID"; exit 1; }
log "simulation is up"

# Both ros2_control controllers have to be active before anything is driven. If the
# resource manager of the driver came up without the `ros2_control` part of the URDF they
# stay unconfigured: /cmd_vel is then accepted by nothing, the robot never leaves the
# origin, and the evaluation grades the filter against a stationary ground truth without a
# single error message. That happened on 2026-10-06 and produced 588 meaningless rows
# (evidence/phase-3-correctness/2026-10-06). The topics existing is not enough to rule it
# out, because they all publish either way.
controllers_active() {
    local listing
    listing=$(ros2 control list_controllers 2>/dev/null) || return 1
    grep -q "diffdrive_controller.*active" <<< "$listing" &&
        grep -q "joint_state_broadcaster.*active" <<< "$listing"
}
if ! wait_for 90 "both controllers to become active" controllers_active; then
    {
        echo "FAIL: diffdrive_controller and joint_state_broadcaster did not both become active"
        echo "--- ros2 control list_controllers ---"
        ros2 control list_controllers 2>&1
        echo "--- ros2_control lines from the launch log ---"
        grep -iE "ros2_control|hardware|resource manager|command interface" \
            "$OUT/$SCENARIO-launch.log" 2>/dev/null | head -20
    } > "$OUT/$SCENARIO-controllers.txt" 2>&1
    log "FAIL: the controllers never activated, so the robot could not have moved; aborting"
    log "see $SCENARIO-controllers.txt"
    kill -INT "$LAUNCH_PID"
    exit 1
fi
ros2 control list_controllers > "$OUT/$SCENARIO-controllers.txt" 2>&1
log "controllers are active"

log "starting localization_node and localization_evaluator"
ros2 run robotics_localization_bridge localization_node --ros-args \
    -p "map_file:=$MAP" \
    > "$OUT/$SCENARIO-localization_node.log" 2>&1 &
NODE_PID=$!
ros2 run robotics_localization_bridge localization_evaluator --ros-args \
    -p "output_csv:=$CSV" -p "output_json:=$JSON" \
    > "$OUT/$SCENARIO-evaluator.log" 2>&1 &
EVAL_PID=$!
wait_for 60 "the first pose estimate" has_topic /localization/pose || log "no estimate topic yet"
sleep 3

drive() {  # drive <seconds> <linear x> <angular z>
    timeout "$1" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
        "{twist: {linear: {x: $2}, angular: {z: $3}}}" > /dev/null 2>&1
}
kidnap() {
    log "teleporting the robot to ($KIDNAP_X, $KIDNAP_Y, $KIDNAP_YAW)"
    ros2 topic pub --once /kidnap/teleport geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: $KIDNAP_X, y: $KIDNAP_Y, z: 0.0}, \
          orientation: {x: 0.0, y: 0.0, z: $KIDNAP_QZ, w: $KIDNAP_QW}}}" > /dev/null 2>&1
    sleep 2
}

# The route: a square, 0.9 m per side, turning 90 degrees in place at each corner.
log "driving the square route"
for side in 1 2 3 4; do
    log "side $side: forward"
    drive 6 0.15 0.0
    log "side $side: turn"
    drive 4 0.0 0.4
    if [ "$SCENARIO" = kidnap ] && [ "$side" = 2 ]; then
        kidnap
        log "continuing the same route after the kidnap"
    fi
done
log "route finished, letting the filter settle"
sleep 5

ros2 node list  > "$OUT/$SCENARIO-node_list.txt"  2>&1
ros2 topic list > "$OUT/$SCENARIO-topic_list.txt" 2>&1
timeout 15 ros2 topic echo --once /diagnostics > "$OUT/$SCENARIO-diagnostics.yaml" 2>&1

stop() {  # stop <name> <pid>
    local name=$1 pid=$2
    # Signal the whole process group, which is what a terminal does on Ctrl-C and what these
    # jobs need: `ros2 run` is a Python wrapper that starts the Rust executable as a child,
    # so a SIGINT aimed at the job's own pid never reaches the node. Both nodes were then
    # killed 30 s later, and `localization_evaluator` writes its summary only on a clean
    # shutdown, so the JSON summary of the run was never written at all
    # (evidence/phase-3-correctness/2026-10-06-nodes-killed, which has a CSV and no JSON).
    # `set -m` above puts every job in its own process group, so the negative pid cannot
    # reach this script.
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    for _ in $(seq 1 30); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 1
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "$name: still alive 30 s after SIGINT, exit code unknown"
        kill -9 -- "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
    else
        wait "$pid"
        echo "$name: exit code $?"
    fi
}

log "stopping with SIGINT, evaluator first so that it writes its summary"
{
    stop localization_evaluator "$EVAL_PID"
    stop localization_node "$NODE_PID"
    stop ros2_launch "$LAUNCH_PID"
    echo "webots_bin_left: $(pgrep -c -x webots-bin || true)"
} > "$OUT/$SCENARIO-shutdown.txt" 2>&1
cat "$OUT/$SCENARIO-shutdown.txt" | tee -a "$OUT/$SCENARIO-run.log"

if [ -s "$CSV" ]; then
    log "rows written: $(( $(wc -l < "$CSV") - 1 ))"
    # Last line of defence, independent of the controller check above: a run whose ground
    # truth never moved is not a localization run, whatever its rows say. gt_x and gt_y are
    # columns 3 and 4; the route is a 0.9 m square, so anything under 0.2 m of travel means
    # the robot stayed put.
    MOVED=$(awk -F, 'NR==2 {x0=$3; y0=$4}
                     NR>1 {d=sqrt(($3-x0)^2 + ($4-y0)^2); if (d>m) m=d}
                     END {printf "%.3f", m+0}' "$CSV")
    log "ground_truth_max_displacement_m: $MOVED"
    if awk "BEGIN {exit !($MOVED < 0.2)}"; then
        log "FAIL: the robot never moved (max $MOVED m from its start pose); the rows in"
        log "FAIL: $(basename "$CSV") grade the filter against a stationary ground truth"
    fi
else
    log "FAIL: no rows were written to $CSV"
fi
log "done, evidence in $OUT"
