#!/usr/bin/env bash
# One Phase 3 correctness run: EKF and/or MCL localization against simulator ground truth.
#
#   bash scripts/run_phase3_correctness.sh <normal|kidnap> <output directory>
#
# The output directory may be given as a bare name with no slash in it, which is the form
# to prefer: it resolves to <EVIDENCE_ROOT>/phase-3-correctness/<name>, and
# scripts/evidence_root.sh points EVIDENCE_ROOT at the Windows checkout, so the run writes
# across the mount instead of into the WSL mirror where a later rsync --delete could remove
# it. A path containing a slash is used exactly as given, for the cases that need it.
#
# Both scenarios drive the identical route with the identical parameters. The only
# difference is that `kidnap` teleports the robot once, part way through, through the
# Webots Supervisor and without telling either estimator. Everything the run produces is a
# file in the output directory; no number is printed here and copied anywhere by hand.
#
# Environment overrides, all recorded in <scenario>-environment.txt so that a run can be
# read back without this script:
#
#   ROUTE_FILE    route the robot drives          (default sim/webots/config/route_long.yaml)
#   LAPS          how many times to drive it      (default 1)
#   ESTIMATORS    which estimator(s) to run, space separated: "ekf", "mcl", or "ekf mcl"
#                                                 (default "ekf")
#
# Running both together, rather than as two separate invocations with the same configured
# parameters, is deliberate whenever a direct comparison is the point (the MCL-vs-EKF and
# the kidnap recovery evidence both do this): one Webots session means one ground truth
# trajectory and, for the kidnap scenario, one teleport at one simulated instant, so the two
# estimators are compared against literally the same event rather than against two separate
# runs that only share a configured route file and a configured kidnap delay.
#
# Output file names: the EKF's are unsuffixed (<scenario>-run.csv/.json/-verdict.yaml), the
# same names every run before 2026-10-10 used, so nothing that already points at them breaks.
# The MCL's carry a -mcl suffix (<scenario>-run-mcl.csv/.json, <scenario>-verdict-mcl.yaml)
# whether or not the EKF ran alongside it in the same invocation.
#
# What runs:
#   Webots + official webots_ros2_turtlebot driver     (simulation, not Rust)
#   evaluation_supervisor.py                           (ground truth + teleport, not Rust)
#   route_driver                                       (Rust, drives the route; harness)
#   localization_node, and/or mcl_node                 (Rust, the thing(s) under test)
#   one localization_evaluator per active estimator    (Rust, grades it against ground truth)

set -u
set -m   # see the header comment of an earlier revision of this file, kept below at stop()

SCENARIO=${1:?usage: run_phase3_correctness.sh <normal|kidnap> <output directory>}
OUT=${2:?usage: run_phase3_correctness.sh <normal|kidnap> <output directory>}
case "$SCENARIO" in
    normal|kidnap) ;;
    *) echo "scenario must be normal or kidnap" >&2; exit 2 ;;
esac

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
# controllers_active and ground_truth_moved, shared with scripts/verify_phase3_localization.sh
source "$SCRIPT_DIR/phase3_guards.sh"
# EVIDENCE_ROOT; see that file for the runs this indirection exists because of
source "$SCRIPT_DIR/evidence_root.sh"
LAUNCH="$REPO/sim/webots/launch/localization_world.launch.py"
MAP="$REPO/sim/webots/config/landmarks.yaml"
ROUTE_FILE=${ROUTE_FILE:-$REPO/sim/webots/config/route_long.yaml}
LAPS=${LAPS:-1}
ESTIMATORS=${ESTIMATORS:-ekf}
case "$OUT" in
    */*) ;;                                              # a path, used as given
    *) OUT="$EVIDENCE_ROOT/phase-3-correctness/$OUT" ;;  # a bare run name
esac
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)

run_ekf=false
run_mcl=false
for e in $ESTIMATORS; do
    case "$e" in
        ekf) run_ekf=true ;;
        mcl) run_mcl=true ;;
        *) echo "ESTIMATORS must list only ekf and/or mcl, got $e" >&2; exit 2 ;;
    esac
done
if ! $run_ekf && ! $run_mcl; then
    echo "ESTIMATORS listed nothing to run" >&2; exit 2
fi

EKF_CSV="$OUT/$SCENARIO-run.csv"
EKF_JSON="$OUT/$SCENARIO-run.json"
MCL_CSV="$OUT/$SCENARIO-run-mcl.csv"
MCL_JSON="$OUT/$SCENARIO-run-mcl.json"

# The odometry both filters predict from and both evaluators grade as the baseline. One
# topic, read by everyone: a filter fed one stream and graded against another would be a
# rigged comparison. It is the odometry the simulator publishes, unmodified; see
# docs/phase-3-localization/theory.md section 6 for why nothing is degraded on purpose.
ODOM_TOPIC=/odom

# The kidnap target: a free pose in the same room, far from where the route passes.
KIDNAP_X=-1.5
KIDNAP_Y=-1.5
KIDNAP_YAW=2.0          # rad; quaternion below is sin(yaw/2), cos(yaw/2)
KIDNAP_QZ=0.8414709848
KIDNAP_QW=0.5403023059
KIDNAP_AFTER=${KIDNAP_AFTER:-120}   # seconds into the route

# Unbuffered output from every CLI tool this script captures; see the note at the bottom of
# this file for why (a tf2_echo capture this script used to make lost its block buffer to
# SIGTERM otherwise, see evidence/phase-3-correctness/2026-10-10-long-route/NOTE-tf-capture.md).
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

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
    echo "route_file: $ROUTE_FILE"
    echo "route_sha256: $(sha256sum "$ROUTE_FILE" | cut -d' ' -f1)"
    echo "route_waypoints: $(grep -c '^ *- {' "$ROUTE_FILE")"
    echo "laps: $LAPS"
    echo "odometry_topic_under_test: $ODOM_TOPIC"
    echo "odometry_noise: off"
    echo "estimators: $ESTIMATORS"
    if [ "$SCENARIO" = kidnap ]; then
        echo "kidnap_target: x=$KIDNAP_X y=$KIDNAP_Y yaw=$KIDNAP_YAW"
        echo "kidnap_after_s: $KIDNAP_AFTER"
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

# Both ros2_control controllers have to be active before anything is driven; see the
# header of phase3_guards.sh for the run this guard exists because of.
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

EKF_NODE_PID=
EKF_EVAL_PID=
MCL_NODE_PID=
MCL_EVAL_PID=

if $run_ekf; then
    log "starting localization_node (EKF) and its evaluator against $ODOM_TOPIC"
    ros2 run robotics_localization_bridge localization_node --ros-args \
        -p "map_file:=$MAP" \
        -p "topics.odom:=$ODOM_TOPIC" \
        > "$OUT/$SCENARIO-localization_node.log" 2>&1 &
    EKF_NODE_PID=$!
    ros2 run robotics_localization_bridge localization_evaluator --ros-args \
        -r __node:=localization_evaluator_ekf \
        -p "output_csv:=$EKF_CSV" -p "output_json:=$EKF_JSON" \
        -p "topics.odom:=$ODOM_TOPIC" -p "topics.estimate:=localization/pose" \
        > "$OUT/$SCENARIO-evaluator.log" 2>&1 &
    EKF_EVAL_PID=$!
fi
if $run_mcl; then
    log "starting mcl_node (MCL) and its evaluator against $ODOM_TOPIC"
    ros2 run robotics_localization_bridge mcl_node --ros-args \
        -p "map_file:=$MAP" \
        -p "topics.odom:=$ODOM_TOPIC" \
        > "$OUT/$SCENARIO-mcl_node.log" 2>&1 &
    MCL_NODE_PID=$!
    ros2 run robotics_localization_bridge localization_evaluator --ros-args \
        -r __node:=localization_evaluator_mcl \
        -p "output_csv:=$MCL_CSV" -p "output_json:=$MCL_JSON" \
        -p "topics.odom:=$ODOM_TOPIC" -p "topics.estimate:=mcl/pose" \
        > "$OUT/$SCENARIO-evaluator-mcl.log" 2>&1 &
    MCL_EVAL_PID=$!
fi
$run_ekf && { wait_for 60 "the first EKF pose estimate" has_topic /localization/pose || log "no EKF estimate topic yet"; }
$run_mcl && { wait_for 60 "the first MCL pose estimate" has_topic /mcl/pose || log "no MCL estimate topic yet"; }
sleep 3

# The route. route_driver closes the loop on ground truth and exits when the route is
# finished, so this script waits for it instead of timing the legs itself. An open-loop
# route cannot be driven for minutes here: a commanded angular.z is not the rate the robot
# turns at (the 2026-10-06 square was commanded 1.6 rad per corner and turned about 1.38),
# so the geometry of a long open-loop route is decided by wheel slip rather than by design.
log "driving route $ROUTE_FILE ($LAPS lap(s))"
ros2 run robotics_localization_bridge route_driver --ros-args \
    -p "route_file:=$ROUTE_FILE" -p "laps:=$LAPS" \
    > "$OUT/$SCENARIO-route_driver.log" 2>&1 &
ROUTE_PID=$!

kidnap() {
    log "teleporting the robot to ($KIDNAP_X, $KIDNAP_Y, $KIDNAP_YAW)"
    ros2 topic pub --once /kidnap/teleport geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: $KIDNAP_X, y: $KIDNAP_Y, z: 0.0}, \
          orientation: {x: 0.0, y: 0.0, z: $KIDNAP_QZ, w: $KIDNAP_QW}}}" > /dev/null 2>&1
}
if [ "$SCENARIO" = kidnap ]; then
    ( sleep "$KIDNAP_AFTER"; kidnap ) &
    KIDNAP_PID=$!
fi

wait "$ROUTE_PID"
log "route_driver exited with code $?"
if [ "$SCENARIO" = kidnap ]; then
    kill "$KIDNAP_PID" 2>/dev/null
    wait "$KIDNAP_PID" 2>/dev/null
fi
log "route finished, letting the filter(s) settle"
sleep 5

ros2 node list  > "$OUT/$SCENARIO-node_list.txt"  2>&1
ros2 topic list > "$OUT/$SCENARIO-topic_list.txt" 2>&1
timeout 15 ros2 topic echo --once /diagnostics > "$OUT/$SCENARIO-diagnostics.yaml" 2>&1
# No TF dump here. This script measures accuracy; whether the TF tree is complete is a
# property of the launch, and scripts/verify_phase3_localization.sh checks it edge by edge
# including the composed chains down to the sensor frames. See the NOTE-tf-capture.md of
# evidence/phase-3-correctness/2026-10-10-long-route/ for why an earlier attempt to also
# capture it here was abandoned rather than retried a third time.

stop() {  # stop <name> <pid>
    local name=$1 pid=$2
    [ -z "$pid" ] && return 0
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

log "stopping with SIGINT, evaluators first so that they write their summaries"
{
    stop localization_evaluator_ekf "$EKF_EVAL_PID"
    stop localization_evaluator_mcl "$MCL_EVAL_PID"
    stop localization_node "$EKF_NODE_PID"
    stop mcl_node "$MCL_NODE_PID"
    stop ros2_launch "$LAUNCH_PID"
    echo "webots_bin_left: $(pgrep -c -x webots-bin || true)"
} > "$OUT/$SCENARIO-shutdown.txt" 2>&1
cat "$OUT/$SCENARIO-shutdown.txt" | tee -a "$OUT/$SCENARIO-run.log"

check_and_verdict() {  # check_and_verdict <label> <csv> <json> <verdict_out>
    local label=$1 csv=$2 json=$3 verdict_out=$4
    if [ ! -s "$csv" ]; then
        log "FAIL ($label): no rows were written to $(basename "$csv")"
        return 1
    fi
    log "rows written ($label): $(( $(wc -l < "$csv") - 1 ))"
    log "ground_truth_max_displacement_m ($label): $(ground_truth_displacement "$csv")"
    if ! ground_truth_moved "$csv"; then
        log "FAIL ($label): the robot never moved; $(basename "$csv") grades the filter"
        log "FAIL ($label): against a stationary ground truth"
    fi
    if [ "$SCENARIO" = kidnap ]; then
        # The kidnap-specific verdict: error before and after the teleport, and whether and
        # when the estimate recovered. See scripts/phase3_kidnap_verdict.py for the
        # recovery definition.
        python3 -I "$SCRIPT_DIR/phase3_kidnap_verdict.py" "$json" "$csv" \
            "$OUT/$SCENARIO-environment.txt" "$label" > "$verdict_out" 2>&1
    else
        # Criterion P1 of docs/phase-3-localization/problem.md: the filter has to beat the
        # dead reckoning it was given.
        python3 -I "$SCRIPT_DIR/phase3_verdict.py" "$json" "$csv" \
            "$OUT/$SCENARIO-environment.txt" > "$verdict_out" 2>&1
    fi
    cat "$verdict_out" | tee -a "$OUT/$SCENARIO-run.log"
}

# The EKF's verdict file keeps the unsuffixed name for the normal scenario, the one every
# run before 2026-10-10 used and that docs/evidence already link to; the kidnap scenario has
# no such precedent, so its EKF verdict is named -ekf like the task that asked for it.
if $run_ekf; then
    if [ "$SCENARIO" = normal ]; then
        check_and_verdict ekf "$EKF_CSV" "$EKF_JSON" "$OUT/$SCENARIO-verdict.yaml"
    else
        check_and_verdict ekf "$EKF_CSV" "$EKF_JSON" "$OUT/$SCENARIO-verdict-ekf.yaml"
    fi
fi
if $run_mcl; then
    check_and_verdict mcl "$MCL_CSV" "$MCL_JSON" "$OUT/$SCENARIO-verdict-mcl.yaml"
fi
log "done, evidence in $OUT"
