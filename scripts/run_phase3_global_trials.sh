#!/usr/bin/env bash
# Global localization from a uniform prior in the Webots simulation, end to end: the real
# mcl_node, the real pillar detector, the real laser, the real odometry, graded against the
# simulator's ground truth. Many trials, one Webots session.
#
#   STARTS=<csv of x,y,yaw> bash scripts/run_phase3_global_trials.sh <run-name>
#
# One trial is: teleport the robot to a start pose through the evaluation supervisor, then
# start a fresh mcl_node with initial_pose.mode:=uniform_prior (it is told nothing about
# where the robot is), a fresh localization_evaluator, and drive the exploration route that
# `global_trials route` generates for that start. The trial ends when the route does. A
# fresh node per trial is what makes the trials independent; one Webots session for all of
# them is what makes a batch affordable (Webots takes about a minute to come up).
#
# The robot is placed *before* the estimator and the evaluator exist, so neither is told,
# and the evaluator never records a kidnap event: nothing in a trial is a kidnapping.
# The odometry is not reset by the teleport (it integrates wheel motion only), which is
# harmless: the filter reads odometry as increments.
#
# Evidence goes to <EVIDENCE_ROOT>/phase-3-global-localization/<run-name>/trial-NN/, the
# Windows checkout when it is mounted (scripts/evidence_root.sh), and an existing run
# directory is never overwritten.
#
# Environment:
#   STARTS               CSV of x,y,yaw start poses (required)
#   PARTICLES            global.particles for the search            (default 6000)
#   EXPLAINED_FRACTION   global.convergence_min_explained_fraction  (default 0.5)
#   SEED                 mcl_node seed of trial 0, +1 per trial     (default 1)
#   RELEASE              1 to run the release build of the nodes from ros2_ws/install_release
#                        (see scripts/build_release.sh), anything else for the debug build
#                        every other script uses                    (default 0)
#   LAPS                 laps of the exploration square             (default 1)
#   MAX_TRIALS           run only the first N rows of STARTS        (default all)

set -u
set -m   # every job in its own process group, so that `stop` can SIGINT the whole `ros2 run`

RUN=${1:?usage: STARTS=<csv> run_phase3_global_trials.sh <run-name>}
STARTS=${STARTS:?STARTS=<csv of x,y,yaw> is required}
PARTICLES=${PARTICLES:-6000}
EXPLAINED_FRACTION=${EXPLAINED_FRACTION:-0.5}
SEED=${SEED:-1}
RELEASE=${RELEASE:-0}
LAPS=${LAPS:-1}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/build_cache.sh"
source "$SCRIPT_DIR/phase3_guards.sh"
source "$SCRIPT_DIR/evidence_root.sh"
if [ "$RELEASE" = 1 ]; then
    [ -f "$REPO/ros2_ws/install_release/setup.bash" ] || {
        echo "RELEASE=1 but ros2_ws/install_release does not exist; run scripts/build_release.sh" >&2
        exit 2
    }
    # colcon's setup.bash reads variables it never sets, which `set -u` turns into a fatal error.
    set +u
    source "$REPO/ros2_ws/install_release/setup.bash"
    set -u
    BIN_DIR="$REPO/ros2_ws/install_release/robotics_localization_bridge/lib/robotics_localization_bridge"
    BUILD=release
else
    BIN_DIR="$REPO/ros2_ws/install/robotics_localization_bridge/lib/robotics_localization_bridge"
    BUILD=debug
fi
LAUNCH="$REPO/sim/webots/launch/localization_world.launch.py"
MAP="$REPO/sim/webots/config/landmarks.yaml"
TRIALS_BIN="${CARGO_TARGET_DIR:-$REPO/target}/release/examples/global_trials"
( cd "$REPO" && cargo build --release -p robotics_localization --example global_trials ) \
    > /dev/null 2>&1 || { echo "cannot build global_trials" >&2; exit 2; }

OUT="$EVIDENCE_ROOT/phase-3-global-localization/$RUN"
if [ -e "$OUT" ]; then
    echo "refusing to overwrite $OUT (evidence is never overwritten); pick a new run name" >&2
    exit 1
fi
mkdir -p "$OUT"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1
log() { echo "[$RUN] $*" | tee -a "$OUT/run.log"; }

if pgrep -x webots-bin > /dev/null; then
    log "FAIL: a webots-bin process is already running, stop it first"
    exit 1
fi

mapfile -t START_ROWS < <(grep -vE '^\s*(#|x,|$)' "$STARTS")
[ -n "${MAX_TRIALS:-}" ] && START_ROWS=("${START_ROWS[@]:0:$MAX_TRIALS}")
cp "$STARTS" "$OUT/starts.csv"
{
    echo "run: $RUN"
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)"
    echo "kernel: $(uname -r)"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
    echo "ros_distro: ${ROS_DISTRO:-unset}"
    echo "rmw: ${RMW_IMPLEMENTATION:-default}"
    echo "webots: $(webots --version 2>&1 | head -1)"
    echo "rustc: $(rustc --version 2>&1)"
    echo "build: $BUILD"
    echo "mcl_node_sha256: $(sha256sum "$BIN_DIR/mcl_node" | cut -d' ' -f1)"
    echo "map_sha256: $(sha256sum "$MAP" | cut -d' ' -f1)"
    echo "world_sha256: $(sha256sum "$REPO/sim/webots/worlds/turtlebot3_localization_room.wbt" | cut -d' ' -f1)"
    echo "starts_file: $STARTS"
    echo "starts_sha256: $(sha256sum "$STARTS" | cut -d' ' -f1)"
    echo "trials: ${#START_ROWS[@]}"
    echo "global_particles: $PARTICLES"
    echo "steady_state_particles: 6000"
    echo "convergence_min_explained_fraction: $EXPLAINED_FRACTION"
    echo "seed_of_trial_0: $SEED"
    echo "laps: $LAPS"
    # The WSL mirror is not a git repository; the Windows checkout is.
    echo "git_head: $(git -C "$_EVIDENCE_WINDOWS_CHECKOUT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    echo "odometry_noise: off"
} > "$OUT/environment.txt"

log "starting the world with ground truth ($BUILD build, ${#START_ROWS[@]} trials, $PARTICLES particles)"
ros2 launch "$LAUNCH" gui:=false mode:=realtime ground_truth:=true \
    > "$OUT/launch.log" 2>&1 &
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

stop() {  # stop <name> <pid>   (the same reasoning as run_phase3_correctness.sh)
    local name=$1 pid=$2
    [ -z "$pid" ] && return 0
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    for _ in $(seq 1 30); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 1
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "$name: still alive 30 s after SIGINT"
        kill -9 -- "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
    else
        wait "$pid"
        echo "$name: exit code $?"
    fi
}

wait_for 180 "/scan" has_topic /scan || { kill -INT "$LAUNCH_PID"; exit 1; }
wait_for 60 "/ground_truth/pose" has_topic /ground_truth/pose || { kill -INT "$LAUNCH_PID"; exit 1; }
wait_for 90 "both controllers to become active" controllers_active || {
    log "FAIL: the controllers never activated, so no robot could be driven"
    kill -INT "$LAUNCH_PID"; exit 1
}
log "simulation is up, controllers are active"

teleport() {  # teleport <x> <y> <yaw>
    local qz qw
    qz=$(awk "BEGIN {printf \"%.10f\", sin($3 / 2)}")
    qw=$(awk "BEGIN {printf \"%.10f\", cos($3 / 2)}")
    ros2 topic pub --once /kidnap/teleport geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: 0.0}, \
          orientation: {x: 0.0, y: 0.0, z: $qz, w: $qw}}}" > /dev/null 2>&1
}

trial=0
for row in "${START_ROWS[@]}"; do
    IFS=, read -r SX SY SYAW <<< "$row"
    SX=${SX// /}; SY=${SY// /}; SYAW=${SYAW// /}
    T=$(printf '%s/trial-%02d' "$OUT" "$trial")
    mkdir -p "$T"
    echo "start: x=$SX y=$SY yaw=$SYAW" > "$T/start.txt"

    if ! "$TRIALS_BIN" route --map "$MAP" --x "$SX" --y "$SY" --yaw "$SYAW" --laps "$LAPS" \
            > "$T/route.yaml" 2> "$T/route.err"; then
        log "trial $trial: no clear exploration route from ($SX, $SY, $SYAW), skipped"
        echo "skipped: no clear exploration route" > "$T/SKIPPED.txt"
        trial=$((trial + 1)); continue
    fi

    log "trial $trial: placing the robot at ($SX, $SY, $SYAW)"
    teleport "$SX" "$SY" "$SYAW"
    sleep 4    # the supervisor applies it between two steps; let the lidar and odometry settle

    ros2 run robotics_localization_bridge mcl_node --ros-args \
        -p "map_file:=$MAP" -p topics.odom:=/odom \
        -p initial_pose.mode:=uniform_prior \
        -p "global.particles:=$PARTICLES" \
        -p "global.convergence_min_explained_fraction:=$EXPLAINED_FRACTION" \
        -p "seed:=$((SEED + trial))" \
        > "$T/mcl_node.log" 2>&1 &
    NODE_PID=$!
    ros2 run robotics_localization_bridge localization_evaluator --ros-args \
        -r __node:=localization_evaluator_mcl \
        -p "output_csv:=$T/run.csv" -p "output_json:=$T/run.json" \
        -p topics.odom:=/odom -p topics.estimate:=mcl/pose \
        > "$T/evaluator.log" 2>&1 &
    EVAL_PID=$!
    wait_for 60 "the first MCL pose estimate" has_topic /mcl/pose || log "no MCL estimate topic yet"
    sleep 3

    ros2 run robotics_localization_bridge route_driver --ros-args \
        -p "route_file:=$T/route.yaml" -p laps:=1 > "$T/route_driver.log" 2>&1 &
    ROUTE_PID=$!
    wait "$ROUTE_PID"
    log "trial $trial: route_driver exited with code $?"
    sleep 5
    # mcl_node's own status, selected by name. A bare `--once` takes whichever node publishes
    # on /diagnostics first, which is often not mcl_node, and left the first Webots runs with
    # the detector and timing readings missing for most trials. The filter makes `--once`
    # wait for a message that carries the mcl_node status; mcl_node publishes one a second
    # (diagnostics_period_s), so the timeout only matters if the node is gone, and then the
    # empty file is what the verdict reports as `diagnostics_captured: False`.
    timeout 20 ros2 topic echo --once \
        --filter "any(s.name.startswith('mcl_node') for s in m.status)" \
        /diagnostics > "$T/diagnostics.yaml" 2>&1

    {
        stop localization_evaluator "$EVAL_PID"
        stop mcl_node "$NODE_PID"
    } > "$T/shutdown.txt" 2>&1
    cat "$T/shutdown.txt" >> "$OUT/run.log"
    python3 -I "$SCRIPT_DIR/phase3_global_verdict.py" trial "$T" > "$T/verdict.yaml" 2>&1
    log "trial $trial: $(grep -E '^  category:' "$T/verdict.yaml" | head -1 | sed 's/^ *//')"
    trial=$((trial + 1))
done

log "all trials done, stopping the world"
{
    stop ros2_launch "$LAUNCH_PID"
    echo "webots_bin_left: $(pgrep -c -x webots-bin || true)"
} > "$OUT/shutdown.txt" 2>&1
cat "$OUT/shutdown.txt" >> "$OUT/run.log"
python3 -I "$SCRIPT_DIR/phase3_global_verdict.py" summary "$OUT" > "$OUT/summary.yaml" 2>&1
cat "$OUT/summary.yaml" | tee -a "$OUT/run.log"
log "done, evidence in $OUT"
