#!/usr/bin/env bash
# One Phase 4 mapping run: the mapper against the Webots world with ground-truth poses.
#
#   bash scripts/run_phase4_mapping.sh <development|evaluation> <run name> [route file]
#
# The role is written into the run's summary.yaml and read by scripts/phase4_verdict.py. The
# development run is the one that fixes the mapper's parameters; the evaluation runs use
# routes that played no part in choosing them (docs/phase-4-mapping/problem.md section 4), and
# the verdict script checks that every run of a group used the same mapper binary and settings.
#
# Output goes to <EVIDENCE_ROOT>/phase-4-mapping/<run name>/, which scripts/evidence_root.sh
# resolves to the Windows checkout, so a run is written across the mount as it is produced and
# the rsync --delete of scripts/sync_to_wsl.sh cannot destroy it (it also excludes evidence/).
#
# What runs, through robotics_bringup/launch/phase4_mapping.launch.py:
#   Webots + official webots_ros2_turtlebot driver     (simulation, not Rust)
#   evaluation_supervisor.py                           (ground truth)
#   ground_truth_tf_adapter                            (Rust, owns map -> odom, SIMULATION ONLY)
#   mapping_node                                       (Rust, the thing under test)
#   mapping_evaluator                                  (Rust, grades /map against the world)
# and, started here after the controllers are active (the guard of scripts/phase3_guards.sh):
#   route_driver                                       (Rust, drives the route)
#
# Nothing a verdict says is printed here and copied by hand: every number is in a file.
set -u
set -m   # own process group per job; see the note in scripts/run_phase3_correctness.sh

ROLE=${1:?usage: run_phase4_mapping.sh <development|evaluation> <run name> [route file]}
NAME=${2:?usage: run_phase4_mapping.sh <development|evaluation> <run name> [route file]}
case "$ROLE" in development|evaluation) ;; *) echo "role must be development or evaluation" >&2; exit 2 ;; esac
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/phase3_guards.sh"
source "$SCRIPT_DIR/evidence_root.sh"
ROUTE_FILE=${3:-$REPO/sim/webots/config/route_long.yaml}
LAPS=${LAPS:-1}
NO_RETURN=${NO_RETURN:-clear_to_max_range}
TRACER=${TRACER:-supercover}
CLEAR_MARGIN=${CLEAR_MARGIN:-0.10}
SCAN_TIME_OFFSET=${SCAN_TIME_OFFSET:--0.06}
OUT="$EVIDENCE_ROOT/phase-4-mapping/$NAME"
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
SOURCE_REPO=$(dirname "$EVIDENCE_ROOT")
TAU_CELLS=$(python3 -I "$SCRIPT_DIR/phase4_verdict.py" --print-tau-cells)

export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1
log() { echo "[phase4 $ROLE] $*" | tee -a "$OUT/run.log"; }

if pgrep -x webots-bin > /dev/null; then
    log "FAIL: a webots-bin process is already running, stop it first"
    exit 1
fi
PREFIX=$(ros2 pkg prefix robotics_mapping_ros 2>/dev/null) || {
    log "FAIL: robotics_mapping_ros is not installed; run colcon build in ros2_ws"; exit 1; }
MAPPER="$PREFIX/lib/robotics_mapping_ros/mapping_node"
[ -x "$MAPPER" ] || { log "FAIL: $MAPPER not found"; exit 1; }

{
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)"
    echo "kernel: $(uname -r)"
    echo "os: $(. /etc/os-release && echo "$PRETTY_NAME")"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
    echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unavailable)"
    echo "ros_distro: ${ROS_DISTRO:-unset}"
    echo "rmw: ${RMW_IMPLEMENTATION:-default}"
    echo "webots: $(webots --version 2>&1 | head -1)"
    echo "rustc: $(rustc --version 2>&1)"
    echo "build_profile: debug (colcon default)"
    echo "world_sha256: $(sha256sum "$REPO/sim/webots/worlds/turtlebot3_localization_room.wbt" | cut -d' ' -f1)"
    echo "landmarks_sha256: $(sha256sum "$REPO/sim/webots/config/landmarks.yaml" | cut -d' ' -f1)"
    echo "route_file: $ROUTE_FILE"
    echo "route_sha256: $(sha256sum "$ROUTE_FILE" | cut -d' ' -f1)"
    echo "laps: $LAPS"
    echo "mapping_node_executable: $MAPPER"
    echo "mapping_node_executable_sha256: $(sha256sum "$MAPPER" | cut -d' ' -f1)"
    echo "verdict_script_sha256: $(sha256sum "$SCRIPT_DIR/phase4_verdict.py" | cut -d' ' -f1)"
    if git -C "$SOURCE_REPO" rev-parse --is-inside-work-tree > /dev/null 2>&1; then
        echo "source_git_commit: $(git -C "$SOURCE_REPO" rev-parse HEAD)"
        if [ -n "$(git -C "$SOURCE_REPO" status --porcelain -- crates ros2_ws scripts sim Cargo.toml Cargo.lock)" ]; then
            echo "source_git_dirty: true"
        else
            echo "source_git_dirty: false"
        fi
    else
        echo "source_git_commit: unavailable"
    fi
} > "$OUT/environment.txt"
if git -C "$SOURCE_REPO" rev-parse --is-inside-work-tree > /dev/null 2>&1; then
    git -C "$SOURCE_REPO" status --short -- crates ros2_ws scripts sim Cargo.toml Cargo.lock > "$OUT/source-status.txt"
    git -C "$SOURCE_REPO" diff --binary HEAD -- crates ros2_ws scripts sim Cargo.toml Cargo.lock > "$OUT/source.patch"
fi

log "starting the world, the adapter, the mapper and the evaluator"
ros2 launch robotics_bringup phase4_mapping.launch.py gui:=false mode:=realtime \
    owner:=ground_truth evaluate:=true capture:=true output_dir:="$OUT" tau_cells:="$TAU_CELLS" \
    no_return:="$NO_RETURN" tracer:="$TRACER" clear_margin:="$CLEAR_MARGIN" scan_time_offset:="$SCAN_TIME_OFFSET" > "$OUT/launch.log" 2>&1 &
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
abort() { kill -INT -- "-$LAUNCH_PID" 2>/dev/null; sleep 10; pkill -x webots-bin 2>/dev/null; exit 1; }

wait_for 180 "/scan" has_topic /scan || abort
wait_for 60 "/ground_truth/pose" has_topic /ground_truth/pose || abort
wait_for 60 "/map" has_topic /map || abort
if ! wait_for 90 "both controllers to become active" controllers_active; then
    ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
    log "FAIL: the controllers never activated, so the robot could not have moved; aborting"
    abort
fi
ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
log "controllers are active; letting the adapter and the mapper settle"
sleep 8

# Provenance, recorded from the running graph and not from what this script meant to start.
ros2 node list > "$OUT/node_list.txt" 2>&1
ros2 topic list -t > "$OUT/topic_list.txt" 2>&1
ros2 node info /mapping_node > "$OUT/mapping_node_info.txt" 2>&1
ros2 topic info /tf -v > "$OUT/tf_publishers.txt" 2>&1
OWNERS=""
grep -qx "/ground_truth_tf_adapter" "$OUT/node_list.txt" && OWNERS="ground_truth_tf_adapter"
if grep -qx "/localization_node" "$OUT/node_list.txt"; then
    OWNERS="${OWNERS:+$OWNERS+}localization_node"
fi
case "$OWNERS" in
    ground_truth_tf_adapter|localization_node) OWNER=$OWNERS ;;
    "") OWNER=none ;;
    *) OWNER="conflict:$OWNERS" ;;
esac
# The mapper's own subscriptions, taken from `ros2 node info`: the lines between
# "Subscribers:" and the next section header.
if awk '/^  Subscribers:/{s=1;next} /^  [A-Z][A-Za-z ]*:/{s=0} s' "$OUT/mapping_node_info.txt" \
        | grep -q "ground_truth"; then
    SUBSCRIBES_GT=true
else
    SUBSCRIBES_GT=false
fi
log "owner of map -> odom: $OWNER; mapping_node subscribes to ground truth: $SUBSCRIBES_GT"

log "driving route $ROUTE_FILE ($LAPS lap(s))"
ros2 run robotics_localization_bridge route_driver --ros-args \
    -p "route_file:=$ROUTE_FILE" -p "laps:=$LAPS" > "$OUT/route_driver.log" 2>&1 &
ROUTE_PID=$!
wait "$ROUTE_PID"
ROUTE_RC=$?
log "route_driver exited with code $ROUTE_RC"
log "route finished, letting the last scans be integrated"
sleep 6
timeout 15 ros2 topic echo --once /diagnostics > "$OUT/diagnostics.yaml" 2>&1
timeout 15 ros2 topic echo --once /esdf --field encoding > "$OUT/esdf_sample.txt" 2>&1
timeout 15 ros2 topic echo --once /esdf --field width >> "$OUT/esdf_sample.txt" 2>&1
timeout 15 ros2 topic echo --once /esdf --field step >> "$OUT/esdf_sample.txt" 2>&1

log "stopping with SIGINT: the mapper writes its grid and the evaluator grades, both at shutdown"
kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null
for _ in $(seq 1 400); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ros2_launch: still alive 400 s after SIGINT" >> "$OUT/shutdown.txt"
    kill -9 -- "-$LAUNCH_PID" 2>/dev/null || kill -9 "$LAUNCH_PID" 2>/dev/null
else
    wait "$LAUNCH_PID"; echo "ros2_launch: exit code $?" >> "$OUT/shutdown.txt"
fi
pkill -x webots-bin 2>/dev/null
echo "webots_bin_left: $(pgrep -c -x webots-bin || true)" >> "$OUT/shutdown.txt"

# M2: replay what the mapper integrated and compare with the grid it wrote.
REPLAY_SCANS=none
if [ -s "$OUT/integrated-scans.csv" ] && [ -s "$OUT/final-grid.txt" ]; then
    log "replaying the recorded scans (claim M2)"
    ( cd "$REPO" && cargo run -q --release -p robotics_mapping --example replay_scans -- \
        "$OUT/integrated-scans.csv" "$OUT/final-grid.txt" "$OUT/replay-grid.txt" \
        "$NO_RETURN" "$TRACER" "$CLEAR_MARGIN" ) > "$OUT/replay.log" 2>&1
    REPLAY_SCANS=$(grep -o 'replayed [0-9]*' "$OUT/replay.log" | grep -o '[0-9]*' || echo none)
fi

{
    echo "role: $ROLE"
    echo "owner_of_map_odom: $OWNER"
    echo "mapping_node_subscribes_ground_truth: $SUBSCRIBES_GT"
    echo "route_driver_exit_code: $ROUTE_RC"
    echo "replay_scans: $REPLAY_SCANS"
    echo "launch_arguments: owner=ground_truth no_return=$NO_RETURN tracer=$TRACER clear_margin=$CLEAR_MARGIN scan_time_offset=$SCAN_TIME_OFFSET tau_cells=$TAU_CELLS route=$(basename "$ROUTE_FILE") laps=$LAPS"
    grep -E '^(source_git_commit|source_git_dirty|mapping_node_executable_sha256|route_sha256|world_sha256):' "$OUT/environment.txt"
} > "$OUT/summary.yaml"

log "grading"
python3 -I "$SCRIPT_DIR/phase4_verdict.py" "$OUT" > "$OUT/verdict.console" 2>&1
log "verdict written to $OUT/phase4-verdict.yaml"
