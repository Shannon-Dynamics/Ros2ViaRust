#!/usr/bin/env bash
# Phase 5, Stage A: drive a route in Webots with the official driver and record EVERY scan.
#
#   bash scripts/run_phase5_record.sh <run name> [route file]
#
# Output: <EVIDENCE_ROOT>/phase-5-slam/<run name>/ (scripts/evidence_root.sh: the Windows checkout)
#   integrated-scans.csv   every scan of the run, in the schema of the Phase 4 recordings (so every Phase 5
#                          tool reads it unchanged); the pose columns are the ground truth at the measurement
#                          instant (stamp + scan_time_offset_s, -0.06 s) as the Phase 4 mapper looked it up
#   ground-truth.csv       the dense ground-truth track, 50 Hz (scripts/phase5_record_gt.py)
#   recording-turns.yaml   what scripts/phase5_recording_turns.py finds in the recording (the Stage A gate)
#   environment.txt, launch.log, route_driver.log, run.log, controllers.txt, node_list.txt, shutdown.txt
#
# What differs from scripts/run_phase4_mapping.sh, and it is the whole point: the mapper node is the same binary
# with its motion gate and motion filter OFF (launch/phase5_record.launch.py), so scans taken while the robot
# turns are recorded instead of skipped. Nothing here grades anything.
#
# It REFUSES to overwrite a run that already has a recording: evidence is never replaced.
set -u
set -m   # own process group per job; see the note in scripts/run_phase3_correctness.sh

NAME=${1:?usage: run_phase5_record.sh <run name> [route file]}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/phase3_guards.sh"
source "$SCRIPT_DIR/evidence_root.sh"
source "$SCRIPT_DIR/env.sh" > /dev/null 2>&1
ROUTE_FILE=${2:-$REPO/sim/webots/config/route_slam_eval_rectangle.yaml}
LAPS=${LAPS:-3}
DRIVE_ANGULAR=${DRIVE_ANGULAR:-}
SCAN_TIME_OFFSET=${SCAN_TIME_OFFSET:--0.06}
OUT="$EVIDENCE_ROOT/phase-5-slam/$NAME"
if [ -e "$OUT/integrated-scans.csv" ]; then
    echo "refusing to overwrite $OUT/integrated-scans.csv: evidence is never replaced" >&2
    exit 3
fi
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
export RCUTILS_LOGGING_BUFFERED_STREAM=0 PYTHONUNBUFFERED=1
log() { echo "[phase5 record] $*" | tee -a "$OUT/run.log"; }

pgrep -x webots-bin > /dev/null && { log "FAIL: a webots-bin process is already running, stop it first"; exit 1; }
[ -f "$ROUTE_FILE" ] || { log "FAIL: no route file $ROUTE_FILE"; exit 1; }
PREFIX=$(ros2 pkg prefix robotics_mapping_ros 2>/dev/null) || { log "FAIL: robotics_mapping_ros is not installed"; exit 1; }
MAPPER="$PREFIX/lib/robotics_mapping_ros/mapping_node"
[ -x "$MAPPER" ] || { log "FAIL: $MAPPER not found"; exit 1; }
LAUNCH_FILE="$REPO/ros2_ws/src/robotics_bringup/launch/phase5_record.launch.py"

{
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)"
    echo "kernel: $(uname -r)"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
    echo "ros_distro: ${ROS_DISTRO:-unset}"
    echo "rmw: ${RMW_IMPLEMENTATION:-default}"
    echo "webots: $(webots --version 2>&1 | head -1)"
    echo "world_sha256: $(sha256sum "$REPO/sim/webots/worlds/turtlebot3_localization_room.wbt" | cut -d' ' -f1)"
    echo "route_file: $ROUTE_FILE"
    echo "route_sha256: $(sha256sum "$ROUTE_FILE" | cut -d' ' -f1)"
    echo "laps: $LAPS"
    echo "drive_angular_override: ${DRIVE_ANGULAR:-none}"
    echo "scan_time_offset_s: $SCAN_TIME_OFFSET"
    echo "motion_gate: off"
    echo "motion_filter: off"
    echo "launch_file_sha256: $(sha256sum "$LAUNCH_FILE" | cut -d' ' -f1)"
    echo "mapping_node_executable_sha256: $(sha256sum "$MAPPER" | cut -d' ' -f1)"
    echo "source_git_commit: $(git -C "$(dirname "$EVIDENCE_ROOT")" rev-parse HEAD 2>/dev/null || echo unavailable)"
} > "$OUT/environment.txt"

log "starting the world, the ground-truth adapter and the recording mapper (gate and filter off)"
ros2 launch "$LAUNCH_FILE" repo:="$REPO" gui:=false mode:=realtime output_dir:="$OUT" scan_time_offset:="$SCAN_TIME_OFFSET" > "$OUT/launch.log" 2>&1 &
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
if ! wait_for 90 "both controllers to become active" controllers_active; then
    ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
    log "FAIL: the controllers never activated, so the robot could not have moved; aborting"
    abort
fi
ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
log "controllers are active; letting the adapter and the mapper settle"
sleep 8
ros2 node list > "$OUT/node_list.txt" 2>&1
ros2 topic list -t > "$OUT/topic_list.txt" 2>&1
ros2 param get /mapping_node motion_gate.window_s > "$OUT/mapper_gate_param.txt" 2>&1
ros2 param get /mapping_node motion_filter.min_translation_m >> "$OUT/mapper_gate_param.txt" 2>&1
ros2 topic info /tf -v > "$OUT/tf_publishers.txt" 2>&1

# not -I: rclpy lives on PYTHONPATH, which isolated mode discards (the first Stage A run lost its ground-truth.csv to that)
python3 "$SCRIPT_DIR/phase5_record_gt.py" "$OUT/ground-truth.csv" > "$OUT/ground-truth.log" 2>&1 &
GT_PID=$!
sleep 2

log "driving route $ROUTE_FILE ($LAPS lap(s))"
ROUTE_ARGS=(-p "route_file:=$ROUTE_FILE" -p "laps:=$LAPS")
[ -n "$DRIVE_ANGULAR" ] && ROUTE_ARGS+=(-p "drive.angular:=$DRIVE_ANGULAR")
ros2 run robotics_localization_bridge route_driver --ros-args "${ROUTE_ARGS[@]}" > "$OUT/route_driver.log" 2>&1 &
ROUTE_PID=$!
sleep 4
ros2 param get /route_driver drive.angular > "$OUT/route_driver_angular.txt" 2>&1
wait "$ROUTE_PID"
ROUTE_RC=$?
log "route_driver exited with code $ROUTE_RC"
log "route finished, letting the last scans be recorded"
sleep 6

kill -INT "$GT_PID" 2>/dev/null
wait "$GT_PID" 2>/dev/null
log "stopping with SIGINT: the mapper closes its recording at shutdown"
kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null
for _ in $(seq 1 120); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ros2_launch: still alive 120 s after SIGINT" >> "$OUT/shutdown.txt"
    kill -9 -- "-$LAUNCH_PID" 2>/dev/null || kill -9 "$LAUNCH_PID" 2>/dev/null
else
    wait "$LAUNCH_PID"; echo "ros2_launch: exit code $?" >> "$OUT/shutdown.txt"
fi
pkill -x webots-bin 2>/dev/null
sleep 2
echo "webots_bin_left: $(pgrep -c -x webots-bin || true)" >> "$OUT/shutdown.txt"

# The Stage A gate: did the robot move, and does the recording have turning scans?
python3 - "$OUT/ground-truth.csv" <<'PY' | tee -a "$OUT/run.log"
import sys, math, os
if not os.path.exists(sys.argv[1]):
    print("[phase5 record] GATE FAIL: no ground-truth.csv was written (see ground-truth.log)"); sys.exit(0)
rows = [l.split(",") for l in open(sys.argv[1]) if not l.startswith("#") and l.strip()]
if not rows:
    print("[phase5 record] GATE FAIL: no ground-truth rows"); sys.exit(0)
x0, y0 = float(rows[0][1]), float(rows[0][2])
d = max(math.hypot(float(r[1]) - x0, float(r[2]) - y0) for r in rows)
print(f"[phase5 record] ground truth: {len(rows)} poses, largest distance from the first pose {d:.3f} m ({'moved' if d >= 0.2 else 'DID NOT MOVE'})")
PY
if [ -s "$OUT/integrated-scans.csv" ]; then
    python3 -I "$SCRIPT_DIR/phase5_recording_turns.py" --yaml "$OUT/recording-turns.yaml" "$OUT/integrated-scans.csv" | tee "$OUT/recording-turns.txt" | tee -a "$OUT/run.log"
else
    log "GATE FAIL: no integrated-scans.csv was written"
fi
echo "route_driver_exit_code: $ROUTE_RC" >> "$OUT/shutdown.txt"
log "done: $OUT"
