#!/usr/bin/env bash
# Phase 5, Stage D: the SLAM nodes against a LIVE Webots run, `phase5_slam.launch.py source:=live`.
#
#   bash scripts/run_phase5_live.sh <run name> [route file] [loop_closure on|off]
#
# What runs, in order:
#   Webots + the official webots_ros2_turtlebot driver + evaluation_supervisor.py (ground truth)
#                                                            sim/webots/launch/localization_world.launch.py
#   probes (harness): scripts/phase5_map_tf_probe.py   /map /tf /tf_static /slam/path /diagnostics
#                     scripts/phase5_record_gt.py       the dense ground-truth track
#                     scripts/phase5_record_scans.py    every /scan, as integrated-scans.csv, ground-truth pose columns
#   ros2 launch robotics_bringup phase5_slam.launch.py source:=live ...     the two SLAM nodes. NOT the replayer.
#   route_driver (Rust, closed loop on the ground truth), started once the controllers are active.
#
# Nothing here starts ground_truth_tf_adapter or any other owner of `map -> odom`: pose_graph_node owns it. The
# SLAM nodes read /scan and the driver's TF and nothing else; the ground truth is read by the harness above only.
#
# Output: <EVIDENCE_ROOT>/phase-5-slam/<run name>/ ; it REFUSES to overwrite a run that has a recording.
set -u
set -m   # own process group per job; see the note in scripts/run_phase3_correctness.sh

NAME=${1:?usage: run_phase5_live.sh <run name> [route file] [loop_closure on|off]}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
WS=${WS:-$HOME/slam_nodes_ws}
source "$SCRIPT_DIR/phase3_guards.sh"
source "$SCRIPT_DIR/evidence_root.sh"
set +u; source "$WS/scripts/env.sh" > /dev/null 2>&1; source "$WS/ros2_ws/install_release/setup.bash" > /dev/null 2>&1; set -u
export ROS_DOMAIN_ID=${PHASE5_LIVE_ROS_DOMAIN_ID:-92}
[ "$ROS_DOMAIN_ID" = 42 ] && { echo "refusing ROS domain 42" >&2; exit 3; }
ROUTE_FILE=${2:-$REPO/sim/webots/config/route_slam_eval_rectangle.yaml}
LOOP=${3:-on}
case "$LOOP" in on) LC=true ;; off) LC=false ;; *) echo "loop_closure must be on or off" >&2; exit 2 ;; esac
LAPS=${LAPS:-3}
OUT="$EVIDENCE_ROOT/phase-5-slam/$NAME"
if [ -e "$OUT/integrated-scans.csv" ]; then
    echo "refusing to overwrite $OUT/integrated-scans.csv: evidence is never replaced" >&2
    exit 3
fi
mkdir -p "$OUT/node"
OUT=$(cd "$OUT" && pwd)
export RCUTILS_LOGGING_BUFFERED_STREAM=0 PYTHONUNBUFFERED=1
log() { echo "[phase5 live] $*" | tee -a "$OUT/run.log"; }

pgrep -x webots-bin > /dev/null && { log "FAIL: a webots-bin process is already running, stop it first"; exit 1; }
[ -f "$ROUTE_FILE" ] || { log "FAIL: no route file $ROUTE_FILE"; exit 1; }
ros2 pkg prefix robotics_slam_ros > /dev/null 2>&1 || { log "FAIL: robotics_slam_ros is not installed (bash scripts/build_release.sh slam in $WS)"; exit 1; }
BIN="$(ros2 pkg prefix robotics_slam_ros)/lib/robotics_slam_ros"
{
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)"
    echo "kernel: $(uname -r)"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
    echo "ros_distro: ${ROS_DISTRO:-unset}"
    echo "ros_domain_id: $ROS_DOMAIN_ID"
    echo "rmw: ${RMW_IMPLEMENTATION:-default}"
    echo "webots: $(webots --version 2>&1 | head -1)"
    echo "world_sha256: $(sha256sum "$REPO/sim/webots/worlds/turtlebot3_localization_room.wbt" | cut -d' ' -f1)"
    echo "route_file: $ROUTE_FILE"
    echo "route_sha256: $(sha256sum "$ROUTE_FILE" | cut -d' ' -f1)"
    echo "laps: $LAPS"
    echo "loop_closure: $LOOP"
    echo "install_space: $(ros2 pkg prefix robotics_slam_ros)"
    for exe in scan_matcher_node pose_graph_node; do echo "${exe}_sha256: $(sha256sum "$BIN/$exe" | cut -d' ' -f1)"; done
    echo "launch_file_sha256: $(sha256sum "$REPO/ros2_ws/src/robotics_bringup/launch/phase5_slam.launch.py" | cut -d' ' -f1)"
    echo "build_profile: release (install_release)"
    echo "source_git_commit: $(git -C "$(dirname "$EVIDENCE_ROOT")" rev-parse HEAD 2>/dev/null || echo unavailable)"
} > "$OUT/environment.txt"

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
has_node() { ros2 node list 2>/dev/null | grep -qx "$1"; }
abort() { kill -INT -- "-${SLAM_PID:-0}" 2>/dev/null; kill -INT -- "-$WORLD_PID" 2>/dev/null; sleep 10; pkill -x webots-bin 2>/dev/null; exit 1; }

log "starting Webots, the official driver and the ground-truth supervisor"
ros2 launch "$REPO/sim/webots/launch/localization_world.launch.py" gui:=false mode:=realtime ground_truth:=true > "$OUT/world.log" 2>&1 &
WORLD_PID=$!
wait_for 180 "/scan" has_topic /scan || abort
wait_for 60 "/ground_truth/pose" has_topic /ground_truth/pose || abort
if ! wait_for 90 "both controllers to become active" controllers_active; then
    ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
    log "FAIL: the controllers never activated, so the robot could not have moved; aborting"; abort
fi
ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
log "controllers are active"

python3 "$SCRIPT_DIR/phase5_map_tf_probe.py" "$OUT/probe.json" > "$OUT/probe.log" 2>&1 &
PROBE_PID=$!
python3 "$SCRIPT_DIR/phase5_record_gt.py" "$OUT/ground-truth.csv" > "$OUT/ground-truth.log" 2>&1 &
GT_PID=$!
python3 "$SCRIPT_DIR/phase5_record_scans.py" "$OUT/integrated-scans.csv" > "$OUT/scan-recorder.log" 2>&1 &
REC_PID=$!
sleep 3

log "starting the SLAM nodes: phase5_slam.launch.py source:=live loop_closure:=$LC (no replayer, no ground_truth_tf_adapter)"
ros2 launch robotics_bringup phase5_slam.launch.py source:=live loop_closure:=$LC publish_map:=true publish_map_odom:=true \
    capture:=true output_dir:="$OUT/node" > "$OUT/slam_launch.log" 2>&1 &
SLAM_PID=$!
wait_for 60 "/pose_graph_node" has_node /pose_graph_node || abort
wait_for 60 "/scan_matcher_node" has_node /scan_matcher_node || abort
sleep 5

# Provenance, read from the running graph and not from what this script meant to start.
ros2 node list > "$OUT/node_list.txt" 2>&1
ros2 topic list -t > "$OUT/topic_list.txt" 2>&1
ros2 topic info /scan -v > "$OUT/scan_publishers_subscribers.txt" 2>&1
ros2 topic info /tf -v > "$OUT/tf_publishers.txt" 2>&1
ros2 node info /scan_matcher_node > "$OUT/scan_matcher_node-info.txt" 2>&1
ros2 node info /pose_graph_node > "$OUT/pose_graph_node-info.txt" 2>&1
ros2 param get /pose_graph_node loop_closure.enabled > "$OUT/loop_closure_param.txt" 2>&1
ros2 param get /pose_graph_node tf.publish_map_odom >> "$OUT/loop_closure_param.txt" 2>&1
grep -qx "/ground_truth_tf_adapter" "$OUT/node_list.txt" && log "FAIL: ground_truth_tf_adapter is running: two owners of map -> odom" && abort
log "SLAM nodes are up; scan_matcher_node subscribes to /scan: $(grep -c '/scan' "$OUT/scan_matcher_node-info.txt") line(s) of its node info mention it"

log "driving route $ROUTE_FILE ($LAPS lap(s))"
ros2 run robotics_localization_bridge route_driver --ros-args -p "route_file:=$ROUTE_FILE" -p "laps:=$LAPS" > "$OUT/route_driver.log" 2>&1 &
ROUTE_PID=$!
sleep 30
timeout 8 ros2 topic echo --once /diagnostics > "$OUT/diagnostics-mid-run.txt" 2>&1
wait "$ROUTE_PID"
ROUTE_RC=$?
log "route_driver exited with code $ROUTE_RC"
sleep 8
timeout 8 ros2 topic echo --once /diagnostics > "$OUT/diagnostics-end.txt" 2>&1

# Order, changed on 2026-10-15 from "probe, then SLAM nodes" (run live-1): the nodes publish their last /map and
# their last map -> odom while they shut down, and a probe that is already gone cannot count them (three of the five
# Stage B checks that failed on live-1 were that). The scan and ground-truth recorders stop first because nothing
# after the route is graded on them; the probe stops last.
log "stopping: the scan and ground-truth recorders, then the SLAM nodes (they write their final files at shutdown), then the probe, then Webots"
kill -INT "$REC_PID" "$GT_PID" 2>/dev/null; wait "$REC_PID" "$GT_PID" 2>/dev/null
kill -INT -- "-$SLAM_PID" 2>/dev/null || kill -INT "$SLAM_PID" 2>/dev/null
for _ in $(seq 1 120); do kill -0 "$SLAM_PID" 2>/dev/null || break; sleep 1; done
if kill -0 "$SLAM_PID" 2>/dev/null; then echo "slam_launch: still alive 120 s after SIGINT" >> "$OUT/shutdown.txt"; kill -9 -- "-$SLAM_PID" 2>/dev/null; else wait "$SLAM_PID"; echo "slam_launch: exit code $?" >> "$OUT/shutdown.txt"; fi
sleep 2
kill -INT "$PROBE_PID" 2>/dev/null; wait "$PROBE_PID" 2>/dev/null
kill -INT -- "-$WORLD_PID" 2>/dev/null || kill -INT "$WORLD_PID" 2>/dev/null
for _ in $(seq 1 60); do kill -0 "$WORLD_PID" 2>/dev/null || break; sleep 1; done
kill -0 "$WORLD_PID" 2>/dev/null && kill -9 -- "-$WORLD_PID" 2>/dev/null
pkill -x webots-bin 2>/dev/null; sleep 2
echo "webots_bin_left: $(pgrep -c -x webots-bin || true)" >> "$OUT/shutdown.txt"
echo "route_driver_exit_code: $ROUTE_RC" >> "$OUT/shutdown.txt"

if [ -s "$OUT/integrated-scans.csv" ]; then
    python3 -I "$SCRIPT_DIR/phase5_recording_turns.py" --yaml "$OUT/recording-turns.yaml" "$OUT/integrated-scans.csv" | tee "$OUT/recording-turns.txt" | tee -a "$OUT/run.log"
else
    log "no integrated-scans.csv was written"
fi
log "done: $OUT"
