#!/usr/bin/env bash
# Record the lidar scan-skew evidence of Phase 4 open question 3, then analyse it.
#
#   bash scripts/run_phase4_scan_skew.sh <run name>
#
# Starts the Phase 3 world with ground truth, lets scripts/phase4_scan_skew_record.py hold the
# robot still, drive straight, and turn in place while it records /scan and
# /ground_truth/pose, stops the simulation, and runs scripts/phase4_scan_skew_analysis.py on
# what was recorded. Output goes to <EVIDENCE_ROOT>/phase-4-mapping/<run name>/, which
# scripts/evidence_root.sh resolves to the Windows checkout, so nothing here is written
# only into the WSL mirror (the rsync --delete of scripts/sync_to_wsl.sh cannot reach it).
set -u
set -m   # own process group per job; see the note in scripts/run_phase3_correctness.sh

NAME=${1:?usage: run_phase4_scan_skew.sh <run name>}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/phase3_guards.sh"
source "$SCRIPT_DIR/evidence_root.sh"
OUT="$EVIDENCE_ROOT/phase-4-mapping/$NAME"
LAUNCH="$REPO/sim/webots/launch/localization_world.launch.py"
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)

export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1
log() { echo "[scan-skew] $*" | tee -a "$OUT/run.log"; }

if pgrep -x webots-bin > /dev/null; then
    log "FAIL: a webots-bin process is already running, stop it first"
    exit 1
fi

{
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "kernel: $(uname -r)"
    echo "ros_distro: ${ROS_DISTRO:-unset}"
    echo "webots: $(webots --version 2>&1 | head -1)"
    echo "world_sha256: $(sha256sum "$REPO/sim/webots/worlds/turtlebot3_localization_room.wbt" | cut -d' ' -f1)"
    echo "recorder_sha256: $(sha256sum "$SCRIPT_DIR/phase4_scan_skew_record.py" | cut -d' ' -f1)"
    echo "analysis_sha256: $(sha256sum "$SCRIPT_DIR/phase4_scan_skew_analysis.py" | cut -d' ' -f1)"
    echo "mode: realtime"
} > "$OUT/environment.txt"

log "starting the world with ground truth"
ros2 launch "$LAUNCH" gui:=false mode:=realtime ground_truth:=true > "$OUT/launch.log" 2>&1 &
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

stop_launch() {
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 40); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -9 -- "-$LAUNCH_PID" 2>/dev/null || kill -9 "$LAUNCH_PID" 2>/dev/null
    fi
    pkill -x webots-bin 2>/dev/null
    echo "webots_bin_left: $(pgrep -c -x webots-bin || true)" >> "$OUT/shutdown.txt"
}

wait_for 180 "/scan" has_topic /scan || { stop_launch; exit 1; }
wait_for 60 "/ground_truth/pose" has_topic /ground_truth/pose || { stop_launch; exit 1; }
wait_for 90 "both controllers to become active" controllers_active || { stop_launch; exit 1; }
ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
timeout 10 ros2 topic echo --once /scan --field header > "$OUT/scan_header_sample.yaml" 2>&1
timeout 10 ros2 topic hz /scan --window 20 > "$OUT/scan_hz.txt" 2>&1 &
HZ_PID=$!
log "recording"
python3 "$SCRIPT_DIR/phase4_scan_skew_record.py" "$OUT" > "$OUT/record.log" 2>&1
log "recorder exited with code $?"
wait "$HZ_PID" 2>/dev/null
stop_launch

log "analysing"
python3 -I "$SCRIPT_DIR/phase4_scan_skew_analysis.py" "$OUT" > "$OUT/analysis.log" 2>&1
log "analysis exited with code $?; report in $OUT/skew-report.yaml"
