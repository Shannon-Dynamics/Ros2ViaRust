#!/usr/bin/env bash
# One smoke run of the Phase 3 localization world, recorded as evidence.
#
#   bash sim/webots/scripts/record_smoke_run.sh <output directory> [record seconds]
#
# What it does, in order: start the world with a GUI and the movie recorder, wait for the
# topics, drive the robot over /cmd_vel while the recorder is running, capture the rate and
# one sample message of every topic, then stop the launch with SIGINT and record the exit
# code it actually returned. Everything lands in the output directory as files; nothing is
# summarized into prose here, and nothing is retyped by hand afterwards.
#
# It writes no numbers of its own: every file is the untouched output of the command above
# it, so a claim in the documentation can be checked against the file it came from.

set -u

# Job control, so that the SIGINT this script sends at the end is a real one. A
# non-interactive shell starts a background job with SIGINT and SIGQUIT set to SIG_IGN, so
# that Ctrl-C on the terminal cannot reach it, and CPython does not install its own handler
# over an inherited SIG_IGN. Without `set -m` the `kill -INT` below is therefore a no-op:
# the launch keeps running, the 90 s wait always expires, and the run records a shutdown
# failure that belongs to this script rather than to the launch. Measured as
# `SigIgn: ...1001006` on the live `ros2 launch` process, SIGINT bit set. With job control
# the job gets its own process group and the default disposition.
set -m

OUT=${1:?usage: record_smoke_run.sh <output directory> [record seconds]}
REC_SECONDS=${2:-40}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/../../.." && pwd)
LAUNCH="$REPO/sim/webots/launch/localization_world.launch.py"
WORLD="$REPO/sim/webots/worlds/turtlebot3_localization_room.wbt"
TOPICS="/scan /odom /imu /joint_states /clock"

mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
MOVIE="$OUT/phase3_smoke.mp4"
STATUS="$OUT/movie_status.json"
rm -f "$STATUS" "$STATUS.started"

log() { echo "[run] $*" | tee -a "$OUT/run.log"; }

if pgrep -x webots-bin > /dev/null; then
    log "FAIL: a webots-bin process is already running, stop it first"
    exit 1
fi

# Environment of this run, so the numbers can be read in context later.
{
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)"
    echo "kernel: $(uname -r)"
    echo "os: $(. /etc/os-release && echo "$PRETTY_NAME")"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
    echo "ros_distro: ${ROS_DISTRO:-unset}"
    echo "rmw: ${RMW_IMPLEMENTATION:-default}"
    echo "webots: $(webots --version 2>&1 | head -1)"
    echo "webots_home: ${WEBOTS_HOME:-unset}"
    echo "display: ${DISPLAY:-unset} wayland: ${WAYLAND_DISPLAY:-unset}"
    echo "world: $WORLD"
    echo "world_sha256: $(sha256sum "$WORLD" | cut -d' ' -f1)"
    echo "record_seconds_requested: $REC_SECONDS"
} > "$OUT/environment.txt"

log "starting the launch, gui:=true mode:=realtime record:=true"
ros2 launch "$LAUNCH" \
    gui:=true mode:=realtime \
    record:=true "record_file:=$MOVIE" "record_seconds:=$REC_SECONDS" \
    "record_status_file:=$STATUS" \
    > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
log "launch pid $LAUNCH_PID"

wait_for() {  # wait_for <seconds> <description> <command...>
    local limit=$1 what=$2; shift 2
    for _ in $(seq 1 "$limit"); do
        if "$@" > /dev/null 2>&1; then return 0; fi
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then log "FAIL: launch died while waiting for $what"; return 1; fi
        sleep 1
    done
    log "FAIL: timed out after ${limit}s waiting for $what"
    return 1
}

scan_is_up() { ros2 topic list 2>/dev/null | grep -qx /scan; }
recording_started() { test -f "$STATUS.started"; }

wait_for 150 "/scan to appear" scan_is_up || { log "aborting"; kill -INT "$LAUNCH_PID"; exit 1; }
log "topics are up"

# The topics publish whether or not ros2_control came up, so their presence proves nothing
# about /cmd_vel. If the controllers are not active the robot cannot move, and the movie and
# the /odom sample below would record a robot parked at the origin while every file still
# looks like a successful run. See the comment on the placeholder description in
# ../launch/localization_world.launch.py for how that happens.
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
            "$OUT/launch.log" 2>/dev/null | head -20
    } > "$OUT/controllers.txt" 2>&1
    log "FAIL: the controllers never activated, the robot could not have moved; aborting"
    log "see controllers.txt"
    kill -INT "$LAUNCH_PID"
    exit 1
fi
ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
log "controllers are active"

wait_for 120 "the recorder to start" recording_started || log "continuing without a confirmed recording start"

# Drive the robot so that the movie shows motion instead of a parked robot. Jazzy's
# diff_drive_controller takes TwistStamped.
drive() {  # drive <seconds> <linear x> <angular z>
    log "cmd_vel for $1 s: v=$2 w=$3"
    timeout "$1" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
        "{twist: {linear: {x: $2}, angular: {z: $3}}}" > /dev/null 2>&1
}
drive 8 0.15 0.0
drive 6 0.0 0.6
drive 8 0.15 0.3
drive 6 0.0 -0.6
drive 8 0.12 0.0
log "driving finished"

log "capturing graph, rates and one sample message per topic"
ros2 node list     > "$OUT/node_list.txt"     2>&1
ros2 topic list -t > "$OUT/topic_list.txt"    2>&1
: > "$OUT/topic_hz.txt"
for t in $TOPICS; do
    echo "\$ ros2 topic hz -w 20 $t" >> "$OUT/topic_hz.txt"
    timeout 20 ros2 topic hz -w 20 "$t" 2>&1 | head -2 >> "$OUT/topic_hz.txt"
    echo >> "$OUT/topic_hz.txt"
done
for t in $TOPICS; do
    f="$OUT/echo_$(echo "${t#/}" | tr '/' '_').yaml"
    timeout 20 ros2 topic echo --once --full-length "$t" > "$f" 2>&1
done
timeout 20 ros2 topic echo --once --full-length /tf        > "$OUT/echo_tf.yaml"        2>&1
timeout 20 ros2 topic echo --once --full-length /tf_static > "$OUT/echo_tf_static.yaml" 2>&1
timeout 20 ros2 run tf2_ros tf2_echo odom base_link        > "$OUT/tf2_echo_odom_base_link.txt" 2>&1

log "waiting for the recorder to finish encoding"
wait_for 180 "the movie status file" test -f "$STATUS" || log "no movie status file was written"

log "stopping the launch with SIGINT"
SHUTDOWN_START=$(date +%s.%N)
kill -INT "$LAUNCH_PID"
for _ in $(seq 1 90); do
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    sleep 1
done
{
    echo "signal: SIGINT to the ros2 launch process (pid $LAUNCH_PID)"
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "result: still alive 90 s after SIGINT, exit code unknown"
        echo "exit_code: none"
    else
        wait "$LAUNCH_PID"
        echo "exit_code: $?"
        echo "seconds_to_exit: $(echo "$(date +%s.%N) - $SHUTDOWN_START" | bc)"
    fi
    echo "webots_bin_processes_left: $(pgrep -c -x webots-bin || true)"
    echo "ros2_launch_processes_left: $(pgrep -cf 'ros2 launch .*localization_world' || true)"
} > "$OUT/shutdown.txt"
log "shutdown recorded:"
cat "$OUT/shutdown.txt" | tee -a "$OUT/run.log"

if [ -f "$MOVIE" ]; then
    log "movie: $MOVIE ($(stat -c %s "$MOVIE") bytes)"
    ffprobe -hide_banner "$MOVIE" > "$OUT/movie_ffprobe.txt" 2>&1 || true
else
    log "movie: NOT created"
fi
log "done, evidence in $OUT"
