#!/usr/bin/env bash
# Phase 5 integration test of scan_matcher_node and pose_graph_node, by REPLAY ONLY.
#
#   bash scripts/verify_phase5_slam_nodes.sh [output directory]
#
# No simulator is started, ever: the scans come from recorded files through `scan_replayer`
# (docs/phase-5-slam/implementation.md section 8.6). Follows the shape of
# scripts/verify_phase4_mapping.sh: every check prints PASS or FAIL with a name, the counts are
# summarised at the end, and the exit code is non-zero if anything failed.
#
#   A  the pieces exist and are installed                                   (no ROS graph)
#   B  loop closure is OFF unless a parameter turns it on: the default is false (source and
#      `ros2 param`); this replaced the earlier "cannot be switched on" checks when the node gained
#      loop closure (docs/phase-5-slam/implementation.md 8.7)
#   C  each recorded stream replayed through the LAUNCH FILE with the map and map -> odom OFF, nodes'
#      files compared with the in-process reference of the same stream and graded against the file's
#      ground truth (scripts/phase5_node_replay_check.py: R1 R2 K1 K2 G E1). This is the regression
#      that the node with its new features switched off is the node that was verified before.
#   D  on a live graph (the same configuration): the topics and their types, no /tf, no /map, and
#      `loop_closure: unsupported` in /diagnostics
#   E  a second replay of the same stream gives the same files
#   G  loop closure, /map and map -> odom ON, on a synthetic route and on the Webots recordings with
#      turning scans (scripts/phase5_stage_b_check.py): /map once per closure and once at the end, the
#      map equal to a render from files alone, the path republished after a closure, map -> odom
#      equal to the estimate recomputed from the files, no second owner of the edge
#   F  shutdown leaves nothing behind
#
# It is built to leave a machine that is busy with something else alone, because it was written to
# run while the Phase 3 kidnap benchmark owned the CPU: everything runs niced, pinned to two cores
# (PIN, default 14,15), in its own ROS domain (default 91; it REFUSES domain 42, which the benchmark
# uses), and builds happen in a private workspace (WS, default ~/slam_nodes_ws), never in the mirror
# the benchmark runs from.
#
# Output goes to <EVIDENCE_ROOT>/phase-5-slam-nodes/<date> by default (scripts/evidence_root.sh).

set -u
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/evidence_root.sh"
WS=${WS:-$HOME/slam_nodes_ws}
PIN=${PIN:-14,15}
RATE=${RATE:-200}           # upper bound in lockstep, where each scan waits for the previous registration
REALTIME_RATE=${REALTIME_RATE:-5}
# Section G's replay rate. Lockstep makes the replayer wait for the scan matcher's registration only; the pose graph node, which
# now closes loops and renders a map per keyframe, is a second stage it does not wait for, and at the 200 Hz of sections C and E
# it falls behind and is shut down with a backlog (found by this script's first run of section G). 20 Hz is four times real time.
G_RATE=${G_RATE:-20}
export ROS_DOMAIN_ID=${PHASE5_ROS_DOMAIN_ID:-91}
if [ "$ROS_DOMAIN_ID" = 42 ]; then
    echo "refusing to run in ROS domain 42: the kidnap benchmark's Webots stack lives there" >&2
    exit 3
fi
OUT=${1:-$EVIDENCE_ROOT/phase-5-slam-nodes/$(date +%Y-%m-%d)}
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
LOG="$OUT/verify.log"
: > "$LOG"
export RCUTILS_LOGGING_BUFFERED_STREAM=0 PYTHONUNBUFFERED=1
exec > >(tee -a "$LOG") 2>&1

set +u
source "$WS/scripts/env.sh" > /dev/null 2>&1
set -u
RUN="nice -n 19 taskset -c $PIN"
# The release install, like Phase 3's scripts/build_release.sh: a front end that has lost track costs
# seconds per step in a debug build, and a replay test must not measure the build profile.
if [ -d "$WS/ros2_ws/install_release/robotics_slam_ros" ]; then INSTALL="$WS/ros2_ws/install_release"; else INSTALL="$WS/ros2_ws/install"; fi
set +u; source "$INSTALL/setup.bash" > /dev/null 2>&1; set -u
BIN="$INSTALL/robotics_slam_ros/lib/robotics_slam_ros"
REFBIN="${CARGO_TARGET_DIR:-$WS/target}/release/examples/slam_replay_reference"
CHECK="$SCRIPT_DIR/phase5_node_replay_check.py"
P5="$REPO/evidence/phase-5-slam"
P4="$REPO/evidence/phase-4-mapping"

PASS=0; FAIL=0; FAILED_NAMES=()
ok()  { PASS=$((PASS + 1)); printf 'PASS  %s\n' "$1"; }
bad() { FAIL=$((FAIL + 1)); FAILED_NAMES+=("$1"); printf 'FAIL  %s\n' "$1"; }
check()  { local n=$1; shift; if "$@" > /dev/null 2>&1; then ok "$n"; else bad "$n"; fi; }
refute() { local n=$1; shift; if "$@" > /dev/null 2>&1; then bad "$n"; else ok "$n"; fi; }

echo "== environment: domain $ROS_DOMAIN_ID, cores $PIN, install space $INSTALL, load $(cut -d' ' -f1-3 /proc/loadavg), $(date -u +%FT%TZ)"
echo "== workspace $WS, repo $REPO, commit $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null)"
echo "== webots running now: $(pgrep -c -f webots-bin || true) process(es); this script starts none"

echo "== A: the pieces exist and are installed"
for exe in scan_matcher_node pose_graph_node scan_replayer; do
    check "A installed executable $exe" test -x "$BIN/$exe"
done
# The launch file comes from robotics_bringup, which may live in an underlay of $INSTALL.
check "A launch file installed" bash -c 'test -f "$(ros2 pkg prefix robotics_bringup)/share/robotics_bringup/launch/phase5_slam.launch.py"'
check "A the installed launch file knows lockstep (not a stale install)" bash -c 'grep -q "lockstep" "$(ros2 pkg prefix robotics_bringup)/share/robotics_bringup/launch/phase5_slam.launch.py"'
check "A package.xml installed" test -f "$INSTALL/robotics_slam_ros/share/robotics_slam_ros/package.xml"
check "A ros2 sees the package's executables" bash -c "ros2 pkg executables robotics_slam_ros | grep -q pose_graph_node"
check "A in-process reference built" test -x "$REFBIN"

echo "== B: loop closure is off unless a parameter turns it on"
SRC="$REPO/ros2_ws/src/robotics_slam_ros/src"
check "B the switch is a parameter whose default is false" bash -c "grep -A1 '\"loop_closure.enabled\"' '$SRC/pose_graph_node.rs' | grep -q 'false'"
check "B the builder's configuration defaults to loop closure off" grep -qE 'loop_closure: false' "$SRC/lib.rs"
check "B the status of a run that did not ask is still the word unsupported" grep -q 'LOOP_CLOSURE_STATUS: &str = "unsupported"' "$SRC/lib.rs"
check "B the launch file's loop_closure argument defaults to false" grep -qE "'loop_closure': \('false'" "$REPO/ros2_ws/src/robotics_bringup/launch/phase5_slam.launch.py"
check "B nothing in the scan matcher or the replayer calls the loop detector" bash -c "! grep -v '^[[:space:]]*//' '$SRC/scan_matcher_node.rs' '$SRC/scan_replayer.rs' | grep -qE 'propose_loops|process_loops|add_loop|Verification'"

# The configuration of sections C, D, E and R: the node as it was verified before it could close loops, render a map
# or own map -> odom. Section G overrides these.
BASE_ARGS=(publish_map:=false publish_map_odom:=false)
run_stream() {  # run_stream <name> <scans csv> <metrics yaml or -> <replay rate> <lockstep true|false> <extra launch args...>
    local name=$1 csv=$2 metrics=$3 rate=$4 lock=$5; shift 5
    local dir="$OUT/$name"
    rm -rf "$dir"; mkdir -p "$dir/node" "$dir/ref" "$dir/ref64"
    local n_scans; n_scans=$(grep -vc '^#' "$csv")
    $RUN ros2 launch robotics_bringup phase5_slam.launch.py \
        replay_file:="$csv" rate_hz:="$rate" lockstep:="$lock" capture:=true output_dir:="$dir/node" "${BASE_ARGS[@]}" "$@" \
        > "$dir/launch.log" 2>&1 &
    LAUNCH_PID=$!
}

wait_launch() {  # wait_launch <name>  : waits for the launch to end by itself, with a ceiling
    local dir="$OUT/$1" waited=0
    while kill -0 "$LAUNCH_PID" 2>/dev/null; do
        sleep 2; waited=$((waited + 2))
        if [ "$waited" -gt "${CEILING:-900}" ]; then bad "C $1 launch did not end by itself within ${CEILING:-900} s"; kill -INT "$LAUNCH_PID"; sleep 5; kill -KILL "$LAUNCH_PID" 2>/dev/null; return 1; fi
    done
    wait "$LAUNCH_PID"; LAUNCH_RC=$?
    return 0
}

compare_stream() {  # compare_stream <name> <csv> <metrics or ->
    local name=$1 csv=$2 metrics=$3 dir="$OUT/$1"
    $RUN "$REFBIN" --scans "$csv" --out "$dir/ref" > "$dir/ref.log" 2>&1
    local extra=()
    if [ "$metrics" != "-" ]; then
        $RUN "$REFBIN" --scans "$csv" --out "$dir/ref64" --f64-metadata > "$dir/ref64.log" 2>&1
        extra=(--metrics "$metrics" --f64-reference "$dir/ref64")
    fi
    python3 -I "$CHECK" --scans "$csv" --node "$dir/node" --reference "$dir/ref" "${extra[@]}" --out "$dir/check.yaml" > "$dir/check.console" 2>&1
    local rc=$?
    sed 's/^/    /' "$dir/check.console"
    if [ "$rc" -eq 0 ]; then ok "C $name: nodes equal the in-process reference and are graded ($(grep -c '^PASS' "$dir/check.console") checks)"; else bad "C $name: replay check (see $dir/check.console)"; fi
}

if [ "${SKIP_OLD:-0}" != 1 ]; then   # SKIP_OLD=1 runs only section G (to iterate on it); a verification that is to be believed runs everything
echo "== C and D: replay through the launch file"
SYN=(2-eval-cross 3-eval-diamond 4-eval-rectangle 1-dev-route-long)
REC=("1-dev-route-long:development-3-route-long" "2-eval-cross:evaluation-1-cross" "3-eval-wall-lobes:evaluation-2-wall-lobes" "4-eval-diamond:evaluation-3-diamond-slow")
for s in "${SYN[@]}"; do
    name="replay-synthetic-$s"; csv="$P5/v2-synthetic-$s/scans.csv"
    if [ ! -f "$csv" ]; then bad "C $name: input $csv missing"; continue; fi
    echo "-- $name ($(grep -vc '^#' "$csv") scans, lockstep)"
    run_stream "$name" "$csv" "$P5/v2-synthetic-$s/slam-metrics.yaml" "$RATE" true
    wait_launch "$name" || continue
    check "C $name launch exited 0" test "${LAUNCH_RC:-1}" -eq 0
    check "C $name replayer published every scan" grep -q "replay finished: $(grep -vc '^#' "$csv") scans published" "$OUT/$name/launch.log"
    check "C $name replayer really ran in lockstep" grep -q "lockstep true" "$OUT/$name/launch.log"
    compare_stream "$name" "$csv" "$P5/v2-synthetic-$s/slam-metrics.yaml"
done
for r in "${REC[@]}"; do
    s=${r%%:*}; d=${r##*:}
    name="replay-recorded-$s"; csv="$P4/2026-10-11-$d/integrated-scans.csv"
    if [ ! -f "$csv" ]; then bad "C $name: input $csv missing"; continue; fi
    echo "-- $name ($(grep -vc '^#' "$csv") scans, lockstep; the file has no turning scans, so the front end is expected to lose track at the gaps)"
    run_stream "$name" "$csv" - "$RATE" true
    wait_launch "$name" || continue
    check "C $name launch exited 0" test "${LAUNCH_RC:-1}" -eq 0
    check "C $name replayer published every scan" grep -q "replay finished: $(grep -vc '^#' "$csv") scans published" "$OUT/$name/launch.log"
    check "C $name replayer really ran in lockstep" grep -q "lockstep true" "$OUT/$name/launch.log"
    compare_stream "$name" "$csv" -
    echo "    status counts: $(python3 -I -c "import yaml;print(yaml.safe_load(open('$OUT/$name/check.yaml'))['facts'].get('status_counts'))" 2>/dev/null)"
done

echo "== R: at the real-time rate ($REALTIME_RATE Hz, no lockstep: as lossy as a real driver)"
name="realtime-synthetic-2-eval-cross"; csv="$P5/v2-synthetic-2-eval-cross/scans.csv"
run_stream "$name" "$csv" "$P5/v2-synthetic-2-eval-cross/slam-metrics.yaml" "$REALTIME_RATE" false
# D: look at the live graph during the real-time run, which lasts long enough to be looked at.
sleep 8
ros2 topic list -t > "$OUT/$name/topics.txt" 2>&1
ros2 node list > "$OUT/$name/nodes.txt" 2>&1
timeout 6 ros2 topic echo /diagnostics > "$OUT/$name/diagnostics-sample.txt" 2>&1
ros2 node info /pose_graph_node > "$OUT/$name/pose_graph_node-info.txt" 2>&1
ros2 node info /scan_matcher_node > "$OUT/$name/scan_matcher_node-info.txt" 2>&1
ros2 param list /pose_graph_node > "$OUT/$name/pose_graph_node-params.txt" 2>&1
check "D topic registration is PoseWithCovarianceStamped" grep -q "^/registration \[geometry_msgs/msg/PoseWithCovarianceStamped\]" "$OUT/$name/topics.txt"
check "D topic scan_matched is LaserScan" grep -q "^/scan_matched \[sensor_msgs/msg/LaserScan\]" "$OUT/$name/topics.txt"
check "D topic slam/pose is PoseWithCovarianceStamped" grep -q "^/slam/pose \[geometry_msgs/msg/PoseWithCovarianceStamped\]" "$OUT/$name/topics.txt"
check "D topic slam/path is nav_msgs/Path" grep -q "^/slam/path \[nav_msgs/msg/Path\]" "$OUT/$name/topics.txt"
refute "D no /tf topic exists" grep -q "^/tf " "$OUT/$name/topics.txt"
refute "D no /tf_static topic exists" grep -q "^/tf_static " "$OUT/$name/topics.txt"
refute "D no /map topic exists" grep -qE "^/map |^/esdf " "$OUT/$name/topics.txt"
refute "D pose_graph_node publishes nothing named tf or map" grep -qE "/tf|/map" <(sed -n '/Publishers:/,/Service Servers:/p' "$OUT/$name/pose_graph_node-info.txt")
refute "D scan_matcher_node publishes nothing named tf or map" grep -qE "/tf|/map" <(sed -n '/Publishers:/,/Service Servers:/p' "$OUT/$name/scan_matcher_node-info.txt")
check "D pose_graph_node diagnostics say loop_closure: unsupported" bash -c "grep -A1 'key: loop_closure\$' '$OUT/$name/diagnostics-sample.txt' | grep -q 'value: unsupported'"
check "D diagnostics say no map and no map->odom is published" bash -c "grep -A1 'key: map_odom_published' '$OUT/$name/diagnostics-sample.txt' | grep -q 'value: none'"
check "D loop_closure.enabled is a parameter of pose_graph_node and its value is false" bash -c "grep -q 'loop_closure.enabled' '$OUT/$name/pose_graph_node-params.txt' && ros2 param get /pose_graph_node loop_closure.enabled 2>/dev/null | grep -qi false" 
check "D the scan matcher takes no odometry input" bash -c "grep -A1 'key: inputs' '$OUT/$name/diagnostics-sample.txt' | grep -q 'scan only'"
wait_launch "$name"
nreg=$(($(wc -l < "$OUT/$name/node/registrations.csv") - 1)); nscans=$(grep -vc '^#' "$csv")
check "R cross at $REALTIME_RATE Hz: every scan was registered ($nreg of $nscans)" test "$nreg" -eq "$nscans"
python3 -I - "$OUT/$name/node/registrations.csv" <<'PYEOF'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
lat = sorted(float(r["latency_us"]) for r in rows if r.get("latency_us"))
print(f"    INFO  registration latency at real time, us: median {lat[len(lat)//2]:.0f}, p95 {lat[int(.95*len(lat))]:.0f}, p99 {lat[int(.99*len(lat))]:.0f}, max {lat[-1]:.0f} (the scan period is 200000)")
PYEOF
name="realtime-recorded-4-eval-diamond"; csv="$P4/2026-10-11-evaluation-3-diamond-slow/integrated-scans.csv"
run_stream "$name" "$csv" - "$REALTIME_RATE" false
wait_launch "$name"
nreg=$(($(wc -l < "$OUT/$name/node/registrations.csv") - 1)); nscans=$(grep -vc '^#' "$csv")
echo "    INFO  recorded diamond at $REALTIME_RATE Hz without lockstep: $nreg of $nscans scans registered (the file has no turning scans: the front end loses track at its gaps, and a lost step costs far more than the scan period)"

echo "== E: a second replay of the same stream gives the same files"
name="replay-synthetic-2-eval-cross-again"; csv="$P5/v2-synthetic-2-eval-cross/scans.csv"
run_stream "$name" "$csv" - "$RATE" true
wait_launch "$name"
first="$OUT/replay-synthetic-2-eval-cross/node"; second="$OUT/$name/node"
check "E keyframes.csv is identical on a second replay" cmp -s "$first/keyframes.csv" "$second/keyframes.csv"
check "E registrations.csv is identical on a second replay, latency column aside" cmp -s <(cut -d, -f1-11 "$first/registrations.csv") <(cut -d, -f1-11 "$second/registrations.csv")

fi   # SKIP_OLD

echo "== G: loop closure, /map and map -> odom ON, replayed through the launch file (lockstep, at $G_RATE Hz at most)"
RENDER="${CARGO_TARGET_DIR:-$WS/target}/release/examples/render_check"
PROBE="$SCRIPT_DIR/phase5_map_tf_probe.py"
STAGEB="$SCRIPT_DIR/phase5_stage_b_check.py"
check "G the render_check example is built" test -x "$RENDER"
run_g() {  # run_g <name> <scans csv> <loop on|off> <beam layout> : one replay with the map and map -> odom owned, probed, checked
    local name=$1 csv=$2 loop=$3 layout=$4 dir="$OUT/$1"
    if [ ! -f "$csv" ]; then bad "G $name: input $csv missing"; return; fi
    rm -rf "$dir"; mkdir -p "$dir/node"
    echo "-- $name ($(grep -vc '^#' "$csv") scans, loop closure $loop)"
    python3 "$PROBE" "$dir/probe.json" > "$dir/probe.log" 2>&1 &
    PROBE_PID=$!
    sleep 3
    local lc=false; [ "$loop" = on ] && lc=true
    $RUN ros2 launch robotics_bringup phase5_slam.launch.py \
        replay_file:="$csv" rate_hz:="$G_RATE" lockstep:=true capture:=true output_dir:="$dir/node" \
        loop_closure:=$lc publish_map:=true publish_map_odom:=true odom_tf:=auto map_beam_layout:=$layout > "$dir/launch.log" 2>&1 &
    LAUNCH_PID=$!
    wait_launch "$name" || { kill -INT "$PROBE_PID" 2>/dev/null; return; }
    sleep 1
    kill -INT "$PROBE_PID" 2>/dev/null; wait "$PROBE_PID" 2>/dev/null
    check "G $name launch exited 0" test "${LAUNCH_RC:-1}" -eq 0
    check "G $name the nodes wrote the files of a run with a map" bash -c "test -s '$dir/node/final-map.txt' && test -s '$dir/node/keyframes_final.csv' && test -f '$dir/node/loops.csv'"
    $RUN "$RENDER" --scans "$csv" --keyframes "$dir/node/keyframes_final.csv" --out "$dir/render-final.txt" --layout "$layout" > "$dir/render-final.log" 2>&1
    $RUN "$RENDER" --scans "$csv" --keyframes "$dir/node/keyframes.csv" --out "$dir/render-created.txt" --layout "$layout" > "$dir/render-created.log" 2>&1
    check "G $name every keyframe's scan was found in the file for the render" bash -c "grep -q ' 0 without a scan' '$dir/render-final.log'"
    python3 -I "$STAGEB" --dir "$dir" --loop "$loop" > "$dir/stage-b-check.console" 2>&1
    local rc=$?
    sed 's/^/    /' "$dir/stage-b-check.console"
    if [ "$rc" -eq 0 ]; then ok "G $name: $(grep -c '^PASS' "$dir/stage-b-check.console") checks on /map, map -> odom, the path and loop closure"; else bad "G $name: see $dir/stage-b-check.console"; fi
}
# Rusty's lidar publishes -pi, +2pi/360, which the Webots pixel_centres layout refuses (it is Webots' own); a synthetic stream is rendered as published.
run_g "replay-g-synthetic-4-eval-rectangle-loop-on" "$P5/v2-synthetic-4-eval-rectangle/scans.csv" on published
run_g "replay-g-synthetic-4-eval-rectangle-loop-off" "$P5/v2-synthetic-4-eval-rectangle/scans.csv" off published
for rec in "2026-10-14-webots-rectangle-recording" "2026-10-14-webots-rectangle-recording-2"; do
    run_g "replay-g-webots-${rec#2026-10-14-webots-rectangle-}-loop-on" "$P5/$rec/integrated-scans.csv" on pixel_centres
done
# The map and the TF can be switched off together, and then the node is the one that was verified before.
name="replay-g-switches-off"; csv="$P5/v2-synthetic-4-eval-rectangle/scans.csv"
rm -rf "$OUT/$name"; mkdir -p "$OUT/$name/node"
python3 "$PROBE" "$OUT/$name/probe.json" > "$OUT/$name/probe.log" 2>&1 &
PROBE_PID=$!; sleep 3
$RUN ros2 launch robotics_bringup phase5_slam.launch.py replay_file:="$csv" rate_hz:="$G_RATE" lockstep:=true capture:=true output_dir:="$OUT/$name/node" \
    publish_map:=false publish_map_odom:=false map_beam_layout:=published > "$OUT/$name/launch.log" 2>&1 &
LAUNCH_PID=$!
wait_launch "$name"; sleep 1; kill -INT "$PROBE_PID" 2>/dev/null; wait "$PROBE_PID" 2>/dev/null
$RUN "$RENDER" --scans "$csv" --keyframes "$OUT/$name/node/keyframes_final.csv" --out "$OUT/$name/render-final.txt" --layout published > "$OUT/$name/render-final.log" 2>&1
$RUN "$RENDER" --scans "$csv" --keyframes "$OUT/$name/node/keyframes.csv" --out "$OUT/$name/render-created.txt" --layout published > /dev/null 2>&1
python3 -I "$STAGEB" --dir "$OUT/$name" --loop off --tf off --map off > "$OUT/$name/stage-b-check.console" 2>&1
rc=$?; sed 's/^/    /' "$OUT/$name/stage-b-check.console"
if [ "$rc" -eq 0 ]; then ok "G $name: with the map and map -> odom off nothing of them is published"; else bad "G $name: see $OUT/$name/stage-b-check.console"; fi

echo "== F: shutdown leaves nothing behind"
sleep 2
refute "F no SLAM node or replayer process is left" pgrep -f "robotics_slam_ros/(scan_matcher_node|pose_graph_node|scan_replayer)"
refute "F no node needed escalating from SIGINT in any replay" grep -l "failed to terminate" "$OUT"/*/launch.log
check "F the launch logs of every replay show both nodes shutting down" bash -c "for f in '$OUT'/replay-*/launch.log; do grep -q 'pose_graph_node shutting down' \"\$f\" && grep -q 'scan_matcher_node shutting down' \"\$f\" || exit 1; done"

echo
echo "passed: $PASS   failed: $FAIL"
[ "$FAIL" -eq 0 ] || { printf 'failed: %s\n' "${FAILED_NAMES[@]}"; exit 1; }
exit 0
