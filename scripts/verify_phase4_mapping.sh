#!/usr/bin/env bash
# Phase 4 integration test: brings the mapping stack up through the bringup launch and checks
# it one assertion at a time, with the standard ROS 2 CLI. Follows the shape of
# scripts/verify_phase3_localization.sh: every check prints PASS or FAIL with a name, the
# counts are summarised at the end, and the exit code is non-zero if anything failed.
#
#   bash scripts/verify_phase4_mapping.sh [output directory]
#
# What it covers:
#   A  the pieces exist and are installed                        (no simulator)
#   B  the verdict script refuses and grades correctly, on hand-made fixtures (no simulator)
#   C  the stack comes up through the bringup launch
#   D  every expected topic exists with the right type
#   E  the TF chain is complete and `map -> odom` has one owner, the adapter
#   F  provenance: the mapper reads no ground truth, the adapter does
#   G  the mapper maps: scans integrated, occupied cells appear, the ESDF has its contract
#   H  the stack shuts down and leaves nothing behind
#
# B is the part worth explaining. The fixtures in $OUT/fixtures are written by this script
# to exercise the verdict script's refusals (an M5 run, an invalid run, a tolerance mismatch)
# and its pass/fail logic; they are not evidence of anything and sit in the output directory
# only so that a failure can be read. Evidence is the runs of run_phase4_mapping.sh.
#
# Output goes to <EVIDENCE_ROOT>/phase-4-mapping-runtime/<date> by default, never to /tmp
# (WSL clears /tmp), through scripts/evidence_root.sh, which is the Windows checkout.

set -u
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/phase3_guards.sh"
source "$SCRIPT_DIR/evidence_root.sh"
OUT=${1:-$EVIDENCE_ROOT/phase-4-mapping-runtime/$(date +%Y-%m-%d)}
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
LOG="$OUT/verify.log"
LAUNCH_LOG="$OUT/launch.log"
: > "$LOG"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1
exec > >(tee -a "$LOG") 2>&1

ROUTE="$REPO/sim/webots/config/route_smoke.yaml"
MAP="$REPO/sim/webots/config/landmarks.yaml"
VERDICT="$SCRIPT_DIR/phase4_verdict.py"
INSTALL="$REPO/ros2_ws/install"
BIN="$INSTALL/robotics_mapping_ros/lib/robotics_mapping_ros"

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
# Within the few lines after a block start: /diagnostics interleaves entries from several
# nodes, so a bare grep for "message: RUNNING" would pass on any of them.
check_block_match() {  # check_block_match <name> <block_start_regex> <contains_regex> <file> [lines_after]
    local name=$1 start=$2 contains=$3 file=$4 n=${5:-1}
    if [ -s "$file" ] && grep -A"$n" -E "$start" "$file" | grep -qE "$contains"; then ok "$name"; else bad "$name"; fi
}
section() { printf '\n===== %s =====\n' "$1"; }

printf '=== %s  ROS_DISTRO=%s  RMW=%s ===\n' "$(date -Is)" "${ROS_DISTRO:-unset}" "${RMW_IMPLEMENTATION:-default}"
printf 'repository: %s\noutput:     %s\n' "$REPO" "$OUT"

# ----------------------------------------------------------------- A: the pieces exist
section "A: the pieces exist and are installed"
check "A1  the colcon install space exists" test -d "$INSTALL"
for exe in ground_truth_tf_adapter mapping_node mapping_evaluator; do
    check "A   $exe is installed and executable" test -x "$BIN/$exe"
done
check "A5  the bringup launch is installed" \
    test -f "$INSTALL/robotics_bringup/share/robotics_bringup/launch/phase4_mapping.launch.py"
check "A6  the world launch exists" test -f "$REPO/sim/webots/launch/localization_world.launch.py"
check "A7  the landmark map is readable" test -r "$MAP"
check "A8  the smoke route is readable" test -r "$ROUTE"
check "A9  the verdict script runs and prints its tolerance" \
    bash -c "python3 -I '$VERDICT' --print-tau-cells | grep -qx '1.0'"

# ----------------------------------------------------------------- B: the verdict script
section "B: the verdict script refuses and grades correctly (hand-made fixtures)"
FIX="$OUT/fixtures"
rm -rf "$FIX"; mkdir -p "$FIX"
make_fixture() {  # make_fixture <name> <owner> <subscribes_gt> <tau_cells> <p95_cells> <max_cells> <skipped_motion>
    local d="$FIX/$1"; mkdir -p "$d"
    cat > "$d/summary.yaml" <<EOF
role: evaluation
owner_of_map_odom: $2
mapping_node_subscribes_ground_truth: $3
route_driver_exit_code: 0
replay_scans: 100
launch_arguments: fixture
source_git_commit: fixture
mapping_node_executable_sha256: fixture
EOF
    cat > "$d/map-metrics.yaml" <<EOF
maps_received: 10
positions_kept: 50
resolution_m: 0.05
tau_m: 0.05
tau_cells: $4
occupied_cells: 500
free_cells: 9000
unknown_cells: 1000
surface_error_mean_m: 0.02
surface_error_p95_m: 0.025
surface_error_max_m: 0.05
surface_error_mean_cells: 0.4
surface_error_p95_cells: $5
surface_error_max_cells: $6
false_occupied_cells: 1
false_occupied_fraction: 0.002
samples_total: 2780
samples_observable: 2500
observable_fraction: 0.9
completeness: 0.99
false_free_samples: 10
false_free_fraction: 0.004
samples_outside_grid: 0
reachable_cells: 9500
coverage: 0.99
EOF
    printf 'x,y\n0.0,0.0\n1.0,0.0\n2.0,0.5\n' > "$d/trajectory.csv"
    {
        echo "stamp_s,outcome,latency_us,queue_wait_us,beams,hits,no_return_cleared,invalid,cells_free,cells_occupied,window_translation_m,window_yaw_rad"
        for i in $(seq 1 100); do echo "$i.0,integrated,1500.0,2000.0,360,250,100,0,12000,250,0.01,0.001"; done
        for i in $(seq 1 "$7"); do echo "$i.5,skipped_motion,30.0,2000.0,360,,,,,,0.01,0.1"; done
    } > "$d/update-latency.csv"
    echo "x" > "$d/final-grid.txt"; echo "x" > "$d/replay-grid.txt"
}
verdict() { python3 -I "$VERDICT" "$FIX/$1" > "$FIX/$1/console.txt" 2>&1; }
make_fixture good ground_truth_tf_adapter false 1 0.5 1.2 20
make_fixture m5run localization_node false 1 0.5 1.2 20
make_fixture readsgt ground_truth_tf_adapter true 1 0.5 1.2 20
make_fixture wrongtau ground_truth_tf_adapter false 2 0.5 1.2 20
make_fixture inaccurate ground_truth_tf_adapter false 1 2.0 6.0 20
make_fixture skipsmost ground_truth_tf_adapter false 1 0.5 1.2 150
for f in good m5run readsgt wrongtau inaccurate skipsmost; do verdict $f; done
check_match "B1  a valid, accurate run with the adapter as owner passes M1" '^M1: PASS' "$FIX/good/phase4-verdict.yaml"
check_match "B2  a run whose map -> odom owner is an estimator is refused, not graded" '^M1: REFUSED' "$FIX/m5run/phase4-verdict.yaml"
check_match "B3  a mapper that read ground truth makes the run invalid" '^valid: false' "$FIX/readsgt/phase4-verdict.yaml"
check_match "B4  an M1 verdict is withheld from an invalid run" '^M1: NOT_GRADED' "$FIX/readsgt/phase4-verdict.yaml"
check_match "B5  metrics taken at another tolerance make the run invalid" 'tau_cells' "$FIX/wrongtau/phase4-verdict.yaml"
check_match "B6  an inaccurate map fails M1" '^M1: FAIL' "$FIX/inaccurate/phase4-verdict.yaml"
check_match "B7  a run that skipped most scans for motion is invalid" 'skipped for motion' "$FIX/skipsmost/phase4-verdict.yaml"
check_match "B8  the verdict records the hash of the script that wrote it" '^verdict_script_sha256: [0-9a-f]{64}' "$FIX/good/phase4-verdict.yaml"

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

echo "--- ros2 launch robotics_bringup phase4_mapping.launch.py (own process group) ---"
setsid ros2 launch robotics_bringup phase4_mapping.launch.py \
    gui:=false mode:=realtime owner:=ground_truth drive:=false evaluate:=false \
    > "$LAUNCH_LOG" 2>&1 &
LAUNCH=$!
cleanup() {
    local rc=$?
    echo "--- cleanup: the test is leaving before the shutdown section, stopping the launch ---"
    kill -INT -- -"$LAUNCH" 2>/dev/null
    sleep 5
    pkill -x webots-bin 2>/dev/null
    exit "$rc"
}
trap cleanup EXIT

check "C1  /scan appears within 180 s" wait_for 180 has_topic /scan
check "C2  /ground_truth/pose appears within 60 s" wait_for 60 has_topic /ground_truth/pose
check "C3  both controllers become active within 90 s" wait_for 90 controllers_active
check "C4  the ground_truth_tf_adapter node is up" wait_for 30 has_node /ground_truth_tf_adapter
check "C5  the mapping_node node is up" wait_for 30 has_node /mapping_node
node_absent() { ! has_node "$1"; }
check "C6  the evaluator is not part of this launch" node_absent /mapping_evaluator
sleep 10   # let the adapter publish and the mapper integrate a few scans

# ----------------------------------------------------------------- D: topics
section "D: every expected topic exists with the right type"
ros2 topic list -t > "$OUT/topic_list.txt" 2>&1
check_match "D1  /map is a nav_msgs/msg/OccupancyGrid" '^/map \[nav_msgs/msg/OccupancyGrid\]' "$OUT/topic_list.txt"
check_match "D2  /esdf is a sensor_msgs/msg/Image" '^/esdf \[sensor_msgs/msg/Image\]' "$OUT/topic_list.txt"
check_match "D3  /map_entropy is a std_msgs/msg/Float64" '^/map_entropy \[std_msgs/msg/Float64\]' "$OUT/topic_list.txt"
check_match "D4  /ground_truth/pose is a geometry_msgs/msg/PoseStamped" '^/ground_truth/pose \[geometry_msgs/msg/PoseStamped\]' "$OUT/topic_list.txt"
ros2 topic info /map -v > "$OUT/map_info.txt" 2>&1
check_match "D5  /map is published with transient-local durability, so a late subscriber gets it" 'TRANSIENT_LOCAL' "$OUT/map_info.txt"

# ----------------------------------------------------------------- E: TF
section "E: the TF chain is complete and map -> odom has one owner"
timeout 12 ros2 run tf2_ros tf2_echo map LDS-01 > "$OUT/tf_map_laser.txt" 2>&1
check_match "E1  map -> LDS-01 resolves through the whole chain" 'Translation' "$OUT/tf_map_laser.txt"
timeout 12 ros2 run tf2_ros tf2_echo map base_link > "$OUT/tf_map_base.txt" 2>&1
check_match "E2  map -> base_link resolves" 'Translation' "$OUT/tf_map_base.txt"
ros2 topic info /tf -v > "$OUT/tf_publishers.txt" 2>&1
check_match "E3  the ground_truth_tf_adapter publishes /tf" 'Node name: ground_truth_tf_adapter' "$OUT/tf_publishers.txt"
refute "E4  mapping_node publishes no /tf" bash -c "awk '/Publisher count/{p=1} /Subscription count/{p=0} p' '$OUT/tf_publishers.txt' | grep -q 'Node name: mapping_node'"
refute "E5  localization_node is not running, so the edge has one owner" has_node /localization_node

# ----------------------------------------------------------------- F: provenance
section "F: provenance, who reads ground truth"
ros2 node info /mapping_node > "$OUT/mapping_node_info.txt" 2>&1
ros2 node info /ground_truth_tf_adapter > "$OUT/adapter_info.txt" 2>&1
refute "F1  mapping_node does not subscribe to /ground_truth/pose" \
    bash -c "awk '/^  Subscribers:/{s=1;next} /^  [A-Z][A-Za-z ]*:/{s=0} s' '$OUT/mapping_node_info.txt' | grep -q ground_truth"
check "F2  mapping_node subscribes to the scan" \
    bash -c "awk '/^  Subscribers:/{s=1;next} /^  [A-Z][A-Za-z ]*:/{s=0} s' '$OUT/mapping_node_info.txt' | grep -q '/scan'"
check "F3  the adapter subscribes to /ground_truth/pose" \
    bash -c "awk '/^  Subscribers:/{s=1;next} /^  [A-Z][A-Za-z ]*:/{s=0} s' '$OUT/adapter_info.txt' | grep -q ground_truth"
timeout 10 ros2 topic echo /diagnostics > "$OUT/diagnostics_idle.yaml" 2>&1
check_block_match "F4  the adapter's diagnostics name it as a ground-truth source" 'name: ground_truth_tf_adapter' 'provenance' "$OUT/diagnostics_idle.yaml" 12
check_block_match "F5  the mapper's diagnostics say the pose source is TF" 'name: mapping_node' 'pose_source' "$OUT/diagnostics_idle.yaml" 40

# ----------------------------------------------------------------- G: the mapper maps
section "G: the mapper maps"
echo "--- driving the smoke route so the map grows ---"
ros2 run robotics_localization_bridge route_driver --ros-args -p "route_file:=$ROUTE" -p laps:=1 \
    > "$OUT/route_driver.log" 2>&1
check "G1  route_driver finished the route" test "$?" = "0"
sleep 4
timeout 10 ros2 topic echo /diagnostics > "$OUT/diagnostics_after.yaml" 2>&1
check_block_match "G2  the mapper reports RUNNING" 'name: mapping_node' 'message: RUNNING' "$OUT/diagnostics_after.yaml" 3
python3 - "$OUT/diagnostics_after.yaml" > "$OUT/mapper_counts.txt" 2>&1 <<'PY'
import re, sys
text = open(sys.argv[1]).read()
blocks = text.split("name: mapping_node")
vals = {}
if len(blocks) > 1:
    for k, v in re.findall(r"key: (\w+)\s+value: '?([^'\n]*)'?", blocks[1].split("---")[0]):
        vals[k] = v
for k in ("scans_integrated", "occupied_cells", "free_cells", "skipped_no_transform", "skipped_motion", "pose_source"):
    print(f"{k}: {vals.get(k, 'missing')}")
PY
integrated=$(grep -oE '^scans_integrated: [0-9]+' "$OUT/mapper_counts.txt" | grep -oE '[0-9]+$' || echo 0)
occupied=$(grep -oE '^occupied_cells: [0-9]+' "$OUT/mapper_counts.txt" | grep -oE '[0-9]+$' || echo 0)
if [ "${integrated:-0}" -ge 20 ]; then ok "G3  at least 20 scans were integrated ($integrated)"; else bad "G3  at least 20 scans were integrated (${integrated:-0})"; fi
if [ "${occupied:-0}" -ge 100 ]; then ok "G4  the map holds occupied cells ($occupied)"; else bad "G4  the map holds occupied cells (${occupied:-0})"; fi
check_match "G5  the mapper reports its pose source as tf" '^pose_source: tf' "$OUT/mapper_counts.txt"
timeout 15 ros2 topic echo --once /esdf --field encoding > "$OUT/esdf_fields.txt" 2>&1
timeout 15 ros2 topic echo --once /esdf --field width >> "$OUT/esdf_fields.txt" 2>&1
timeout 15 ros2 topic echo --once /esdf --field step >> "$OUT/esdf_fields.txt" 2>&1
check_match "G6  the ESDF is 32FC1" '32FC1' "$OUT/esdf_fields.txt"
check "G7  the ESDF row step is four bytes per cell" \
    bash -c "w=\$(sed -n 3p '$OUT/esdf_fields.txt' | grep -oE '[0-9]+' | head -1); s=\$(sed -n 5p '$OUT/esdf_fields.txt' | grep -oE '[0-9]+' | head -1); [ -n \"\$w\" ] && [ \"\$s\" = \"\$((w * 4))\" ]"
timeout 15 ros2 topic echo --once /map_entropy > "$OUT/map_entropy.txt" 2>&1
check_match "G8  the entropy topic publishes a number" '^data: [0-9]' "$OUT/map_entropy.txt"

# ----------------------------------------------------------------- H: shutdown
section "H: the stack shuts down and leaves nothing behind"
trap - EXIT
kill -INT -- -"$LAUNCH" 2>/dev/null
for _ in $(seq 1 60); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
if kill -0 "$LAUNCH" 2>/dev/null; then bad "H1  the launch exits within 60 s of SIGINT"; kill -9 -- -"$LAUNCH" 2>/dev/null; else ok "H1  the launch exits within 60 s of SIGINT"; fi
sleep 3
pkill -x webots-bin 2>/dev/null
check "H2  no ground_truth_tf_adapter process is left" bash -c "! pgrep -x ground_truth_tf_adapter"
check "H3  no mapping_node process is left" bash -c "! pgrep -x mapping_node"
check "H4  no webots-bin process is left" bash -c "! pgrep -x webots-bin"
check_match "H5  mapping_node logged an orderly shutdown" 'mapping_node shutting down' "$LAUNCH_LOG"

# ----------------------------------------------------------------- summary
section "summary"
TOTAL=$((PASS + FAIL))
{
    echo "checks: $TOTAL"
    echo "passed: $PASS"
    echo "failed: $FAIL"
    if [ "$FAIL" -gt 0 ]; then
        echo "failed_checks:"
        for n in "${FAILED_NAMES[@]}"; do echo "  - $n"; done
    fi
    echo "verdict: $([ "$FAIL" -eq 0 ] && echo PASS || echo FAIL)"
} | tee "$OUT/summary.yaml"
[ "$FAIL" -eq 0 ]
