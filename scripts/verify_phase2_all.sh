#!/bin/bash
# Phase 2 closure: the complete verification sequence, in order.
#
#   1 cargo fmt --check        2 clippy -D warnings        3 cargo test --workspace
#   4 interface underlay check 5 colcon build              6 source ROS + underlay + ws
#   7–16 runtime CLI evidence (scripts/verify_phase2_runtime.sh: nodes, topics, rates,
#        QoS, parameters, odom→base_link, base_link→laser, estimator metrics, Ctrl-C)
#   17 Rust integration test (scripts/integration_test_phase2.sh)
#
# Run from Windows as
#   wsl.exe -d Ubuntu-24.04 -- bash "/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/scripts/wsl_run.sh" bash scripts/verify_phase2_all.sh
# Summary: ~/phase2_final/summary.log. Exit code 0 only if every step passed.
set +u
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
OUT="$HOME/phase2_final"
mkdir -p "$OUT"
exec > >(tee "$OUT/summary.log") 2>&1
source scripts/env.sh > /dev/null 2>&1
declare -a NAMES RESULTS
record() { NAMES+=("$1"); RESULTS+=("$2"); echo ">>> $1: $2"; }

echo "=== Phase 2 final verification $(date -Is) on $(lsb_release -ds), ROS_DISTRO=$ROS_DISTRO, RMW=$RMW_IMPLEMENTATION, $(cargo --version) ==="

echo "=== 1. cargo fmt --all --check ==="
cargo fmt --all --check && record "1 fmt" PASS || record "1 fmt" FAIL

echo "=== 2. cargo clippy --workspace --all-targets --all-features -- -D warnings ==="
cargo clippy --workspace --all-targets --all-features -- -D warnings > "$OUT/clippy.log" 2>&1 \
    && record "2 clippy" PASS || { tail -30 "$OUT/clippy.log"; record "2 clippy" FAIL; }

echo "=== 3. cargo test --workspace ==="
cargo test --workspace > "$OUT/test.log" 2>&1; rc=$?
grep -E "^ +(Running|Doc-tests)|^test result" "$OUT/test.log" \
    | sed -E 's#^ +##; s#target/debug/deps/([a-z_0-9]+)-[0-9a-f]+#\1#; s#; finished in .*##'
awk '/^test result/ {p+=$4; f+=$6} END {print "TOTAL: " p " passed, " f " failed"}' "$OUT/test.log"
[ $rc -eq 0 ] && ! grep -q "FAILED" "$OUT/test.log" && record "3 test" PASS || record "3 test" FAIL

echo "=== 4. interface underlay (Rust message crates) ==="
UNDERLAY="${INTERFACES_WS:-$HOME/ros2_rust_interfaces_ws}/install"
missing=""
for pkg in rcl_interfaces rosgraph_msgs std_msgs geometry_msgs sensor_msgs nav_msgs tf2_msgs diagnostic_msgs; do
    [ -d "$UNDERLAY/$pkg/share/$pkg/rust" ] || missing="$missing $pkg"
done
[ -z "$missing" ] && { echo "all Rust interface crates present in $UNDERLAY"; record "4 underlay" PASS; } \
    || { echo "missing:$missing (run scripts/setup_interfaces_underlay.sh)"; record "4 underlay" FAIL; }

echo "=== 5. colcon build (ros2_ws) ==="
( cd ros2_ws && colcon build --event-handlers console_cohesion+ desktop_notification- status- ) > "$OUT/colcon.log" 2>&1
grep -E "^(Finished|Failed|Summary)|packages failed" "$OUT/colcon.log"
grep -q "^Summary: .* packages finished" "$OUT/colcon.log" && ! grep -q "packages failed" "$OUT/colcon.log" \
    && record "5 colcon" PASS || record "5 colcon" FAIL

echo "=== 6. source ROS 2 + underlay + workspace ==="
source ros2_ws/install/setup.bash
ros2 pkg executables robotics_sim_bridge && ros2 pkg executables robotics_estimation && ros2 pkg executables robotics_integration \
    && record "6 environment" PASS || record "6 environment" FAIL

echo "=== 7-16. runtime CLI verification (scripts/verify_phase2_runtime.sh, log ~/phase2_runtime/verify.log) ==="
bash scripts/verify_phase2_runtime.sh 60 > "$OUT/runtime.log" 2>&1
grep -qE "^launch exit code: 0" "$OUT/runtime.log" && grep -qE "^none$" "$OUT/runtime.log" \
    && grep -q "average rate" "$OUT/runtime.log" && grep -q "Translation: \[0.050, 0.000, 0.000\]" "$OUT/runtime.log" \
    && record "7-16 runtime" PASS || record "7-16 runtime" FAIL
grep -E "=== ros2 topic hz|average rate:|^launch exit code" "$OUT/runtime.log"

echo "=== 17. Rust integration test (scripts/integration_test_phase2.sh) ==="
bash scripts/integration_test_phase2.sh > "$OUT/integration.log" 2>&1 && record "17 integration" PASS || record "17 integration" FAIL
grep -E "checks, .* passed|FAIL\]" "$OUT/integration.log"

echo "=== summary ==="
rc=0
for i in "${!NAMES[@]}"; do
    printf "%-16s %s\n" "${NAMES[$i]}" "${RESULTS[$i]}"
    [ "${RESULTS[$i]}" = PASS ] || rc=1
done
echo "overall: $([ $rc -eq 0 ] && echo PASS || echo FAIL)"
exit $rc
