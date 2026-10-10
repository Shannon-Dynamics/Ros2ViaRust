#!/bin/bash
# Run every quality gate from the WSL mirror (called through scripts/wsl_run.sh or directly
# from a sourced shell at the repository root).
#
#   cargo fmt --check · clippy -D warnings · cargo test --workspace · colcon build
#
# Exit code is non-zero if any gate fails; a summary is printed at the end.
set +u
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
declare -A RESULT
run() {
    local name="$1"; shift
    echo "=== $name: $* ==="
    if "$@"; then RESULT[$name]=PASS; else RESULT[$name]=FAIL; fi
}
run fmt cargo fmt --all --check
run clippy cargo clippy --workspace --all-targets --all-features -- -D warnings
echo "=== test: cargo test --workspace ==="
cargo test --workspace 2>&1 | tee /tmp/gates_test.log | grep -E "^test result|FAILED|panicked"
if grep -q "FAILED\|error\[" /tmp/gates_test.log; then RESULT[test]=FAIL; else RESULT[test]=PASS; fi
awk '/^test result/ {p+=$4; f+=$6} END {print "total tests: passed=" p " failed=" f}' /tmp/gates_test.log
echo "=== colcon: colcon build (ros2_ws) ==="
( cd ros2_ws && colcon build --event-handlers console_cohesion+ desktop_notification- status- ) > /tmp/gates_colcon.log 2>&1
if grep -q "^Summary: .* packages finished" /tmp/gates_colcon.log && ! grep -q "packages failed" /tmp/gates_colcon.log; then RESULT[colcon]=PASS; else RESULT[colcon]=FAIL; fi
grep -E "^(Finished|Failed|Summary)|packages failed" /tmp/gates_colcon.log
echo "=== summary ==="
rc=0
for g in fmt clippy test colcon; do
    echo "$g: ${RESULT[$g]}"
    [ "${RESULT[$g]}" = PASS ] || rc=1
done
exit $rc
