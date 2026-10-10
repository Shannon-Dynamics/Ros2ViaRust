#!/bin/bash
# Run every quality gate from the WSL mirror (called through scripts/wsl_run.sh or directly
# from a sourced shell at the repository root).
#
#   cargo fmt --check · clippy -D warnings · cargo test --workspace · colcon build · docs
#
# `docs` is scripts/phase3_global_tables.py --check: the global localization tables of the
# documentation are generated from the saved evidence, and the gate fails if a document no
# longer matches it (an experiment was added, or a table was edited by hand). The same gate
# regenerates the wrong-claim symmetry analysis (scripts/phase3_wrong_claim_analysis.py) and
# fails if the saved wrong-claims-yaw.yaml no longer matches the evidence.
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
echo "=== docs: the global localization tables match the evidence ==="
if python3 -I scripts/phase3_global_tables.py --check docs/phase-3-localization/benchmark.md \
    && python3 -I scripts/phase3_wrong_claim_analysis.py "${EVIDENCE_ROOT:-/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/evidence}/phase-3-global-localization" \
        --check "${EVIDENCE_ROOT:-/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/evidence}/phase-3-global-localization/2026-10-10-ambiguity/wrong-claims-yaw.yaml"; then
    RESULT[docs]=PASS
else
    RESULT[docs]=FAIL
fi
echo "=== summary ==="
rc=0
for g in fmt clippy test colcon docs; do
    echo "$g: ${RESULT[$g]}"
    [ "${RESULT[$g]}" = PASS ] || rc=1
done
exit $rc
