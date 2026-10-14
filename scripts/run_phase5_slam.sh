#!/bin/bash
# One Phase 5 evidence run: harness, environment record, verdict.
#
#   bash scripts/run_phase5_slam.sh synthetic NAME ROUTE.yaml [harness args...]
#   bash scripts/run_phase5_slam.sh recorded  NAME integrated-scans.csv [harness args...]
#
# Writes $EVIDENCE_ROOT/phase-5-slam/NAME/ (EVIDENCE_ROOT: scripts/evidence_root.sh, the Windows
# checkout's evidence/ directory, so a run is never only in the mirror): slam-metrics.yaml, the
# recorded scan stream for a synthetic run, environment.txt, source-sha256.txt, run.log,
# phase5-verdict.yaml, verdict.console. It REFUSES to overwrite a run that already has metrics:
# evidence is never edited and never replaced. The harness binaries are the release examples built
# in the WSL mirror (`cargo build --release -p robotics_slam --examples`); BIN_DIR overrides.
#
# The harness does not know the tolerances and this script does not pass any: they live in
# scripts/phase5_verdict.py, which is run last, on the metrics file the harness wrote.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/build_cache.sh"
source "$SCRIPT_DIR/evidence_root.sh"
BIN_DIR="${BIN_DIR:-${CARGO_TARGET_DIR:-$HOME/ros2-rust-robotics/target}/release/examples}"
MODE="${1:?mode: synthetic | recorded}"; NAME="${2:?run name}"; INPUT="${3:?route or scans file}"
shift 3
OUT="$EVIDENCE_ROOT/phase-5-slam/$NAME"
if [ -e "$OUT/slam-metrics.yaml" ]; then
    echo "refusing to overwrite $OUT/slam-metrics.yaml: evidence is never replaced" >&2
    exit 3
fi
mkdir -p "$OUT"
case "$MODE" in
    synthetic) BIN="$BIN_DIR/slam_synthetic_run"; ARGS=(--route "$INPUT") ;;
    recorded)  BIN="$BIN_DIR/slam_recorded_run";  ARGS=(--scans "$INPUT") ;;
    *) echo "unknown mode $MODE" >&2; exit 2 ;;
esac
[ -x "$BIN" ] || { echo "no harness binary at $BIN" >&2; exit 2; }

{
    echo "date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | sed 's/.*: //')"
    echo "kernel: $(uname -r)"
    echo "rustc: $(rustc --version 2>/dev/null)"
    echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unavailable)"
    echo "source_git_commit: $(git -C "$REPO" rev-parse HEAD 2>/dev/null)"
    echo "source_git_dirty_paths: $(git -C "$REPO" status --short -- crates/robotics_slam scripts/phase5_verdict.py scripts/phase5_verdict_selftest.py scripts/run_phase5_slam.sh Cargo.toml 2>/dev/null | tr '\n' ';')"
    echo "harness_binary: $BIN"
    echo "harness_binary_sha256: $(sha256sum "$BIN" | cut -d' ' -f1)"
    echo "verdict_script_sha256: $(sha256sum "$REPO/scripts/phase5_verdict.py" | cut -d' ' -f1)"
    echo "input: $INPUT"
    echo "input_sha256: $(sha256sum "$INPUT" | cut -d' ' -f1)"
    echo "harness_args: ${ARGS[*]} $*"
    echo "mode: $MODE"
} > "$OUT/environment.txt"
( cd "$REPO" && find crates/robotics_slam -type f \( -name '*.rs' -o -name Cargo.toml \) | sort | xargs sha256sum
  sha256sum Cargo.toml scripts/phase5_verdict.py ) > "$OUT/source-sha256.txt"

echo "running $MODE $NAME ..."
start=$(date +%s)
"$BIN" --out "$OUT" "${ARGS[@]}" "$@" > "$OUT/run.log" 2>&1
rc=$?
echo "harness exit code $rc after $(( $(date +%s) - start )) s" | tee -a "$OUT/run.log"
[ "$rc" -eq 0 ] || exit "$rc"
python3 -I "$REPO/scripts/phase5_verdict.py" "$OUT" > "$OUT/verdict.console"
vrc=$?
echo "verdict exit code $vrc (0 graded and nothing failed, 1 a claim failed, 2 refused)"
exit "$vrc"
