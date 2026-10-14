#!/usr/bin/env bash
# Global localization from a uniform prior, model-level experiment: the real McLocalizer
# against a simulated robot and simulated pillar detections, many trials.
#
#   bash scripts/run_phase3_global_synthetic.sh <run-name> [extra global_trials flags...]
#
# One sub-directory per configuration under
# <EVIDENCE_ROOT>/phase-3-global-localization/<run-name>/, holding trials.csv (one row per
# trial), summary.yaml (computed from it, nothing typed) and the exact command line.
# Configurations run in parallel and share start poses (same --starts-seed), so they are
# paired comparisons.
#
# Environment:
#   PARTICLES   space separated global particle counts, one configuration each
#               (default "6000")
#   TRIALS      trials per configuration (default 40)
#   SEED        filter RNG seed (default 1)
#   STARTS      "random" (default) or a CSV of x,y,yaw start poses
#   SEQUENTIAL  1 to run the configurations one after another instead of in parallel. A
#               timing measurement needs it: configurations sharing a machine inflate each
#               other's cost per scan, which is why the parallel sweeps' timing columns are
#               not the ones to read.
#
# This is not the Webots experiment. It has no detector, no beam geometry and exact
# odometry; see the header of crates/robotics_localization/examples/global_trials.rs for
# exactly what it does and does not include. Run scripts/run_phase3_global_trials.sh for
# the end-to-end version.
set -u
RUN=${1:?usage: run_phase3_global_synthetic.sh <run-name> [extra global_trials flags]}
shift
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/evidence_root.sh"   # EVIDENCE_ROOT: the Windows checkout when mounted
[ -f "$SCRIPT_DIR/env.sh" ] && source "$SCRIPT_DIR/env.sh" > /dev/null 2>&1

OUT_ROOT="$EVIDENCE_ROOT/phase-3-global-localization/$RUN"
if [ -e "$OUT_ROOT" ]; then
    echo "refusing to overwrite $OUT_ROOT (evidence is never overwritten); pick a new run name" >&2
    exit 1
fi
mkdir -p "$OUT_ROOT"
( cd "$REPO" && cargo build --release -p robotics_localization --example global_trials ) \
    > "$OUT_ROOT/build.log" 2>&1 || { echo "build failed, see $OUT_ROOT/build.log" >&2; exit 1; }
BIN="${CARGO_TARGET_DIR:-$REPO/target}/release/examples/global_trials"
MAP="$REPO/sim/webots/config/landmarks.yaml"
{
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "host: $(hostname)"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
    echo "rustc: $(rustc --version)"
    echo "binary_sha256: $(sha256sum "$BIN" | cut -d' ' -f1)"
    echo "map_sha256: $(sha256sum "$MAP" | cut -d' ' -f1)"
    echo "git_head: $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    echo "PARTICLES: ${PARTICLES:-6000}"
    echo "TRIALS: ${TRIALS:-40}"
    echo "SEED: ${SEED:-1}"
    echo "STARTS: ${STARTS:-random}"
    echo "extra_flags: $*"
} > "$OUT_ROOT/environment.txt"

pids=()
for n in ${PARTICLES:-6000}; do
    d="$OUT_ROOT/p$n"
    mkdir -p "$d"
    cmd=("$BIN" trials --map "$MAP" --particles "$n" --trials "${TRIALS:-40}" --seed "${SEED:-1}"
         --starts "${STARTS:-random}" --label "$RUN-p$n" --out "$d" "$@")
    printf '%q ' "${cmd[@]}" > "$d/command.txt"; echo >> "$d/command.txt"
    if [ "${SEQUENTIAL:-0}" = 1 ]; then
        "${cmd[@]}" > "$d/run.log" 2>&1 || rc_seq=1
    else
        "${cmd[@]}" > "$d/run.log" 2>&1 &
        pids+=($!)
    fi
done
rc=${rc_seq:-0}
for p in "${pids[@]}"; do wait "$p" || rc=1; done
echo "done, evidence in $OUT_ROOT (exit $rc)"
exit $rc
