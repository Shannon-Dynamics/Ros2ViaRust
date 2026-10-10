#!/usr/bin/env bash
# Model-level kidnap recovery of the tracking filter, exactly as configured, many trials.
#
#   STEADY="6000 48000" TRIALS=200 bash scripts/run_phase3_kidnap_model.sh <run-name>
#
# One sub-directory per steady-state particle count under
# <EVIDENCE_ROOT>/phase-3-global-localization/<run-name>/. The tracking filter's kidnap
# tuning (20-scan persistence, 100 per cent reset) is not a flag here on purpose: this is a
# measurement of it as it is. The particle count is the one what-if, and a row at a count
# other than 6000 is labelled as such in the documentation.
set -u
RUN=${1:?usage: run_phase3_kidnap_model.sh <run-name>}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/evidence_root.sh"
[ -f "$SCRIPT_DIR/env.sh" ] && source "$SCRIPT_DIR/env.sh" > /dev/null 2>&1
OUT_ROOT="$EVIDENCE_ROOT/phase-3-global-localization/$RUN"
[ -e "$OUT_ROOT" ] && { echo "refusing to overwrite $OUT_ROOT" >&2; exit 1; }
mkdir -p "$OUT_ROOT"
( cd "$REPO" && cargo build --release -p robotics_localization --example global_trials ) \
    > "$OUT_ROOT/build.log" 2>&1 || { echo "build failed" >&2; exit 1; }
BIN="$REPO/target/release/examples/global_trials"
MAP="$REPO/sim/webots/config/landmarks.yaml"
{
    echo "date_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ //')"
    echo "rustc: $(rustc --version)"
    echo "binary_sha256: $(sha256sum "$BIN" | cut -d' ' -f1)"
    echo "map_sha256: $(sha256sum "$MAP" | cut -d' ' -f1)"
    echo "STEADY: ${STEADY:-6000}"
    echo "TRIALS: ${TRIALS:-100}"
} > "$OUT_ROOT/environment.txt"
pids=()
for n in ${STEADY:-6000}; do
    d="$OUT_ROOT/p$n"; mkdir -p "$d"
    cmd=("$BIN" kidnap --map "$MAP" --steady "$n" --trials "${TRIALS:-100}" --seed "${SEED:-1}" \
         --label "$RUN-p$n" --out "$d")
    printf '%q ' "${cmd[@]}" > "$d/command.txt"; echo >> "$d/command.txt"
    "${cmd[@]}" > "$d/run.log" 2>&1 &
    pids+=($!)
done
rc=0; for p in "${pids[@]}"; do wait "$p" || rc=1; done
echo "done, evidence in $OUT_ROOT (exit $rc)"; exit $rc
