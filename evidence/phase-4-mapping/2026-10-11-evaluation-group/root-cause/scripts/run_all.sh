#!/usr/bin/env bash
# Read-only root-cause analysis of the four Phase 4 runs. Changes nothing under crates/, ros2_ws/ or sim/;
# reads the four evidence directories and writes only ../output/*.txt. About 5 minutes.
#   bash run_all.sh
set -eu
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export RC_EVIDENCE=$(cd "$HERE/../../.." && pwd)        # evidence/phase-4-mapping
WORK=$(mktemp -d)                                        # pickles of the replays live here, not in the evidence tree
cp "$HERE"/*.py "$WORK"/
cd "$WORK"
mkdir -p "$HERE/../output"
i=0
for s in validate validate2 tiebreak lead1 lead1_table profile pillars qstats nearmiss lead2_basic lead2 residual warp warp2 predict timeoffset omega_geom; do
    i=$((i + 1)); out=$(printf '%s/../output/%02d-%s.txt' "$HERE" "$i" "$s")
    echo "running $s"; python3 -I "$s.py" > "$out" 2>&1
done
rm -rf "$WORK"
