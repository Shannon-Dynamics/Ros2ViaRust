#!/usr/bin/env bash
# Capped pillar diagnostic, clamp sweep tested by replay (replay_scans.rs + map_metrics.rs, no Webots), beam layout pixel_centres throughout:
#   tracer supercover (the frozen choice) or bresenham (an existing option), and the upper log-odds clamp l_max (read by replay_scans from the template grid header).
#   bash pillar_candidates.sh <mirror repo root> <evidence/phase-4-mapping>
set -eu
REPO=${1:?}; EVID=${2:?}; W=$(mktemp -d); cd "$REPO"
cargo build -q --release -p robotics_mapping --examples
echo "run                         variant                              occupied false occ  p95  complete false free   M1 checks passed (of 7)"
for run in development-3-route-long evaluation-1-cross evaluation-2-wall-lobes evaluation-3-diamond-slow; do
    D="$EVID/2026-10-11-$run"
    for variant in "supercover 3.4760988" "supercover 4.0" "supercover 5.0" "supercover 6.0" "supercover 8.0" "bresenham 3.4760988" "bresenham 4.0" "bresenham 5.0" "bresenham 6.0" "bresenham 8.0"; do
        set -- $variant; tracer=$1; lmax=$2
        sed "s/^l_max .*/l_max $lmax/" "$D/final-grid.txt" > "$W/template.txt"
        cargo run -q --release -p robotics_mapping --example replay_scans -- "$D/integrated-scans.csv" "$W/template.txt" "$W/out.txt" clear_to_max_range "$tracer" 0.10 pixel_centres 2> /dev/null
        cargo run -q --release -p robotics_mapping --example map_metrics -- "$W/out.txt" sim/webots/config/landmarks.yaml "$D/trajectory.csv" 1.0 "$W/m.yaml"
        python3 -I - "$W/m.yaml" "$run" "$tracer l_max=$lmax" <<'PY'
import sys
m = {}
for l in open(sys.argv[1]):
    if ":" in l: k, v = l.split(":", 1); m[k.strip()] = v.strip()
f = lambda k: float(m[k])
checks = [f("surface_error_p95_cells") <= 1.0 + 1e-9, f("surface_error_max_cells") <= 1.5 + 1e-9, f("false_occupied_fraction") <= 0.02 + 1e-9, f("completeness") >= 0.95 - 1e-9, f("false_free_fraction") <= 0.05 + 1e-9, f("coverage") >= 0.9 - 1e-9, f("observable_fraction") >= 0.5 - 1e-9]
print(f"{sys.argv[2]:27s} {sys.argv[3]:36s} {int(m['occupied_cells']):6d} {int(m['false_occupied_cells']):9d} {f('surface_error_p95_cells'):5.2f} {f('completeness'):8.4f} {f('false_free_fraction')*100:8.2f}%   {sum(checks)}")
PY
    done
done
rm -rf "$W"
