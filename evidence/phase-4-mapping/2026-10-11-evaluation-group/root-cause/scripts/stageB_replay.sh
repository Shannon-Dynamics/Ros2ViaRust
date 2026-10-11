#!/usr/bin/env bash
# Stage B verification: replay the four recorded scan streams with replay_scans.rs under each beam layout, grade with map_metrics.rs.
# No Webots. Run from the build mirror after `source scripts/env.sh`:   bash stageB_replay.sh <repo root of the mirror> <evidence/phase-4-mapping>
set -eu
REPO=${1:?mirror repo root}; EVID=${2:?evidence/phase-4-mapping}
W=$(mktemp -d); cd "$REPO"
cargo build -q --release -p robotics_mapping --examples
for run in development-3-route-long evaluation-1-cross evaluation-2-wall-lobes evaluation-3-diamond-slow; do
    D="$EVID/2026-10-11-$run"
    for layout in published pixel_centres; do
        cargo run -q --release -p robotics_mapping --example replay_scans -- "$D/integrated-scans.csv" "$D/final-grid.txt" "$W/$run-$layout.txt" clear_to_max_range supercover 0.10 "$layout" 2> "$W/$run-$layout.log"
        cargo run -q --release -p robotics_mapping --example map_metrics -- "$W/$run-$layout.txt" sim/webots/config/landmarks.yaml "$D/trajectory.csv" 1.0 "$W/$run-$layout.yaml"
    done
    if cmp -s "$W/$run-published.txt" "$D/final-grid.txt"; then echo "$run: published replay equals the saved final-grid.txt byte for byte"; else echo "$run: PUBLISHED REPLAY DIFFERS from final-grid.txt"; fi
    echo "$run: pixel_centres: $(cat "$W/$run-pixel_centres.log")"
done
python3 -I - "$W" <<'PY'
import sys, re
W = sys.argv[1]
runs = ["development-3-route-long", "evaluation-1-cross", "evaluation-2-wall-lobes", "evaluation-3-diamond-slow"]
def kv(p):
    d = {}
    for l in open(p):
        if ":" in l: k, v = l.split(":", 1); d[k.strip()] = v.strip()
    return d
print(f"\n{'run':27s} {'layout':13s} {'occupied':>8s} {'false occ':>9s} {'fraction':>8s} {'p95 cells':>9s} {'max cells':>9s} {'complete':>8s} {'false free':>10s} {'coverage':>8s}")
for r in runs:
    for L in ("published", "pixel_centres"):
        m = kv(f"{W}/{r}-{L}.yaml")
        print(f"{r:27s} {L:13s} {int(m['occupied_cells']):8d} {int(m['false_occupied_cells']):9d} {float(m['false_occupied_fraction'])*100:7.2f}% {float(m['surface_error_p95_cells']):9.2f} {float(m['surface_error_max_cells']):9.2f} {float(m['completeness']):8.4f} {float(m['false_free_fraction'])*100:9.2f}% {float(m['coverage']):8.4f}")
PY
python3 -I - "$W" <<'PY'
import sys, collections
W = sys.argv[1]
def grid(p):
    L = open(p).read().split("\n"); hdr = {}
    for l in L:
        if l.strip() == "data": break
        k, _, v = l.partition(" "); hdr[k] = v
    rows = [list(map(float, l.split())) for l in L[L.index("data") + 1:] if l.strip()]
    thr = float(hdr["occupied_threshold"]); return [[v >= thr for v in r] for r in rows]
def cells(w, m):
    if w == "E": return [(126 + k, m) for k in range(6)]
    if w == "W": return [(13 - k, m) for k in range(6)]
    if w == "N": return [(m, 126 + k) for k in range(6)]
    return [(m, 13 - k) for k in range(6)]
print("\nroute 3 wall cross-section in the grids written by replay_scans.rs: [inner cell, outer cell, outer+1 cell] counted from the true face, X = occupied; positions along each wall 14..125")
for layout in ("published", "pixel_centres"):
    g = grid(f"{W}/evaluation-3-diamond-slow-{layout}.txt"); line = []
    for w in "ENWS":
        pat = collections.Counter("".join("X" if g[j][i] else "." for i, j in cells(w, m))[3:] for m in range(14, 126))
        line.append(f"{w}: " + " ".join(f"{k}:{v}" for k, v in pat.most_common(5)))
    print(f"  {layout:13s} " + " | ".join(line))
PY
rm -rf "$W"
