#!/usr/bin/env bash
# Build nodes in release mode into their own colcon install space, so the standard debug build
# every gate and script uses is left alone.
#
#   bash scripts/build_release.sh            # Phase 3: robotics_localization_bridge (the default, unchanged)
#   bash scripts/build_release.sh slam       # Phase 5: robotics_slam_ros (scan_matcher_node, pose_graph_node)
#
# Why it exists. `colcon build` builds Rust packages in the debug profile, and the node of a
# global localization search with tens of thousands of particles does O(particles) work per
# scan: in a debug build that is slower than the 5 Hz scan rate, which would make a
# convergence time a measurement of the build profile rather than of the filter. The nodes
# in ros2_ws/install_release are what RELEASE=1 of scripts/run_phase3_global_trials.sh runs.
# The directories are ignored by git and excluded from scripts/sync_to_wsl.sh's --delete.
#
# Phase 5 has the same problem for a different reason: a scan matcher that has lost track (a
# recording with no turning scans, for instance) costs seconds per step in a debug build, so a
# replay test of the SLAM nodes would measure the build profile. `slam` builds robotics_slam_ros
# into the same install_release space; scripts/verify_phase5_slam_nodes.sh uses it when it exists.
# The launch file (robotics_bringup) and robotics_ros_bridge come from the debug install that
# scripts/env.sh sources as the underlay; only the node binaries are release builds.
#
# Extra arguments after the target are passed to colcon (e.g. --executor sequential).
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/env.sh" > /dev/null 2>&1
cd "$SCRIPT_DIR/../ros2_ws"
TARGET=${1:-localization}
[ $# -gt 0 ] && shift
case "$TARGET" in
    localization) PKG=robotics_localization_bridge ;;
    slam)         PKG=robotics_slam_ros ;;
    *) echo "build_release.sh: unknown target '$TARGET' (localization | slam)" >&2; exit 2 ;;
esac
colcon build --build-base "${COLCON_BUILD_BASE_ROOT:+$COLCON_BUILD_BASE_ROOT/}build_release" --install-base install_release \
    --packages-select "$PKG" --cargo-args --release \
    --allow-overriding "$PKG" "$@"
