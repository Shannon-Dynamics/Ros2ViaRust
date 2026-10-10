#!/usr/bin/env bash
# Build the localization nodes in release mode into their own colcon install space, so the
# standard debug build every gate and script uses is left alone.
#
#   bash scripts/build_release.sh
#
# Why it exists. `colcon build` builds Rust packages in the debug profile, and the node of a
# global localization search with tens of thousands of particles does O(particles) work per
# scan: in a debug build that is slower than the 5 Hz scan rate, which would make a
# convergence time a measurement of the build profile rather than of the filter. The nodes
# in ros2_ws/install_release are what RELEASE=1 of scripts/run_phase3_global_trials.sh runs.
# The directories are ignored by git and excluded from scripts/sync_to_wsl.sh's --delete.
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/env.sh" > /dev/null 2>&1
cd "$SCRIPT_DIR/../ros2_ws"
colcon build --build-base build_release --install-base install_release \
    --packages-select robotics_localization_bridge --cargo-args --release \
    --allow-overriding robotics_localization_bridge
