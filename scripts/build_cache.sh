#!/bin/bash
# Source this file (do not execute it). Heavy, disposable build output goes to D: instead of the WSL
# distro's own vhdx on C:, which grows when cargo/colcon write and does not shrink when they delete.
#
#   source "$SCRIPT_DIR/build_cache.sh"      # env.sh does this; scripts that do not source env.sh do it themselves
#
# What it sets, only when $BUILD_CACHE_ROOT (default /mnt/d/build-cache) exists, so a machine without D: keeps
# the old in-tree behaviour:
#   CARGO_TARGET_DIR          $BUILD_CACHE_ROOT/cargo-target/<workspace directory name>
#   COLCON_BUILD_BASE_ROOT    $BUILD_CACHE_ROOT/colcon/<workspace directory name>   (scripts/build_release.sh and the
#                             gates add --build-base under it; colcon's INSTALL base stays in the workspace, it is small
#                             and the generated .cargo/config.toml patch paths point at it)
#   COLCON_LOG_PATH          $COLCON_BUILD_BASE_ROOT/log
# One sub-directory per workspace (the mirror ~/ros2-rust-robotics, the private ~/slam_nodes_ws, ...), never one shared
# target directory: a shared one would make a build wait for another workspace's cargo lock and could replace a binary
# that a long benchmark in the other workspace is running. Both variables honour a value already in the environment.
#
# Cost, measured 2026-10-15 for `cargo build --release -p robotics_slam_ros --example render_check`: cold 1m02s on the WSL
# filesystem against 1m16s on D:; a one-file incremental rebuild 4.1 s against 12.9 s; a no-op build 0.1 s against 2.8 s.
_BC_ROOT="${BUILD_CACHE_ROOT:-/mnt/d/build-cache}"
if [ -d "$_BC_ROOT" ]; then
    _BC_WS="$(basename "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)")"
    export BUILD_CACHE_ROOT="$_BC_ROOT"
    export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$_BC_ROOT/cargo-target/$_BC_WS}"
    export COLCON_BUILD_BASE_ROOT="${COLCON_BUILD_BASE_ROOT:-$_BC_ROOT/colcon/$_BC_WS}"
    export COLCON_LOG_PATH="${COLCON_LOG_PATH:-$COLCON_BUILD_BASE_ROOT/log}"
    unset _BC_WS
fi
unset _BC_ROOT
