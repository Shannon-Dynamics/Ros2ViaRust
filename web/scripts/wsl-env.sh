#!/bin/bash
# Source this in WSL before building from the ~/rvr-web copy:  source scripts/wsl-env.sh
# (see scripts/wsl-sync.sh). Points the build and the checks at the real repository on
# Windows, and keeps Rust build output on the Linux filesystem.
export PATH="$HOME/.local/node/bin:$HOME/.cargo/bin:$PATH"
export BOOK_REPO_ROOT="${BOOK_REPO_ROOT:-/mnt/d/Shannon/ROS2 via RUST/ros2-rust-robotics}"
export ROBOTICS_WASM_CRATE="$BOOK_REPO_ROOT/crates/robotics_wasm"
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$HOME/cargo_target_wasm}"
