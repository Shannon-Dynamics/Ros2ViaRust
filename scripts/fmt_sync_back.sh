#!/bin/bash
# Format all Rust sources in the WSL mirror and copy the formatted .rs files (and the
# Cargo.lock that builds in the mirror resolve) back to the Windows checkout — the only
# files that ever flow back.
set +u
bash "/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/scripts/sync_to_wsl.sh" > /dev/null || exit 1
cd "$HOME/ros2-rust-robotics" || exit 1
source scripts/env.sh > /dev/null 2>&1
cargo fmt --all || exit 1
DST="/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics"
rsync -a --include='*/' --include='*.rs' --exclude='*' crates/ "$DST/crates/"
rsync -a --include='*/' --include='*.rs' --exclude='target/' --exclude='*' ros2_ws/src/ "$DST/ros2_ws/src/"
cargo metadata --format-version 1 > /dev/null && cp Cargo.lock "$DST/Cargo.lock"
cargo fmt --all --check && echo "fmt clean, formatted sources copied back"
