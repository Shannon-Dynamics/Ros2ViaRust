#!/bin/bash
# Mirror the Windows checkout into the Linux filesystem for building (run inside WSL).
#
# The source of truth is the Windows path (git repo); builds happen in $HOME so cargo and
# colcon do not pay the /mnt/d filesystem penalty. Build products never flow back.
#
#   bash scripts/sync_to_wsl.sh            # from anywhere inside WSL
set -e
SRC="${SRC:-/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/}"
DST="${DST:-$HOME/ros2-rust-robotics/}"
mkdir -p "$DST"
rsync -a --delete \
    --exclude '.git/' \
    --exclude 'target/' \
    --exclude 'ros2_ws/build/' \
    --exclude 'ros2_ws/install/' \
    --exclude 'ros2_ws/log/' \
    --exclude 'ros2_ws/.cargo/' \
    "$SRC" "$DST"
echo "synced $SRC -> $DST"
