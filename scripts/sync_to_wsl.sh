#!/bin/bash
# Mirror the Windows checkout into the Linux filesystem for building (run inside WSL).
#
# The source of truth is the Windows path (git repo); builds happen in $HOME so cargo and
# colcon do not pay the /mnt/d filesystem penalty. Build products never flow back.
#
#   bash scripts/sync_to_wsl.sh            # from anywhere inside WSL
#
# WARNING, and it has cost a full set of runs already. `--delete` means anything that
# exists only in the mirror is removed, and that includes output a run just produced: the
# runs under evidence/ and sim/webots/evidence/ are written by scripts executed in the
# mirror, so until they have been copied to the Windows checkout the next sync deletes
# them. On 2026-10-10 a sync run between a measurement and the integration test erased both
# a 6270-row evaluation and a partial test log that way; neither was recoverable. Copy new
# evidence to the Windows path first, then sync.
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
