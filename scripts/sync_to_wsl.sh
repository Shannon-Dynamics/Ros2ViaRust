#!/bin/bash
# Mirror the Windows checkout into the Linux filesystem for building (run inside WSL).
#
# The source of truth is the Windows path (git repo); builds happen in $HOME so cargo and
# colcon do not pay the /mnt/d filesystem penalty. Build products never flow back.
#
#   bash scripts/sync_to_wsl.sh            # from anywhere inside WSL
#
# `--delete` means anything that exists only in the mirror is removed. That cost three sets
# of runs before this script was fixed: run output is written by scripts executing in the
# mirror, so until it had been copied to the Windows checkout the next sync deleted it. On
# 2026-10-10 a sync between a measurement and the integration test erased both a 6270-row
# evaluation and a partial test log that way, and the MCL measurement runs were lost the
# same way again.
#
# Two changes close that window, and both are in place:
#
#  1. evidence/ and benchmarks/ are excluded below, so --delete cannot touch run output in
#     the mirror no matter when the sync happens.
#  2. scripts/evidence_root.sh resolves EVIDENCE_ROOT to the evidence/ directory of the
#     WINDOWS checkout, so the run scripts write their results across the mount as they
#     produce them and there is no mirror-only copy to lose in the first place.
#
# Because of (1) this script no longer makes evidence/ in the mirror match the Windows side.
# Treat the Windows checkout as the only place evidence is read from; the mirror's
# evidence/ is whatever happened to be written there and is not authoritative.
set -e
SRC="${SRC:-/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/}"
DST="${DST:-$HOME/ros2-rust-robotics/}"
mkdir -p "$DST"
rsync -a --delete \
    --exclude '.git/' \
    --exclude 'target/' \
    --exclude 'ros2_ws/build/' \
    --exclude 'ros2_ws/install/' \
    --exclude 'ros2_ws/build_release/' \
    --exclude 'ros2_ws/install_release/' \
    --exclude 'ros2_ws/log/' \
    --exclude 'ros2_ws/.cargo/' \
    --exclude 'evidence/' \
    --exclude 'benchmarks/' \
    "$SRC" "$DST"
echo "synced $SRC -> $DST"
