#!/bin/bash
# Copy web/ from the Windows checkout into the Linux filesystem for building (run in WSL).
#
# node_modules and Next's cache are tens of thousands of small files; on /mnt/d every one of
# them is a cross-filesystem round trip. So the editable source stays on Windows (the source
# of truth, like the rest of the repo) and this script copies it to ~/rvr-web, where
# `npm ci`, `npm run build` and the checks run. One direction only: nothing flows back.
#
# `--delete` is safe here and only here: the destination holds nothing but this copy and its
# own build products, and those (node_modules, .next, out, generated wasm) are excluded, so
# --delete never touches them. Never point this at a directory that holds anything else.
set -euo pipefail
SRC="${SRC:-/mnt/d/Shannon/ROS2 via RUST/ros2-rust-robotics/web/}"
DST="${DST:-$HOME/rvr-web/}"
mkdir -p "$DST"
rsync -a --delete \
  --exclude 'node_modules/' --exclude '.next/' --exclude 'out/' --exclude '.source/' \
  --exclude 'lib/wasm/pkg/' --exclude 'public/wasm/' --exclude 'public/data/' --exclude '.checks/' --exclude 'next-env.d.ts' \
  "$SRC" "$DST"
echo "synced $SRC -> $DST"
