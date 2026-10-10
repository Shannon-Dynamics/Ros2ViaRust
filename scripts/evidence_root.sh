#!/bin/bash
# Source this file (do not execute it) to get EVIDENCE_ROOT: the directory every run script
# writes its evidence into.
#
#   source "$SCRIPT_DIR/evidence_root.sh"
#   mkdir -p "$EVIDENCE_ROOT/phase-3-correctness/$(date +%Y-%m-%d)"
#
# Why this file exists, and it is not a style preference. Builds and runs happen in the WSL
# mirror ($HOME/ros2-rust-robotics) while the git repository lives on the Windows side, and
# scripts/wsl_run.sh calls scripts/sync_to_wsl.sh (rsync -a --delete, Windows -> WSL) at the
# start of every run. Any file that exists only in the mirror is deleted by that sync. Run
# output is written in the mirror, so between a run finishing and its evidence being copied
# to the Windows checkout there was a window in which the next sync destroyed it. That
# window cost three sets of runs: a 6270-row evaluation and a partial integration-test log
# on 2026-10-10, and the MCL measurement runs noted in
# docs/phase-3-localization/implementation.md.
#
# Two things close it, and both are applied rather than one:
#
#  1. This file resolves EVIDENCE_ROOT to the evidence/ directory of the WINDOWS checkout
#     whenever that checkout is mounted, so a run writes its results across the mount as it
#     produces them. There is then no moment at which the only copy lives in the mirror, so
#     no copy-back step to forget and no window to lose.
#  2. scripts/sync_to_wsl.sh excludes evidence/ and benchmarks/ from the rsync, so even a
#     run that did write into the mirror (no mount, or an explicit output path) cannot have
#     its output deleted by the next sync.
#
# Override by setting EVIDENCE_ROOT before sourcing. Set it to "$REPO/evidence" to get the
# old in-mirror behaviour back deliberately.
#
# Evidence is small text (CSV, JSON, YAML, logs), so writing it over the /mnt/d filesystem
# costs nothing that matters; the reason builds stay in the mirror is cargo's and colcon's
# many-small-file churn, which this is not.

# shellcheck disable=SC2034
_EVIDENCE_WINDOWS_CHECKOUT="${EVIDENCE_WINDOWS_CHECKOUT:-/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics}"

if [ -z "${EVIDENCE_ROOT:-}" ]; then
    _er_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    if [ -d "$_EVIDENCE_WINDOWS_CHECKOUT/.git" ] && [ -w "$_EVIDENCE_WINDOWS_CHECKOUT" ]; then
        EVIDENCE_ROOT="$_EVIDENCE_WINDOWS_CHECKOUT/evidence"
    else
        EVIDENCE_ROOT="$_er_repo/evidence"
        echo "evidence_root.sh: the Windows checkout is not writable at" \
             "$_EVIDENCE_WINDOWS_CHECKOUT, writing evidence into the mirror at" \
             "$EVIDENCE_ROOT instead; copy it out before the next sync" >&2
    fi
    unset _er_repo
fi
export EVIDENCE_ROOT
