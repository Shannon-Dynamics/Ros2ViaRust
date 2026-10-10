#!/usr/bin/env bash
# The two guards a Phase 3 run is protected by, as functions, so that the run script and
# the integration test use the same code rather than two copies that can drift.
#
#   source scripts/phase3_guards.sh
#
# Both exist because of the same failure: on 2026-10-06 a run produced 588 rows of a
# complete evaluation in which the robot had never moved, and nothing in the output said
# so. The resource manager inside the Webots driver had initialized from a URDF without the
# `ros2_control` block, both controllers stayed unconfigured, /cmd_vel was accepted by
# nothing, and every topic kept publishing exactly as it does on a healthy run. The run is
# kept as evidence/phase-3-correctness/2026-10-06.

# True when both ros2_control controllers the robot needs are active. The topics existing
# proves nothing: they publish whether or not a command interface was ever claimed.
controllers_active() {
    local listing
    listing=$(ros2 control list_controllers 2>/dev/null) || return 1
    controllers_active_in "$listing"
}

# The predicate of controllers_active, on a listing given as text. Split out so the
# integration test can check it against listings that are not the live one.
controllers_active_in() {
    local listing=$1
    grep -q "diffdrive_controller.*active" <<< "$listing" &&
        grep -q "joint_state_broadcaster.*active" <<< "$listing"
}

# Largest distance the ground truth ever reached from its pose in the first row, in metres,
# printed to three decimals. gt_x and gt_y are columns 3 and 4 of the evaluator CSV.
ground_truth_displacement() {
    local csv=$1
    [ -s "$csv" ] || { echo "0.000"; return 1; }
    awk -F, 'NR==2 {x0=$3; y0=$4}
             NR>1 {d=sqrt(($3-x0)^2 + ($4-y0)^2); if (d>m) m=d}
             END {printf "%.3f", m+0}' "$csv"
}

# Smallest travel a real run has to show. Every route in sim/webots/config is several
# metres long, so anything under this means the robot stayed where it was spawned.
PHASE3_MIN_DISPLACEMENT_M=0.2

# True when the run described by this CSV actually moved.
ground_truth_moved() {
    local moved
    moved=$(ground_truth_displacement "$1") || return 1
    awk "BEGIN {exit !($moved >= $PHASE3_MIN_DISPLACEMENT_M)}"
}
