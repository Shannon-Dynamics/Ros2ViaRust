#!/bin/bash
# Source this file (do not execute it) to get a build/run environment for the framework.
#
#   source scripts/env.sh
#
# It layers, in order: ROS 2 Jazzy, the Rust-bindings interface underlay, the cargo
# toolchain, and this repository's colcon install space if it exists.
# Override INTERFACES_WS to point at a different underlay.

set +u
INTERFACES_WS="${INTERFACES_WS:-$HOME/ros2_rust_interfaces_ws}"
_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "env.sh: /opt/ros/jazzy not found" >&2
fi

if [ -f "$INTERFACES_WS/install/setup.bash" ]; then
    source "$INTERFACES_WS/install/setup.bash"
else
    echo "env.sh: interface underlay not built at $INTERFACES_WS (run scripts/setup_interfaces_underlay.sh)" >&2
fi

[ -f "$HOME/.cargo/env" ] && source "$HOME/.cargo/env"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [ -f "$_HERE/../ros2_ws/install/setup.bash" ]; then
    source "$_HERE/../ros2_ws/install/setup.bash"
fi
unset _HERE
