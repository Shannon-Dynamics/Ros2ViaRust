#!/bin/bash
# Build the ROS 2 Jazzy interface underlay with Rust bindings.
#
# rclrs 0.8 obtains message crates through the `ros-env` crate, which includes every Rust
# crate that `rosidl_generator_rs` generated into a sourced AMENT_PREFIX_PATH. The binary
# ROS 2 distribution does not ship those crates, so the standard interface packages must
# be rebuilt once, with the Rust generator present, into an underlay workspace.
#
# Usage:  scripts/setup_interfaces_underlay.sh [workspace_dir]     (default ~/ros2_rust_interfaces_ws)
# Needs:  ROS 2 Jazzy, cargo, colcon-cargo, colcon-ros-cargo, cargo-ament-build, vcstool, git
set -e
set +u
WS="${1:-$HOME/ros2_rust_interfaces_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "$WS/src"
cd "$WS"

if [ ! -d src/ros2-rust/rosidl_rust ]; then
    echo ">>> importing interface repositories into $WS/src"
    vcs import src < "$HERE/interfaces.repos"
fi

# Python-only package inside common_interfaces that is not needed and slows the build.
mkdir -p src/ros2/common_interfaces/sensor_msgs_py
touch src/ros2/common_interfaces/sensor_msgs_py/COLCON_IGNORE

# Keep only tf2_msgs from geometry2 (the rest is C++ and already installed by apt).
if [ -d src/ros2/geometry2 ]; then
    for d in src/ros2/geometry2/*/; do
        case "$(basename "$d")" in
            tf2_msgs) ;;
            *) touch "$d/COLCON_IGNORE" ;;
        esac
    done
fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
source "$HOME/.cargo/env"

echo ">>> building interface underlay (this takes several minutes)"
# rcl_interfaces and rosgraph_msgs are required by rclrs itself (parameter services, rosout).
colcon build \
    --packages-up-to rcl_interfaces rosgraph_msgs std_msgs geometry_msgs sensor_msgs nav_msgs tf2_msgs diagnostic_msgs visualization_msgs example_interfaces \
    --cmake-args -DBUILD_TESTING=OFF \
    --event-handlers console_cohesion+ desktop_notification- status-

echo ">>> Rust crates generated:"
ls -d install/*/share/*/rust
echo ">>> done. Source $WS/install/setup.bash (scripts/env.sh does this for you)."
