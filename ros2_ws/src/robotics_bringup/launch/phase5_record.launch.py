#!/usr/bin/env python3
"""Phase 5: record every scan of a Webots run, turning scans included (Stage A of the live validation).

    ros2 launch robotics_bringup phase5_record.launch.py gui:=false output_dir:=/tmp/run

Why this file exists. The recordings Phase 4 made (`integrated-scans.csv`) are what the mapper
integrated, and the mapper skips a scan when the sensor moved more than 5 cm or turned more than
0.015 rad across the 0.2 s its timestamp leaves open (`MotionGate`), and when the sensor has not
moved since the last integrated scan (`MotionFilter`). Both are right for a mapper and wrong for a
SLAM front end: the gate removes exactly the scans taken while the robot turns, so no Phase 4
recording has a turning scan (docs/phase-5-slam/implementation.md 8.6). Here the same mapper node
records with both switched off, so the file has the same schema, the same pose columns (the ground
truth at the measurement instant, `scan_time_offset_s` before the stamp) and every scan.

Nothing else differs from phase4_mapping.launch.py: the same world launch, the same
`ground_truth_tf_adapter` owning `map -> odom` (SIMULATION ONLY, the ground truth is what the
recorded pose columns are), the same mapper binary. The grid the mapper builds is a by-product and
is not evidence of anything here: with the gate off it integrates scans taken during a turn at a
pose that is off by up to the stamp's uncertainty, which is the reason the gate exists.
No route_driver is started here; scripts/run_phase5_record.sh starts it once the controllers are
active.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SHARE = get_package_share_directory('robotics_bringup')
DEFAULT_REPO = os.path.normpath(os.path.join(SHARE, *([os.pardir] * 5)))

ARGS = {
    'repo': (DEFAULT_REPO, 'repository root, which is where sim/webots is looked for'),
    'gui': ('false', 'false: minimize the Webots window and switch rendering off'),
    'mode': ('realtime', 'Webots mode: realtime | fast | pause'),
    'port': ('1234', 'Webots controller port the extern controllers connect to'),
    'output_dir': ('', 'directory integrated-scans.csv is written to (required)'),
    'scan_time_offset': ('-0.06', 'seconds added to a scan stamp to get its measurement instant (the pose columns are the ground truth there)'),
    'beam_layout': ('pixel_centres', 'mapper: bearings of the beams; the recorded ranges are as published whatever this is'),
}


def setup(context):
    lc = context.launch_configurations
    repo = lc['repo']
    world_launch = os.path.join(repo, 'sim', 'webots', 'launch', 'localization_world.launch.py')
    if not os.path.isfile(world_launch):
        raise RuntimeError(f'cannot find {world_launch}; pass repo:=/path/to/ros2-rust-robotics')
    out = lc['output_dir']
    if not out:
        raise RuntimeError('output_dir:= is required')
    os.makedirs(out, exist_ok=True)

    mapper_params = {
        'scan_time_offset_s': float(lc['scan_time_offset']),
        'scan_beam_layout': lc['beam_layout'],
        # The two filters of the mapper, off: every scan the driver publishes is recorded.
        'motion_gate.window_s': 0.0,
        'motion_filter.min_translation_m': 0.0,
        'motion_filter.min_yaw_rad': 0.0,
        'output.record_csv': os.path.join(out, 'integrated-scans.csv'),
        'output.latency_csv': os.path.join(out, 'update-latency.csv'),
    }
    return [
        LogInfo(msg=f'phase 5 recording: repo {repo}, motion gate OFF, motion filter OFF, '
                    'map -> odom owner: ground_truth_tf_adapter'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={'gui': lc['gui'], 'mode': lc['mode'], 'port': lc['port'],
                              'ground_truth': 'true'}.items()),
        Node(package='robotics_mapping_ros', executable='ground_truth_tf_adapter',
             name='ground_truth_tf_adapter', output='screen'),
        Node(package='robotics_mapping_ros', executable='mapping_node', name='mapping_node',
             output='screen', sigterm_timeout='60', sigkill_timeout='30',
             parameters=[mapper_params]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, (default, description) in ARGS.items()]
        + [OpaqueFunction(function=setup)]
    )
