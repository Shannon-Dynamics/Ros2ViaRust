#!/usr/bin/env python3
"""Phase 4 mapping, the whole stack in one launch.

    ros2 launch robotics_bringup phase4_mapping.launch.py
    ros2 launch robotics_bringup phase4_mapping.launch.py drive:=true evaluate:=true output_dir:=/tmp/run

What it starts, and who owns what:

    Webots + official webots_ros2_turtlebot driver   /scan /odom /imu /joint_states /clock
                                                     TF odom -> base_link, base_link -> *
    evaluation_supervisor.py  (ground truth)         /ground_truth/pose
    ground_truth_tf_adapter   SIMULATION ONLY        TF map -> odom, from ground truth
    mapping_node              THE MAPPER             /map /esdf /map_entropy, TF in only
    mapping_evaluator         optional, harness      grades /map against the analytic world
    route_driver              optional, harness      /ground_truth/pose -> /cmd_vel

TF ownership, exactly one owner per edge:

    map -> odom           ground_truth_tf_adapter   (owner:=ground_truth) or
                          localization_node         (owner:=ekf, the Phase 3 EKF)
    odom -> base_link     diffdrive_controller      (/tf, inside the Webots driver)
    base_link -> *        robot_state_publisher     (/tf, /tf_static)

`mapping_node` has no pose-source parameter, reads no ground truth and owns no TF edge:
which pose it maps with is decided here, by which node is started to own `map -> odom`, and
the run records that choice. `owner:=ground_truth` is simulation only; `owner:=ekf` is claim
M5 and is not run until MCL's route to `map -> odom` (decision D3) is settled, because M5
reports one run per estimator and the choice of estimator is a Phase 3 outcome.

Readers of `/ground_truth/pose`: the supervisor publishes it; `ground_truth_tf_adapter`,
`mapping_evaluator` and `route_driver` read it. `mapping_node` does not.

Paths are found relative to the repository root, as in phase3_localization.launch.py; pass
`repo:=/path/to/ros2-rust-robotics` when the package is installed anywhere else.
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
    'gui': ('true', 'false: minimize the Webots window and switch rendering off'),
    'mode': ('realtime', 'Webots mode: realtime | fast | pause'),
    'port': ('1234', 'Webots controller port the extern controllers connect to'),
    'map_file': ('', 'ground truth of the world; default <repo>/sim/webots/config/landmarks.yaml'),
    'route_file': ('', 'route for drive:=true; default <repo>/sim/webots/config/route_long.yaml'),
    'owner': ('ground_truth', 'who owns map -> odom: ground_truth (simulation only) | ekf'),
    'drive': ('false', 'start route_driver and drive route_file'),
    'laps': ('1', 'how many times route_driver drives the route'),
    'evaluate': ('false', 'start mapping_evaluator; needs output_dir'),
    'output_dir': ('', 'directory the evaluator and the mapper write their files to'),
    'capture': ('false', 'mapper writes latency, trace, replay record and final grid to output_dir'),
    'tau_cells': ('1.0', 'evaluation tolerance in cells; scripts/phase4_verdict.py checks it'),
    'no_return': ('clear_to_max_range', 'mapper policy for beams with no return'),
    'tracer': ('supercover', 'mapper ray traversal: supercover | bresenham'),
    'clear_margin': ('0.10', 'mapper: a no-return beam clears up to range_max less this (m)'),
}


def setup(context):
    lc = context.launch_configurations
    repo = lc['repo']
    world_launch = os.path.join(repo, 'sim', 'webots', 'launch',
                                'localization_world.launch.py')
    map_file = lc['map_file'] or os.path.join(repo, 'sim', 'webots', 'config',
                                              'landmarks.yaml')
    route_file = lc['route_file'] or os.path.join(repo, 'sim', 'webots', 'config',
                                                  'route_long.yaml')
    truthy = lambda name: lc[name].lower() == 'true'
    missing = [p for p in (world_launch, map_file) if not os.path.isfile(p)]
    if truthy('drive') and not os.path.isfile(route_file):
        missing.append(route_file)
    if missing:
        raise RuntimeError(
            'phase4_mapping.launch.py cannot find:\n  ' + '\n  '.join(missing)
            + f'\nrepo was resolved to {repo!r}. Pass repo:=/path/to/ros2-rust-robotics.')
    if lc['owner'] not in ('ground_truth', 'ekf'):
        raise RuntimeError(f"owner:= must be ground_truth or ekf, not {lc['owner']!r}")
    if (truthy('evaluate') or truthy('capture')) and not lc['output_dir']:
        raise RuntimeError('evaluate:=true and capture:=true require output_dir:=')
    out = lc['output_dir']

    actions = [
        LogInfo(msg=f'phase 4 bringup: repo {repo}, map -> odom owner: {lc["owner"]}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={
                'gui': lc['gui'],
                'mode': lc['mode'],
                'port': lc['port'],
                # The ground truth topic exists for the adapter, the evaluator and the route
                # driver. The mapper does not subscribe to it.
                'ground_truth': 'true',
            }.items(),
        ),
    ]

    if lc['owner'] == 'ground_truth':
        actions.append(Node(
            package='robotics_mapping_ros', executable='ground_truth_tf_adapter',
            name='ground_truth_tf_adapter', output='screen'))
    else:
        actions.append(Node(
            package='robotics_localization_bridge', executable='localization_node',
            name='localization_node', output='screen',
            parameters=[{'map_file': map_file, 'topics.odom': '/odom', 'publish_tf': True}]))

    mapper_params = {
        'model.no_return': lc['no_return'],
        'model.tracer': lc['tracer'],
        'model.clear_margin_m': float(lc['clear_margin']),
    }
    if truthy('capture'):
        mapper_params.update({
            'output.latency_csv': os.path.join(out, 'update-latency.csv'),
            'output.trace_csv': os.path.join(out, 'entropy-trace.csv'),
            'output.record_csv': os.path.join(out, 'integrated-scans.csv'),
            'output.final_grid': os.path.join(out, 'final-grid.txt'),
        })
    actions.append(Node(
        package='robotics_mapping_ros', executable='mapping_node', name='mapping_node',
        output='screen', sigterm_timeout='60', sigkill_timeout='30',
        parameters=[mapper_params]))

    if truthy('evaluate'):
        actions.append(Node(
            package='robotics_mapping_ros', executable='mapping_evaluator',
            name='mapping_evaluator', output='screen',
            # The evaluator grades once, at shutdown, and a debug build needs a while for the
            # visibility tests; launch's default of 5 s before SIGKILL would lose the result.
            sigterm_timeout='300', sigkill_timeout='30',
            parameters=[{'map_file': map_file, 'output_dir': out,
                         'tau_cells': float(lc['tau_cells'])}]))

    if truthy('drive'):
        actions.append(Node(
            package='robotics_localization_bridge', executable='route_driver',
            name='route_driver', output='screen',
            parameters=[{'route_file': route_file, 'laps': int(lc['laps'])}]))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, (default, description) in ARGS.items()]
        + [OpaqueFunction(function=setup)]
    )
