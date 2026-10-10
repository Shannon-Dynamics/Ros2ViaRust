#!/usr/bin/env python3
"""Phase 3 localization, the whole stack in one launch.

    ros2 launch robotics_bringup phase3_localization.launch.py
    ros2 launch robotics_bringup phase3_localization.launch.py drive:=true evaluate:=true

What it starts, and who owns what:

    Webots + official webots_ros2_turtlebot driver   /scan /odom /imu /joint_states /clock
                                                     TF odom -> base_link, base_link -> *
    evaluation_supervisor.py  (ground truth)         /ground_truth/pose, /kidnap/events
                              (teleport)             subscribes /kidnap/teleport
    route_driver              optional, harness      /ground_truth/pose -> /cmd_vel
    localization_node         THE EKF STACK UNDER TEST   /localization/pose, TF map -> odom
    mcl_node                  THE MCL STACK UNDER TEST   /mcl/pose, no TF (see mcl_node.rs)
    localization_evaluator    optional, harness, one instance per estimator started

This replaces the combination of `sim/webots/launch/localization_world.launch.py` plus the
three or four `ros2 run` calls that `scripts/run_phase3_correctness.sh` used to make by
hand. The run script now exists only to sequence a measurement (wait for the controllers,
drive, collect, stop and judge); bringing the stack up is this file's job.

TF ownership in this launch, exactly one owner per edge:

    map -> odom           localization_node         (/tf; the correction, REP 105)
    odom -> base_link     diffdrive_controller      (/tf, inside the Webots driver)
    base_link -> *        robot_state_publisher     (/tf, /tf_static)
    base_link -> base_footprint   static_transform_publisher

Three nodes here read `/ground_truth/pose`: the supervisor publishes it, the evaluator
grades against it and the route driver steers by it. `localization_node` does not, and that
is the property the whole evaluation rests on.

Paths: the world, the landmark map and the route live in `sim/webots/`, which is not a ROS
package, so they are found relative to the repository root. The default root is derived from
the install location of this package (`<repo>/ros2_ws/install/robotics_bringup/share/robotics_bringup`,
five levels up). Pass `repo:=/path/to/ros2-rust-robotics` when the package is installed
anywhere else; the launch fails with the expected paths rather than starting half a stack.
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
    'map_file': ('', 'landmark map; default <repo>/sim/webots/config/landmarks.yaml'),
    'route_file': ('', 'route for drive:=true; default <repo>/sim/webots/config/route_long.yaml'),
    'ground_truth': ('true', 'publish /ground_truth/pose and accept /kidnap/teleport'),
    'localize': ('true', 'start localization_node (EKF), one of the stacks under test'),
    'publish_tf': ('true', 'localization_node broadcasts map -> odom'),
    'evaluate': ('false', 'start an evaluator for localization_node; needs output_csv/json'),
    'output_csv': ('', 'per sample rows for evaluate:=true (EKF)'),
    'output_json': ('', 'run summary for evaluate:=true (EKF)'),
    'mcl': ('false', 'start mcl_node, the other stack under test'),
    'mcl_evaluate': ('false', 'start an evaluator for mcl_node; needs mcl_output_csv/json'),
    'mcl_output_csv': ('', 'per sample rows for mcl_evaluate:=true (MCL)'),
    'mcl_output_json': ('', 'run summary for mcl_evaluate:=true (MCL)'),
    'drive': ('false', 'start route_driver and drive route_file'),
    'laps': ('1', 'how many times route_driver drives the route'),
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
    missing = [p for p in (world_launch, map_file) if not os.path.isfile(p)]
    if lc['drive'].lower() == 'true' and not os.path.isfile(route_file):
        missing.append(route_file)
    if missing:
        raise RuntimeError(
            'phase3_localization.launch.py cannot find:\n  '
            + '\n  '.join(missing)
            + f'\nrepo was resolved to {repo!r}. Pass repo:=/path/to/ros2-rust-robotics.')

    truthy = lambda name: lc[name].lower() == 'true'
    # The odometry the filter predicts from and the evaluator grades as the baseline: one
    # topic, read by both, and it is what the simulator publishes. Nothing here degrades it
    # on purpose; docs/phase-3-localization/theory.md section 6 says why it does not need to.
    odom_topic = '/odom'

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
            'gui': lc['gui'],
            'mode': lc['mode'],
            'port': lc['port'],
            'ground_truth': lc['ground_truth'],
        }.items(),
    )

    actions = [
        LogInfo(msg=f'phase 3 bringup: repo {repo}'),
        LogInfo(msg=f'phase 3 bringup: map {map_file}, odometry {odom_topic}'),
        world,
    ]

    if truthy('localize'):
        actions.append(Node(
            package='robotics_localization_bridge',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[{
                'map_file': map_file,
                'topics.odom': odom_topic,
                'publish_tf': truthy('publish_tf'),
            }],
        ))

    if truthy('evaluate'):
        if not lc['output_csv'] or not lc['output_json']:
            raise RuntimeError('evaluate:=true requires output_csv:= and output_json:=')
        actions.append(Node(
            package='robotics_localization_bridge',
            executable='localization_evaluator',
            name='localization_evaluator',
            output='screen',
            parameters=[{
                'topics.odom': odom_topic,
                'topics.estimate': 'localization/pose',
                'output_csv': lc['output_csv'],
                'output_json': lc['output_json'],
            }],
        ))

    if truthy('mcl'):
        actions.append(Node(
            package='robotics_localization_bridge',
            executable='mcl_node',
            name='mcl_node',
            output='screen',
            parameters=[{
                'map_file': map_file,
                'topics.odom': odom_topic,
            }],
        ))

    if truthy('mcl_evaluate'):
        if not lc['mcl_output_csv'] or not lc['mcl_output_json']:
            raise RuntimeError('mcl_evaluate:=true requires mcl_output_csv:= and mcl_output_json:=')
        # A second instance of the same evaluator node, grading mcl_node instead of
        # localization_node; it needs a distinct node name or the two collide in the graph.
        actions.append(Node(
            package='robotics_localization_bridge',
            executable='localization_evaluator',
            name='localization_evaluator_mcl',
            output='screen',
            parameters=[{
                'topics.odom': odom_topic,
                'topics.estimate': 'mcl/pose',
                'output_csv': lc['mcl_output_csv'],
                'output_json': lc['mcl_output_json'],
            }],
        ))

    if truthy('drive'):
        actions.append(Node(
            package='robotics_localization_bridge',
            executable='route_driver',
            name='route_driver',
            output='screen',
            parameters=[{
                'route_file': route_file,
                'laps': int(lc['laps']),
            }],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, (default, description) in ARGS.items()]
        + [OpaqueFunction(function=setup)]
    )
