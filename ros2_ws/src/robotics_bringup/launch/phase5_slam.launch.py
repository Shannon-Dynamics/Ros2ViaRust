#!/usr/bin/env python3
"""Phase 5 SLAM, the two nodes in one launch (no simulator in this file).

    ros2 launch robotics_bringup phase5_slam.launch.py replay_file:=/path/to/scans.csv
    ros2 launch robotics_bringup phase5_slam.launch.py replay_file:=scans.csv capture:=true output_dir:=/tmp/run
    ros2 launch robotics_bringup phase5_slam.launch.py source:=live
    ros2 launch robotics_bringup phase5_slam.launch.py source:=live loop_closure:=true

What it starts:

    scan_replayer      (source:=replay, harness)   a recorded scan stream -> /scan, then exits 0
    scan_matcher_node  LiDAR odometry              /scan -> registration + scan_matched
    pose_graph_node    keyframes, odometry edges   registration + scan_matched -> slam/pose, slam/path

With `source:=replay` the launch ends when the replayer has finished its stream: its exit shuts the
other two down, which flushes their CSV files. The replayer ignores the ground-truth pose columns of
the file; nothing on the graph reads ground truth.

What `pose_graph_node` does beyond the keyframe chain (implementation.md sections 8.6 and 8.7):

  * **loop closure**, only with `loop_closure:=true`. The default is off, and then it reports
    `loop_closure: unsupported` exactly as before the argument existed. Whether it should be the
    default is the owner decision of problem.md section 5.
  * **`/map`** (`publish_map:=true`, the default): a replay of every keyframe's scan at its optimized
    pose into a fresh Phase 4 grid (DS17), after each loop closure that changed the graph and once at
    the end of the run.
  * **`map -> odom`** (`publish_map_odom:=true`, the default): `pose_graph_node` owns that edge. The
    `map` frame is the first keyframe's sensor pose. It needs `odom -> <scan frame>` in TF: from the
    driver in a live run; from the replayer's stationary odometry (`odom_tf:=identity`, test
    scaffolding, the default for a replay that owns `map -> odom`) in a replay. Never start it with
    `ground_truth_tf_adapter` or `localization_node`: they own the same edge.
  * `publish_map:=false publish_map_odom:=false` gives back the node as it was before: no `/map`, no
    TF, poses in `slam_origin`.

What this file does not start: a simulator. `source:=live` starts only the SLAM nodes and expects a
driver (Webots, started on its own) to publish `scan` and `odom -> base_link`.

Paths are found relative to the repository root, as in phase4_mapping.launch.py; pass
`repo:=/path/to/ros2-rust-robotics` when the package is installed anywhere else.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

SHARE = get_package_share_directory('robotics_bringup')
DEFAULT_REPO = os.path.normpath(os.path.join(SHARE, *([os.pardir] * 5)))

ARGS = {
    'repo': (DEFAULT_REPO, 'repository root'),
    'source': ('replay', 'replay: start scan_replayer on replay_file | live: only the two nodes, /scan and odom -> base_link come from a driver started elsewhere'),
    'replay_file': ('', 'recorded scan stream (CSV in the integrated-scans.csv format); required for source:=replay'),
    'rate_hz': ('5.0', 'replay: scans per second (5 is real time for the Webots lidar)'),
    'frame_id': ('LDS-01', 'replay: frame_id of the published scans'),
    'lockstep': ('false', 'replay: publish a scan only after the previous one was registered, so that no scan is dropped whatever the build or the machine (tests); false: as lossy as a real driver'),
    'max_scans': ('0', 'replay: publish only the first this many scans (0: all)'),
    'linger_s': ('3.0', 'replay: stay up this long after the last scan before exiting'),
    'scan_topic': ('scan', 'laser scan topic'),
    'loop_closure': ('false', 'pose graph: run the loop detector and verifier and optimize on an accepted loop. false: loop_closure: unsupported'),
    'publish_map': ('true', 'pose graph: render and publish /map (after a loop closure and at the end of the run)'),
    'publish_map_odom': ('true', 'pose graph: own and publish the map -> odom edge (never with another owner of it)'),
    'odom_tf': ('auto', 'replay: none | identity (stationary odometry, test scaffolding) | auto: identity when publish_map_odom is true, else none'),
    'scan_time_offset': ('-0.06', 'pose graph: seconds added to a scan stamp to get its measurement instant, where odom -> sensor is looked up (the Webots lidar: -0.06)'),
    'map_beam_layout': ('pixel_centres', 'pose graph: bearings of the beams in the map render, published | pixel_centres. pixel_centres is the Webots lidar (Phase 4 frozen); it refuses a scan whose bearings are not symmetric about the x axis, as Rusty\'s are: use published for a synthetic stream'),
    'map_render_period': ('0.0', 'pose graph: also render /map every this many seconds while keyframes are added (0: only after a loop closure and at the end)'),
    'capture': ('false', 'nodes write registrations.csv, keyframes.csv, keyframes_final.csv, loops.csv and final-map.txt to output_dir'),
    'output_dir': ('', 'directory capture:=true writes to'),
    'info_inflation': ('', 'scan matcher: divides the ICP information matrix; empty keeps the node default, frozen at 1.5 on 2026-10-11 (docs/phase-5-slam/benchmark.md section 6)'),
    'keyframe_translation': ('', 'pose graph: keyframe after this much translation (m); empty keeps the default'),
    'keyframe_rotation': ('', 'pose graph: keyframe after this much rotation (rad); empty keeps the default'),
}


def setup(context):
    lc = context.launch_configurations
    truthy = lambda name: lc[name].lower() == 'true'
    if lc['source'] not in ('replay', 'live'):
        raise RuntimeError(f"source:= must be replay or live, not {lc['source']!r}")
    if lc['source'] == 'replay' and not os.path.isfile(lc['replay_file']):
        raise RuntimeError(
            f"source:=replay needs replay_file:= to be an existing CSV, got {lc['replay_file']!r}")
    if truthy('capture') and not lc['output_dir']:
        raise RuntimeError('capture:=true requires output_dir:=')
    out = lc['output_dir']
    if truthy('capture'):
        os.makedirs(out, exist_ok=True)

    matcher_params = {'topics.scan': lc['scan_topic']}
    graph_params = {
        'loop_closure.enabled': truthy('loop_closure'),
        'map.enabled': truthy('publish_map'),
        'tf.publish_map_odom': truthy('publish_map_odom'),
        'scan_time_offset_s': float(lc['scan_time_offset']),
        'map.render_period_s': float(lc['map_render_period']),
        'map.beam_layout': lc['map_beam_layout'],
    }
    if lc['odom_tf'] not in ('auto', 'none', 'identity'):
        raise RuntimeError(f"odom_tf:= must be auto, none or identity, not {lc['odom_tf']!r}")
    odom_tf = lc['odom_tf']
    if odom_tf == 'auto':
        odom_tf = 'identity' if truthy('publish_map_odom') else 'none'
    if lc['info_inflation']:
        matcher_params['icp.info_inflation'] = float(lc['info_inflation'])
    if lc['keyframe_translation']:
        graph_params['keyframe.translation_m'] = float(lc['keyframe_translation'])
    if lc['keyframe_rotation']:
        graph_params['keyframe.rotation_rad'] = float(lc['keyframe_rotation'])
    if truthy('capture'):
        matcher_params['output.registrations_csv'] = os.path.join(out, 'registrations.csv')
        graph_params['output.keyframes_csv'] = os.path.join(out, 'keyframes.csv')
        graph_params['output.keyframes_final_csv'] = os.path.join(out, 'keyframes_final.csv')
        graph_params['output.loops_csv'] = os.path.join(out, 'loops.csv')
        if truthy('publish_map'):
            graph_params['output.final_map'] = os.path.join(out, 'final-map.txt')

    actions = [
        LogInfo(msg=f'phase 5 SLAM: source {lc["source"]}; loop_closure: '
                    f'{"enabled" if truthy("loop_closure") else "unsupported"}; '
                    f'/map {"published" if truthy("publish_map") else "off"}; '
                    f'map -> odom {"owned by pose_graph_node" if truthy("publish_map_odom") else "not published"}'),
    ]
    if lc['source'] == 'live':
        actions.append(LogInfo(
            msg='source:=live expects a driver publishing /scan and, for map -> odom, odom -> base_link'))
    # The sinks first, the source last, so that the replayer finds its subscriber.
    actions.append(Node(
        package='robotics_slam_ros', executable='pose_graph_node', name='pose_graph_node',
        output='screen', sigterm_timeout='30', sigkill_timeout='15', parameters=[graph_params]))
    actions.append(Node(
        package='robotics_slam_ros', executable='scan_matcher_node', name='scan_matcher_node',
        output='screen', sigterm_timeout='30', sigkill_timeout='15', parameters=[matcher_params]))
    if lc['source'] == 'replay':
        replayer = Node(
            package='robotics_slam_ros', executable='scan_replayer', name='scan_replayer',
            output='screen',
            parameters=[{'file': lc['replay_file'], 'topics.scan': lc['scan_topic'],
                         'frame_id': lc['frame_id'], 'rate_hz': float(lc['rate_hz']),
                         'max_scans': int(lc['max_scans']), 'linger_s': float(lc['linger_s']),
                         'wait_subscribers': 1, 'lockstep': truthy('lockstep'),
                         'odom_tf': odom_tf, 'odom_tf_time_offset_s': float(lc['scan_time_offset'])}])
        actions.append(replayer)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=replayer,
            on_exit=[LogInfo(msg='scan_replayer finished: shutting the SLAM nodes down'),
                     EmitEvent(event=Shutdown(reason='the replayed stream ended'))])))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, (default, description) in ARGS.items()]
        + [OpaqueFunction(function=setup)]
    )
