#!/usr/bin/env python3
"""Launch the Phase 3 localization world with the official TurtleBot3 Webots driver.

What this file does and does not do:

* It starts Webots on ``sim/webots/worlds/turtlebot3_localization_room.wbt`` and attaches
  the driver of the ``webots_ros2_turtlebot`` package to the robot, using that package's
  own URDF (``resource/turtlebot_webots.urdf``) and ros2_control parameters
  (``resource/ros2control.yml``) unchanged. Nothing about the driver is reimplemented or
  wrapped here, and no Rust node is started: the point is that the standard topics
  (/scan, /odom, /imu, /joint_states, /clock, /tf) exist, so any Rust consumer node can
  subscribe to them without the driver knowing about it.
* It does not start localization, mapping, planning or control. The Rust
  ``localization_node`` of Phase 3 is launched separately and only reads those topics.

Why Webots is started with ExecuteProcess instead of ``WebotsLauncher``: on a WSL2 kernel
``webots_ros2_driver.utils.is_wsl()`` is true, so ``WebotsLauncher`` and
``WebotsController`` assume Webots runs on the Windows side, pass the world as a
``\\\\wsl.localhost\\...`` path and connect the controllers over TCP to the Windows host.
Webots is installed natively in Linux here, so the world path has to stay a Linux path and
the controllers connect over the local IPC protocol. Everything else, including the
``webots-controller`` launcher script, the Ros2Supervisor and the driver itself, is the
official one. See ../README.md for the setup this assumes.

Usage:

    ros2 launch sim/webots/launch/localization_world.launch.py
    ros2 launch sim/webots/launch/localization_world.launch.py gui:=false mode:=fast

This file is runnable by path, so ``sim/webots`` does not have to be a ROS 2 package.
"""

import os
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
WORLD = os.path.normpath(os.path.join(THIS_DIR, os.pardir, 'worlds',
                                      'turtlebot3_localization_room.wbt'))

ARGS = {
    'gui': ('true', 'false: minimize the Webots window and switch rendering off'),
    'mode': ('realtime', 'Webots mode: realtime | fast | pause'),
    'port': ('1234', 'Webots controller port the extern controllers connect to'),
    'use_sim_time': ('true', 'drive the driver from the simulator clock on /clock'),
    'ground_truth': ('false', 'true: publish /ground_truth/pose and accept /kidnap/teleport'),
    'ground_truth_rate_hz': ('50', 'publish rate of the ground truth pose'),
    'record': ('false', 'true: record the 3D view to record_file, needs gui:=true'),
    'record_file': ('', 'output movie path, required when record:=true'),
    'record_seconds': ('40', 'simulated seconds to record'),
    'record_start_delay': ('6', 'simulated seconds to wait before recording starts'),
    'record_status_file': ('', 'JSON status written when encoding finished, optional'),
}


def setup(context):
    lc = context.launch_configurations
    port = lc['port']
    turtlebot_dir = get_package_share_directory('webots_ros2_turtlebot')
    driver_share = get_package_share_directory('webots_ros2_driver')
    driver_prefix = get_package_prefix('webots_ros2_driver')
    robot_description = os.path.join(turtlebot_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(turtlebot_dir, 'resource', 'ros2control.yml')

    webots_cmd = ['webots', '--batch', f'--port={port}', f'--mode={lc["mode"]}']
    if lc['gui'].lower() != 'true':
        webots_cmd += ['--minimize', '--no-rendering', '--stdout', '--stderr']
    webots = ExecuteProcess(cmd=webots_cmd + [WORLD], output='screen', name='webots')

    # Official supervisor controller: publishes /clock and offers the spawn and remove
    # services. It needs WEBOTS_HOME pointing at the driver prefix, as upstream does.
    supervisor = Node(
        package='webots_ros2_driver',
        executable='ros2_supervisor.py',
        namespace='Ros2Supervisor',
        remappings=[('/Ros2Supervisor/clock', '/clock')],
        output='screen',
        respawn=True,
        additional_env={
            'WEBOTS_CONTROLLER_URL': f'ipc://{port}/Ros2Supervisor',
            'WEBOTS_HOME': driver_prefix,
        },
    )

    # The TurtleBot3 driver, started through the official webots-controller script.
    # ROS 2 Jazzy: diff_drive_controller takes TwistStamped on
    # /diffdrive_controller/cmd_vel; both topics are remapped to the conventional names.
    driver = ExecuteProcess(
        cmd=[
            os.path.join(driver_share, 'scripts', 'webots-controller'),
            '--robot-name=TurtleBot3Burger',
            '--protocol=ipc',
            f'--port={port}',
            'ros2', '--ros-args',
            '-r', '/diffdrive_controller/cmd_vel:=/cmd_vel',
            '-r', '/diffdrive_controller/odom:=/odom',
            '-p', f'robot_description:={robot_description}',
            '-p', f'use_sim_time:={lc["use_sim_time"]}',
            '-p', 'set_robot_state_publisher:=true',
            '--params-file', ros2_control_params,
        ],
        additional_env={'WEBOTS_HOME': driver_prefix},
        output='screen',
        name='driver_turtlebot3',
        respawn=True,
    )

    # As in the official example launch file: the driver was told
    # set_robot_state_publisher:=true, so it hands the robot description generated from
    # the Webots robot to this node, which then publishes the robot's own frames
    # (base_link, LDS-01, imu_link, wheels) on /tf and /tf_static.
    #
    # The description below is only the placeholder this node starts with. Upstream uses an
    # empty `<robot name=""><link name=""/></robot>` for it, and on a loaded machine that
    # empty description is a race the run can lose: the controller_manager inside the driver
    # initializes its resource manager from whichever /robot_description reaches it first,
    # and when the empty one wins it fails with "URDF cannot be parsed by a `ros2_control`
    # component parser: no 'ros2_control' tag found in the URDF", then refuses the real one
    # ("ResourceManager has already loaded a urdf and is initialized"). The run that follows
    # has no command interfaces at all: both controllers stay unconfigured, /cmd_vel is
    # accepted by nothing, and the robot stands at the origin for the whole route while
    # every topic keeps publishing. That is a silent failure, not a loud one, and it
    # produced a full Phase 3 evaluation of a stationary robot on 2026-10-06
    # (../../evidence/phase-3-correctness/2026-10-06, kept as the example).
    #
    # So the placeholder is the official URDF itself, with the one empty link that
    # robot_state_publisher needs in order to parse it. Whichever description initializes
    # the resource manager now carries the same `ros2_control` block, and the driver still
    # replaces this one as soon as it is up.
    with open(robot_description) as urdf_file:
        placeholder_description = urdf_file.read().replace(
            '</robot>', '  <link name="base_link"/>\n</robot>')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': placeholder_description,
                     'use_sim_time': lc['use_sim_time'].lower() == 'true'}],
    )

    # Static leaf of the TF tree, as in the official example launch file.
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
    )

    # ros2_control spawners: the differential drive controller that publishes /odom and
    # consumes /cmd_vel, and the broadcaster that publishes /joint_states.
    timeout = ['--controller-manager-timeout', '60']
    spawners = [
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['diffdrive_controller'] + timeout),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster'] + timeout),
    ]

    # Optional evaluation supervisor: reads the true robot pose out of the scene tree and
    # teleports the robot on request. For evaluation and for the kidnapped robot
    # experiment only; no estimator subscribes to what it publishes.
    evaluation = []
    if lc['ground_truth'].lower() == 'true':
        webots_home = os.environ.get('WEBOTS_HOME', '/usr/local/webots')
        evaluation = [ExecuteProcess(
            cmd=[os.path.join(webots_home, 'webots-controller'),
                 '--robot-name=evaluation_supervisor', '--protocol=ipc', f'--port={port}',
                 os.path.join(THIS_DIR, os.pardir, 'scripts', 'evaluation_supervisor.py')],
            additional_env={
                'WEBOTS_HOME': webots_home,
                'PYTHONUNBUFFERED': '1',
                'GROUND_TRUTH_RATE_HZ': lc['ground_truth_rate_hz'],
            },
            output='screen',
            name='evaluation_supervisor',
            respawn=True,
        )]

    # Optional movie recorder: a second extern supervisor, started with the Webots
    # controller launcher of the Webots installation itself (not the driver's), attached to
    # the devices-free `movie_recorder` robot of the world. The official driver and the
    # official Ros2Supervisor are untouched by it.
    recorder = []
    if lc['record'].lower() == 'true':
        if not lc['record_file']:
            raise RuntimeError('record:=true requires record_file:=<path to the movie>')
        if lc['gui'].lower() != 'true':
            raise RuntimeError('record:=true requires gui:=true, recording renders the 3D view')
        webots_home = os.environ.get('WEBOTS_HOME', '/usr/local/webots')
        env = {
            'WEBOTS_HOME': webots_home,
            'PYTHONUNBUFFERED': '1',
            'MOVIE_FILE': lc['record_file'],
            'MOVIE_SECONDS': lc['record_seconds'],
            'MOVIE_START_DELAY': lc['record_start_delay'],
        }
        if lc['record_status_file']:
            env['MOVIE_STATUS_FILE'] = lc['record_status_file']
        recorder = [ExecuteProcess(
            cmd=[os.path.join(webots_home, 'webots-controller'),
                 '--robot-name=movie_recorder', '--protocol=ipc', f'--port={port}',
                 os.path.join(THIS_DIR, os.pardir, 'scripts', 'movie_recorder.py')],
            additional_env=env,
            output='screen',
            name='movie_recorder',
        )]

    # Ending the launch has to end Webots, and the process launch signals is not the one
    # that has to die: `webots` on PATH is a shell wrapper that starts `webots-bin` as a
    # child. A surviving webots-bin holds port 1234, and the next run then starts a second
    # Webots on port 1235 whose controllers connect to nothing, which is the failure the
    # first operational note of ../README.md describes.
    #
    # On SIGTERM `webots-bin` has been seen both to exit and to need SIGKILL, so this
    # escalates and prints which of the two ended it. The launch log of every run therefore
    # says what happened rather than leaving it to be guessed.
    #
    # Honest note on the evidence: the three runs of 2026-10-06 that look like proof of all
    # this (`shutdown.txt` in ../evidence/, "still alive 90 s after SIGINT") prove nothing
    # about the launch. Their SIGINT never arrived: the run scripts sent it to a background
    # job of a non-interactive shell, where SIGINT is SIG_IGN and CPython keeps it that way,
    # so the launch was never asked to shut down and this handler never ran. That is fixed
    # in the scripts with `set -m`; see the comment there. What remains true without any
    # measurement is the process tree: the wrapper's child is not the launch's child.
    #
    # Matching on the port leaves a Webots started for anything else alone, and SIGKILL
    # loses nothing: `--batch` already told Webots not to save anything on exit.
    #
    # The pair of lines appears twice in a launch log of a normal run, because Shutdown is
    # emitted twice: once by the signal, and once by the OnProcessExit handler above when
    # Webots is gone. `pkill` on a pattern that matches nothing is a no-op, so this is
    # harmless.
    def stop_webots(_context, *_args, **_kwargs):
        pattern = f'webots-bin .*--port={port}'
        alive = lambda: subprocess.run(['pgrep', '-f', pattern],
                                       stdout=subprocess.DEVNULL).returncode == 0
        print(f'[launch] shutdown: stopping {pattern}', flush=True)
        subprocess.run(['pkill', '-f', pattern], check=False)
        for _ in range(10):
            if not alive():
                print('[launch] shutdown: webots exited on SIGTERM', flush=True)
                return []
            time.sleep(0.5)
        print('[launch] shutdown: webots ignored SIGTERM, sending SIGKILL', flush=True)
        subprocess.run(['pkill', '-KILL', '-f', pattern], check=False)
        return []

    return [
        LogInfo(msg=f'world {WORLD}'),
        webots,
        RegisterEventHandler(OnProcessExit(target_action=webots,
                                           on_exit=[EmitEvent(event=Shutdown())])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=stop_webots)])),
        supervisor,
        driver,
        robot_state_publisher,
        footprint_publisher,
        *spawners,
        *evaluation,
        *recorder,
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, (default, description) in ARGS.items()]
        + [OpaqueFunction(function=setup)]
    )
