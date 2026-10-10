# Webots simulation

The shared arena for the "ROS via Rust" material ([../../docs/README.md](../../docs/README.md)):
a TurtleBot3 Burger in Webots, driven by the official Cyberbotics and ROBOTIS packages, on
which both the Rust nodes of this repository and the reference Python and C++ stacks are
run and compared.

## 1. What is Rust here and what is not

| Part | Language | Where it comes from |
|---|---|---|
| Webots simulator | C++ | installed separately, R2025a on the machine this was tested on |
| TurtleBot3 Burger model, LDS-01 lidar | Webots PROTO | pulled by URL from the Webots R2025a release, see the world file |
| `webots_ros2_driver`, `webots_ros2_turtlebot` | Python and C++ | `ros-jazzy-webots-ros2-turtlebot` and its dependencies, used exactly as published |
| robot description and ros2_control parameters | URDF and YAML | taken from the installed `webots_ros2_turtlebot` share directory, not copied and not edited |
| `diff_drive_controller`, `joint_state_broadcaster` | C++ | standard `ros2_control` |
| world file, landmark ground truth, launch file | Webots world, YAML, Python launch | this directory |
| **every consumer node: localization, mapping, planning, control** | **Rust** | `crates/` and `ros2_ws/src/` of this repository |

The boundary is the topic. The driver publishes `/scan`, `/odom`, `/imu`, `/joint_states`,
`/clock` and TF; the Rust nodes subscribe to them. The driver is simulation
infrastructure, not a porting target: it is not rewritten in Rust, not wrapped in Rust, and
not patched. A Rust node that works against this simulation works against a real
TurtleBot3 publishing the same topics.

This is also why `crates/robotics_sim` is not retired. It stays the seeded, headless,
bit-reproducible world used by the unit and regression tests of filters and sensor models
([../../docs/SIMULATION.md](../../docs/SIMULATION.md)), where starting a simulator GUI per
test would be absurd. Webots is where the full stack runs and where the benchmarks are
taken. Different jobs, both kept.

## 2. Files

| File | Content |
|---|---|
| [`worlds/turtlebot3_localization_room.wbt`](worlds/turtlebot3_localization_room.wbt) | Phase 3 world: empty 6 by 6 m room, four static pillars, the TurtleBot3Burger proto with an extern controller, and the `Ros2Supervisor` robot |
| [`config/landmarks.yaml`](config/landmarks.yaml) | ground truth of the room and the four pillars, read out of the world file |
| [`launch/localization_world.launch.py`](launch/localization_world.launch.py) | starts Webots, the supervisor, the official driver and the two ros2_control spawners, and optionally the ground truth supervisor and the movie recorder |
| [`scripts/evaluation_supervisor.py`](scripts/evaluation_supervisor.py) | extern Webots supervisor: publishes the true pose on `/ground_truth/pose` and teleports the robot on `/kidnap/teleport` |
| [`scripts/movie_recorder.py`](scripts/movie_recorder.py) | extern Webots supervisor that records the 3D view to a movie file and reports what it did as JSON |
| [`scripts/record_smoke_run.sh`](scripts/record_smoke_run.sh) | one recorded smoke run of this world, writing every measurement to `evidence/<run>/` as raw output |

## 3. Requirements

* ROS 2 Jazzy, and `sudo apt install ros-jazzy-webots-ros2-turtlebot`, which pulls in
  `webots_ros2_driver`, `webots_ros2_control` and the controller packages.
* Webots installed natively on Linux and on `PATH` as `webots` (R2025a here).
* Network access on the first run: the world refers to the TurtleBot3, lidar, arena and
  background PROTO files by URL, pinned to the `R2025a` tag, and Webots caches them under
  `~/.cache/Cyberbotics`.

## 4. Running it

```bash
source /opt/ros/jazzy/setup.bash
cd <repo root>
ros2 launch sim/webots/launch/localization_world.launch.py                  # GUI
ros2 launch sim/webots/launch/localization_world.launch.py gui:=false       # headless
ros2 launch sim/webots/launch/localization_world.launch.py mode:=fast       # faster than real time
```

Arguments: `gui` (default `true`), `mode` (`realtime`, `fast` or `pause`), `port` (default
`1234`), `use_sim_time` (default `true`). The launch file is run by path, so this directory
does not need to be a ROS 2 package or be built.

Two groups of optional arguments, both off by default, both served by a robot of their own
in the world file:

* `ground_truth:=true` starts [`scripts/evaluation_supervisor.py`](scripts/evaluation_supervisor.py),
  which publishes the true pose on `/ground_truth/pose` at `ground_truth_rate_hz` (default
  `50`) and teleports the robot when a pose arrives on `/kidnap/teleport`. No estimator
  subscribes to it; it exists to grade them and to stage the kidnapped robot experiment.
* `record:=true record_file:=<path.mp4>` starts
  [`scripts/movie_recorder.py`](scripts/movie_recorder.py), which records the 3D view for
  `record_seconds` (default `40`) of simulated time after `record_start_delay` (default
  `6`), and writes `record_status_file` as JSON when the encoding finished. Recording
  renders the 3D view, so it requires `gui:=true`.

A whole recorded run, with the measurements captured as files, is one command:

```bash
bash sim/webots/scripts/record_smoke_run.sh sim/webots/evidence/<run name> [record seconds]
```

Drive the robot by hand, remembering that `diff_drive_controller` on Jazzy takes a
`TwistStamped`:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.15}, angular: {z: 0.3}}}"
```

Two operational notes, both learned the hard way:

* **Stop the previous run before starting a new one.** If a Webots instance is still
  alive, the new one prints `Port 1234 is already in use ... Using port 1235 instead` and
  silently keeps running, while the controllers keep trying port 1234 and never connect.
  The symptom is `Cannot connect to Webots instance, retrying ...` in the launch output.
  `pkill -x webots-bin` cleans it up; the wrapper on `PATH` is a shell script, so killing
  `webots` alone leaves the real process behind.
* **WSL2.** `webots_ros2_driver.utils.is_wsl()` is true on a WSL2 kernel, so the upstream
  `WebotsLauncher` and `WebotsController` assume Webots runs on the Windows side: they pass
  the world as a `\\wsl.localhost\...` path and connect the controllers over TCP to the
  Windows host. With Webots installed natively in Linux, that fails before the world even
  loads. The launch file therefore starts Webots with `ExecuteProcess` and connects the
  official controllers over the local IPC protocol instead. The driver, the supervisor and
  the `webots-controller` launcher script are all still the upstream ones. On a setup where
  Webots really does run on Windows, use the upstream
  `webots_ros2_turtlebot/launch/robot_launch.py` pattern instead.

## 5. Verified run

Measurements live in [`evidence/`](evidence/), one directory per run, raw output written by
the run script and never retyped into this file. The numbers that used to stand here were
removed: they mixed two runs, one of them in a 6 by 6 m world and one in the earlier 8 by
8 m world, and they asserted a clean shutdown that had never been measured.

Each run directory holds `environment.txt` (host, kernel, ROS and Webots versions, RMW, and
the sha256 of the world), `node_list.txt` and `topic_list.txt`, `controllers.txt` with the
state of both ros2_control controllers, `topic_hz.txt`, one `echo_*.yaml` sample per topic,
`tf2_echo_odom_base_link.txt`, `movie_status.json` and the movie, `shutdown.txt` with the
exit code the launch actually returned, and `launch.log` and `run.log` in full. Read the
rates in the context of their own `environment.txt`: a GUI run that is also encoding a movie
has a lower real time factor than a headless one, so the same simulation reports lower
wall-clock rates, and any rate assertion in a test has to use the simulation clock.

The runs of 2026-10-06 are kept because of what they caught, not because they are tidy:

| Run | What it establishes |
|---|---|
| [`2026-10-06-gui-realtime`](evidence/2026-10-06-gui-realtime/) | The world, the driver, the controllers, the movie recorder and every topic work end to end: the robot drives, and `/odom` and `odom -> base_link` follow it. First attempt at measuring the shutdown, and the first run of this directory at all. |
| [`2026-10-06-after-fix`](evidence/2026-10-06-after-fix/) | First run with the placeholder-description fix and with the controller check in the run script. `controllers.txt` records both controllers `active`, which is the thing that had been assumed and never checked. |
| [`2026-10-06-sigint-ignored`](evidence/2026-10-06-sigint-ignored/) | The run that showed the shutdown verdicts of the two above to be worthless. Its `launch.log` has no `[launch] shutdown:` line at all, because the launch was never asked to shut down: the `kill -INT` of the run script went to a background job of a non-interactive shell, where SIGINT is `SIG_IGN`. Every "still alive 90 s after SIGINT" measured the script. |
| [`2026-10-06-sigint-fixed`](evidence/2026-10-06-sigint-fixed/) | The first run whose SIGINT actually arrives, after `set -m` in the run script, and so the first real measurement of the shutdown: the launch exits by itself, with exit code 0, in about two seconds, leaving no Webots and no launch process behind. The numbers are in its `shutdown.txt` and the `[launch] shutdown:` line of its `launch.log` names the signal that ended Webots. |

Three defects were found this way, every one of them silent, and each is described where it
lives rather than summarized here:

1. **The placeholder robot description** could leave a run with no command interface at all,
   so the robot stood still while every topic kept publishing. Comment in
   [the launch file](launch/localization_world.launch.py); the Phase 3 evaluation it ruined
   is kept with its own post mortem in
   [`../../evidence/phase-3-correctness/2026-10-06/INVALID.md`](../../evidence/phase-3-correctness/2026-10-06/INVALID.md).
2. **Webots outliving the launch**, because `webots` on `PATH` is a wrapper and the real
   process is its child, not the launch's. Handled by the shutdown handler at the bottom of
   the launch file, and it is the first operational note above.
3. **The run scripts' own SIGINT being ignored**, which turned every shutdown assertion into
   a statement about bash. Fixed with `set -m`; the comment at the top of
   [`scripts/record_smoke_run.sh`](scripts/record_smoke_run.sh) explains why a background
   job in a script cannot be Ctrl-C'd.

The third one is the reason the numbers that used to stand in this section were deleted
rather than corrected: a claim here could not be traced back to the file it came from, so
nobody noticed that one of them had never been measured at all.

## 6. Known gaps

* **No map.** The pillar positions exist as
  [`config/landmarks.yaml`](config/landmarks.yaml); an occupancy grid of the room arrives
  with Phase 4.
* **This launch file starts no Rust node.** It brings up the simulation and the official
  driver only. The Phase 3 nodes are started next to it by
  [`../../scripts/run_phase3_correctness.sh`](../../scripts/run_phase3_correctness.sh),
  which runs `localization_node` and `localization_evaluator` from
  `ros2_ws/src/robotics_localization_bridge` against this world. See
  [../../docs/phase-3-localization/implementation.md](../../docs/phase-3-localization/implementation.md).
