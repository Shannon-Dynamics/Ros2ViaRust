# This run is not valid evidence

The robot never moved. Keep the files, do not use the numbers.

`normal-run.csv` holds 588 rows in which `gt_x`, `gt_y` and `gt_theta` never leave the
origin and every `odom_*` column stays at zero, over 367 s of simulated time, while
`normal-run.log` reports the full square route being driven. The position errors in that
file (around 0.008 m) are the error of a filter watching a stationary robot from a
stationary prior. They say nothing about localization.

## Why

`normal-launch.log:56`:

```
[controller_manager]: Exception caught while loading and initializing components:
URDF cannot be parsed by a `ros2_control` component parser: no 'ros2_control' tag found in the URDF
```

followed at line 59 by

```
[controller_manager]: ResourceManager has already loaded a urdf and is initialized.
Ignoring attempt to reload a robot description.
```

The `controller_manager` inside the official driver initializes its resource manager from
the first `/robot_description` it receives. The launch file started
`robot_state_publisher` with the empty placeholder description that the upstream example
uses, `<robot name=""><link name=""/></robot>`, and on this run that empty description
arrived first. It has no `ros2_control` block, so no hardware component loaded, and the
real URDF that the driver published 6 ms later was refused.

The consequences are all in the logs of this run: `diffdrive_controller` could not be
activated because the command interface `left wheel motor/velocity` was not available
(line 73), `joint_state_broadcaster` found no state interfaces (lines 90-91), and from
line 100 onwards every `/cmd_vel` message is answered with `Can't accept new commands.
subscriber is inactive`. `normal-diagnostics.yaml` records the end state:
`No hardware components are loaded!`, `diffdrive_controller: inactive`.

Nothing failed loudly. The launch kept running, all the topics kept publishing, the
evaluator kept writing rows, and the script reported success.

## What was changed because of it

* `sim/webots/launch/localization_world.launch.py` now starts `robot_state_publisher`
  with the official URDF plus one empty link instead of the empty placeholder, so that
  whichever description initializes the resource manager carries the same `ros2_control`
  block.
* `scripts/run_phase3_correctness.sh` and `sim/webots/scripts/record_smoke_run.sh` wait for
  `diffdrive_controller` and `joint_state_broadcaster` to report `active` before anything is
  driven, and abort with the controller listing if they do not.
* `scripts/run_phase3_correctness.sh` additionally fails the run if the ground truth never
  moves more than 0.2 m, whatever the rows say.

The replacement run is in `../2026-10-06-after-fix`.
