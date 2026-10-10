# Phase 3 integration test runs

One directory per run of
[`scripts/verify_phase3_localization.sh`](../../scripts/verify_phase3_localization.sh), the
Phase 3 counterpart of the 56-check `phase2_stack_test` and of
`scripts/verify_phase2_runtime.sh`. The test brings the whole localization stack up through
the single bringup launch
([`phase3_localization.launch.py`](../../ros2_ws/src/robotics_bringup/launch/phase3_localization.launch.py))
and then asserts, one named check at a time, with the standard ROS 2 CLI.

Read `summary.yaml` for the verdict and `verify.log` for the checks, each printed as `PASS`
or `FAIL` with its name. The rest of a directory is the raw CLI output the checks were made
from: the launch log, the node and topic listings, the controller states, the resolved
transforms, the diagnostics of each node, the parameter dump with the read-only rejection in
it, the route driver log, and the two CSV fixtures the run guards are tested against.

| Run | Checks | Verdict |
|---|---|---|
| [`2026-10-10`](2026-10-10/) | 71 | **PASS**, all of them. The first run of this phase to have an integration test at all. |

## What the checks cover

| Section | What it asserts |
|---|---|
| A | the four pieces exist and are installed: the binaries, the bringup launch, the world launch, the map and the routes |
| B | the two run guards of [`../../scripts/phase3_guards.sh`](../../scripts/phase3_guards.sh) accept a healthy run and reject an unconfigured controller, a missing broadcaster, an empty listing, a stationary run and a missing CSV |
| C | the stack comes up through the bringup launch: clock, scan, ground truth, both controllers active, and every node in the graph |
| D | every expected topic exists with the right message type |
| E | those topics actually publish, and at what rate |
| F | the TF tree is complete: `map -> odom`, `odom -> base_link`, `base_link -> {LDS-01, imu_link, base_footprint}`, and the composed chains `map -> base_link`, `map -> LDS-01`, `map -> imu_link`; plus that the `map -> odom` edge appears on `/tf` and that `localization_node` is a publisher there |
| G | `localization_node` reports `RUNNING`, reports `map -> odom` as published with a non-zero `tf_sent`, took the laser mount from TF rather than from parameters, and refuses to have its read-only parameters set |
| H | `route_driver` steers, the base accepts its commands, the robot reaches the waypoints of the smoke route, and the node exits 0 when the route is finished |
| I | the stack shuts down on SIGINT and leaves no `webots-bin` holding the controller port and no node running |

Section B is worth its own sentence. It tests the guards by calling the functions a real run
calls, from the file both scripts source, rather than a copy of them: the two silent
failures of 2026-10-06 are the reason those guards exist, and a guard nobody tests is a
guard that stops working without anyone noticing.
