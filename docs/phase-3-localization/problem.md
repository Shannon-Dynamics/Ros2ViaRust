# Phase 3, problem: where is the robot?

## 1. The situation

A TurtleBot3 Burger stands in an empty 6 by 6 metre room with four pillars at known
positions ([../../sim/webots/worlds/turtlebot3_localization_room.wbt](../../sim/webots/worlds/turtlebot3_localization_room.wbt)).
It knows three things about itself at every instant:

* wheel angles, from the encoders, at 50 Hz;
* angular rate and linear acceleration, from the IMU, at 20 Hz;
* 360 range readings per revolution of the LDS-01 lidar, at 5 Hz.

From the wheel angles it computes odometry, the dead reckoned pose produced by integrating
every small motion increment since start-up. Odometry is smooth, available at full rate,
and never jumps. It is also wrong, and it gets wronger forever.

## 2. Why odometry alone fails

Integration accumulates. Every wheel slips a little on the floor, every encoder quantizes,
and the two wheel radii are never exactly equal to the number in the configuration file. A
small systematic error in the effective wheel separation turns into a heading error that
grows with the distance driven, and a heading error turns into a position error that grows
with the distance driven after it. Nothing in the chain ever corrects anything, because
dead reckoning has no way of noticing it is wrong: there is no second opinion in it.

The reference stack admits this in its own configuration. The official
`webots_ros2_turtlebot` controller parameters carry

```yaml
wheel_separation: 0.16
# The real separation between wheels is not resulting in a perfect odometry
wheel_separation_multiplier: 1.112
```

which is an 11 per cent correction factor hand-tuned so that the published `/odom` looks
acceptable in that specific simulation. It is a calibration, not an estimator: it reduces
the drift rate, and leaves the structure of the problem untouched. Drive long enough and
the belief and the world part company again.

Two separate failures follow from this, and they need different answers.

## 3. Problem statement

Let `x_t = (x, y, theta)` in SE(2) be the pose of `base_link` in the map frame, `u_1:t` the
control or odometry inputs, `z_1:t` the lidar and IMU measurements, and `m` the map of the
room (the four pillar positions, the four walls). The robot must maintain the posterior

```text
bel(x_t) = p(x_t | z_1:t, u_1:t, m)
```

recursively and in real time, under two initial conditions.

**P1, position tracking.** The initial pose is known to within a few centimetres and a few
degrees, `bel(x_0) = N(x_0, Sigma_0)` with small `Sigma_0`. The posterior stays unimodal;
the question is only whether the error stays bounded while the robot drives, instead of
growing with distance as odometry does.

**P2, global localization.** The initial pose is unknown: `bel(x_0)` is uniform over the
free space of the room. The posterior starts multimodal, since four identical pillars and
four similar walls make several poses equally plausible, and it must collapse onto the
right mode from motion and measurement alone. The special case where a correct, confident
belief is suddenly invalidated, because the robot was picked up and put somewhere else, is
the same problem with a worse starting point: the filter has to notice that its own belief
has become impossible and recover from it.

## 4. What counts as solved

The answer is a number, measured in the same world for every implementation compared.

| Question | Metric | Target |
|---|---|---|
| P1, does the error stay bounded? | absolute trajectory error of the estimate against ground truth, mean and 95th percentile, over a fixed drive | bounded over distance, and strictly better than the drift of `/odom` on the same run |
| P1, does the filter know how wrong it is? | NEES against the reported covariance, with chi-square bounds ([`consistency.rs`](../../crates/robotics_filters/src/consistency.rs)) | inside the 95 per cent band, so the covariance is neither optimistic nor useless |
| P2, does it find itself? | time and distance from a uniform prior to a single surviving mode within tolerance of the true pose | converges; failure rate over repeated seeded runs reported, not hidden |
| P2, does it recover? | time to recover after a teleport of the robot | recovers, or reports that it cannot, instead of insisting on a wrong pose |
| cost | CPU time per update at a given particle count, and per lidar scan | comparable to or better than the reference implementations, see [benchmark.md](benchmark.md) |

Where each row stands is a verdict file, not a sentence here. The runs are listed in
[benchmark.md](benchmark.md) section 6; the current ones are in
[`../../evidence/phase-3-correctness/2026-10-10T144153-final-global-reset/`](../../evidence/phase-3-correctness/2026-10-10T144153-final-global-reset)
for P1 and
[`../../evidence/phase-3-correctness/2026-10-10T143418-mcl-global-reset/`](../../evidence/phase-3-correctness/2026-10-10T143418-mcl-global-reset)
for the kidnapped case. In outline: both EKF and MCL answer the first P1 row; the NEES row
is unevaluated for both because the evaluator CSV carries only `sigma_xy`; MCL answers the
P2 recovery row while EKF does not; and the P2 convergence row from a genuinely uniform
initial prior still has no run because `mcl_node` starts with `McLocalizer::new_at`.

Ground truth for these metrics comes from the simulator, not from the estimate under test.
It is wired out of Webots by
[`evaluation_supervisor.py`](../../sim/webots/scripts/evaluation_supervisor.py) on
`/ground_truth/pose`, which the estimator never subscribes to; see
[benchmark.md](benchmark.md), section 4.

## 5. Why this is the next step

Phase 2 already fuses wheel odometry and the IMU with an error-state EKF on SE(2) and
publishes `odom -> base_link` ([STATE_ESTIMATION.md](../STATE_ESTIMATION.md)). That filter
has exactly the weakness described above: every input it fuses is relative, so the
posterior is smooth and continuous but its error is unbounded in the long run. Phase 3 adds
the first absolute information, the lidar against a known map, and with it the `map` frame.
The structure stays conventional (REP 105): the localization result corrects `map -> odom`
and never touches `odom -> base_link`, so a consumer that wants continuity reads `odom` and
a consumer that wants global accuracy reads `map`.

Continue with [theory.md](theory.md), then [ros-graph.md](ros-graph.md) for where the node
sits in the ROS 2 graph.
