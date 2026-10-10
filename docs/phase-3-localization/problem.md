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

## 6. P2 has two halves, and the second one is harder

Section 3 folds two problems into P2 and they are not the same problem.

**P2b, the kidnapped robot.** The filter *had* a good belief and it was invalidated. It has
something to notice (the scans stop fitting) and a place to start from (the old belief, which
it can throw away). [benchmark.md](benchmark.md) section 6 is about this.

**P2a, global localization from nothing.** The filter never had a belief. The starting
distribution is uniform over the free space of the room and over every heading,

```text
bel(x_0) = Uniform(F x (-pi, pi]),    F = the room minus a disc around every pillar
```

where each disc has the radius of a pillar plus the robot's footprint, because those are
poses the robot cannot be in. There is no occupancy grid yet (Phase 4), so `F` is the room
square minus those discs, and "uniform over the free cells" is the continuous version of it.
Nothing signals that the filter is lost, because there is nothing to be lost from: the cloud
starts as wide as it can be and has to narrow onto the truth from the scans and the motion
alone.

What makes it hard here is the map. The four pillars are identical, so the filter has to
solve the correspondence problem as well as the localization problem (which detection is
which pillar is not given), and they are close to rotationally symmetric about the middle of
the room. [`scripts/phase3_map_symmetry.py`](../../scripts/phase3_map_symmetry.py) measures
how close, from the map file: its output is
[`map-symmetry.yaml`](../../evidence/phase-3-global-localization/2026-10-10-ambiguity/map-symmetry.yaml).
A rotated copy of the map lands each pillar within a few tenths of a metre of a pillar, which
is within a factor of two of the association gate, so a pose rotated by a quarter or half
turn can explain a scan with only a subset of the pillars well enough to pass.
[`global_trials ambiguity`](../../crates/robotics_localization/examples/global_trials.rs)
ranks start poses by how many separate regions of pose space explain one static scan from
them; the ranking is
[`ambiguity.csv`](../../evidence/phase-3-global-localization/2026-10-10-ambiguity/ambiguity.csv).
It is a relative ranking, not a count of hypotheses: the pose grid fragments a ring of
poses into pieces, so read its order and not its magnitudes.

### What counts as solved

Global localization is not solved by "the estimate ended up near the truth": a filter that
is wrong and says so is a different thing from one that is wrong and says it is converged.
So a trial has a *claim*, the filter's own declaration of convergence
([theory.md](theory.md) section 8 and `crates/robotics_localization/src/global.rs` define
exactly when it is made), and it is graded against ground truth the filter never sees:

| Outcome | Meaning |
|---|---|
| `SUCCESS` | a claim was made, the pose was within 0.15 m and 0.15 rad of the truth when it was made, and still was at the end of the route |
| `WRONG_CONVERGENCE` | a claim was made and the pose was not within tolerance: the filter was confident and wrong |
| `LOST_AFTER_CONVERGENCE` | a correct claim that the filter later lost |
| `NO_CONVERGENCE` | no claim was made by the end of the route |

The tolerances are those of the kidnap verdict
([`phase3_kidnap_verdict.py`](../../scripts/phase3_kidnap_verdict.py)), so P2a and P2b are
graded by one yardstick. `WRONG_CONVERGENCE` is the dangerous outcome for a robot that acts on
the claim, and the reason the criterion has a knob that trades it for `NO_CONVERGENCE`. The
reported metrics are the success rate over repeated trials with its 95 per cent Wilson
interval, and, among the successes, the time and the distance driven from the first scan to
the claim and the position RMSE after it. They are in [benchmark.md](benchmark.md) section 7,
generated from the evidence, not typed.

A trial is repeated from many start poses, because the outcome of one is a draw: the particles
are random, and so is how many of them start near the truth. The start poses come in three
kinds, all recorded as files next to the results: a development set used to choose settings, a
fresh set that played no part in choosing, and a set picked by the ambiguity ranking and the
map's symmetry, which is where a wrong-but-plausible hypothesis is most likely to survive.

Continue with [theory.md](theory.md), then [ros-graph.md](ros-graph.md) for where the node
sits in the ROS 2 graph.
