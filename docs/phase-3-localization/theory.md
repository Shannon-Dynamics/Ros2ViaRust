# Phase 3, theory: from the Bayes filter to AMCL

Only as much theory as [problem.md](problem.md) needs. Full derivations live in
*Probabilistic Robotics* (Thrun, Burgard, Fox), chapters 7, 8, 11 and 12; what follows is
the path from the one recursion that everything is a special case of, to the filter that
answers both P1 and P2, with links to the Rust that already implements each piece.

## 1. One recursion

Every estimator here is the same two lines, applied once per cycle:

```text
predict    bel(x_t)  = integral p(x_t | u_t, x_{t-1}) bel(x_{t-1}) dx_{t-1}
update     bel(x_t) ~= eta p(z_t | x_t, m) bel(x_t)
```

Predict spreads the belief using a motion model and makes it less certain. Update
multiplies it by how well each pose explains the measurement and makes it more certain.
The trait both steps are expressed through is
[`BayesFilter`](../../crates/robotics_filters/src/bayes.rs); every filter in
`robotics_filters` implements it, so the parts below differ only in how `bel` is
represented and how the two lines are approximated.

The motion model `p(x_t | u_t, x_{t-1})` for a differential drive base is already written
in [`motion/odometry.rs`](../../crates/robotics_models/src/motion/odometry.rs) (relative
motion decomposed into rotate, translate, rotate, with noise on each) and
[`motion/velocity.rs`](../../crates/robotics_models/src/motion/velocity.rs) (constant
curvature arcs from `v` and `omega`). Both expose a sampler and a density, which is exactly
the split a particle filter and a Gaussian filter need.

## 2. Gaussian belief: EKF localization

Represent `bel(x_t)` as a mean and a covariance on SE(2). Predict by pushing the mean
through the motion model and the covariance through its Jacobian; update by linearizing the
measurement model around the predicted mean and applying the Kalman gain. That machinery
is [`Ekf`](../../crates/robotics_filters/src/ekf.rs), generic over the state manifold, and
[`EskfSe2`](../../crates/robotics_filters/src/eskf_se2.rs), the error-state form Phase 2
already runs for wheel and IMU fusion.

What Phase 3 adds is the measurement: an absolute one. For the four pillars, the natural
model is range and bearing to a known landmark,
[`sensor/landmark.rs`](../../crates/robotics_models/src/sensor/landmark.rs), which provides
`h(x, m_j)` and its pose Jacobian:

```text
r     = sqrt((m_x - x)^2 + (m_y - y)^2)
phi   = atan2(m_y - y, m_x - x) - theta
```

Two practical problems come with it:

* **Which pillar is which.** The lidar returns a cluster of points, not a label. Maximum
  likelihood association picks the landmark that best explains the observation, and a
  chi-square gate on the innovation rejects observations no landmark explains well enough.
  The same gate is the detector for "my belief has become impossible".
* **Linearization is a local approximation.** The Gaussian is unimodal by construction, so
  the EKF can track a pose it already roughly knows (P1) and cannot represent "I am in one
  of four places" (P2). This is a property of the representation, not a tuning failure.

The health of the result is measurable while it runs: the normalized innovation squared
and its chi-square bounds in
[`consistency.rs`](../../crates/robotics_filters/src/consistency.rs) say whether the
reported covariance matches the errors actually observed.

## 3. Sampled belief: Monte Carlo localization

Represent `bel(x_t)` by `M` weighted samples instead. Predict by passing every particle
through the motion model sampler, update by multiplying every weight by `p(z_t | x_t, m)`,
then resample. The container, the log weight arithmetic, the effective sample size and both
resamplers (low variance and multinomial) are
[`particle.rs`](../../crates/robotics_filters/src/particle.rs). The representation is
non-parametric, so a multimodal posterior is simply several clusters of particles: MCL
answers P2 where the EKF cannot.

The weight needs `p(z | x, m)` for a full lidar scan against the map. Two models exist:

* [`beam.rs`](../../crates/robotics_models/src/sensor/beam.rs), the four-way mixture of a
  correct reading, an unexpected obstacle, a maximum range reading and random noise. It is
  physically faithful and needs a ray cast per beam per particle, which is the expensive
  part.
* [`likelihood_field.rs`](../../crates/robotics_models/src/sensor/likelihood_field.rs),
  which projects each endpoint into the map and reads its distance to the nearest occupied
  cell out of a precomputed exact distance transform. No ray casting, much smoother in the
  pose, and the usual choice for MCL.

A full 360 beam scan makes the likelihood extremely peaked, which starves all but a handful
of particles after one update. The standard answers are to subsample beams and to temper
the likelihood with an exponent `beta`, `p(z | x)^beta`, implemented in
[`tempering.rs`](../../crates/robotics_models/src/sensor/tempering.rs) including the
variant that picks `beta` to hit a target effective sample size.

## 4. Augmented MCL and adaptive sample size

Plain MCL converges and then cannot recover: once every particle sits in one cluster, no
particle is anywhere else, and a teleported robot stays lost. Two corrections, both from
chapter 12, make it practical.

**Augmented MCL** tracks a fast and a slow moving average of the average particle weight,
`w_fast` and `w_slow`. When the fast average drops well below the slow one, the measurement
stopped agreeing with the belief, and the filter injects random particles drawn uniformly
over the free space with probability `max(0, 1 - w_fast / w_slow)`. This is the recovery
mechanism for the kidnapped case, and it costs nothing while the filter is healthy.

What was actually built is neither of these two in full, and
[`mcl.rs`](../../crates/robotics_localization/src/mcl.rs) says so at each decision rather
than leaving the gap to be discovered. Recovery is a fixed injection fraction fired by a
sustained low-fit streak, not the `w_fast`/`w_slow` averages, because that scheme needs two
more decay-rate constants that nothing in this phase gives a reason to prefer one value of;
the threshold the streak is judged against is the EKF's own chi-square gate expressed as a
likelihood ratio, so it is a number already defended elsewhere rather than one tuned until a
demo looked right. A fixed particle count stands in for KLD sizing, and a weighted mean
stands in for dominant-cluster extraction, both for the same reason: they are enough to
decide whether the filter recovers at all, which is the question this phase asks.

The part of this section that survived contact with a simulator least well is the weight
update. An observation that no pose explains gave every particle an unbounded penalty and
made the recovery trigger fire during normal tracking. Reweighting now applies the EKF's
per-observation likelihood-ratio gate. Recovery deliberately reads the raw scores, but
summarises them with the best particle's upper median, so a minority outlier cannot hide
the observations that still fit. Twenty consecutive low medians cause one full global
reset. This separation preserves kidnap sensitivity while protecting normal tracking; the
failed intermediate runs and final evidence are in
[implementation.md](implementation.md) section 5.

**KLD sampling** adapts the number of particles to how spread out the belief is. Bin the
particles, count how many bins are occupied, and draw only as many samples as the KL bound
requires for that many bins:
[`kld.rs`](../../crates/robotics_filters/src/kld.rs), with `Se2Binning` for the planar case.
Global localization starts with particles everywhere and many occupied bins, so the filter
pays for thousands of particles for a few seconds; once converged, a few hundred suffice.
The CPU cost therefore follows the uncertainty instead of a constant worst case, which is
what makes the comparison in [benchmark.md](benchmark.md) interesting.

## 5. Where the map frame comes in

The output of localization is not a new odometry. Following REP 105, the pose estimate is
published as the correction `map -> odom`, while the Phase 2 estimator keeps publishing a
continuous `odom -> base_link` that is never rewritten. Reading `map -> base_link` through
the TF chain then gives the globally accurate pose, including the jumps every correction
makes, and reading `odom -> base_link` gives the smooth one. Exactly one node owns each
edge; the TF buffer in [`core::tf`](../../crates/robotics_core/src/tf.rs) detects it when
two nodes claim the same one.

## 6. What the simulator's odometry actually does

Everything above assumes the robot has an odometry error worth correcting. In this
simulator that assumption needs checking rather than believing, and checking it changed how
the Phase 3 runs are driven. This section records what is in the chain, measured, because a
reader who skips it will mistake a property of the reference configuration for a property
of dead reckoning.

**Where the noise is, and where it is not.** Reading the official stack from the device up:

| Element | Source | Noise or slip model |
|---|---|---|
| wheel encoders | `PositionSensor` in `TurtleBot3Burger.proto` | none; `resolution 0.00628` rad is quantization, which is deterministic |
| gyro, accelerometer, compass, inertial unit | the same proto | none; no `lookupTable` and no noise field, so Webots returns the exact value |
| lidar | `noise 0.0043` in `RobotisLds01.proto`, applied as a fraction of `maxRange 3.5` | Gaussian, about 15 mm of standard deviation. This is the only stochastic sensor on the robot |
| wheel odometry | `diff_drive_controller/DiffDriveController` | none. Its parameter list has no noise and no slip entry at all; it is a deterministic kinematic integrator |
| hardware interface | `webots_ros2_control::Ros2ControlSystem` | none; it forwards velocity commands to the motors and reads encoder positions back |
| the floor | the physics engine | **yes**, and it is the only place slip comes from |

**The slip is real, and the reference configuration cancels it.** The physics engine does
make the wheels slip, and the effect is large: the 2026-10-06 route commanded `angular.z`
of 0.4 rad/s for 4 s at each corner, 1.6 rad, and ground truth turned about 1.38 rad, so
the robot rotates at roughly 86 per cent of the commanded rate
([evidence/phase-3-correctness/2026-10-06-after-fix](../../evidence/phase-3-correctness/2026-10-06-after-fix)).
The official `ros2control.yml` answers that with one constant:

```yaml
wheel_separation: 0.16
# The real separation between wheels is not resulting in a perfect odometry
wheel_separation_multiplier: 1.112
```

`diff_drive_controller` integrates with `wheel_separation * wheel_separation_multiplier`,
so it assumes 0.17792 m where the proto puts the wheel anchors 0.16 m apart, and reports
`0.16 / 0.17792 = 0.899` of the rotation ideal kinematics would predict. That is the slip,
to within the precision of this measurement, divided back out. The published `/odom` is
therefore not accurate because dead reckoning is accurate here; it is accurate because
upstream calibrated the integrator against the slip of this specific world, which is what
the comment in their file says.

**Why this changed the route rather than the robot.** A calibration is one constant and the
slip it corrects is not constant, so odometry still drifts; the question is only how far the
robot has to drive before the drift is large enough to compare anything against. Over the
0.9 m square of 2026-10-06, 3.1 m of path in 40 s, it drifts about two millimetres. That is
a floor, not a baseline: the filter was five times worse than it and the comparison said
nothing about either, because what it measured was the lidar noise of the filter against
the arithmetic of an integrator that had barely been asked to integrate. The route was
therefore replaced with
[`route_long.yaml`](../../sim/webots/config/route_long.yaml), 51 m of path and 21 sharp
corners, for the reason stated at the top of that file: a corner is where the two wheels
disagree most and where odometry earns its error, and a straight line costs it almost
nothing.

**No number in Phase 3 comes from injected noise.** The run of
[2026-10-10](../../evidence/phase-3-correctness/2026-10-10-long-route) is driven on the
odometry the simulator publishes, unmodified, and its verdict file says so in its
`odometry_noise: off` line. Lengthening the route was enough on its own, so nothing was
injected and the question of what sigma would have been defensible never had to be
answered.

An injector was written first, before the long route was tried, because the honest reading
of the table above is that this robot's dead reckoning is better than any real one's. It
resampled each odometry step through the motion model of section 2 with explicit
`alpha1..alpha4`. It is not in the repository, for two reasons in that order. It was not
needed, which is the result above. It was also wrong: decomposing a 50 Hz odometry step into
rotate-translate-rotate makes `rot1` the direction of a motion of a few micrometres, which
is numerically arbitrary, so the injected rotation noise scaled with an angle that was
effectively random at every step and compounded. Driving a 1.4 m test route through it
produced 3.25 m of injected drift. Keeping a default-off instrument with that defect in it
would be keeping a way to generate numbers that look like measurements, so it was deleted
rather than repaired.

If a later phase needs a realistic dead reckoning baseline, the thing to get right is the
one this got wrong: decompose the motion over a distance that makes `rot1` meaningful, not
over whatever arrived in the last 20 ms, and state the sigma and the seed in the run's own
environment file so that no number produced under injection can be mistaken for one the
simulator produced.

## 7. What this leaves to the implementation

| Decision | Options | Where it is settled |
|---|---|---|
| filter for P1 | **settled for this phase: EKF and fixed-count MCL with landmarks both pass the measured route.** The verdict files of [benchmark.md](benchmark.md) section 6 are why | [implementation.md](implementation.md) |
| filter for P2 | **settled: a particle filter, since only it can.** The simplifications taken against full AMCL with KLD sampling are each named and justified in `mcl.rs` | [implementation.md](implementation.md) |
| sensor model | beam versus likelihood field | [implementation.md](implementation.md), measured in [benchmark.md](benchmark.md) |
| map source | the four pillars as landmarks now, an occupancy grid after Phase 4 | [implementation.md](implementation.md) |
| node interface | topics, frames, parameters | [ros-graph.md](ros-graph.md) |

## 8. Global localization from a uniform prior

Sections 3 and 4 describe a filter that is given a good belief and has to keep it, or to
notice that it has lost it. This section is about a filter that is given none.

### The prior, and what it costs

Draw `M` particles from `Uniform(F x (-pi, pi])`, `F` being the free space of the room (the
room minus a disc around each pillar, see [problem.md](problem.md) section 6). That is a
uniform draw over a volume `V_prior = area(F) * 2 pi` in `(x, y, theta)`. After the first
scans the posterior is concentrated on a region of volume `V_post`: the poses from which the
detected pillars would have been seen at the ranges and bearings they were, to within the
measurement noise. The number of particles that start inside it is Poisson with mean

```text
lambda = M * V_post / V_prior
```

and the chance that *no* particle starts inside it is `exp(-lambda)`. That is the whole
difficulty of global localization with a sampled belief, and it explains the shape of the
results before any simulation is run. `V_post` is small: the range and bearing noise is a few
centimetres and a few hundredths of a radian, and a pose has to match all of them at once.
`V_prior` is not: it is a room's worth of area times a full turn of heading. So `lambda` is
small for the particle counts that are comfortable to run, and when it is small the posterior
region is empty of particles, and the filter does not narrow onto it. It narrows onto whatever
*does* have particles and happens to explain the scan.

Resampling then makes this permanent. Low-variance resampling keeps a particle in proportion
to its weight, so a hypothesis with no particle cannot come back, and a hypothesis with a few
is amplified into the whole cloud. This is *particle deprivation*, and it is why the failure
that matters is not "the filter diverged" but "the filter collapsed, confidently, onto a
hypothesis that was available and not the one that was true". More motion helps only if there
is something in the cloud for the new scans to select.

Two things change `V_post`, and so how many particles a start needs. The number of pillars in
view: one pillar fixes the pose to a ring (range and bearing to one known point is a
one-dimensional family), four fix it to a point, so a start that sees one pillar has a much
larger `V_post` than one that sees four, and a larger `V_post` is *easier* to hit but leaves
the ambiguity to be resolved by motion. And the unknown correspondence: the filter does not
know which pillar a detection is, so it associates each detection to whichever pillar
explains it best for the particle under test (`McLocalizer::best_landmark_log_likelihood`,
Thrun Table 8.2's maximum likelihood association), and the posterior has a region for every
assignment that fits. Those are what the symmetry of the map multiplies.

The measured success rate is therefore not a property of the filter alone: it is a
property of the filter *and a count*. [benchmark.md](benchmark.md) section 7 reports the
success rate at each particle count tried, and the shape is the one this section predicts,
a rate that rises with `M`. It does not rise as `1 - exp(-M / M0)` with one `M0`, because the
starts are not alike: each one has its own `V_post`, and the aggregate is a mixture of
exponentials with different `M0`.

### Why the kidnap trigger has to be off during a search

The tracking filter's trigger (section 4 and `McLocalizer::observe`) fires when the best
particle's upper-median fit stays below the gate for a number of consecutive scans, and
responds by throwing the cloud away and starting again from a uniform draw. Both halves of
that are wrong for a search. A search starts with a cloud that, by construction, explains
very little yet; firing on it would reset a cloud that is doing what it should, over and
over. And the reset *is* a uniform draw, which is what the search already is. So while the
filter is in `LocalizationPhase::GlobalSearch` the trigger is inert and its statistic is not
even computed. When the search converges the filter becomes an ordinary tracking filter and
the trigger is armed again, with its tuning exactly as it was.

This is a statement about *when the trigger runs*, not a change to it, and the unit test
`a_search_converges_hands_over_and_re_arms_the_kidnap_trigger` checks both sides: no
injections during the search, injections after it.

### The convergence criterion

A sampled belief has no flag that says it has found the answer, and nothing outside the
filter can supply one without being told the answer. So the filter judges itself from its own
cloud, and the judgement is a claim that can be wrong. The claim is made on the first scan at
which, for `convergence_sustained_scans` scored scans in a row,

1. the cloud's RMS position spread `sigma_xy = sqrt(var x + var y)` is below
   `convergence_sigma_xy_m`;
2. the cloud's circular heading spread `sigma_theta = sqrt(-2 ln R)`, with `R` the mean
   resultant length of the weighted headings, is below `convergence_sigma_theta_rad`;
3. the cloud's weighted mean pose explains at least `convergence_min_explained_fraction` of
   the scan's detections, a detection being explained if some pillar fits it with a per-
   observation log likelihood above the same threshold the kidnap trigger uses.

Position spread is the root of the *trace*, not one axis, so a cloud stretched along a ring
(what one visible pillar leaves) is not taken for a tight one. Heading spread is circular
because a linear deviation about a circular mean is wrong for a cloud that straddles `+-pi`
and for one holding two opposite headings. The third condition ties the claim to the
sensor, so that a tight cloud that the laser disagrees with is not a claim.

The two spread limits are one third of the tolerance the experiments grade success against,
so that a cloud meeting them has, to three standard deviations, a mean inside the tolerance.
They were fixed from that argument and from the filter's steady-state tracking spread
(`global.rs` says which file and which columns to read), and were not tuned to the
results.

The third condition has a strictness, and the results are why it is a parameter. The first
version was the upper median of the per-observation scores at the mean, the rule the kidnap
trigger uses, which is satisfied by one of two detections. It is lenient, and the
experiments show what leniency costs: a rotated twin of the true pose can explain *some* of
a scan, the claim is then made, and it is wrong. Requiring more of the scan to be explained
does **not** make the filter find the truth more often (the same trials succeed); it turns
some confident wrong claims into no claim. That is a trade of one failure for a safer one, and
the filter cannot do better than the particles it started with. Requiring the mean to explain
every detection is stricter than a real detector may tolerate, since one spurious detection
in a scan then blocks it; the end-to-end trials are where that cost, if there is one, shows.

### Where the criterion is blind

A claim is a statement about the cloud, and a cloud that has collapsed onto a twin is a tight
cloud. If the twin explains every detection the filter sees, nothing in sections 1 to 3 can
tell it from the truth, and the claim is made and is wrong. The map's near rotational
symmetry makes such twins; [problem.md](problem.md) section 6 says how near. What would
tell them apart is information the cloud no longer holds, since the particles near the true
pose are gone. The remedies are to keep them (more particles, and the cost that comes with
them) or to be able to bring them back (a restart of the search when the cloud stops
explaining the scans, which the kidnap trigger does after convergence and which a search
deliberately does not do). The second is a design change and is listed as open in
[implementation.md](implementation.md) section 6.

### Heading spread breathes at corners

The published spread of a converged filter is not steady. It rises for a few seconds around
each turn in place while the pose error stays small (`est_sigma_xy_m` against
`position_error_m` of
`evidence/phase-3-correctness/2026-10-10T144153-final-global-reset/normal-run-mcl.csv`). So a
criterion judged on spread is delayed by turns: a converged filter falls out of the criterion
at a corner and re-enters on the next straight leg. That is a property of the filter's
covariance, measured, and the convergence time of a search includes it.
