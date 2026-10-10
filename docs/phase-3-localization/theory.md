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

## 6. What this leaves to the implementation

| Decision | Options | Where it is settled |
|---|---|---|
| filter for P1 | EKF with landmarks, or MCL with a map | [implementation.md](implementation.md) |
| filter for P2 | AMCL with KLD sampling | [implementation.md](implementation.md) |
| sensor model | beam versus likelihood field | [implementation.md](implementation.md), measured in [benchmark.md](benchmark.md) |
| map source | the four pillars as landmarks now, an occupancy grid after Phase 4 | [implementation.md](implementation.md) |
| node interface | topics, frames, parameters | [ros-graph.md](ros-graph.md) |
