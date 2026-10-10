# Phase 3, implementation: the Rust side

> **State.** Both filters exist and run, against the same pillar map, in
> [`crates/robotics_localization`](../../crates/robotics_localization/): `map`, `pillars`,
> [`ekf`](../../crates/robotics_localization/src/ekf.rs) (one Gaussian belief) and
> [`mcl`](../../crates/robotics_localization/src/mcl.rs) (a particle population). They are
> wired to the topics by
> [`localization_node`](../../ros2_ws/src/robotics_localization_bridge/src/localization_node.rs)
> and [`mcl_node`](../../ros2_ws/src/robotics_localization_bridge/src/mcl_node.rs), and each
> is graded against the simulator's ground truth by its own `localization_evaluator`
> instance. Runs and their raw output live in
> [`../../evidence/phase-3-correctness/`](../../evidence/phase-3-correctness/).
>
> What the evidence supports is split by problem, and section 5 below is the detail:
>
> * **P1, position tracking: EKF and MCL pass.** Read
>   `normal-verdict.yaml` and `normal-verdict-mcl.yaml` of
>   [`2026-10-10T144153-final-global-reset`](../../evidence/phase-3-correctness/2026-10-10T144153-final-global-reset) for
>   both verdicts from one simulation session.
> * **P2, kidnap recovery: MCL recovers, the EKF does not.** Read
>   `kidnap-verdict-mcl.yaml` and `kidnap-verdict-ekf.yaml` of
>   [`2026-10-10T143418-mcl-global-reset`](../../evidence/phase-3-correctness/2026-10-10T143418-mcl-global-reset), which
>   grade both estimators against one teleport at one simulated instant.
>
> Still absent: Augmented MCL's `w_fast`/`w_slow` injection and KLD adaptive sizing (the
> fixed-count, sustained-trigger global-reset simplification is justified in `mcl.rs`),
> dominant-cluster pose extraction, the grid localization debugging mode, `/particle_cloud`,
> `/amcl_pose` and `/initialpose`, and every benchmark row. Sections 2 and 3 below name
> some paths that were planned rather than built; where they differ, the parenthetical says
> so. See [ROADMAP.md](../ROADMAP.md) for the phase contract.

## 1. Already in the repository

The estimation pieces [theory.md](theory.md) needs are written and tested; Phase 3 composes
them rather than starting over.

| Piece | Where | State |
|---|---|---|
| Bayes filter trait | [`bayes.rs`](../../crates/robotics_filters/src/bayes.rs) | implemented |
| EKF, generic over the manifold | [`ekf.rs`](../../crates/robotics_filters/src/ekf.rs) | implemented |
| Error-state EKF on SE(2), absolute position and heading updates | [`eskf_se2.rs`](../../crates/robotics_filters/src/eskf_se2.rs) | implemented |
| Particle set, log weights, low variance and multinomial resampling, ESS | [`particle.rs`](../../crates/robotics_filters/src/particle.rs) | implemented |
| KLD adaptive sample size, `Se2Binning` | [`kld.rs`](../../crates/robotics_filters/src/kld.rs) | implemented |
| NEES and NIS with chi-square bounds | [`consistency.rs`](../../crates/robotics_filters/src/consistency.rs) | implemented |
| Odometry and velocity motion models, sampler and density | [`motion/`](../../crates/robotics_models/src/motion/) | implemented |
| Beam model, likelihood field over an exact EDT, range and bearing landmarks, tempering | [`sensor/`](../../crates/robotics_models/src/sensor/) | implemented |
| SE(2) manifold, TF buffer with authority conflict detection | [`se2.rs`](../../crates/robotics_core/src/se2.rs), [`tf.rs`](../../crates/robotics_core/src/tf.rs) | implemented |
| Message conversion, QoS presets, TF broadcaster, diagnostics, shutdown | [`robotics_ros_bridge`](../../ros2_ws/src/robotics_ros_bridge/) | implemented |

## 2. What Phase 3 adds

**TODO**, planned shape only, to be replaced by a walkthrough of the merged code:

* `crates/robotics_localization`, a ROS-free crate: EKF localization with maximum
  likelihood association and a chi-square gate (**built**), MCL with the same landmark
  model, low-variance resampling triggered by effective sample size, and a fixed-fraction
  random-injection recovery strategy (**built**, as
  [`mcl.rs`](../../crates/robotics_localization/src/mcl.rs)); Augmented MCL's
  `w_fast`/`w_slow` injection with KLD sizing and dominant-cluster pose extraction, and
  grid localization kept as a debugging mode (**not built**, and `mcl.rs` says which
  simplification it made instead of each and why).
* The thin rclrs node that wires it to the topics in [ros-graph.md](ros-graph.md)
  (**built**, as
  [`ros2_ws/src/robotics_localization_bridge/src/localization_node.rs`](../../ros2_ws/src/robotics_localization_bridge/src/localization_node.rs),
  not as the `robotics_localization` package this section originally planned; the crate name
  is taken by the ROS-free crate above). The MCL half is a second node next to it,
  [`mcl_node.rs`](../../ros2_ws/src/robotics_localization_bridge/src/mcl_node.rs),
  publishing `/mcl/pose` and deliberately broadcasting no TF at all, so that running both
  in one launch to compare them head to head cannot produce two owners of `map -> odom`;
  its file header gives the reasoning.
* A map source. The four pillars of
  [the world](../../sim/webots/worlds/turtlebot3_localization_room.wbt) are available as
  landmarks today ([landmarks.yaml](../../sim/webots/config/landmarks.yaml)); an occupancy
  grid for the beam and likelihood field models arrives with Phase 4, so until then the
  grid-based path is tested against a hand-built map of the same room.
* Launch and parameter files in `robotics_bringup`, plus the Webots bringup that runs the
  simulation and the node together (**built**, as
  [`phase3_localization.launch.py`](../../ros2_ws/src/robotics_bringup/launch/phase3_localization.launch.py),
  which includes the world launch and starts the localizer, the evaluator, the route driver
  and the odometry noise injector behind one set of arguments).
* Integration test in the style of
  [`phase2_stack_test`](../../ros2_ws/src/robotics_integration/src/phase2_stack_test.rs):
  topics, frames, rates, TF ownership and clean shutdown asserted check by check (**built**,
  as [`scripts/verify_phase3_localization.sh`](../../scripts/verify_phase3_localization.sh)
  rather than a Rust binary; the reasoning is in that file's header, and in short it is that
  most of what Phase 3 adds lives between nodes rather than inside one). It covers both
  nodes: section G is the EKF's behaviour and diagnostics, section H the MCL's, and F11
  asserts that `mcl_node` is not a `/tf` publisher. The check count and the pass or fail of
  each check are in the `verify.log` and `summary.yaml` of the run directory under
  [`../../evidence/phase-3-localization-runtime/`](../../evidence/phase-3-localization-runtime/),
  not in this sentence.
* The harness the measurements are made with:
  [`route_driver`](../../ros2_ws/src/robotics_localization_bridge/src/route_driver.rs), which
  drives a waypoint route closed loop on ground truth. It is not part of the stack under
  test, and it is the second node after the evaluator that is allowed to read ground truth;
  the reason it exists rather than a list of `ros2 topic pub` calls is in its file header,
  and in short it is that a commanded `angular.z` is not the rate this robot turns at.

## 3. Open questions to settle in code

**TODO**, each of these is a decision the implementation has to make and defend with a
measurement, not with an opinion:

1. **Settled for the EKF path: a landmark extractor.** Does the lidar feed a landmark
   extractor (cluster the scan, fit pillar centres) or go straight into a likelihood field
   over an occupancy grid? The first matches the sparse room and the existing landmark
   model; the second generalizes to Phase 4 and beyond. The built EKF takes range and
   bearing observations from
   [`pillars.rs`](../../crates/robotics_localization/src/pillars.rs); the likelihood field
   path stays open for the MCL half, which needs the Phase 4 occupancy grid anyway.
2. How many beams of the 360 are used per update, and with which tempering exponent, to
   keep the effective sample size healthy.
3. **Settled: a separate estimator, and it owns `map -> odom`.** Whether the ESKF of
   Phase 2 absorbs the absolute correction directly (`update_position` and `update_heading`
   already exist) or the localization filter stays a separate estimator that only publishes
   `map -> odom`. `localization_node` is separate; it publishes
   `geometry_msgs/PoseWithCovarianceStamped` in the `map` frame and broadcasts `map -> odom`
   as `(map -> base_link) * (odom -> base_link)^-1` with the stamp of the estimate, which is
   the division of labour [ros-graph.md](ros-graph.md) and REP 105 describe. It publishes no
   other edge: `odom -> base_link` belongs to the base and stays continuous. Until
   2026-10-10 the node published no TF at all, on the argument that an unverified edge is
   worse than none; the edge is now verified, by checks F1 to F10 of
   [`scripts/verify_phase3_localization.sh`](../../scripts/verify_phase3_localization.sh),
   which resolve it and compose the whole chain down to the sensor frames.
4. **Settled for the measured normal and kidnapped scenarios.** What the node does
   when the chi-square gate rejects everything for several seconds: report degraded on
   `/diagnostics`, trigger injection, or both. `mcl_node` does both: a sustained low-fit
   streak injects particles and the node reports the injection on `/diagnostics`. The
   number of consecutive scans required and the fraction injected are
   `MclConfig::recovery_sustained_updates` and `recovery_injection_fraction`. The trigger
   watches the best upper median of raw per-observation likelihoods from one particle for
   20 consecutive scans. It then resets all 6000 particles across the room. Section 5 shows
   why this statistic and reset replaced the earlier mean and partial injection.
5. Particle count floor and ceiling, and what gets published on `/particle_cloud` at what
   rate, so visualization does not dominate the measured cost.

## 4. Where the evidence is

Nothing in this file quotes a measurement. The runs live under
[`../../evidence/phase-3-correctness/`](../../evidence/phase-3-correctness/), one directory
each, and [benchmark.md](benchmark.md) section 6 lists them and says what each is for. The
verdict on criterion P1 is a file in the run directory, `<scenario>-verdict.yaml` for the
EKF and `<scenario>-verdict-mcl.yaml` for MCL, written by
[`../../scripts/phase3_verdict.py`](../../scripts/phase3_verdict.py) from the CSV and the
JSON the run produced. A kidnap run is graded instead by
[`../../scripts/phase3_kidnap_verdict.py`](../../scripts/phase3_kidnap_verdict.py) into
`kidnap-verdict-ekf.yaml` and `kidnap-verdict-mcl.yaml`, because P1 asks whether tracking
error stays bounded and P2 asks whether an invalidated belief finds its way back; the two
questions need different numbers, and what "recovered" means is defined, with its three
constants, in that script's docstring and repeated into every file it writes.

The integration test keeps its own evidence, under
[`../../evidence/phase-3-localization-runtime/`](../../evidence/phase-3-localization-runtime/):
the full check list as `verify.log`, the launch log, the node and topic listings, the TF
dump, the diagnostics and a `summary.yaml` with the counts and the verdict. The final run,
[`2026-10-10T145912-mcl-final`](../../evidence/phase-3-localization-runtime/2026-10-10T145912-mcl-final),
passes all 83 checks with the installed 6000-particle default.

## 5. Two defects this phase found, and what was done about each

Kept because the reasoning is the material: both were found by a measurement disagreeing
with an expectation, and both fixes were checked on the normal and kidnapped scenarios.

### 5.1 MCL diverged for want of a per-observation gate (fixed, and measured)

MCL was implemented, its unit tests passed, and it then diverged to route-scale error the
first time it was driven in simulation. The symptom was not "less precise than the EKF";
read `normal-verdict-mcl.yaml` next to `normal-verdict.yaml` in
[`2026-10-10-mcl-comparison`](../../evidence/phase-3-correctness/2026-10-10-mcl-comparison)
for the two numbers from that one session.

The cause was a missing gate. The EKF's `observe()` rejects an observation no landmark
explains within its chi-square gate: it *skips* it, and counts it in `rejected`. MCL had no
equivalent, so every particle's weight carried the full unbounded penalty of every
observation, including the occasional spurious cluster from `pillars::detect`, a reflection,
or a wall edge briefly read as a pillar. That penalty is identical for every particle
whatever its pose, so it does not change which particles resampling favours. What it does
change is how the scan *looks*: it makes the scan appear to fit nothing at all, which is
exactly the statistic the recovery trigger watches, so the filter's own corrective mechanism
became its largest error source.

The fix is [`McLocalizer::gated_log_likelihood`](../../crates/robotics_localization/src/mcl.rs):
reweighting scores an observation only if its best landmark clears the same threshold the
EKF's gate uses, and an observation that fails contributes nothing instead of its
arbitrarily negative raw value. The recovery trigger deliberately keeps reading raw,
ungated per-observation likelihoods, so that gating cannot blind the filter to a real
kidnapping. Both paths and the reason they have to be separate are documented at their
definitions. Compare `normal-verdict-mcl.yaml` of
[`2026-10-10-mcl-gated`](../../evidence/phase-3-correctness/2026-10-10-mcl-gated) against
the pre-fix directory above for the size of the improvement.

One earlier attempt is worth recording because it failed: widening the initial particle
spread. It is not in the code. The spread was a real bug of its own, which is why
`MclConfig::initial_sigma_xy` exists and says so, but fixing it did not fix the divergence,
because the divergence was never about the prior.

### 5.2 The recovery trigger misfired on a healthy run (fixed, and measured)

The gate above fixed reweighting and did not fix everything. On a normal run with no
teleport anywhere in it, `mcl_node` still logs sustained-low-fit injections, and
`normal-verdict-mcl.yaml` of
[`2026-10-10-mcl-gated`](../../evidence/phase-3-correctness/2026-10-10-mcl-gated) therefore
still fails criterion P1: the count is in that run's `normal-mcl_node.log`, and the error
trace in `normal-run-mcl.csv` shows the filter tracking tightly between injections and
being kicked off the truth by each one.

The reason is structural rather than a bad constant. The trigger compares
`total_log_likelihood / observations.len()`, a plain mean over every observation in the
scan, against a per-observation threshold. A mean is not robust: one unexplainable detection
at a few hundred nats swamps three good ones at a fraction of a nat each, so a single
spurious cluster drags the statistic far below the threshold no matter how well the rest of
the scan fits. Five such scans in a row fire an injection. This is the raw, ungated number
by design, for the reason 5.1 gives, so the defect is not that it is ungated; it is that the
mean is the wrong summary of an ungated score.

The implemented fix keeps the recovery signal raw but computes an upper median across the
observations for each particle, then selects the best such median across particles. Every
observation in a median comes from the same particle, so a scan cannot look consistent by
matching each observation to a different pose. One bad detection cannot dominate four good
ones, while a kidnapping still produces a low median when the existing cloud explains fewer
than half of the detections. Empty scans do not advance the low-fit streak. With one
observation, that observation alone decides it. Nonfinite model evaluations become negative
infinity in the raw statistic and cannot make a bad scan look good.

The retained attempts show why the other two defaults matter. With 300 particles the median
still left nine injections and failed P1. With 3000 particles and a 20-scan streak there
were no injections, but MCL narrowly failed to beat odometry. With 6000 particles it passed
P1. Shortening the streak to five scans recovered from the kidnap but caused seven false
injections and failed the next normal run. Keeping 20 scans while replacing only 20 per cent
of the cloud did not recover. The final configuration keeps 6000 particles and 20 scans,
then performs one full global reset. It caused zero injections in the final normal run and
recovered in 6.92 s in the final kidnap run. The paired EKF did not recover.

### 5.3 Evidence kept being destroyed by the build mirror (fixed)

Not an algorithm defect, and it cost more runs than either of the above. Builds happen in a
WSL mirror while the git repository lives on the Windows side, and
[`scripts/wsl_run.sh`](../../scripts/wsl_run.sh) calls
[`scripts/sync_to_wsl.sh`](../../scripts/sync_to_wsl.sh), an `rsync -a --delete` from
Windows into the mirror, at the start of every run. Run output was written in the mirror, so
anything not yet copied to the Windows side was deleted by the next sync. That destroyed a
6270-row evaluation, a partial integration-test log, and later a set of MCL measurement runs.

Two changes close the window, and both are in place rather than one:

1. [`scripts/evidence_root.sh`](../../scripts/evidence_root.sh) resolves `EVIDENCE_ROOT` to
   the `evidence/` directory of the Windows checkout whenever it is mounted, and
   [`run_phase3_correctness.sh`](../../scripts/run_phase3_correctness.sh) and
   [`verify_phase3_localization.sh`](../../scripts/verify_phase3_localization.sh) both write
   there. A run's results cross the mount as they are produced, so there is no moment at
   which the only copy is in the mirror and no copy-back step to forget.
2. `sync_to_wsl.sh` excludes `evidence/` and `benchmarks/` from the rsync, so `--delete`
   cannot reach run output even for a run that did write into the mirror, which still
   happens when the mount is absent or an explicit output path is given.

The cost of (2) is that the mirror's `evidence/` no longer mirrors anything, and
`sync_to_wsl.sh` says so where someone would look. The Windows checkout is the only
authoritative copy. Evidence is small text, so writing it across the mount costs nothing
that matters; what makes builds stay in the mirror is cargo's and colcon's many-small-file
churn, which this is not.

## 6. How this part will be written

Same pattern as the rest of the material: every claim points at a file and a line that can
be read, every number comes from a run recorded in [benchmark.md](benchmark.md), and
anything that does not work yet says so in the text instead of being quietly omitted.
