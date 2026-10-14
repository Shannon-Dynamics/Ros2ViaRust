# Roadmap

Each phase ends with: library tests green, ROS node running under `ros2 launch`, CLI
evidence recorded in `README.md`, and `SOURCE_MAPPING.md` updated. Phases are ordered by
dependency, not by chapter number.

Problem-based write-up of these phases, one part each: [README.md](README.md).

## Phase 1 — Foundation ✅

* Cargo + colcon workspace; interface underlay with Rust bindings; `scripts/env.sh`.
* `robotics_core`: `Se2` (exp/log/adjoint/⊞/⊟), `Manifold`, `Gaussian`/`Canonical`,
  `Timestamp`, `FrameId`, internal data model.
* `robotics_models`: diff-drive kinematics, `WheelOdometry` with covariance compounding,
  velocity and odometry motion models (sampler + density).
* `robotics_filters`: `BayesFilter`, `HistogramFilter1D`, `KalmanFilter` (Joseph form).
* `robotics_ros_bridge`: converters, QoS presets, `TfBroadcaster`, diagnostics.
* Nodes: `qos_talker`, `qos_listener`, `fake_wheel_encoders`, `wheel_odometry_node`;
  launch files and YAML config.

## Phase 2 — State estimation, sensor models, simulation ✅ (closed 2026-09-27)

* ✅ `robotics_filters`: `Ekf`/`Ukf` generic over `Manifold`, error-state EKF on `Se2`
  (Ch. 7); `InformationFilter` + `RtsSmoother` (Ch. 6); `ParticleSet` with log weights,
  low-variance and multinomial resampling, ESS; KLD-adaptive sizing with pluggable
  binning (`kld`, Ch. 8); NEES/NIS with χ² bounds (`consistency`).
* ✅ `robotics_models::sensor`: beam model with EM intrinsics (Table 6.2), likelihood
  field over an exact EDT, range–bearing landmark model with pose Jacobian and pose
  sampler, likelihood tempering incl. ESS-targeted β (Ch. 10).
* ✅ `robotics_sim`: segment `World` with exact ray casting, Rusty kinematics, slip,
  encoders, gyro, 2-D LiDAR, per-source RNG streams, bit-exact replay (Ch. 4); `sim_node`
  publishes `/scan`, `/joint_states`, `/imu/data`, ground truth, static
  `base_link → {imu_link, laser}`.
* ✅ `state_estimator_node`: fuses `/wheel/odometry` + `/imu/data` with the ESKF; publishes
  `/odom`, `/state_estimate`, `odom → base_link` (sole owner), mean NIS on `/diagnostics`.
* ✅ TF buffer (`robotics_core::tf`) and `TfListener` in the bridge (rclrs has none), with
  TF-authority conflict detection; Rust-first `launch_testing` equivalent
  (`robotics_integration/phase2_stack_test`, 56 checks incl. SIGINT shutdown).

Exit evidence: `scripts/verify_phase2_all.sh` (fmt, clippy `-D warnings`, `cargo test`,
underlay, `colcon build`, runtime CLI checks, integration test), results in `README.md`
§ Evidence and `docs/STATE_ESTIMATION.md` § 4. Nothing from the Phase 2 contract was
deferred.

## Phase 3 — Localization ✅ (closed 2026-10-10, with carried-over items)

Built and running: the EKF half, and the infrastructure around it.

* `crates/robotics_localization` (`map`, `pillars`, `ekf`): EKF localization on SE(2) with
  maximum likelihood association and a chi-square gate.
* `localization_node` and `localization_evaluator` in
  `ros2_ws/src/robotics_localization_bridge`; the node publishes `/localization/pose` and,
  since 2026-10-10, owns the `map → odom` correction (REP 105), so the TF chain
  `map → odom → base_link → {LDS-01, imu_link}` is complete.
* One bringup for the whole phase,
  `ros2_ws/src/robotics_bringup/launch/phase3_localization.launch.py`, which includes the
  Webots world launch and starts the localizer, the evaluator, the route driver and the
  odometry noise injector behind one set of arguments.
* The integration test, `scripts/verify_phase3_localization.sh`: checks over the installed
  binaries, the run guards, bringup, topics and types, publication, the whole TF tree, both
  nodes' diagnostics and parameters, the route driver and a clean shutdown. Section G covers
  the EKF, section H the MCL, and F11 asserts that `mcl_node` claims no TF edge. The check
  count and the result of each check are in the `verify.log` and `summary.yaml` of the run
  under `evidence/phase-3-localization-runtime/`, not in this line.
* The measurement harness: the Webots world with its pillars, the ground truth and teleport
  supervisor, and `route_driver`, which drives a waypoint route closed loop on ground truth
  so that the geometry of a long route is a property of the route file rather than of how
  much the wheels slipped. Runs are kept as raw output under
  `evidence/phase-3-correctness/` and judged by `scripts/phase3_verdict.py`, never by a
  number typed into prose.

Also built and run: the MCL half. `crates/robotics_localization/src/mcl.rs` is particle
filter localization on the same landmark map and the same pillar detector as the EKF, with
low-variance resampling triggered by effective sample size and a fixed-fraction random
injection recovery strategy, wired up as `mcl_node` publishing `/mcl/pose` and no TF. The
kidnap scenario now has runs as well as a harness: `scripts/run_phase3_correctness.sh
kidnap` drives the route and teleports the robot once through the Webots supervisor, and
grades both estimators against that one event in one simulation session. `mcl_node` also
starts, opt-in, from a uniform prior over the free space (`initial_pose.mode:=uniform_prior`),
a separate code path from the kidnap recovery (`crates/robotics_localization/src/global.rs`);
`scripts/verify_phase3_localization.sh` checks it in its section K.

Not built yet: Augmented MCL's `w_fast`/`w_slow` injection and KLD adaptive sizing, which
`mcl.rs` justifies simplifying rather than silently omitting; dominant-cluster pose
extraction; grid localization as a debug mode; `/particle_cloud`, `/amcl_pose` and
`/initialpose`; and every benchmark row in `benchmarks/phase-3-localization.csv` is still
empty, so no Rust against rclpy against Nav2 comparison exists yet.

Where the two filters stand is split, and each half is a verdict file rather than a claim
in this paragraph. Read the normal evidence in
`evidence/phase-3-correctness/2026-10-10T144153-final-global-reset/` and the kidnap evidence
in `evidence/phase-3-correctness/2026-10-10T143418-mcl-global-reset/`:

* **P1, position tracking.** EKF and MCL pass the two parts a run can decide. Each beats the
  dead reckoning it was given and keeps its error bounded over the 51.54 m route. MCL uses
  6000 particles and records zero recovery injections in the final normal run.
* **P2b, kidnap recovery.** MCL recovers from a teleport the EKF structurally cannot
  (`kidnap-verdict-mcl.yaml` against `kidnap-verdict-ekf.yaml`, both from one teleport at
  one simulated instant). The trigger uses the best upper median of raw per-observation
  likelihoods for 20 scans, then performs one full global reset of the particle cloud. MCL
  meets the position and yaw recovery window; the earlier single run did so 6.920 s after the
  teleport, which is not representative. Over twenty real Webots trials of one fixed
  teleport (seeds 1001 to 1020,
  `evidence/phase-3-correctness/2026-10-10T2026-c56f55d-verdicts/kidnap-trials-summary.yaml`)
  MCL recovered 20 of 20, Wilson 95% interval 0.8389 to 1.0000, with a minimum recovery time
  of 15.480 s, a median of 47.070 s and a maximum of 206.580 s; the EKF recovered 0 of 20
  (0.0000 to 0.1611). Those figures come from
  `scripts/phase3_kidnap_trials_summary.py`. That is one kidnap geometry. On the model sample
  that varies the geometry
  (`evidence/phase-3-global-localization/2026-10-10-kidnap-model-existing-tuning/`, n=102)
  the same `mcl.rs` recovers about 39% as shipped, and 81% in the 48000-particle what-if
  (n=36). The two results cannot be compared directly, because the kidnaps they cover
  differ; `benchmark.md` section 7.5 has both.
* **P2a, global localization from a uniform prior.** Built and measured, and the honest status
  is "works, with a cost, and is not reliable at the shipped particle count". The robot is
  told nothing; `mcl_node -p initial_pose.mode:=uniform_prior` draws the particles over the
  free space and declares convergence itself. At the steady-state count of 6000 particles most
  searches end in a confident wrong pose, nearly always a copy of the true one rotated by a
  quarter or half turn (the pillars are nearly rotationally symmetric); the success rate rises
  with the number of particles used for the search, and the counts that do best cost more per
  scan than the scan period on one core. A stricter convergence criterion trades confident
  wrong answers for no answer and does not find the pose more often. The tables are
  `docs/phase-3-localization/benchmark.md` section 7, generated from
  `evidence/phase-3-global-localization/` and checked by the `docs` gate of `scripts/gates.sh`.

The NEES part of the P1 row is not evaluated for either filter, because it needs the full
covariance of each estimate and the evaluator CSV carries only `sigma_xy`.

Known limitations carried out of Phase 3 (the phase is closed on its correctness evidence;
none of the items below is claimed done):

* **Known limitation: the kidnap "recovered" criterion has no time bound.**
  `scripts/phase3_kidnap_verdict.py` counts a trial as recovered whenever the position and yaw
  windows are eventually met and held, however late. A trial that took 206.580 s (trial-14 of
  the Webots group) is counted as recovered exactly like one that took 15.480 s, so the
  20 of 20 above says that MCL eventually recovers, not that it recovers in a time a robot
  could use. A deadline for the criterion is not implemented.
* **Known limitation: 20/20 recovery has only been verified for one kidnap geometry
  (seed-only variation, seeds 1001–1020).** Generalization to random kidnaps is measured in
  the model at ~39–81% and has not been validated in Webots. Varying the geometry (this needs
  a change to `run_phase3_correctness.sh`, ~20 sessions, ~2.5 hours) is deferred to the
  backlog, not cancelled.

Still open in Phase 3, in the order they bear on the results above:

* a restart for a search that has collapsed onto the wrong pose (the kidnap path has one,
  after convergence; a search deliberately has none), which is the obvious next lever;
* the search cost: linear in the particle count and independent per particle, so it
  parallelizes, and it has not been parallelized;
* the kidnap tuning, whose measured recovery rate is the baseline for any change to it;
* adaptive particle sizing and the cross-language benchmark.

The normal tracking and kidnapped recovery work, and global localization from a uniform
prior as a built and measured feature, are in; the last is not solved.
`docs/phase-3-localization/benchmark.md` section 6 lists the runs, and `theory.md` section 6
explains why the route had to be lengthened before the comparison meant anything. Details
and the open decisions are in `docs/phase-3-localization/implementation.md`.

Inputs already available from Phase 2: `/scan` with a static `base_link → laser`, `/odom`
and `odom → base_link`, the Rust `TfListener`, `World::raycast` as a `z*` source, beam /
likelihood-field / landmark models, tempering, `ParticleSet` + KLD sizing with
`Se2Binning`, `EskfSe2::update_position` / `update_heading` for absolute corrections.

* `robotics_localization`: EKF localization with ML association + χ² gating (Ch. 11),
  MCL and Augmented MCL with `w_fast/w_slow` injection, KLD adaptation, dominant-cluster
  estimate (Ch. 12), grid localization as a debug mode.
* `localization_node`: `/scan` + `/odom` + `/map` (+ `/initialpose`) → `/pose`,
  `/particle_cloud`, `map → odom`. Nav2-compatible outputs (`/amcl_pose`).

## Phase 4 — Mapping ✅ (closed 2026-10-11, with carried-over items)

* `robotics_mapping`: log-odds `OccupancyGrid` with hand-crafted inverse model and
  Bresenham/supercover integration, entropy (Ch. 13); exact EDT/ESDF (Ch. 19). *(The 2-D TSDF named here originally is moved
  out of Phase 4 on 2026-10-10; see Phase 5.)*
* `mapping_node`: `/scan` + `map → base_link` → `/map` (latched), `/map_entropy`.

**Status (2026-10-11): T1 to T8 built; M1 met on each of three fresh evaluation routes, M2 and M4 pass on every
run; M5 waits for decision D3.** Round one (three routes) missed different M1 checks on each and was burned by the analysis
that followed (`evidence/phase-4-mapping/2026-10-11-evaluation-group/`). It found, and the work since fixed, a mismatch between
the lidar bearings the Webots driver publishes and what Webots measures (a mapper option, `scan_beam_layout`, default
`published`), the erosion of pillar cells by grazing views against a narrow occupied-to-clamp margin (Bresenham traversal and a higher
upper clamp), and two properties of the evaluator (observability ignoring incidence, now limited to 60 degrees; an asymmetric wall
lookup, now symmetric). Frozen together before development run 4: `pixel_centres`, Bresenham, `l_max` 5.0, the evaluator's two rules.
Development run 4 and round two, on three new routes (oblique wall approaches, arcs round the pillars, the perimeter with detours),
each pass all seven pre-registered M1 checks, one binary and no tuning on them, spread of the mean surface error 0.001 cells; each is a row
of its own in `evidence/phase-4-mapping/2026-10-11-evaluation-4..6-*`, with `2026-10-11-evaluation-group-2/NOTE-round-two.md`. Not
claimed: an estimated pose (M5), a more complex world (M1-extended), real hardware; the parameters were chosen on the burned runs,
and the evaluator's rules changed between the rounds, so round two is graded against the metric of 2026-10-11 and not the one round one
failed (the note gives the number). Open: a residual beam-angle error at the ends of the scan, the clamp's cost in a changing world, and
`benchmarks/phase-4-mapping.csv` (implementation.md 5.3). Original status line: *planned and confirmed, nothing built*. Design doc:
[phase-4-mapping.md](phase-4-mapping.md), with the detail in `docs/phase-4-mapping/`:
[problem](phase-4-mapping/problem.md), [implementation](phase-4-mapping/implementation.md),
[benchmark](phase-4-mapping/benchmark.md). The two bullets above are the original scope with
one change: the 2-D TSDF (and the quadtree) of Ch. 19 is **out of Phase 4**, see Phase 5.
In outline:

* **What kind of claim.** Phase 4 is mapping with known poses and its claims are *not*
  rates. M1, accuracy against the analytic ground truth of the world, is judged on one
  development run and three evaluation runs, each reported on its own with the spread; no
  success rate, no Wilson interval, no trial set. M2 (replay is bit-identical) and M3 (grid
  rules) are tests; M4 is a latency distribution over the scans of a run; M5 (the map under
  the real estimator's pose) waits for Phase 3. Reasons and non-claims are in `problem.md`
  section 4.
* **Pose, without a flag in the mapper.** `mapping_node` has no pose-source parameter and
  reads `map → base_link` from TF only. In simulation a separate
  `ground_truth_tf_adapter` provides that chain by publishing `map → odom` from the
  simulator's ground truth, exclusive with `localization_node` (one owner of the edge). The
  adapter, not the mapper, is the extra reader of `/ground_truth/pose`, and a run records
  which node owned `map → odom`. No Phase 4 result may be called one of the localization
  stack before Phase 3 closes.
* **Forward dependency for M5.** `mcl_node` publishes `/mcl/pose` and no TF, so mapping with
  MCL's pose needs either a TF publisher in `mcl_node` (a Phase 3 change that also changes
  its check F11) or an adapter like the ground-truth one that turns `/mcl/pose` into
  `map → odom`. Undecided, and a Phase 3 matter either way.
* **Staging.** (1) pure crate: grid, inverse model, ray integration, entropy, accuracy
  metrics; (2) signed ESDF over the existing exact EDT; (3) the adapter, `mapping_node`,
  harness and verdict script on ground-truth pose; (4) reference implementations, which are
  informative and do not block; (5) the estimated-pose integration, after Phase 3.
* **Evaluation.** The analytic geometry of the world is the baseline and the only blocking
  yardstick: surface error, completeness, false-occupied and false-free cells, coverage,
  entropy as a progress trace (never alone), per-scan update latency as median/95th/99th
  percentile. An rclpy port of the same update and `nav2_costmap_2d` are additional
  information and block nothing; `slam_toolbox` is excluded (it estimates its own pose).
* **Naming.** Crate `crates/robotics_mapping`; the ROS package, holding
  `ground_truth_tf_adapter`, `mapping_node` and `mapping_evaluator`, is
  `ros2_ws/src/robotics_mapping_ros` (a workspace cannot hold two packages called
  `robotics_mapping`). `ARCHITECTURE.md` and `SOURCE_MAPPING.md` are updated to match.

Phase 3 is not closed by this plan: the Nav2/rclpy benchmark and the remaining Phase 3
items above are untouched and still open.

## Phase 5 — SLAM (in progress, **not closed**: the core, the nodes, `/map` and `map -> odom` are built and the nodes ran once against live Webots; S4 is met on Webots data, S5's zero wrong constraints only on a recording whose ground truth was corrected, and fails on both recordings as recorded)

**Status (2026-10-15): not closed. Stage B (submap-rendered `/map`, `map -> odom` owned by `pose_graph_node`, DS17/DS2) is built and replay-verified; S4 and S5 were measured on Webots data (two recordings of `route_slam_eval_rectangle.yaml`, 13 turns each) and `source:=live` ran against a live Webots driver for the first time. S4 PASSES on both recordings and on the corrected one (graph ATE RMS 1.4 to 1.6 cm, graph/chain ratio 0.60 to 0.72), which disagrees with the Rusty prediction that the ratio check fails on natural runs. S5 FAILS on both recordings as recorded (54 and 63 accepted constraints labelled wrong) and PASSES on the corrected recording 2 (0 of 157); the recorded poses of scans taken in turns are wrong by p95 0.14 rad, above S5's 0.03 rad, and the corrected copy is the only one that can be made (recording 1 has no dense ground-truth track). The correction's lattice phase (0.198 s) disagrees with the earlier probe's (0.0603 s), unexplained, and was selected by agreement with the matcher's own rotations. The live run produced `slam/pose`, `slam/path`, `/map` and `map -> odom` and shut down cleanly; the replay checker passes 20 of 25 on it, the five failures explained in benchmark.md 7.1, not re-verified. S8 is not claimable. Phase 5 is not marked closed: whether S5 met on one corrected recording counts as met is the owner's call. Details: `docs/phase-5-slam/benchmark.md` 7.1.**
Previous status line (2026-10-11), kept: **Status (2026-10-11): design written; the pure crate `robotics_slam` built (76 unit and 40
integration tests, fmt and clippy clean) with the verdict script and two harnesses; the first evidence
runs of S1a, S1b, S2, S4, S5, S6 and S7 are made on Rusty (eight synthetic runs, four routes, natural and
with an injected error) and on the four Phase 4 recordings (the Webots half of S1a); S4 fails its
pre-registered "graph not worse than the chain" check on all four natural runs because there is no drift to
repair, and passes with a declared error injected; S5 found no wrong constraint among 1638 accepted, on data
with almost no false candidates; the first batch also found a front-end bug, fixed. `scan_matcher_node` and
`pose_graph_node` (package `robotics_slam_ros`) are now built — scan-only LiDAR odometry and the pose graph,
no `/map`, no TF, no `map -> odom`, `loop_closure: unsupported`, matching the dependency argument in
problem.md section 6 — and node-replay-verified by `scripts/verify_phase5_slam_nodes.sh` against a release
build (`scripts/build_release.sh slam`): 66 of 66 checks pass (four synthetic routes, four Phase 4
recordings, a real-time-rate run, replay determinism, clean shutdown), `cargo test` on `robotics_slam` and
`robotics_slam_ros` together is 126 passed 0 failed, fmt and clippy clean. Node replay found and fixed a
second front-end bug beyond the one above: a settled registration with no usable line normal reported a
zero information matrix that a consumer could not invert; it now reports the configured fallback instead
(`StepResult::information_fallback`, section 8.3 item 13 of implementation.md). On the two Phase 4
recordings with no turning scans, the front end loses track at the gaps as expected and check G (trajectory
against ground truth) is measured, not graded, at several metres ATE — this is the known limit of a
recording without a turn, not a regression, and is why a live Webots run with a turning-scan recording is
still needed before S4 and S5 can be measured on Webots data. Submap-rendered `/map` and ownership of
`map -> odom` are not built; they waited for Phase 4's M1, which is met since Phase 4 closed on 2026-10-11, so
they are no longer blocked. S8 is not claimable.**
What each run shows and does not show: `evidence/phase-5-slam/NOTE-first-evidence-runs.md`; the node replay
runs are under `evidence/phase-5-slam-nodes/`. Previous status
line: *design written; the pure crate built, no evidence run made; `scan_matcher_node` and `pose_graph_node`
are deferred pending Phase 4's M1 and are not started, not even as stubs*. Earlier status
line: *design written; the pure crate built, no evidence run made*. Original status line: *scope bullets only, nothing built*. Phase 5 was
started while Phase 4 is open, as Phase 4 was started while Phase 3 was open, and only for the part that
does not consume the occupancy grid: the ICP, NDT and optimizer core reads scans and relative poses, and the
one place a grid appears (the RBPF's own) is private to it, so `cargo tree -p robotics_slam` has no
`robotics_mapping`. Design doc: [phase-5-slam.md](phase-5-slam.md), with the detail in
`docs/phase-5-slam/`: [problem](phase-5-slam/problem.md), [implementation](phase-5-slam/implementation.md),
[benchmark](phase-5-slam/benchmark.md). In outline:

* **Claims S1 to S8** (not M-numbers: Phase 4 owns M1 to M5). Scan-matching accuracy is a distribution
  over registrations on representative runs; ICP's basin is a seeded trial set, model level only;
  the information matrix is reported, never pass or fail; trajectory accuracy is one development run and
  three evaluation runs against ground truth (never against Webots' calibrated odometry); loop closure is
  counts with **zero accepted wrong constraints pre-registered**; determinism and cost. **"SLAM produces a
  usable map" (S8) is not claimable** until Phase 4's M1, a measured S5 and the rendering node all exist.
* **Loop closure is verified, never faked.** The library has a detector and a verifier (fit, not
  degenerate, consistent under a χ² gate on the exact marginal covariance, unambiguous under restarts
  including quarter turns) with candidates designed to be rejected in its tests. S5 has been run on synthetic
  data only, and weakly against false candidates (problem.md section 5): a node reports `loop_closure: unsupported`
  until the project owner decides that is enough.
* **TSDF and quadtree: decided, no.** Matching is against points, the room is 14 400 cells, the ESDF
  exists, and any fused map is rebuilt by replay after a loop closure (DS13, with the triggers to
  revisit).
* **Deferred, pending Phase 4's M1:** submap-rendered `/map` and ownership of `map -> odom`. The two ROS
  nodes themselves are no longer deferred on this — they are built and node-replay-verified without either
  feature, by design (section 6 of problem.md records the dependency argument that made this possible).

Scope as originally written:

* `robotics_slam`: point-to-point / point-to-plane ICP with voxel-hash correspondence and
  KISS-style adaptive threshold, NDT, ICP information matrix (Ch. 16); factor-graph
  optimizer (GN/LM, robust kernels, sparse via `faer`, Ch. 15); pose graph with loop
  candidates, ICP verification and χ² gate; EKF-SLAM (Ch. 14) and grid RBPF (Ch. 17) as
  alternative back ends.
* `scan_matcher_node` (LiDAR odometry) and `pose_graph_node` (keyframes, loop closure,
  optimization, submap-rendered `/map`, `map → odom`). No faked loop closure: until the
  detector and verifier are implemented, the node reports `loop_closure: unsupported`.

* **Inherited from Phase 4:** the 2-D TSDF and quadtree (Ch. 19). Whether submap rendering needs a
  TSDF was to be decided here, not in Phase 4. **Decided 2026-10-11: it does not** (implementation.md
  DS13); a revisit needs one of the triggers listed there.

## Phase 6 — Planning

* `robotics_planning`: lattice A*/Dijkstra with octile heuristic, PRM, RRT/RRT* with
  `Steer` trait, Dubins words, C-space with ESDF clearance (Ch. 20); MDP value/policy
  iteration and POMDP (α-vectors, PBVI, POMCP) as decision layer (Ch. 21–22).
* `planner_node`: `/map` (+ `/esdf`) + `/goal_pose` → `/plan`; `NavigateToPose` action.

## Phase 7 — Control

* `robotics_control`: pure-pursuit baseline, DWA, MPPI with information-theoretic
  weights, cross term, Savitzky–Golay smoothing, ESDF obstacle cost (Ch. 23).
* `controller_node`: `/plan` + `/odom` + `/esdf` → `/cmd_vel`; safety envelope (max
  speeds, minimum clearance, stop on stale inputs) enforced before optimization.

## Phase 8 — Exploration

* `robotics_navigation`: frontier detection/clustering, ray-cast information gain,
  navigation cost field, utility scoring, stop rules (Ch. 24).
* `explorer_node` with an `Explore` action (feedback: coverage, entropy, gain rate).

## Phase 9 — Complete autonomy stack

* Supervisor state machine (`Explore | Navigate | Relocalize | Recover | Done`),
  dual-EMA kidnap detector, NIS divergence gate, watchdogs, chance-constrained safety
  margin (Ch. 26); mission manager; bringup for sim and for a real diff-drive base.
* Benchmarks (`criterion`): PF update, ICP, map update, planner, MPPI rollouts.

## Cross-cutting backlog

* **M1-extended (Phase 4):** a second, more complex Webots world (interior wall or
  furniture) so the map-accuracy claim means more than it does in the empty room. Deferred
  until the Phase 3 trials are finished, since it changes simulation assets they run on.
* **Known doc-drift, Phase 3 (not fixed):** the package table of `ARCHITECTURE.md` section
  1.3 still lists the Phase 3 node package as `robotics_localization` with
  `localization_node` "planned"; the package that was built is `robotics_localization_bridge`
  (`localization_node`, `mcl_node`, `route_driver`, `localization_evaluator`). To be corrected
  when Phase 3 closes.
* Lifecycle nodes: rclrs 0.8 has no lifecycle support; nodes expose `~/reset` services
  and rely on launch-level supervision until upstream adds it.
* Composition: rclrs has no component container; one process per node for now.
* Custom interfaces (`robotics_interfaces`) once phase 5 needs graph/landmark messages.
* TF: planar buffer only; a 3-D buffer when a 3-D consumer appears (`docs/TF_BUFFER.md`).
* Integration tests are binaries run by `scripts/integration_test_*.sh`: `colcon test`
  does not run Rust tests.
