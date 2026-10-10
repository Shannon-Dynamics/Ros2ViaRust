# Source Mapping: *Probabilistic Robotics via Rust* → ros2-rust-robotics

Reference repository (read-only):
`D:\Shannon\ROS2 via RUST\ProbabilisticRoboticRust-main\ProbabilisticRoboticRust-main`.
It contains **no `.rs` files and no `Cargo.toml`**. "Rust" exists only as fenced listings
inside `web/content/chapters/chNN-*.mdx` (about 140 blocks, ~8 800 lines); the runnable
implementation is a TypeScript port under `web/lib/` (~31 000 lines) pinned by numerical
checks in `web/lib/__checks*.ts`. The design docs are `Chapter-NN.md`. The repository has
no license file; nothing from it is copied (see `NOTICE`).

Status legend for the book's Rust: **C** complete listing · **F** fragment (helpers
elided) · **P** prose/pseudocode only · **N** none (TypeScript only).

## Chapter → crate / node

| Ch. | Topic | Book Rust | TS port | Framework destination | Phase | Status |
|---|---|---|---|---|---|---|
| 1 | Discrete belief, argmax fallacy, dead reckoner | C | widgets only | `robotics_filters::histogram` (concept), `robotics_models::motion::velocity::predict_exact` | 1 | ✅ (exact arc) |
| 2 | Gaussians (moments/canonical), sampling, entropy | C | `prob/*.ts` | `robotics_core::prob::{Gaussian, Canonical, seeded_rng}` | 1 | ✅ |
| 3 | SE(2) exp/log/adjoint, ⊞/⊟, frame newtypes | C | `geom/se2.ts`, `screw.ts` | `robotics_core::se2::{Se2, Manifold, Tangent2}`, `robotics_core::angle` | 1 | ✅ |
| 4 | Diff-drive kinematics, encoders, odometry; World/LiDAR sim; log/replay | F (World stubs, `Lidar::scan` N) | `sim/*.ts` | `robotics_models::diff_drive` ✅; `robotics_core::types::LaserScan` (+ `validate`) ✅; `robotics_sim::{RustySim, World, Lidar}` ✅ → `sim_node` (`/scan`, `/joint_states`, `/imu/data`, ground truth) ✅. Log/replay = seeded bit-exact re-simulation; recording is `ros2 bag` | 1–2 | ✅ |
| 5 | `BayesFilter` trait, histogram filter | C (not compilable as printed) | `filters/bayes.ts` | `robotics_filters::{bayes, histogram}` | 1 | ✅ |
| 6 | KF (Joseph), information filter, RTS, NEES/NIS | C | `filters/kf.ts`, `info.ts`, `consistency.ts` | `robotics_filters::{kalman, information, smoother, consistency}` ✅; χ² quantiles in `robotics_core::prob::stats` | 1–2 | ✅ |
| 7 | EKF/UKF generic over `Manifold`, ESKF on SE(2) | C (UKF filter struct N, ESKF F) | `ekf.ts`, `ukf.ts`, `on-manifold-se2.ts` | `robotics_filters::{ekf, ukf, eskf_se2, fusion}` ✅ → `state_estimator_node` (`/odom`, `/state_estimate`, `odom → base_link`) ✅ | 2 | ✅ |
| 8 | Particle set (log weights), resamplers, KLD, log-odds cell | C | `pf.ts` (Pose2-specific), `kld.ts` | `robotics_filters::particle` (generic `ParticleSet<S>`, low-variance + multinomial, ESS) ✅; `robotics_filters::kld` (bound, `kld_resample`, `ParticleBinning` with `GridBinning`/`Se2Binning`/`BinFn`) ✅; log-odds cell → Phase 4 with the occupancy grid (`robotics_mapping`, Ch. 13) | 2 | ✅ (log-odds: 4) |
| 9 | Velocity/odometry motion models, tangent-noise model | C (`NoiseKind` F, `MapConditioned` P) | `models/motion*.ts` | `robotics_models::motion::{velocity, odometry}` ✅; tangent model → `WheelOdometry` covariance compounding ✅ | 1 | ✅ |
| 10 | Beam model + EM, likelihood field (exact EDT), landmark model, tempering | C/F (`rasterize` N) | `models/sensor.ts`, `beam-em.ts`, `mapping/edt.ts` | `robotics_models::sensor::{beam (+ `em_step`/`fit_em`), likelihood_field (FH exact EDT), landmark, tempering}` ✅; ray casting for `z*` in `robotics_sim::World`; rasterizing a `World` into the EDT → Phase 3 with MCL | 2 | ✅ |
| 11 | EKF localization, ML association, χ² gates, MHT | C (MHT F) | `filters/ekf-localization.ts` | `robotics_localization::ekf` → `localization_node` | 3 | planned |
| 12 | MCL, Augmented MCL (`w_fast/w_slow`), grid localization, novelty filter | C (grid F, KLD-MCL P) | `localize/*.ts`, MCL loop in widget | `robotics_localization::mcl` → `localization_node` (owns `map→odom`) | 3 | planned |
| 13 | Occupancy grid (log odds), inverse models, MAP mapping, fusion | C (learned/MAP F) | `mapping/occgrid.ts`, `map-occgrid.ts` | `robotics_mapping::occupancy` → `mapping_node` | 4 | planned |
| 14 | EKF-SLAM with provisional landmarks | C | `slam/ekf-slam.ts` | `robotics_slam::ekf_slam` (pedagogical back end) | 5 | planned |
| 15 | Factor graphs, GN/LM, robust kernels, Schur, ordering, factrs | C/F (sparse helpers N) | `optim/*.ts` (dense) | `robotics_slam::factor_graph` (dense first, `faer` sparse second) | 5 | planned |
| 16 | ICP/NDT, voxel map, pose graph, loop closure, RustSLAM-2D | C/F (`Slam2d`, `Submap`, loop detect N) | `slam/icp.ts`, `ndt.ts`, `posegraph.ts`, `slam2d.ts` | `robotics_slam::{icp, ndt, pose_graph, pipeline}` → `scan_matcher_node`, `pose_graph_node` | 5 | planned |
| 17 | FastSLAM 1.0/2.0, grid RBPF | C (helpers N) | `filters/fastslam.ts`, `rbpf.ts` | `robotics_slam::{fastslam, rbpf}` | 5 | planned |
| 18 | Pinhole, tiny BA, IMU preintegration, sliding window | C/F | `vision/*.ts` | `robotics_slam::vio` (library only; no camera front end in the book) | later | deferred |
| 19 | Quadtree, TSDF, ESDF, marching squares | C (quadtree N) | `mapping/tsdf.ts`, `esdf.ts`, `quadtree.ts` | `robotics_mapping::esdf` (Phase 4); `{tsdf, quadtree}` moved out of Phase 4 by decision, a Phase 5+ concern | 4 (ESDF), 5+ (TSDF, quadtree) | planned |
| 20 | C-space, A*/Dijkstra, PRM, RRT/RRT*, Dubins, potentials | C (Dubins bodies N, hybrid A* P) | `plan/*.ts` | `robotics_planning::{search, sampling, dubins, cspace}` → `planner_node` | 6 | planned |
| 21 | MDP value/policy iteration, gridworld | C | `decision/*.ts` | `robotics_planning::decision::mdp` | 6 | planned |
| 22 | POMDP (α-vectors, PBVI, POMCP), AMDP | C/F (POMCP helpers N) | `pomdp/*.ts` | `robotics_planning::decision::pomdp` | 6 | planned |
| 23 | MPPI, DWA, track-and-clear cost | C (DWA N) | `control/mppi.ts`, `dwa.ts` | `robotics_control::{mppi, dwa}` → `controller_node` | 7 | planned |
| 24 | Frontiers, info gain, graph log-det, utility, stop rules, active localization | C/F | `explore/*.ts` | `robotics_navigation::explore` → `explorer_node` (+ `Explore` action) | 8 | planned |
| 25 | Calibration, differentiable KF, soft resampling, temperature fit (candle beam net F) | C/F | `learn/*.ts` | `robotics_filters::diagnostics` (calibration, soft resample); learned models behind a feature flag | later | deferred |
| 26 | Capstone: bus/tasks, supervisor, detectors, safety margin, mission | C/F (task impls N) | `capstone/stack.ts` | `robotics_navigation::{supervisor, safety}` → `supervisor_node`; ROS node graph replaces the in-process bus | 9 | planned |

## Conventions carried over (and where they are enforced)

| Convention | Source | Enforced in |
|---|---|---|
| Tangent order `(v_x, v_y, ω)`, right perturbation `x ⊞ δ = x·exp(δ)` | Ch. 3, 7 | `robotics_core::se2` (tests: pinned round-trip set, adjoint identity) |
| Angles in `(−π, π]` | Ch. 3 | `robotics_core::angle::normalize_angle` |
| Thrun naming `R` = motion noise, `Q` = measurement noise (the TS port uses control-theory `Q/R`; the framework does **not**) | Ch. 6 | `robotics_filters::kalman` docs |
| Velocity-model noise variances in squared form `α₁v² + α₂ω²`, `alphas[0..6]` | Ch. 9 | `robotics_models::motion::velocity` |
| Odometry decomposition `(δ_rot1, δ_trans, δ_rot2)`, pure spin → `rot2` | Ch. 9 | `robotics_models::motion::odometry` |
| Exact constant-curvature arc for diff-drive integration | Ch. 4 | `robotics_models::diff_drive` (`zero_noise_dead_reckoning_is_exact`) |
| Seeded `SmallRng` everywhere, `BOOK_SEED = 0x5EED_2026` | Ch. 2, 4 | `robotics_core::prob::rng` |
| Evidence returned by `correct` | Ch. 5 | `BayesFilter::correct -> Result<f64>` |
| Valid beam = finite range within `[range_min, range_max]`; max-range/NaN kept but skipped | Ch. 4, 10 | `LaserScan::valid_beams` |

## Deliberate, documented deviations

1. **Velocity-model density sign** (`VelocityMotionModel::prob`): Thrun's Table 5.1 sets
   `v̂ = Δθ·r*/Δt`, which has the wrong sign for right-hand turns; the framework uses
   `v̂ = |Δθ|·r*/Δt · sign(forward progress)` so the density peaks at the sampler's mean
   for every turn direction (tested for ω ∈ {+0.6, −0.6, 0}). The TS port keeps the
   verbatim form.
2. **`BayesFilter::predict` returns `Result`** and `correct` returns `Result<f64>`; the book's
   trait cannot fail. Production nodes must be able to reject a bad control or measurement.
3. **`HistogramFilter1D::mode`** returns the lowest index on ties (deterministic).
4. **Odometry covariance**: the book's simulator uses per-wheel multiplicative slip; the
   node publishes a covariance grown by a simple per-step model (`OdometryNoise`),
   compounded with `Ad(Δ⁻¹)` as in the Ch. 9 tangent-noise model. `robotics_sim` now
   simulates slip; measured against it the Phase 1 covariance is over-confident (pose NEES
   145–291, `docs/STATE_ESTIMATION.md` § 4) and is kept unchanged as the raw-odometry
   baseline.
5. **LiDAR range encoding**: the book's simulator reports `z_max` for no return and clamps
   to `[0, z_max]`; the framework's default follows REP 117 (`+inf` no return, `−inf` too
   close) because that is what `sensor_msgs/LaserScan` consumers expect. The book's
   encoding is available as `NoReturnPolicy::MaxRange`. The beam model treats non-finite
   ranges as max-range readings, so both encodings score identically.
6. **Ray casting without `parry2d`**: analytic ray–segment intersection over all segments
   instead of a BVH (21 segments; avoids a second `nalgebra` version). See
   `docs/SIMULATION.md` § 7.
7. **Landmark signature**: the book's continuous signature `s` with variance `σ_s²` is a
   discrete landmark id (known correspondence); `id = None` (unknown correspondence) is an
   error, not a guess — data association is Chapter 11.
8. **EM for the beam model**: Table 6.2 is implemented unmodified, including the
   untruncated M-steps the book itself flags (Ch. 10, step 6). Consequently the data
   log-likelihood is not strictly monotone: the tests measured dips of ~10⁻⁴ nats/beam near
   convergence and assert a tolerance instead of strict monotonicity.
9. **KLD sampling** separates proposal and weighting: `kld_resample` draws, proposes and
   bins; the caller applies the measurement update afterwards (Table 8.4 does both in one
   loop). The particle machinery thereby never sees a sensor model.
10. **Information filter evidence**: `correct` returns `N(z; Cμ̄, CΣ̄Cᵀ + Q)` for a proper
    prior and `0` (the limit) for an improper, zero-information prior, which the
    canonical form can represent and moments form cannot.
11. **Tempering** additionally offers `Tempering::for_target_ess` (largest `β` keeping the
    ESS above a target, the annealed-particle-filter rule); the book calibrates `κ` offline
    only.

## Worked examples reproduced as tests (phase 2)

| Book example | Test |
|---|---|
| Ch. 6 cart in information form: `Ω̄₁ = ξ̄₁ = 0.8`, evidence `N(1.7; 1, 1.75)`, posterior equal to the KF (`μ = 2.280645161290323`, `Σ = 0.2741935483870968`) | `information::tests::worked_example_1d_cart_agrees_with_kalman` |
| Ch. 6 cart, RTS by hand: `μ₁|₂ = 85/62`, `Σ₁|₂ = 15/62`, `μ₀|₂ = 18.4/62`, `Σ₀|₂ = 22/62` | `smoother::tests::worked_example_1d_cart` |
| Ch. 8 KLD table (ε = 0.05, δ = 0.01): k = 3 → 93, 10 → 217, 100 → 1347, 500 → 5755 | `kld::tests::worked_example_ch8_table` |
| Ch. 10 EM, one iteration on Z = {4.5, 5, 3, 7, 10}: z = (0.394282, 0.127210, 0.2, 0.278508), σ_hit = 1.093905, λ = 0.283737 (book, 6-decimal intermediates; 0.2837390 in full precision), mean LL −2.275038 | `beam::tests::worked_example_ch10_em_iteration` |
| Ch. 10 landmark model: bearing residual wrapped across ±π; Table 6.5 poses reproduce the observation | `landmark::tests::*` |
| Ch. 4 LiDAR: hand-computed wall distances in the arena at two poses; dropout does not reshuffle surviving noise | `lidar_regression.rs`, `lidar::tests::*` |
| χ² quantiles: χ²₁(0.95) = 3.841459, χ²₂(0.95) = 5.991465, χ²₁₀₀(0.99) = 135.8067 | `prob::stats::tests::*` |

## Worked examples reproduced as tests (phase 1)

| Book example | Test |
|---|---|
| `exp((1, 0, π/2)) = (2/π, 2/π, π/2)` | `se2::tests::worked_example_quarter_arc` |
| Pinned exp/log round-trip set, adjoint identity, square dance closes | `se2::tests::*` |
| `N(5,4)·N(6.5,1) = N(6.2, 0.8)`; 2-D `det Σ = 0.39`, entropy gain `1.16395145` nats | `prob::gaussian::tests::*` |
| `ℓ = 0.16, Δs_L = 0.72, Δs_R = 0.88 → (0.673176788, 0.367758155, 1.0)` | `diff_drive::tests::worked_example_wheel_arcs` |
| `v = 1, ω = 0.5, Δt = 1 → (0.958851077208, 0.244834876219, 0.5)` | `motion::velocity::tests::worked_example_exact_arc` |
| Odometry `apply(decompose(a, b)) = b`, pure spin in `rot2` | `motion::odometry::tests::decomposition_round_trip` |
| 4-cell hallway: evidence `0.5`, belief `(0.1, 0.4, 0.1, 0.4)`; entropy `log₂100` | `histogram::tests::*` |
| 1-D cart KF: `μ = 1.5, Σ = 2.5/7` then `μ = 2.280645161290323, Σ = 0.2741935483870968` | `kalman::tests::worked_example_1d_cart` |
| Scalar KF `K = 0.5`, predict → `P = 1.5` | `kalman::tests::scalar_update_then_predict` |
