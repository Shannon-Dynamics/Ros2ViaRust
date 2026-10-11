# Chapter 2: State Estimation: Fusing Wheels and a Gyro (design doc)

Implements the state-estimation half of Phase 2 of `docs/ROADMAP.md` (marked ✅, closed
2026-09-27). Written from `docs/STATE_ESTIMATION.md` (§§ 1–6), `docs/ROADMAP.md`,
`docs/SOURCE_MAPPING.md` (chapter rows 6 and 7, deviation 4 and the Phase 2 worked-example
table) and the source of `robotics_filters::{eskf_se2, fusion, consistency, information,
smoother}` and `robotics_core::prob::stats` as they stand in the repository. Nothing in it
comes from Phase 3. The sensor models and the simulator are Chapter 3; this chapter uses the
simulator (Rusty) as a black box that produces wheel angles, a gyro and the truth.

## 1. Purpose and story arc

**The problem.** The TurtleBot3's wheel odometry is accurate for a few seconds and wrong for a
few minutes, and it does not know it. Stated so a run can produce the number: drive Rusty's
180 s mixed trajectory (straight runs, turns in place, arcs, a spin across ±π, stops) with
per-step wheel slip of 2 %, a right wheel radius 0.2 % too large and a gyro bias of
0.005 rad/s. On the repository's recorded native run (seed 1), raw wheel odometry ends with
RMSE 0.710 m in position and 0.2564 rad in heading, and its own covariance is so small that
its mean pose NEES is 185.24 where a consistent filter averages 3. Fusing the gyro with an
error-state Kalman filter gives 0.152 m and 0.0476 rad, NEES 0.45, and a mean NIS of 0.90
where 2 is expected: better, and no longer lying about it (`docs/STATE_ESTIMATION.md` §4.1).

**Arc.** *Problem → what we try first and why it fails → the idea → the Rust → the number
again.* Dead reckoning drifts because the wheels carry a *systematic* error (a radius
mismatch) plus slip; w2.1 shows the path and the 3σ ellipse the raw odometry claims, which
the truth leaves within seconds. The obvious fix is the gyro, which measures rotation
directly; w2.2 shows that integrating it alone is worse than the wheels, because a gyro has
a bias. The idea is to let each sensor correct the other's weakness: the wheels observe the
bias (when the robot is nearly still, the gyro should read the same rate as the wheels), the
gyro observes heading. That is an error-state filter: a nominal pose on SE(2) that is
integrated exactly, a 5-dimensional error state (position, heading, speed, bias) that is
small enough for a Gaussian, an update that observes the error through the wheel/gyro
difference, and an injection-and-reset that moves the error back into the nominal state.
Then the question the book's rhythm demands: *does the answer deserve trust?* NEES needs
the truth (so only the simulator can compute it), NIS does not (so the robot can); both
are χ² statistics with known means. w2.3 shows the fused filter *below* the χ² band (conservative, on purpose) and the
raw odometry far above it, and shows the limits of both (toward a 2 % radius error even the
fused filter leaves the band, on every one of twelve seeds tried); w2.4 shows a policy
choice (when may the filter correct the bias?) that the repository's author measured and
that is worth more than a paragraph; w2.5 shows a timing cliff. The chapter ends on the
number again, for five seeds.

**Epigraph** (verified verbatim against the first page of the paper, read from a PDF of
*Transactions of the ASME, Journal of Basic Engineering* 82(1), 1960; it is item (2) of the
abstract): *"A nonlinear difference (or differential) equation is derived for the covariance
matrix of the optimal estimation error."* — R. E. Kalman, *A New Approach to Linear Filtering
and Prediction Problems* (1960). The chapter's point is the other half of that sentence's
promise: a filter carries a covariance of its own error, and that covariance can be wrong.

## 2. Prerequisites and position

Chapter 1 (SE(2), `⊞`/`⊟`, the Kalman filter on a cart, the Joseph form). Matrices and the
idea of a covariance. No ROS knowledge: the chapter is about `robotics_filters`, not about
`state_estimator_node`, which only converts messages into the calls shown here.

**Introduces:** `robotics_filters::{EskfSe2, PlanarState, ProcessNoise, OdomImuFusion,
FusionConfig, consistency::{nees, nis, ConsistencyStats}, InformationFilter, RtsSmoother}`;
`robotics_core::prob::chi2_mean_interval`; the Rusty simulator and `WheelOdometry` as
*sources of data* (their models are Chapter 3 and Chapter 1).
**Reuses:** `Se2`, `Manifold`, `Tangent2`, `KalmanFilter` (Chapter 1).

**Derived from:** `docs/STATE_ESTIMATION.md`; `docs/SOURCE_MAPPING.md` rows 6 and 7 and
deviation 4; `docs/ROADMAP.md` Phase 2.

**Does not cover (and says so):** the sensor models and the simulator's internals (Chapter 3);
the generic `Ekf`/`Ukf` over any `Manifold`; the particle set and KLD sizing (they belong to
the localization of Phase 3, which is not written here); the ROS node, QoS and TF ownership.

## 3. Foundation

Statement-first; the algebra in `<Derivation>` blocks.

1. **Why not a plain Kalman filter on `(x, y, θ)`.** Heading wraps; a covariance on a
   wrapped coordinate is wrong near ±π; and the motion `x ⊞ τ` is not linear in `(x, y, θ)`.
   The error-state filter integrates the *nominal* pose exactly on SE(2) (Chapter 1's `exp`)
   and keeps a Gaussian only over the *error*, which is small and lives in a vector space.
2. **State.** Nominal `x̂ = (T̂, v̂, b̂)`, `T̂ ∈ SE(2)`, forward speed `v`, gyro bias `b`. Error
   `δx = (δρ_x, δρ_y, δθ, δv, δb) ∈ ℝ⁵` with the *right* perturbation `T = T̂ · Exp(δξ)`,
   `δξ = (δρ_x, δρ_y, δθ)`; position errors are in the body frame of the estimate.
3. **Process.** Gyro `ω = ω_m − b̂` held between samples, `τ = (v̂ Δt, 0, ω Δt)`,
   `T̂ ← T̂ · Exp(τ)`. Error propagation `δx ← F δx + noise` with
   `F = [[Ad(Exp τ)⁻¹, Δt e_x, −Δt e_θ], [0, 1, 0], [0, 0, 1]]` (first order, `J_r(τ) ≈ I`
   for the short steps) and `Q = diag(σ_lat²Δt, σ_lat²Δt, σ_g²Δt, σ_a²Δt, σ_bw²Δt)`.
   Derivation of the `−Δt` coupling between `δb` and `δθ`: a bias error `δb` makes the
   integrated rate wrong by `−δb`, so heading error grows at `−δb Δt` per step.
4. **Measurement.** The wheel twist gives `v_w` and `ω_w`; the gyro gives `ω_g = ω + b +
   noise`. Define `z = (v_w, ω_w − ω_g)` and `h(x) = (v, −b)`. Then `H = [[0,0,0,1,0],
   [0,0,0,0,−1]]`, `R = diag(σ_v², σ_{ωw}² + σ_{ωg}²)`. The innovation `r = z − h(x̂)` and its
   covariance `S = H P Hᵀ + R`; Joseph form `P ← (I − KH) P (I − KH)ᵀ + K R Kᵀ`.
5. **Injection and reset.** `x̂ ← x̂ ⊞ δ̂`, and because the error is re-expressed at the new
   estimate, `P ← G P Gᵀ`, `G = diag(I₃ − ½ ad(δ̂ξ), 1, 1)`. Derivation in a `<Derivation>`.
6. **A policy, and why it is not in the equations.** The yaw-rate difference
   `ω_w − ω_g` is only trustworthy as a measurement of `−b` when the wheels are not wrong
   in a way that scales with motion. A radius mismatch is *correlated over many seconds*;
   white measurement noise cannot represent it. So the update uses the difference only while
   `|v| ≤ bias_update_max_speed = 0.05 m/s` (standstill and turns in place: a zero-velocity
   update) and observes speed alone otherwise. Recorded effect (`docs/STATE_ESTIMATION.md`
   §4.1, native run): updating the bias at every speed raises the fused error to 0.62–0.79 m /
   0.19–0.27 rad over the five seeds and drops the yaw ±3σ coverage to 42–89 %.
7. **Timing.** Measurements are integrated by their own stamps. A sample older than
   `max_measurement_delay = 50 ms` behind the filter is rejected; one within the window is
   applied at the filter time, without propagation; a gap above `max_dt = 0.5 s` is not
   integrated across, the covariance is inflated instead.
8. **Consistency.** NEES `ε = eᵀP⁻¹e` with `e = x_true ⊟ x̂` (needs truth); NIS `ν = rᵀS⁻¹r`
   (does not). For a consistent filter `ε ~ χ²_N`, `ν ~ χ²_M`, so `E[ε] = N`, `E[ν] = M`. The
   time average of `n` samples is tested against the two-sided interval
   `[χ²_{nN}(α/2)/n, χ²_{nN}(1−α/2)/n]`. **Worked example** (checked):
   `χ²₁(0.95) = 3.841459`, `χ²₂(0.95) = 5.991465` (test `prob::stats::tests`; asserted through
   `ch02_chi2_mean_band(dof, [1], 0.10)`). The interval assumes independent samples;
   successive filter outputs are correlated, so it is indicative. The chapter says so.
9. **The rest of the toolbox, on the Chapter 1 cart.** The information filter is the Kalman
   filter in canonical form; the RTS smoother runs a backward pass over a filter's stored
   beliefs. **Worked example** (checked, tests `information::tests::worked_example_1d_cart_agrees_with_kalman`
   and `smoother::tests::worked_example_1d_cart`): `Ω̄₁ = ξ̄₁ = 0.8`; posterior equal to the
   KF's (`μ = 2.280645161290323`, `Σ = 0.2741935483870968`); smoothed `μ₁|₂ = 85/62`,
   `Σ₁|₂ = 15/62`, `μ₀|₂ = 18.4/62`, `Σ₀|₂ = 22/62`.

**Worked numbers that carry the chapter, with where each is asserted.**

| Number | Source | Asserted by |
|---|---|---|
| The five-seed native table (raw/fused RMSE, bias, 3σ coverage, NEES, NIS) | `docs/STATE_ESTIMATION.md` §4.1 | `ch02::tests::the_documented_table_is_reproduced_natively` (`cargo test`) |
| Bias at every speed: 0.62–0.79 m, 0.19–0.27 rad, 42–89 % | same | same test |
| Noise-free 60 s: raw position 1.8·10⁻⁹ m, fused heading 2.7·10⁻⁸ rad, fused position 5.7 mm | same | `the_noise_free_numbers_are_reproduced` (`cargo test`) and `checks/ch02.ts` on the module |
| χ²₁(0.95), χ²₂(0.95) | `docs/SOURCE_MAPPING.md` | `checks/ch02.ts` |
| The cart in information form and smoothed | `docs/SOURCE_MAPPING.md` | `checks/ch02.ts` and `ch02::tests::toolbox_wrappers_reproduce_the_book_examples` |
| What the browser's realization of each seed gives | live in the widgets | `checks/ch02.ts` pins the module's five-seed table and its qualitative invariants |

## 4. Conceptual

In the order the reader meets them. **Every widget that draws noise says, in its caption, that
the browser's seed draws a different realization than the repository's recorded run** (finding
in §8), and quotes only what it shows.

* **w2.1, dead reckoning and the covariance it claims.** Play across 180 s. The truth
  (dashed), the raw wheel-odometry path, and the raw odometry's own 3σ position ellipse at the
  current time. One control: play/scrub. Kills "an odometry covariance tells you how far off
  the pose can be".
* **w2.2, a gyro alone.** Heading error against time for the wheels, for the raw gyro
  integrated with no bias estimate (a *strawman*, marked, not code the robot runs), and for
  the fused filter with its ±3σ band. One slider: the gyro's bias. Kills "a gyro measures
  rotation directly, so integrating it is enough".
* **w2.3, does the answer deserve trust?** Running mean NEES of the raw odometry and of the
  fused estimate on a log axis against the χ² acceptance band, plus the NIS means. One slider:
  the right wheel's radius error. Kills "a Kalman filter's covariance is right by
  construction, so there is nothing to test" (and, at radius 0, "fusing a second sensor
  always improves the estimate").
* **w2.4, when may the filter correct the bias?** A two-position control: standstill only
  (the repository's default) versus every speed. The bias estimate with its ±σ against the
  true bias, and the fused heading error with its ±3σ band and the share of samples inside it.
  Kills "the more often the filter may correct the bias, the better".
* **w2.5, a late wheel sample.** One slider: a constant delay on the wheel stream, 0 to 120
  ms. Counts of samples accepted, applied late and rejected, and the fused path against the
  truth. Kills "a late measurement is just a slightly worse measurement".

## 5. Practical

Excerpts printed (every non-blank line is checked to occur, in order, in the named file):

* `crates/robotics_filters/src/eskf_se2.rs`: `predict_inner` (the process model and `F`),
  `update_wheel_with_gyro` (the measurement), `update` (Joseph correction, injection, reset).
* `crates/robotics_filters/src/fusion.rs`: the arms of `on_wheel` that choose between the
  two-degree-of-freedom update, the speed-only update and wheel-only mode (the policy of
  §3.6).
* `crates/robotics_filters/src/consistency.rs`: `quadratic_form`, `ConsistencyStats::report`.
* `crates/robotics_core/src/prob/stats.rs`: `chi2_mean_interval`.
* A worked example with expected output as a `text` block: the cart in information form and
  smoothed (every number asserted by `checks/ch02.ts`).

## 6. Widget manifest

| Id | Component | Rust called (`robotics_wasm`) | Foregrounded parameter | Kills the misconception that |
|---|---|---|---|---|
| w2.1 | `DeadReckoning` (`dead-reckoning.tsx`) | `ch02_run` (Rusty → `WheelOdometry` → `OdomImuFusion`), `ch02_ellipse` | time (play / scrub) | an odometry covariance tells you how far off the pose can be |
| w2.2 | `GyroAlone` (`gyro-alone.tsx`) | `ch02_run` (`EskfSe2` through `OdomImuFusion`; the raw-gyro strawman is marked) | gyro bias | a gyro measures rotation directly, so integrating it is enough |
| w2.3 | `Trust` (`trust.tsx`) | `ch02_run` (`consistency::nees`, `ConsistencyStats`), `ch02_chi2_mean_band` (`chi2_mean_interval`) | right wheel radius error | a Kalman filter's covariance is right by construction, so there is nothing to test |
| w2.4 | `BiasPolicy` (`bias-policy.tsx`) | `ch02_run` (`FusionConfig::bias_update_max_speed`) | standstill only vs every speed | the more often the filter may correct the gyro bias, the better |
| w2.5 | `LateWheel` (`late-wheel.tsx`) | `ch02_run` (`FusionConfig::max_measurement_delay`, the `OdomImuFusion` stats) | wheel delay | a late measurement is just a slightly worse measurement |

## 7. Exercises

1. **F, difficulty 2.** Show that with the right perturbation a bias error `δb` produces the
   `−Δt` entry of `F` between `δθ` and `δb`. *Sketch:* `ω = ω_m − b̂ − δb`, so the heading
   increment is wrong by `−δb Δt`.
2. **F, difficulty 3.** Derive the reset Jacobian `G = I − ½ ad(δ̂ξ)` to first order. *Sketch:*
   `Exp(δξ⁺) = Exp(δ̂ξ)⁻¹ Exp(δξ)`; use the BCH formula to first order in `δ̂ξ`.
3. **C, difficulty 1.** Predict, then check in w2.2: with the gyro bias set to 0, which error is
   smaller at the end, the gyro's or the wheels'? *Sketch:* the gyro's; each option has a `because`.
4. **C, difficulty 2.** Predict, then check in w2.3: at radius error 0, is the raw odometry
   covariance consistent? Is the fused position better than the raw? *Sketch:* the raw NEES falls
   to 2.8–24 over seeds 1–8 (against over 100 at 0.2 %), the fused heading is worse than the
   wheels' on all eight, because the gyro adds a bias to learn that the wheels did not have.
5. **C, difficulty 2.** Predict, then check in w2.5: a constant wheel delay of 40 ms versus
   60 ms. *Sketch:* 40 ms is applied late and harmless; 60 ms is beyond the 50 ms window and
   every wheel sample is rejected.
6. **P, difficulty 2.** In `crates/robotics_filters/src/consistency.rs`, add a test that
   errors drawn from `N(0, P)` pass the interval and errors from `N(0, P/4)` are reported
   over-confident; compare with `samples_from_the_model_are_consistent`.

## 8. Modernization notes

* **Finding: a seed names a different noise stream on WebAssembly.** `robotics_core::seeded_rng`
  returns `rand` 0.10's `SmallRng`, which is Xoshiro256++ on 64-bit targets and Xoshiro128++
  on 32-bit ones. `seeded_rng(1)` yields `14971601782005023387` natively and
  `11103346041267129828` in the module (`ch02_seeded_rng_probe`, pinned both ways). Every
  seeded quantity therefore differs between the browser and the robot for the same seed.
  Consequence for this chapter: the repository's recorded run (0.710 m for seed 1) is quoted
  as the *native* result, `cargo test` asserts it, and the widgets show the browser's own
  realization (seed 1: raw 0.726 m, fused 0.455 m) and say so. What does not depend on a random
  stream (closed forms, the noise-free run) agrees on both targets. The qualitative results
  (fused heading beats the wheels on all five seeds; the raw NEES is two orders of magnitude
  over 3; the fused one sits near 1) hold on both and are asserted on both. Not fixed here:
  the fix is a fixed-algorithm generator in `robotics_core` that is bit-identical to
  `SmallRng` on 64-bit, which touches a crate Phase 3 depends on.
* The fused position error in the module's seed 1 (0.455 m) is larger than the recorded
  native seed 1 (0.152 m), and larger than any recorded native seed. The chapter does not
  explain this away: the five module seeds range 0.122–0.455 m, the five native seeds
  0.108–0.317 m, and the filter has no absolute position measurement, so its position error
  is a random walk whose size depends on the realization.
* The repository's raw odometry covariance is Phase 1's `OdometryNoise`, which models neither
  slip nor a radius error; `docs/SOURCE_MAPPING.md` (deviation 4) records that it is
  over-confident against the simulator. The chapter shows it, using it as the baseline.
* Known limitations, stated and not hidden (`docs/STATE_ESTIMATION.md` §6): no absolute
  measurement, so position drifts and heading drifts with the bias random walk; a
  wheel-speed latency of one sample (about 6 mm at a 0.3 m/s step); planar only; late samples
  are rejected, not re-ordered.
* Two toolbox filters (information form, RTS) are shown on the Chapter 1 cart only, because
  the robot does not run them yet; the chapter does not claim they are used by the node.
* The strawman in w2.2 (the raw gyro integrated with no bias estimate) lives in
  `robotics_wasm::ch02`; it is not code the robot runs.
