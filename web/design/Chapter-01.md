# Chapter 1: Foundation: Poses, Odometry and the First Filters (design doc)

Implements Phase 1 of `docs/ROADMAP.md` (marked ✅). Written from the Phase 1 section of the
roadmap, the "What is in Phase 1" section of `README.md`, `docs/SOURCE_MAPPING.md` (chapter
mapping, conventions, and the Phase 1 worked-example table) and the source of
`robotics_core`, `robotics_models` and `robotics_filters` as it stands in the repository.
Nothing in this document comes from the Phase 2 or Phase 3 results.

## 1. Purpose and story arc

**The problem.** The TurtleBot3 has two wheel encoders and no idea where it is. Every pose it
will ever use is something it computed, so the first question is how a pose is represented and
how a motion changes it. Stated so a run can produce the number: drive the robot on a circle
of radius 2 m (`v = 1 m/s`, `ω = 0.5 rad/s`) for 12 s in `n` integration steps, and measure
how far the integrated pose ends from the true one. The exact step ends within 1e-12 m for
every `n`; the textbook Euler step is off by 0.2134 m at `n = 8` (`Δt = 1.5 s`) and 0.4374 m at
`n = 4` (`Δt = 3 s`).

**Arc.** *Problem → what we try first and why it fails → the idea → the Rust → the number
again.* The first attempt is the Euler step, which every reader would write; it fails
systematically (its poses lie on a larger, shifted circle), not randomly. The idea is that a
planar pose is an element of a group, SE(2), and that a body-frame twist applied for a step
is `exp` of that twist: the exact constant-curvature arc. That one operation is the odometry
step (`⊞`), the motion model, and the retraction every later filter uses. Real wheels are
noisy, so the same step becomes a *distribution* of poses (the velocity motion model), and the
distribution has a shape a single number cannot carry. Two representations of that
distribution follow, in order of how much they assume: a histogram over a corridor (the Bayes
filter, assumption-free, exponential in dimension) and a Gaussian (the Kalman filter, two
numbers, exact only if everything is linear and Gaussian). The chapter ends on the number
again: the Kalman gain of the worked cart example, 5/7, and the worked wheel-arc example.

**Epigraph** (verified against the MIT Press catalog description of the book as repeated by
several library and bookseller pages; it is the publisher's description, not a sentence from
the body of the text, and the chapter says so):
*"Probabilistic robotics is a new and growing area in robotics, concerned with perception and
control in the face of uncertainty."*

## 2. Prerequisites and position

Calculus, matrices, and the idea of a probability density. No ROS knowledge; the chapter is
about the two crates underneath ROS, not about the nodes.

**Introduces:** `robotics_core::{Se2, Tangent2, Manifold, Gaussian}`;
`robotics_models::diff_drive::{DiffDriveParams, WheelOdometry}`;
`robotics_models::motion::{VelocityMotionModel, VelocityCmd}`;
`robotics_filters::{BayesFilter, HistogramFilter1D, KalmanFilter}`.
**Reuses:** nothing. This is the first chapter.

**Derived from:** `docs/ROADMAP.md` Phase 1; `docs/SOURCE_MAPPING.md` chapter rows 1, 2, 3, 5, 9
and the parts of 4 and 6 that Phase 1 built; `README.md` "What is in Phase 1".

**Does not cover (and says so):** the odometry noise model's covariance bookkeeping beyond one
sentence, the odometry motion model (rot1, trans, rot2), the information filter, and
everything in Phase 2. The Phase 1 `/odom` covariance is over-confident against simulated
wheel slip, which `docs/SOURCE_MAPPING.md` (deviation 4) records; the chapter states that and
defers the measurement to a later chapter without quoting its numbers.

## 3. Foundation

Statement-first. Definitions and results, with the algebra in `<Derivation>` blocks.

1. **A pose** `x = (x, y, θ)` is an element of SE(2); composition `x₁ x₂` applies `x₂` in the
   body frame of `x₁`; the inverse; angles in `(−π, π]`.
2. **The exponential map.** For a body twist `τ = (v_x, v_y, ω)`:
   `exp(τ) = (a v_x − b v_y, b v_x + a v_y, ω)` with `a = sin ω / ω`, `b = (1 − cos ω) / ω`.
   Derivation in a `<Derivation>`. **Worked example** (checked): `exp((1, 0, π/2)) = (2/π, 2/π,
   π/2)`; test `se2::tests::worked_example_quarter_arc`; closed form asserted in `checks/ch01.ts`.
3. **`⊞` and `⊟`.** `x ⊞ δ = x · exp(δ)`; `y ⊟ x = log(x⁻¹ y)`. The exact step is
   `x ⊞ (v Δt, 0, ω Δt)`.
4. **Wheels to a twist.** `Δs = (Δs_R + Δs_L)/2`, `Δθ = (Δs_R − Δs_L)/ℓ`. **Worked example**
   (checked): `ℓ = 0.16, Δs_L = 0.72, Δs_R = 0.88 → Δs = 0.80, Δθ = 1.0 → (0.673176788,
   0.367758155, 1.0)`; test `diff_drive::tests::worked_example_wheel_arcs`; closed form
   `(Δs/Δθ) sin Δθ`, `(Δs/Δθ)(1 − cos Δθ)` asserted.
5. **Why Euler fails, exactly.** With heading updated after position, the Euler vertices are
   `p_N = v Δt (1 − e^{iNφ}) / (1 − e^{iφ})`, `φ = ω Δt`: a regular polygon inscribed in a circle of
   radius `v Δt / (2 sin(φ/2)) = r (φ/2)/sin(φ/2)`, `r = v/ω`. Its error is bounded and
   systematic, not a spiral. Numbers (checked, closed form): 2.048 m at `Δt = 1.5 s`
   against `r = 2 m`.
6. **Noise.** The velocity motion model perturbs `(v, ω)` and adds a final-heading
   perturbation `γ`, with variances `α₁v² + α₂ω²`, `α₃v² + α₄ω²`, `α₅v² + α₆ω²`. **Worked
   example** (checked): `v = 1, ω = 0.5, Δt = 1 → (0.958851077208, 0.244834876219, 0.5)`,
   which is `exp((1, 0, 0.5))`; closed form `2 sin 0.5`, `2 (1 − cos 0.5)`.
7. **Belief and the Bayes filter.** `bel̄ = ∫ p(x_t | u_t, x_{t−1}) bel(x_{t−1})`,
   `bel = η p(z | x) bel̄`; `correct` returns the evidence `1/η`. **Worked example**
   (checked): four 1 m cells, likelihood (0.2, 0.8, 0.2, 0.8): evidence 0.5, belief
   (0.1, 0.4, 0.1, 0.4); a uniform belief over 100 cells has entropy `log₂ 100` bits.
8. **Kalman filter.** The scalar case derived as the product of two Gaussians
   (`N(5,4)·N(6.5,1) ∝ N(6.2, 0.8)`, checked), the gain `K = Σ̄/(Σ̄ + Q)`, the Joseph form.
   **Worked example** (checked, test `kalman::tests::worked_example_1d_cart`): prior `N(0,1)`,
   `u = 1, z = 1.7, R = 0.25, Q = 0.5` gives `μ̄ = 1, Σ̄ = 1.25, S = 1.75, K = 5/7, μ = 1.5,
   Σ = 2.5/7`; a second step with `u = 1, z = 2.1` gives `μ = 2.280645161290323,
   Σ = 0.2741935483870968`.

## 4. Conceptual

In the order the reader meets them.

* **w1.1, the exact arc.** One slider (number of steps in 12 s). Exact poses sit on the dashed
  circle at any step length; Euler poses sit on a larger, shifted circle. Kills "short straight
  steps are good enough, so the step length does not matter".
* **w1.2, one command, many outcomes.** One slider (noise as a multiple of the book's
  `VelocityAlphas::DEFAULT`), a second for the number of commands in a row, a visible seed.
  300 robots sample the repository's velocity motion model. Kills "noise blurs the pose
  symmetrically around the exact end point"; the cloud is a curved band along the arc.
* **w1.3, the hallway.** One slider (motion noise σ, true and assumed), play/pause/step, a
  visible seed. A `HistogramFilter1D` over 100 cells; the belief after the move (orange) and
  after the sensor reading (purple), the truth (dashed), the most probable cell (tick).
  Kills "the filter's answer is its most probable cell".
* **w1.4, one Kalman step.** One slider (measurement noise `Q`). The prior, prediction,
  measurement and posterior of the worked cart example. Kills "the filter splits the
  difference between prediction and measurement half and half".

## 5. Practical

Excerpts printed (every non-blank line is checked to occur, in order, in the named file):

* `crates/robotics_core/src/se2.rs`: `Se2::exp` (the core type's central operation) and the
  `Manifold<3>` impl (`boxplus`, `boxminus`).
* `crates/robotics_models/src/diff_drive.rs`: `wheel_arcs_to_tangent` and `WheelOdometry::apply`
  (dead reckoning with covariance compounding).
* `crates/robotics_models/src/motion/velocity.rs`: `VelocityMotionModel::sample` (Table 5.3).
* `crates/robotics_filters/src/bayes.rs`: the `BayesFilter` trait.
* `crates/robotics_filters/src/histogram.rs`: `HistogramFilter1D::correct`.
* `crates/robotics_filters/src/kalman.rs`: `KalmanFilter::predict` and `correct`.
* A worked example with its expected output as a `text` block (the cart, two steps), every
  number of which is asserted in `checks/ch01.ts`.

## 6. Widget manifest

| Id | Component | Rust called (`robotics_wasm`) | Foregrounded parameter | Kills the misconception that |
|---|---|---|---|---|
| w1.1 | `ExactArc` (`exact-arc.tsx`) | `ch01_integrate_twist` (exact: `Se2::boxplus`; Euler: strawman) | steps in 12 s | integrating a velocity in short straight steps is good enough, so the step length does not matter |
| w1.2 | `NoisyArc` (`noisy-arc.tsx`) | `ch01_velocity_samples` (`VelocityMotionModel::sample`), `ch01_integrate_twist` | noise scale | noise only blurs the dead-reckoned pose symmetrically around the exact end point |
| w1.3 | `Hallway` (`hallway.tsx`) | `ch01_hallway` (`HistogramFilter1D::{predict, correct}`), `ch01_hallway_doors` | motion noise σ | the filter's answer is its most probable cell, and one number is all it needs to carry |
| w1.4 | `KalmanStep` (`kalman-step.tsx`) | `ch01_kf_step` (`KalmanFilter::{predict, correct}`) | measurement noise `Q` | the filter splits the difference between what it predicted and what it measured, half and half |

## 7. Exercises

1. **F, difficulty 2.** Show that the Euler vertices lie on one circle and find its radius.
   *Sketch:* the sum is geometric, `p_N = v Δt (1 − e^{iNφ})/(1 − e^{iφ})`, so
   `|p_N − c|` is constant for `c = v Δt/(1 − e^{iφ})`; `|1 − e^{iφ}| = 2 sin(φ/2)`.
2. **C, difficulty 1.** Predict, then check in w1.4: with `Q` very large, where is the
   posterior? *Sketch:* `K → 0`, the posterior is the prediction; each wrong option states why.
3. **C, difficulty 2.** Predict, then check in w1.3: after the detector says "door" once, how
   many bumps does the belief have? *Sketch:* one per door, because the sensor cannot tell them apart.
4. **F, difficulty 2.** Derive `K = Σ̄/(Σ̄ + Q)` and `Σ = (1 − K) Σ̄` by multiplying two
   Gaussians. *Sketch:* add the precisions, weight the means by them.
5. **P, difficulty 1.** In `crates/robotics_models/src/diff_drive.rs`, turn a wheel-arc pair
   into a pose with `wheel_arcs_to_tangent` and `Se2::exp`, and check your result against
   the 0.72/0.88 example.
6. **P, difficulty 2.** Add a test to `crates/robotics_filters/src/histogram.rs` that a wrapped
   `HistogramFilter1D` moves a one-cell belief by whole cells under `predict` and ends where it started
   after the corridor's length in cells. *Sketch:* correct with a one-cell likelihood, then `predict(&1.0)`
   `n` times; compare with `exact_shift_moves_mass`.

## 8. Modernization notes

* The repository's velocity model integrates the exact constant-curvature arc (`VelocityCmd::predict_exact`
  and `VelocityMotionModel::integrate` in `velocity.rs`; straight-line only below `|ω| < 1e-6`), so a
  reader of this chapter never has to choose a step length for accuracy. The Euler step exists
  only in `robotics_wasm::ch01` as the strawman w1.1 measures.
* Deviations recorded in `docs/SOURCE_MAPPING.md` that this chapter repeats: `BayesFilter::predict`
  and `correct` return `Result` (the book's trait cannot fail); `HistogramFilter1D::mode` returns
  the lowest index on ties, which is why w1.3 shows an arbitrary most probable cell on a flat
  belief; the velocity-model density's sign convention (not exercised here, only `sample` is).
* **Known limitation, stated in the chapter and not hidden:** the Phase 1 odometry covariance is
  over-confident when the wheels slip (`docs/SOURCE_MAPPING.md`, deviation 4). It is kept unchanged as
  the raw-odometry baseline.
* The scaffold's first widget caption said Euler "spirals outwards" and its step count did not
  guarantee `steps × Δt = T`; both were wrong and are fixed here (found by computing the
  Euler vertices in closed form; `ch01::tests::euler_vertices_lie_on_a_larger_off_centre_circle`).
* Two widgets (w1.3, w1.4) use 1-D problems, because the full-dimensional versions of the same
  filters are the subject of Phase 2 and are not claimed here.
