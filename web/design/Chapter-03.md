# Chapter 3: Sensor Models and the Simulator (design doc)

Implements the sensor-model and simulation half of Phase 2 of `docs/ROADMAP.md` (marked ✅,
closed 2026-09-27). Written from `docs/SIMULATION.md` (§§ 1–7), `docs/SOURCE_MAPPING.md` (chapter
rows 4 and 10, deviations 5–8, 11, and the Phase 2 worked-example table) and the source of
`robotics_sim::{world, lidar, rusty}` and `robotics_models::sensor::{beam, likelihood_field,
landmark, tempering}` as they stand in the repository. Nothing in it comes from Phase 3: the
sensor models are shown being *scored*, never used to localize a robot, which is the work of
Phase 3 and is not written here.

## 1. Purpose and story arc

**The problem.** Chapter 2 ended with a pose that drifts. To correct it the robot has a lidar:
360 numbers ten times a second. A number means nothing until a model says how probable it is, and
localization (Phase 3) needs exactly that: `p(z | x, m)`, the probability of a whole scan `z` if
the robot were at pose `x` in map `m`. Stated so a run can produce the numbers: take one scan at the
true pose and score it at poses slid away from it; the score must peak at the truth and its peak
must be sharp enough to localize with (the beam model's peak is within 1 cm of the truth on every
pose and seed the build tries, and the likelihood field's within 3 cm and about twice as wide, never under 1.7 times);
and the posterior built from those scores must not claim more than the scan justifies (the field's
claim is over-confident at the rms error/σ ratio of about 1.4, until it is tempered).

**Arc.** *Problem → what we try first and why it fails → the idea → the Rust → the number
again.* Producing a scan comes first, because the book needs a sensor it can replay from a seed:
the Rusty simulator casts 360 rays exactly against a map of walls, adds Gaussian range noise, drops
a beam with a small probability, and encodes "nothing there" as `+inf`. A beam that returns nothing
is the first misconception (w3.1): it can be a wall farther than the sensor sees or a dropout, and
only a map can tell. Replay is the second property (w3.2): one random stream per noise source, so adding
a sensor changes none of the others, and a seed is a recording. Scoring comes next. The first thing one
tries is a Gaussian around the expected range, which is the heart of the *beam model*; but a real range
sensor also reads short (something in front of the wall), reads its limit and reads garbage, so the model is a
four-part mixture (w3.3), and its six parameters are learned by expectation–maximization from
readings and their expected ranges instead of tuned by hand (w3.4). The beam model needs a ray cast per
beam per hypothesis, so a cheaper score is built on a distance map: the *likelihood field* (w3.5), whose
price is a wider peak and a map error that biases every beam together. Beams multiplied as if independent make
the posterior too sharp, and tempering, a power `β ≤ 1` on the likelihood, repairs it without moving the peak (w3.6).
A landmark sensor closes the chapter (w3.7): a range and a bearing, and the one bug that bites angles, the
difference across ±π.

**Epigraph** (verified verbatim: read in the text of the first page of the paper, an open-access
copy of the ICRA 1999 paper; it is the first sentence of its abstract): *"To navigate reliably in indoor
environments, a mobile robot must know where it is."* — F. Dellaert, D. Fox, W. Burgard and S. Thrun,
*Monte Carlo Localization for Mobile Robots* (1999).

## 2. Prerequisites and position

Chapter 1 (SE(2), `⊞`, the idea of a likelihood) and the first page of Chapter 2 (a seed names
different noise on the page than on the robot: this chapter meets the same finding in the simulator and
refers to it, it does not re-derive it). Gaussians and the idea of a mixture; no ROS.

**Introduces:** `robotics_sim::{World, Lidar, LidarConfig, NoReturnPolicy, RustySim}` (as a source of scans);
`robotics_models::sensor::{BeamModel, BeamModelParams, DistanceGrid, LikelihoodFieldModel, LandmarkModel,
Tempering}`.
**Reuses:** `Se2`, `Manifold`; `robotics_filters::particle::effective_sample_size` (the ESS, as a number
a tempered posterior has).

**Derived from:** `docs/SIMULATION.md`; `docs/SOURCE_MAPPING.md` rows 4 and 10 and deviations 5, 6, 8 and 11.

**Does not cover (and says so):** using any model to localize (Phase 3: EKF and Monte Carlo
localization); correlation-based models; unknown data association (the landmark model needs the landmark's id);
the ROS node `sim_node` and its QoS; the moving-obstacle and collision models Rusty does not have.

## 3. Foundation

1. **The forward model of the lidar.** The sensor is at `T_sensor = x ∘ T_base_laser` (mount `(0.05, 0, 0)`).
   Beam `k` has bearing `φ_k = −π + k·1°`. Its ideal range is the first hit of the ray with any segment:
   with ray direction `d` and segment `a + s e`, `t = (a − o) × e / (d × e)`, `s = (a − o) × d / (d × e)`, a hit if
   `t ≥ 0` and `s ∈ [−ε, 1 + ε]`, the smallest such `t`, if within 8 m. The measurement is `z = z* + σ n`, `n ~ N(0, 1)`,
   with no return if a uniform draw is below the dropout probability or `z*` does not exist. **Worked numbers**
   (checked, no random stream, `tests/lidar_regression.rs` and `checks/ch03.ts`): from the origin facing +x with no
   noise, beam 180 reads 7.45 m, beam 0 reads 5.55 m, beam 90 reads 4.0 m, beam 270 reads nothing.
2. **Encoding of "nothing".** REP 117: `+inf` (no return or beyond range) and `−inf` (closer than the minimum).
   The book's convention: `z_max`. Both encodings score the same under the beam model, which treats a
   non-finite reading as a max-range one (`docs/SOURCE_MAPPING.md`, deviation 5).
3. **Determinism.** One seeded generator per noise source (wheel slip, gyro white noise, gyro bias walk, lidar),
   derived from the master seed; every beam draws its uniform and its normal whether or not it uses them. So: the same
   seed replays every stream; removing the lidar changes none of the others; changing the dropout does not reshuffle
   the noise of the surviving beams. **Checked:** `ch03_sim_replay`, `checks/ch03.ts`, `lidar_does_not_perturb_the_other_noise_streams`.
   The encoder stream does not depend on the seed at all: the simulator integrates the commanded wheel angle, and slip and the
   radius error enter only the distance the robot travels. That is a fact the checks assert and Chapter 2's drift rests on.
4. **The beam model** (Thrun Table 6.1). `p(z | z*) = z_hit p_hit + z_short p_short + z_max p_max + z_rand p_rand`:
   `p_hit = N(z; z*, σ_hit²)/η` on `[0, z_max]`, `p_short = λ e^{−λz}/(1 − e^{−λz*})` on `[0, z*]`, `p_max` a point
   mass at `z_max`, `p_rand = 1/z_max`. Defaults `(0.74, 0.10, 0.06, 0.10)`, `σ_hit = 0.12`, `λ = 1`. It integrates to 1 (the
   test `beam_model_normalizes`; checked here for three `z*`). A scan's score is `Σ_k ln p(z_k | z*_k)`.
5. **Learning the parameters by EM** (Table 6.2). E-step: responsibilities `e_{i,c} = z_c p_c(z_i)/Σ`; M-step: `z_c = mean_i e_{i,c}`,
   `σ_hit² = Σ e_{i,hit}(z_i − z_i*)²/Σ e_{i,hit}`, `λ = Σ e_{i,short}/Σ e_{i,short} z_i`. **Worked example** (checked against
   `docs/SOURCE_MAPPING.md`): readings `{4.5, 5, 3, 7, 10}` at `z* = 5`, `z_max = 10`, uniform weights, `σ_hit = 1`, `λ = 1/2` give
   `(0.394282, 0.127210, 0.2, 0.278508)`, `σ_hit = 1.093905`, `λ = 0.283737` (0.2837390 in full precision), mean log-likelihood −2.275038.
   **Honest limit:** the M-step ignores that `p_hit` and `p_short` are truncated, so `λ` is biased and the trace can dip by about
   10⁻⁴ nats per beam (`docs/SOURCE_MAPPING.md`, deviation 8); the page shows both.
6. **The likelihood field** (Table 6.3). Precompute `d(p)`, the distance from a point to the nearest wall, on a grid with the exact
   Euclidean distance transform (Felzenszwalb and Huttenlocher, linear time). Score each non-max-range beam end point
   `q += ln(z_hit N(d; 0, σ_hit²) + z_rand/z_max)`. It is not a generative model of a scan; it is a smooth, cheap score.
   Defaults `(0.9, 0.1)`, `σ_hit = 0.2 m`.
7. **Tempering.** `p(z | x)^β`, `0 < β ≤ 1`: the same peak, a flatter surface. `Tempering::for_target_ess` picks the largest β that keeps the
   effective sample size above a target. **Measured** (checked on 64 scans, three base seeds): the field's rms error/σ is 1.4 at `β = 1`
   and under 1 by `β = 0.5`; the beam model's is under 1 already; the posterior mean's error does not move.
8. **The landmark model** (Table 6.4). `p(r, φ | x, m) = N(r − r̂; 0, σ_r²) N(wrap(φ − φ̂); 0, σ_φ²)`, with `r̂ = ‖p‖`, `φ̂ = atan2(p_y, p_x)`,
   `p` the landmark in the sensor frame. The bearing residual is wrapped to `(−π, π]`. The Jacobian for an EKF on SE(2) is derived in a
   `<Derivation>` and checked against a central difference. Known correspondence only.

## 4. Conceptual

* **w3.1, one sweep.** Drag the robot, slide its heading, the noise σ and the dropout. Kills "a beam that returns nothing means there is nothing out there".
* **w3.2, replay.** Four runs of the same drive: A, A again, A without the lidar, and the next seed, with the fingerprints of the four streams.
  Kills "adding a sensor to a simulation changes the noise of the sensors already there".
* **w3.3, one beam.** The four-part density with its point mass; z* and σ_hit. Kills "a range sensor reads the expected distance plus a little Gaussian noise, and nothing else".
* **w3.4, learning the intrinsics.** EM from a bad start on 20 000 simulated readings; play it, or scrub. Kills "a sensor model's parameters have to be tuned by hand against logged scans".
* **w3.5, scoring a pose.** One scan at the true pose scored at hypotheses along x, y or heading, for both models, with the distance map as a picture. Kills
  "the model that is the exact physics of the sensor is also the best score for finding the pose" (the beam model is sharper, the field is cheaper and smoother, and the picture shows why).
* **w3.6, tempering.** β, over 64 scans, both models. Kills "the sharper a posterior is, the better it is".
* **w3.7, the seam.** A landmark behind the robot, the reading dragged across ±π; the wrapped and unwrapped likelihoods. Kills "the difference between two angles is just their subtraction".

## 5. Practical

Excerpts printed (every non-blank line is checked to occur, in order, in the named file):

* `crates/robotics_sim/src/world.rs`: `World::raycast`.
* `crates/robotics_sim/src/lidar.rs`: `Lidar::scan` (the forward model, every beam drawing both numbers).
* `crates/robotics_models/src/sensor/beam.rs`: `BeamModel::components` (the four terms) and `likelihood`; the E and M steps of `em_step`.
* `crates/robotics_models/src/sensor/likelihood_field.rs`: `LikelihoodFieldModel::log_likelihood`.
* `crates/robotics_models/src/sensor/landmark.rs`: `residual`, `log_likelihood` and `jacobian`.
* `crates/robotics_models/src/sensor/tempering.rs`: `apply_log` and the heart of `for_target_ess`.
* A worked example with its expected output as a `text` block: the EM iteration, every number asserted in `checks/ch03.ts`.

## 6. Widget manifest

| Id | Component | Rust called (`robotics_wasm`) | Foregrounded parameter | Kills the misconception that |
|---|---|---|---|---|
| w3.1 | `ArenaScan` (`arena-scan.tsx`) | `ch03_lidar_scan` (`Lidar::scan`, `World::raycast`), `ch03_scan_points`, `ch03_sensor_pose`, `ch03_arena_segments` | the robot's position (drag) | a beam that returns nothing means there is nothing out there |
| w3.2 | `Replay` (`replay.tsx`) | `ch03_sim_replay` (`RustySim::step`) | the seed (re-roll) | adding a sensor to a simulation changes the noise of the sensors already there |
| w3.3 | `BeamDensity` (`beam-density.tsx`) | `ch03_beam_density` (`BeamModel::{components, likelihood}`) | the expected range z* | a range sensor reads the expected distance plus a little Gaussian noise, and nothing else |
| w3.4 | `EmFit` (`em-fit.tsx`) | `ch03_em_fit` (`BeamModel::em_step`; the data generator is marked as not crate code) | EM iterations | a sensor model's parameters have to be tuned by hand against logged scans |
| w3.5 | `FieldVsBeam` (`field-vs-beam.tsx`) | `ch03_slice_scores` (`LikelihoodFieldModel::log_likelihood`, `BeamModel::log_likelihood`), `ch03_likelihood_field` (`DistanceGrid`), `ch03_lidar_scan`, `ch03_scan_points` | the axis the hypothesis slides along | the model that is the exact physics of the sensor is also the best score for finding the pose |
| w3.6 | `Tempering` (`tempering.tsx`) | `ch03_study` (`Tempering`, `effective_sample_size`, both models) | β | the sharper a posterior is, the better it is |
| w3.7 | `Landmark` (`landmark.tsx`) | `ch03_landmark_loglik` (`LandmarkModel::log_likelihood`; the unwrapped strawman is marked), `ch03_landmark_residual` | the measured bearing | the difference between two angles is just their subtraction |

## 7. Exercises

1. **F, difficulty 2.** Show that `p_hit`, as written, integrates to 1 on `[0, z_max]`, and say what would be wrong with the
   mixture if the normalizer `η` were omitted. *Sketch:* `η = Φ((z_max − z*)/σ) − Φ(−z*/σ)`; without it the hit component loses
   the mass of the Gaussian's tails outside `[0, z_max]`, which matters near a wall.
2. **F, difficulty 2.** Derive the landmark Jacobian column for the heading, `∂ĥ/∂ω`, for a sensor at the base origin.
   *Sketch:* `p_b` rotates by `−ω`; `∂p/∂ω = (p_by, −p_bx)`; chain through `r = ‖p‖` and `φ = atan2`.
3. **C, difficulty 1.** Predict, then check in w3.1: set the dropout to 0 and the noise to 0 and move the robot to the middle of the arena; how many beams
   read `+inf`, and why is it not 0? *Sketch:* those whose ray leaves through no wall within 8 m.
4. **C, difficulty 2.** Predict, then check in w3.5: which model's peak is wider? *Sketch:* the field's, about twice; each option has a `because`.
5. **C, difficulty 2.** Predict, then check in w3.6: with the beam model, is tempering needed? *Sketch:* its ratio is already under 1 because its ray casts are exact.
6. **P, difficulty 2.** In `crates/robotics_models/src/sensor/likelihood_field.rs`, add a test that scores a scan taken inside a 1 m square room at the
   room's centre and at 10 cm off, with the exact distance field of the room's four walls, and checks that the centre scores higher.

## 8. Modernization notes

* **Finding (same as Chapter 2): a seed names a different noise stream on WebAssembly.** Every noisy quantity in this chapter is the page's own
  draw. Pinned both ways: `cargo test` pins the native fingerprint of one noisy scan (`8a526fd3ad0062b1`), `checks/ch03.ts` pins the
  WebAssembly one (`f7e036522696593c`). Quantities with no random stream (ray casts, the beam density, the EM worked example, the distance
  transform, the landmark model) are the same on both targets and are checked against closed forms and the repository's numbers. The EM
  data generator reproduces the crate's own test data only natively; on the page it draws a different data set from the same distribution,
  and the recovery is asserted over six seeds.
* **Three pieces of code in `robotics_wasm::ch03` are not crate code, and each says so where it lives**: `rasterize` (the repository has no
  rasterizer yet; `docs/SOURCE_MAPPING.md` row 10 assigns it to Phase 3), the EM data generator (the crate has the density and the fit, not a
  sampler), and the unwrapped landmark likelihood (a strawman the chapter measures the wrapped one against).
* Deviations in `docs/SOURCE_MAPPING.md` the chapter repeats: ray casting without a BVH (analytic over all 21 segments: 7 560 segment tests
  per scan); the REP 117 encoding of no-return; the EM implemented as in Table 6.2 including its untruncated M-steps; the landmark's signature
  is a discrete id.
* The likelihood field's score in w3.5 is a *score*, not a probability of anything; the chapter says so. Its map is a one-cell-thick outline of the
  walls on a 5 cm grid; that discretization is the map error behind the over-confidence of w3.6, and the chapter says it is this
  page's rasterizer, not the crate's.
* Known limitation, stated and not hidden: Rusty is not a collision model and its world is 21 static segments; a person walking through a scan (the
  usual reason beams are not independent) is not simulated, so the over-confidence the chapter measures comes from the map error alone, and the real
  cause the repository's docs name (correlated beams from unmodeled obstacles) is described, not measured.
