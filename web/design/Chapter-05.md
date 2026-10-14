# Chapter 5: EKF Localization: Tracking a Known Pose (design doc)

Implements the EKF half of Phase 3 of `docs/ROADMAP.md` (marked ✅, closed 2026-10-10). Written from
`docs/phase-3-localization/{problem,theory,implementation,benchmark}.md`, `docs/ROADMAP.md` (Phase 3 section), the
source of `robotics_localization::{map, pillars, ekf}` as it stands in the repository, and the verdict files
under `evidence/phase-3-correctness/`. It covers problem P1 (position tracking) and the *structural* half of P2b (why one Gaussian
cannot recover from a kidnapping). **Everything about particles, global localization from a uniform prior and the measured recovery
of the Monte Carlo filter belongs to Chapter 5 and is not written here.**

## 1. Purpose and story arc

**The problem.** Chapters 1 and 2 ended with a pose that drifts: dead reckoning has no second opinion, so its error grows with the
distance driven, forever. Phase 3 gives the TurtleBot3 a second opinion, the lidar against a map of four pillars, and a question that
has to be answered with numbers: *does the error stay bounded while the robot drives, instead of growing with distance, and does the
filter know how wrong it is?* (P1 of `problem.md` §3: "bounded over distance, and strictly better than the drift of `/odom` on the same
run", plus NEES inside its 95 % band.) The measured answer in Webots, on the 51.540 m route of `route_long.yaml`
(`2026-10-10T144153-final-global-reset/normal-verdict.yaml`): the EKF's position RMSE is 0.008945 m against the odometry's 0.015186 m (ratio 0.589),
its error grows by a factor of 1.167 from the first quarter of the run to the last where the odometry's grows by 3.130, and the verdict is PASS. The NEES half of P1 is
**not evaluated** in Webots, because the evaluator's CSV carries only `sigma_xy`; the chapter says so.

**Arc.** *Problem → what we try first and why it fails → the idea → the Rust → the number again.* The problem is drift (w5.1), and
w5.1 is also where the first honest surprise lives: the Webots odometry drifts by *centimetres* because upstream calibrated it against this
world's slip (`theory.md` §6), while Rusty's is uncalibrated and drifts by tenths of a metre; and when the wheels are made exact the filter is the
one that is off, because a correction is not free. The measurement is next: a lidar gives 360 distances and no labels, so a *detector* turns clusters
of beams into range and bearing to a pillar's centre (w5.2), and the detector is wrong in two ways the chapter measures (it overshoots the centre by about
3 cm, and it reports clusters on walls that are not pillars). A detection has no name, so the filter must decide which pillar it is: *maximum-likelihood
association*, with a χ² gate that rejects what no pillar explains (w5.3), and the gate's reach is set by the belief's own uncertainty, not by the room. The
update is then the Kalman correction on SE(2) with the landmark Jacobian of Chapter 3. Whether the filter is *honest* about its error is measured with NEES and
NIS (w5.4), and the surprise is that overconfidence is not only a reporting error: it shrinks the gate, which throws away the observations that would have corrected it. The chapter
ends where Chapter 5 begins (w5.5): carry the robot away and the single Gaussian cannot represent "I am somewhere else"; the filter does notice (it rejects most of what it sees) but nothing in
its update can *use* a rejected observation, so it stays wrong. In Webots the EKF recovered 0 of 20 kidnaps (Wilson 95 % interval 0.0000 to 0.1611, one fixed geometry).

**Epigraph** (verified verbatim, read in the abstract on arXiv, `https://arxiv.org/abs/1304.3111`): *"The estimates are probabilistic in nature, an advance over the previous, very conservative, worst-case approaches to the problem."* — R. Smith, M. Self and P. Cheeseman,
*Estimating Uncertain Spatial Relationships in Robotics* (abstract of the paper reprinted in *Autonomous Robot Vehicles*, 1990).

## 2. Prerequisites and position

Chapter 1 (SE(2), `⊞`/`⊟`, the Kalman filter on a hallway), Chapter 2 (the error-state idea, NEES/NIS and their χ² bands, drift; this chapter reuses
`ch02_ellipse` and `ch02_chi2_mean_band` of its module) and Chapter 3 (the range–bearing landmark model, its Jacobian and the bearing seam, the lidar simulator).
No ROS.

**Introduces:** `robotics_localization::{MapFile, PillarDetector, detect, EkfLocalizer, OdometryMotion, LocalizerConfig, UpdateReport}`.
**Reuses:** `Se2`, `Manifold`, `Ekf` (the generic filter, Joseph form), `LandmarkModel`, `consistency::nees`, `robotics_sim::{World, Lidar, RustySim}`,
`robotics_models::WheelOdometry`.

**Derived from:** `docs/phase-3-localization/problem.md` §§1–4, `theory.md` §§1–2 and 6, `implementation.md` §§1–5, `benchmark.md` §6 and the
EKF files of `evidence/phase-3-correctness/`. `docs/SOURCE_MAPPING.md` row 11 (EKF localization, ML association, χ² gates) still says *planned*; it is a
repository document outside `web/` and this chapter does not change it.

**Does not cover (and says so):** particle filters, global localization, the kidnap recovery that works, and every number about them (Chapter 5);
multi-hypothesis tracking (named, not built: `SOURCE_MAPPING.md` row 11 lists it); the ROS node's plumbing and TF (`ros-graph.md`: the node owns `map → odom`,
and that is mentioned, not taught); the benchmark against `nav2_amcl` (`benchmarks/phase-3-localization.csv` is header-only).

## 3. Foundation

1. **The problem, as a belief.** `bel(x_t) = p(x_t | z_{1:t}, u_{1:t}, m)` with the map `m` of four pillar centres (`landmarks.yaml`). P1: `bel(x_0) = N(x_0, Σ_0)` with small `Σ_0`
   (σ = 0.05 m and 0.05 rad in the node) and the question is whether the error stays bounded.
2. **The filter's state and its prediction.** The state is a pose on SE(2); the control is the *relative* motion of the odometry between two samples, `u = x̂_{k−1}⁻¹ x̂_k` (`EkfLocalizer::relative_motion`),
   composed on the right: `x_k = x_{k−1} ∘ u`. With a right (body-frame) perturbation the error-state Jacobian of that composition is `Ad(u⁻¹)` (`OdometryMotion::jacobian`:
   `u.inverse().adjoint()`), so `P_k = Ad(u⁻¹) P_{k−1} Ad(u⁻¹)ᵀ + R(u)`. The noise `R` is the odometry motion model of Thrun's chapter 5 in the body frame: along-track variance
   `α₃ d² + α₄ θ² + σ²_min`, lateral variance a fraction (0.1) of it plus the floor, heading variance `α₁ θ² + α₂ d² + σ²_min`, with `α = (0.05, 0.02, 0.05, 0.02)`, `σ_min = 10⁻³`. **Worked number** (the crate's own test, `prediction_alone_keeps_the_mean_and_grows_the_covariance`: a 0.5 m step moves the mean to `x = 0.5` exactly and increases `P_xx`).
   **A derivation** (`<Derivation>`): why the adjoint appears, in four steps.
3. **The measurement.** For the laser mounted at `T_base_laser = (−0.03, 0, 0)` and a pillar centre `p_j`, `h(x, p_j) = (r, φ)`, `r = ‖p_j^{sensor}‖`, `φ = atan2(p_{j,y}^{sensor}, p_{j,x}^{sensor})` with
   `p^{sensor} = (x ∘ T_base_laser)⁻¹ p_j`: Chapter 3's landmark model, reused (`LandmarkModel`, its Jacobian `H` for a right perturbation, the wrapped residual). Noise `Q = diag(σ_r², σ_φ²)`,
   node default `σ_r = σ_φ = 0.05`.
4. **The correction.** `S = H P Hᵀ + Q`, `K = P Hᵀ S⁻¹`, `x ← x ⊞ K ν`, `P ← (I − K H) P (I − K H)ᵀ + K Q Kᵀ` (Joseph form: `Ekf::correct`), with `ν = z ⊟ h` and the bearing wrapped.
5. **Data association by maximum likelihood, with a gate.** The detector's clusters have no names. For each observation `z` the filter evaluates, for every landmark `j`, `ν_j`, `S_j` and
   `NIS_j = ν_jᵀ S_j⁻¹ ν_j`, keeps the smallest, and accepts the observation only if `NIS ≤ γ`, where `γ = χ²₂(0.99) = −2 ln 0.01 = 9.2103` (closed form for two degrees of freedom; checked). An
   observation that no pillar explains inside the gate is dropped and counted in `rejected` (Thrun, table 7.2 with unknown correspondences). The gate depends on `S`, which depends on `P`: a belief that claims
   to be certain has a small gate. That is the mechanism of w5.3 and of w5.4's surprise.
6. **What the detector gives the filter.** A scan is split into clusters of consecutive beams with no range jump above 0.12 m (the scan is circular); clusters under 2 beams or with a chord over 0.6 m (walls) are
   dropped; each remaining cluster is reported as the centroid of its points pushed out along its bearing by the pillar radius, 0.15 m. **Measured, not stated in the repository** (`ch05.rs`, `native_table…` and
   `the_detector_on_a_grid_of_poses_without_noise`; `checks/ch05.ts`): on a grid of 1176 scans without noise the detector reports 2823 pillar centres and 225 clusters that are not pillars; the reported
   *range* is too long every time, by 2.5–3.0 cm in the median and 5.3 cm at most, because the centroid of the visible arc lies behind the near surface so adding the full radius overshoots the centre. With the lidar's
   1.5 cm of noise the clusters that are not pillars rise to 1087 on this module's draw (short runs of a wall seen at a grazing angle). Neither is a defect of the filter: the 5 cm assumed σ covers the first, and
   the gate is the answer to the second.
7. **Consistency.** `NEES = eᵀ P⁻¹ e` with `e = x_true ⊟ x̂` (three degrees of freedom) and `NIS` (two); their means should be 3 and 2 and the running mean has a two-sided χ² band that shrinks with the number of samples
   (`ch02_chi2_mean_band`; successive filter outputs are correlated, so the band is indicative, as `ConsistencyStats::report` says).
8. **The limit.** The belief is one Gaussian. After a carry, `P` is small, every `NIS_j` exceeds `γ`, every observation is dropped, the mean stays where odometry takes it, and `P` grows slowly and never admits the truth.
   That is a statement about the representation (`ekf.rs` says so at its head and tests it: `a_teleported_robot_is_not_recovered_by_a_single_gaussian`), not a tuning failure.

**Worked examples and the numbers each chapter claim rests on** (every one is asserted; the table is the index into `checks/ch05.ts` and the native tests in `crates/robotics_wasm/src/ch05.rs`):

| Claim | Value | Asserted by |
|---|---|---|
| the map and the route are the repository's files | 4 pillars, 21 waypoints, spawn pose `(0, 0, 0)`, room ±3.0 m | `checks/ch05.ts` against `landmarks.yaml` and `route_long.yaml` |
| the gate | `γ = −2 ln 0.01 = 9.21034` | `ch05_filter_defaults`, closed form |
| the detector on 1176 noise-free scans | 2823 pillars, 225 other clusters; range bias 2.5–3.0 cm (median), ≤ 5.3 cm; centre error ≤ 5.1 cm | native test and `checks/ch05.ts` (no random stream: same on both targets) |
| Rusty, eight seeds (module draw) | odometry RMSE 0.3762–0.4895 m, filter RMSE 0.0076–0.0080 m, filter max ≤ 0.0167 m, mean NEES 0.5–0.65, truth inside the 3σ ellipse at every scan | `checks/ch05.ts` (the native draw differs, pinned in `cargo test`) |
| exact wheels | odometry RMSE < 1 mm, filter > 5 mm and > 5× the odometry's | both |
| a tenth of the assumed σ | NEES > 20, truth inside the ellipse < 5 % of scans, RMSE > 2× the honest filter's | both |
| the carry | error < 5 cm before, > 3 m in every scan after, rejects > 5× what it accepts | both |
| Webots | the values of `normal-verdict.yaml`, `kidnap-verdict-ekf.yaml`, `kidnap-trials-summary.yaml` quoted in §1 | `checks/ch05.ts` reads the files under `evidence/` |

## 4. Conceptual

* **w5.1, drift against correction.** The 51 m route in Rusty with the wheel odometry (orange), the filter (purple) and the truth (dashed gray); error against distance on a log axis; sliders for the wheel radius error and the
  slip; a switch for the correction. Kills "a filter that adds a second sensor to the wheels cannot do worse than the wheels alone" (with exact wheels it does).
* **w5.2, what the detector reports.** One scan from a pose you drag, the beams' end points, the detections as rings with a dotted line to the true centre, the clusters that are not pillars as crosses; noise and heading. Kills "the lidar measures where
  a pillar's centre is, and every cluster it finds is a pillar".
* **w5.3, which pillar is which.** The robot is at the origin; the belief is wrong by an amount that sweeps 0 → 1.5 m → 0 until you take a slider; each detection is drawn where the filter, believing its pose, would put it, with the line to the pillar the gate
  allows, or a cross if it throws it away; the belief's own σ is a slider. Kills "the filter knows which pillar each detection is, and keeps correcting itself back onto the map however far off it is".
* **w5.4, is the filter honest?** A zoomed window of the filter's 3σ ellipse and the truth, the running mean NEES against its band on a log axis, and a slider for the σ the filter assumes. Kills "a filter that reports a small covariance is a precise one".
* **w5.5, carried away.** The Webots kidnap (−1.5, −1.5, 2.0) at a time you choose, or never; error against time; accepted and rejected observations since the carry. Kills "a filter that notices its observations stop fitting will sort itself out".

## 5. Practical

Excerpts printed (every non-blank line is checked to occur, in order, in the named file; paths as written):

* `crates/robotics_localization/src/ekf.rs`: `OdometryMotion::{propagate, process_noise}` and the process Jacobian (`u.inverse().adjoint()`); `EkfLocalizer::observe` (association, gate, correction).
* `crates/robotics_localization/src/pillars.rs`: the last half of `detect` (the chord rule and the centroid with the radius pushed out).
* `crates/robotics_localization/src/map.rs`: `MapFile::landmark_map` (the file becomes the model's map).
* `crates/robotics_wasm/src/ch05.rs`: the *pipeline* of `Ch05Session::physics_step` and `scan_update`, to show that the page runs the node's calls in the node's order (marked as the harness, not as crate code).
* A worked example with expected output as a `text` block: the association table of w5.3's default scene, every number asserted in `checks/ch05.ts`.

## 6. Widget manifest

| Id | Component | Rust called (`robotics_wasm`) | Foregrounded parameter | Kills the misconception that |
|---|---|---|---|---|
| w5.1 | `DriftVsCorrection` (`drift-vs-correction.tsx`) | `Ch05Session` (`RustySim`, `WheelOdometry`, `Lidar`, `pillars::detect`, `EkfLocalizer::{predict, observe}`), `ch02_ellipse` | right wheel radius error | a filter that adds a second sensor to the wheels cannot do worse than the wheels alone |
| w5.2 | `Detector` (`detector.tsx`) | `ch05_scan` (`Lidar::scan`, `pillars::detect`), `ch05_detector_defaults`, `ch05_landmarks`, `ch05_room_half` | the robot's position (drag) | the lidar measures where a pillar's centre is, and every cluster it finds is a pillar |
| w5.3 | `Association` (`association.tsx`) | `ch05_associate` (`EkfLocalizer::observe`, `chi2_quantile`), `ch05_filter_defaults` | how far the belief is from the truth | the filter knows which pillar each detection is, and keeps correcting itself back onto the map however far off it is |
| w5.4 | `HonestFilter` (`honest-filter.tsx`) | `Ch05Session`, `ch02_ellipse`, `ch02_chi2_mean_band` | the σ the filter assumes for a pillar | a filter that reports a small covariance is a precise one |
| w5.5 | `Kidnap` (`kidnap.tsx`) | `Ch05Session` (with the carry) | when the robot is carried | a filter that notices its observations stop fitting will sort itself out |

## 7. Exercises

1. **F, difficulty 2.** Show that composing on the right, `x_k = x_{k−1} ∘ u`, gives the error-state Jacobian `Ad(u⁻¹)` with respect to the previous right perturbation and the identity with respect to the noise in the *new* body frame.
   *Sketch:* `x_{k−1} ⊞ δ ∘ u = x_{k−1} ∘ u ∘ (u⁻¹ exp(δ) u)` and `u⁻¹ exp(δ) u = exp(Ad(u⁻¹) δ)`.
2. **F, difficulty 2.** If the belief were perfectly certain, `S = Q = diag(0.05², 0.05²)`. How far in range can an observation differ from the prediction before the 99 % gate rejects it, with a zero bearing residual? *Sketch:*
   `ν_r²/0.0025 ≤ 9.21034`, so `|ν_r| ≤ √(9.21034 × 0.0025) = 0.1517 m` (the check reproduces the hand value).
3. **C, difficulty 1.** Predict, then check in w5.3: with σ = 0.05 m, about how far off can the belief be before the gate rejects every detection? *Sketch:* a few tens of centimetres: at the widget's default scene and seed the number accepted is 4, 2, 0 as the error grows from 0.2 to 0.3 to 0.4 m (asserted); each option has a `because`.
4. **C, difficulty 2.** Predict, then check in w5.4: if the filter assumes a tenth of the real noise, is the error larger or smaller than with the honest setting? *Sketch:* larger by a factor above two on every seed tried, because the gate shrinks with `S` and rejects the observations that would have corrected it.
5. **C, difficulty 2.** Predict, then check in w5.1: with exact wheels, which is closer to the truth, the odometry or the filter? *Sketch:* the odometry; the filter's error is its own detector noise and bias.
6. **P, difficulty 2.** In `crates/robotics_localization/src/pillars.rs`, add a test that places a pillar at 1, 2 and 3 m from a noise-free sensor and asserts that the reported range is longer than the true range by between 2 and 4 cm at each. Does the bias depend on the distance?
   *Sketch:* it hardly does (medians of 2.3 to 3.1 cm in each metre of range, asserted); the cause is the centroid of the visible arc, not the distance.

## 8. Modernization notes

* **Finding (same as Chapters 2 and 3): a seed names a different noise stream on WebAssembly.** Every noisy quantity on the page is this module's draw. `cargo test` in `crates/robotics_wasm` pins the native realization of
  eight seeds (odometry RMSE 0.3884–0.5068 m, filter RMSE 0.0076–0.0079 m); `checks/ch05.ts` pins this module's (0.3762–0.4895 m, 0.0076–0.0080 m) and asserts that seed 1 differs (0.4383 here, 0.4179 natively).
  Quantities with no random stream (the files, the defaults, the noise-free detector grid, the gate, the association properties that do not depend on the noise draw) are the same on both and are asserted on both.
* **The page runs Rusty; Webots is not in the browser.** Rusty's odometry is deliberately uncalibrated (2 % slip per step, 0.2 % right-wheel radius error: Chapter 2's values) so that its drift is visible, and its detector sees polygon pillars through
  a ray caster. The Webots numbers are recorded evidence, read by `checks/ch05.ts` from the verdict files, and the chapter always says which one a number is. The two odometries differ in kind: the Webots one
  was calibrated upstream against this world's slip (`theory.md` §6), so its drift is centimetres, and the repository's own record is that a short route cannot compare a filter with it (`theory.md` §6: a 0.9 m square made the filter look five times worse).
* **Not crate code, each marked where it lives in `ch05.rs`:** the route follower (a restatement of `route_driver.rs`'s control law, because that node is an rclrs process); the 24-gon that stands in for each round pillar (Rusty's world holds segments); the carry
  (a rigid transform on the truth, as the Webots supervisor's teleport, with the simulator and odometry unaware).
* **Findings the chapter reports that the repository's docs do not:** the detector's range bias (2.5–3.0 cm median), the count of clusters that are not pillars and how noise multiplies it, the gate's reach as a function of the belief's σ, and that an assumed σ of a tenth makes the
  filter less accurate and not only mis-calibrated. Each is asserted by `checks/ch05.ts` and by a native test.
* **Known limitations, stated and not hidden:** the NEES half of P1 is not evaluated in Webots (`nees_in_95_band: not_evaluated`); the EKF kidnap result is one fixed geometry (seed-independent by construction, since the EKF has no random stream);
  the benchmark against `nav2_amcl` does not exist; the detector assumes free-standing round pillars and straight walls, which is this world's whole map.
* **Cross-chapter dependency:** the widgets call `ch02_ellipse` and `ch02_chi2_mean_band` from Chapter 2's module rather than duplicate them. `web/lib/use-simulation.ts` gained a `play()` so a widget can restart itself when a parameter changes; nothing else shared was changed.
