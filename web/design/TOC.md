# TOC: the contract of the ROS via Rust webbook

This file is the single source of truth for what the book is allowed to show. Nothing in
`web/` may contradict it; to change the book's structure, change this file **first**, then
`lib/book-structure.ts`, `content/chapters/meta.json` and the chapter files.
`npm run check:book` fails the build when they disagree.

## 1. What the book is

*ROS via Rust* is the problem-based companion of the `ros2-rust-robotics` repository. Every
chapter opens with a concrete problem a TurtleBot3 has, states it precisely enough to be
measured, derives only as much theory as the problem needs, and then shows the Rust that
solves it. The structure of each chapter is the **FCP** rhythm: **F**oundation (the
mathematics), **C**onceptual (interactive widgets), **P**ractical (the real crate source).

**The core promise: the widgets run the repository's Rust.** `crates/robotics_wasm` compiles
`robotics_core`, `robotics_models`, `robotics_filters`, `robotics_sim`, `robotics_mapping`, `robotics_localization` and `robotics_slam` to WebAssembly
(`npm run wasm`); every widget calls that module. There is no TypeScript port of any
algorithm. The listings printed in a chapter are excerpts of the crate source, with its path
in the code-fence title, and every number a chapter quotes is asserted: by `npm run check`
against the compiled module, or, for the repository's recorded *native* runs of a seeded
simulation, by `cargo test` in `crates/robotics_wasm` (§5, "A seed names a different noise stream
on WebAssembly").

**The cadence is incremental.** A phase gets chapters only after it is closed in
`docs/ROADMAP.md` (marked ✅), verified, documented and committed. Phases that are still
running experiments are listed below as *planned* and have no chapter, no stub and no
sidebar entry. Each installment ends with something viewable.

Running characters: the robot is the **TurtleBot3 Burger** in Webots for the phase problems;
the in-repo simulator **Rusty** (`robotics_sim`, differential drive, 2-D lidar) is the
seeded, headless world the widgets run in. Use those names and no others.

## 2. Notation, color code, crates

### Notation

The table is typeset on `/notation` from `lib/notation.ts`; the symbols are:

| Symbol | Meaning | Rust |
|---|---|---|
| `x_t = (x, y, θ)` | robot pose, an element of SE(2) | `robotics_core::Se2` |
| `τ = (v_x, v_y, ω)` | tangent vector, translation first | `Tangent2` |
| `x ⊞ δ = x·exp(δ)` | right (body-frame) retraction | `Manifold::boxplus` |
| `y ⊟ x = log(x⁻¹ y)` | local difference | `Manifold::boxminus` |
| `u_t` | control over (t−1, t] | `VelocityCmd`, `OdomDelta` |
| `z_t` | measurement | `LaserScan`, `ImuMeasurement` |
| `bel(x_t)` | posterior belief | `BayesFilter::correct` |
| `bel̄(x_t)` | predicted belief | `BayesFilter::predict` |
| `R_t`, `Q_t` | motion noise, measurement noise (Thrun naming) | `KalmanFilter` |
| `μ`, `Σ` | Gaussian mean and covariance | `Gaussian` |
| `ε = eᵀP⁻¹e` | NEES | `consistency::nees` |
| `ν = rᵀS⁻¹r` | NIS | `consistency::nis` |
| `z*` | expected range from ray casting | `World::raycast` |

Conventions that never change silently: tangent order translation-first; perturbations are
right/body-frame; angles in (−π, π]; a pose covariance lives in the tangent space at the
pose in (x, y, θ) order; `R` is motion noise and `Q` is measurement noise. A chapter may
*add* symbols in its own notation table; it may not redefine one of these. The KaTeX macros
that spell them (`\bel`, `\belbar`, `\SEtwo`, `\bplus`, `\bminus`, `\Ad`, …) live in
`lib/katex-macros.ts`, the only macro table.

### The five-role color code

| Role | Meaning | CSS custom property | Light | Dark | Equation class |
|---|---|---|---|---|---|
| prior | what we believed before this step | `--pr-prior` | `#3b82f6` | `#60a5fa` | `term-prior` |
| prediction | after the motion model, before the sensor | `--pr-prediction` | `#ea580c` | `#fb923c` | `term-prediction` |
| measurement | what the sensor said | `--pr-measurement` | `#16a34a` | `#4ade80` | `term-measurement` |
| posterior | after the update | `--pr-posterior` | `#9333ea` | `#c084fc` | `term-posterior` |
| truth | ground truth, always dashed | `--pr-truth` | `#6b7280` | `#94a3b8` | `term-truth` |

The `--pr-*` properties are the names and values the widgets, the charts, the KaTeX term classes and `readPalette()` actually use. They are defined once in
`app/global.css` (`:root`) and redefined under `.dark`, and `app/global.css` is the source of the values: if this table and that file ever disagree, the file is right and this table is stale. (An earlier version of this table
named the properties `--belief-*` and gave other hex values; it was aligned to the file on 2026-10-11.)
Two other spellings exist in that file and neither is the primary: `--belief-*` are *aliases* of the `--pr-*` properties (`--belief-prior: var(--pr-prior)`), kept for the old API, and no component reads them; and the Tailwind `@theme` block declares
`--color-prior` and its four siblings with slightly different values (for example `#f97316` for prediction), which no component uses. Canvas code reads the palette through `readPalette()`; DOM and SVG use `var(--pr-…)`. A hex literal for a data
color anywhere else is a bug (`check:book` greps for it). The chrome accent is teal, deliberately outside the data hues, and neutrals (`--ink`, `--ink-muted`, `--grid`) carry everything that is not one of the five roles.

### Verified crate and tool table

Verified means: `npm run check:book` compares every row with the manifest named in the last
column and fails on a mismatch.

| Crate / tool | Version | Used for | Verified against |
|---|---|---|---|
| nalgebra | 0.35 | fixed-size states, covariances, Jacobians | `Cargo.toml` workspace |
| rand | 0.10 | seeded `SmallRng` | `Cargo.toml` workspace |
| rand_distr | 0.6 | normal sampling | `Cargo.toml` workspace |
| thiserror | 2 | typed errors | `Cargo.toml` workspace |
| wasm-bindgen | 0.2.129 | Rust ↔ JavaScript glue (CLI pinned to the same version) | `crates/robotics_wasm/Cargo.lock` |
| getrandom | 0.4 (`wasm_js`) | the one build blocker, see §5 | `crates/robotics_wasm/Cargo.toml` |
| next | 16.3.0 | static export | `web/package.json` |
| fumadocs-core / -ui | 16.14.3 | chapter chrome, sidebar, search | `web/package.json` |
| fumadocs-mdx | 15.2.3 | MDX pipeline | `web/package.json` |
| katex | 0.18.4 | math (pinned, with an `overrides` entry) | `web/package.json` |
| tailwindcss | 4.3.3 | styling | `web/package.json` |
| @nivo/line | 0.99.0 | dashboards | `web/package.json` |

## 3. Phases and chapters

Only `published` rows have chapters. A phase becomes published when `docs/ROADMAP.md` marks
it ✅ *and* its chapters are written; the checker reads the roadmap.

<!-- phase-table:start -->
| Phase | Title | Status | Chapters | Source chapters |
|---|---|---|---|---|
| 1 | Foundation | published | 1 | Ch. 1–3, 5, 9; Ch. 4 and 6 in part |
| 2 | State estimation, sensor models, simulation | published | 2, 3 | Ch. 4, 6–8, 10 |
| 3 | Localization | published | 5 | Ch. 11–12 |
| 4 | Mapping | published | 4 | Ch. 13, 19 |
| 5 | SLAM | published | 6 | Ch. 14–17 |
| 6 | Planning | planned | – | Ch. 20–22 |
| 7 | Control | planned | – | Ch. 23 |
| 8 | Exploration | planned | – | Ch. 24 |
| 9 | Complete autonomy stack | planned | – | Ch. 26 |
<!-- phase-table:end -->

(*Source chapters* are chapters of *Probabilistic Robotics via Rust*, mapped to crates in
`docs/SOURCE_MAPPING.md`.)

<!-- chapter-table:start -->
| Chapter | Slug | Phase | Title |
|---|---|---|---|
| 1 | ch01-foundation | 1 | Foundation: Poses, Odometry and the First Filters |
| 2 | ch02-state-estimation | 2 | State Estimation: Fusing Wheels and a Gyro |
| 3 | ch03-sensor-models | 2 | Sensor Models and the Simulator |
| 5 | ch05-ekf-localization | 3 | EKF Localization: Tracking a Known Pose |
| 4 | ch04-mapping | 4 | Mapping: Occupancy Grids from Known Poses |
| 6 | ch06-slam | 5 | SLAM: Scan Matching, Pose Graphs and Loops You Can Trust |
<!-- chapter-table:end -->

**Phase 2 is closed in the repository and is written here.** It was published chapter by chapter: as soon as Chapter 2
passed every gate the phase became `published` with that one chapter listed, and Chapter 3 (*Sensor Models and the Simulator*) joined the
same row when it passed in turn. Nothing links to a chapter that is not written. (An earlier version of this file kept the phase `planned`
until both chapters existed; that made the first of them impossible to build and look at, because the consistency check refuses a chapter
file in a planned phase.)

**Why Phase 2 is two chapters.** Phase 2 delivers two things that answer different
questions and share almost no code: (a) *how do we fuse wheels and a gyro into one pose, and
how do we know the answer deserves trust?* (the error-state filter and its NEES/NIS tests,
`robotics_filters`), and (b) *what does a sensor reading mean, and what world produced it?*
(beam, likelihood-field and landmark models, the ray-cast lidar and the deterministic
simulator, `robotics_models::sensor` and `robotics_sim`). One chapter would hold roughly
nine widgets and two unrelated problems; Phase 1, in contrast, is a single connected story
(a pose, how a step moves it, how that step is noisy, how a filter tracks the noise) and is
one chapter.

**Phase 3 is closed in the repository (✅ in `docs/ROADMAP.md`, 2026-10-10) and is `published` here with one chapter, Chapter 5, *EKF Localization: Tracking a Known Pose*** (problem P1, position tracking, and the structural reason one Gaussian cannot be kidnapped). It was first written as Chapter 4 and gated. On 2026-10-11 the maintainer decided that **Mapping takes the number 4** ("overwrite Chapter 4"), and the EKF chapter was archived to `../archive-ch04-ekf-localization-2026-10-11/` (outside the repository). The same day the maintainer asked for Part 3 to be filled from that material; the chapter was restored under the next free number, **5**, with its widgets renamed w5.x, its checks in `checks/ch05.ts` and its bindings in `ch05.rs`. The Monte Carlo chapter (problems P2a and P2b: global localization and kidnap recovery) is **not written**; the text of Chapter 5 says so where it points to it. The roadmap's limitations on the Phase 3 results travel with the chapter (its design doc, §8).
Chapters 2 and 3 say "Part 3" when they mean localization, and Chapter 3 calls Part 3 "particle-filter localization"; that was written in the future tense before Part 3 existed and was left as it is. Chapter numbers are *not* contiguous with part numbers (Part 3 holds Chapter 5, Part 4 holds Chapter 4, Parts 1 and 2 hold Chapters 1 to 3). The sidebar shows Parts 1–4 in order, so the chapter links run Chapter 1, 2, 3, 5, 4.

**Phase 4 is closed in the repository (✅ in `docs/ROADMAP.md`, 2026-10-11, with carried-over items: M5, which waits for a decision on the localizer's route to `map → odom`; the Rust-versus-rclpy and Nav2 comparison, which is not built; a second, more complex world) and is written here as one chapter,** Chapter 4, *Mapping: Occupancy Grids from Known Poses*. One chapter because the part is one connected story: a log-odds grid, the rays that fill it, how it is judged against the true room, and what the first runs
got wrong. The roadmap's limitations travel with it into the text (design doc §8), and the benchmark section is published **incomplete on purpose**: the numbers it has are the repository's, the cross-language comparison is stated as pending.

**Phase 5 is closed in the repository (✅ in `docs/ROADMAP.md`, 2026-10-15, with carried-over items and an S5 caveat written into the heading: S5 met on measured-instant labels, not on the recorded ones) and is written here as one chapter,** Chapter 6, *SLAM: Scan Matching, Pose Graphs and Loops You Can Trust*. One chapter because the part is one connected story (a chain of scan matches drifts, a graph repairs it, a wrong loop closure poisons it, so loops are verified, and what "wrong" means depends on the ground truth they are graded against), told with three widgets. It takes the number 6, the next free one: Chapters 1 to 5 are untouched, and the sidebar order is Part 5 after Part 4. The part closed on the maintainer's delegation of the decision after the gate of its closing prompt was not met on the recorded labels; the roadmap's caveat travels with the chapter (design doc §8). Source chapters 14 (EKF-SLAM) and 17 (grid RBPF) are covered in one short section without a widget; place recognition, landmark FastSLAM and the 2-D TSDF are not built and the chapter says so.

## 4. Publishing architecture

* `web/`: Next.js 16 App Router, `output: 'export'`, Fumadocs for the chapter chrome,
  `remark-math` + `rehype-katex`, Nivo for dashboards, Tailwind v4.
* `crates/robotics_wasm`: a standalone Cargo workspace (own `Cargo.lock`) so the ROS 2
  workspace and its lock file are untouched. `npm run wasm` builds it for
  `wasm32-unknown-unknown` and runs `wasm-bindgen --target web`.
* `npm run check` runs `checks/run.ts` under Node against the same `.wasm` the browser
  fetches: closed-form results, worked examples pinned by the repository's Rust tests, and
  the numbers the chapters quote.
* `npm run check:book` is the consistency gate: published-chapter count vs this file,
  chapter files vs `meta.json` vs `lib/book-structure.ts`, widget ids vs the design docs'
  manifests, cross-links and citations, no `Math.random`, no raw hex data colors, no raw
  `<a href="/…">` (a GitHub Pages sub-path breaks them).
* `npm run smoke` opens every page in headless Chromium (light and dark, desktop and phone)
  and fails on console errors, failed requests (the `.wasm` included), KaTeX error boxes,
  widgets stuck loading and sideways scroll.
* `.github/workflows/deploy.yml`: WASM build → `check` → `check:book` → `next build` →
  smoke → deploy to Pages on push to `main`; every gate but the deploy on pull requests.

## 5. Findings from compiling the algorithm crates to WebAssembly

| Finding | Detail | Resolution |
|---|---|---|
| `getrandom` 0.4 refuses `wasm32-unknown-unknown` | `rand` 0.10's default features pull `getrandom`, whose build script stops with "enable the `wasm_js` feature" | `crates/robotics_wasm` enables `getrandom/wasm_js` for the wasm target only. No pure crate's manifest changed. The backend is linked but unreachable: every generator in the pure crates is `seeded_rng`, and `check:book` greps the crates for OS-entropy entry points. |
| wasm-pack | archived upstream | the build uses `wasm-bindgen-cli` directly, pinned to the crate's `wasm-bindgen` version |
| `robotics_sim` is outside the requested list | it is pure and compiles unchanged, and Phase 2's "simulation" scope cannot be taught without it | included in the module |
| `robotics_mapping` | compiled into the module since Phase 4 closed (2026-10-11) for Chapter 4 | compiles to wasm32 **with no source change**, to it or to any crate it depends on. It depends on `robotics_core`, which brings in `rand`, so a standalone `cargo check --target wasm32-unknown-unknown -p robotics_mapping` stops at the same `getrandom` message as every other crate; the backend `robotics_wasm` selects resolves it. No generator in `robotics_mapping` or in its page bindings is seeded or random at all, so the native-versus-wasm32 stream divergence below does not touch Chapter 4: the module's replay of a recorded run writes the same grid, byte for byte, as the robot's node (`checks/ch04.ts`, and `cargo test` natively). The recording the widget replays is not compiled in (1.5 MB): `npm run wasm` copies the evidence file to `public/data/` and a check asserts the copy. Nothing was hand-ported |
| `robotics_localization` | compiled into the module for Chapter 5 (EKF localization), restored 2026-10-11 beside `robotics_mapping` | compiles to wasm32 with no source change. `MapFile::load` reads through `std::fs` and cannot work in a browser, so the module calls `MapFile::parse` on `include_str!` copies of `sim/webots/config/landmarks.yaml` and `route_long.yaml`, which keeps the page on the repository's own files. Its generators are `seeded_rng`, so the native-versus-wasm32 stream divergence below applies to it as to every other crate |
| A seed names a different noise stream on WebAssembly | `robotics_core::seeded_rng` returns `rand`'s `SmallRng`, which is Xoshiro256++ where `usize` is 64 bits and Xoshiro128++ where it is 32 bits (`rand` 0.10, `rngs/small.rs`). `seeded_rng(1)` yields `14971601782005023387` natively and `11103346041267129828` in the module. Everything seeded (Rusty's slip, gyro and bias streams, the velocity-model sampler, the hallway sensor) therefore draws a *different realization* in the browser than on the robot, for the same seed | Not changed: replacing the generator in `robotics_core` touches a crate the running Phase 3 depends on, and is the user's decision (the fix is a fixed-algorithm generator that is bit-identical to `SmallRng` on 64-bit). Instead: the chapter quotes the repository's recorded native numbers as native, and says so beside every widget that draws noise; `cargo test` pins those native numbers; `npm run check` pins the module's own realization and the divergence itself (`ch02_seeded_rng_probe`), so any change on either side fails the build until the text is revisited. Results that use no random stream (closed forms, the noise-free run) agree on both targets and are checked against the repository's numbers directly. |
| `robotics_slam` | compiled into the module for Chapter 6 (SLAM), 2026-10-15, beside the other five | compiles to wasm32 **with no source change**, including its sparse factor-graph solver (`faer` 0.24 with `std` and `sparse-linalg`, no `rayon`); nothing in it reads a clock or the OS entropy. **The module's replay of a recorded scan stream reproduces the native harness's trajectory errors to better than 1e-12 m and its loop-closure counts exactly** on four streams (`checks/ch06.ts`), even though ICP, the optimizer and the loop verifier run on a different floating-point library on wasm32: there is no random stream in the replay, so the seed divergence below does not touch it. The one function that draws noise (`ch06_verify`, two scans of a seeded room) is pinned by verdicts that hold for six seeds. The three recordings the widgets replay (1.9 MB and two of 2.5 MB) are not compiled in: `npm run wasm` copies the evidence files to `public/data/` and a check asserts each copy. A finding about the crate that the page makes visible: its χ² gate is 7.775, the Wilson–Hilferty approximation of the 95 % point of χ² with three degrees of freedom, where the exact value is 7.815 |
| Nothing else | `robotics_core`, `robotics_models`, `robotics_filters`, `robotics_sim`, `robotics_mapping`, `robotics_localization`, `robotics_slam` compile to wasm32 with no source change | none needed |

## 6. The installments

1. **Installment 1 (this one):** scaffold, contract, tooling, CI, then the chapters of Phases 1 and 2 written one at a time, each finished (design doc, widgets, checks, gates, a look in a browser) before the next starts: Chapter 1, then 2, then 3. Chapter 1 sets the voice the others follow.
2. **Installment 2 (Phase 3):** EKF localization was written and gated as Chapter 4; Installment 3 replaced that Chapter 4 (see §3) and it was restored as Chapter 5 on 2026-10-11. The Monte Carlo chapter was not started.
3. **Installment 3 (Phase 4):** Chapter 4, Mapping, replacing the EKF chapter under that number on the maintainer's decision; one chapter, with the benchmark comparison published as pending.
4. **Installment 4 (Phase 5):** Chapter 6, SLAM, written after the part closed in the roadmap (2026-10-15); one chapter, three widgets, the benchmark published with its caveat.
5. Each later phase gets its own installment, started only after that phase closes in
   `docs/ROADMAP.md`: add its rows to §3 and `lib/book-structure.ts`, write
   `design/Chapter-NN.md`, then the widgets, the WASM module additions, the checks and the
   MDX, and finish with a viewable build.
