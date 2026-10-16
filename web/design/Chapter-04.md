# Chapter 4: Mapping: Occupancy Grids from Known Poses (design doc)

Chapter 4 of the book. Written from the repository's mapping documents (`docs/phase-4-mapping.md` and `docs/phase-4-mapping/{problem,implementation,benchmark}.md`,
the roadmap's section on Mapping, closed 2026-10-11 with carried-over items), the source of `robotics_mapping::{occupancy, integrate, raycast, layout, entropy, esdf, eval, gate}`
as committed (`bcdb84f` to `13e0076`), and the evidence under `evidence/phase-4-mapping/2026-10-11-*`. It covers problem **M1 to M4** of the mapping claims (accuracy against the analytic
room, determinism, the rules of the grid, update cost) with the pose given by the simulator; **M5** (the map built from an estimated pose) is not covered and the chapter says so.

> **A note on the number.** This chapter took the number 4 on the maintainer's explicit decision of 2026-10-11 ("overwrite Chapter 4"), replacing the chapter on EKF localization that held it.
> That chapter was copied, with its wasm bindings, checks, widgets and the TOC it was built against, to `../archive-ch04-ekf-localization-2026-10-11/` (outside the repository; `web/` is untracked, so that copy is the
> only record). The localization part of the book is *planned* again (TOC §3) and will take the next free numbers when it is written. Chapters 1 to 3 are untouched.

## 1. Purpose and story arc

**The problem.** The TurtleBot3 of Part 3 knew where it was and nothing about the room except four points. A planner needs more: which places are free, which are occupied, and which have never been looked at.
The measurable form (claim M1 of the repository, seven checks whose limits were written before the first run): *a grid whose occupied cells lie within one 5 cm cell of the true surfaces
(95th percentile of the surface error at most 1 cell, maximum at most 1.5), with at most 2 % false-occupied cells, completeness at least 0.95, at most 5 % false-free samples, coverage at least 0.9 and an observable fraction of at least 0.5*, given the robot's true
pose at every scan. The chapter says plainly that this shows the *mapper* is implemented correctly: with the pose given, the only errors left are lidar noise and discretization.

**Arc.** *Problem → what we try first and why it fails → the idea → the Rust → the number again.* A scan is not a map (three reasons: it is in the sensor's frame, a beam says two things of unequal strength, evidence has to accumulate)
and the first attempt, "mark where each beam ended", already fails on the first claim: it cannot tell *free* from *unknown* (w4.1 shows the map being written from real scans). The idea is the log-odds update, an addition, with a clamp
so a cell can change its mind; four hits, not one, take a cell to occupied (the widget's foregrounded parameter, and the first thing the real runs changed). Which cells a beam crosses is a second decision (w4.3). Then comes the part of the story that makes this chapter
a chapter of *Probabilistic Robotics* and not a recipe: **the first runs failed**, for reasons that were not in the algorithm: a lidar stamp that lags its measurement, and then, on the held-out routes, an error that appeared on some routes and not on others and turned out, after a
root-cause investigation that falsified both of its leads, to be a mismatch between the bearings the Webots driver *publishes* for the lidar and the ones Webots *measures* at (pixel centres against N−1 intervals): half a degree at the ends of the scan, invisible at normal incidence, 6.8 cm of range at a wall seen at 70° (w4.2).
The fix was a correction in the mapper's input, verified by replaying recorded scans (not by new simulator runs). The chapter ends with the number again, measured honestly: all seven checks pass on four runs, M2 holds byte for byte, M4 costs about a millisecond a scan, and **what that does not show**.

**Epigraph.** "The final map shows regions probably occupied, probably unoccupied, and unknown areas." H. P. Moravec and A. Elfes, *High Resolution Maps from Wide Angle Sonar* (1985), abstract
(quoted from the CMU copy, https://frc.ri.cmu.edu/~hpm/project.archive/robot.papers/1985/al2.html; the paper is in the proceedings of the 1985 IEEE International Conference on Robotics and Automation, pp. 116–121, per https://www.ri.cmu.edu/publications/high-resolution-maps-from-wide-angle-sonar).

## 2. Prerequisites and position

* **Needs:** Chapter 1 (poses, `Se2`, the idea of a belief), Chapter 3 (the lidar model, ray casting, what a range reading means, the simulator Rusty). It does *not* need the localization part: the pose is handed to the mapper.
* **Introduces:** `OccupancyGrid`, `OccupancyParams`, `ScanIntegrator`, the two traversals `Bresenham` and `Supercover`, `BeamLayout`, `grid_entropy`, `signed_esdf` (a section, no widget) and the map-accuracy evaluator `eval` (its metrics, in the benchmark section).
* **Reuses:** `robotics_core::{Se2, LaserScan}`, `robotics_sim::World::raycast` (the beam-angle experiment), the color code (map belief is the *posterior*, a scan's returns the *measurement*).
* **Derived from:** the four mapping documents named above and `docs/SOURCE_MAPPING.md` (*Probabilistic Robotics* Ch. 9, Table 9.1 and Eq. 9.7).
* **Does not cover (and says so):** mapping with an estimated pose (M5, undecided in the repository); the rclpy and `nav2_costmap_2d` references (pending); 2-D TSDF and quadtrees (a later part); the ROS nodes beyond naming them; a more complex world; a real robot.

## 3. Foundation

Statement first, as in the chapter.

1. **The belief.** `bel(m) = p(m | z_1:t, x_1:t) ≈ ∏_i p(m_i | z_1:t, x_1:t)`: one Bernoulli per cell. The product is an assumption (two cells of one wall are not independent) and it is what makes the update one addition.
2. **Log-odds.** `l = ln(p/(1−p))`; Bayes' rule in odds form turns the recursive update into `l_t = l_{t−1} + l_inv − l_0` with prior `l_0 = 0` (probability one half): the update is a constant added to a number. *(Derivation, collapsed.)*
3. **The inverse sensor model, hand-crafted.** A beam that ends in a cell adds `l_occ = ln(7/3) = 0.847`; every cell it crosses first adds `l_free = ln(2/3) = −0.405`; nothing is said of cells behind the end (that is what *unknown* means).
   Values: the module's `model()` (asserted: `ln(0.7/0.3)`, `ln(0.4/0.6)`, lower clamp `ln(0.12/0.88)`, upper clamp 5.0, occupied threshold `ln 19`, free threshold `ln(0.35/0.65)`).
4. **One update per cell per scan.** The scan's cells are collected first (occupied wins over free), then each cell is updated once; the result does not depend on beam order, bit for bit.
5. **Thresholds and hits.** Three hits give 2.542, four give 3.389, against an occupied threshold of 2.944: four hits (asserted from `model()`). The upper clamp 5.0 means a saturated cell takes 6 misses to stop being occupied and 14 to be called free; the library's default clamp (probability 0.97) took 2 and 11.
   **Worked example 1** (pinned by `checks/ch04.ts`). *The cost of the clamp is slower forgetting*; the room is static, so it cannot be priced here (said in the text).
6. **No-return beams** clear free space up to `range_max` less 0.10 m (the margin and why).
7. **Entropy:** `H(l)` per cell in bits; an unobserved cell is one bit; never a quality measure alone (a confidently wrong map has low entropy). **Worked example 2:** an empty 140 × 140 grid has 19 600 bits; after the oblique run 12 131, with 15 098 cells decided (asserted).
8. **The bearing error.** Webots places column `j` of `N` at `fov/2 − (j + ½) fov/N`; the driver publishes `fov/2 − j fov/(N−1)`. The error `ε_j = j (fov/(N−1) − fov/N) − fov/(2N)`: −8.727 mrad at beam 0, −0.024 at beam 179, +8.727 at beam 359 (asserted for all 360 beams).
   Two consequences, derived in a `<Derivation>`: the *range residual* of a beam at a wall is `r tan(i) ε` and the *distance of the mapped hit from the face* is `r sin(i) ε`. **Worked example 3:** a wall 1.0 m away at 70° incidence: beam 0 reads 2.855 m, its range is off by 6.84 cm, its hit lands 2.34 cm from the face (asserted).
9. **The ESDF:** signed, positive in free space, zero level at the cell faces, `d − r/2` outside and `−(d − r/2)` inside, unknown counted as free; a short section with the listing, no widget and no number.
10. **How the map is judged** (M1's metrics; the incidence limit of 60° and the closed-cell completeness lookup are *decisions*, stated as such).

## 4. Conceptual

In the order the reader meets them. The chapter opens with a measurable problem and *immediately* shows the answer running (w4.1 sits in "The problem"); the other two come where the reader needs them.

1. **w4.1, "A map written one scan at a time"** (in *The problem*). The oblique evaluation route's 504 recorded Webots scans, replayed in order through the repository's mapper (compiled to wasm32), with the true pose. Autoplays (about 6 s). Foregrounded: the number of hits that calls a cell occupied.
   Kills "one lidar return is enough evidence to call a cell occupied": at one hit, 29 of 986 occupied cells are farther than a cell from every true surface (2.94 %, over the 2 % limit); at four there is one. Click-to-probe shows a cell's log-odds history against the thresholds and the clamp.
2. **w4.3, "Which cells does a beam cross?"** (in *Building intuition*, where free cells are first written). Both traversals of `robotics_mapping::raycast`, one beam, a draggable angle. Kills "every ray-tracing method marks the same free cells".
   **Decision: does a third widget earn its place? Yes**, for three reasons, and the reasons are the bar for any later such widget: (a) the project's frozen choice (Bresenham, with the supercover as the measured alternative) rests on it, and a reader who does not see the two disagree cannot follow why the choice was a decision;
   (b) the difference is a property of the beam's *angle*, which is exactly what a drag shows and a figure does not; (c) it is cheap and entirely crate code. What it does not claim: that the traversal matters much *here*. At the frozen clamp of 5.0 either traversal passes all seven checks on all four routes the project swept; below 5.0 the wall-lobes route fails a check with either (asserted from `30-stageC_clamp_sweep_new_evaluator.txt`).
3. **w4.2, "Half a degree, and a wall at an angle"** (in the beam-angle section: the root-cause story, first-class). A wall, a draggable incidence; the module reads the same 360 true-bearing ranges with the published bearings and with the pixel-centre ones; a zoomed panel in 5 cm cells; a curve of beam 0's error against the wall's incidence.
   Foregrounded: the incidence. Kills "a calibration error of half a degree cannot matter to a map". Honest limits stated in the caption: this table is the part the correction removes; the real sensor has about twice as much at its two ends, unexplained.

## 5. Practical

Listings (each a literal excerpt, asserted by `check:book`; none of the lines printed contains the word that the book never says):

* `crates/robotics_mapping/src/occupancy.rs`: `OccupancyParams`, `logit`, `OccupancyGrid::add_at` (the clamp), `classify`.
* `crates/robotics_mapping/src/integrate.rs`: the beam loop and the one-update-per-cell pass of `ScanIntegrator::integrate`.
* `crates/robotics_mapping/src/raycast.rs`: `Bresenham::next`.
* `crates/robotics_mapping/src/layout.rs`: the `PixelCentres` branch of `BeamLayout::apply`.
* `crates/robotics_mapping/src/esdf.rs`: the sign and the half-cell in `signed_esdf`.
* `crates/robotics_wasm/src/ch04.rs`: `frozen_params` and `Ch04Replay::integrate_into` (the page runs the node's code path, in order).
* A worked example with its expected output (a `text` block of the four hit counts, asserted).

**Findings from compiling the algorithm crates (TOC §5).** `robotics_mapping` compiles to `wasm32-unknown-unknown` **with no source change** to it or to any crate it depends on. Its dependency on `robotics_core` brings in `rand`, so a standalone
`cargo check --target wasm32-unknown-unknown -p robotics_mapping` stops at `getrandom`'s "enable the `wasm_js` feature" like every other crate; inside `robotics_wasm` the already-selected backend resolves it. There is no blocker and no hand-ported TypeScript.
**Unlike Chapters 2 and 3 there is no seeded stream in this chapter's module**, so the native-versus-wasm32 `SmallRng` divergence of TOC §5 does not touch it; the proof is not an argument but a check: the module (wasm32) replaying the recorded scans four at a time writes **exactly** the `final-grid.txt` the node saved, byte for byte (169 953 characters), and the native `cargo test` says the same.
**Data on the page.** The recording is not compiled into the module (1.5 MB); `npm run wasm` copies `integrated-scans.csv` of the oblique evaluation run to `public/data/` and the page fetches it; `checks/ch04.ts` asserts the copy equals the evidence file.
The bindings are in the book's single module (`crates/robotics_wasm/src/ch04.rs`, output `lib/wasm/pkg/`), not in a separate `lib/wasm/mapping/` bundle: a second module would double the `.wasm` the reader downloads and break the one loader of TOC §4.

## 6. Widget manifest

| Id | Component | Rust called (`robotics_wasm`) | Foregrounded parameter | Kills the misconception that |
|---|---|---|---|---|
| w4.1 | `ReplayMap` | `Ch04Replay::{new, step, set_occupied_probability, states, counts, entropy, phantom_cells, last_pose, last_points, route, trace_cell, model, geometry, hits_to_occupy}` | hits needed to call a cell occupied (1 to 6) | one lidar return is enough evidence to call a cell occupied |
| w4.2 | `BeamAngle` | `ch04_beam_wall`, `ch04_beam_wall_max_incidence`, `ch04_beam_error_table` | incidence of the straight-behind beam on the wall | a calibration error of half a degree in a beam's bearing cannot matter to a map |
| w4.3 | `TwoTracers` | `ch04_trace` | beam angle | every ray-tracing method marks the same free cells along a beam |

## 7. Exercises

1. **F, difficulty 2, "From odds to an addition".** Derive `l_t = l_{t−1} + ln(p(m|z)/(1−p(m|z))) − l_0` from Bayes' rule for a cell, and say what `l_0` is. *Sketch:* Bayes in odds form; independence of the measurements given the cell; take logs.
2. **F, difficulty 3, "The offset is not the residual".** Show that the range residual of a beam at a wall is `r tan(i) ε` and the hit's distance from the face is `r sin(i) ε`, and explain the factor `cos(i)`. *Sketch:* `r = d / cos(i)`; rotate the bearing by `ε`; project on the normal.
3. **C, difficulty 1, "Four hits".** Predict how many hits take an unknown cell to occupied at probability 0.95 (w4.1). *Answer:* four (3.389 ≥ 2.944 > 2.542).
4. **C, difficulty 2, "Where do the phantoms stand".** Predict, before lowering w4.1 to one hit, where the false cells appear. *Answer:* in the row just outside the walls, where a hit lands past the face and no later beam crosses the cell to clear it.
5. **C, difficulty 2, "Which end of the scan".** Predict which of beams 0 and 359 puts its hit beyond the wall for a wall tilted to the left (w4.2). *Answer:* read off the widget; they miss on opposite sides, and only one side creates a cell outside the wall.
6. **P, difficulty 2, "A smaller clamp".** Change the upper clamp in `frozen_params` (`crates/robotics_wasm/src/ch04.rs`) to the library default, probability 0.97, and say what happens to the pillar cells in w4.1 and how many misses it now takes to un-occupy a saturated cell (answer: 2, against 6).

## 8. Modernization notes

* **Against the 2005 text.** *Probabilistic Robotics* Table 9.1 updates every cell in the perceptual field of the measurement once, with an inverse sensor model; so does the crate, and for the same reason (the update is order-independent). The book's `l_0` is zero here. Differences: a clamp (OctoMap's policy), a hand-crafted model with no beam width, an explicit
  no-return policy, a motion gate and a repeat filter for a lidar whose time stamp lags its measurement, and a beam-bearing correction for one sensor. Thrun's later work learns the inverse model with forward sensor models (a reference); this chapter does not.
* **Limitations carried from the repository, repeated and not hidden.** The parameters were chosen on the four runs the first analysis burned; the evaluator's two rules (60° incidence limit, closed-cell completeness) changed between round one and round two, so round two is graded against the metric of 2026-10-11 and the old metric would still have failed one stream;
  three routes in one room are rows and not a rate; the maximum surface error is exactly on its limit (1.5 cells) in two runs; nothing here concerns an estimated pose, another world or a real robot; about −9 and +10 mrad of beam error at the two ends of the scan is unexplained; a higher clamp trades slower forgetting for stability and a static room cannot price it.
* **The benchmark section ships honestly incomplete.** The M1, M2 and M4 numbers are the repository's own, from the verdict files, every one asserted. The comparison of the Rust mapper with an rclpy port and with `nav2_costmap_2d` is **pending**: no number, no placeholder.
  The update latencies were measured on a debug build.
* **References** (all verified, see the chapter): Moravec and Elfes 1985; Elfes 1989; Thrun, Burgard and Fox 2005; Thrun 2003; Hornung et al. 2013; Amanatides and Woo 1987; Bresenham 1965; Millane et al. 2024.
