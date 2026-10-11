# Phase 5, implementation: the Rust side

> **Status (2026-10-11): design settled; `robotics_slam`, the verdict script and the example are
> built (section 8).** Section 8 is the only authority on what exists; anything it does not list
> does not exist. No ROS package, node or launch file exists for this phase, and none is
> started: that is a decision (DS2), not an omission. The decisions of section 5 were made on the
> instruction to decide and document them, so every one is open to the project owner's revision;
> none was separately confirmed, unlike Phase 4's 5.1. The claims S1 to S8 of
> [problem.md](problem.md) section 4 are likewise a proposal until confirmed.

## 1. Already in the repository

| What | Where | Used for |
|---|---|---|
| `Se2` with exact `exp`/`log`/`adjoint`, `⊞`/`⊟` through the `Manifold` trait | [`robotics_core::se2`](../../crates/robotics_core/src/se2.rs) | every pose, every residual |
| `Gaussian`, χ² quantiles, `seeded_rng` | `robotics_core::prob` | the gate's threshold, seeded tests |
| `LaserScan`, `PointCloud2`, `Timestamp` | `robotics_core::types` | scan input; no new scan type |
| `ParticleSet<S>` with log weights, low-variance resampling, ESS | [`robotics_filters::particle`](../../crates/robotics_filters/src/particle.rs) | the RBPF's population |
| odometry motion model with a sampler | `robotics_models::motion::odometry` | the RBPF's proposal, EKF-SLAM's process noise |
| NEES with χ² bounds | `robotics_filters::consistency` | S2 and S7 |
| Rusty: segment `World`, exact ray cast, seeded 2-D lidar, slip, encoders | [`robotics_sim`](../../crates/robotics_sim/src/) | synthetic runs with exact ground truth (dev-dependency only) |
| `PillarDetector` | `robotics_localization::pillars` | landmark detections for EKF-SLAM in node-level use; the library takes detections as arguments and does not depend on it |
| evidence layout, verdict-as-a-file convention | `scripts/evidence_root.sh`, `scripts/phase4_verdict.py` | the same discipline for `scripts/phase5_verdict.py` |
| recorded scans with the sensor pose they were integrated at | `evidence/phase-4-mapping/2026-10-11-development-*/integrated-scans.csv` | recorded data for later S1a/S4 runs (read only; the poses in it are ground-truth-adapter poses) |

Not in the repository, and decided not to be needed: a TSDF and a quadtree (DS13).

## 2. What Phase 5 adds

| Id | Task | Where | Built in this pass? |
|---|---|---|---|
| T1 | SE(2) calculus: left and right Jacobians, the relative-pose residual with exact Jacobians | `robotics_slam::geom` | yes |
| T2 | voxel-hash point map with a deterministic hasher, point cap and ring search | `robotics_slam::voxel` | yes |
| T3 | ICP, point-to-point (closed form) and point-to-plane (Gauss-Newton on the SE(2) tangent), robust weighting, the ICP information matrix and degeneracy flag | `robotics_slam::icp` | yes |
| T4 | KISS-style adaptive correspondence threshold and the LiDAR odometry front end that uses it | `robotics_slam::{adaptive, odometry}` | yes |
| T5 | NDT, with its own gradient and Gauss-Newton Hessian | `robotics_slam::ndt` | yes |
| T6 | factor graph over poses and point landmarks; GN and LM; Huber, Cauchy and Geman-McClure kernels; dense and sparse (`faer`) solvers; marginal covariances | `robotics_slam::{kernel, factor_graph}` | yes |
| T7 | pose graph: keyframes, odometry edges, loop-candidate generation, ICP verification with the ambiguity, degeneracy and χ² checks | `robotics_slam::pose_graph` | yes |
| T8 | EKF-SLAM with provisional landmarks | `robotics_slam::ekf_slam` | yes |
| T9 | grid RBPF with a copy-on-write grid | `robotics_slam::rbpf` | yes |
| T10 | `scripts/phase5_verdict.py` with every tolerance, committed before the first evidence run; its self-test | `scripts/` | yes (script and self-test; **no evidence run**) |
| T11 | `slam_synthetic_run` example: a seeded Rusty run that writes the metrics file the verdict script reads | `crates/robotics_slam/examples/` | the example, yes; **not run into `evidence/`** |
| T12 | `scan_matcher_node` and `pose_graph_node`, submap rendering into `/map`, `map -> odom` | `ros2_ws/src/…` | **no, deferred pending Phase 4's M1 (DS2)** |
| T13 | the evidence runs: S1 to S6 on synthetic and recorded data, then Webots | `evidence/phase-5-slam/` | **no** |
| T14 | informative references: `slam_toolbox` (installed in `/opt/ros/jazzy`) on the same recorded scan stream, graded against the same ground truth | `scripts/` | **no**, non-blocking |

## 3. Conventions

Those of `robotics_core`, unchanged (see its crate docs and `docs/CLAUDE.md`): tangent order
translation-first `(v_x, v_y, ω)`, right perturbation `X ⊞ δ = X · exp(δ)`, `Y ⊟ X = log(X⁻¹ Y)`,
angles in (−π, π], covariance of a pose in the tangent space at the pose, `R` motion noise and
`Q` measurement noise.

A relative-pose measurement `Z` of `X_j` seen from `X_i` has the residual

```text
e(X_i, X_j) = log( Z⁻¹ · X_i⁻¹ · X_j )                      (the tangent at Z)
∂e/∂δ_j =   J_r⁻¹(e)
∂e/∂δ_i = − J_r⁻¹(e) · Ad(T⁻¹),   T = X_i⁻¹ X_j
```

with the exact right Jacobian of SE(2) (`geom::right_jacobian`, from the series
`Σ ad_τⁿ/(n+1)!`), not the shortcut `J_r⁻¹ ≈ I`. A loop closure enters an optimization with a
residual of metres and a radian, where the shortcut is wrong by a factor; the unit tests
compare every Jacobian with central differences at such residuals. The reference chapters
use several tangent orderings in one chapter (translation-first for the graph, rotation-first
for the point-to-plane step, translation-first again for the information matrix, and
`(t_x, t_y, θ)` for NDT) and a small-angle step about the centroid for point-to-plane. This
crate uses one ordering and one perturbation everywhere (DS3).

## 4. Planned interface

Planned; the code in section 8's list is the authority. All types are plain data, there is no
I/O and nothing reads a clock.

```text
// scan matching
VoxelMap::new(cell, max_per_cell) -> insert(points) -> nearest(q, radius) -> Option<(idx, p)>
icp::register(source, target_map, init: Se2, &IcpConfig) -> IcpResult
    { pose: Se2, converged, iterations, inlier_fraction, rmse, information: Matrix3, degenerate }
adaptive::AdaptiveThreshold::{new, threshold, update(pred_vs_est: Se2)}
odometry::LidarOdometry::{new, step(scan) -> StepResult { delta, pose, threshold, icp }}
ndt::NdtMap::from_points(points, cell) ; ndt::register(source, &NdtMap, init, &NdtConfig) -> NdtResult

// optimization
FactorGraph::{add_pose, add_landmark, add_factor(Factor, noise, kernel)}
FactorGraph::optimize(&mut values, &OptimizerConfig) -> OptimizeReport { iterations, initial_cost, final_cost, converged }
FactorGraph::marginal_covariance(&values, &[VarKey]) -> DMatrix       // joint block

// the pose graph and the loop pipeline
PoseGraph::{new(cfg), add_keyframe(scan, odom_delta, odom_info) -> KeyframeId}
PoseGraph::propose_loops(KeyframeId) -> Vec<LoopCandidate>
PoseGraph::verify(&LoopCandidate) -> Verification { Accepted{measurement, info} | Rejected(Reason) }
Reason = { NotConverged, PoorFit, Degenerate, Ambiguous, GateExceeded{chi2}, NoOverlap }

// alternative back ends
EkfSlam::{new, predict(delta, R), update(&[RangeBearing]) -> UpdateReport}
Rbpf::{new(cfg, seed), predict(delta), update(&LaserScan), pose_estimate, n_eff}
```

The ROS graph of the two nodes that would use this is in `docs/ROS2_INTERFACES.md`
(`/slam/pose`, `/slam/path`, `/slam/graph`, `/map`, `map -> odom`). It is not repeated here
because nothing implements it yet.

## 5. Decisions

### 5.1 Not yet confirmed by the project owner

None of the decisions below has been confirmed. They are listed so the owner can change any
of them before the evidence runs; changing one *after* an evidence run is the visible edit
`scripts/phase5_verdict.py`'s history exists to expose.

### 5.2 The decision record

| Id | Decision | Where it is argued |
|---|---|---|
| DS1 | The claims are S1 to S8, not M-numbers (Phase 4 owns M1 to M5). | problem.md section 4 |
| DS2 | No node, no ROS package and **no `todo!()` stub of one** is created. | below |
| DS3 | One SE(2) convention, exact Jacobians, residual `log(Z⁻¹ X_i⁻¹ X_j)`. | section 3 |
| DS4 | ICP: point-to-plane is Gauss-Newton on the SE(2) tangent; point-to-point is the closed-form 2-D alignment; both use a hard correspondence threshold and a Geman-McClure weight. | below |
| DS5 | The voxel hash uses a fixed hasher and breaks every tie by insertion index. | below |
| DS6 | The adaptive threshold follows the KISS-ICP recipe with a per-scan constant-velocity prediction, never a per-second one. | below |
| DS7 | The information matrix is `(1/σ²) Σ w JᵀJ`, regularized without a hidden nudge, with a degeneracy flag; `info_inflation` is a documented, frozen parameter. | below |
| DS8 | NDT keeps the outlier terms `d1`, `d2` that the reference chapter drops. | below |
| DS9 | The optimizer: Levenberg-Marquardt with Marquardt scaling on the accepted-step test, IRLS kernels, a prior factor for the gauge, and an error (not a nudge) when the system is singular. | below |
| DS10 | Keyframes are created by distance or rotation since the last one. | below |
| DS11 | The loop gate is a χ² test on the **exact marginal covariance** of the relative pose plus the ICP covariance, and it is one of four checks. | section 6 |
| DS12 | The RBPF has its own copy-on-write grid and does not use `robotics_mapping`. | below |
| DS13 | **No TSDF and no quadtree. Submap rendering does not need either.** | below |
| DS14 | EKF-SLAM has provisional landmarks with the gates of Chapter 14; no existence log-odds and no retirement. | below |
| DS15 | The pipeline is deterministic: no iteration order of a hash map reaches a result, no OS entropy, and the optimizer has no parallelism. | below |
| DS16 | Where this crate departs from the reference chapters, it says so (list below). | below |
| DS17 | A map is rendered by replaying keyframe scans at the optimized poses; nothing is repaired incrementally. | below |
| DS18 | Every default parameter is either derived (with the derivation next to it) or marked as taken from a reference and unvalidated; all are frozen after the development run. | below |

**DS2. No node, no stub.** The brief allows stubs "if useful to shape the crate's API". None is
useful: the API is already fixed by what the tests need, and a package named
`pose_graph_node` containing `todo!()` would appear in `ros2 pkg list`, in the colcon build and
in `ARCHITECTURE.md`'s table, as the beginning of a node that publishes `/map`. That is the
thing this pass must not start. When Phase 4's M1 closes, the nodes are written from scratch
against a library that has already been tested, and the first commit of each says so.

**DS4. ICP.** *Point-to-plane* (the default for scan-to-map) minimizes `Σ w_k (n_k · (T p_k − q_k))²`
with `T ⊞ δ`, the residual Jacobian taken on the SE(2) tangent, the step solved from the 3 by 3
normal equations with a ridge proportional to the trace (not an absolute one: an absolute ridge
"slides the scan" in a degenerate direction, as the reference chapter notes) and the correspondences
found again every iteration. *Point-to-point* is the closed-form weighted alignment
(`θ = atan2(Σ w a×b, Σ w a·b)` on centred clouds; there is no SVD in 2-D), iterated with
re-association. Correspondences farther than the threshold `τ` are dropped; the rest are weighted
by Geman-McClure with scale `τ/3`, so the weight at the rejection boundary is one per cent, which
makes the cost continuous at the edge instead of having a cliff. Normals come from a PCA of the
target neighbours within 0.20 m, and a normal is refused when the smaller eigenvalue is more than
0.1 of the larger: derived from a 15 mm lidar noise over a ±0.2 m support (ratio about 0.02 for
a wall; a corner is far above 0.1). A point with no usable normal falls back to a point-to-point
residual, never to a guessed normal. Convergence: translation step under 1e-4 m and rotation step
under 1e-4 rad (a fifth of the lidar noise and far below it), or 50 iterations; a run that stops
on the iteration limit reports `converged = false`. Nearest neighbours flip between iterations on a
discrete map, so a registration that has reached the answer can jitter by a fraction of a millimetre
without ever meeting that tolerance; the result therefore also carries `settled`, true when the last
step is under ten times the tolerance, and the front end and the loop verifier believe `settled`
(found on the first long run of the front end, section 8).

**DS5. Voxel hash.** `std::collections::HashMap` seeds its hasher randomly per process, and
iterating it would make ICP differ between two runs of the same binary. The map uses a fixed
multiplicative hasher over the two cell indices, and no result depends on iteration order: a
query visits cells in a fixed ring order and ties on distance go to the lower insertion index.
Cell size and point cap are parameters (initial: 0.10 m and 8; a wall at 5 cm spacing puts about
two points per cell per scan, so eight keeps a few overlapping scans and bounds a cell). A
returned range at or beyond `range_max` is dropped, not clamped: it is a statement about
nothing.

**DS6. Adaptive threshold and prediction.** After each registration the model deviation
`δ = ‖Δt‖ + 2 r_max sin(Δθ/2)` of the *prediction against the result* is the worst displacement
the missed motion could cause at the end of the sweep (`r_max` is the scan's `range_max`, 3.5 m
here). `σ² = mean(δ²)` over the history, updated only when `δ` exceeds a floor (initial 0.01 m,
the lidar's noise order: below it the sample is noise and including it collapses `τ` to zero when the
robot is still), and `τ = 3σ`, with an initial `τ₀ = 0.5 m` before any sample. The prediction is
the **last per-scan increment**, applied once. It is deliberately not scaled by the time between
stamps: the Webots lidar measures on a regular 0.2 s lattice and stamps irregularly
([Phase 4 implementation 5.2, item 2](../phase-4-mapping/implementation.md)), so a per-second velocity
times a stamp difference would inject the stamp lag into the prediction. The front end does not
deskew, for the reason D9 of Phase 4 gives (a Webots scan is one instant); a real-robot driver
would need it.

**DS7. Information matrix.** For the final correspondences, `Λ = (1/σ²) Σ w_k J_kᵀ J_k` with `J_k`
the residual Jacobian on the SE(2) tangent and `σ²` the weighted residual variance floored at the
square of the lidar noise (so a perfect synthetic fit does not report infinite certainty).
`info_inflation` divides `Λ`; it is **1.0 until S2 has been measured on the development run, then
frozen**, and a different value is a visible edit. The reference chapter admits its inflation factor
is a fudge, and so is this one; the point is that it is one named number with a measurement behind
it, not a constant inherited from a widget. The matrix is regularized by adding `ε·tr(Λ)/3·I` with
`ε = 1e-6`, nothing absolute. A `degenerate` flag is set when the smaller eigenvalue of the
translation block is under 0.02 of the larger (a corridor, or one wall in view): the information
matrix is then correct about being rank-deficient and the flag lets the verifier refuse to use the
registration as a loop constraint. Point-to-point has no normals, so its `J_k` uses both axes of
the displacement. **For point-to-plane the sum runs over the correspondences that have a normal
only.** A correspondence without one (a corner, or a wall sampled too sparsely for a normal) still
steers the solve through its point-to-point residual, but it contributes no information: along a sparse
wall it invents a constraint along the wall, and the first version, which counted it, did not flag a
corridor as degenerate (found by the corridor test, section 8).

**DS8. NDT.** 2-D cells of 0.5 m with at least 5 points; the cell covariance has its smaller
eigenvalue clamped to 0.01 of the larger (a wall cell is otherwise rank one); score
`S = Σ d1 exp(−(d2/2) dᵀΣ⁻¹d)` over the 3 by 3 cells around the transformed point, with the
Magnusson constants `d1, d2` computed from an outlier ratio of 0.1 and the cell area. With those
constants `d1 = ln(c2/(c1+c2))` is negative, so `S` is the negated Magnusson score and is minimized:
it equals `d1` at a perfect fit. The reference
chapter drops `d1, d2` (its score has unit amplitude and unit scale), which is a simplification
of the standard method, not the method; the standard form is kept and the difference is listed
under DS16. Gradient exact on the SE(2) tangent, Hessian the Gauss-Newton term only, so the step
is a descent direction (the same reasoning as the reference), with a Levenberg term.

**DS9. The optimizer.** `(H + λ·diag(H)) Δ = −b`, accepted when the cost falls, with
`λ ← λ · max(1/3, 1 − (2ρ − 1)³)` on the gain ratio `ρ` and `λ ← 10 λ` on rejection; initial `λ`
is 1e-3. A kernel enters by IRLS: a factor's contribution to `H` and `b` is multiplied by `w(e)`
with `e = ‖Σ^{-1/2} r‖`, **the weight and not its square root, in both** (the reference sources
differ on this, and the two conventions give different normal equations). Convergence when the
relative cost decrease is under 1e-10 or the step is under 1e-9. The gauge is fixed by a prior
factor on one pose, and if the Cholesky factorization fails the optimizer returns
`SlamError::SingularSystem` instead of nudging the diagonal until it succeeds: a missing prior is
a bug in the caller and a silent nudge hides it. Kernels: Huber (convex, the default), Cauchy,
Geman-McClure. A redescending kernel is non-convex and can lock a bad start onto a confident wrong
map; Huber cannot reject a gross lie (the reference chapter's own table shows it barely helps a
1.6 m one), and that is what the gate is for, so the loop edges default to Huber with
`k = √χ²(3, 0.95)` (2.796) and the gate is the defence against large lies. Linear solve: a dense
`nalgebra` Cholesky as the reference oracle, and the sparse `faer` LLT (the normal equations are
assembled from triplets; the symbolic factorization is built once per structure). The two must
agree to 1e-9 on every test graph.

**DS10. Keyframes.** A keyframe is created when the pose since the last keyframe has moved 0.5 m or
rotated 0.35 rad. 0.5 m is about a seventh of the sensor range, so consecutive keyframes share most
of their view; 0.35 rad is the order at which a scan in this room loses a wall. These are design
choices, not derived from data, and are frozen after the development run. Each keyframe stores its
downsampled scan points in the sensor frame.

**DS12. RBPF grid.** `robotics_mapping::OccupancyGrid` is a flat row-major `Vec<f32>`; cloning one per
particle per resample is `O(cells)`, and the standard answer is copy-on-write chunks. More
importantly, using the Phase 4 grid would make every RBPF number partly a statement about a mapper
whose M1 is open (problem.md section 6). So the RBPF carries a small private log-odds grid in 16 by 16
chunks behind `Arc`, cloned lazily on first write (`Arc::make_mut`). It is a likelihood structure
for weighting, not the published map, and says so. This duplicates a few dozen lines of a log-odds
update, which is the price of independence; it is revisited when Phase 4 closes. The proposal is the
odometry motion model (FastSLAM 1.0 style), the weight is the tempered scan likelihood on the
particle's own grid, resampling is low-variance when `N_eff < N/2`. There is no scan-matching
refinement of the proposal in this pass (the GMapping recipe of the reference chapter), which is a
stated limit: it means the RBPF needs more particles than the book's variant for the same accuracy.

**DS13. TSDF and quadtree: no.** The question was inherited from Phase 4 with the words "whether
submap rendering needs a TSDF is decided here". It is decided, and the answer is no, for five reasons,
each checkable.
1. *The matching does not use one.* Scan matching here is against points (a voxel hash), as in the
   reference chapter's own pipeline, which has no TSDF or quadtree in it (checked: its Chapter 16
   text matches against a point set; the occupancy-grid `Submap` appears only in its design
   document, not in the text or its code).
2. *The room is small.* 6 by 6 m at 5 cm is 14 400 cells, tens of kilobytes as a flat `f32` array.
   The reference chapter's own memory formula puts the quadtree's crossover with a flat grid at about
   1.5 cm for *its* apartment (108 m², 20-byte nodes), and it reports 239 kB for the tree against 43 kB
   for the flat grid at 5 cm there (its Chapter 19, section 3.1 and the comparison table). Those are
   the book's numbers for the book's map, not measurements of this room; they say the quadtree buys
   memory in large, mostly empty volumes and loses in a small, fully traversed 2-D one, which is what
   this room is. Phase 8's larger spaces are handled by growing the grid, as Phase 4 already decided.
3. *The ESDF Phase 6 and 7 need already exists* (T5 of Phase 4, the exact EDT), so a TSDF is not
   needed to seed one.
4. *Rendering after a loop closure is a replay for every representation.* A fused TSDF, like a log-odds
   grid, cannot be edited when poses move; the scans must be re-integrated at the corrected poses.
   The reference chapter says as much, and no representation has an advantage on this point.
5. *Sub-cell surfaces are not asked for.* A TSDF's gain is sub-cell accuracy and marching-squares
   surface extraction; nothing in the ROADMAP's consumers of `/map` needs a contour.

Revisit triggers, so the answer is not permanent by accident: a map extent beyond about a million
cells, or Phase 6 asking for sub-cell surfaces, or scan-to-map matching against a distance field
becoming the measured bottleneck. None is true of the room.

**DS14. EKF-SLAM.** State `[x y θ | l₁ l₂ …]`, dynamic size (`DVector`/`DMatrix`, since the const-generic
`Ekf` cannot grow). The pose starts as the origin with zero covariance (the gauge). Prediction touches the
pose block and the pose-landmark strip only. Measurements are range-bearing detections; association is
maximum likelihood over mapped landmarks with the Chapter 14 gates, taken from the χ²(2) quantiles
(0.99 and 0.999, 9.21 and 13.82): inside the first, update; between the two, discard (a deliberate dead
band); beyond the second, a candidate in a world-frame provisional list, promoted to a landmark after 3
consistent sightings (the number is a reference's, unvalidated). Observations are applied one at a
time, the covariance in Joseph form. No existence log-odds and no retirement (the reference's values
for those are in its widget code only): landmarks in the four-pillar room never go away, and
this is listed as a limit. Landmarks are detections the caller extracts (the Phase 3 pillar detector
has a +2.5 to 3.0 cm range bias on the pillar centre, which the caller is responsible for).

**DS15. Determinism.** Fixed hasher (DS5); no `HashMap` iteration in a result path; no `rayon` (the `faer`
dependency is built without it, and the sparse solves run single-threaded); the RBPF takes a seed and uses
`robotics_core::seeded_rng`; no clock. S6's bit-identical replay is then a property that can be tested
rather than hoped for, with one caveat recorded here: `faer`'s floating-point summation order in its
supernodal factorization is fixed for a given structure, but an update of the crate can change it, so the
replay claim is for a given lock file.

**DS16. Departures from the reference chapters.** (a) Exact `J_r⁻¹` Jacobians (the Chapter 16 text uses
`I` and `−Ad(Z⁻¹)`; the two agree at zero residual only). (b) One tangent ordering. (c) The kernel weight
multiplies the factor's `H` and `b`, not its square root. (d) NDT keeps `d1, d2`. (e) The gauge is a prior
factor, not a hard clamp of rows and columns. (f) The loop gate uses the exact marginal covariance from the
sparse factorization (which the reference says is the exact version of its own approximation) instead of a
covariance compounded over the gap; and it adds the ICP covariance to it, which is the covariance of the
innovation. (g) The gate is one of four checks (section 6); the reference has two. (h) The reference's
prediction is odometry; KISS-ICP's, and here, is constant velocity per scan. (i) No hidden diagonal nudge.
Each of these is a place where the repository's `Se2`, the reference's text and its own widget code
disagree, or where the text is silent, and the choice made is the one a test in this crate can pin.

**DS17. Rendering a map.** The pose graph keeps each keyframe's scan. A map is produced by integrating those
scans, at the optimized poses, into a Phase 4 `OccupancyGrid` with the Phase 4 `ScanIntegrator`, from scratch.
No incremental repair exists or is planned: after a loop closure every keyframe moves a little and a replay of
a few hundred scans costs less than reasoning about which cells to undo. This is a note for the deferred node;
the library does not render a map, because the grid it would render into is the thing Phase 4's M1 has not
validated.

**DS18. Parameters.** Initial values in this file and in the crate's `Default` impls are of three kinds, and
each says which: *derived* (a sentence of arithmetic from a sensor or geometry fact), *chosen* (a design
choice with a stated reason and no data behind it) and *reference* (a value from the reference chapters'
widget code, recorded here as unvalidated: the EKF-SLAM promotion count, the RBPF weight tempering). All are
frozen after the development run, as in Phase 4; the evaluation runs do not tune them.

## 6. Loop closure: the detector and the verifier

Two stages, and the second is the one that matters.

**Proposal (`propose_loops`).** For a new keyframe `k` and every earlier keyframe `j` with a path length
between them of at least 6 m (not an index gap, which depends on the keyframe rate), take the graph's
current estimate of the relative position and its exact marginal covariance (a joint marginal of
`(x_j, x_k)` pushed through the relative-pose Jacobian). `j` is a candidate when the estimated distance is
under `r_overlap + 3σ_max`, where `r_overlap` (1.5 m, chosen: two scans of range 3.5 m overlap well inside
it) is how far apart two views can be and still share a wall, and `σ_max` is the largest standard deviation
of the relative position, capped so the radius never exceeds 3.0 m. The nearest three candidates are kept.
Proposal is pure geometry on the estimated trajectory; it cannot find a loop the drift has carried out of
its own covariance, and the documents say so (problem.md 4.1).

**Verification (`verify`).** Register keyframe `k`'s scan against a submap of the points of keyframes
`j−1, j, j+1`, **excluding keyframe `k` and everything after it** (a scan must never be registered against
itself: the first version included the candidate's neighbours without that rule and, on adjacent keyframes,
matched a scan to itself with zero residual; found by the first test of a rejected candidate), starting
from the graph's current relative pose. A candidate is rejected for the **first** check that fails, in this
order, and accepted only if none does:

0. *Overlap.* At least 20 per cent of the source points have a correspondence within the loop threshold
   (0.6 m); otherwise `NoOverlap`. `NotConverged` if the registration has not settled (DS4).
1. *Fit.* The registration settled and at least half the source points, and at least 80 of them, are
   **explained**: a target point lies within 5 cm of the transformed source point (`derived`: about three
   lidar sigmas; the fraction and count are `chosen`). Explained points, not the RMS of the correspondences: the
   RMS over everything within the search threshold is polluted by the points a changed scene adds (the first
   version used it and refused a loop whose scene had a wall newly hiding part of the view while the
   rest matched to a centimetre, which is a good constraint). A *partly changed* scene whose remaining points match is accepted, deliberately:
   robustness to a moved object is what the support fraction buys.
2. *Not degenerate.* The information matrix is not flagged degenerate (DS7): a corridor or one wall
   in view is not a place.
3. *Consistent.* With `ν = log(Z_icp⁻¹ · T_pred)` (`T_pred` the graph's current relative pose) and
   `S = Σ_rel + Σ_icp` (the marginal covariance of the relative pose plus the inverse of the registration's
   information, which is the covariance of the innovation), `νᵀ S⁻¹ ν ≤ χ²(3, 0.95)`. Rejecting here costs
   a missed loop; accepting a false one costs the map. This check **cannot** reject a false candidate that
   sits inside the graph's own uncertainty, and the document does not pretend otherwise: it removes the large
   lies, checks 2 and 4 are what face the small, aliased ones.
4. *Unambiguous.* ICP is re-run from eleven other starts: the solution moved ±0.3 m along each axis,
   ±0.3 m along the diagonal in both directions, rotated ±0.25 rad, and rotated by a quarter turn either way and by a half turn (the
   symmetry of a square room, which no local perturbation reaches). A restart that settles on a different
   pose (more than 5 cm or 0.02 rad from the main one) **and also fits** is an alias when it explains at least
   as much of the scan as the main solution less a margin of 5 per cent of the points. The margin is the
   difference between "another good fit exists" and "another equally good fit exists": a first version that
   refused on any alternative fit refused a room whose symmetry was broken only by one small box. The margin is `chosen`, and it is the number to look at first if the verifier turns
   out too permissive. This check is not in the reference chapter (which stops at fit and χ²).

An accepted candidate becomes a between factor with the registration's information (scaled by
`info_inflation`) and the Huber kernel of DS9. The pose graph returns the reason for every rejection
(`Reason`), because S5 is graded on counts of them and a count of "rejected" without the reason cannot be
audited. The tests (section 8) include, besides true loops that must be accepted, candidates designed to be
rejected for each reason: a pair from different parts of the room with no overlap, a 90-degree alias in an
otherwise symmetric room, a corridor, a true loop shifted outside the gate, and a registration that is
perturbed so ICP does not converge.

## 7. What waits for Phase 4

| Part of Phase 5 | Needs Phase 4's M1? | Why |
|---|---|---|
| T1 to T11 (the pure core, the verdict script, the example) | no | consumes scans, relative poses and covariances; no occupancy grid anywhere |
| T12 submap rendering, `/map`, `map -> odom`, the two nodes | **yes** | the rendered `/map` is Phase 4's grid, and its accuracy is the open question |
| the S8 claim | **yes**, and S5 | problem.md section 4 |
| T13 evidence runs of S1 to S6 on Rusty | no | synthetic, exact ground truth, no Phase 4 output |
| T13 on Webots | no for S1, S2, S4, S5, S6 (they read scans and ground truth); running Webots is shared with Phase 4's runs | |
| T14 `slam_toolbox` reference | no | informative |

Nothing in T1 to T11 reads a map. The one place a grid appears, the RBPF's own, is private to it (DS12).

## 8. What is built

Everything below was built on 2026-10-11 in the Windows checkout and built and tested in the WSL mirror
(`~/ros2-rust-robotics`, per `docs/BUILD.md`). Nothing here has been committed. Anything not listed does
not exist.

### 8.1 The crate `crates/robotics_slam`

| Module | What | Unit tests |
|---|---|---|
| `geom` | left and right SE(2) Jacobians (closed form, checked against their series), the relative-pose residual with exact Jacobians | 5 |
| `kernel` | L2, Huber, Cauchy, Geman-McClure: weight and cost | 6 |
| `adaptive` | the KISS-style threshold | 5 |
| `voxel` | voxel-hash point map, fixed hasher, cap, ring search, downsampling | 8 |
| `icp` | point-to-point and point-to-plane ICP, `settled`, line-feature information matrix, degeneracy flag, support fraction | 5 |
| `odometry` | the LiDAR odometry front end and the edge accumulator | 2 |
| `factor_graph` | pose and landmark variables; prior, between and range-bearing factors; GN and LM; IRLS kernels; dense and sparse (`faer`) solvers; marginal covariances | 11 |
| `pose_graph` | keyframes, odometry and loop edges, proposal, the six-reason verifier (section 6), `process_loops` | 0 (integration only) |
| `ndt` | NDT with the standard `d1, d2`, analytic gradient and Gauss-Newton Hessian | 9 |
| `ekf_slam` | EKF-SLAM with provisional landmarks | 10 |
| `rbpf` | grid RBPF with a private copy-on-write chunk grid | 15 |

That is **76 unit tests** and **39 integration tests**, 115 in all: `front_end_it` 6 (ICP on Rusty scans
with exact truth, degeneracy of a corridor, the front end over two laps, determinism), `pose_graph_it` 13
(two laps with an injected yaw error: verified loops reduce the trajectory error and every accepted
constraint is correct, the graph without loops is the odometry chain, and the designed-to-be-rejected
candidates below), `ndt_it` 10, `ekf_slam_it` 5, `rbpf_it` 5.

**Candidates designed to be rejected, one test each, each asserted to be rejected for its own reason:**
views that do not overlap (two rooms of different size); a corridor, however well it fits
(`Degenerate`); a true loop displaced beyond the graph's covariance (`GateExceeded`); the 90-degree alias
of an empty square room (`Ambiguous`); a registration that cannot settle (`NotConverged`); a scene changed
so much that the fit floor refuses it (`PoorFit`); and a rejected verification passed to `add_loop`
(an error). Beside them: a true loop is accepted with a measurement within 2 cm; a partly changed scene
whose remainder matches is accepted; and the same alias is accepted once a box breaks the room's symmetry.
The tests each give a *model-level* result on seeded Rusty scans; none is a claim S1 to S8.

**Gates, in the mirror:** `cargo fmt -p robotics_slam --check` clean; `cargo clippy -p robotics_slam
--all-targets -- -D warnings` clean; `cargo test -p robotics_slam` 115 passed, 0 failed. Not run: the
workspace-wide gate `scripts/gates.sh` (it includes a Phase 3 `docs` step that already failed before this
work), and the colcon build, which this crate does not enter.

**Dependencies.** `robotics_core`, `robotics_models`, `robotics_filters`, `nalgebra`, `faer`
(0.24, `default-features = false`, features `std` and `sparse-linalg`: no `rayon`, no `rand`), `rand`,
`thiserror`; `robotics_sim` as a dev-dependency. `cargo tree -p robotics_slam` contains neither
`robotics_mapping` nor `rayon` (checked, 0 matches each). The workspace `Cargo.toml` gains the member, the
default member, `faer`, and a dev-profile override that optimizes only this package (a debug build of
scan matching is about fifty times slower than an optimized one and made the integration tests take
minutes); `Cargo.lock` gains `faer` and its dependencies.

### 8.2 Scripts and the example

* `scripts/phase5_verdict.py`: every tolerance, each with its derivation, and the metrics-file schema;
  written before the example was ever run. `scripts/phase5_verdict_selftest.py` grades fabricated fixtures
  (written to a temporary directory and thrown away, never under `evidence/`) and checks that each claim fails
  for its own reason and that malformed files are refused: all its checks pass.
* `crates/robotics_slam/examples/slam_synthetic_run.rs`: the seeded Rusty harness that writes
  `slam-metrics.yaml`. **No evidence run has been made.** The example was run once into a scratch directory
  to check that its file satisfies the schema and that the grader accepts it; that output was discarded, it is
  not evidence, and no tolerance was changed after it.

### 8.3 What the build found

Each of these was found by a test, and each changed the design above. They are model-level findings.

1. The target submap of a loop candidate included the candidate's own keyframe when the two were adjacent,
   so a scan was matched to itself with zero residual (section 6).
2. ICP at a small threshold can reach the answer and then jitter for ever, losing a registration in a front
   end that had tracked 292 other scans: `settled` (DS4).
3. A corridor was not flagged degenerate while sparse-wall correspondences counted as information (DS7).
4. An RMS-based fit measure was polluted by the points a changed scene adds; fit is now explained points
   (section 6, check 1). A pass-or-fail alias test refused a room with one small asymmetry; it is now a comparison of
   support with a margin (check 4).
5. The label of a false loop closure by distance between keyframes was wrong: a constraint 1.7 m long with an
   accurate measurement is harmless. The label is now the error of the accepted measurement (benchmark.md 2).
6. The Magnusson NDT constants make `d1` negative, so the score to minimize is the negated one (DS8).
7. NDT's basin of convergence depends on the scene and the direction; `ndt_it.rs` asserts only the ranges that
   were measured to hold, and the module's own documentation says `converged` certifies a local optimum and not
   the true pose.
8. EKF-SLAM is statistically consistent over a short horizon on a four-landmark loop (25 seeds, 100 steps,
   asserted) and over-confident on a long loop with a noisy heading (20 seeds, 200 steps, asserted): the
   behaviour the reference describes.
9. The grid RBPF beats dead reckoning in its test only when the odometry carries a large systematic yaw
   error (asserted for that setting on 8 seeds). With half that error the author's exploratory runs lost on two
   of eight seeds; those runs are not saved as a test. It has no proposal refinement (DS12), and its
   copy-on-write grid shares little inside a small enclosed room, because every scan touches nearly every chunk
   it sees.

### 8.4 Not built

`scan_matcher_node`, `pose_graph_node`, a ROS package, a launch file, anything that publishes `/map` or
`map -> odom` for SLAM (DS2); submap rendering (DS17); every evidence run (T13), including S1a on Webots data,
which also needs the motion-gated ground-truth join; the S7 harness for the two back ends; landmark
FastSLAM (Chapter 17's first half); the `Slam2d` pipeline wrapper of the reference's design document; a
place-recognition descriptor; the `slam_toolbox` reference (T14); `benchmarks/phase-5-slam.csv`.

## 9. Open questions

1. **Evaluation routes.** S4 needs one development and three evaluation routes with revisits.
   `route_long.yaml` visits the pentagram twice and so has real loops. Three further routes
   (`route_eval_cross.yaml`, `route_eval_wall_lobes.yaml`, `route_eval_diamond.yaml`) were written for Phase 4's
   M1 by the other prompt; they were designed to cover the room for mapping, and whether they revisit places
   enough to produce loop candidates has not been checked. Whether Phase 5 reuses them or writes its own is
   open and is not decided here.
2. **A place-recognition descriptor** (Scan Context and relatives) for loops the drift has hidden from the
   covariance gate. Not built, not planned; the limit is stated in problem.md 4.1.
3. **Whether the sparse solver is worth it at this size.** A route of a few hundred keyframes is a
   ~1000-variable system, which a dense Cholesky solves in milliseconds. The sparse path is required by the
   ROADMAP and is tested against the dense one, and S6 will say whether it earns its dependency.
4. **The RBPF's duplicate grid** (DS12) once Phase 4 closes.
