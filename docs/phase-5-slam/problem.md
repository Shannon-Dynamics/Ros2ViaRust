# Phase 5, problem: where am I, and what does this room look like, at the same time?

> **Status (2026-10-11): the plan below is the design of the pure core, written before any
> code, and it is being built in the same pass.** What exists is listed in section 8 of
> [implementation.md](implementation.md), and nothing else exists. There are no numbers in this
> file: results, when there are any, are verdict files under `evidence/phase-5-slam/`. This
> phase was started while Phase 4 is still open, on purpose and for a stated reason (section 6).

## 1. The situation

Phase 3 gave the robot a position in a map it was handed. Phase 4 gave it a map when it was
handed a position. Both phases lean on something the robot does not have in a new room:

* Phase 3 reads `landmarks.yaml`, a list somebody measured.
* Phase 4 reads `map -> base_link`, which in simulation comes from the simulator's ground truth
  (`ground_truth_tf_adapter`). Its claim M1 is explicitly "the mapper is implemented correctly",
  because with the pose given the pose-induced blur is invisible by construction.

A robot dropped in a room it has never seen has the 5 Hz, 360-beam, 3.5 m LDS-01 scans and
wheel odometry, and nothing else. It has to produce both the trajectory and the map from those,
and each is an input to the other: the map is only as good as the poses it was written at, and
the poses can only be checked against the map. Phase 5 is that loop.

## 2. Why this is hard, in the four ways this phase meets it

1. **Odometry drifts and the drift is not noise.** Errors in heading turn into errors in
   position that grow with distance, and they are correlated along the whole path. A filter
   that forgets old poses (EKF-SLAM, Chapter 14) can never repair them; a smoother that keeps
   them (the pose graph, Chapters 15 and 16) can. *In this repository's Webots world there is
   almost no odometry drift to repair* (`docs/phase-3-localization/theory.md` section 6), which
   is a finding about the simulator and shapes the evaluation, see section 4.
2. **A scan is only comparable with another scan through a pose, and the pose is what is being
   asked.** Registering two scans (ICP, NDT) estimates the pose between them from the scans
   alone, at the cost of local minima: a rotation of a square room by 90 degrees looks like the
   same room, and a straight corridor does not constrain motion along it.
3. **A loop closure is the only evidence that removes accumulated error, and a wrong one is
   worse than none.** A missed loop costs drift. A false loop closure bends the whole map into a
   shape that is self-consistent and wrong, and nothing downstream can tell. This asymmetry is
   the design constraint of the loop-closure part: it is verified before it is believed, and
   the criterion for believing it is stricter than the criterion for finding it.
4. **The room is a hard test for the verifier and an easy one for the front end.** It is
   a 6 by 6 m square with four pillars, so it has a four-fold near-symmetry
   (`scripts/phase3_map_symmetry.py` measures how near), and every wall is a long straight
   line. Scan-to-scan matching works in it (there are always two walls in view); loop
   verification by geometry alone is exactly what a symmetric room fools.

## 3. Problem statement

Let `x_1:t` be the poses of `base_link` (keyframes), `z_1:t` the scans, `u_1:t` the odometry
increments and `m` the map. The full SLAM posterior is

```text
p(x_1:t, m | z_1:t, u_1:t)
```

and this phase builds three ways to approximate it, kept apart on purpose because they make
different claims:

* **Graph SLAM (the main line).** Scan matching turns scans into relative-pose constraints
  (with an information matrix), a factor graph over keyframe poses holds them, and a
  Gauss-Newton or Levenberg-Marquardt optimizer solves the maximum-a-posteriori trajectory. A
  loop candidate is a pair of keyframes that may have seen the same place; ICP verifies it and a
  chi-squared test on the graph's own covariance decides whether to believe it. The map is
  *rendered afterwards* from the keyframe scans at the optimized poses (section 4, what is
  deferred).
* **EKF-SLAM (Chapter 14), a back end for comparison and teaching.** Robot and landmarks in one
  Gaussian, with landmarks extracted from the scan (the four pillars). It cannot revise an old
  decision, which is the point of having it next to the graph.
* **Grid RBPF (Chapter 17), a second back end.** A particle per trajectory hypothesis, each
  with its own occupancy grid.

The library is pure Rust with no ROS; the two nodes that would drive it
(`scan_matcher_node`, `pose_graph_node`) are not part of this pass (section 6).

## 4. What counts as solved, and what kind of claim that is

This section is a decision, made here so it is not made implicitly later. It follows the
shape of Phase 4's section 4 (claims are not all rates, and each says what it is graded on) and
it needs the project owner's confirmation before work goes past the pure library.

The claims are named **S1 to S8** and not M-numbers. Phase 4 already owns M1 to M5, and "blocked
on Phase 4 M1" is a phrase this phase has to be able to write without ambiguity.

| Claim | Kind | Evidence | What is reported |
|---|---|---|---|
| **S1a** scan matching is accurate: the estimated relative pose of a registered scan pair against the ground-truth relative pose | distribution over the registrations of a run, on representative runs | synthetic Rusty runs with exact ground truth and exact timing, and one recorded Webots run on the registrations whose ground truth is trustworthy (section 4.2) | translation and rotation error per registration: median, 95th percentile, maximum; and how many registrations were graded out of how many made. No success rate. |
| **S1b** ICP finds the right answer from a poor start | **a rate**, on a seeded trial set, model level only | scan pairs from Rusty with random initial offsets drawn from stated ranges, the event "ends within a stated error of truth" defined before the trials | successes over trials per offset range, with the Wilson interval, as Phase 3 reports P2. Not extrapolated to Webots and not compared with a Webots number. |
| **S2** the ICP information matrix tells the truth about the error | statistic over the registrations of S1a runs | normalized estimation error squared (NEES) of the relative pose under the reported information, against the chi-squared bounds | CONSISTENT, OVERCONFIDENT or CONSERVATIVE, with the mean. **No pass or fail:** the reported information is known to be overconfident (the beams are not independent), and a gate on it would force a fudge factor to be chosen after the result. The `info_inflation` that the graph uses is set from the development run and frozen. |
| **S3** the optimizer is correct | deterministic tests | `cargo test` | Jacobians equal central differences; dense and sparse solves agree; a known optimum is recovered; a robust kernel down-weights an injected outlier; the same graph gives the same answer twice. |
| **S4** the trajectory is accurate: keyframe poses against ground truth | representative runs | one development run, then three evaluation runs on routes that played no part in choosing the parameters (route_long.yaml is the development route) | per run: absolute trajectory error (RMS, maximum) and drift per metre travelled, for the LiDAR odometry alone and for the optimized graph, each next to the other. No rate, no interval. **Without accepted loop closures the optimized chain equals the odometry chain**, so the comparison is only meaningful once S5 holds; until then S4 reports the front end's drift and nothing else. |
| **S5** loop closure is safe, and how much it finds | counts over the candidate set, with a stated cost for a false accept | every candidate the detector proposes on the S4 runs, labelled by ground truth (the adversarial candidates are in the unit and integration tests) | accepted-and-wrong (the **safety count**: an accepted constraint whose error against the ground-truth relative pose exceeds the limits of benchmark.md section 2, pre-registered to be zero), accepted-and-correct, and the rejections by reason, which include true loops that were missed (their cost is drift and nothing else). Candidate-level precision with a Wilson interval; the rule of three for what "zero false accepts in n" supports. Recall is reported and **not** a pass criterion. |
| **S6** the pipeline is deterministic and cheap enough | deterministic test, and a distribution | bit-identical replay of the whole pipeline on a recorded scan stream; per-scan registration time and per-optimization time over a run | the replay is pass or fail; latency as median, 95th and 99th percentile, with the S1a accuracy of the same run next to it (benchmarks rule 6). |
| **S7** the two back ends work on the problem they were designed for | informative, **non-blocking** | EKF-SLAM and grid RBPF on the same synthetic run | EKF-SLAM: NEES of the pose over time; both: trajectory error against ground truth. No ranking of back ends: the odometry noise of Rusty is a configurable property of a simulator, so who wins is a statement about that setting. |
| **S8** *(not claimable)* SLAM produces a usable map | none | none | **Blocked on three things, none of which this phase can unblock alone:** Phase 4's M1 (the grid it renders into does not meet its own pre-registered tolerances yet), a real loop closure whose S5 safety count has been measured, and the submap-rendering node, which is not built. It stays `unsupported`, see section 5. |

### 4.1 What is explicitly not claimed

* No "SLAM succeeds N per cent of the time". The only rates are S1b (a model-level
  trial set) and the candidate-level precision of S5, each with its denominator and interval.
* No claim that the graph beats wheel odometry in Webots. The Webots odometry is calibrated
  to this world by upstream (`theory.md` section 6) and is very accurate, so a comparison with
  it measures nothing about SLAM. The graph is graded against **ground truth**, and the drift
  that loop closure repairs is the LiDAR odometry's own, which is real: scan matching
  accumulates error from the lidar noise and from the geometry, with no injected noise.
  (Phase 3 found and deleted a synthetic odometry noise injector that was wrong, and it is
  not reintroduced.)
* No claim that a loop closure that passed the chi-squared gate is *true*. The gate tests
  whether the ICP result is consistent with where the graph already believes the robot is; a
  false candidate that sits inside the graph's uncertainty passes it however wrong it is (an
  alias of a symmetric room does exactly this). That is why the verifier has three other
  checks (implementation section 6) and why S5 counts false accepts.
* No claim about places the detector never proposes. The detector searches around the
  *estimated* trajectory within the graph's own covariance; a drift larger than that
  covariance hides a loop from it. A place-recognition descriptor (Scan Context and relatives)
  would find it and is not built.
* No claim from the book's worked numbers. The reference chapters report their own
  experiments; none of them is a result of this repository.

### 4.2 Ground truth, and the time it is taken at

The Webots lidar stamps a scan up to a scan period after it was measured (Phase 4,
[implementation section 5.2 item 2](../phase-4-mapping/implementation.md), which also has
the measurement of the lattice). A scan matcher
works on scan content, so the lag does not touch *estimating* a relative pose, and it must
not be "corrected" by scaling velocity with stamp differences (the measurements lie on a
regular 0.2 s lattice and the stamps do not). It does touch *grading*: the ground-truth pose
at a scan's stamp is the pose of up to a period earlier or later than the scan, which at the
route's turning rate is more than the error the scan matcher makes.

So on Webots data a registration is graded only when both its scans pass the same motion gate
as Phase 4's mapper (D13, `robotics_mapping::MotionGate`), and the number graded out of the
number made is reported with every distribution. On Rusty the stamps are exact and every
registration is graded. A claim that a Webots number was measured on a subset is a claim about
that subset.

**World.** The room of Phase 3 and 4: four pillars and four walls, no clutter, no corridors.
It is a benign front-end world and a hostile verification world (section 2, item 4). A second,
more complex world is the backlog item *M1-extended* of the ROADMAP, which would also make S5
a stronger statement; it is not part of this phase.

## 5. `loop_closure: unsupported`, and what it means after this pass

The ROADMAP's rule is that no loop closure is faked: until a detector and a verifier are
implemented, the node reports `loop_closure: unsupported`. After this pass the library has a
detector and a verifier, with tests, **and they have not been evaluated**: S5 has no run.
So three states have to be kept apart, and the words are fixed here:

| State | Meaning | Today |
|---|---|---|
| `unsupported` | no detector or no verifier exists | true of every node, since no node exists |
| implemented, unevaluated | the library has both and unit tests that include candidates designed to be rejected; S5 has no run | **the library, after this pass** |
| enabled | the S5 safety count was measured on the evaluation runs and is zero | nothing |

A node may switch loop closure on only in the third state, and until then it reports
`unsupported` whatever the library contains. There is no placeholder heuristic: a loop edge
enters the graph only through the verifier of implementation section 6, never through a
distance test alone, and not through a flag that bypasses it.

## 6. Why part of this could start before Phase 4 closes, and what could not

Phase 4 is not closed: its claim M1 is not met. The state, with the runs and numbers it comes
from, is the Phase 4 status line of [../ROADMAP.md](../ROADMAP.md); it is not repeated here because
another prompt is working on it and it changes. (When this phase was started, six of the seven
pre-registered checks passed on the development run and the three evaluation routes were not
written; they have been run since, and M1 is still not met.) Phase 4 itself was started while Phase 3 was open, and the ROADMAP records that
instead of hiding it; this is the same arrangement, and the reason it is safe is a dependency
argument that can be checked.

| Part of Phase 5 | Consumes | Depends on Phase 4's mapper? |
|---|---|---|
| ICP, NDT, the information matrix, the adaptive threshold | raw scans and a pose guess | no |
| factor-graph optimizer, pose graph, loop detection and verification | relative poses, covariances, raw scans | no |
| EKF-SLAM | range-bearing detections | no |
| grid RBPF | raw scans, odometry | **would, if it used `robotics_mapping::OccupancyGrid`; it does not.** It carries its own copy-on-write grid (decision DS12), so no RBPF number can be a Phase 4 number in disguise |
| submap rendering into `/map`, the `map -> odom` owner | the occupancy grid of Phase 4 | **yes. Not built.** Any map error in a SLAM-rendered `/map` could be the mapper's or the trajectory's, and with M1 open there would be no way to tell which |
| `scan_matcher_node`, `pose_graph_node` | everything above, plus `/map` publication | yes through `/map`. Not built. |

`cargo tree -p robotics_slam` has no `robotics_mapping` in it, and that is how the first row
and the third are kept honest; section 8 of the implementation file records the check.

The two nodes are deferred pending Phase 4's M1. `scan_matcher_node` alone would not publish
`/map` and could be built earlier, but the two nodes were specified as a pair and nothing
consumes LiDAR odometry without the graph, so the ROADMAP entry says both are deferred.

## 7. Why this is the next step

Everything after Phase 5 reads a map, and a map that has to be produced from scans needs
poses it can trust. SLAM is Phase 4's mapper with the poses turned into unknowns, so Phase 5
inherits every defect Phase 4 ships with; that is the argument for finishing Phase 4 first and
the reason the part that does not inherit anything is the part started now.

Continue with [implementation.md](implementation.md), then [benchmark.md](benchmark.md).
