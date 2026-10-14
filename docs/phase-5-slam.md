# Phase 5: SLAM

Drop the assumption that the map was given: the robot has to build it and localize in it at
the same time, which means scan matching, a pose graph, and loop closures that are verified
before they are believed.

> **Status (2026-10-11): the design is written and the pure core, `robotics_slam`, is built
> (116 tests); the first evidence runs of S1a, S1b, S2, S4, S5, S6 and S7 are made (`evidence/phase-5-slam/NOTE-first-evidence-runs.md`).
> `scan_matcher_node` and `pose_graph_node` (package `robotics_slam_ros`) are also built, without `/map`,
> TF, or `map -> odom`, and node-replay-verified against a release build: 66 of 66 checks.** Section 8 of
> [implementation](phase-5-slam/implementation.md) lists exactly what is built, including the nodes (8.6).
> Phase 5 was started while Phase 4 was still open, and only the part that does not depend on Phase 4's
> mapper. Phase 4 closed on 2026-10-11 with M1 met, so submap rendering into `/map` and ownership of
> `map -> odom` are no longer blocked; neither is built. A live
> Webots run with a route that turns, feeding the nodes through their own `/scan` timing, has not been
> made; the recordings replayed so far have no turning scans (implementation.md 8.6).

## Where the detail is

| Document | Content |
|---|---|
| [problem](phase-5-slam/problem.md) | the situation, why SLAM is hard in four ways, the problem statement, **what kind of claim each of S1 to S8 is, and that "SLAM produces a usable map" is not claimable**, the meaning of `loop_closure: unsupported`, and why part of this could start before Phase 4 closed |
| [implementation](phase-5-slam/implementation.md) | tasks T1 to T14, the planned interface, decisions DS1 to DS18 (including the TSDF and quadtree decision), the loop-closure detector and verifier, what waits for Phase 4, and what is built |
| [benchmark](phase-5-slam/benchmark.md) | the metrics, the metrics file, the baselines and what each is and is not compared with |

Scope and exit criteria in the roadmap: [ROADMAP.md, Phase 5](ROADMAP.md). Theory and ROS graph
documents are written when there is something built to describe, as in Phases 3 and 4.

## The design in one page

**The problem.** The poses are now unknowns: estimate `p(x_1:t, m | z_1:t, u_1:t)` from scans and
odometry alone. Scan matching gives relative-pose constraints, a factor graph holds them, and a
verified loop closure is the only thing that removes accumulated error.

**The claims, and which are rates.** S1a scan-matching accuracy is a distribution over the
registrations of a run; S1b the basin of convergence is a seeded trial set, model level only; S2 the
honesty of the information matrix is a statistic with no pass or fail; S3 the optimizer is tests; S4
trajectory accuracy is one development run and three evaluation runs, graded against ground truth and
never against Webots' calibrated odometry; S5 loop closure is counts, with **zero accepted false loops
pre-registered** and recall reported but not required; S6 determinism and cost; S7 the two back ends
(non-blocking); **S8 a usable map is not claimable** until Phase 4's M1 (met 2026-10-11), a measured S5 and the
rendering node all exist.

**Loop closure is verified, never faked.** A candidate must pass four checks: fit, not degenerate,
unambiguous under perturbed restarts (the square room's 90-degree alias is why), and a χ² test on the
graph's own marginal covariance. The χ² test cannot reject an alias that sits inside the graph's
uncertainty, and the documents say so. The library has a detector and a verifier; they
were measured on synthetic runs only, and weakly against false candidates; a node reports `unsupported` until the project owner decides that is enough (problem.md section 5).

**TSDF and quadtree: no.** Scan matching is against points, the room is 14 400 cells, the ESDF
already exists, and a fused map of any kind is rebuilt by replay after a loop closure. Revisit triggers
are listed in DS13.

**Packages.** Crate `crates/robotics_slam`; ROS package `ros2_ws/src/robotics_slam_ros`
(`scan_matcher_node`, `pose_graph_node`, and `scan_replayer` for replay tests); launch file
`robotics_bringup/launch/phase5_slam.launch.py`.
