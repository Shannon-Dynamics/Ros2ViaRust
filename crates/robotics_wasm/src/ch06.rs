//! Chapter 06 bindings: SLAM, scan matching, a pose graph and a loop closure that is verified
//! before it is believed (see web/design/Chapter-06.md).
//!
//! Two wrappers over `robotics_slam`, each calling what the robot's `scan_matcher_node` and
//! `pose_graph_node` call and flattening what comes back:
//!
//! * [`Ch06Replay`]: a recorded scan stream (the CSV format of Phase 4's `integrated-scans.csv`:
//!   one scan per line, the sensor's ground-truth pose in columns 2 to 4) fed in order through
//!   `LidarOdometry` (point-to-plane ICP against a local map, adaptive threshold), cut into keyframes
//!   by `PoseGraph::wants_keyframe`, with `PoseGraph::process_loops` proposing, verifying and
//!   accepting loop closures and the factor graph optimizing after each accepted one. The wiring is
//!   the one of `crates/robotics_slam/examples/common/mod.rs::pipeline`, which produced the evidence
//!   files `evidence/phase-5-slam/v2-synthetic-4*/slam-metrics.yaml`; a test below asserts that this
//!   replay writes the same trajectory errors, to twelve digits. [`Ch06Replay::icp_trace`] registers
//!   two scans of the stream from a chosen starting pose, one iteration limit at a time, which is how
//!   the page shows ICP converging (or not).
//! * [`ch06_verify`]: one loop candidate built from a seeded Rusty room, handed to
//!   `PoseGraph::verify`. It is the construction of `crates/robotics_slam/tests/pose_graph_it.rs`
//!   (`pair`), with the room, the rotation between the two views and the graph's belief as inputs.
//!
//! What is *not* a crate algorithm, and says so where it lives: the CSV parser (it mirrors
//! `examples/common/mod.rs::read_frames`, an example and not library code), the rooms of
//! [`ch06_verify`] (copies of `tests/common/mod.rs`), the trajectory error (translation distance
//! to the ground truth, as `trajectory_block` computes it), and the labelling of an accepted loop
//! constraint as wrong (limits 0.05 m and 0.03 rad, the ones `phase5_verdict.py` insists on).
//!
//! **Randomness.** [`Ch06Replay`] draws none: the recording is data. [`ch06_verify`] draws the lidar
//! noise of two scans from `seeded_rng`, so on WebAssembly a seed names a different realization than
//! natively (design/TOC.md section 5); the verdicts it is used for do not depend on the realization
//! (a native test and a check on the compiled module assert the same six verdicts).
//!
//! Flat-array layouts are documented on each function.

use robotics_core::nalgebra::{Matrix3, Vector2, Vector3};
use robotics_core::{LaserScan, Manifold, Se2, Timestamp};
use robotics_sim::{Lidar, LidarConfig, World};
use robotics_slam::icp::{register, scan_points, IcpConfig, IcpMetric};
use robotics_slam::odometry::{EdgeAccumulator, LidarOdometry, OdometryConfig, TrackingStatus};
use robotics_slam::pose_graph::{
    LoopCandidate, LoopOutcome, PoseGraph, PoseGraphConfig, Reason, Verification,
};
use robotics_slam::voxel::{downsample, VoxelMap};
use wasm_bindgen::prelude::*;

fn err<E: std::fmt::Display>(e: E) -> String {
    e.to_string()
}

/// A loop constraint is wrong when its error against the ground-truth relative pose exceeds
/// these: the limits `scripts/phase5_verdict.py` pins (claim S5).
pub const WRONG_M: f64 = 0.05;
/// See [`WRONG_M`].
pub const WRONG_RAD: f64 = 0.03;
/// Two keyframes closer than this by ground truth are a true loop (`examples/common/mod.rs`).
pub const TRUE_LOOP_M: f64 = 1.5;

/// The status code of a verification, the order of `phase5_verdict.py`'s `REASONS` after
/// `0 = accepted`: 1 no overlap, 2 not converged, 3 poor fit, 4 degenerate, 5 gate exceeded
/// (the chi-squared test), 6 ambiguous.
fn status_code(v: &Verification) -> f64 {
    match v {
        Verification::Accepted { .. } => 0.0,
        Verification::Rejected { reason, .. } => match reason {
            Reason::NoOverlap => 1.0,
            Reason::NotConverged => 2.0,
            Reason::PoorFit => 3.0,
            Reason::Degenerate => 4.0,
            Reason::GateExceeded { .. } => 5.0,
            Reason::Ambiguous => 6.0,
        },
    }
}

fn info(sx: f64, sth: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(
        1.0 / (sx * sx),
        1.0 / (sx * sx),
        1.0 / (sth * sth),
    ))
}

fn points_of(scan: &LaserScan) -> Vec<Vector2<f64>> {
    downsample(&scan_points(scan), 0.05).expect("a positive voxel size")
}

fn flat_xy(points: &[Vector2<f64>]) -> Vec<f64> {
    points.iter().flat_map(|p| [p.x, p.y]).collect()
}

fn flat_poses(poses: &[Se2]) -> Vec<f64> {
    poses
        .iter()
        .flat_map(|p| [p.x(), p.y(), p.theta()])
        .collect()
}

// ---------------------------------------------------------------------------------------
// The recording
// ---------------------------------------------------------------------------------------

/// One recorded scan with the ground-truth pose of its sensor.
#[derive(Clone)]
struct Frame {
    gt: Se2,
    scan: LaserScan,
}

fn parse_frames(csv: &str) -> Result<Vec<Frame>, String> {
    let mut out = Vec::new();
    for (ln, line) in csv.lines().enumerate() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 9 {
            return Err(format!("line {}: too few fields", ln + 1));
        }
        let num = |i: usize| -> Result<f64, String> {
            f[i].trim()
                .parse::<f64>()
                .map_err(|e| format!("line {}: field {i}: {e}", ln + 1))
        };
        let n = num(8)? as usize;
        if f.len() != 9 + n {
            return Err(format!(
                "line {}: {} ranges, header says {n}",
                ln + 1,
                f.len() - 9
            ));
        }
        let mut ranges = Vec::with_capacity(n);
        for r in &f[9..] {
            ranges.push(
                r.trim()
                    .parse::<f32>()
                    .map_err(|e| format!("line {}: {e}", ln + 1))?,
            );
        }
        out.push(Frame {
            gt: Se2::new(num(1)?, num(2)?, num(3)?),
            scan: LaserScan {
                stamp: Timestamp::from_secs_f64(num(0)?),
                angle_min: num(4)?,
                angle_increment: num(5)?,
                range_min: num(6)?,
                range_max: num(7)?,
                ranges,
                ..LaserScan::default()
            },
        });
    }
    if out.len() < 2 {
        return Err("a recording needs at least two scans".into());
    }
    Ok(out)
}

/// The pipeline of `examples/common/mod.rs::pipeline`, one scan at a time, over a recording.
///
/// `yaw_bias_per_keyframe` is the systematic error the evidence runs inject into the odometry
/// edges the graph is handed (`--yaw-bias`; zero leaves only the scan matcher's own drift). With a
/// non-zero bias the edge information is the fixed 3 cm and 0.01 rad the harness hands it, for the
/// reason `benchmark.md` section 6 gives (the ICP information knows nothing of an injection).
#[wasm_bindgen]
pub struct Ch06Replay {
    frames: Vec<Frame>,
    next: usize,
    odo: LidarOdometry,
    graph: PoseGraph,
    acc: EdgeAccumulator,
    yaw_bias: f64,
    truth: Vec<Se2>,
    dead: Vec<Se2>,
    outcomes: Vec<LoopOutcome>,
    lost: usize,
}

#[wasm_bindgen]
impl Ch06Replay {
    /// Parse a recording (the text of a scan CSV) and start with an empty graph.
    #[wasm_bindgen(constructor)]
    pub fn new(csv: &str, yaw_bias_per_keyframe: f64) -> Result<Ch06Replay, String> {
        if !yaw_bias_per_keyframe.is_finite() {
            return Err("yaw bias is not finite".into());
        }
        Ok(Ch06Replay {
            frames: parse_frames(csv)?,
            next: 0,
            odo: LidarOdometry::new(OdometryConfig::default()).map_err(err)?,
            graph: PoseGraph::new(PoseGraphConfig::default()).map_err(err)?,
            acc: EdgeAccumulator::new(),
            yaw_bias: yaw_bias_per_keyframe,
            truth: Vec::new(),
            dead: Vec::new(),
            outcomes: Vec::new(),
            lost: 0,
        })
    }

    /// Scans in the recording.
    pub fn total(&self) -> u32 {
        self.frames.len() as u32
    }

    /// Scans processed so far.
    pub fn processed(&self) -> u32 {
        self.next as u32
    }

    /// Keyframes in the graph.
    pub fn keyframes(&self) -> u32 {
        self.graph.keyframes().len() as u32
    }

    /// Accepted loop edges in the graph.
    pub fn loop_edges(&self) -> u32 {
        self.graph.loop_edge_count() as u32
    }

    /// Scans for which the front end did not believe its registration (the pose advanced by the
    /// prediction).
    pub fn lost_registrations(&self) -> u32 {
        self.lost as u32
    }

    /// Process up to `n` more scans; returns how many were processed in total.
    pub fn step(&mut self, n: u32) -> Result<u32, String> {
        for _ in 0..n {
            if self.next >= self.frames.len() {
                break;
            }
            let k = self.next;
            let fr = self.frames[k].clone();
            let step = self.odo.step(&fr.scan).map_err(err)?;
            self.next += 1;
            if step.status == TrackingStatus::Predicted {
                self.lost += 1;
            }
            if k == 0 {
                self.graph
                    .add_keyframe(points_of(&fr.scan), &Se2::IDENTITY, &info(0.03, 0.01))
                    .map_err(err)?;
                self.truth.push(self.frames[0].gt.between(&fr.gt));
                self.dead.push(Se2::IDENTITY);
                continue;
            }
            self.acc
                .push(&step.delta, &step.information)
                .map_err(err)?;
            if self.graph.wants_keyframe(&self.acc.delta()) {
                let (delta, chain_info) = self.acc.take().map_err(err)?;
                let delta = delta.boxplus(&Vector3::new(0.0, 0.0, self.yaw_bias));
                let chain_info = if self.yaw_bias != 0.0 {
                    info(0.03, 0.01)
                } else {
                    chain_info
                };
                let id = self
                    .graph
                    .add_keyframe(points_of(&fr.scan), &delta, &chain_info)
                    .map_err(err)?;
                self.truth.push(self.frames[0].gt.between(&fr.gt));
                let last = *self.dead.last().expect("the first keyframe exists");
                self.dead.push(last.compose(&delta));
                let out = self.graph.process_loops(id).map_err(err)?;
                self.outcomes.extend(out);
            }
        }
        Ok(self.next as u32)
    }

    /// The optimized keyframe poses, `x, y, theta` per keyframe, in the frame of the first
    /// keyframe's sensor.
    pub fn graph_poses(&self) -> Vec<f64> {
        flat_poses(self.graph.poses())
    }

    /// The odometry chain: the keyframe poses with no loop closure, `x, y, theta` per keyframe.
    pub fn chain_poses(&self) -> Vec<f64> {
        flat_poses(&self.dead)
    }

    /// The recorded ground-truth sensor pose of each keyframe, in the same frame.
    pub fn truth_poses(&self) -> Vec<f64> {
        flat_poses(&self.truth)
    }

    /// Path length of the ground truth over the keyframes so far (m).
    pub fn path_length(&self) -> f64 {
        self.truth.windows(2).map(|w| w[0].distance_to(&w[1])).sum()
    }

    /// Trajectory error against the ground truth over the keyframes so far, for the graph
    /// (`which` 0) or the chain (`which` 1): `[ate_rms_m, ate_max_m, final_error_m, keyframes]`.
    pub fn trajectory_errors(&self, which: u32) -> Vec<f64> {
        let est: &[Se2] = if which == 0 {
            self.graph.poses()
        } else {
            &self.dead
        };
        let errs: Vec<f64> = est
            .iter()
            .zip(&self.truth)
            .map(|(a, b)| a.distance_to(b))
            .collect();
        if errs.is_empty() {
            return vec![f64::NAN, f64::NAN, f64::NAN, 0.0];
        }
        let rms = (errs.iter().map(|e| e * e).sum::<f64>() / errs.len() as f64).sqrt();
        let max = errs.iter().cloned().fold(0.0, f64::max);
        vec![rms, max, *errs.last().expect("non-empty"), errs.len() as f64]
    }

    /// Translation error of each keyframe against the ground truth, for the graph (`which` 0) or
    /// the chain (`which` 1), in metres, one number per keyframe.
    pub fn keyframe_errors(&self, which: u32) -> Vec<f64> {
        let est: &[Se2] = if which == 0 {
            self.graph.poses()
        } else {
            &self.dead
        };
        est.iter()
            .zip(&self.truth)
            .map(|(a, b)| a.distance_to(b))
            .collect()
    }

    /// Every loop candidate so far, eight numbers each: `from, to, status, chi2, true_loop,
    /// wrong, error_m, error_rad`. `status` is 0 for accepted and 1 to 6 for the reason of the
    /// rejection (see [`status_code`]); `chi2` is NaN unless the candidate reached the gate;
    /// `true_loop` is 1 when the two keyframes are closer than 1.5 m by ground truth; for an
    /// accepted candidate `error_m` and `error_rad` are the error of the measurement against the
    /// ground-truth relative pose and `wrong` is 1 when either exceeds its limit (claim S5's label),
    /// otherwise both are NaN and `wrong` is 0.
    pub fn candidates(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(8 * self.outcomes.len());
        for o in &self.outcomes {
            let c = &o.candidate;
            let gt = self.truth[c.from].between(&self.truth[c.to]);
            let true_loop = gt.translation().norm() < TRUE_LOOP_M;
            let (chi2, err_m, err_r, wrong) = match &o.verification {
                Verification::Accepted {
                    measurement, chi2, ..
                } => {
                    let e = gt.between(measurement).log();
                    let (m, r) = (e.xy().norm(), e[2].abs());
                    (*chi2, m, r, if m < WRONG_M && r < WRONG_RAD { 0.0 } else { 1.0 })
                }
                Verification::Rejected {
                    reason: Reason::GateExceeded { chi2 },
                    ..
                } => (*chi2, f64::NAN, f64::NAN, 0.0),
                Verification::Rejected { .. } => (f64::NAN, f64::NAN, f64::NAN, 0.0),
            };
            out.extend([
                c.from as f64,
                c.to as f64,
                status_code(&o.verification),
                chi2,
                if true_loop { 1.0 } else { 0.0 },
                wrong,
                err_m,
                err_r,
            ]);
        }
        out
    }

    /// Claim S5's counts over the candidates so far: `[candidates, accepted_correct,
    /// accepted_wrong, true_loops_proposed, true_loops_accepted, no_overlap, not_converged,
    /// poor_fit, degenerate, gate_exceeded, ambiguous]`.
    pub fn s5_counts(&self) -> Vec<f64> {
        let rows = self.candidates();
        let mut v = vec![0.0; 11];
        for r in rows.chunks(8) {
            v[0] += 1.0;
            let status = r[2] as usize;
            if r[4] == 1.0 {
                v[3] += 1.0;
            }
            if status == 0 {
                if r[5] == 1.0 {
                    v[2] += 1.0;
                } else {
                    v[1] += 1.0;
                }
                if r[4] == 1.0 {
                    v[4] += 1.0;
                }
            } else {
                v[4 + status] += 1.0;
            }
        }
        v
    }

    /// The keyframes' scan points in the frame of the first keyframe, placed at the optimized
    /// poses (`graph_frame` true) or at the odometry chain's poses, as `x, y` pairs, every
    /// `stride`-th point of every keyframe.
    pub fn map_points(&self, graph_frame: bool, stride: u32) -> Vec<f64> {
        let poses: &[Se2] = if graph_frame {
            self.graph.poses()
        } else {
            &self.dead
        };
        let stride = stride.max(1) as usize;
        let mut out = Vec::new();
        for (kf, pose) in self.graph.keyframes().iter().zip(poses) {
            for p in kf.points.iter().step_by(stride) {
                let q = pose.act(p);
                out.extend([q.x, q.y]);
            }
        }
        out
    }

    /// Register scan `j` of the recording against scan `i`, starting from the ground-truth
    /// relative pose displaced by `(dx, dy, dtheta)` (in scan `i`'s frame), with the iteration
    /// limit set to `0, 1, ..., max_iterations` in turn. Deterministic, so the run with limit `k`
    /// is the first `k` iterations of the run with a larger limit (and once a run has stopped by
    /// itself the later rows are copies of its last). Rows of eight numbers:
    /// `iteration limit, x, y, theta, rmse, correspondences, settled, converged`; row 0 is the
    /// starting pose. The threshold is 0.3 m, the one claim S1 uses; `point_to_plane` selects the
    /// residual.
    pub fn icp_trace(
        &self,
        i: u32,
        j: u32,
        dx: f64,
        dy: f64,
        dtheta: f64,
        point_to_plane: bool,
        max_iterations: u32,
    ) -> Result<Vec<f64>, String> {
        let (fi, fj) = self.pair(i, j)?;
        let source = points_of(&fj.scan);
        let mut target = VoxelMap::new(0.1, 8).map_err(err)?;
        target.insert_all(&points_of(&fi.scan));
        let truth = fi.gt.between(&fj.gt);
        let init = truth.boxplus(&Vector3::new(dx, dy, dtheta));
        let mut out = vec![0.0, init.x(), init.y(), init.theta(), f64::NAN, 0.0, 0.0, 0.0];
        for k in 1..=max_iterations.max(1) {
            let cfg = IcpConfig {
                metric: if point_to_plane {
                    IcpMetric::PointToPlane
                } else {
                    IcpMetric::PointToPoint
                },
                threshold: 0.3,
                max_iterations: k as usize,
                ..IcpConfig::default()
            };
            let r = register(&source, &target, init, &cfg).map_err(err)?;
            let row = |limit: u32| {
                [
                    limit as f64,
                    r.pose.x(),
                    r.pose.y(),
                    r.pose.theta(),
                    r.rmse,
                    r.correspondences as f64,
                    if r.settled { 1.0 } else { 0.0 },
                    if r.converged { 1.0 } else { 0.0 },
                ]
            };
            out.extend(row(k));
            // A run that stopped by itself before the limit gives the same answer at every larger
            // limit (the iteration is deterministic), so the remaining rows are copies.
            if (r.iterations as u32) < k {
                for later in k + 1..=max_iterations.max(1) {
                    out.extend(row(later));
                }
                break;
            }
        }
        Ok(out)
    }

    /// The ground-truth pose of scan `j` in scan `i`'s sensor frame: `x, y, theta`.
    pub fn pair_truth(&self, i: u32, j: u32) -> Result<Vec<f64>, String> {
        let (fi, fj) = self.pair(i, j)?;
        let t = fi.gt.between(&fj.gt);
        Ok(vec![t.x(), t.y(), t.theta()])
    }

    /// The downsampled points of scan `i` in its own sensor frame, as `x, y` pairs.
    pub fn scan_points(&self, i: u32) -> Result<Vec<f64>, String> {
        let f = self
            .frames
            .get(i as usize)
            .ok_or_else(|| format!("no scan {i}"))?;
        Ok(flat_xy(&points_of(&f.scan)))
    }

    /// The recording's ground-truth sensor poses, `x, y, theta` per scan, relative to the first.
    pub fn route(&self) -> Vec<f64> {
        let origin = self.frames[0].gt;
        self.frames
            .iter()
            .flat_map(|f| {
                let p = origin.between(&f.gt);
                [p.x(), p.y(), p.theta()]
            })
            .collect()
    }
}

impl Ch06Replay {
    fn pair(&self, i: u32, j: u32) -> Result<(&Frame, &Frame), String> {
        let get = |k: u32| {
            self.frames
                .get(k as usize)
                .ok_or_else(|| format!("no scan {k}"))
        };
        Ok((get(i)?, get(j)?))
    }
}

// ---------------------------------------------------------------------------------------
// One loop candidate
// ---------------------------------------------------------------------------------------

fn v2(x: f64, y: f64) -> Vector2<f64> {
    Vector2::new(x, y)
}

/// Three rooms of `crates/robotics_slam/tests/common/mod.rs`. `0` the empty square room (fourfold
/// symmetric), `1` the square room with one box that breaks the symmetry, `2` the pillar room of
/// Parts 3 to 5 (four pillars). (The tests' 2 m wide hall is not offered: whether it is refused as
/// degenerate or as not converged depends on the noise realization, which on this page differs from the
/// robot's.)
fn room(kind: u32) -> Result<World, String> {
    let mut w = World::empty();
    match kind {
        0..=2 => {
            w.add_rectangle(v2(-3.0, -3.0), v2(3.0, 3.0)).map_err(err)?;
            if kind == 1 {
                w.add_box(v2(1.5, 0.9), v2(0.6, 0.4), 0.4).map_err(err)?;
            }
            if kind == 2 {
                for (cx, cy) in [(1.8, 1.2), (-1.4, 2.0), (-2.0, -1.0), (0.9, -2.1)] {
                    let ring: Vec<_> = (0..12)
                        .map(|k| {
                            let a = std::f64::consts::TAU * k as f64 / 12.0;
                            v2(cx + 0.15 * a.cos(), cy + 0.15 * a.sin())
                        })
                        .collect();
                    w.add_polyline(&ring, true).map_err(err)?;
                }
            }
        }
        _ => return Err(format!("no room {kind}")),
    }
    Ok(w)
}

/// What the verifier said about one candidate. Poses are `x, y, theta` of the second view in the
/// first view's frame.
#[wasm_bindgen]
pub struct Ch06Verdict {
    status: f64,
    chi2: f64,
    rmse: f64,
    settled: bool,
    believed: Vec<f64>,
    truth: Vec<f64>,
    registered: Vec<f64>,
    points_a: Vec<f64>,
    points_b: Vec<f64>,
}

#[wasm_bindgen]
impl Ch06Verdict {
    /// 0 accepted; 1 no overlap, 2 not converged, 3 poor fit, 4 degenerate, 5 gate exceeded,
    /// 6 ambiguous (the first check that failed).
    pub fn status(&self) -> f64 {
        self.status
    }
    /// The chi-squared of the innovation if the candidate reached the gate, NaN otherwise.
    pub fn chi2(&self) -> f64 {
        self.chi2
    }
    /// Root-mean-square residual of the registration over its final correspondences (m), NaN if
    /// none was run.
    pub fn rmse(&self) -> f64 {
        self.rmse
    }
    /// Whether the registration settled (stopped on the step tolerance with enough correspondences).
    pub fn settled(&self) -> bool {
        self.settled
    }
    /// Where the graph believes the second view is: the odometry edge.
    pub fn believed(&self) -> Vec<f64> {
        self.believed.clone()
    }
    /// Where the second view really is.
    pub fn truth(&self) -> Vec<f64> {
        self.truth.clone()
    }
    /// What the registration found (the measurement if accepted, the last ICP pose if rejected
    /// after a registration, the belief if no registration was run).
    pub fn registered(&self) -> Vec<f64> {
        self.registered.clone()
    }
    /// The first view's points in its sensor frame, `x, y` pairs.
    pub fn points_a(&self) -> Vec<f64> {
        self.points_a.clone()
    }
    /// The second view's points in its own sensor frame, `x, y` pairs.
    pub fn points_b(&self) -> Vec<f64> {
        self.points_b.clone()
    }
}

/// The chi-squared gate of the loop verifier at its default confidence: `chi2_quantile(3, 0.95)`,
/// the number an innovation's squared Mahalanobis distance is compared with.
#[wasm_bindgen]
pub fn ch06_chi2_gate() -> f64 {
    robotics_core::prob::chi2_quantile(3, PoseGraphConfig::default().loops.chi2_alpha)
        .expect("three degrees of freedom and a valid confidence")
}

/// Build a two-keyframe graph and verify the loop candidate between its keyframes.
///
/// `room_kind` is as [`room`]. View A is at `(0, 0, 0)` for rooms 0 and 1 and at `(0.5, 0.2, 0.3)`
/// for room 2; view B is `(dx, dy, rotation)` away from it in A's frame, with `(dx, dy)` fixed per
/// room (room 0: none, 1: `(0.05, 0.02)`, 2: `(0.08, 0.07)`), as the tests of `pose_graph_it.rs` place them. The graph is told `belief_error`
/// metres more along x than the truth, with a position sigma of 5 cm. `seed` seeds the lidar noise
/// (see the module's note on WebAssembly).
#[wasm_bindgen]
pub fn ch06_verify(
    room_kind: u32,
    rotation: f64,
    belief_error: f64,
    seed: u64,
) -> Result<Ch06Verdict, String> {
    if !(rotation.is_finite() && belief_error.is_finite()) {
        return Err("rotation and belief error must be finite".into());
    }
    let world = room(room_kind)?;
    let (pose_a, rel) = match room_kind {
        0 => (Se2::IDENTITY, Se2::new(0.0, 0.0, rotation)),
        1 => (Se2::IDENTITY, Se2::new(0.05, 0.02, rotation)),
        _ => (Se2::new(0.5, 0.2, 0.3), Se2::new(0.08, 0.07, rotation)),
    };
    let pose_b = pose_a.compose(&rel);
    let mut lidar = Lidar::new(
        LidarConfig {
            mount: Se2::IDENTITY,
            range_min: 0.12,
            range_max: 3.5,
            range_sigma: 0.015,
            dropout_probability: 0.0,
            rate_hz: 5.0,
            ..LidarConfig::default()
        },
        seed,
    )
    .map_err(err)?;
    let sa = lidar.scan(&world, &pose_a, Timestamp::from_secs_f64(0.0));
    let sb = lidar.scan(&world, &pose_b, Timestamp::from_secs_f64(0.2));
    let (pa, pb) = (points_of(&sa), points_of(&sb));
    let believed = rel.boxplus(&Vector3::new(belief_error, 0.0, 0.0));
    let mut g = PoseGraph::new(PoseGraphConfig::default()).map_err(err)?;
    g.add_keyframe(pa.clone(), &Se2::IDENTITY, &info(0.05, 0.01))
        .map_err(err)?;
    g.add_keyframe(pb.clone(), &believed, &info(0.05, 0.01))
        .map_err(err)?;
    let cand = LoopCandidate {
        from: 0,
        to: 1,
        predicted: g.poses()[0].between(&g.poses()[1]),
        distance: believed.translation().norm(),
        sigma_position: 0.05,
    };
    let v = g.verify(&cand).map_err(err)?;
    let (chi2, registered, rmse, settled) = match &v {
        Verification::Accepted {
            chi2,
            measurement,
            icp,
            ..
        } => (*chi2, *measurement, icp.rmse, icp.settled),
        Verification::Rejected { reason, icp } => (
            match reason {
                Reason::GateExceeded { chi2 } => *chi2,
                _ => f64::NAN,
            },
            icp.map(|r| r.pose).unwrap_or(believed),
            icp.map(|r| r.rmse).unwrap_or(f64::NAN),
            icp.map(|r| r.settled).unwrap_or(false),
        ),
    };
    let p = |s: &Se2| vec![s.x(), s.y(), s.theta()];
    Ok(Ch06Verdict {
        status: status_code(&v),
        chi2,
        rmse,
        settled,
        believed: p(&believed),
        truth: p(&rel),
        registered: p(&registered),
        points_a: flat_xy(&pa),
        points_b: flat_xy(&pb),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const RUN: &str = "v2-synthetic-4-eval-rectangle";

    fn evidence(run: &str, file: &str) -> String {
        let path = format!(
            "{}/../../evidence/phase-5-slam/{run}/{file}",
            env!("CARGO_MANIFEST_DIR")
        );
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
    }

    /// `key: {ate_rms_m: …}` of an `s4` block of a metrics file.
    fn s4_ate(yaml: &str, which: &str) -> f64 {
        let line = yaml
            .lines()
            .find(|l| l.trim_start().starts_with(&format!("{which}: {{ate_rms_m")))
            .unwrap_or_else(|| panic!("no {which} line"));
        let v = line.split("ate_rms_m: ").nth(1).unwrap();
        v.split(',').next().unwrap().trim().parse().unwrap()
    }

    fn yaml_number(yaml: &str, key: &str) -> f64 {
        let line = yaml
            .lines()
            .find(|l| l.trim_start().starts_with(&format!("{key}:")))
            .unwrap_or_else(|| panic!("no {key}"));
        line.split(':').nth(1).unwrap().trim().parse().unwrap()
    }

    /// The replay on the page's code path writes the trajectory errors and the S5 counts of the
    /// evidence run, for the natural stream and for the stream with 0.004 rad of injected yaw error
    /// per keyframe (`v2-synthetic-4c-eval-rectangle-injected`).
    #[test]
    fn replay_reproduces_the_evidence_runs() {
        let csv = evidence(RUN, "scans.csv");
        for (run, bias) in [(RUN, 0.0), ("v2-synthetic-4c-eval-rectangle-injected", 0.004)] {
            let yaml = evidence(run, "slam-metrics.yaml");
            let mut r = Ch06Replay::new(&csv, bias).unwrap();
            r.step(100_000).unwrap();
            let g = r.trajectory_errors(0);
            let c = r.trajectory_errors(1);
            assert!((g[0] - s4_ate(&yaml, "graph")).abs() < 1e-9, "{run}: graph {}", g[0]);
            assert!((c[0] - s4_ate(&yaml, "odometry")).abs() < 1e-9, "{run}: chain {}", c[0]);
            assert_eq!(g[3], yaml_number(&yaml, "keyframes_made"));
            let s5 = r.s5_counts();
            assert_eq!(s5[0], yaml_number(&yaml, "candidates"), "{run}");
            assert_eq!(s5[1], yaml_number(&yaml, "accepted_correct"), "{run}");
            assert_eq!(s5[2], yaml_number(&yaml, "accepted_wrong"), "{run}");
        }
    }

    /// Stepping in pieces is stepping at once.
    #[test]
    fn stepping_in_pieces_is_stepping_at_once() {
        let csv = evidence(RUN, "scans.csv");
        let mut a = Ch06Replay::new(&csv, 0.0).unwrap();
        let mut b = Ch06Replay::new(&csv, 0.0).unwrap();
        while a.processed() < a.total() {
            a.step(7).unwrap();
        }
        b.step(100_000).unwrap();
        let bits = |v: Vec<f64>| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(a.graph_poses()), bits(b.graph_poses()));
        assert_eq!(bits(a.candidates()), bits(b.candidates()));
    }

    /// The last row of the trace with a generous limit is `register` run with that limit, and a
    /// small displacement is recovered.
    #[test]
    fn icp_trace_recovers_a_small_displacement() {
        let csv = evidence(RUN, "scans.csv");
        let r = Ch06Replay::new(&csv, 0.0).unwrap();
        let mid = r.total() / 3;
        let t = r.icp_trace(mid, mid + 5, 0.1, 0.05, 0.1, true, 12).unwrap();
        let truth = r.pair_truth(mid, mid + 5).unwrap();
        let last = &t[t.len() - 8..];
        assert!(((last[1] - truth[0]).powi(2) + (last[2] - truth[1]).powi(2)).sqrt() < 0.02);
        assert!((last[3] - truth[2]).abs() < 0.01);
        assert_eq!(last[6], 1.0);
    }

    /// Four of the verdicts of `crates/robotics_slam/tests/pose_graph_it.rs`, through the page's
    /// wrapper: a true loop is accepted; an empty square room cannot tell a quarter turn from none;
    /// the same room with a box can; a belief 0.3 m off fails the gate.
    #[test]
    fn the_wrapper_gives_the_verdicts_of_the_integration_tests() {
        let s = |room, rot, bel| ch06_verify(room, rot, bel, 31).unwrap().status;
        assert_eq!(s(2, 0.04, 0.0), 0.0, "a true loop in the pillar room");
        assert_eq!(s(0, std::f64::consts::FRAC_PI_2, 0.0), 6.0, "alias, empty square");
        assert_eq!(s(1, std::f64::consts::FRAC_PI_2, 0.0), 0.0, "alias, square with a box");
        assert_eq!(s(2, 0.04, 0.3), 5.0, "the graph is sure of the wrong place");
    }
}
