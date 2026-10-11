//! The pose graph: keyframes, odometry edges, and loop closures that are verified before
//! they are believed.
//!
//! A [`PoseGraph`] holds one pose per keyframe (the sensor pose in the odometry frame), the
//! keyframe's downsampled scan, and a [`FactorGraph`] of a gauge prior, odometry edges and
//! accepted loop edges. Loop closure has two stages (implementation section 6):
//!
//! 1. [`PoseGraph::propose_loops`] is geometry on the *estimated* trajectory: earlier
//!    keyframes at least `min_path_gap` of path away whose estimated distance is within the
//!    overlap radius plus three standard deviations of the estimated relative position (from
//!    the exact marginal covariance). It cannot find a loop that drift has carried out of its
//!    own covariance.
//! 2. [`PoseGraph::verify`] registers the new keyframe's scan against a submap around the
//!    candidate and accepts only if the registration **fits**, is **not degenerate**, is
//!    **consistent** with the graph (χ² on the relative-pose innovation) and is
//!    **unambiguous** (perturbed and quarter-turn restarts do not land on another good fit that
//!    explains nearly as much of the scan as the main solution).
//!    The first failing check is the reason returned. The χ² check cannot reject a false
//!    candidate that lies inside the graph's own uncertainty; the other three exist for those.
//!
//! No loop edge enters the graph except through [`PoseGraph::add_loop`] with an accepted
//! [`Verification`]. There is no distance-only shortcut.

use crate::factor_graph::{
    Factor, FactorGraph, LinearSolver, OptimizeReport, OptimizerConfig, Values, VarKey,
};
use crate::icp::{register, support_fraction, IcpConfig, IcpMetric, IcpResult};
use crate::kernel::RobustKernel;
use crate::voxel::VoxelMap;
use crate::SlamError;
use nalgebra::{Matrix3, Vector2, Vector3};
use robotics_core::prob::chi2_quantile;
use robotics_core::{Manifold, Se2};

/// Loop-closure parameters. Initial values, frozen after the development run (DS18); each
/// says in its comment whether it is derived or chosen.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopConfig {
    /// Fewest metres of path between two keyframes for one to be a loop candidate. Chosen.
    pub min_path_gap: f64,
    /// How far apart two views can be and still share a wall (m). Chosen: well inside the
    /// 3.5 m sensor range.
    pub overlap_radius: f64,
    /// Never search beyond this distance (m).
    pub max_radius: f64,
    /// Multiplier on the relative-position standard deviation in the search radius.
    pub sigma_multiplier: f64,
    /// Candidates kept per new keyframe, nearest first.
    pub max_candidates: usize,
    /// Keyframes on each side of the candidate whose points form the target submap.
    pub submap_half_width: usize,
    /// ICP settings for verification (`threshold` is the verification threshold).
    pub icp: IcpConfig,
    /// Fit: smallest fraction of source points explained within `support_radius`. Chosen.
    pub fit_inlier_fraction_min: f64,
    /// Fit: fewest explained points. Chosen.
    pub fit_inliers_min: usize,
    /// Below this fraction of source points with a correspondence there is no overlap at all.
    pub overlap_fraction_min: f64,
    /// χ² gate confidence, `dof = 3`.
    pub chi2_alpha: f64,
    /// Ambiguity: translation offset of the perturbed restarts (m).
    pub ambiguity_offset: f64,
    /// Ambiguity: rotation offset of the perturbed restarts (rad).
    pub ambiguity_rotation: f64,
    /// Ambiguity: translation tolerance for agreement with the main solution (m).
    pub ambiguity_tol_trans: f64,
    /// Ambiguity: rotation tolerance for agreement (rad).
    pub ambiguity_tol_rot: f64,
    /// Ambiguity: a restart that lands elsewhere with a good fit is an alias unless it explains
    /// at least this much less of the scan than the main solution does (fraction of source
    /// points). Chosen: a fit test alone passes an alias that leaves a few features unexplained.
    pub ambiguity_margin: f64,
    /// Ambiguity: radius within which a source point counts as explained (m). Derived: about
    /// three times the 15 mm range noise.
    pub support_radius: f64,
    /// Kernel of an accepted loop edge. Huber at `√χ²(3, 0.95)`: convex, so it cannot lock a
    /// bad start; it limits pull, and the gate is what keeps out large lies.
    pub kernel: RobustKernel,
}

impl Default for LoopConfig {
    fn default() -> Self {
        Self {
            min_path_gap: 6.0,
            overlap_radius: 1.5,
            max_radius: 3.0,
            sigma_multiplier: 3.0,
            max_candidates: 3,
            submap_half_width: 1,
            icp: IcpConfig {
                metric: IcpMetric::PointToPlane,
                threshold: 0.6,
                ..IcpConfig::default()
            },
            fit_inlier_fraction_min: 0.5,
            fit_inliers_min: 80,
            overlap_fraction_min: 0.2,
            chi2_alpha: 0.95,
            ambiguity_offset: 0.3,
            ambiguity_rotation: 0.25,
            ambiguity_tol_trans: 0.05,
            ambiguity_tol_rot: 0.02,
            ambiguity_margin: 0.05,
            support_radius: 0.05,
            kernel: RobustKernel::Huber { k: 2.796 },
        }
    }
}

/// Pose-graph parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseGraphConfig {
    /// A keyframe is created after this much translation (m). Chosen: a seventh of the range.
    pub keyframe_translation: f64,
    /// ... or this much rotation (rad). Chosen.
    pub keyframe_rotation: f64,
    /// Standard deviation of the gauge prior on the first keyframe (translation, m; rotation, rad).
    pub prior_sigma: f64,
    /// Kernel of odometry edges.
    pub odometry_kernel: RobustKernel,
    /// Loop closure.
    pub loops: LoopConfig,
    /// The optimizer.
    pub optimizer: OptimizerConfig,
}

impl Default for PoseGraphConfig {
    fn default() -> Self {
        Self {
            keyframe_translation: 0.5,
            keyframe_rotation: 0.35,
            prior_sigma: 1e-3,
            odometry_kernel: RobustKernel::L2,
            loops: LoopConfig::default(),
            optimizer: OptimizerConfig::default(),
        }
    }
}

/// A keyframe: the points of one scan in its sensor frame and how far the robot had travelled.
#[derive(Debug, Clone)]
pub struct Keyframe {
    /// Index of the keyframe, equal to its pose index in the graph.
    pub id: usize,
    /// Downsampled scan points in the sensor frame.
    pub points: Vec<Vector2<f64>>,
    /// Path length from the first keyframe to this one (m).
    pub arc_length: f64,
}

/// A pair of keyframes that may have seen the same place.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopCandidate {
    /// The earlier keyframe.
    pub from: usize,
    /// The new keyframe.
    pub to: usize,
    /// The graph's current estimate of `X_from⁻¹ X_to`.
    pub predicted: Se2,
    /// Estimated distance between the two (m).
    pub distance: f64,
    /// Largest standard deviation of the estimated relative position (m).
    pub sigma_position: f64,
}

/// Why a candidate was rejected: the first failing check, in the order they are tried.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reason {
    /// Too few source points found a correspondence at all: the views do not overlap.
    NoOverlap,
    /// ICP did not converge.
    NotConverged,
    /// It converged, but the fit is poor (RMS, inlier fraction or count).
    PoorFit,
    /// The registration's information matrix is rank-deficient (a corridor, one wall).
    Degenerate,
    /// The registration disagrees with the graph by more than its covariance allows.
    GateExceeded {
        /// The innovation's squared Mahalanobis distance.
        chi2: f64,
    },
    /// A restart converged to another good fit.
    Ambiguous,
}

/// The outcome of verifying one candidate.
#[derive(Debug, Clone, Copy)]
pub enum Verification {
    /// Believed: add it with [`PoseGraph::add_loop`].
    Accepted {
        /// The measured `X_from⁻¹ X_to`.
        measurement: Se2,
        /// Information of the measurement.
        information: Matrix3<f64>,
        /// The innovation's squared Mahalanobis distance.
        chi2: f64,
        /// The registration.
        icp: IcpResult,
    },
    /// Not believed.
    Rejected {
        /// The first failing check.
        reason: Reason,
        /// The registration, if one was run.
        icp: Option<IcpResult>,
    },
}

/// A candidate with its verdict, as recorded by [`PoseGraph::process_loops`].
#[derive(Debug, Clone, Copy)]
pub struct LoopOutcome {
    /// The candidate.
    pub candidate: LoopCandidate,
    /// What verification said.
    pub verification: Verification,
}

/// The graph.
#[derive(Debug, Clone)]
pub struct PoseGraph {
    cfg: PoseGraphConfig,
    graph: FactorGraph,
    values: Values,
    keyframes: Vec<Keyframe>,
    loop_edges: usize,
}

/// A registration fits when it settled and explains enough of the scan within a few lidar
/// sigmas. The tight radius, not the loose search threshold, defines "explained": the RMS of
/// everything within the search threshold is polluted by the points a changed scene adds.
fn fit_ok(r: &IcpResult, source: &[Vector2<f64>], target: &VoxelMap, cfg: &LoopConfig) -> bool {
    if !r.settled {
        return false;
    }
    let frac = support_fraction(source, target, &r.pose, cfg.support_radius);
    frac >= cfg.fit_inlier_fraction_min
        && frac * source.len() as f64 >= cfg.fit_inliers_min.min(source.len()) as f64
}

impl PoseGraph {
    /// An empty graph.
    pub fn new(cfg: PoseGraphConfig) -> Result<Self, SlamError> {
        cfg.loops.icp.validate()?;
        cfg.loops.validate()?;
        if !(cfg.prior_sigma.is_finite() && cfg.prior_sigma > 0.0) {
            return Err(SlamError::InvalidParameter {
                what: "prior_sigma",
                value: cfg.prior_sigma,
            });
        }
        chi2_quantile(3, cfg.loops.chi2_alpha).map_err(|_| SlamError::InvalidParameter {
            what: "chi2_alpha",
            value: cfg.loops.chi2_alpha,
        })?;
        Ok(Self {
            cfg,
            graph: FactorGraph::new(),
            values: Values::default(),
            keyframes: Vec::new(),
            loop_edges: 0,
        })
    }

    /// Whether `since_last` (the motion since the last keyframe) calls for a new one.
    pub fn wants_keyframe(&self, since_last: &Se2) -> bool {
        since_last.translation().norm() >= self.cfg.keyframe_translation
            || since_last.theta().abs() >= self.cfg.keyframe_rotation
    }

    /// Add a keyframe. The first one is the origin of the graph and carries the gauge prior;
    /// later ones are placed at `previous · delta` with an odometry edge of that measurement and
    /// information.
    pub fn add_keyframe(
        &mut self,
        points: Vec<Vector2<f64>>,
        delta: &Se2,
        information: &Matrix3<f64>,
    ) -> Result<usize, SlamError> {
        let id = self.keyframes.len();
        if id == 0 {
            self.values.poses.push(Se2::IDENTITY);
            let s = self.cfg.prior_sigma;
            let prior = Matrix3::from_diagonal(&Vector3::repeat(1.0 / (s * s)));
            self.graph
                .add(Factor::prior(0, Se2::IDENTITY, &prior, RobustKernel::L2)?);
            self.keyframes.push(Keyframe {
                id,
                points,
                arc_length: 0.0,
            });
            return Ok(0);
        }
        if !delta.is_finite() {
            return Err(SlamError::NonFinite("odometry increment"));
        }
        let pose = self.values.poses[id - 1].compose(delta);
        self.graph.add(Factor::between(
            id - 1,
            id,
            *delta,
            information,
            self.cfg.odometry_kernel,
        )?);
        self.values.poses.push(pose);
        let arc = self.keyframes[id - 1].arc_length + delta.translation().norm();
        self.keyframes.push(Keyframe {
            id,
            points,
            arc_length: arc,
        });
        Ok(id)
    }

    /// Current pose estimates, one per keyframe.
    pub fn poses(&self) -> &[Se2] {
        &self.values.poses
    }

    /// The keyframes.
    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// Accepted loop edges so far.
    pub fn loop_edge_count(&self) -> usize {
        self.loop_edges
    }

    /// The underlying factor graph.
    pub fn factor_graph(&self) -> &FactorGraph {
        &self.graph
    }

    /// The current values.
    pub fn values(&self) -> &Values {
        &self.values
    }

    /// The covariance of `X_from⁻¹ X_to` at the current estimate, in the tangent at that
    /// relative pose, from the joint marginal of the two poses.
    pub fn relative_covariance(&self, from: usize, to: usize) -> Result<Matrix3<f64>, SlamError> {
        let c = self.graph.marginal_covariance(
            &self.values,
            &[VarKey::Pose(from), VarKey::Pose(to)],
            self.cfg.optimizer.solver,
        )?;
        let blk = |r: usize, k: usize| Matrix3::from_fn(|i, j| c[(3 * r + i, 3 * k + j)]);
        let t = self.values.poses[from].between(&self.values.poses[to]);
        // δT = −Ad(T⁻¹) δ_from + δ_to for right perturbations.
        let a = t.inverse().adjoint();
        let cov =
            a * blk(0, 0) * a.transpose() + blk(1, 1) - a * blk(0, 1) - blk(1, 0) * a.transpose();
        Ok((cov + cov.transpose()) * 0.5)
    }

    /// Earlier keyframes that may have seen what keyframe `to` sees.
    pub fn propose_loops(&self, to: usize) -> Result<Vec<LoopCandidate>, SlamError> {
        let lc = &self.cfg.loops;
        if to >= self.keyframes.len() {
            return Err(SlamError::UnknownVariable(format!("Pose({to})")));
        }
        let mut out = Vec::new();
        for j in 0..to {
            if self.keyframes[to].arc_length - self.keyframes[j].arc_length < lc.min_path_gap {
                continue;
            }
            let rel = self.values.poses[j].between(&self.values.poses[to]);
            let distance = rel.translation().norm();
            if distance > lc.max_radius {
                continue;
            }
            let cov = self.relative_covariance(j, to)?;
            let sigma = cov[(0, 0)].max(cov[(1, 1)]).max(0.0).sqrt();
            if distance <= (lc.overlap_radius + lc.sigma_multiplier * sigma).min(lc.max_radius) {
                out.push(LoopCandidate {
                    from: j,
                    to,
                    predicted: rel,
                    distance,
                    sigma_position: sigma,
                });
            }
        }
        out.sort_by(|a, b| a.distance.total_cmp(&b.distance).then(a.from.cmp(&b.from)));
        out.truncate(lc.max_candidates);
        Ok(out)
    }

    /// The points of the keyframes around `centre`, in its frame, excluding keyframe `new` and
    /// everything after it: a scan must never be registered against itself.
    fn submap(&self, centre: usize, new: usize) -> Result<VoxelMap, SlamError> {
        let lc = &self.cfg.loops;
        let mut map = VoxelMap::new(0.1, 8)?;
        let lo = centre.saturating_sub(lc.submap_half_width);
        let hi = (centre + lc.submap_half_width).min(new.saturating_sub(1));
        let origin = self.values.poses[centre];
        for m in lo..=hi {
            let rel = origin.between(&self.values.poses[m]);
            for p in &self.keyframes[m].points {
                map.insert(rel.act(p));
            }
        }
        Ok(map)
    }

    /// Verify one candidate against the graph as it is now. Does not change the graph.
    pub fn verify(&self, cand: &LoopCandidate) -> Result<Verification, SlamError> {
        let lc = &self.cfg.loops;
        let source = &self.keyframes[cand.to].points;
        let target = self.submap(cand.from, cand.to)?;
        let reject = |reason, icp| Ok(Verification::Rejected { reason, icp });
        let r = register(source, &target, cand.predicted, &lc.icp)?;
        if r.inlier_fraction < lc.overlap_fraction_min {
            return reject(Reason::NoOverlap, Some(r));
        }
        if !r.settled {
            return reject(Reason::NotConverged, Some(r));
        }
        if !fit_ok(&r, source, &target, lc) {
            return reject(Reason::PoorFit, Some(r));
        }
        if r.degenerate {
            return reject(Reason::Degenerate, Some(r));
        }
        // Consistency: the innovation of the registration against the graph's belief.
        let nu = r.pose.between(&cand.predicted).log();
        let sigma_rel = self.relative_covariance(cand.from, cand.to)?;
        let info = r.information;
        let s = sigma_rel
            + info
                .try_inverse()
                .ok_or(SlamError::SingularSystem("registration information"))?;
        let s_inv = s
            .try_inverse()
            .ok_or(SlamError::SingularSystem("innovation covariance"))?;
        let chi2 = nu.dot(&(s_inv * nu));
        let gate = chi2_quantile(3, lc.chi2_alpha).map_err(|_| SlamError::InvalidParameter {
            what: "chi2_alpha",
            value: lc.chi2_alpha,
        })?;
        if chi2 > gate {
            return reject(Reason::GateExceeded { chi2 }, Some(r));
        }
        // Ambiguity: restarts around the solution, and its quarter turns (the symmetry of a
        // square room), must not converge to another good fit.
        let (d, a) = (lc.ambiguity_offset, lc.ambiguity_rotation);
        let mut starts: Vec<Vector3<f64>> = vec![
            Vector3::new(d, 0.0, 0.0),
            Vector3::new(-d, 0.0, 0.0),
            Vector3::new(0.0, d, 0.0),
            Vector3::new(0.0, -d, 0.0),
            Vector3::new(0.0, 0.0, a),
            Vector3::new(0.0, 0.0, -a),
            Vector3::new(d, d, 0.0),
            Vector3::new(-d, -d, 0.0),
        ];
        let q = std::f64::consts::FRAC_PI_2;
        starts.extend([
            Vector3::new(0.0, 0.0, q),
            Vector3::new(0.0, 0.0, -q),
            Vector3::new(0.0, 0.0, 2.0 * q),
        ]);
        for off in &starts {
            let other = register(source, &target, r.pose.boxplus(off), &lc.icp)?;
            if fit_ok(&other, source, &target, lc) {
                let e = r.pose.between(&other.pose).log();
                if e.xy().norm() > lc.ambiguity_tol_trans || e.z.abs() > lc.ambiguity_tol_rot {
                    let main = support_fraction(source, &target, &r.pose, lc.support_radius);
                    let alias = support_fraction(source, &target, &other.pose, lc.support_radius);
                    if alias >= main - lc.ambiguity_margin {
                        return reject(Reason::Ambiguous, Some(r));
                    }
                }
            }
        }
        Ok(Verification::Accepted {
            measurement: r.pose,
            information: info,
            chi2,
            icp: r,
        })
    }

    /// Add an accepted loop edge. A rejected verification is an error, not a no-op.
    pub fn add_loop(
        &mut self,
        cand: &LoopCandidate,
        verification: &Verification,
    ) -> Result<(), SlamError> {
        match verification {
            Verification::Accepted {
                measurement,
                information,
                ..
            } => {
                self.graph.add(Factor::between(
                    cand.from,
                    cand.to,
                    *measurement,
                    information,
                    self.cfg.loops.kernel,
                )?);
                self.loop_edges += 1;
                Ok(())
            }
            Verification::Rejected { .. } => Err(SlamError::InvalidParameter {
                what: "add_loop called with a rejected verification",
                value: cand.to as f64,
            }),
        }
    }

    /// Optimize the graph in place.
    pub fn optimize(&mut self) -> Result<OptimizeReport, SlamError> {
        self.graph.optimize(&mut self.values, &self.cfg.optimizer)
    }

    /// Propose, verify and add the loops for keyframe `to`, then optimize once if any was added.
    /// Every candidate is verified against the graph as it was before this call.
    pub fn process_loops(&mut self, to: usize) -> Result<Vec<LoopOutcome>, SlamError> {
        let cands = self.propose_loops(to)?;
        let mut out = Vec::with_capacity(cands.len());
        for c in cands {
            out.push(LoopOutcome {
                candidate: c,
                verification: self.verify(&c)?,
            });
        }
        let mut added = false;
        for o in &out {
            if matches!(o.verification, Verification::Accepted { .. }) {
                self.add_loop(&o.candidate, &o.verification)?;
                added = true;
            }
        }
        if added {
            self.optimize()?;
        }
        Ok(out)
    }

    /// The solver in use (for tests and reports).
    pub fn solver(&self) -> LinearSolver {
        self.cfg.optimizer.solver
    }
}

impl LoopConfig {
    fn validate(&self) -> Result<(), SlamError> {
        for (what, v) in [
            ("min_path_gap", self.min_path_gap),
            ("overlap_radius", self.overlap_radius),
            ("max_radius", self.max_radius),
            ("ambiguity_offset", self.ambiguity_offset),
            ("ambiguity_rotation", self.ambiguity_rotation),
            ("ambiguity_tol_trans", self.ambiguity_tol_trans),
            ("ambiguity_tol_rot", self.ambiguity_tol_rot),
            ("support_radius", self.support_radius),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return Err(SlamError::InvalidParameter { what, value: v });
            }
        }
        for (what, v) in [
            ("fit_inlier_fraction_min", self.fit_inlier_fraction_min),
            ("overlap_fraction_min", self.overlap_fraction_min),
            ("ambiguity_margin", self.ambiguity_margin),
        ] {
            if !(v.is_finite() && (0.0..=1.0).contains(&v)) {
                return Err(SlamError::InvalidParameter { what, value: v });
            }
        }
        self.kernel.validate()
    }
}
