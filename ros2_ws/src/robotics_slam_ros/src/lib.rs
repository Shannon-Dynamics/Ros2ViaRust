//! `robotics_slam_ros`: the plumbing between ROS 2 messages and the pure `robotics_slam` crate,
//! kept out of the node binaries so that it can be unit-tested without a running ROS graph.
//!
//! ```text
//!  /scan ──► scan_matcher_node ──► registration   (geometry_msgs/PoseWithCovarianceStamped)
//!            (LidarOdometry)   └─► scan_matched   (the same LaserScan, reliable, in step with it)
//!                                      │
//!                                      ▼
//!                               pose_graph_node ──► slam/pose, slam/path, /diagnostics
//!                               (GraphBuilder)
//! ```
//!
//! **What a registration is.** The pose of the scan's sensor frame relative to the sensor frame of
//! the previous scan the scan matcher published (the increment `previous⁻¹ · current`), stamped
//! with the scan's own stamp, in `header.frame_id` = the scan's frame. Its covariance is the inverse
//! of the ICP information matrix of that increment (scaled by `info_inflation`), in the tangent
//! space at the increment with the `(x, y, θ)` slots of the 6×6 message covariance (the framework's
//! planar convention, `robotics_ros_bridge::convert::cov3_to_ros`). It is not a pose in any map.
//!
//! **What the pose graph builds.** Keyframes and odometry edges, and, **only when
//! [`GraphBuilderConfig::loop_closure`] is set**, the loop edges of `robotics_slam`'s detector and
//! verifier. It is off by default, and while it is off the builder never calls either, the node's
//! diagnostics say `loop_closure: unsupported` ([`LOOP_CLOSURE_STATUS`]) and the graph is exactly
//! the chain of increments: the decision of `docs/phase-5-slam/problem.md` section 5 stands for
//! every run that does not ask. A run that asks says `loop_closure: enabled`
//! ([`LOOP_CLOSURE_ENABLED_STATUS`]) in every message, so no log can be mistaken for the other.
//!
//! **The map.** [`render`] replays the stored scan of every keyframe at its current pose into a
//! fresh Phase 4 grid (decision DS17); the node publishes it as `/map` after every loop closure
//! that changed the graph and once more at the end of a run, and owns `map -> odom`
//! (`pose_graph_node`). Claim S8 (a usable map) stays unclaimable: nothing grades this map.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use robotics_core::nalgebra::Matrix3;
use robotics_core::{FrameId, LaserScan, Se2, Timestamp};
use robotics_ros_bridge::convert::{
    cov3_from_ros, cov3_to_ros, header, pose_from_ros, pose_to_ros, time_from_ros, ConvertError,
};
use robotics_slam::odometry::EdgeAccumulator;
use robotics_slam::pose_graph::{LoopOutcome, PoseGraph, PoseGraphConfig, Verification};
use robotics_slam::SlamError;
use ros_env::geometry_msgs;
use std::collections::BTreeMap;

/// What the pose graph node reports about loop closure unless it was asked to do it. The detector
/// and the verifier exist in `robotics_slam` and are not called while this is what is reported
/// (see the module documentation).
pub const LOOP_CLOSURE_STATUS: &str = "unsupported";

/// What it reports when [`GraphBuilderConfig::loop_closure`] is set. It names the switch, not a
/// verdict: whether loop closure should be the default is the owner's decision of
/// `docs/phase-5-slam/problem.md` section 5, taken on the evidence of claim S5.
pub const LOOP_CLOSURE_ENABLED_STATUS: &str = "enabled";

/// Why [`LOOP_CLOSURE_STATUS`] is what it is, for the diagnostics.
pub const LOOP_CLOSURE_REASON: &str =
    "S5 measured on synthetic data only and weakly against false \
candidates; no Webots-level run; owner decision pending (docs/phase-5-slam/problem.md section 5)";

/// What the diagnostics say instead of [`LOOP_CLOSURE_REASON`] when loop closure is enabled.
pub const LOOP_CLOSURE_ENABLED_REASON: &str =
    "enabled by the parameter loop_closure.enabled; whether it is the default is the owner decision of \
docs/phase-5-slam/problem.md section 5";

pub mod render;

/// Errors of the plumbing.
#[derive(Debug, thiserror::Error)]
pub enum PlumbingError {
    /// A message could not be converted.
    #[error("conversion: {0}")]
    Convert(#[from] ConvertError),
    /// The SLAM library refused something.
    #[error("slam: {0}")]
    Slam(#[from] SlamError),
    /// A registration whose covariance is not invertible or not positive.
    #[error("registration covariance is not a valid covariance")]
    BadCovariance,
}

/// One registration: the increment between two consecutive scans and its information.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Registration {
    /// Stamp of the later scan.
    pub stamp: Timestamp,
    /// `previous⁻¹ · current`, in the previous scan's sensor frame.
    pub delta: Se2,
    /// Information matrix of `delta` in its own tangent.
    pub information: Matrix3<f64>,
}

/// A registration as a `geometry_msgs/PoseWithCovarianceStamped` (see the module documentation).
pub fn registration_to_ros(
    stamp: Timestamp,
    frame: &FrameId,
    delta: &Se2,
    information: &Matrix3<f64>,
) -> Result<geometry_msgs::msg::PoseWithCovarianceStamped, PlumbingError> {
    let cov = information
        .try_inverse()
        .ok_or(PlumbingError::BadCovariance)?;
    if !cov.iter().all(|v| v.is_finite()) {
        return Err(PlumbingError::BadCovariance);
    }
    Ok(geometry_msgs::msg::PoseWithCovarianceStamped {
        header: header(stamp, frame),
        pose: geometry_msgs::msg::PoseWithCovariance {
            pose: pose_to_ros(delta),
            covariance: cov3_to_ros(&cov),
        },
    })
}

/// The inverse of [`registration_to_ros`].
pub fn registration_from_ros(
    m: &geometry_msgs::msg::PoseWithCovarianceStamped,
) -> Result<Registration, PlumbingError> {
    let delta = pose_from_ros(&m.pose.pose)?;
    let cov = cov3_from_ros(&m.pose.covariance)?;
    let information = cov.try_inverse().ok_or(PlumbingError::BadCovariance)?;
    if !information.iter().all(|v| v.is_finite()) {
        return Err(PlumbingError::BadCovariance);
    }
    Ok(Registration {
        stamp: time_from_ros(&m.header.stamp),
        delta,
        information,
    })
}

/// A keyframe the graph has just created.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NewKeyframe {
    /// Keyframe index.
    pub id: usize,
    /// Stamp of its scan (ns).
    pub stamp_ns: i64,
    /// Its pose in the frame of the first keyframe's sensor.
    pub pose: Se2,
    /// Path length from the first keyframe (m).
    pub arc_length: f64,
}

/// Counters the node reports.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BuilderCounters {
    /// Scans handed in.
    pub scans: u64,
    /// Registrations handed in.
    pub registrations: u64,
    /// Registrations joined with their scan and processed.
    pub processed: u64,
    /// Registrations at or before the last processed stamp (ignored).
    pub out_of_order: u64,
    /// Buffered scans dropped because they would never be joined.
    pub scans_dropped: u64,
    /// Edges the library refused (a singular compounded information).
    pub edge_errors: u64,
    /// Loop candidates the detector proposed (zero unless loop closure is enabled).
    pub loop_candidates: u64,
    /// Candidates the verifier accepted and the graph took as constraints.
    pub loop_accepted: u64,
    /// Candidates the verifier rejected.
    pub loop_rejected: u64,
    /// Optimizations run because a loop edge was added (each moves every keyframe).
    pub optimizations: u64,
    /// Times the library refused to process the loops of a keyframe.
    pub loop_errors: u64,
}

/// What loop closure did for one new keyframe.
#[derive(Debug, Clone, Default)]
pub struct LoopEvent {
    /// The keyframe the loops were looked for.
    pub keyframe: usize,
    /// The stamp (ns) of that keyframe's scan.
    pub stamp_ns: i64,
    /// Every candidate with the verifier's verdict.
    pub outcomes: Vec<LoopOutcome>,
}

impl LoopEvent {
    /// How many of the outcomes were accepted.
    pub fn accepted(&self) -> usize {
        self.outcomes
            .iter()
            .filter(|o| matches!(o.verification, Verification::Accepted { .. }))
            .count()
    }
}

/// Configuration of [`GraphBuilder`].
#[derive(Debug, Clone, Copy)]
pub struct GraphBuilderConfig {
    /// The pose graph (keyframe policy, gauge prior). Its loop settings are used only when
    /// [`loop_closure`](Self::loop_closure) is set.
    pub graph: PoseGraphConfig,
    /// Most scans or registrations kept waiting for their other half.
    pub max_buffered: usize,
    /// Run the detector and the verifier on every new keyframe and optimize when a loop is
    /// accepted. **Off by default** (see the module documentation).
    pub loop_closure: bool,
}

impl Default for GraphBuilderConfig {
    fn default() -> Self {
        Self {
            graph: PoseGraphConfig::default(),
            max_buffered: 400,
            loop_closure: false,
        }
    }
}

/// Joins registrations to their scans by stamp, accumulates the increments between keyframes and
/// builds the pose graph's keyframes and odometry edges, and, when asked to, its loop edges.
///
/// The first registration processed makes its scan the origin keyframe and its increment is
/// discarded (it is relative to a scan this builder never saw). Registrations are processed in
/// stamp order, each once, when both halves are present.
pub struct GraphBuilder {
    cfg: GraphBuilderConfig,
    graph: PoseGraph,
    acc: EdgeAccumulator,
    scans: BTreeMap<
        i64,
        (
            Vec<robotics_core::nalgebra::Vector2<f64>>,
            Option<LaserScan>,
        ),
    >,
    regs: BTreeMap<i64, Registration>,
    stamps: Vec<i64>,
    /// The full scan of every keyframe (`None` for one made without it), for rendering a map.
    kf_scans: Vec<Option<LaserScan>>,
    last_processed: Option<i64>,
    counters: BuilderCounters,
    broken: Option<String>,
    /// Bumped whenever an optimization moved the keyframes already published.
    pose_epoch: u64,
    loop_events: Vec<LoopEvent>,
    last_loop_error: String,
}

impl GraphBuilder {
    /// An empty builder.
    pub fn new(cfg: GraphBuilderConfig) -> Result<Self, PlumbingError> {
        Ok(Self {
            graph: PoseGraph::new(cfg.graph)?,
            cfg,
            acc: EdgeAccumulator::new(),
            scans: BTreeMap::new(),
            regs: BTreeMap::new(),
            stamps: Vec::new(),
            kf_scans: Vec::new(),
            last_processed: None,
            counters: BuilderCounters::default(),
            broken: None,
            pose_epoch: 0,
            loop_events: Vec::new(),
            last_loop_error: String::new(),
        })
    }

    /// The loop-closure status: [`LOOP_CLOSURE_STATUS`] unless the configuration enabled it, then
    /// [`LOOP_CLOSURE_ENABLED_STATUS`]. Nothing else changes it.
    pub fn loop_closure(&self) -> &'static str {
        if self.cfg.loop_closure {
            LOOP_CLOSURE_ENABLED_STATUS
        } else {
            LOOP_CLOSURE_STATUS
        }
    }

    /// Hand in the downsampled points of a scan (sensor frame), keyed by its stamp. A keyframe made
    /// from it has no stored scan, so a map cannot be rendered with it (see [`Self::on_scan_with_raw`]).
    pub fn on_scan(
        &mut self,
        stamp: Timestamp,
        points: Vec<robotics_core::nalgebra::Vector2<f64>>,
    ) -> Vec<NewKeyframe> {
        self.counters.scans += 1;
        self.scans.insert(stamp.as_nanos(), (points, None));
        self.drain()
    }

    /// Hand in a scan with its downsampled points; a keyframe made from it keeps the whole scan
    /// for the map render.
    pub fn on_scan_with_raw(
        &mut self,
        stamp: Timestamp,
        points: Vec<robotics_core::nalgebra::Vector2<f64>>,
        raw: LaserScan,
    ) -> Vec<NewKeyframe> {
        self.counters.scans += 1;
        self.scans.insert(stamp.as_nanos(), (points, Some(raw)));
        self.drain()
    }

    /// Hand in a registration.
    pub fn on_registration(&mut self, reg: Registration) -> Vec<NewKeyframe> {
        self.counters.registrations += 1;
        let key = reg.stamp.as_nanos();
        if self.last_processed.is_some_and(|l| key <= l) {
            self.counters.out_of_order += 1;
            return Vec::new();
        }
        self.regs.insert(key, reg);
        self.drain()
    }

    fn drain(&mut self) -> Vec<NewKeyframe> {
        let mut out = Vec::new();
        while self.broken.is_none() {
            let Some((&key, _)) = self.regs.iter().next() else {
                break;
            };
            let Some((points, raw)) = self.scans.remove(&key) else {
                break;
            };
            let reg = self.regs.remove(&key).expect("key came from the map");
            // Scans before this one can never be joined: registrations arrive in stamp order.
            let stale: Vec<i64> = self.scans.range(..key).map(|(k, _)| *k).collect();
            for k in stale {
                self.scans.remove(&k);
                self.counters.scans_dropped += 1;
            }
            self.counters.processed += 1;
            self.last_processed = Some(key);
            if let Some(kf) = self.process(key, reg, points, raw) {
                out.push(kf);
            }
        }
        while self.scans.len() > self.cfg.max_buffered {
            if let Some((&k, _)) = self.scans.iter().next() {
                self.scans.remove(&k);
                self.counters.scans_dropped += 1;
            }
        }
        if self.regs.len() > self.cfg.max_buffered && self.broken.is_none() {
            self.broken = Some(format!(
                "{} registrations are waiting for a scan that never came: the chain of increments is broken",
                self.regs.len()
            ));
        }
        out
    }

    fn process(
        &mut self,
        key: i64,
        reg: Registration,
        points: Vec<robotics_core::nalgebra::Vector2<f64>>,
        raw: Option<LaserScan>,
    ) -> Option<NewKeyframe> {
        if self.graph.keyframes().is_empty() {
            // The origin: this scan's increment is relative to a scan we never saw.
            self.graph
                .add_keyframe(points, &Se2::IDENTITY, &Matrix3::identity())
                .ok()?;
            self.stamps.push(key);
            self.kf_scans.push(raw);
            return Some(self.latest(key));
        }
        if self.acc.push(&reg.delta, &reg.information).is_err() {
            self.counters.edge_errors += 1;
            return None;
        }
        if !self.graph.wants_keyframe(&self.acc.delta()) {
            return None;
        }
        let (delta, information) = match self.acc.take() {
            Ok(x) => x,
            Err(_) => {
                self.counters.edge_errors += 1;
                return None;
            }
        };
        match self.graph.add_keyframe(points, &delta, &information) {
            Ok(id) => {
                self.stamps.push(key);
                self.kf_scans.push(raw);
                if self.cfg.loop_closure {
                    self.close_loops(id, key);
                }
                Some(self.latest(key))
            }
            Err(_) => {
                self.counters.edge_errors += 1;
                None
            }
        }
    }

    /// Propose, verify and add the loops of keyframe `id`, optimizing once if any was accepted.
    fn close_loops(&mut self, id: usize, key: i64) {
        match self.graph.process_loops(id) {
            Ok(outcomes) => {
                let accepted = outcomes
                    .iter()
                    .filter(|o| matches!(o.verification, Verification::Accepted { .. }))
                    .count() as u64;
                self.counters.loop_candidates += outcomes.len() as u64;
                self.counters.loop_accepted += accepted;
                self.counters.loop_rejected += outcomes.len() as u64 - accepted;
                if accepted > 0 {
                    self.counters.optimizations += 1;
                    self.pose_epoch += 1;
                }
                if !outcomes.is_empty() {
                    self.loop_events.push(LoopEvent {
                        keyframe: id,
                        stamp_ns: key,
                        outcomes,
                    });
                }
            }
            Err(e) => {
                self.counters.loop_errors += 1;
                self.last_loop_error = e.to_string();
            }
        }
    }

    fn latest(&self, key: i64) -> NewKeyframe {
        let id = self.graph.keyframes().len() - 1;
        NewKeyframe {
            id,
            stamp_ns: key,
            pose: self.graph.poses()[id],
            arc_length: self.graph.keyframes()[id].arc_length,
        }
    }

    /// Keyframes so far.
    pub fn keyframe_count(&self) -> usize {
        self.graph.keyframes().len()
    }

    /// Keyframe poses (in the first keyframe's sensor frame) and their stamps (ns).
    pub fn keyframes(&self) -> (&[Se2], &[i64]) {
        (self.graph.poses(), &self.stamps)
    }

    /// The stored scan of every keyframe, for [`render::render_map`] (`None`: made without one).
    pub fn keyframe_scans(&self) -> Vec<Option<&LaserScan>> {
        self.kf_scans.iter().map(Option::as_ref).collect()
    }

    /// How many optimizations have moved the keyframes already handed out. A consumer that
    /// published poses or a map at epoch `n` must publish them again when this is not `n`.
    pub fn pose_epoch(&self) -> u64 {
        self.pose_epoch
    }

    /// The loop events since the last call (every keyframe that had at least one candidate).
    pub fn take_loop_events(&mut self) -> Vec<LoopEvent> {
        std::mem::take(&mut self.loop_events)
    }

    /// The most recent error the library gave when processing loops (empty if none).
    pub fn last_loop_error(&self) -> &str {
        &self.last_loop_error
    }

    /// Loop edges in the graph.
    pub fn loop_edge_count(&self) -> usize {
        self.graph.loop_edge_count()
    }

    /// The best current estimate of the sensor pose, in the graph's frame, at the scan of the
    /// last processed registration: the newest keyframe's pose composed with the increments
    /// accumulated since it. Stamp in ns. `None` before the first keyframe.
    pub fn current_estimate(&self) -> Option<(i64, Se2)> {
        let stamp = self.last_processed?;
        let kf = *self.graph.poses().last()?;
        Some((stamp, kf.compose(&self.acc.delta())))
    }

    /// The counters.
    pub fn counters(&self) -> BuilderCounters {
        self.counters
    }

    /// Scans and registrations waiting for their other half.
    pub fn waiting(&self) -> (usize, usize) {
        (self.scans.len(), self.regs.len())
    }

    /// Set when the chain of increments cannot be trusted any more, with the reason.
    pub fn broken(&self) -> Option<&str> {
        self.broken.as_deref()
    }

    /// The marginal covariance of the newest keyframe in the tangent at its pose, or `None`.
    pub fn newest_covariance(&self) -> Option<Matrix3<f64>> {
        use robotics_slam::factor_graph::VarKey;
        let n = self.graph.keyframes().len().checked_sub(1)?;
        let c = self
            .graph
            .factor_graph()
            .marginal_covariance(
                self.graph.values(),
                &[VarKey::Pose(n)],
                self.cfg.graph.optimizer.solver,
            )
            .ok()?;
        Some(Matrix3::from_fn(|i, j| c[(i, j)]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robotics_core::nalgebra::Vector2;
    use robotics_core::seeded_rng;
    use robotics_sim::{Lidar, LidarConfig, World};
    use robotics_slam::icp::scan_points;
    use robotics_slam::odometry::{LidarOdometry, OdometryConfig};
    use robotics_slam::voxel::downsample;

    fn room() -> World {
        let mut w = World::empty();
        w.add_rectangle(Vector2::new(-3.0, -3.0), Vector2::new(3.0, 3.0))
            .unwrap();
        w.add_box(Vector2::new(1.0, 1.0), Vector2::new(0.4, 0.25), 0.6)
            .unwrap();
        w.add_box(Vector2::new(-1.5, -0.8), Vector2::new(0.2, 0.5), -0.3)
            .unwrap();
        w
    }

    fn lidar() -> Lidar {
        Lidar::new(
            LidarConfig {
                mount: Se2::IDENTITY,
                range_min: 0.12,
                range_max: 3.5,
                range_sigma: 0.015,
                dropout_probability: 0.0,
                rate_hz: 5.0,
                ..LidarConfig::default()
            },
            3,
        )
        .unwrap()
    }

    /// A short drive: straight, a turn in place, straight.
    fn route() -> Vec<Se2> {
        let mut p = Vec::new();
        for k in 0..30 {
            p.push(Se2::new(-1.0 + 0.036 * k as f64, -1.5, 0.0));
        }
        let end = *p.last().unwrap();
        for k in 1..=10 {
            p.push(Se2::new(end.x(), end.y(), 0.16 * k as f64));
        }
        let end = *p.last().unwrap();
        for k in 1..=30 {
            p.push(Se2::new(end.x(), end.y() + 0.036 * k as f64, end.theta()));
        }
        p
    }

    /// (stamp, points, step result) for every scan of the route.
    fn run() -> Vec<(
        Timestamp,
        Vec<Vector2<f64>>,
        robotics_slam::odometry::StepResult,
    )> {
        let (world, mut lid) = (room(), lidar());
        let mut odo = LidarOdometry::new(OdometryConfig::default()).unwrap();
        route()
            .iter()
            .enumerate()
            .map(|(k, p)| {
                let stamp = Timestamp::from_secs_f64(0.2 * k as f64);
                let scan = lid.scan(&world, p, stamp);
                let step = odo.step(&scan).unwrap();
                (stamp, downsample(&scan_points(&scan), 0.05).unwrap(), step)
            })
            .collect()
    }

    fn reg(stamp: Timestamp, step: &robotics_slam::odometry::StepResult) -> Registration {
        Registration {
            stamp,
            delta: step.delta,
            information: step.information,
        }
    }

    /// The same composition written directly against the library: the reference.
    fn reference(
        run: &[(
            Timestamp,
            Vec<Vector2<f64>>,
            robotics_slam::odometry::StepResult,
        )],
    ) -> Vec<(i64, Se2)> {
        let mut graph = PoseGraph::new(PoseGraphConfig::default()).unwrap();
        let mut acc = EdgeAccumulator::new();
        let mut out = Vec::new();
        for (k, (stamp, pts, step)) in run.iter().enumerate() {
            if k == 0 {
                graph
                    .add_keyframe(pts.clone(), &Se2::IDENTITY, &Matrix3::identity())
                    .unwrap();
                out.push((stamp.as_nanos(), Se2::IDENTITY));
                continue;
            }
            acc.push(&step.delta, &step.information).unwrap();
            if graph.wants_keyframe(&acc.delta()) {
                let (d, i) = acc.take().unwrap();
                let id = graph.add_keyframe(pts.clone(), &d, &i).unwrap();
                out.push((stamp.as_nanos(), graph.poses()[id]));
            }
        }
        out
    }

    fn feed(order: impl Fn(usize) -> bool) -> GraphBuilder {
        let mut b = GraphBuilder::new(GraphBuilderConfig::default()).unwrap();
        for (k, (stamp, pts, step)) in run().into_iter().enumerate() {
            // `order` decides, per scan, whether the registration arrives before its scan.
            if order(k) {
                b.on_registration(reg(stamp, &step));
                b.on_scan(stamp, pts);
            } else {
                b.on_scan(stamp, pts);
                b.on_registration(reg(stamp, &step));
            }
        }
        b
    }

    #[test]
    fn the_builder_makes_the_same_keyframes_as_the_library_composition() {
        let r = run();
        let want = reference(&r);
        assert!(want.len() > 5, "the route should make keyframes");
        let b = feed(|_| false);
        let (poses, stamps) = b.keyframes();
        assert_eq!(poses.len(), want.len());
        for ((p, s), (ws, wp)) in poses.iter().zip(stamps).zip(&want) {
            assert_eq!(s, ws);
            assert!(p.between(wp).log().norm() < 1e-12, "{p:?} {wp:?}");
        }
        assert_eq!(b.counters().processed, r.len() as u64);
    }

    #[test]
    fn the_order_in_which_a_scan_and_its_registration_arrive_does_not_matter() {
        let a = feed(|_| false);
        let b = feed(|_| true);
        let c = feed(|k| k % 3 == 0);
        let bits = |g: &GraphBuilder| {
            g.keyframes()
                .0
                .iter()
                .flat_map(|p| [p.x().to_bits(), p.y().to_bits(), p.theta().to_bits()])
                .collect::<Vec<_>>()
        };
        assert_eq!(bits(&a), bits(&b));
        assert_eq!(bits(&a), bits(&c));
    }

    #[test]
    fn the_first_registration_is_discarded_and_its_scan_is_the_origin() {
        let r = run();
        let mut b = GraphBuilder::new(GraphBuilderConfig::default()).unwrap();
        // Start at the fifth scan: its increment is relative to a scan the builder never saw.
        let (stamp, pts, step) = r[5].clone();
        b.on_scan(stamp, pts);
        let kfs = b.on_registration(reg(stamp, &step));
        assert_eq!(kfs.len(), 1);
        assert_eq!(kfs[0].id, 0);
        assert!(kfs[0].pose.between(&Se2::IDENTITY).log().norm() < 1e-15);
    }

    #[test]
    fn a_registration_that_is_not_newer_than_the_last_is_ignored_and_counted() {
        let r = run();
        let mut b = GraphBuilder::new(GraphBuilderConfig::default()).unwrap();
        for (stamp, pts, step) in r.iter().take(6).cloned() {
            b.on_scan(stamp, pts);
            b.on_registration(reg(stamp, &step));
        }
        let before = b.keyframe_count();
        b.on_registration(reg(r[3].0, &r[3].2));
        assert_eq!(b.counters().out_of_order, 1);
        assert_eq!(b.keyframe_count(), before);
    }

    #[test]
    fn scans_that_will_never_be_joined_are_dropped_not_kept_for_ever() {
        let r = run();
        let mut b = GraphBuilder::new(GraphBuilderConfig::default()).unwrap();
        // The scan matcher skipped scans 1 and 2 (too few points): scans arrive, registrations do not.
        for (k, (stamp, pts, step)) in r.iter().take(8).cloned().enumerate() {
            b.on_scan(stamp, pts);
            if k != 1 && k != 2 {
                b.on_registration(reg(stamp, &step));
            }
        }
        assert_eq!(b.waiting(), (0, 0));
        assert_eq!(b.counters().scans_dropped, 2);
    }

    #[test]
    fn registrations_with_no_scan_break_the_chain_visibly() {
        let r = run();
        let cfg = GraphBuilderConfig {
            max_buffered: 5,
            ..GraphBuilderConfig::default()
        };
        let mut b = GraphBuilder::new(cfg).unwrap();
        for (stamp, _, step) in r.iter().take(12).cloned() {
            b.on_registration(reg(stamp, &step));
        }
        assert!(b.broken().is_some());
        assert_eq!(b.keyframe_count(), 0);
    }

    #[test]
    fn loop_closure_is_unsupported_and_the_graph_has_no_loop_edges() {
        let b = feed(|_| false);
        assert_eq!(b.loop_closure(), "unsupported");
        assert_eq!(LOOP_CLOSURE_STATUS, "unsupported");
        assert_eq!(b.graph.loop_edge_count(), 0);
        // Odometry edges and the gauge prior only: one prior plus one edge per later keyframe.
        assert_eq!(b.graph.factor_graph().factors().len(), b.keyframe_count());
    }

    /// A pillar room and two laps of a circle that comes back to where it started: the route with
    /// loops in it, from which a builder with loop closure on must make loop edges.
    fn loop_run() -> Vec<(
        Timestamp,
        LaserScan,
        Vec<Vector2<f64>>,
        robotics_slam::odometry::StepResult,
    )> {
        let mut world = World::empty();
        world
            .add_rectangle(Vector2::new(-3.0, -3.0), Vector2::new(3.0, 3.0))
            .unwrap();
        for (cx, cy) in [(1.8, 1.2), (-1.4, 2.0), (-2.0, -1.0), (0.9, -2.1)] {
            let ring: Vec<_> = (0..12)
                .map(|k| {
                    let a = std::f64::consts::TAU * k as f64 / 12.0;
                    Vector2::new(cx + 0.15 * a.cos(), cy + 0.15 * a.sin())
                })
                .collect();
            world.add_polyline(&ring, true).unwrap();
        }
        let mut lid = lidar();
        let mut odo = LidarOdometry::new(OdometryConfig::default()).unwrap();
        let (radius, step) = (1.4_f64, 0.06_f64);
        let n = (std::f64::consts::TAU * radius * 2.0 / step).round() as usize;
        (0..n)
            .map(|k| {
                let a = k as f64 * step / radius;
                let pose = Se2::new(
                    radius * a.cos(),
                    radius * a.sin(),
                    a + std::f64::consts::FRAC_PI_2,
                );
                let stamp = Timestamp::from_secs_f64(0.2 * k as f64);
                let scan = lid.scan(&world, &pose, stamp);
                let st = odo.step(&scan).unwrap();
                let pts = downsample(&scan_points(&scan), 0.05).unwrap();
                (stamp, scan, pts, st)
            })
            .collect()
    }

    fn feed_loops(loop_closure: bool) -> GraphBuilder {
        let cfg = GraphBuilderConfig {
            loop_closure,
            ..GraphBuilderConfig::default()
        };
        let mut b = GraphBuilder::new(cfg).unwrap();
        for (stamp, scan, pts, step) in loop_run() {
            b.on_scan_with_raw(stamp, pts, scan);
            b.on_registration(reg(stamp, &step));
        }
        b
    }

    #[test]
    fn with_loop_closure_off_nothing_is_proposed_and_the_status_stays_unsupported() {
        let b = feed_loops(false);
        // 2 laps of a 1.4 m circle is 17.6 m of path, a keyframe every 0.5 m.
        assert!(b.keyframe_count() > 25, "{} keyframes", b.keyframe_count());
        assert_eq!(b.loop_closure(), "unsupported");
        let c = b.counters();
        assert_eq!(
            (
                c.loop_candidates,
                c.loop_accepted,
                c.optimizations,
                c.loop_errors
            ),
            (0, 0, 0, 0)
        );
        assert_eq!(b.loop_edge_count(), 0);
        assert_eq!(b.pose_epoch(), 0);
    }

    #[test]
    fn with_loop_closure_on_the_revisited_places_become_loop_edges_and_the_graph_is_optimized() {
        let mut b = feed_loops(true);
        assert_eq!(b.loop_closure(), "enabled");
        let c = b.counters();
        println!("keyframes {}, {c:?}", b.keyframe_count());
        assert!(
            c.loop_candidates > 0,
            "the second lap proposed nothing: {c:?}"
        );
        assert!(c.loop_accepted > 0, "no loop was accepted: {c:?}");
        assert_eq!(c.loop_accepted + c.loop_rejected, c.loop_candidates);
        assert_eq!(b.loop_edge_count() as u64, c.loop_accepted);
        assert!(c.optimizations > 0 && c.optimizations <= c.loop_accepted);
        assert_eq!(
            b.pose_epoch(),
            c.optimizations,
            "the epoch counts optimizations"
        );
        assert_eq!(c.loop_errors, 0, "{}", b.last_loop_error());
        let events = b.take_loop_events();
        assert!(!events.is_empty());
        assert_eq!(
            events.iter().map(|e| e.accepted() as u64).sum::<u64>(),
            c.loop_accepted
        );
        assert!(
            b.take_loop_events().is_empty(),
            "events are handed out once"
        );
        // No loop can connect two keyframes closer than the detector's path gap.
        let arcs: Vec<f64> = b.graph.keyframes().iter().map(|k| k.arc_length).collect();
        for e in &events {
            for o in &e.outcomes {
                assert!(arcs[o.candidate.to] - arcs[o.candidate.from] >= 6.0);
            }
        }
    }

    #[test]
    fn the_first_loop_closure_moves_keyframes_that_had_already_been_published() {
        // The reason a consumer has to republish: the path it sent is not the path any more.
        let cfg = GraphBuilderConfig {
            loop_closure: true,
            ..GraphBuilderConfig::default()
        };
        let mut b = GraphBuilder::new(cfg).unwrap();
        let mut published: Vec<Se2> = Vec::new();
        let mut moved = false;
        for (stamp, scan, pts, step) in loop_run() {
            b.on_scan_with_raw(stamp, pts, scan);
            let epoch = b.pose_epoch();
            for kf in b.on_registration(reg(stamp, &step)) {
                assert_eq!(published.len(), kf.id);
                published.push(kf.pose);
            }
            if b.pose_epoch() != epoch && !moved {
                moved = published
                    .iter()
                    .zip(b.keyframes().0)
                    .take(published.len() - 1)
                    .any(|(old, new)| old.between(new).log().norm() > 1e-9);
            }
        }
        assert!(moved, "an optimization changed no earlier keyframe");
    }

    #[test]
    fn keyframes_made_from_raw_scans_keep_them_and_the_others_do_not() {
        let b = feed_loops(false);
        let scans = b.keyframe_scans();
        assert_eq!(scans.len(), b.keyframe_count());
        assert!(scans.iter().all(Option::is_some));
        let plain = feed(|_| false);
        assert!(plain.keyframe_scans().iter().all(Option::is_none));
    }

    #[test]
    fn the_current_estimate_is_the_newest_keyframe_composed_with_what_followed_it() {
        let r = run();
        let mut b = GraphBuilder::new(GraphBuilderConfig::default()).unwrap();
        assert!(b.current_estimate().is_none());
        let mut last_kf = Se2::IDENTITY;
        let mut seen_between = false;
        for (stamp, pts, step) in r.iter().cloned() {
            b.on_scan(stamp, pts);
            let kfs = b.on_registration(reg(stamp, &step));
            let (key, est) = b.current_estimate().expect("a registration was processed");
            assert_eq!(key, stamp.as_nanos());
            if let Some(kf) = kfs.last() {
                last_kf = kf.pose;
                assert!(
                    est.between(&last_kf).log().norm() < 1e-12,
                    "at a keyframe the estimate is the keyframe"
                );
            } else {
                seen_between = true;
                assert!(
                    last_kf.between(&est).log().norm() > 0.0,
                    "between keyframes it has moved on"
                );
            }
        }
        assert!(seen_between);
    }

    #[test]
    fn a_registration_survives_the_message_round_trip() {
        let (stamp, _, step) = run().remove(7);
        let m = registration_to_ros(
            stamp,
            &FrameId::new("laser"),
            &step.delta,
            &step.information,
        )
        .unwrap();
        assert_eq!(m.header.frame_id, "laser");
        let back = registration_from_ros(&m).unwrap();
        assert_eq!(back.stamp, stamp);
        assert!(back.delta.between(&step.delta).log().norm() < 1e-12);
        let rel = (back.information - step.information).abs().max() / step.information.abs().max();
        assert!(rel < 1e-9, "relative information error {rel}");
        // The message's covariance is the inverse of the information, in the (x, y, yaw) slots.
        let cov = step.information.try_inverse().unwrap();
        assert!((m.pose.covariance[0] - cov[(0, 0)]).abs() < 1e-15);
        assert!((m.pose.covariance[5 * 6 + 5] - cov[(2, 2)]).abs() < 1e-15);
    }

    #[test]
    fn a_singular_information_is_refused_not_published() {
        let r = registration_to_ros(
            Timestamp::ZERO,
            &FrameId::new("laser"),
            &Se2::IDENTITY,
            &Matrix3::zeros(),
        );
        assert!(matches!(r, Err(PlumbingError::BadCovariance)));
        let _ = seeded_rng(0);
    }
}
