//! A ROS-independent planar transform buffer: the data model behind a TF listener.
//!
//! `rclrs` 0.8 has no `tf2` binding, so the framework keeps its own buffer. ROS adapters
//! (`robotics_ros_bridge::tf::TfListener`) convert `/tf` and `/tf_static` messages into
//! [`TransformStamped2`] and insert them here; algorithms and nodes call
//! [`TransformBuffer::lookup_transform`].
//!
//! # Semantics (the subset of `tf2::BufferCore` Phase 2 needs)
//!
//! * The frames form a **forest**: every child frame has exactly one parent. An edge is
//!   either **static** (one value, valid at all times) or **dynamic** (a time-ordered
//!   history).
//! * `lookup_transform(target, source, time)` returns `T_target_source`, the pose of
//!   `source` expressed in `target` (it maps points from `source` into `target`), exactly
//!   like `tf2::BufferCore::lookupTransform(target, source, time)`.
//! * Time: [`LookupTime::Latest`] resolves to the latest instant at which **every**
//!   dynamic edge on the path has data (tf2's "time 0"); [`LookupTime::At`] interpolates
//!   each dynamic edge on the SE(2) geodesic between the bracketing samples. Requests
//!   outside an edge's history (beyond `extrapolation_tolerance`) fail with
//!   [`TfError::Extrapolation`]; nothing is extrapolated silently.
//! * Expiry: each dynamic edge keeps `cache_duration` seconds of history behind its
//!   newest sample (tf2 default: 10 s); [`TransformBuffer::prune_before`] drops older
//!   samples against an external clock.
//! * Cycles are rejected at insertion; lookups additionally stop after `MAX_DEPTH` hops.
//! * A child published both as static and as dynamic is rejected
//!   ([`TfError::StaticDynamicConflict`]); a dynamic child that changes parent is
//!   re-parented and its history is reset (reported as [`InsertOutcome::Reparented`]).
//!
//! # Limitations
//! Planar only (`Se2`): z, roll and pitch are not represented. The ROS adapter rejects
//! transforms with non-negligible roll/pitch rather than projecting them silently. No
//! lookups "through time" (`lookupTransform` with a fixed frame) and no waiting/timeouts:
//! callers poll [`TransformBuffer::can_transform`].

use crate::frame::FrameId;
use crate::se2::{Manifold, Se2};
use crate::time::Timestamp;
use std::collections::{HashMap, VecDeque};

/// Hard limit on the length of a frame chain (guards against corrupted state).
const MAX_DEPTH: usize = 256;

/// A planar transform `parent → child` at `stamp`: `transform` is the pose of `child`
/// expressed in `parent` (`T_parent_child`).
#[derive(Debug, Clone, PartialEq)]
pub struct TransformStamped2 {
    /// Time the transform is valid for (ignored for static edges).
    pub stamp: Timestamp,
    /// Parent frame (`header.frame_id` in ROS).
    pub parent: FrameId,
    /// Child frame (`child_frame_id` in ROS).
    pub child: FrameId,
    /// `T_parent_child`.
    pub transform: Se2,
}

impl TransformStamped2 {
    /// Convenience constructor.
    pub fn new(
        stamp: Timestamp,
        parent: impl Into<FrameId>,
        child: impl Into<FrameId>,
        transform: Se2,
    ) -> Self {
        Self {
            stamp,
            parent: parent.into(),
            child: child.into(),
            transform,
        }
    }
}

/// When a lookup should be evaluated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LookupTime {
    /// The latest time at which the whole chain is available (tf2's `Time(0)`).
    Latest,
    /// A specific instant.
    At(Timestamp),
}

/// Errors reported by the buffer.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum TfError {
    /// A frame id was empty.
    #[error("empty frame id")]
    EmptyFrameId,
    /// A transform from a frame to itself was inserted.
    #[error("transform from frame '{0}' to itself")]
    SelfTransform(FrameId),
    /// The transform contained NaN or infinity.
    #[error("non-finite transform {parent} -> {child}")]
    NonFinite {
        /// Parent frame.
        parent: FrameId,
        /// Child frame.
        child: FrameId,
    },
    /// The frame has never been seen.
    #[error("unknown frame '{0}'")]
    UnknownFrame(FrameId),
    /// Both frames exist but belong to different trees.
    #[error("frames '{target_frame}' and '{source_frame}' are not connected")]
    NotConnected {
        /// Target frame of the lookup.
        target_frame: FrameId,
        /// Source frame of the lookup.
        source_frame: FrameId,
    },
    /// The requested time is outside the stored history of one edge.
    #[error("lookup of {parent} -> {child} at {requested:?} outside its history [{earliest:?}, {latest:?}]")]
    Extrapolation {
        /// Parent of the offending edge.
        parent: FrameId,
        /// Child of the offending edge.
        child: FrameId,
        /// Requested time.
        requested: Timestamp,
        /// Oldest stored sample.
        earliest: Timestamp,
        /// Newest stored sample.
        latest: Timestamp,
    },
    /// Inserting the edge would close a loop in the frame graph.
    #[error("inserting {parent} -> {child} would create a cycle")]
    Cycle {
        /// Parent frame of the rejected edge.
        parent: FrameId,
        /// Child frame of the rejected edge.
        child: FrameId,
    },
    /// The same child was published as both static and dynamic.
    #[error("frame '{child}' is published both as static and as dynamic")]
    StaticDynamicConflict {
        /// The contested child frame.
        child: FrameId,
    },
    /// A buffer parameter was invalid.
    #[error("invalid buffer parameter {what} = {value}")]
    InvalidParameter {
        /// Parameter name.
        what: &'static str,
        /// Offending value.
        value: f64,
    },
    /// The frame chain exceeded the depth guard (corrupted state).
    #[error("frame chain longer than {0} hops")]
    ChainTooLong(usize),
}

/// What an insertion did.
#[derive(Debug, Clone, PartialEq)]
pub enum InsertOutcome {
    /// A new edge was created.
    Inserted,
    /// An existing edge received a new value / sample.
    Updated,
    /// The child moved to a different parent; its history was reset.
    Reparented {
        /// The parent it had before.
        old_parent: FrameId,
    },
}

/// Buffer configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransformBufferConfig {
    /// History kept behind the newest sample of each dynamic edge (s).
    pub cache_duration: f64,
    /// How far outside an edge's history a lookup may reach and still be answered with
    /// the nearest sample (s). `0` = never extrapolate (tf2 behaviour).
    pub extrapolation_tolerance: f64,
}

impl Default for TransformBufferConfig {
    fn default() -> Self {
        Self {
            cache_duration: 10.0,
            extrapolation_tolerance: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
enum EdgeData {
    Static { stamp: Timestamp, transform: Se2 },
    Dynamic(VecDeque<(Timestamp, Se2)>),
}

#[derive(Debug, Clone)]
struct Edge {
    parent: FrameId,
    data: EdgeData,
}

impl Edge {
    fn newest(&self) -> Option<Timestamp> {
        match &self.data {
            EdgeData::Static { .. } => None,
            EdgeData::Dynamic(h) => h.back().map(|s| s.0),
        }
    }
}

/// The planar transform buffer. See the module documentation for semantics.
#[derive(Debug, Clone, Default)]
pub struct TransformBuffer {
    cfg: TransformBufferConfig,
    edges: HashMap<FrameId, Edge>,
}

impl TransformBuffer {
    /// A buffer with the given configuration.
    pub fn new(cfg: TransformBufferConfig) -> Result<Self, TfError> {
        for (what, value) in [
            ("cache_duration", cfg.cache_duration),
            ("extrapolation_tolerance", cfg.extrapolation_tolerance),
        ] {
            if !(value.is_finite() && value >= 0.0) {
                return Err(TfError::InvalidParameter { what, value });
            }
        }
        Ok(Self {
            cfg,
            edges: HashMap::new(),
        })
    }

    /// Configuration.
    pub fn config(&self) -> &TransformBufferConfig {
        &self.cfg
    }

    /// Insert a transform. `is_static` selects the static (`/tf_static`) or dynamic
    /// (`/tf`) store for its child frame.
    pub fn set_transform(
        &mut self,
        t: &TransformStamped2,
        is_static: bool,
    ) -> Result<InsertOutcome, TfError> {
        if t.parent.is_empty() || t.child.is_empty() {
            return Err(TfError::EmptyFrameId);
        }
        if t.parent == t.child {
            return Err(TfError::SelfTransform(t.child.clone()));
        }
        if !t.transform.is_finite() {
            return Err(TfError::NonFinite {
                parent: t.parent.clone(),
                child: t.child.clone(),
            });
        }
        // Would `child` become its own ancestor? Walk up from the new parent.
        let mut cursor = Some(t.parent.clone());
        let mut hops = 0;
        while let Some(f) = cursor {
            if f == t.child {
                return Err(TfError::Cycle {
                    parent: t.parent.clone(),
                    child: t.child.clone(),
                });
            }
            hops += 1;
            if hops > MAX_DEPTH {
                return Err(TfError::ChainTooLong(MAX_DEPTH));
            }
            cursor = self.edges.get(&f).map(|e| e.parent.clone());
        }

        let cache = self.cfg.cache_duration;
        match self.edges.get_mut(&t.child) {
            None => {
                let data = if is_static {
                    EdgeData::Static {
                        stamp: t.stamp,
                        transform: t.transform,
                    }
                } else {
                    EdgeData::Dynamic(VecDeque::from([(t.stamp, t.transform)]))
                };
                self.edges.insert(
                    t.child.clone(),
                    Edge {
                        parent: t.parent.clone(),
                        data,
                    },
                );
                Ok(InsertOutcome::Inserted)
            }
            Some(edge) => {
                let was_static = matches!(edge.data, EdgeData::Static { .. });
                if was_static != is_static {
                    return Err(TfError::StaticDynamicConflict {
                        child: t.child.clone(),
                    });
                }
                let reparented = edge.parent != t.parent;
                let old_parent = std::mem::replace(&mut edge.parent, t.parent.clone());
                match &mut edge.data {
                    EdgeData::Static { stamp, transform } => {
                        *stamp = t.stamp;
                        *transform = t.transform;
                    }
                    EdgeData::Dynamic(history) => {
                        if reparented {
                            history.clear();
                        }
                        insert_sorted(history, t.stamp, t.transform);
                        if let Some(&(newest, _)) = history.back() {
                            let horizon = newest.plus_secs_f64(-cache);
                            while history.len() > 1
                                && history.front().is_some_and(|s| s.0 < horizon)
                            {
                                history.pop_front();
                            }
                        }
                    }
                }
                Ok(if reparented {
                    InsertOutcome::Reparented { old_parent }
                } else {
                    InsertOutcome::Updated
                })
            }
        }
    }

    /// Drop dynamic samples older than `t` (e.g. `now − cache_duration` on a wall clock).
    /// Edges whose whole history expires are removed; static edges are never touched.
    pub fn prune_before(&mut self, t: Timestamp) {
        self.edges.retain(|_, e| match &mut e.data {
            EdgeData::Static { .. } => true,
            EdgeData::Dynamic(h) => {
                while h.front().is_some_and(|s| s.0 < t) {
                    h.pop_front();
                }
                !h.is_empty()
            }
        });
    }

    /// Remove every dynamic edge (e.g. after a simulation-time jump backwards).
    pub fn clear_dynamic(&mut self) {
        self.edges
            .retain(|_, e| matches!(e.data, EdgeData::Static { .. }));
    }

    /// Whether the frame appears in the buffer (as a child or as a parent).
    pub fn has_frame(&self, frame: &FrameId) -> bool {
        self.edges.contains_key(frame) || self.edges.values().any(|e| &e.parent == frame)
    }

    /// All known frames, sorted.
    pub fn frames(&self) -> Vec<FrameId> {
        let mut out: Vec<FrameId> = self
            .edges
            .iter()
            .flat_map(|(c, e)| [c.clone(), e.parent.clone()])
            .collect();
        out.sort();
        out.dedup();
        out
    }

    /// Parent of `child`, if it has one.
    pub fn parent(&self, child: &FrameId) -> Option<&FrameId> {
        self.edges.get(child).map(|e| &e.parent)
    }

    /// Whether `child`'s edge is static (`None` if the child is unknown).
    pub fn is_static(&self, child: &FrameId) -> Option<bool> {
        self.edges
            .get(child)
            .map(|e| matches!(e.data, EdgeData::Static { .. }))
    }

    /// `(oldest, newest)` stamps of a dynamic edge.
    pub fn time_range(&self, child: &FrameId) -> Option<(Timestamp, Timestamp)> {
        match &self.edges.get(child)?.data {
            EdgeData::Dynamic(h) => Some((h.front()?.0, h.back()?.0)),
            EdgeData::Static { .. } => None,
        }
    }

    /// Number of stored samples of a dynamic edge (1 for a static edge).
    pub fn sample_count(&self, child: &FrameId) -> usize {
        match self.edges.get(child).map(|e| &e.data) {
            Some(EdgeData::Dynamic(h)) => h.len(),
            Some(EdgeData::Static { .. }) => 1,
            None => 0,
        }
    }

    /// Chain `[frame, parent, grandparent, …, root]`.
    fn ancestors(&self, frame: &FrameId) -> Result<Vec<FrameId>, TfError> {
        let mut chain = vec![frame.clone()];
        let mut cursor = frame;
        while let Some(e) = self.edges.get(cursor) {
            if chain.len() > MAX_DEPTH {
                return Err(TfError::ChainTooLong(MAX_DEPTH));
            }
            chain.push(e.parent.clone());
            cursor = &e.parent;
        }
        Ok(chain)
    }

    /// Evaluate the edge `parent(child) → child` at `t`.
    fn edge_at(&self, child: &FrameId, t: Option<Timestamp>) -> Result<Se2, TfError> {
        let edge = self
            .edges
            .get(child)
            .ok_or_else(|| TfError::UnknownFrame(child.clone()))?;
        match &edge.data {
            EdgeData::Static { transform, .. } => Ok(*transform),
            EdgeData::Dynamic(h) => {
                let (earliest, latest) = match (h.front(), h.back()) {
                    (Some(a), Some(b)) => (a.0, b.0),
                    _ => return Err(TfError::UnknownFrame(child.clone())),
                };
                let t = t.unwrap_or(latest);
                let tol = self.cfg.extrapolation_tolerance;
                if t < earliest {
                    if earliest - t <= tol {
                        return Ok(h[0].1);
                    }
                } else if t > latest {
                    if t - latest <= tol {
                        return Ok(h[h.len() - 1].1);
                    }
                } else {
                    // First sample with stamp >= t.
                    let i = h.partition_point(|s| s.0 < t);
                    let (tb, b) = h[i];
                    if tb == t || i == 0 {
                        return Ok(b);
                    }
                    let (ta, a) = h[i - 1];
                    let s = (t - ta) / (tb - ta);
                    return Ok(a.boxplus(&(b.boxminus(&a) * s)));
                }
                Err(TfError::Extrapolation {
                    parent: edge.parent.clone(),
                    child: child.clone(),
                    requested: t,
                    earliest,
                    latest,
                })
            }
        }
    }

    /// `T_target_source` at `time`. See the module documentation.
    pub fn lookup_transform(
        &self,
        target: &FrameId,
        source: &FrameId,
        time: LookupTime,
    ) -> Result<TransformStamped2, TfError> {
        if target.is_empty() || source.is_empty() {
            return Err(TfError::EmptyFrameId);
        }
        for f in [target, source] {
            if !self.has_frame(f) {
                return Err(TfError::UnknownFrame(f.clone()));
            }
        }
        let requested = match time {
            LookupTime::At(t) => Some(t),
            LookupTime::Latest => None,
        };
        if target == source {
            return Ok(TransformStamped2 {
                stamp: requested.unwrap_or(Timestamp::ZERO),
                parent: target.clone(),
                child: source.clone(),
                transform: Se2::IDENTITY,
            });
        }
        let up_source = self.ancestors(source)?;
        let up_target = self.ancestors(target)?;
        // Lowest common ancestor: first frame of the source chain present in the target's.
        let (i_s, i_t) = up_source
            .iter()
            .enumerate()
            .find_map(|(i, f)| up_target.iter().position(|g| g == f).map(|j| (i, j)))
            .ok_or_else(|| TfError::NotConnected {
                target_frame: target.clone(),
                source_frame: source.clone(),
            })?;
        // Edges on the path are identified by their child frames.
        let path_children: Vec<&FrameId> = up_source[..i_s]
            .iter()
            .chain(up_target[..i_t].iter())
            .collect();
        let at = match requested {
            Some(t) => Some(t),
            None => path_children
                .iter()
                .filter_map(|c| self.edges.get(*c).and_then(Edge::newest))
                .min(),
        };
        // T_lca_source = T(lca→…)…T(parent(source)→source), composed from the top down.
        let mut t_lca_source = Se2::IDENTITY;
        for child in up_source[..i_s].iter().rev() {
            t_lca_source = t_lca_source.compose(&self.edge_at(child, at)?);
        }
        let mut t_lca_target = Se2::IDENTITY;
        for child in up_target[..i_t].iter().rev() {
            t_lca_target = t_lca_target.compose(&self.edge_at(child, at)?);
        }
        Ok(TransformStamped2 {
            stamp: at.unwrap_or(Timestamp::ZERO),
            parent: target.clone(),
            child: source.clone(),
            transform: t_lca_target.inverse().compose(&t_lca_source),
        })
    }

    /// Whether [`TransformBuffer::lookup_transform`] would succeed.
    pub fn can_transform(&self, target: &FrameId, source: &FrameId, time: LookupTime) -> bool {
        self.lookup_transform(target, source, time).is_ok()
    }
}

fn insert_sorted(h: &mut VecDeque<(Timestamp, Se2)>, stamp: Timestamp, value: Se2) {
    match h.back() {
        Some(&(last, _)) if stamp > last => h.push_back((stamp, value)),
        None => h.push_back((stamp, value)),
        _ => {
            let i = h.partition_point(|s| s.0 < stamp);
            if i < h.len() && h[i].0 == stamp {
                h[i].1 = value;
            } else {
                h.insert(i, (stamp, value));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn ts(s: f64) -> Timestamp {
        Timestamp::from_secs_f64(100.0 + s)
    }
    fn f(name: &str) -> FrameId {
        FrameId::new(name)
    }
    fn close(a: &Se2, b: &Se2) -> bool {
        a.boxminus(b).norm() < 1e-9
    }
    fn tf(t: f64, p: &str, c: &str, x: f64, y: f64, th: f64) -> TransformStamped2 {
        TransformStamped2::new(ts(t), p, c, Se2::new(x, y, th))
    }

    #[test]
    fn direct_and_inverse_lookup() {
        let mut b = TransformBuffer::default();
        b.set_transform(&tf(0.0, "odom", "base_link", 1.0, 2.0, FRAC_PI_2), false)
            .unwrap();
        let t = b
            .lookup_transform(&f("odom"), &f("base_link"), LookupTime::Latest)
            .unwrap();
        assert!(close(&t.transform, &Se2::new(1.0, 2.0, FRAC_PI_2)));
        assert_eq!(t.stamp, ts(0.0));
        assert_eq!(t.parent, f("odom"));
        assert_eq!(t.child, f("base_link"));
        let inv = b
            .lookup_transform(&f("base_link"), &f("odom"), LookupTime::Latest)
            .unwrap();
        assert!(close(
            &inv.transform,
            &Se2::new(1.0, 2.0, FRAC_PI_2).inverse()
        ));
        let same = b
            .lookup_transform(&f("odom"), &f("odom"), LookupTime::Latest)
            .unwrap();
        assert!(close(&same.transform, &Se2::IDENTITY));
    }

    #[test]
    fn two_hop_and_longer_chains() {
        let mut b = TransformBuffer::default();
        b.set_transform(&tf(0.0, "map", "odom", 10.0, 0.0, 0.0), false)
            .unwrap();
        b.set_transform(&tf(0.0, "odom", "base_link", 1.0, 0.0, FRAC_PI_2), false)
            .unwrap();
        b.set_transform(&tf(0.0, "base_link", "laser", 0.1, 0.0, 0.0), true)
            .unwrap();
        b.set_transform(&tf(0.0, "base_link", "imu_link", 0.0, -0.05, 0.0), true)
            .unwrap();
        // Two hops: odom -> laser.
        let t = b
            .lookup_transform(&f("odom"), &f("laser"), LookupTime::Latest)
            .unwrap();
        assert!(close(&t.transform, &Se2::new(1.0, 0.1, FRAC_PI_2)));
        // Three hops: map -> laser.
        let t = b
            .lookup_transform(&f("map"), &f("laser"), LookupTime::Latest)
            .unwrap();
        assert!(close(&t.transform, &Se2::new(11.0, 0.1, FRAC_PI_2)));
        // Sibling frames through their common parent: laser -> imu_link.
        let t = b
            .lookup_transform(&f("laser"), &f("imu_link"), LookupTime::Latest)
            .unwrap();
        assert!(close(&t.transform, &Se2::new(-0.1, -0.05, 0.0)));
        // Downward chain (target is a descendant of the source).
        let t = b
            .lookup_transform(&f("laser"), &f("map"), LookupTime::Latest)
            .unwrap();
        assert!(close(
            &t.transform,
            &Se2::new(11.0, 0.1, FRAC_PI_2).inverse()
        ));
    }

    #[test]
    fn static_edges_are_timeless() {
        let mut b = TransformBuffer::default();
        b.set_transform(&tf(0.0, "base_link", "laser", 0.2, 0.0, 0.0), true)
            .unwrap();
        for t in [
            LookupTime::Latest,
            LookupTime::At(ts(-50.0)),
            LookupTime::At(ts(1e4)),
        ] {
            let r = b.lookup_transform(&f("base_link"), &f("laser"), t).unwrap();
            assert!(close(&r.transform, &Se2::new(0.2, 0.0, 0.0)));
        }
        assert_eq!(b.is_static(&f("laser")), Some(true));
        // Mixing static and dynamic publication of one child is a configuration error.
        assert!(matches!(
            b.set_transform(&tf(1.0, "base_link", "laser", 0.0, 0.0, 0.0), false),
            Err(TfError::StaticDynamicConflict { .. })
        ));
        // Updating a static edge replaces it.
        assert_eq!(
            b.set_transform(&tf(2.0, "base_link", "laser", 0.3, 0.0, 0.0), true),
            Ok(InsertOutcome::Updated)
        );
        let r = b
            .lookup_transform(&f("base_link"), &f("laser"), LookupTime::Latest)
            .unwrap();
        assert!(close(&r.transform, &Se2::new(0.3, 0.0, 0.0)));
    }

    #[test]
    fn dynamic_updates_interpolate_on_se2() {
        let mut b = TransformBuffer::default();
        b.set_transform(&tf(0.0, "odom", "base_link", 0.0, 0.0, 3.0), false)
            .unwrap();
        assert_eq!(
            b.set_transform(&tf(1.0, "odom", "base_link", 1.0, 0.0, -3.0), false),
            Ok(InsertOutcome::Updated)
        );
        // Latest returns the newest sample.
        let r = b
            .lookup_transform(&f("odom"), &f("base_link"), LookupTime::Latest)
            .unwrap();
        assert_eq!(r.stamp, ts(1.0));
        assert!(close(&r.transform, &Se2::new(1.0, 0.0, -3.0)));
        // Midpoint: heading interpolates across ±π (3.0 → −3.0 is 0.283 rad, not 6 rad).
        let m = b
            .lookup_transform(&f("odom"), &f("base_link"), LookupTime::At(ts(0.5)))
            .unwrap();
        assert!((m.transform.theta().abs() - std::f64::consts::PI).abs() < 1e-9);
        // Out-of-order insertion is sorted in.
        b.set_transform(&tf(0.25, "odom", "base_link", 5.0, 5.0, 0.0), false)
            .unwrap();
        assert_eq!(b.sample_count(&f("base_link")), 3);
        let q = b
            .lookup_transform(&f("odom"), &f("base_link"), LookupTime::At(ts(0.25)))
            .unwrap();
        assert!(close(&q.transform, &Se2::new(5.0, 5.0, 0.0)));
        // Extrapolation is refused.
        assert!(matches!(
            b.lookup_transform(&f("odom"), &f("base_link"), LookupTime::At(ts(1.5))),
            Err(TfError::Extrapolation { .. })
        ));
        assert!(matches!(
            b.lookup_transform(&f("odom"), &f("base_link"), LookupTime::At(ts(-0.5))),
            Err(TfError::Extrapolation { .. })
        ));
    }

    #[test]
    fn latest_uses_the_common_time_of_the_chain() {
        let mut b = TransformBuffer::default();
        for (t, x) in [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)] {
            b.set_transform(&tf(t, "odom", "base_link", x, 0.0, 0.0), false)
                .unwrap();
        }
        b.set_transform(&tf(0.0, "map", "odom", 0.0, 10.0, 0.0), false)
            .unwrap();
        b.set_transform(&tf(1.0, "map", "odom", 0.0, 20.0, 0.0), false)
            .unwrap();
        let r = b
            .lookup_transform(&f("map"), &f("base_link"), LookupTime::Latest)
            .unwrap();
        assert_eq!(r.stamp, ts(1.0));
        assert!(close(&r.transform, &Se2::new(1.0, 20.0, 0.0)));
    }

    #[test]
    fn missing_frames_and_disconnected_trees() {
        let mut b = TransformBuffer::default();
        b.set_transform(&tf(0.0, "odom", "base_link", 0.0, 0.0, 0.0), false)
            .unwrap();
        b.set_transform(&tf(0.0, "world", "marker", 0.0, 0.0, 0.0), true)
            .unwrap();
        assert_eq!(
            b.lookup_transform(&f("odom"), &f("camera"), LookupTime::Latest),
            Err(TfError::UnknownFrame(f("camera")))
        );
        assert!(matches!(
            b.lookup_transform(&f("odom"), &f("marker"), LookupTime::Latest),
            Err(TfError::NotConnected { .. })
        ));
        assert!(!b.can_transform(&f("base_link"), &f("world"), LookupTime::Latest));
        assert_eq!(
            b.frames(),
            vec![f("base_link"), f("marker"), f("odom"), f("world")]
        );
    }

    #[test]
    fn cycles_and_invalid_input_are_rejected() {
        let mut b = TransformBuffer::default();
        b.set_transform(&tf(0.0, "a", "b", 1.0, 0.0, 0.0), false)
            .unwrap();
        b.set_transform(&tf(0.0, "b", "c", 1.0, 0.0, 0.0), false)
            .unwrap();
        assert!(matches!(
            b.set_transform(&tf(0.0, "c", "a", 1.0, 0.0, 0.0), false),
            Err(TfError::Cycle { .. })
        ));
        // Re-parenting "b" under "c" would also close a loop (c is b's descendant).
        assert!(matches!(
            b.set_transform(&tf(0.0, "c", "b", 1.0, 0.0, 0.0), false),
            Err(TfError::Cycle { .. })
        ));
        assert!(matches!(
            b.set_transform(&tf(0.0, "a", "a", 0.0, 0.0, 0.0), false),
            Err(TfError::SelfTransform(_))
        ));
        assert_eq!(
            b.set_transform(&tf(0.0, "", "x", 0.0, 0.0, 0.0), false),
            Err(TfError::EmptyFrameId)
        );
        assert!(matches!(
            b.set_transform(&tf(0.0, "a", "x", f64::NAN, 0.0, 0.0), false),
            Err(TfError::NonFinite { .. })
        ));
        // The rejected inserts left the tree intact.
        let t = b
            .lookup_transform(&f("a"), &f("c"), LookupTime::Latest)
            .unwrap();
        assert!(close(&t.transform, &Se2::new(2.0, 0.0, 0.0)));
    }

    #[test]
    fn reparenting_resets_history() {
        let mut b = TransformBuffer::default();
        b.set_transform(&tf(0.0, "odom", "base_link", 1.0, 0.0, 0.0), false)
            .unwrap();
        let out = b
            .set_transform(&tf(1.0, "map", "base_link", 2.0, 0.0, 0.0), false)
            .unwrap();
        assert_eq!(
            out,
            InsertOutcome::Reparented {
                old_parent: f("odom")
            }
        );
        assert_eq!(b.parent(&f("base_link")), Some(&f("map")));
        assert_eq!(b.sample_count(&f("base_link")), 1);
    }

    #[test]
    fn expired_dynamic_data_is_dropped() {
        let mut b = TransformBuffer::new(TransformBufferConfig {
            cache_duration: 10.0,
            extrapolation_tolerance: 0.0,
        })
        .unwrap();
        b.set_transform(&tf(0.0, "odom", "base_link", 0.0, 0.0, 0.0), false)
            .unwrap();
        b.set_transform(&tf(5.0, "odom", "base_link", 1.0, 0.0, 0.0), false)
            .unwrap();
        b.set_transform(&tf(20.0, "odom", "base_link", 2.0, 0.0, 0.0), false)
            .unwrap();
        // Samples older than newest − 10 s were evicted on insertion.
        assert_eq!(b.time_range(&f("base_link")), Some((ts(20.0), ts(20.0))));
        assert!(matches!(
            b.lookup_transform(&f("odom"), &f("base_link"), LookupTime::At(ts(0.0))),
            Err(TfError::Extrapolation { .. })
        ));
        // Pruning against an external clock removes the edge entirely once all data is old,
        // but never touches static edges.
        b.set_transform(&tf(0.0, "base_link", "laser", 0.1, 0.0, 0.0), true)
            .unwrap();
        b.prune_before(ts(30.0));
        assert!(!b.has_frame(&f("odom")));
        assert!(b.has_frame(&f("laser")));
        assert!(TransformBuffer::new(TransformBufferConfig {
            cache_duration: -1.0,
            extrapolation_tolerance: 0.0
        })
        .is_err());
    }

    #[test]
    fn extrapolation_tolerance_allows_small_overshoot() {
        let mut b = TransformBuffer::new(TransformBufferConfig {
            cache_duration: 10.0,
            extrapolation_tolerance: 0.05,
        })
        .unwrap();
        b.set_transform(&tf(0.0, "odom", "base_link", 1.0, 0.0, 0.0), false)
            .unwrap();
        let r = b
            .lookup_transform(&f("odom"), &f("base_link"), LookupTime::At(ts(0.04)))
            .unwrap();
        assert!(close(&r.transform, &Se2::new(1.0, 0.0, 0.0)));
        assert!(b
            .lookup_transform(&f("odom"), &f("base_link"), LookupTime::At(ts(0.06)))
            .is_err());
    }
}
