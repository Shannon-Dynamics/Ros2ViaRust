//! TF for `rclrs`: a broadcaster and a listener backed by a Rust transform buffer.
//!
//! `rclrs` 0.8 ships no `tf2_ros` binding, so both directions talk to the `/tf` and
//! `/tf_static` topics directly, with the QoS `tf2_ros` uses:
//!
//! ```text
//!  /tf (volatile), /tf_static (transient local)          tf2_msgs/TFMessage
//!        │  TfListener subscriptions (node-level callbacks)
//!        ▼  convert::transform_stamped_from_ros   (planar check, frame-id cleanup)
//!  robotics_core::TransformBuffer                   (ROS-independent, unit tested)
//!        │
//!        ▼  TfListener::lookup_transform(target, source, time)
//! ```
//!
//! The listener also records which publisher (DDS GID) provides each child frame, so a
//! launch with two nodes broadcasting the same edge — a TF authority conflict that `tf2`
//! only warns about — is detectable ([`TfListener::authority_conflicts`]).
//!
//! Limitations: planar transforms only (see `robotics_core::tf`); no blocking
//! `lookup_transform(..., timeout)` — poll [`TfListener::can_transform`] from a timer.

use crate::convert::{transform_stamped, transform_stamped_from_ros};
use crate::qos;
use crate::shared::Shared;
use rclrs::{IntoPrimitiveOptions, MessageInfo, Node, Publisher, RclrsError, Subscription};
use robotics_core::{
    FrameId, LookupTime, Se2, TfError, Timestamp, TransformBuffer, TransformStamped2,
};
use ros_env::tf2_msgs;
use std::collections::{BTreeMap, BTreeSet};

/// Publishes `parent → child` transforms.
pub struct TfBroadcaster {
    publisher: Publisher<tf2_msgs::msg::TFMessage>,
}

impl TfBroadcaster {
    /// Broadcaster for dynamic transforms on `/tf`.
    pub fn new(node: &Node) -> Result<Self, RclrsError> {
        let publisher = node.create_publisher::<tf2_msgs::msg::TFMessage>("/tf".qos(qos::tf()))?;
        Ok(Self { publisher })
    }

    /// Broadcaster for static transforms on `/tf_static` (transient local, so late
    /// listeners still receive them).
    pub fn new_static(node: &Node) -> Result<Self, RclrsError> {
        let publisher =
            node.create_publisher::<tf2_msgs::msg::TFMessage>("/tf_static".qos(qos::tf_static()))?;
        Ok(Self { publisher })
    }

    /// Send one transform.
    pub fn send(
        &self,
        stamp: Timestamp,
        parent: &FrameId,
        child: &FrameId,
        pose: &Se2,
    ) -> Result<(), RclrsError> {
        let msg = tf2_msgs::msg::TFMessage {
            transforms: vec![transform_stamped(stamp, parent, child, pose)],
        };
        self.publisher.publish(msg)
    }

    /// Send several transforms in one message. For a static broadcaster, send *all*
    /// static edges of the node in one call: with transient-local durability only the
    /// latest message per publisher is guaranteed to reach late joiners.
    pub fn send_many(
        &self,
        stamp: Timestamp,
        transforms: &[(FrameId, FrameId, Se2)],
    ) -> Result<(), RclrsError> {
        let msg = tf2_msgs::msg::TFMessage {
            transforms: transforms
                .iter()
                .map(|(p, c, pose)| transform_stamped(stamp, p, c, pose))
                .collect(),
        };
        self.publisher.publish(msg)
    }
}

/// Listener counters.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TfListenerStats {
    /// Transforms received on `/tf`.
    pub dynamic_received: u64,
    /// Transforms received on `/tf_static`.
    pub static_received: u64,
    /// Transforms stored in the buffer.
    pub inserted: u64,
    /// Transforms rejected by conversion (non-planar, malformed) or by the buffer
    /// (cycle, static/dynamic conflict, non-finite).
    pub rejected: u64,
    /// The most recent rejection reason.
    pub last_error: String,
    /// Distinct publishers (GIDs) seen per child frame.
    publishers: BTreeMap<String, BTreeSet<Vec<u8>>>,
}

impl TfListenerStats {
    /// Number of distinct publishers seen for `child`.
    pub fn publisher_count(&self, child: &str) -> usize {
        self.publishers.get(child).map_or(0, BTreeSet::len)
    }
}

/// Subscribes to `/tf` and `/tf_static` and keeps a [`TransformBuffer`] up to date.
pub struct TfListener {
    buffer: Shared<TransformBuffer>,
    stats: Shared<TfListenerStats>,
    _dynamic: Subscription<tf2_msgs::msg::TFMessage>,
    _static: Subscription<tf2_msgs::msg::TFMessage>,
}

fn ingest(
    buffer: &Shared<TransformBuffer>,
    stats: &Shared<TfListenerStats>,
    msg: &tf2_msgs::msg::TFMessage,
    info: &MessageInfo,
    is_static: bool,
) {
    let gid = info.publisher_gid.data.to_vec();
    let mut st = stats.lock();
    let mut buf = buffer.lock();
    for t in &msg.transforms {
        if is_static {
            st.static_received += 1;
        } else {
            st.dynamic_received += 1;
        }
        let converted = transform_stamped_from_ros(t)
            .map_err(|e| e.to_string())
            .and_then(|t2| {
                buf.set_transform(&t2, is_static)
                    .map(|_| t2.child)
                    .map_err(|e| e.to_string())
            });
        match converted {
            Ok(child) => {
                st.inserted += 1;
                st.publishers
                    .entry(child.as_str().to_owned())
                    .or_default()
                    .insert(gid.clone());
            }
            Err(e) => {
                st.rejected += 1;
                st.last_error = format!("{} -> {}: {e}", t.header.frame_id, t.child_frame_id);
            }
        }
    }
}

impl TfListener {
    /// Start listening; transforms are inserted into `buffer` from the node's executor.
    pub fn new(node: &Node, buffer: Shared<TransformBuffer>) -> Result<Self, RclrsError> {
        let stats = Shared::new(TfListenerStats::default());
        let dynamic =
            node.create_subscription::<tf2_msgs::msg::TFMessage, _>("/tf".qos(qos::tf()), {
                let (buffer, stats) = (buffer.clone(), stats.clone());
                move |msg: tf2_msgs::msg::TFMessage, info: MessageInfo| {
                    ingest(&buffer, &stats, &msg, &info, false)
                }
            })?;
        let statik = node.create_subscription::<tf2_msgs::msg::TFMessage, _>(
            "/tf_static".qos(qos::tf_static()),
            {
                let (buffer, stats) = (buffer.clone(), stats.clone());
                move |msg: tf2_msgs::msg::TFMessage, info: MessageInfo| {
                    ingest(&buffer, &stats, &msg, &info, true)
                }
            },
        )?;
        Ok(Self {
            buffer,
            stats,
            _dynamic: dynamic,
            _static: statik,
        })
    }

    /// The shared buffer (lock it briefly; callbacks insert into it).
    pub fn buffer(&self) -> &Shared<TransformBuffer> {
        &self.buffer
    }

    /// A snapshot of the counters.
    pub fn stats(&self) -> TfListenerStats {
        self.stats.lock().clone()
    }

    /// `T_target_source` at `time` (see [`TransformBuffer::lookup_transform`]).
    pub fn lookup_transform(
        &self,
        target: &str,
        source: &str,
        time: LookupTime,
    ) -> Result<TransformStamped2, TfError> {
        self.buffer
            .lock()
            .lookup_transform(&FrameId::new(target), &FrameId::new(source), time)
    }

    /// Whether the lookup would currently succeed.
    pub fn can_transform(&self, target: &str, source: &str, time: LookupTime) -> bool {
        self.lookup_transform(target, source, time).is_ok()
    }

    /// Child frames that received transforms from more than one publisher, with the
    /// number of distinct publishers (empty when every edge has a single owner).
    pub fn authority_conflicts(&self) -> Vec<(String, usize)> {
        self.stats
            .lock()
            .publishers
            .iter()
            .filter(|(_, g)| g.len() > 1)
            .map(|(c, g)| (c.clone(), g.len()))
            .collect()
    }
}
