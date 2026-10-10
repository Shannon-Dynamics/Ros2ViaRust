//! Coordinate-frame identifiers.

use std::fmt;
use std::sync::Arc;

/// The name of a coordinate frame (`"map"`, `"odom"`, `"base_link"`, `"laser"`, …).
///
/// Cheap to clone (reference-counted), compared by string value. The framework never
/// interprets frame names; the TF tree design in `docs/ROS2_INTERFACES.md` says which
/// node owns which edge.
#[derive(Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FrameId(Arc<str>);

impl FrameId {
    /// Create a frame id from any string-like value.
    pub fn new(name: impl AsRef<str>) -> Self {
        Self(Arc::from(name.as_ref()))
    }
    /// The frame name.
    #[inline]
    pub fn as_str(&self) -> &str {
        &self.0
    }
    /// `true` for the empty frame id (unset).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl Default for FrameId {
    fn default() -> Self {
        Self(Arc::from(""))
    }
}

impl fmt::Debug for FrameId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "FrameId({:?})", self.as_str())
    }
}

impl fmt::Display for FrameId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl From<&str> for FrameId {
    fn from(s: &str) -> Self {
        Self::new(s)
    }
}

impl From<String> for FrameId {
    fn from(s: String) -> Self {
        Self::new(s)
    }
}
