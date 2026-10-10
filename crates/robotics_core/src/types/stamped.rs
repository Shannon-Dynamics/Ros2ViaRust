//! Generic time- and frame-stamped wrapper.

use crate::frame::FrameId;
use crate::time::Timestamp;

/// A value with its acquisition time and coordinate frame.
///
/// Reference: *Probabilistic Robotics via Rust*, Chapter 26 (`Stamped<T, F>` and
/// staleness-driven watchdogs in the capstone stack).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Stamped<T> {
    /// Acquisition time.
    pub stamp: Timestamp,
    /// Frame the value is expressed in.
    pub frame: FrameId,
    /// The payload.
    pub value: T,
}

impl<T> Stamped<T> {
    /// Wrap a value.
    pub fn new(stamp: Timestamp, frame: impl Into<FrameId>, value: T) -> Self {
        Self {
            stamp,
            frame: frame.into(),
            value,
        }
    }

    /// Age of the value at `now`, in seconds (negative if stamped in the future).
    #[inline]
    pub fn age(&self, now: Timestamp) -> f64 {
        now - self.stamp
    }

    /// Map the payload, keeping stamp and frame.
    pub fn map<U>(self, f: impl FnOnce(T) -> U) -> Stamped<U> {
        Stamped {
            stamp: self.stamp,
            frame: self.frame,
            value: f(self.value),
        }
    }
}
