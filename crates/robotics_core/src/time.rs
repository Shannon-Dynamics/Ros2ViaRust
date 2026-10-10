//! Timestamps with nanosecond resolution, independent of any clock implementation.

use std::ops::Sub;

/// A point in time: nanoseconds since the epoch of whichever clock produced it
/// (ROS time, steady time or simulation time). Comparing stamps from different clocks is
/// a caller error the type cannot detect.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Timestamp {
    nanos: i64,
}

impl Timestamp {
    /// The epoch itself.
    pub const ZERO: Timestamp = Timestamp { nanos: 0 };

    /// From a raw nanosecond count.
    #[inline]
    pub const fn from_nanos(nanos: i64) -> Self {
        Self { nanos }
    }

    /// From the `(sec, nanosec)` pair ROS messages carry.
    #[inline]
    pub const fn from_sec_nanosec(sec: i32, nanosec: u32) -> Self {
        Self {
            nanos: sec as i64 * 1_000_000_000 + nanosec as i64,
        }
    }

    /// From floating-point seconds (rounded to the nearest nanosecond).
    #[inline]
    pub fn from_secs_f64(secs: f64) -> Self {
        Self {
            nanos: (secs * 1e9).round() as i64,
        }
    }

    /// Raw nanosecond count.
    #[inline]
    pub const fn as_nanos(&self) -> i64 {
        self.nanos
    }

    /// Seconds as `f64` (loses sub-microsecond precision for large stamps; fine for durations).
    #[inline]
    pub fn as_secs_f64(&self) -> f64 {
        self.nanos as f64 * 1e-9
    }

    /// Split into the `(sec, nanosec)` pair used by `builtin_interfaces/Time`.
    ///
    /// Negative stamps are floored so `nanosec` stays in `[0, 1e9)`.
    #[inline]
    pub const fn sec_nanosec(&self) -> (i32, u32) {
        let sec = self.nanos.div_euclid(1_000_000_000);
        let nanosec = self.nanos.rem_euclid(1_000_000_000);
        (sec as i32, nanosec as u32)
    }

    /// Signed elapsed seconds `self − earlier`.
    #[inline]
    pub fn seconds_since(&self, earlier: Timestamp) -> f64 {
        (self.nanos - earlier.nanos) as f64 * 1e-9
    }

    /// Stamp shifted by a signed number of seconds.
    #[inline]
    pub fn plus_secs_f64(&self, secs: f64) -> Timestamp {
        Timestamp {
            nanos: self.nanos + (secs * 1e9).round() as i64,
        }
    }
}

impl Sub for Timestamp {
    /// Difference in seconds.
    type Output = f64;
    #[inline]
    fn sub(self, rhs: Timestamp) -> f64 {
        self.seconds_since(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sec_nanosec_round_trip() {
        let t = Timestamp::from_sec_nanosec(1_700_000_000, 123_456_789);
        assert_eq!(t.sec_nanosec(), (1_700_000_000, 123_456_789));
        assert_eq!(t.as_nanos(), 1_700_000_000_123_456_789);
        let neg = Timestamp::from_nanos(-1);
        assert_eq!(neg.sec_nanosec(), (-1, 999_999_999));
    }

    #[test]
    fn differences_are_seconds() {
        let a = Timestamp::from_secs_f64(10.0);
        let b = Timestamp::from_secs_f64(10.25);
        assert!((b - a - 0.25).abs() < 1e-12);
        assert!((a.plus_secs_f64(0.25) - b).abs() < 1e-12);
    }
}
