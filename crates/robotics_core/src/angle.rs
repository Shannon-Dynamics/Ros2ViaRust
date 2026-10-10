//! Angle utilities. Every heading in the framework is kept in `(−π, π]`.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 3 (the wrap-around trap).

use std::f64::consts::{PI, TAU};

/// Normalize an angle to the half-open interval `(−π, π]`.
///
/// `normalize_angle(3π) == π` and `normalize_angle(−π) == π`, matching the book's
/// `normalizeAngle` invariant so heading comparisons never straddle the branch cut.
#[inline]
pub fn normalize_angle(a: f64) -> f64 {
    if !a.is_finite() {
        return a;
    }
    let mut r = a % TAU;
    if r <= -PI {
        r += TAU;
    } else if r > PI {
        r -= TAU;
    }
    r
}

/// Signed shortest difference `a − b`, normalized to `(−π, π]`.
#[inline]
pub fn angle_diff(a: f64, b: f64) -> f64 {
    normalize_angle(a - b)
}

/// Circular mean of a set of angles (returns `0.0` for an empty iterator).
pub fn circular_mean<I: IntoIterator<Item = f64>>(angles: I) -> f64 {
    let (mut s, mut c, mut n) = (0.0, 0.0, 0usize);
    for a in angles {
        s += a.sin();
        c += a.cos();
        n += 1;
    }
    if n == 0 {
        0.0
    } else {
        s.atan2(c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn wraps_into_half_open_interval() {
        assert_relative_eq!(normalize_angle(3.0 * PI), PI, epsilon = 1e-12);
        assert_relative_eq!(normalize_angle(-PI), PI, epsilon = 1e-12);
        assert_relative_eq!(normalize_angle(PI), PI, epsilon = 1e-12);
        assert_relative_eq!(normalize_angle(-3.5 * PI), 0.5 * PI, epsilon = 1e-12);
        assert_relative_eq!(normalize_angle(0.3), 0.3, epsilon = 1e-15);
    }

    #[test]
    fn diff_takes_shortest_way_round() {
        assert_relative_eq!(angle_diff(3.0, -3.0), 6.0 - TAU, epsilon = 1e-12);
        assert_relative_eq!(angle_diff(-3.0, 3.0), TAU - 6.0, epsilon = 1e-12);
    }

    #[test]
    fn circular_mean_handles_branch_cut() {
        let m = circular_mean([PI - 0.1, -PI + 0.1]);
        assert!((m.abs() - PI).abs() < 1e-12, "mean {m} should sit at ±π");
        assert_eq!(circular_mean(std::iter::empty()), 0.0);
    }
}
