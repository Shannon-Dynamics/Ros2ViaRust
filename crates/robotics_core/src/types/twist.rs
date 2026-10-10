//! Planar body-frame velocity.

use crate::se2::Tangent2;

/// Body-frame velocity: forward `v_x` (m/s), lateral `v_y` (m/s), yaw rate `ω` (rad/s).
///
/// The field order matches the SE(2) tangent convention, so `twist.tangent(dt)` is the
/// increment to feed to [`crate::Se2::exp`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Twist2 {
    /// Forward velocity (m/s).
    pub vx: f64,
    /// Lateral velocity (m/s); zero for a differential drive.
    pub vy: f64,
    /// Yaw rate (rad/s), counter-clockwise positive.
    pub omega: f64,
}

impl Twist2 {
    /// Zero motion.
    pub const ZERO: Twist2 = Twist2 {
        vx: 0.0,
        vy: 0.0,
        omega: 0.0,
    };

    /// A unicycle / differential-drive command `(v, ω)`.
    #[inline]
    pub const fn unicycle(v: f64, omega: f64) -> Self {
        Self {
            vx: v,
            vy: 0.0,
            omega,
        }
    }

    /// The tangent increment covered in `dt` seconds under constant velocity.
    #[inline]
    pub fn tangent(&self, dt: f64) -> Tangent2 {
        Tangent2::new(self.vx * dt, self.vy * dt, self.omega * dt)
    }

    /// `true` when all components are finite.
    #[inline]
    pub fn is_finite(&self) -> bool {
        self.vx.is_finite() && self.vy.is_finite() && self.omega.is_finite()
    }
}
