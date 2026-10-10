//! Scripted command sequences.

use crate::SimError;
use robotics_core::Twist2;

/// Hold `twist` for `duration` seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    /// Duration (s).
    pub duration: f64,
    /// Commanded body twist.
    pub twist: Twist2,
}

/// A piecewise-constant command script, optionally repeating.
#[derive(Debug, Clone, PartialEq)]
pub struct Trajectory {
    segments: Vec<Segment>,
    period: f64,
    repeat: bool,
}

impl Trajectory {
    /// Build from segments; every duration must be positive and finite.
    pub fn new(segments: Vec<Segment>, repeat: bool) -> Result<Self, SimError> {
        if segments.is_empty() {
            return Err(SimError::EmptyTrajectory);
        }
        for s in &segments {
            if !(s.duration.is_finite() && s.duration > 0.0) {
                return Err(SimError::InvalidParameter {
                    what: "segment duration",
                    value: s.duration,
                });
            }
            if !s.twist.is_finite() {
                return Err(SimError::InvalidParameter {
                    what: "segment twist",
                    value: f64::NAN,
                });
            }
        }
        let period = segments.iter().map(|s| s.duration).sum();
        Ok(Self {
            segments,
            period,
            repeat,
        })
    }

    /// The Phase 2 test drive (≈ 60 s per lap): straight runs, turns in place, left and
    /// right arcs, a spin that crosses ±π several times, and a stop.
    pub fn mixed(repeat: bool) -> Self {
        let seg = |duration: f64, v: f64, w: f64| Segment {
            duration,
            twist: Twist2::unicycle(v, w),
        };
        Self::new(
            vec![
                seg(2.0, 0.0, 0.0),
                seg(8.0, 0.3, 0.0),
                seg(std::f64::consts::FRAC_PI_2 / 0.6, 0.0, 0.6),
                seg(12.0, 0.3, 0.3),
                seg(5.0, 0.3, 0.0),
                seg(8.0, 0.0, -0.9),
                seg(10.0, 0.25, -0.4),
                seg(6.0, 0.35, 0.1),
                seg(3.0, 0.0, 0.0),
            ],
            repeat,
        )
        .expect("static trajectory is valid")
    }

    /// Total duration of one pass (s).
    pub fn period(&self) -> f64 {
        self.period
    }

    /// Whether the script repeats after its last segment.
    pub fn repeats(&self) -> bool {
        self.repeat
    }

    /// Command at time `t` seconds since the start (zero after the end unless repeating).
    pub fn command(&self, t: f64) -> Twist2 {
        if t < 0.0 {
            return Twist2::ZERO;
        }
        let mut t = if self.repeat { t % self.period } else { t };
        for s in &self.segments {
            if t < s.duration {
                return s.twist;
            }
            t -= s.duration;
        }
        Twist2::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn piecewise_commands() {
        let tr = Trajectory::mixed(false);
        assert_eq!(tr.command(1.0), Twist2::ZERO);
        assert_eq!(tr.command(3.0), Twist2::unicycle(0.3, 0.0));
        assert_eq!(tr.command(tr.period() + 1.0), Twist2::ZERO);
        let rep = Trajectory::mixed(true);
        assert_eq!(rep.command(rep.period() + 3.0), Twist2::unicycle(0.3, 0.0));
        assert!(Trajectory::new(vec![], false).is_err());
        assert!(Trajectory::new(
            vec![Segment {
                duration: 0.0,
                twist: Twist2::ZERO
            }],
            false
        )
        .is_err());
    }
}
