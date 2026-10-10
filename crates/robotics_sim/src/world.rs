//! Static 2-D world geometry and exact ray casting.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 4 (`World` as polyline geometry:
//! "exact ray/segment intersection is cheap, exact, and needs no discretization").
//!
//! The world is a set of line segments (walls, box outlines, polylines). A ray is cast
//! against every segment analytically: `O(n_segments)` per ray, which for the handful of
//! segments a Phase 2 world has is faster than building an acceleration structure. The
//! book uses `parry2d`'s BVH for large maps; it is not needed here (see
//! `docs/SIMULATION.md`, "Dependencies").
//!
//! The world is **not** a collision model: the simulator never stops the robot at a
//! wall. Worlds are designed so that the scripted trajectory stays clear of all geometry
//! (checked by `rusty_arena_is_clear_of_the_trajectory`).

use crate::SimError;
use nalgebra::Vector2;

/// A wall segment from `a` to `b` (world frame, metres).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment2 {
    /// First endpoint.
    pub a: Vector2<f64>,
    /// Second endpoint.
    pub b: Vector2<f64>,
}

impl Segment2 {
    /// Euclidean distance from `p` to the segment.
    pub fn distance_to(&self, p: &Vector2<f64>) -> f64 {
        let e = self.b - self.a;
        let s = ((p - self.a).dot(&e) / e.norm_squared()).clamp(0.0, 1.0);
        (self.a + e * s - p).norm()
    }
}

/// Tolerance on the segment parameter so rays through a shared corner cannot slip
/// between the two segments that meet there.
const PARAM_EPS: f64 = 1e-12;

#[inline]
fn cross(a: &Vector2<f64>, b: &Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

/// A static world made of segments.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct World {
    segments: Vec<Segment2>,
}

impl World {
    /// An empty world (every ray escapes).
    pub fn empty() -> Self {
        Self::default()
    }

    /// Add one segment; endpoints must be finite and distinct.
    pub fn add_segment(&mut self, a: Vector2<f64>, b: Vector2<f64>) -> Result<(), SimError> {
        if !(a.iter().chain(b.iter()).all(|v| v.is_finite())) {
            return Err(SimError::InvalidParameter {
                what: "segment endpoint",
                value: f64::NAN,
            });
        }
        let len = (b - a).norm();
        if len < 1e-9 {
            return Err(SimError::InvalidParameter {
                what: "segment length",
                value: len,
            });
        }
        self.segments.push(Segment2 { a, b });
        Ok(())
    }

    /// Add a polyline through `points`, closed back to the first point if `closed`.
    pub fn add_polyline(&mut self, points: &[Vector2<f64>], closed: bool) -> Result<(), SimError> {
        if points.len() < 2 {
            return Err(SimError::InvalidParameter {
                what: "polyline point count",
                value: points.len() as f64,
            });
        }
        for w in points.windows(2) {
            self.add_segment(w[0], w[1])?;
        }
        if closed && points.len() > 2 {
            self.add_segment(points[points.len() - 1], points[0])?;
        }
        Ok(())
    }

    /// Add the outline of an axis-aligned rectangle `[min, max]` (a room's walls or a
    /// box obstacle).
    pub fn add_rectangle(&mut self, min: Vector2<f64>, max: Vector2<f64>) -> Result<(), SimError> {
        if !(max.x > min.x && max.y > min.y) {
            return Err(SimError::InvalidParameter {
                what: "rectangle extent",
                value: (max - min).min(),
            });
        }
        self.add_polyline(
            &[
                min,
                Vector2::new(max.x, min.y),
                max,
                Vector2::new(min.x, max.y),
            ],
            true,
        )
    }

    /// Add the outline of a box centred at `center` with half extents `half` rotated by
    /// `yaw`.
    pub fn add_box(
        &mut self,
        center: Vector2<f64>,
        half: Vector2<f64>,
        yaw: f64,
    ) -> Result<(), SimError> {
        if !(half.x > 0.0 && half.y > 0.0 && yaw.is_finite()) {
            return Err(SimError::InvalidParameter {
                what: "box half extent",
                value: half.min(),
            });
        }
        let (s, c) = yaw.sin_cos();
        let corner = |dx: f64, dy: f64| center + Vector2::new(c * dx - s * dy, s * dx + c * dy);
        self.add_polyline(
            &[
                corner(-half.x, -half.y),
                corner(half.x, -half.y),
                corner(half.x, half.y),
                corner(-half.x, half.y),
            ],
            true,
        )
    }

    /// The segments.
    pub fn segments(&self) -> &[Segment2] {
        &self.segments
    }

    /// Distance from `p` to the nearest segment (`∞` in an empty world).
    pub fn clearance(&self, p: &Vector2<f64>) -> f64 {
        self.segments
            .iter()
            .map(|s| s.distance_to(p))
            .fold(f64::INFINITY, f64::min)
    }

    /// First intersection of the ray `origin + t·(cos angle, sin angle)`, `t ≥ 0`, with
    /// any segment, if it lies within `max_range`. Rays parallel to a segment do not hit
    /// it (a grazing ray along a wall is a measure-zero event for a real sensor too).
    pub fn raycast(&self, origin: &Vector2<f64>, angle: f64, max_range: f64) -> Option<f64> {
        let d = Vector2::new(angle.cos(), angle.sin());
        let mut best = f64::INFINITY;
        for seg in &self.segments {
            let e = seg.b - seg.a;
            let denom = cross(&d, &e);
            if denom.abs() < 1e-15 {
                continue;
            }
            let ao = seg.a - origin;
            let t = cross(&ao, &e) / denom;
            let s = cross(&ao, &d) / denom;
            if t >= 0.0 && (-PARAM_EPS..=1.0 + PARAM_EPS).contains(&s) && t < best {
                best = t;
            }
        }
        (best <= max_range).then_some(best)
    }

    /// The Phase 2 test world: a 13 m × 13 m room around the Rusty mixed trajectory
    /// (which, repeated, orbits inside `x ∈ [−3.8, 5.6]`, `y ∈ [−2.2, 7.2]`), with box
    /// obstacles and a wall stub near the corners so scans have structure in every
    /// direction. The robot starts at the origin facing +x.
    pub fn rusty_arena() -> Self {
        let mut w = World::empty();
        let v = Vector2::new;
        // Outer walls.
        w.add_rectangle(v(-5.5, -4.0), v(7.5, 9.0))
            .expect("static geometry");
        // Box obstacles in three corners.
        w.add_rectangle(v(-5.0, -3.5), v(-4.2, -2.7))
            .expect("static geometry");
        w.add_rectangle(v(6.3, 7.8), v(7.0, 8.5))
            .expect("static geometry");
        w.add_box(v(6.6, -3.1), v(0.3, 0.3), 0.5)
            .expect("static geometry");
        // A wall stub on the west wall and a pillar in the north-west corner.
        w.add_segment(v(-5.5, 5.5), v(-4.6, 5.5))
            .expect("static geometry");
        w.add_box(v(-4.8, 8.3), v(0.2, 0.2), 0.0)
            .expect("static geometry");
        w
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    fn room() -> World {
        let mut w = World::empty();
        w.add_rectangle(Vector2::new(-2.0, -1.0), Vector2::new(4.0, 3.0))
            .unwrap();
        w
    }

    #[test]
    fn rays_hit_the_walls_of_a_room() {
        let w = room();
        let o = Vector2::new(0.0, 0.0);
        assert_relative_eq!(w.raycast(&o, 0.0, 10.0).unwrap(), 4.0, epsilon = 1e-12);
        assert_relative_eq!(w.raycast(&o, PI, 10.0).unwrap(), 2.0, epsilon = 1e-12);
        assert_relative_eq!(
            w.raycast(&o, FRAC_PI_2, 10.0).unwrap(),
            3.0,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            w.raycast(&o, -FRAC_PI_2, 10.0).unwrap(),
            1.0,
            epsilon = 1e-12
        );
        // Diagonal into the (−2, −1) corner region: hits y = −1 first at t = √2.
        assert_relative_eq!(
            w.raycast(&o, -3.0 * FRAC_PI_4, 10.0).unwrap(),
            std::f64::consts::SQRT_2,
            epsilon = 1e-12
        );
        // Beyond max range → no hit.
        assert_eq!(w.raycast(&o, 0.0, 3.9), None);
        assert_eq!(World::empty().raycast(&o, 0.3, 100.0), None);
    }

    #[test]
    fn rays_through_a_corner_do_not_leak() {
        let w = room();
        // Exactly at the (4, 3) corner from a point on the diagonal.
        let o = Vector2::new(1.0, 0.0);
        let d = (Vector2::new(4.0, 3.0) - o).norm();
        let a = (3.0f64).atan2(3.0);
        assert_relative_eq!(w.raycast(&o, a, 10.0).unwrap(), d, epsilon = 1e-9);
    }

    #[test]
    fn nearest_obstacle_wins_and_clearance() {
        let mut w = room();
        w.add_box(Vector2::new(2.0, 0.0), Vector2::new(0.5, 0.5), 0.0)
            .unwrap();
        assert_relative_eq!(
            w.raycast(&Vector2::zeros(), 0.0, 10.0).unwrap(),
            1.5,
            epsilon = 1e-12
        );
        assert_relative_eq!(w.clearance(&Vector2::zeros()), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn invalid_geometry_is_rejected() {
        let mut w = World::empty();
        let z = Vector2::zeros();
        assert!(w.add_segment(z, z).is_err());
        assert!(w.add_segment(z, Vector2::new(f64::NAN, 0.0)).is_err());
        assert!(w
            .add_rectangle(Vector2::new(1.0, 0.0), Vector2::new(0.0, 1.0))
            .is_err());
        assert!(w.add_polyline(&[z], false).is_err());
        assert!(w.add_box(z, Vector2::new(0.0, 1.0), 0.0).is_err());
        assert!(w.segments().is_empty());
    }
}
