//! Planar points and point clouds.

use crate::frame::FrameId;
use crate::time::Timestamp;
use nalgebra::Vector2;

/// A point in the plane (m).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point2 {
    /// x (m).
    pub x: f64,
    /// y (m).
    pub y: f64,
}

impl Point2 {
    /// Construct a point.
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    /// As an `nalgebra` vector.
    #[inline]
    pub fn vector(&self) -> Vector2<f64> {
        Vector2::new(self.x, self.y)
    }
    /// Euclidean distance to another point.
    #[inline]
    pub fn distance_to(&self, other: &Point2) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

impl From<Vector2<f64>> for Point2 {
    fn from(v: Vector2<f64>) -> Self {
        Self { x: v.x, y: v.y }
    }
}

impl From<Point2> for Vector2<f64> {
    fn from(p: Point2) -> Self {
        Vector2::new(p.x, p.y)
    }
}

/// A stamped set of planar points expressed in `frame`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PointCloud2 {
    /// Acquisition time.
    pub stamp: Timestamp,
    /// Frame the points are expressed in.
    pub frame: FrameId,
    /// The points.
    pub points: Vec<Point2>,
}

impl PointCloud2 {
    /// Number of points.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }
    /// `true` when there are no points.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}
