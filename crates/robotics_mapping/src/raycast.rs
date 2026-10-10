//! Walking a beam through the grid.
//!
//! Two traversals, because they answer slightly different questions and the difference is
//! worth measuring rather than assuming away:
//!
//! * [`Bresenham`]: the classic integer line between two *cell indices*. It joins cell centres
//!   with the fewest cells, steps diagonally whenever the error term says so, and therefore
//!   can step across the corner of a cell that the real beam passes through. Cheap and
//!   conventional.
//! * [`Supercover`]: every cell the *continuous segment* crosses, found with the voxel
//!   traversal of Amanatides and Woo (1987, "A fast voxel traversal algorithm for ray
//!   tracing"). No cell the beam passes through is skipped, which matters for a thin wall
//!   hit at a grazing angle: a skipped free cell is a gap in the evidence, and a skipped cell
//!   one short of the hit is a hit that lands in the wrong place.
//!
//! **What "crossed" means.** A cell is crossed when the segment spends a positive length inside
//! it. A segment that passes exactly through a lattice corner touches the two side cells at a
//! single point and does not cross them; it steps diagonally. Two cells are always visited
//! whatever the geometry: the one containing the start and the one containing the end, with a
//! point exactly on an edge belonging to the cell on its positive side (the convention of
//! [`crate::occupancy::GridGeometry::world_to_cell`]).
//!
//! Both traversals work in cell-index space and return signed indices, so a beam that leaves
//! the grid is not an error here; whoever consumes the cells decides what outside means.

use robotics_core::nalgebra::Vector2;

/// Integer Bresenham line from cell `a` to cell `b`, both ends included.
#[derive(Debug, Clone)]
pub struct Bresenham {
    x: i64,
    y: i64,
    x1: i64,
    y1: i64,
    dx: i64,
    dy: i64,
    sx: i64,
    sy: i64,
    err: i64,
    done: bool,
}

impl Bresenham {
    /// A line from `a` to `b`.
    pub fn new(a: (i32, i32), b: (i32, i32)) -> Self {
        let (x0, y0, x1, y1) = (a.0 as i64, a.1 as i64, b.0 as i64, b.1 as i64);
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        Self {
            x: x0,
            y: y0,
            x1,
            y1,
            dx,
            dy,
            sx: if x0 < x1 { 1 } else { -1 },
            sy: if y0 < y1 { 1 } else { -1 },
            err: dx + dy,
            done: false,
        }
    }
}

impl Iterator for Bresenham {
    type Item = (i32, i32);

    fn next(&mut self) -> Option<(i32, i32)> {
        if self.done {
            return None;
        }
        let out = (self.x as i32, self.y as i32);
        if self.x == self.x1 && self.y == self.y1 {
            self.done = true;
            return Some(out);
        }
        let e2 = 2 * self.err;
        if e2 >= self.dy {
            self.err += self.dy;
            self.x += self.sx;
        }
        if e2 <= self.dx {
            self.err += self.dx;
            self.y += self.sy;
        }
        Some(out)
    }
}

/// Longest traversal this module will start, in cells. A beam is at most `range_max` long, a
/// few hundred cells; anything beyond a million is a bug or a non-finite input.
const MAX_STEPS: f64 = 1_048_576.0;

/// The cells crossed by a continuous segment, in order from start to end.
#[derive(Debug, Clone)]
pub struct Supercover {
    ix: i64,
    iy: i64,
    end_x: i64,
    end_y: i64,
    step_x: i64,
    step_y: i64,
    t_max_x: f64,
    t_max_y: f64,
    t_delta_x: f64,
    t_delta_y: f64,
    done: bool,
}

impl Supercover {
    /// The cells crossed by the segment from `p0` to `p1` (world metres), on a grid whose cell
    /// `(0, 0)` has its lower-left corner at `origin` and whose cells are `resolution` wide.
    /// A non-finite input, or a segment longer than a million cells, yields no cells.
    pub fn new(
        p0: &Vector2<f64>,
        p1: &Vector2<f64>,
        origin: &Vector2<f64>,
        resolution: f64,
    ) -> Self {
        let u0 = (p0.x - origin.x) / resolution;
        let v0 = (p0.y - origin.y) / resolution;
        let u1 = (p1.x - origin.x) / resolution;
        let v1 = (p1.y - origin.y) / resolution;
        let empty = Self {
            ix: 0,
            iy: 0,
            end_x: 0,
            end_y: 0,
            step_x: 0,
            step_y: 0,
            t_max_x: f64::INFINITY,
            t_max_y: f64::INFINITY,
            t_delta_x: f64::INFINITY,
            t_delta_y: f64::INFINITY,
            done: true,
        };
        let all = [u0, v0, u1, v1];
        if all.iter().any(|v| !v.is_finite() || v.abs() > 1e9) {
            return empty;
        }
        if (u1 - u0).abs() + (v1 - v0).abs() > MAX_STEPS {
            return empty;
        }
        let (ix, iy) = (u0.floor() as i64, v0.floor() as i64);
        let (end_x, end_y) = (u1.floor() as i64, v1.floor() as i64);
        let (dx, dy) = (u1 - u0, v1 - v0);
        let axis = |i: i64, u: f64, d: f64| -> (i64, f64, f64) {
            if d > 0.0 {
                (1, ((i + 1) as f64 - u) / d, 1.0 / d)
            } else if d < 0.0 {
                (-1, (i as f64 - u) / d, -1.0 / d)
            } else {
                (0, f64::INFINITY, f64::INFINITY)
            }
        };
        let (step_x, t_max_x, t_delta_x) = axis(ix, u0, dx);
        let (step_y, t_max_y, t_delta_y) = axis(iy, v0, dy);
        Self {
            ix,
            iy,
            end_x,
            end_y,
            step_x,
            step_y,
            t_max_x,
            t_max_y,
            t_delta_x,
            t_delta_y,
            done: false,
        }
    }
}

impl Iterator for Supercover {
    type Item = (i32, i32);

    fn next(&mut self) -> Option<(i32, i32)> {
        if self.done {
            return None;
        }
        let out = (self.ix as i32, self.iy as i32);
        if self.ix == self.end_x && self.iy == self.end_y {
            self.done = true;
            return Some(out);
        }
        // Which boundary comes first. An axis already at its end index never steps again, so
        // rounding in the parametric values cannot send the walk past the end cell, and the
        // loop terminates after at most |end_x - ix| + |end_y - iy| steps.
        let x_open = self.ix != self.end_x && self.step_x != 0;
        let y_open = self.iy != self.end_y && self.step_y != 0;
        let go_x;
        let go_y;
        if x_open && y_open {
            if self.t_max_x < self.t_max_y {
                (go_x, go_y) = (true, false);
            } else if self.t_max_y < self.t_max_x {
                (go_x, go_y) = (false, true);
            } else {
                // Exactly through a corner: the two side cells are touched at a point only.
                (go_x, go_y) = (true, true);
            }
        } else {
            (go_x, go_y) = (x_open, y_open);
        }
        if !go_x && !go_y {
            // Unreachable for finite input (the end cell was not reached but no axis may
            // move); stop rather than loop.
            self.done = true;
            return Some(out);
        }
        if go_x {
            self.ix += self.step_x;
            self.t_max_x += self.t_delta_x;
        }
        if go_y {
            self.iy += self.step_y;
            self.t_max_y += self.t_delta_y;
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::collections::BTreeSet;

    fn v(x: f64, y: f64) -> Vector2<f64> {
        Vector2::new(x, y)
    }

    fn unit_supercover(p0: Vector2<f64>, p1: Vector2<f64>) -> Vec<(i32, i32)> {
        Supercover::new(&p0, &p1, &v(0.0, 0.0), 1.0).collect()
    }

    /// The textbook line from (0, 0) to (5, 2), traced by hand with the error term:
    /// err = 3, then the cells (0,0) (1,0) (2,1) (3,1) (4,2) (5,2).
    #[test]
    fn worked_example_bresenham_five_by_two() {
        let cells: Vec<_> = Bresenham::new((0, 0), (5, 2)).collect();
        assert_eq!(cells, vec![(0, 0), (1, 0), (2, 1), (3, 1), (4, 2), (5, 2)]);
    }

    #[test]
    fn bresenham_every_octant_includes_both_ends_and_is_contiguous() {
        let ends = [
            (7, 0),
            (7, 3),
            (7, 7),
            (3, 7),
            (0, 7),
            (-3, 7),
            (-7, 7),
            (-7, 3),
            (-7, 0),
            (-7, -3),
            (-7, -7),
            (-3, -7),
            (0, -7),
            (3, -7),
            (7, -7),
            (7, -3),
            (0, 0),
        ];
        for b in ends {
            let cells: Vec<_> = Bresenham::new((0, 0), b).collect();
            let n = b.0.abs().max(b.1.abs()) as usize + 1;
            assert_eq!(cells.len(), n, "line to {b:?}");
            assert_eq!(cells[0], (0, 0));
            assert_eq!(*cells.last().unwrap(), b);
            for w in cells.windows(2) {
                assert!((w[1].0 - w[0].0).abs() <= 1 && (w[1].1 - w[0].1).abs() <= 1);
            }
        }
    }

    /// A segment through a lattice corner touches the two side cells at a point and does not
    /// cross them: the supercover steps diagonally, as does Bresenham.
    #[test]
    fn worked_example_supercover_through_a_corner_steps_diagonally() {
        // (0.5, 0.5) to (3.5, 3.5): the corners (1,1), (2,2), (3,3).
        let cells = unit_supercover(v(0.5, 0.5), v(3.5, 3.5));
        assert_eq!(cells, vec![(0, 0), (1, 1), (2, 2), (3, 3)]);
        // The same through corners in the negative quadrant direction: (0.25, 0.75) to
        // (2.25, -1.25) passes (1, 0) and (2, -1), both exactly representable.
        let cells = unit_supercover(v(0.25, 0.75), v(2.25, -1.25));
        assert_eq!(cells, vec![(0, 0), (1, -1), (2, -2)]);
    }

    /// The case that separates the two traversals. The segment (0.95, 0.95) to (2.05, 1.05)
    /// crosses x = 1 at y = 0.9545, below the line y = 1, so it passes through cell (1, 0) and
    /// only then climbs into (1, 1). The cell-centre Bresenham line from (0, 0) to (2, 1) steps
    /// diagonally from (0, 0) to (1, 1) and never visits (1, 0).
    #[test]
    fn worked_example_bresenham_skips_a_cell_the_supercover_crosses() {
        let super_cells = unit_supercover(v(0.95, 0.95), v(2.05, 1.05));
        assert_eq!(super_cells, vec![(0, 0), (1, 0), (1, 1), (2, 1)]);
        let bres: Vec<_> = Bresenham::new((0, 0), (2, 1)).collect();
        assert_eq!(bres, vec![(0, 0), (1, 1), (2, 1)]);
        assert!(!bres.contains(&(1, 0)));
    }

    #[test]
    fn supercover_axis_aligned_and_degenerate_segments() {
        assert_eq!(
            unit_supercover(v(0.5, 0.5), v(3.5, 0.5)),
            vec![(0, 0), (1, 0), (2, 0), (3, 0)]
        );
        assert_eq!(
            unit_supercover(v(0.5, 3.5), v(0.5, 0.5)),
            vec![(0, 3), (0, 2), (0, 1), (0, 0)]
        );
        // A point is its own cell.
        assert_eq!(unit_supercover(v(2.25, 1.75), v(2.25, 1.75)), vec![(2, 1)]);
        // A segment inside one cell visits one cell.
        assert_eq!(unit_supercover(v(2.1, 1.1), v(2.9, 1.9)), vec![(2, 1)]);
    }

    #[test]
    fn supercover_respects_origin_and_resolution() {
        // Cells of 0.5 m with the origin at (-1, -1): the point (-0.8, -0.8) is in cell (0, 0)
        // and (0.8, -0.8) in cell (3, 0).
        let cells: Vec<_> =
            Supercover::new(&v(-0.8, -0.8), &v(0.8, -0.8), &v(-1.0, -1.0), 0.5).collect();
        assert_eq!(cells, vec![(0, 0), (1, 0), (2, 0), (3, 0)]);
    }

    #[test]
    fn supercover_refuses_non_finite_and_absurd_input() {
        let o = v(0.0, 0.0);
        assert_eq!(
            Supercover::new(&v(f64::NAN, 0.0), &v(1.0, 1.0), &o, 1.0).count(),
            0
        );
        assert_eq!(
            Supercover::new(&v(0.0, 0.0), &v(f64::INFINITY, 1.0), &o, 1.0).count(),
            0
        );
        assert_eq!(
            Supercover::new(&v(0.0, 0.0), &v(1e7, 0.0), &o, 1.0).count(),
            0
        );
    }

    /// The reference the traversal is graded against: every cell in the bounding window whose
    /// open box the segment spends positive length in (Liang-Barsky clipping), plus the cells
    /// containing the two end points.
    fn oracle(p0: Vector2<f64>, p1: Vector2<f64>) -> BTreeSet<(i32, i32)> {
        let mut out = BTreeSet::new();
        out.insert((p0.x.floor() as i32, p0.y.floor() as i32));
        out.insert((p1.x.floor() as i32, p1.y.floor() as i32));
        let d = p1 - p0;
        let (xlo, xhi) = (p0.x.min(p1.x).floor() as i32, p0.x.max(p1.x).floor() as i32);
        let (ylo, yhi) = (p0.y.min(p1.y).floor() as i32, p0.y.max(p1.y).floor() as i32);
        for i in xlo..=xhi {
            for j in ylo..=yhi {
                let (mut t0, mut t1) = (0.0f64, 1.0f64);
                let mut inside = true;
                for (p, dd, lo, hi) in [
                    (p0.x, d.x, i as f64, (i + 1) as f64),
                    (p0.y, d.y, j as f64, (j + 1) as f64),
                ] {
                    if dd == 0.0 {
                        if p <= lo || p >= hi {
                            inside = false;
                        }
                    } else {
                        let (a, b) = ((lo - p) / dd, (hi - p) / dd);
                        let (a, b) = if a < b { (a, b) } else { (b, a) };
                        t0 = t0.max(a);
                        t1 = t1.min(b);
                    }
                }
                if inside && t1 - t0 > 1e-12 {
                    out.insert((i, j));
                }
            }
        }
        out
    }

    proptest! {
        /// M3: the traversal visits exactly the cells the segment crosses, in order, each once,
        /// with 4-connected steps (random segments do not pass through lattice corners).
        #[test]
        fn supercover_visits_exactly_the_crossed_cells(
            x0 in -8.0f64..8.0, y0 in -8.0f64..8.0, x1 in -8.0f64..8.0, y1 in -8.0f64..8.0
        ) {
            let (p0, p1) = (v(x0, y0), v(x1, y1));
            let cells = unit_supercover(p0, p1);
            let set: BTreeSet<_> = cells.iter().copied().collect();
            prop_assert_eq!(set.len(), cells.len(), "a cell was visited twice");
            prop_assert_eq!(&set, &oracle(p0, p1));
            prop_assert_eq!(cells[0], (x0.floor() as i32, y0.floor() as i32));
            prop_assert_eq!(*cells.last().unwrap(), (x1.floor() as i32, y1.floor() as i32));
            for w in cells.windows(2) {
                let manhattan = (w[1].0 - w[0].0).abs() + (w[1].1 - w[0].1).abs();
                prop_assert_eq!(manhattan, 1);
            }
        }

        /// Reversing the segment visits the same set of cells, so a beam traced outward and
        /// the same beam traced inward agree on what it crossed.
        #[test]
        fn supercover_is_symmetric_under_reversal(
            x0 in -8.0f64..8.0, y0 in -8.0f64..8.0, x1 in -8.0f64..8.0, y1 in -8.0f64..8.0
        ) {
            let (p0, p1) = (v(x0, y0), v(x1, y1));
            let a: BTreeSet<_> = unit_supercover(p0, p1).into_iter().collect();
            let b: BTreeSet<_> = unit_supercover(p1, p0).into_iter().collect();
            prop_assert_eq!(a, b);
        }

        /// The supercover is never shorter than the Chebyshev distance between its end cells
        /// plus one, which is the length of a Bresenham line between the same cells.
        #[test]
        fn supercover_is_never_shorter_than_the_chebyshev_distance(
            x0 in -8.0f64..8.0, y0 in -8.0f64..8.0, x1 in -8.0f64..8.0, y1 in -8.0f64..8.0
        ) {
            let n = unit_supercover(v(x0, y0), v(x1, y1)).len() as i32;
            let (a, b) = ((x0.floor() as i32, y0.floor() as i32), (x1.floor() as i32, y1.floor() as i32));
            let cheb = (a.0 - b.0).abs().max((a.1 - b.1).abs()) + 1;
            prop_assert!(n >= cheb);
        }
    }
}
