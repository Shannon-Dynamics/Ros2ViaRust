//! The inverse sensor model and the integration of one laser scan.
//!
//! Reference: Thrun, Burgard & Fox, Table 9.1 (`occupancy_grid_mapping`: every cell in the
//! perceptual field of the measurement is updated once, by the inverse sensor model) and
//! Section 9.2 (the inverse measurement model `p(m_i | z_t, x_t)`).
//!
//! The model is hand-crafted and has no beam width: along one beam, every cell the beam
//! crosses before its end is evidence of free space (`l_free`), and the cell containing the
//! end is evidence of an obstacle (`l_occ`). Nothing is said about cells behind the end:
//! they stay as they were, which is how *unknown* stays unknown. The numbers are those of
//! [`OccupancyParams`].
//!
//! # One update per cell per scan
//!
//! A scan is 360 beams, and the cells near the sensor lie on dozens of them. Adding `l_free`
//! once per beam would saturate those cells on the first scan, and would count a duplicated
//! direction twice (the Webots LDS-01 scan has one: its last beam points where its first
//! does). So the integrator first collects the set of cells the scan found free and the set
//! it found occupied, and only then updates each cell once: `l_occ` for an occupied cell,
//! `l_free` for a free one that no beam ended in. A cell that one beam passes through and
//! another ends in is occupied, because a hit is the stronger statement. The result does not
//! depend on the order of the beams at all, not approximately but bit for bit, since each cell
//! receives exactly one addition.
//!
//! # Beams that returned nothing
//!
//! A beam whose range is `+inf`, or finite and at or beyond `range_max`, says that nothing was
//! within range. With [`NoReturn::ClearToMaxRange`] (the default) the cells it crossed up to
//! `range_max` are free; the cell at `range_max` itself is left alone, as the cell of a hit is
//! marked but the cell of a no-return is only partly observed. With [`NoReturn::Drop`] such a
//! beam is ignored. `NaN`, `-inf`, zero, negative and below-`range_min` ranges carry no
//! information and are always dropped. The default is justified by measurement in
//! `docs/phase-4-mapping/implementation.md` section 5.2: in the Webots world `inf` is
//! reported only for beams whose true range exceeds `range_max`.

use crate::occupancy::{GridGeometry, OccupancyGrid};
use crate::raycast::{Bresenham, Supercover};
use crate::MappingError;
use robotics_core::nalgebra::Vector2;
use robotics_core::{LaserScan, Se2};

/// What to do with a beam that returned nothing within range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NoReturn {
    /// Mark the cells along the beam up to `range_max` free.
    #[default]
    ClearToMaxRange,
    /// Ignore the beam.
    Drop,
}

/// How a beam is walked through the grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RayTracer {
    /// Every cell the continuous segment crosses.
    #[default]
    Supercover,
    /// The integer line between the cell of the sensor and the cell of the end.
    Bresenham,
}

/// What one call to [`ScanIntegrator::integrate`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ScanReport {
    /// Beams in the scan.
    pub beams: usize,
    /// Beams with a return inside `[range_min, range_max)`.
    pub hits: usize,
    /// Of those, the beams whose end fell outside the grid (the cells inside still count as free).
    pub hits_outside_grid: usize,
    /// No-return beams that cleared space.
    pub no_return_cleared: usize,
    /// No-return beams ignored because the policy is [`NoReturn::Drop`].
    pub no_return_dropped: usize,
    /// Beams with `NaN`, a non-positive range, or a range below `range_min`.
    pub invalid: usize,
    /// Cells that received `l_free`.
    pub cells_free: usize,
    /// Cells that received `l_occ`.
    pub cells_occupied: usize,
}

const FREE: u8 = 1;
const OCCUPIED: u8 = 2;

/// Integrates laser scans into an [`OccupancyGrid`]. Holds only scratch space, so one
/// integrator can serve any number of grids of the same size, and allocates once.
#[derive(Debug, Clone, Default)]
pub struct ScanIntegrator {
    no_return: NoReturn,
    tracer: RayTracer,
    marks: Vec<u8>,
    touched: Vec<usize>,
}

fn floor_cell(g: &GridGeometry, p: &Vector2<f64>) -> (i32, i32) {
    let f = |v: f64| (v.floor().clamp(-1e9, 1e9)) as i32;
    (
        f((p.x - g.origin.x) / g.resolution),
        f((p.y - g.origin.y) / g.resolution),
    )
}

impl ScanIntegrator {
    /// An integrator with the given policies.
    pub fn new(no_return: NoReturn, tracer: RayTracer) -> Self {
        Self {
            no_return,
            tracer,
            marks: Vec::new(),
            touched: Vec::new(),
        }
    }

    /// The no-return policy.
    pub fn no_return(&self) -> NoReturn {
        self.no_return
    }

    /// The traversal.
    pub fn tracer(&self) -> RayTracer {
        self.tracer
    }

    #[inline]
    fn mark(&mut self, index: usize, what: u8) {
        let m = &mut self.marks[index];
        if *m == 0 {
            self.touched.push(index);
        }
        if what > *m {
            *m = what;
        }
    }

    /// Trace one beam from `origin_cell` to the world point `end` and mark what it found.
    /// `last_is_hit` says whether the final cell is an obstacle (a hit) or merely the limit of
    /// what was observed (a no-return). Returns whether the end cell was inside the grid.
    fn trace(
        &mut self,
        g: &GridGeometry,
        origin: &Vector2<f64>,
        origin_cell: (i32, i32),
        end: &Vector2<f64>,
        last_is_hit: bool,
    ) -> bool {
        let end_cell = floor_cell(g, end);
        let cells: Box<dyn Iterator<Item = (i32, i32)>> = match self.tracer {
            RayTracer::Supercover => {
                Box::new(Supercover::new(origin, end, &g.origin, g.resolution))
            }
            RayTracer::Bresenham => Box::new(Bresenham::new(origin_cell, end_cell)),
        };
        let (w, h) = (g.width as i64, g.height as i64);
        let mut it = cells.peekable();
        let mut end_inside = false;
        while let Some((i, j)) = it.next() {
            let inside = i >= 0 && j >= 0 && (i as i64) < w && (j as i64) < h;
            if !inside {
                // The grid is a rectangle and the beam started in it: once out, always out.
                break;
            }
            let is_end = it.peek().is_none();
            let index = j as usize * g.width + i as usize;
            if is_end {
                end_inside = true;
                if last_is_hit {
                    self.mark(index, OCCUPIED);
                }
            } else {
                self.mark(index, FREE);
            }
        }
        end_inside
    }

    /// Integrate one scan taken from `sensor`, the pose of the laser frame in the grid's
    /// frame at the scan's stamp. One instant per scan: no deskewing (decision D9).
    pub fn integrate(
        &mut self,
        grid: &mut OccupancyGrid,
        sensor: &Se2,
        scan: &LaserScan,
    ) -> Result<ScanReport, MappingError> {
        let finite = |v: f64| v.is_finite();
        if !(finite(scan.angle_min) && finite(scan.angle_increment)) {
            return Err(MappingError::MalformedScan(
                "non-finite angle_min or angle_increment",
            ));
        }
        if !(finite(scan.range_min)
            && finite(scan.range_max)
            && scan.range_min >= 0.0
            && scan.range_max > scan.range_min)
        {
            return Err(MappingError::MalformedScan(
                "range_min, range_max must be finite and ordered",
            ));
        }
        if !sensor.is_finite() {
            return Err(MappingError::MalformedScan("non-finite sensor pose"));
        }
        let g = *grid.geometry();
        let origin = Vector2::new(sensor.x(), sensor.y());
        let origin_cell = floor_cell(&g, &origin);
        if g.world_to_cell(&origin).is_none() {
            return Err(MappingError::SensorOutsideGrid {
                x: origin.x,
                y: origin.y,
            });
        }
        if self.marks.len() != g.cells() {
            self.marks = vec![0; g.cells()];
        }
        self.touched.clear();

        let mut report = ScanReport {
            beams: scan.ranges.len(),
            ..ScanReport::default()
        };
        for (i, &raw) in scan.ranges.iter().enumerate() {
            let z = raw as f64;
            let theta = sensor.theta() + scan.bearing(i);
            let dir = Vector2::new(theta.cos(), theta.sin());
            if z.is_nan() || z <= 0.0 || (z.is_finite() && z < scan.range_min) {
                report.invalid += 1;
            } else if z.is_finite() && z < scan.range_max {
                report.hits += 1;
                let end = origin + dir * z;
                if !self.trace(&g, &origin, origin_cell, &end, true) {
                    report.hits_outside_grid += 1;
                }
            } else {
                // `+inf`, or a finite range at or beyond the limit.
                match self.no_return {
                    NoReturn::ClearToMaxRange => {
                        report.no_return_cleared += 1;
                        let end = origin + dir * scan.range_max;
                        self.trace(&g, &origin, origin_cell, &end, false);
                    }
                    NoReturn::Drop => report.no_return_dropped += 1,
                }
            }
        }

        let (l_occ, l_free) = (grid.params().l_occ, grid.params().l_free);
        for k in 0..self.touched.len() {
            let index = self.touched[k];
            if self.marks[index] == OCCUPIED {
                grid.add_at(index, l_occ);
                report.cells_occupied += 1;
            } else {
                grid.add_at(index, l_free);
                report.cells_free += 1;
            }
            self.marks[index] = 0;
        }
        self.touched.clear();
        Ok(report)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::occupancy::{CellState, OccupancyParams};
    use robotics_core::{FrameId, Timestamp};

    fn geom(w: usize, h: usize) -> GridGeometry {
        GridGeometry {
            width: w,
            height: h,
            resolution: 1.0,
            origin: Vector2::new(0.0, 0.0),
        }
    }

    fn grid(w: usize, h: usize) -> OccupancyGrid {
        OccupancyGrid::new(geom(w, h), OccupancyParams::default()).unwrap()
    }

    fn scan(angle_min: f64, inc: f64, ranges: &[f32], range_max: f64) -> LaserScan {
        LaserScan {
            stamp: Timestamp::from_secs_f64(1.0),
            frame: FrameId::new("laser"),
            angle_min,
            angle_increment: inc,
            time_increment: 0.0,
            scan_time: 0.2,
            range_min: 0.12,
            range_max,
            ranges: ranges.to_vec(),
            intensities: Vec::new(),
        }
    }

    fn integrator() -> ScanIntegrator {
        ScanIntegrator::new(NoReturn::ClearToMaxRange, RayTracer::Supercover)
    }

    /// Worked example, by hand. A sensor at (0.5, 0.5) facing +x with one beam of range 4.2
    /// ends at (4.7, 0.5): cells (0,0) to (3,0) are crossed, cell (4,0) holds the end. Each of
    /// the four free cells gets `l_free`, the end cell gets `l_occ`, and nothing else moves.
    #[test]
    fn worked_example_one_beam_along_a_row() {
        let mut g = grid(8, 4);
        let p = *g.params();
        let r = integrator()
            .integrate(
                &mut g,
                &Se2::new(0.5, 0.5, 0.0),
                &scan(0.0, 0.0, &[4.2], 6.0),
            )
            .unwrap();
        assert_eq!((r.hits, r.cells_free, r.cells_occupied), (1, 4, 1));
        for i in 0..4 {
            assert_eq!(g.log_odds(i, 0), Some(p.l_free), "cell {i}");
        }
        assert_eq!(g.log_odds(4, 0), Some(p.l_occ));
        let (free, occ, unk) = g.count_states();
        assert_eq!(
            (free, occ),
            (0, 1),
            "one miss is not enough to call a cell free"
        );
        assert_eq!(unk, 32 - 1);
        assert_eq!(g.log_odds(5, 0), Some(0.0), "behind the hit stays unknown");
        assert_eq!(g.log_odds(2, 1), Some(0.0), "off the beam stays unknown");
    }

    /// M3: a cell is updated at most once per scan, however many beams cross it.
    #[test]
    fn a_cell_is_updated_once_per_scan_however_many_beams_cross_it() {
        let p = OccupancyParams::default();
        let mut one = grid(8, 4);
        let mut many = grid(8, 4);
        let sensor = Se2::new(0.5, 0.5, 0.0);
        integrator()
            .integrate(&mut one, &sensor, &scan(0.0, 0.0, &[4.2], 6.0))
            .unwrap();
        integrator()
            .integrate(&mut many, &sensor, &scan(0.0, 0.0, &[4.2; 50], 6.0))
            .unwrap();
        assert!(one.bit_identical(&many));
        assert_eq!(many.log_odds(1, 0), Some(p.l_free));
    }

    /// M3: the order in which the beams are visited does not change the result, bit for bit.
    /// The same (bearing, range) pairs are presented three ways: as given, reversed (the bearing
    /// step changes sign), and rotated by three places (the first bearing moves). The beams
    /// cross each other's cells near the sensor and one ends in a cell another passes through.
    #[test]
    fn beam_order_does_not_change_the_result() {
        let ranges: Vec<f32> = vec![3.3, 1.2, f32::INFINITY, 4.4, 2.1, 5.9, 0.8, 3.0];
        let n = ranges.len();
        let sensor = Se2::new(3.2, 3.7, 0.4);
        let inc = std::f64::consts::TAU / n as f64;
        let first = -3.0;
        let run = |angle_min: f64, step: f64, r: &[f32]| {
            let mut g = grid(10, 10);
            integrator()
                .integrate(&mut g, &sensor, &scan(angle_min, step, r, 5.0))
                .unwrap();
            g
        };
        let as_given = run(first, inc, &ranges);

        let mut reversed = ranges.clone();
        reversed.reverse();
        let b = run(first + inc * (n - 1) as f64, -inc, &reversed);
        assert!(as_given.bit_identical(&b));

        let mut rotated = ranges.clone();
        rotated.rotate_left(3);
        let c = run(first + inc * 3.0, inc, &rotated);
        assert!(as_given.bit_identical(&c));
        assert!(
            as_given.count_states().1 > 0,
            "the scan did mark something occupied"
        );
    }

    #[test]
    fn a_hit_beats_a_pass_through_in_the_same_scan() {
        let p = OccupancyParams::default();
        let mut g = grid(8, 4);
        // Two beams along the same row: a short one ending in cell (2, 0) and a long one
        // passing through it. Two bearings, slightly apart in angle but the same cells.
        let r = integrator()
            .integrate(
                &mut g,
                &Se2::new(0.5, 0.5, 0.0),
                &scan(0.0, 0.0, &[2.2, 5.2], 6.0),
            )
            .unwrap();
        assert_eq!(
            g.log_odds(2, 0),
            Some(p.l_occ),
            "occupied, not occupied plus free"
        );
        assert_eq!(g.log_odds(1, 0), Some(p.l_free));
        assert_eq!(
            g.log_odds(3, 0),
            Some(p.l_free),
            "the long beam still clears beyond it"
        );
        assert_eq!(r.cells_occupied, 2, "cells (2,0) and (5,0)");
    }

    #[test]
    fn no_return_clears_up_to_range_max_and_leaves_the_end_cell_alone() {
        let p = OccupancyParams::default();
        let mut g = grid(8, 4);
        // range_max 3.0 from x = 0.5 ends at x = 3.5, the middle of cell 3.
        let r = integrator()
            .integrate(
                &mut g,
                &Se2::new(0.5, 0.5, 0.0),
                &scan(0.0, 0.0, &[f32::INFINITY], 3.0),
            )
            .unwrap();
        assert_eq!(
            (r.no_return_cleared, r.cells_free, r.cells_occupied),
            (1, 3, 0)
        );
        for i in 0..3 {
            assert_eq!(g.log_odds(i, 0), Some(p.l_free));
        }
        assert_eq!(
            g.log_odds(3, 0),
            Some(0.0),
            "the cell at the limit is only partly observed"
        );
        // A finite range at the limit is the same statement.
        let mut g2 = grid(8, 4);
        integrator()
            .integrate(
                &mut g2,
                &Se2::new(0.5, 0.5, 0.0),
                &scan(0.0, 0.0, &[3.0], 3.0),
            )
            .unwrap();
        assert!(g.bit_identical(&g2));
    }

    #[test]
    fn the_drop_policy_ignores_no_return_beams() {
        let mut g = grid(8, 4);
        let mut integ = ScanIntegrator::new(NoReturn::Drop, RayTracer::Supercover);
        let r = integ
            .integrate(
                &mut g,
                &Se2::new(0.5, 0.5, 0.0),
                &scan(0.0, 0.0, &[f32::INFINITY, 6.0], 3.0),
            )
            .unwrap();
        assert_eq!((r.no_return_dropped, r.no_return_cleared), (2, 0));
        assert_eq!(g.count_states(), (0, 0, 32));
    }

    #[test]
    fn invalid_ranges_carry_no_information() {
        let mut g = grid(8, 4);
        let bad = [f32::NAN, f32::NEG_INFINITY, 0.0, -1.0, 0.05];
        let r = integrator()
            .integrate(&mut g, &Se2::new(0.5, 0.5, 0.0), &scan(0.0, 0.1, &bad, 6.0))
            .unwrap();
        assert_eq!(r.invalid, 5);
        assert_eq!(g.count_states(), (0, 0, 32));
    }

    #[test]
    fn a_hit_outside_the_grid_still_clears_the_cells_inside() {
        let p = OccupancyParams::default();
        let mut g = grid(4, 4);
        // From (0.5, 0.5) along +x a return at 5.2 m ends at x = 5.7, outside a 4-wide grid.
        let r = integrator()
            .integrate(
                &mut g,
                &Se2::new(0.5, 0.5, 0.0),
                &scan(0.0, 0.0, &[5.2], 6.0),
            )
            .unwrap();
        assert_eq!((r.hits, r.hits_outside_grid, r.cells_occupied), (1, 1, 0));
        for i in 0..4 {
            assert_eq!(
                g.log_odds(i, 0),
                Some(p.l_free),
                "all four cells inside are crossed"
            );
        }
    }

    #[test]
    fn a_sensor_outside_the_grid_is_an_error() {
        let mut g = grid(4, 4);
        let e = integrator()
            .integrate(
                &mut g,
                &Se2::new(9.0, 0.5, 0.0),
                &scan(0.0, 0.0, &[1.0], 6.0),
            )
            .unwrap_err();
        assert!(matches!(e, MappingError::SensorOutsideGrid { .. }));
        assert_eq!(g.count_states(), (0, 0, 16));
    }

    #[test]
    fn malformed_scans_are_refused_before_anything_changes() {
        let mut g = grid(4, 4);
        let s = Se2::new(0.5, 0.5, 0.0);
        let mut bad = scan(0.0, 0.0, &[1.0], 6.0);
        bad.angle_increment = f64::NAN;
        assert!(integrator().integrate(&mut g, &s, &bad).is_err());
        let mut bad = scan(0.0, 0.0, &[1.0], 6.0);
        bad.range_max = 0.01;
        assert!(integrator().integrate(&mut g, &s, &bad).is_err());
        assert!(integrator()
            .integrate(
                &mut g,
                &Se2::new(f64::NAN, 0.0, 0.0),
                &scan(0.0, 0.0, &[1.0], 6.0)
            )
            .is_err());
        assert_eq!(g.count_states(), (0, 0, 16));
    }

    #[test]
    fn the_sensor_yaw_rotates_the_beams() {
        let p = OccupancyParams::default();
        let mut g = grid(4, 8);
        // Facing +y (yaw pi/2) with a beam at bearing 0.
        integrator()
            .integrate(
                &mut g,
                &Se2::new(0.5, 0.5, std::f64::consts::FRAC_PI_2),
                &scan(0.0, 0.0, &[3.2], 6.0),
            )
            .unwrap();
        assert_eq!(g.log_odds(0, 3), Some(p.l_occ));
        assert_eq!(g.log_odds(0, 2), Some(p.l_free));
        assert_eq!(g.log_odds(3, 0), Some(0.0));
    }

    /// The two traversals differ on the segment (0.95, 0.95) to (2.05, 1.05) of the raycast
    /// tests: the supercover crosses cell (1, 0) and Bresenham steps over its corner.
    #[test]
    fn the_tracer_decides_whether_a_corner_cell_is_cleared() {
        let sensor = Se2::new(0.95, 0.95, 0.0);
        let d = Vector2::new(1.1f64, 0.1);
        let (range, bearing) = (d.norm() as f32, d.y.atan2(d.x));
        let s = scan(bearing, 0.0, &[range], 6.0);

        let mut a = grid(4, 4);
        ScanIntegrator::new(NoReturn::ClearToMaxRange, RayTracer::Supercover)
            .integrate(&mut a, &sensor, &s)
            .unwrap();
        let mut b = grid(4, 4);
        ScanIntegrator::new(NoReturn::ClearToMaxRange, RayTracer::Bresenham)
            .integrate(&mut b, &sensor, &s)
            .unwrap();
        assert_eq!(a.log_odds(1, 0), Some(a.params().l_free));
        assert_eq!(b.log_odds(1, 0), Some(0.0));
        for g in [&a, &b] {
            assert_eq!(g.state(2, 1), Some(CellState::Occupied));
            assert_eq!(g.log_odds(0, 0), Some(g.params().l_free));
            assert_eq!(g.log_odds(1, 1), Some(g.params().l_free));
        }
    }

    /// The Webots scan convention: 360 beams starting at pi with increment -2 pi / 359, so the
    /// last beam repeats the direction of the first. It must integrate to the same grid as the
    /// same directions listed in the other order, with the duplicate counted once.
    #[test]
    fn the_webots_angle_grid_with_its_duplicate_beam_is_harmless() {
        let n = 360usize;
        let inc = -std::f64::consts::TAU / 359.0;
        // A synthetic room: the range along each direction to a 6 m square wall around the
        // sensor at the centre, computed in closed form.
        let range = |theta: f64| -> f32 {
            let (c, s) = (theta.cos().abs(), theta.sin().abs());
            let t = 3.0 / c.max(s);
            if t > 3.5 {
                f32::INFINITY
            } else {
                t as f32
            }
        };
        let ranges: Vec<f32> = (0..n)
            .map(|i| range(std::f64::consts::PI + i as f64 * inc))
            .collect();
        assert_eq!(ranges[0].to_bits(), ranges[n - 1].to_bits());
        let g0 = GridGeometry {
            width: 140,
            height: 140,
            resolution: 0.05,
            origin: Vector2::new(-3.5, -3.5),
        };
        let sensor = Se2::new(0.01, 0.02, 0.0);
        let mut a = OccupancyGrid::new(g0, OccupancyParams::default()).unwrap();
        integrator()
            .integrate(
                &mut a,
                &sensor,
                &scan(std::f64::consts::PI, inc, &ranges, 3.5),
            )
            .unwrap();
        let mut rev = ranges.clone();
        rev.reverse();
        let mut b = OccupancyGrid::new(g0, OccupancyParams::default()).unwrap();
        integrator()
            .integrate(
                &mut b,
                &sensor,
                &scan(std::f64::consts::PI + (n - 1) as f64 * inc, -inc, &rev, 3.5),
            )
            .unwrap();
        assert!(a.bit_identical(&b));
        let (free, occ, _) = a.count_states();
        assert!(
            occ > 0 && free == 0,
            "one scan: cells are occupied or still unknown"
        );
    }
}
