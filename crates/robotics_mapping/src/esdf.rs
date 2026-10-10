//! The signed Euclidean distance field over the occupied cells.
//!
//! Reference: Chapter 19 (ESDF); the exact Euclidean distance transform of Felzenszwalb and
//! Huttenlocher (2012) as implemented, once, in
//! [`robotics_models::sensor::likelihood_field::DistanceGrid`], which this module calls and
//! does not duplicate.
//!
//! A planner needs two things a bare occupancy grid does not say: how far a cell is from the
//! nearest obstacle (to keep clear of it), and, for a cell that *is* inside an obstacle, how
//! deep (to get out by the shortest way). The unsigned transform of the occupied mask answers
//! the first. The same transform of the complement answers the second. Joined with a sign:
//!
//! * outside (not occupied): `+(d_out - r/2)`, `d_out` the centre-to-centre distance to the
//!   nearest occupied cell and `r` the resolution;
//! * inside (occupied): `-(d_in - r/2)`, `d_in` the centre-to-centre distance to the nearest
//!   cell that is not occupied.
//!
//! The `r/2` puts the zero level at the **cell faces**, where the surface is, and not at the
//! cell centres. Without it a free cell next to a wall would read `+r` and the wall cell
//! beside it `-r`, a gap of `2r` across a surface that is `r` wide, and every distance would
//! be biased by half a cell on both sides. With it, the two cells either side of a face read
//! `+r/2` and `-r/2`.
//!
//! **Unknown cells are treated as free space** (not as obstacles): the field is the distance to
//! the nearest *known occupied* cell. A consumer that must not plan through the unknown
//! combines the field with the three-state map; this module does not decide that for it.
//!
//! Values saturate at `+/- truncation`, which is also what a map with no obstacle at all reports
//! (a planner only ever needs the distance up to its clearance radius).

use crate::occupancy::{GridGeometry, OccupancyGrid};
use crate::MappingError;
use robotics_core::nalgebra::Vector2;
use robotics_models::sensor::likelihood_field::DistanceGrid;

/// A signed ESDF over the geometry of the grid it was computed from.
#[derive(Debug, Clone, PartialEq)]
pub struct EsdfGrid {
    geometry: GridGeometry,
    truncation: f64,
    values: Vec<f32>,
}

impl EsdfGrid {
    /// The geometry, identical to the occupancy grid's (the contract of decision D12).
    pub fn geometry(&self) -> &GridGeometry {
        &self.geometry
    }

    /// The saturation distance (m).
    pub fn truncation(&self) -> f64 {
        self.truncation
    }

    /// Signed distances in metres, row-major with `j` as the slow index: positive in free
    /// space, negative inside an obstacle.
    pub fn values(&self) -> &[f32] {
        &self.values
    }

    /// The value at cell `(i, j)`; `None` outside the grid.
    pub fn value(&self, i: usize, j: usize) -> Option<f32> {
        self.geometry.index(i, j).map(|k| self.values[k])
    }

    /// The value of the cell containing a world point; `None` outside the grid.
    pub fn at(&self, p: &Vector2<f64>) -> Option<f32> {
        self.geometry
            .world_to_cell(p)
            .and_then(|(i, j)| self.value(i, j))
    }
}

/// The signed ESDF of the occupied cells of `grid`, saturating at `+/- truncation` metres.
pub fn signed_esdf(grid: &OccupancyGrid, truncation: f64) -> Result<EsdfGrid, MappingError> {
    if !(truncation.is_finite() && truncation > 0.0) {
        return Err(MappingError::InvalidParameter {
            what: "esdf truncation",
            value: truncation,
        });
    }
    let g = *grid.geometry();
    let occupied = grid.occupied_mask();
    let not_occupied: Vec<bool> = occupied.iter().map(|&o| !o).collect();
    let model_err = |e: robotics_models::ModelError| MappingError::Parse(e.to_string());
    let outside =
        DistanceGrid::from_occupancy(&occupied, g.width, g.height, g.resolution, g.origin)
            .map_err(model_err)?;
    let inside =
        DistanceGrid::from_occupancy(&not_occupied, g.width, g.height, g.resolution, g.origin)
            .map_err(model_err)?;

    let half = g.resolution / 2.0;
    let mut values = Vec::with_capacity(g.cells());
    for j in 0..g.height {
        for i in 0..g.width {
            let signed = if occupied[j * g.width + i] {
                // `inside.cell` is infinite when no cell is free: the whole map is an obstacle.
                -(inside.cell(i, j).expect("in range") - half)
            } else {
                outside.cell(i, j).expect("in range") - half
            };
            // `clamp` maps the infinite cases to the truncation distance.
            values.push(signed.clamp(-truncation, truncation) as f32);
        }
    }
    Ok(EsdfGrid {
        geometry: g,
        truncation,
        values,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::occupancy::OccupancyParams;
    use approx::assert_relative_eq;
    use proptest::prelude::*;

    const RES: f64 = 0.1;

    fn grid(w: usize, h: usize, occupied: &[(usize, usize)]) -> OccupancyGrid {
        let mut g = OccupancyGrid::new(
            GridGeometry {
                width: w,
                height: h,
                resolution: RES,
                origin: Vector2::new(-0.2, 0.3),
            },
            OccupancyParams::default(),
        )
        .unwrap();
        let l = g.params().l_occ;
        for &(i, j) in occupied {
            // Two hits: one is not enough to call a cell occupied (see OccupancyParams).
            g.add_evidence(i, j, l);
            g.add_evidence(i, j, l);
        }
        g
    }

    /// Worked example, by hand, one occupied cell at (2, 2) of a 5 by 5 grid of 0.1 m cells.
    /// The occupied cell reads `-(0.1 - 0.05) = -0.05` (its nearest free neighbour is one cell
    /// away); a free 4-neighbour `0.1 - 0.05 = 0.05`; a free diagonal neighbour
    /// `0.1 sqrt 2 - 0.05 = 0.0914214`; two cells away `0.2 - 0.05 = 0.15`.
    #[test]
    fn worked_example_a_single_occupied_cell() {
        let e = signed_esdf(&grid(5, 5, &[(2, 2)]), 10.0).unwrap();
        let v = |i, j| e.value(i, j).unwrap() as f64;
        assert_relative_eq!(v(2, 2), -0.05, epsilon = 1e-6);
        assert_relative_eq!(v(3, 2), 0.05, epsilon = 1e-6);
        assert_relative_eq!(v(2, 1), 0.05, epsilon = 1e-6);
        assert_relative_eq!(v(3, 3), 0.1 * 2f64.sqrt() - 0.05, epsilon = 1e-6);
        assert_relative_eq!(v(0, 2), 0.15, epsilon = 1e-6);
        assert_relative_eq!(v(4, 4), 0.2 * 2f64.sqrt() - 0.05, epsilon = 1e-6);
    }

    /// A wall three cells thick reads how deep a cell is inside it: -0.5 r on each face and
    /// -1.5 r in the middle. The two cells either side of a face are +/- r/2.
    #[test]
    fn inside_distance_is_depth_and_the_zero_level_is_the_face() {
        let wall: Vec<_> = (0..3).flat_map(|j| (3..6).map(move |i| (i, j))).collect();
        let e = signed_esdf(&grid(9, 3, &wall), 10.0).unwrap();
        let v = |i, j| e.value(i, j).unwrap() as f64;
        assert_relative_eq!(v(3, 1), -0.05, epsilon = 1e-6);
        assert_relative_eq!(v(4, 1), -0.15, epsilon = 1e-6);
        assert_relative_eq!(v(5, 1), -0.05, epsilon = 1e-6);
        assert_relative_eq!(v(2, 1), 0.05, epsilon = 1e-6);
        assert_relative_eq!(v(6, 1), 0.05, epsilon = 1e-6);
        assert_relative_eq!(v(2, 1), -v(3, 1), epsilon = 1e-6, max_relative = 1e-6);
    }

    #[test]
    fn unknown_cells_count_as_free_space() {
        let e = signed_esdf(&grid(5, 5, &[(2, 2)]), 10.0).unwrap();
        // Every cell but the occupied one is unknown in this grid, and positive.
        let positive = e.values().iter().filter(|&&x| x > 0.0).count();
        assert_eq!(positive, 24);
    }

    #[test]
    fn truncation_saturates_and_a_map_without_obstacles_is_all_truncation() {
        let e = signed_esdf(&grid(5, 5, &[(2, 2)]), 0.12).unwrap();
        assert_eq!(e.value(0, 2), Some(0.12));
        assert_eq!(e.value(4, 4), Some(0.12));
        assert_relative_eq!(e.value(3, 2).unwrap() as f64, 0.05, epsilon = 1e-6);
        let none = signed_esdf(&grid(4, 4, &[]), 1.5).unwrap();
        assert!(none.values().iter().all(|&x| x == 1.5));
        let all: Vec<_> = (0..4).flat_map(|j| (0..4).map(move |i| (i, j))).collect();
        let full = signed_esdf(&grid(4, 4, &all), 1.5).unwrap();
        assert!(full.values().iter().all(|&x| x == -1.5));
    }

    #[test]
    fn lookup_by_world_point_and_geometry_match_the_grid() {
        let g = grid(5, 5, &[(2, 2)]);
        let e = signed_esdf(&g, 10.0).unwrap();
        assert_eq!(e.geometry(), g.geometry());
        // The centre of cell (2, 2) is at origin + 2.5 r = (0.05, 0.55).
        assert_relative_eq!(
            e.at(&Vector2::new(0.05, 0.55)).unwrap() as f64,
            -0.05,
            epsilon = 1e-6
        );
        assert_eq!(e.at(&Vector2::new(5.0, 5.0)), None);
        assert!(signed_esdf(&g, 0.0).is_err());
        assert!(signed_esdf(&g, f64::NAN).is_err());
    }

    proptest! {
        /// The field agrees with the definition computed by brute force over every pair of
        /// cells, on random occupancy patterns.
        #[test]
        fn matches_the_brute_force_definition(
            mask in proptest::collection::vec(any::<bool>(), 7 * 6)
        ) {
            let cells: Vec<(usize, usize)> = (0..6)
                .flat_map(|j| (0..7).map(move |i| (i, j)))
                .filter(|&(i, j)| mask[j * 7 + i])
                .collect();
            let g = grid(7, 6, &cells);
            let e = signed_esdf(&g, 100.0).unwrap();
            for j in 0..6usize {
                for i in 0..7usize {
                    let occ = mask[j * 7 + i];
                    let nearest = (0..6usize)
                        .flat_map(|jj| (0..7usize).map(move |ii| (ii, jj)))
                        .filter(|&(ii, jj)| mask[jj * 7 + ii] != occ)
                        .map(|(ii, jj)| {
                            (((ii as f64 - i as f64).powi(2) + (jj as f64 - j as f64).powi(2)).sqrt()) * RES
                        })
                        .fold(f64::INFINITY, f64::min);
                    let want = if nearest.is_infinite() {
                        if occ { -100.0 } else { 100.0 }
                    } else if occ {
                        -(nearest - RES / 2.0)
                    } else {
                        nearest - RES / 2.0
                    };
                    prop_assert!((e.value(i, j).unwrap() as f64 - want).abs() < 1e-5,
                        "cell ({}, {}): {} vs {}", i, j, e.value(i, j).unwrap(), want);
                }
            }
        }
    }
}
