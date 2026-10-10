//! Entropy of the map.
//!
//! Each cell is a Bernoulli variable with occupancy probability `p`, whose entropy in bits is
//! `H(p) = -p log2 p - (1 - p) log2 (1 - p)`. The entropy of the grid is the sum over cells,
//! which is the entropy of the map belief *under the product assumption* of
//! [`crate::occupancy`]: two neighbouring cells of one wall are not independent, so this is an
//! upper bound on the entropy of the true joint belief and a progress measure, not a bound on
//! anything a planner needs.
//!
//! An unobserved cell has `p = 1/2` and contributes exactly one bit, the maximum. A cell the
//! map is sure of contributes close to zero. So the entropy of a fresh grid is its cell count,
//! and exploration lowers it.
//!
//! **Entropy is never a quality measure on its own.** A confidently wrong map has low entropy.
//! This is why [`GridEntropy`] reports the number of decided cells next to the bits, and why
//! the benchmark of `docs/phase-4-mapping/benchmark.md` section 2 prints the entropy trace
//! beside the accuracy metrics of the same run.

use crate::occupancy::{CellState, OccupancyGrid};

/// Entropy of one cell with the given log-odds, in bits.
///
/// Exactly `1.0` for log-odds `0` and exactly `0.0` for an infinite log-odds. Symmetric:
/// `H(l) = H(-l)`.
pub fn cell_entropy_bits(log_odds: f32) -> f64 {
    let l = log_odds as f64;
    if l == 0.0 {
        return 1.0;
    }
    if !l.is_finite() {
        return 0.0;
    }
    let p = 1.0 / (1.0 + (-l).exp());
    let q = 1.0 - p;
    // p or q can underflow to 0 for a very large |l|; 0 log 0 is 0.
    let term = |x: f64| if x > 0.0 { -x * x.log2() } else { 0.0 };
    term(p) + term(q)
}

/// The entropy of a grid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridEntropy {
    /// Sum of the cell entropies over every cell (bits).
    pub total_bits: f64,
    /// Sum over the decided cells only: free or occupied under the grid's thresholds (bits).
    pub decided_bits: f64,
    /// Number of cells.
    pub cells: usize,
    /// Number of decided cells.
    pub decided_cells: usize,
}

/// Entropy of a grid, total and over its decided cells.
pub fn grid_entropy(grid: &OccupancyGrid) -> GridEntropy {
    let (mut total, mut decided_bits, mut decided_cells) = (0.0f64, 0.0f64, 0usize);
    for &l in grid.log_odds_slice() {
        let h = cell_entropy_bits(l);
        total += h;
        if grid.classify(l) != CellState::Unknown {
            decided_bits += h;
            decided_cells += 1;
        }
    }
    GridEntropy {
        total_bits: total,
        decided_bits,
        cells: grid.log_odds_slice().len(),
        decided_cells,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::occupancy::{GridGeometry, OccupancyParams};
    use approx::assert_relative_eq;
    use robotics_core::nalgebra::Vector2;

    fn grid(w: usize, h: usize) -> OccupancyGrid {
        OccupancyGrid::new(
            GridGeometry {
                width: w,
                height: h,
                resolution: 0.05,
                origin: Vector2::new(0.0, 0.0),
            },
            OccupancyParams::default(),
        )
        .unwrap()
    }

    /// An unobserved cell contributes exactly one bit, so a fresh grid has exactly as many
    /// bits of entropy as cells, and no decided cell.
    #[test]
    fn an_unobserved_cell_is_exactly_one_bit() {
        assert_eq!(cell_entropy_bits(0.0), 1.0);
        let e = grid_entropy(&grid(140, 140));
        assert_eq!(e.total_bits, 19_600.0);
        assert_eq!(e.cells, 19_600);
        assert_eq!((e.decided_cells, e.decided_bits), (0, 0.0));
    }

    /// Worked example, by hand: the entropy of a cell at probability 0.7 is
    /// -0.7 log2 0.7 - 0.3 log2 0.3 = 0.360201 + 0.521090 = 0.881291 bits.
    #[test]
    fn worked_example_a_cell_at_seven_tenths() {
        let l = (0.7f64 / 0.3).ln() as f32;
        assert_relative_eq!(
            cell_entropy_bits(l),
            0.881_290_899_230_692_6,
            epsilon = 1e-6
        );
    }

    #[test]
    fn entropy_is_symmetric_bounded_and_falls_with_confidence() {
        let mut prev = 1.0;
        for k in 1..60 {
            let l = k as f32 * 0.2;
            let (h, hm) = (cell_entropy_bits(l), cell_entropy_bits(-l));
            assert_relative_eq!(h, hm, epsilon = 1e-12);
            assert!((0.0..=1.0).contains(&h));
            assert!(h < prev, "H must fall as |l| grows: l = {l}");
            prev = h;
        }
        assert_eq!(cell_entropy_bits(f32::INFINITY), 0.0);
        assert_eq!(cell_entropy_bits(f32::NEG_INFINITY), 0.0);
        assert!(cell_entropy_bits(120.0) >= 0.0, "no NaN from underflow");
    }

    #[test]
    fn observing_a_cell_lowers_the_total_and_counts_it_as_decided() {
        let p = OccupancyParams::default();
        let mut g = grid(10, 10);
        let before = grid_entropy(&g);
        for _ in 0..3 {
            g.add_evidence(4, 4, p.l_occ);
        }
        let after = grid_entropy(&g);
        assert!(after.total_bits < before.total_bits);
        assert_eq!(after.decided_cells, 1);
        // Decided bits are that one cell's entropy, and the rest is the 99 unknown cells.
        assert_relative_eq!(after.total_bits, after.decided_bits + 99.0, epsilon = 1e-9);
        assert_relative_eq!(
            after.decided_bits,
            cell_entropy_bits(g.log_odds(4, 4).unwrap()),
            epsilon = 1e-12
        );
    }

    /// A cell the map is wrong about can have lower entropy than an honest unknown: entropy
    /// alone cannot tell a good map from a confidently wrong one.
    #[test]
    fn a_confidently_wrong_cell_has_low_entropy() {
        let p = OccupancyParams::default();
        let mut g = grid(2, 1);
        for _ in 0..20 {
            g.add_evidence(0, 0, p.l_occ); // "occupied", whatever the truth is
        }
        let e = grid_entropy(&g);
        assert!(cell_entropy_bits(g.log_odds(0, 0).unwrap()) < 0.25);
        assert_eq!(e.decided_cells, 1);
    }
}
