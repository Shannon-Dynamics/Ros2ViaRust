//! Discrete Bayes filter over a 1-D metric grid.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 5 (`HistogramFilter`) — the
//! hallway robot: a belief over position along a corridor, with a motion kernel and a
//! measurement likelihood.

use crate::bayes::{BayesFilter, FilterError};

/// Belief over `cells` equal-width cells covering `[0, length)`.
///
/// * `predict(u)` shifts the belief by `u` metres and convolves it with a motion kernel
///   (a density over the shift error); with the default one-cell delta kernel the shift
///   is noise-free.
/// * `correct(likelihood)` multiplies by `p(z | x)` evaluated at cell centres.
#[derive(Debug, Clone, PartialEq)]
pub struct HistogramFilter1D {
    cells: Vec<f64>,
    length: f64,
    wrap: bool,
    kernel: Option<Vec<(f64, f64)>>,
}

/// A motion kernel: `(shift_error_m, density)` pairs are sampled from a user function at
/// cell resolution when [`HistogramFilter1D::set_gaussian_kernel`] is used.
impl HistogramFilter1D {
    /// Uniform belief over `cells` cells spanning `length` metres. `wrap` makes the
    /// corridor cyclic (the book's hallway is not).
    pub fn uniform(cells: usize, length: f64, wrap: bool) -> Result<Self, FilterError> {
        if cells == 0 || !(length > 0.0 && length.is_finite()) {
            return Err(FilterError::NonFinite("grid definition"));
        }
        Ok(Self {
            cells: vec![1.0 / cells as f64; cells],
            length,
            wrap,
            kernel: None,
        })
    }

    /// Cell width (m).
    #[inline]
    pub fn cell_width(&self) -> f64 {
        self.length / self.cells.len() as f64
    }

    /// Centre of cell `i` (m).
    #[inline]
    pub fn center(&self, i: usize) -> f64 {
        (i as f64 + 0.5) * self.cell_width()
    }

    /// Number of cells.
    #[inline]
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    /// `true` if the grid has no cells (never, after construction).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    /// Use a Gaussian motion kernel with standard deviation `sigma` (m), truncated at
    /// `±4σ` and sampled at cell resolution. `sigma <= 0` restores the exact-shift kernel.
    pub fn set_gaussian_kernel(&mut self, sigma: f64) {
        if sigma.is_nan() || sigma <= 0.0 {
            self.kernel = None;
            return;
        }
        let dx = self.cell_width();
        let half = ((4.0 * sigma) / dx).ceil() as i64;
        let norm = 1.0 / (sigma * (2.0 * std::f64::consts::PI).sqrt());
        let taps: Vec<(f64, f64)> = (-half..=half)
            .map(|k| {
                let e = k as f64 * dx;
                (e, norm * (-0.5 * e * e / (sigma * sigma)).exp())
            })
            .collect();
        self.kernel = Some(taps);
    }

    /// Posterior mean position (m); for a wrapped grid this is the linear mean.
    pub fn mean(&self) -> f64 {
        self.cells
            .iter()
            .enumerate()
            .map(|(i, p)| p * self.center(i))
            .sum()
    }

    /// Index of the most probable cell (the lowest index on ties).
    pub fn mode(&self) -> usize {
        let mut best = 0;
        for (i, &p) in self.cells.iter().enumerate() {
            if p > self.cells[best] {
                best = i;
            }
        }
        best
    }

    /// Shannon entropy of the belief in bits.
    pub fn entropy_bits(&self) -> f64 {
        -self
            .cells
            .iter()
            .filter(|p| **p > 0.0)
            .map(|p| p * p.log2())
            .sum::<f64>()
    }

    fn normalize(&mut self) -> Result<f64, FilterError> {
        let total: f64 = self.cells.iter().sum();
        if !total.is_finite() || total <= 0.0 {
            return Err(FilterError::ZeroLikelihood);
        }
        for c in &mut self.cells {
            *c /= total;
        }
        Ok(total)
    }

    fn deposit(&mut self, next: &mut [f64], i: usize, shift_cells: f64, mass: f64) {
        let n = self.cells.len() as f64;
        let target = i as f64 + shift_cells;
        let j = target.round();
        let j = if self.wrap { j.rem_euclid(n) } else { j };
        if j >= 0.0 && j < n {
            next[j as usize] += mass;
        }
    }
}

impl BayesFilter for HistogramFilter1D {
    type Belief = Vec<f64>;
    /// Commanded displacement along the corridor (m).
    type Control = f64;
    /// Measurement likelihood `p(z | x)` as a function of position (m).
    type Measurement = Box<dyn Fn(f64) -> f64>;

    fn predict(&mut self, u: &f64) -> Result<(), FilterError> {
        if !u.is_finite() {
            return Err(FilterError::NonFinite("control"));
        }
        let dx = self.cell_width();
        let mut next = vec![0.0; self.cells.len()];
        match self.kernel.clone() {
            None => {
                for i in 0..self.cells.len() {
                    let m = self.cells[i];
                    if m > 0.0 {
                        self.deposit(&mut next, i, u / dx, m);
                    }
                }
            }
            Some(taps) => {
                for i in 0..self.cells.len() {
                    let m = self.cells[i];
                    if m <= 0.0 {
                        continue;
                    }
                    for (e, dens) in &taps {
                        self.deposit(&mut next, i, (u + e) / dx, m * dens * dx);
                    }
                }
            }
        }
        self.cells = next;
        // Mass leaving a non-wrapped corridor is lost; renormalize so the belief stays a
        // distribution (uniform fallback if everything left).
        if self.normalize().is_err() {
            let n = self.cells.len() as f64;
            self.cells.iter_mut().for_each(|c| *c = 1.0 / n);
        }
        Ok(())
    }

    fn correct(&mut self, likelihood: &Self::Measurement) -> Result<f64, FilterError> {
        for i in 0..self.cells.len() {
            let l = likelihood(self.center(i));
            if !l.is_finite() || l < 0.0 {
                return Err(FilterError::NonFinite("likelihood"));
            }
            self.cells[i] *= l;
        }
        self.normalize()
    }

    fn belief(&self) -> &Vec<f64> {
        &self.cells
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Chapter 5 worked example: four 1 m cells, likelihood 0.8 on 1<x<2 or x>3, else 0.2
    /// → evidence 0.5 and belief (0.1, 0.4, 0.1, 0.4).
    #[test]
    fn worked_example_four_cells() {
        let mut f = HistogramFilter1D::uniform(4, 4.0, false).unwrap();
        let lik: Box<dyn Fn(f64) -> f64> = Box::new(|x| {
            if (1.0 < x && x < 2.0) || x > 3.0 {
                0.8
            } else {
                0.2
            }
        });
        let evidence = f.correct(&lik).unwrap();
        assert_relative_eq!(evidence, 0.5, epsilon = 1e-12);
        let b = f.belief();
        for (got, want) in b.iter().zip([0.1, 0.4, 0.1, 0.4]) {
            assert_relative_eq!(*got, want, epsilon = 1e-12);
        }
        assert_eq!(f.mode(), 1);
    }

    /// Exact one-cell shift: the belief moves right by one cell; mass that leaves a
    /// non-wrapped corridor is dropped and the rest renormalized.
    #[test]
    fn exact_shift_moves_mass() {
        let mut f = HistogramFilter1D::uniform(4, 4.0, false).unwrap();
        let lik: Box<dyn Fn(f64) -> f64> = Box::new(|x| if x < 1.0 { 1.0 } else { 0.0 });
        f.correct(&lik).unwrap();
        f.predict(&1.0).unwrap();
        assert_relative_eq!(f.belief()[1], 1.0, epsilon = 1e-12);
        let mut w = HistogramFilter1D::uniform(4, 4.0, true).unwrap();
        let lik: Box<dyn Fn(f64) -> f64> = Box::new(|x| if x > 3.0 { 1.0 } else { 0.0 });
        w.correct(&lik).unwrap();
        w.predict(&1.0).unwrap();
        assert_relative_eq!(w.belief()[0], 1.0, epsilon = 1e-12);
    }

    /// Book check #20: sensing sharpens, moving smears; mass stays 1.
    #[test]
    fn sensing_sharpens_moving_smears() {
        let mut f = HistogramFilter1D::uniform(100, 10.0, false).unwrap();
        let h0 = f.entropy_bits();
        assert_relative_eq!(h0, 100f64.log2(), epsilon = 1e-9);
        let doors = [2.0, 4.5, 7.5];
        let lik: Box<dyn Fn(f64) -> f64> = Box::new(move |x| {
            if doors.iter().any(|d| (x - d).abs() < 0.3) {
                0.9
            } else {
                0.1
            }
        });
        f.correct(&lik).unwrap();
        let h1 = f.entropy_bits();
        assert!(h1 < h0);
        f.set_gaussian_kernel(0.3);
        f.predict(&1.0).unwrap();
        let h2 = f.entropy_bits();
        assert!(h2 > h1);
        assert_relative_eq!(f.belief().iter().sum::<f64>(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn zero_likelihood_everywhere_is_an_error() {
        let mut f = HistogramFilter1D::uniform(4, 4.0, false).unwrap();
        let lik: Box<dyn Fn(f64) -> f64> = Box::new(|_| 0.0);
        assert_eq!(f.correct(&lik), Err(FilterError::ZeroLikelihood));
    }
}
