//! The 2-D normal distributions transform (NDT): scan registration against a map of Gaussians.
//!
//! The target points are binned into square cells of side `cell`. A cell with at least
//! `min_points` points becomes a Gaussian `N(μ, Σ)` (sample mean and covariance). A source point
//! `p` placed at `x = T·p` is scored against the Gaussians of the 3×3 block of cells around the
//! cell of `x`, so a point near a cell border is not judged by one Gaussian alone. Unlike ICP no
//! correspondences are searched: the map is a smooth function of position and the registration
//! is a plain Newton-type minimization of that function over the pose.
//!
//! # The score
//! With the Magnusson constants of a 2-D cell (`p0` = `outlier_ratio`),
//!
//! ```text
//! c1 = 10 (1 − p0)      c2 = p0 / cell²      d3 = −ln c2
//! d1 = −ln(c1 + c2) − d3      d2 = −2 ln((−ln(c1 e^{−1/2} + c2) − d3) / d1)
//! ```
//!
//! one (point, cell) pair contributes `d1 · exp(−(d2/2) dᵀ Σ⁻¹ d)` with `d = x − μ`, and the
//! objective is `S(T) = Σ_points Σ_cells d1 · exp(…)`. Because `c1 + c2 > c2` the constant `d1` is
//! negative, so `S` is the negated likelihood-like score of Magnusson's thesis: it is minimized,
//! it is `d1 < 0` at a perfect fit (`d = 0`) and it tends to `0` for a point that no Gaussian
//! covers. The unit-amplitude simplification (`d1 = −1`, `d2 = 1`) is deliberately not used.
//!
//! # Gradient and Hessian
//! The pose is perturbed on the right, `T ⊞ δ = T·exp(δ)` with `δ = (v_x, v_y, ω)`. To first
//! order `exp(δ)·p ≈ p + v + ω J p` with `J = [[0, −1], [1, 0]]`, hence
//! `x(δ) = T·exp(δ)·p` has the Jacobian `B = ∂x/∂δ = R_T · [ I₂ | J p ]` (2×3). For one pair
//! with `e = exp(−(d2/2) dᵀ Σ⁻¹ d)`, `w = −d1 > 0` and `u = Bᵀ Σ⁻¹ d`:
//!
//! ```text
//! s = −w e         ∂s/∂δ = w d2 e u         ∂²s/∂δ² = w d2 e (Bᵀ Σ⁻¹ B − d2 u uᵀ + second-order x terms)
//! ```
//!
//! The gradient `g` is the exact sum of the first. The Hessian returned is the sum of
//! `H = w d2 e · Bᵀ Σ⁻¹ B` only. Both dropped parts can make a term indefinite: `−d2 u uᵀ` is
//! negative semi-definite, and the second derivative of `x(δ)` contracted with `Σ⁻¹ d` has no
//! sign. What is kept is a sum of positive semi-definite matrices with non-negative weights, so
//! `H + λI` is positive definite for `λ > 0` and `−(H + λI)⁻¹ g` is a descent direction. The
//! price is a linear instead of a quadratic rate near the optimum, which the stopping rule
//! absorbs.
//!
//! # Honest limits
//! The Gaussians of a wall cell are very thin (the smaller standard deviation is at least
//! `√eig_ratio_floor` times the larger), so the capture range of a single resolution is of the
//! order of the cell side for translation and a few degrees for rotation of a scan with
//! far-away structure. Along a corridor nothing constrains the translation along it; the
//! Hessian shows it (small eigenvalue) and [`register`] does not hide it.

use crate::SlamError;
use nalgebra::{Matrix2, Matrix2x3, Matrix3, SymmetricEigen, Vector2, Vector3};
use robotics_core::{Manifold, Se2};
use std::collections::BTreeMap;

/// Smallest allowed larger eigenvalue of a cell covariance (m²), so that a cell of coincident
/// points still has a finite inverse.
const MIN_VARIANCE: f64 = 1e-8;

/// Largest number of damping increases tried for one iteration of [`register`].
const MAX_RETRIES: usize = 10;

/// Parameters of the NDT map and of the registration.
#[derive(Debug, Clone, PartialEq)]
pub struct NdtConfig {
    /// Cell side (m).
    pub cell: f64,
    /// Fewest points a cell needs to become a Gaussian (≥ 3, a covariance needs a spread).
    pub min_points: usize,
    /// Floor of the smaller covariance eigenvalue as a fraction of the larger, in `(0, 1]`.
    /// A cell on a straight wall is otherwise rank one.
    pub eig_ratio_floor: f64,
    /// Assumed outlier ratio `p0` in `(0, 1)` of the Magnusson score.
    pub outlier_ratio: f64,
    /// Most Newton iterations of [`register`].
    pub max_iterations: usize,
    /// Translation step (m) below which [`register`] has converged (together with the rotation).
    pub step_tol_trans: f64,
    /// Rotation step (rad) below which [`register`] has converged (together with the translation).
    pub step_tol_rot: f64,
    /// Initial Levenberg-Marquardt damping `λ`.
    pub lm_lambda0: f64,
}

impl Default for NdtConfig {
    fn default() -> Self {
        Self {
            cell: 0.5,
            min_points: 5,
            eig_ratio_floor: 0.01,
            outlier_ratio: 0.1,
            max_iterations: 50,
            step_tol_trans: 1e-4,
            step_tol_rot: 1e-4,
            lm_lambda0: 1e-3,
        }
    }
}

impl NdtConfig {
    /// Check every parameter; the first violation is reported.
    pub fn validate(&self) -> Result<(), SlamError> {
        let bad = |what: &'static str, value: f64| Err(SlamError::InvalidParameter { what, value });
        if !(self.cell.is_finite() && self.cell > 0.0) {
            return bad("ndt cell", self.cell);
        }
        if self.min_points < 3 {
            return bad("ndt min_points", self.min_points as f64);
        }
        if !(self.eig_ratio_floor > 0.0 && self.eig_ratio_floor <= 1.0) {
            return bad("ndt eig_ratio_floor", self.eig_ratio_floor);
        }
        if !(self.outlier_ratio > 0.0 && self.outlier_ratio < 1.0) {
            return bad("ndt outlier_ratio", self.outlier_ratio);
        }
        if self.max_iterations == 0 {
            return bad("ndt max_iterations", 0.0);
        }
        if !(self.step_tol_trans.is_finite() && self.step_tol_trans > 0.0) {
            return bad("ndt step_tol_trans", self.step_tol_trans);
        }
        if !(self.step_tol_rot.is_finite() && self.step_tol_rot > 0.0) {
            return bad("ndt step_tol_rot", self.step_tol_rot);
        }
        if !(self.lm_lambda0.is_finite() && self.lm_lambda0 > 0.0) {
            return bad("ndt lm_lambda0", self.lm_lambda0);
        }
        Ok(())
    }
}

/// The Magnusson constants `(d1, d2)` of a 2-D cell of side `cell` and outlier ratio `p0`.
fn score_constants(cell: f64, p0: f64) -> (f64, f64) {
    let c1 = 10.0 * (1.0 - p0);
    let c2 = p0 / (cell * cell);
    let d3 = -c2.ln();
    let d1 = -(c1 + c2).ln() - d3;
    let d2 = -2.0 * ((-(c1 * (-0.5f64).exp() + c2).ln() - d3) / d1).ln();
    (d1, d2)
}

/// One Gaussian of the map.
#[derive(Debug, Clone, Copy, PartialEq)]
struct NdtCell {
    mean: Vector2<f64>,
    /// `Σ⁻¹` after the eigenvalue floor.
    info: Matrix2<f64>,
}

impl NdtCell {
    /// Mean, sample covariance, eigenvalue floor, inverse.
    fn fit(points: &[Vector2<f64>], eig_ratio_floor: f64) -> Self {
        let n = points.len() as f64;
        let mean = points.iter().sum::<Vector2<f64>>() / n;
        let scatter = points.iter().fold(Matrix2::zeros(), |s, p| {
            let d = p - mean;
            s + d * d.transpose()
        });
        let cov = scatter / (n - 1.0);
        let eig = SymmetricEigen::new(cov);
        let (big, small) = if eig.eigenvalues[0] >= eig.eigenvalues[1] {
            (0, 1)
        } else {
            (1, 0)
        };
        let lambda_big = eig.eigenvalues[big].max(MIN_VARIANCE);
        let lambda_small = eig.eigenvalues[small].max(eig_ratio_floor * lambda_big);
        let (u_big, u_small) = (eig.eigenvectors.column(big), eig.eigenvectors.column(small));
        let info =
            u_big * u_big.transpose() / lambda_big + u_small * u_small.transpose() / lambda_small;
        Self { mean, info }
    }
}

/// A map of Gaussians built from a target scan.
#[derive(Debug, Clone, PartialEq)]
pub struct NdtMap {
    cells: BTreeMap<(i64, i64), NdtCell>,
    cell: f64,
    d1: f64,
    d2: f64,
}

/// The cell index of a point; the saturating cast keeps absurd coordinates in range.
fn cell_index(p: &Vector2<f64>, cell: f64) -> (i64, i64) {
    ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64)
}

impl NdtMap {
    /// Bin `points` into cells of side `cfg.cell` and fit a Gaussian to every cell with at least
    /// `cfg.min_points` points. The cells live in a sorted map, so everything computed from
    /// the map is reproducible bit for bit.
    ///
    /// # Errors
    /// The config is invalid ([`SlamError::InvalidParameter`]), a point is not finite
    /// ([`SlamError::NonFinite`]), or no cell has enough points ([`SlamError::EmptyInput`]).
    pub fn from_points(points: &[Vector2<f64>], cfg: &NdtConfig) -> Result<NdtMap, SlamError> {
        cfg.validate()?;
        if points.iter().any(|p| !(p.x.is_finite() && p.y.is_finite())) {
            return Err(SlamError::NonFinite("ndt target points"));
        }
        let mut bins: BTreeMap<(i64, i64), Vec<Vector2<f64>>> = BTreeMap::new();
        for p in points {
            bins.entry(cell_index(p, cfg.cell)).or_default().push(*p);
        }
        let cells: BTreeMap<_, _> = bins
            .iter()
            .filter(|(_, pts)| pts.len() >= cfg.min_points)
            .map(|(k, pts)| (*k, NdtCell::fit(pts, cfg.eig_ratio_floor)))
            .collect();
        if cells.is_empty() {
            return Err(SlamError::EmptyInput(
                "ndt target: no cell has enough points",
            ));
        }
        let (d1, d2) = score_constants(cfg.cell, cfg.outlier_ratio);
        Ok(NdtMap {
            cells,
            cell: cfg.cell,
            d1,
            d2,
        })
    }

    /// Number of cells that became a Gaussian.
    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    /// Score, gradient and Gauss-Newton Hessian of `S` at `pose`, with respect to the right
    /// perturbation `pose ⊞ δ` (see the module docs for the formulas and what the Hessian omits).
    /// Source points that are not finite are skipped.
    pub fn evaluate(&self, source: &[Vector2<f64>], pose: &Se2) -> NdtEvaluation {
        let rot = pose.rotation().into_inner();
        let w = -self.d1;
        let mut ev = NdtEvaluation {
            score: 0.0,
            gradient: Vector3::zeros(),
            hessian: Matrix3::zeros(),
            matched_points: 0,
        };
        for p in source {
            if !(p.x.is_finite() && p.y.is_finite()) {
                continue;
            }
            let x = pose.act(p);
            // ∂x/∂δ = R_T [ I₂ | J p ] with J p = (−p_y, p_x).
            let jac = rot * Matrix2x3::new(1.0, 0.0, -p.y, 0.0, 1.0, p.x);
            let (ci, cj) = cell_index(&x, self.cell);
            let mut matched = false;
            for i in ci.saturating_sub(1)..=ci.saturating_add(1) {
                for j in cj.saturating_sub(1)..=cj.saturating_add(1) {
                    let Some(c) = self.cells.get(&(i, j)) else {
                        continue;
                    };
                    matched = true;
                    let d = x - c.mean;
                    let info_d = c.info * d;
                    let e = (-0.5 * self.d2 * d.dot(&info_d)).exp();
                    let weight = w * self.d2 * e;
                    ev.score += self.d1 * e;
                    ev.gradient += weight * (jac.transpose() * info_d);
                    ev.hessian += weight * (jac.transpose() * c.info * jac);
                }
            }
            ev.matched_points += usize::from(matched);
        }
        // Each term is symmetric only up to rounding; make the sum exactly so.
        ev.hessian = (ev.hessian + ev.hessian.transpose()) * 0.5;
        ev
    }
}

/// The objective of [`NdtMap::evaluate`] and its derivatives at one pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NdtEvaluation {
    /// `S(T) = Σ d1 · exp(−(d2/2) dᵀ Σ⁻¹ d)`; negative when the scan fits the map.
    pub score: f64,
    /// `∂S/∂δ` for `T ⊞ δ`, order `(v_x, v_y, ω)`.
    pub gradient: Vector3<f64>,
    /// Positive semi-definite Gauss-Newton Hessian of `S` for `T ⊞ δ`.
    pub hessian: Matrix3<f64>,
    /// Source points with at least one map cell in their 3×3 neighborhood.
    pub matched_points: usize,
}

/// Outcome of [`register`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NdtResult {
    /// The registration `T`: source sensor frame to the map frame, `q = T·p`.
    pub pose: Se2,
    /// The step fell below both tolerances while at least one source point had a map cell. This
    /// certifies a local optimum of `S`, not that it is the true pose (a corridor or a start
    /// outside the basin also "converges").
    pub converged: bool,
    /// Iterations run.
    pub iterations: usize,
    /// `S` at `pose`.
    pub score: f64,
    /// Source points with at least one map cell in their neighborhood at `pose`.
    pub matched_points: usize,
}

/// Register `source` (points in the source sensor frame) against `map`, starting at `init`.
///
/// Each iteration solves `(H + λI) Δ = −g` and tries `T ← T ⊞ Δ`. The step is accepted only if
/// `S` decreases, and then `λ ← max(λ/3, 1e-9)`; otherwise `λ ← 10 λ` and the step is
/// recomputed, at most ten times. The iteration stops with `converged = true` when the accepted
/// (or, once the damping has grown so far that nothing is accepted, the last tried) step has
/// `‖Δ_trans‖ < step_tol_trans` and `|Δθ| < step_tol_rot`. If no source point has a map cell
/// nearby the objective is flat and the result is `converged = false`.
///
/// # Errors
/// The config is invalid, the source is empty, or the source or `init` is not finite.
pub fn register(
    source: &[Vector2<f64>],
    map: &NdtMap,
    init: Se2,
    cfg: &NdtConfig,
) -> Result<NdtResult, SlamError> {
    cfg.validate()?;
    if source.is_empty() {
        return Err(SlamError::EmptyInput("ndt source points"));
    }
    if source.iter().any(|p| !(p.x.is_finite() && p.y.is_finite())) {
        return Err(SlamError::NonFinite("ndt source points"));
    }
    if !init.is_finite() {
        return Err(SlamError::NonFinite("ndt initial pose"));
    }
    let mut pose = init;
    let mut ev = map.evaluate(source, &pose);
    let mut lambda = cfg.lm_lambda0;
    let mut iterations = 0;
    let mut small_step = false;
    while iterations < cfg.max_iterations {
        iterations += 1;
        if ev.gradient == Vector3::zeros() {
            break;
        }
        let mut accepted = false;
        for _ in 0..MAX_RETRIES {
            let damped = ev.hessian + Matrix3::identity() * lambda;
            let Some(chol) = damped.cholesky() else {
                lambda *= 10.0;
                continue;
            };
            let delta = chol.solve(&(-ev.gradient));
            small_step =
                delta.xy().norm() < cfg.step_tol_trans && delta[2].abs() < cfg.step_tol_rot;
            let candidate = pose.boxplus(&delta);
            let cand_ev = map.evaluate(source, &candidate);
            if cand_ev.score < ev.score {
                pose = candidate;
                ev = cand_ev;
                lambda = (lambda / 3.0).max(1e-9);
                accepted = true;
                break;
            }
            lambda *= 10.0;
        }
        if small_step || !accepted {
            break;
        }
    }
    Ok(NdtResult {
        pose,
        converged: small_step && ev.matched_points > 0,
        iterations,
        score: ev.score,
        matched_points: ev.matched_points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn cfg() -> NdtConfig {
        NdtConfig::default()
    }

    /// Five points on the line `y = 0.25`, all inside the cell `[0, 0.5)²`.
    fn line_points() -> Vec<Vector2<f64>> {
        [0.1, 0.175, 0.25, 0.325, 0.4]
            .iter()
            .map(|&x| Vector2::new(x, 0.25))
            .collect()
    }

    /// A small L-shaped scene: two walls, dense enough for several cells each, away from the
    /// cell borders at 0 (the walls sit in the middle of a row and a column of cells).
    fn corner_points() -> Vec<Vector2<f64>> {
        let mut pts = Vec::new();
        for k in 0..120 {
            let s = k as f64 * 0.025;
            pts.push(Vector2::new(s, 0.17 + 0.003 * (k % 3) as f64));
            pts.push(Vector2::new(0.13 + 0.002 * (k % 4) as f64, s));
        }
        pts
    }

    #[test]
    fn magnusson_constants_match_the_definition() {
        let map = NdtMap::from_points(&line_points(), &cfg()).unwrap();
        // Plain f64 formulas for cell = 0.5, p0 = 0.1.
        let c1: f64 = 10.0 * (1.0 - 0.1);
        let c2: f64 = 0.1 / (0.5 * 0.5);
        assert_relative_eq!(c1, 9.0, epsilon = 1e-15);
        assert_relative_eq!(c2, 0.4, epsilon = 1e-15);
        let d3 = -c2.ln();
        let d1 = -(c1 + c2).ln() - d3;
        let d2 = -2.0 * ((-(c1 * (-0.5f64).exp() + c2).ln() - d3) / d1).ln();
        assert_eq!(map.d1, d1);
        assert_eq!(map.d2, d2);
        // Hand values: d1 = ln(0.4 / 9.4), d2 = 0.3250 (to four digits).
        assert_relative_eq!(d1, (0.4f64 / 9.4).ln(), epsilon = 1e-12);
        assert!(d1 < 0.0 && d2 > 0.0);
        assert_relative_eq!(d2, 0.3250, epsilon = 1e-3);
    }

    #[test]
    fn cell_statistics_of_points_on_a_line() {
        let map = NdtMap::from_points(&line_points(), &cfg()).unwrap();
        assert_eq!(map.cell_count(), 1);
        let c = map.cells[&(0, 0)];
        assert_relative_eq!(c.mean, Vector2::new(0.25, 0.25), epsilon = 1e-15);
        // Sample covariance diag(0.0140625, 0): the smaller eigenvalue is floored at 1 %.
        let big = 0.0140625;
        let small = 0.01 * big;
        assert_relative_eq!(
            c.info,
            Matrix2::new(1.0 / big, 0.0, 0.0, 1.0 / small),
            epsilon = 1e-9
        );
        let eig = SymmetricEigen::new(c.info.try_inverse().unwrap());
        let (lo, hi) = (eig.eigenvalues.min(), eig.eigenvalues.max());
        assert_relative_eq!(lo, 0.01 * hi, max_relative = 1e-12);
    }

    #[test]
    fn score_at_the_mean_and_one_sigma_away() {
        let map = NdtMap::from_points(&line_points(), &cfg()).unwrap();
        let at_mean = map.evaluate(&[Vector2::new(0.25, 0.25)], &Se2::IDENTITY);
        assert_eq!(at_mean.score, map.d1);
        assert_eq!(at_mean.gradient, Vector3::zeros());
        assert_eq!(at_mean.matched_points, 1);
        // One standard deviation along the major axis: dᵀ Σ⁻¹ d = 1.
        let sigma = 0.0140625f64.sqrt();
        let one = map.evaluate(&[Vector2::new(0.25 + sigma, 0.25)], &Se2::IDENTITY);
        assert_relative_eq!(
            one.score,
            map.d1 * (-0.5 * map.d2).exp(),
            max_relative = 1e-12
        );
    }

    #[test]
    fn sparse_cells_are_dropped() {
        let mut pts = line_points();
        pts.pop(); // 4 points: below min_points = 5
        assert_eq!(
            NdtMap::from_points(&pts, &cfg()).unwrap_err(),
            SlamError::EmptyInput("ndt target: no cell has enough points")
        );
        pts.extend((0..6).map(|k| Vector2::new(2.1 + 0.05 * k as f64, 1.2 + 0.01 * k as f64)));
        let map = NdtMap::from_points(&pts, &cfg()).unwrap();
        assert_eq!(map.cell_count(), 1);
        assert!(map.cells.contains_key(&(4, 2)));
    }

    #[test]
    fn point_beyond_the_neighborhood_is_unmatched() {
        let map = NdtMap::from_points(&line_points(), &cfg()).unwrap();
        let far = map.evaluate(&[Vector2::new(1.3, 0.25)], &Se2::IDENTITY);
        assert_eq!(far.matched_points, 0);
        assert_eq!(far.score, 0.0);
    }

    fn numeric_gradient(map: &NdtMap, src: &[Vector2<f64>], pose: &Se2) -> Vector3<f64> {
        let h = 1e-6;
        let mut g = Vector3::zeros();
        for k in 0..3 {
            let mut d = Vector3::zeros();
            d[k] = h;
            let hi = map.evaluate(src, &pose.boxplus(&d)).score;
            let lo = map.evaluate(src, &pose.boxplus(&-d)).score;
            g[k] = (hi - lo) / (2.0 * h);
        }
        g
    }

    #[test]
    fn gradient_matches_central_differences_and_hessian_is_psd() {
        let target = corner_points();
        let map = NdtMap::from_points(&target, &cfg()).unwrap();
        // Offsets that keep every point away from a cell border: the score is only piecewise
        // smooth, it jumps where a point crosses into a new 3x3 neighborhood.
        let truth = Se2::new(0.0537, 0.0411, 0.0);
        let src: Vec<_> = target.iter().map(|p| truth.act_inverse(p)).collect();
        for pose in [
            Se2::new(0.02, -0.03, 0.04),
            Se2::new(-0.04, 0.06, -0.08),
            Se2::new(0.0013, -0.0007, 0.0),
        ] {
            let ev = map.evaluate(&src, &pose);
            let num = numeric_gradient(&map, &src, &pose);
            let scale = ev.gradient.norm().max(1.0);
            assert!(
                (ev.gradient - num).norm() < 1e-5 * scale,
                "{} vs {}",
                ev.gradient,
                num
            );
            assert_relative_eq!(ev.hessian, ev.hessian.transpose(), epsilon = 1e-12);
            let min = SymmetricEigen::new(ev.hessian).eigenvalues.min();
            assert!(min > -1e-9, "eigenvalue {min}");
        }
    }

    #[test]
    fn registration_recovers_a_known_offset() {
        let target = corner_points();
        let map = NdtMap::from_points(&target, &cfg()).unwrap();
        let truth = Se2::new(0.06, -0.04, 0.05);
        let src: Vec<_> = target.iter().map(|p| truth.act_inverse(p)).collect();
        let r = register(&src, &map, Se2::new(0.02, 0.0, 0.0), &cfg()).unwrap();
        assert!(r.converged, "{r:?}");
        assert!(truth.boxminus(&r.pose).norm() < 5e-3, "{:?}", r.pose);
        assert!(r.score < map.evaluate(&src, &Se2::new(0.02, 0.0, 0.0)).score);
    }

    #[test]
    fn invalid_inputs_are_refused() {
        let map = NdtMap::from_points(&line_points(), &cfg()).unwrap();
        let src = [Vector2::new(0.2, 0.2)];
        assert_eq!(
            register(&[], &map, Se2::IDENTITY, &cfg()).unwrap_err(),
            SlamError::EmptyInput("ndt source points")
        );
        let nan = [Vector2::new(f64::NAN, 0.0)];
        assert_eq!(
            register(&nan, &map, Se2::IDENTITY, &cfg()).unwrap_err(),
            SlamError::NonFinite("ndt source points")
        );
        assert_eq!(
            NdtMap::from_points(&nan, &cfg()).unwrap_err(),
            SlamError::NonFinite("ndt target points")
        );
        let nonfinite_pose = Se2::new(f64::INFINITY, 0.0, 0.0);
        assert!(register(&src, &map, nonfinite_pose, &cfg()).is_err());
        for bad in [
            NdtConfig { cell: 0.0, ..cfg() },
            NdtConfig {
                cell: f64::NAN,
                ..cfg()
            },
            NdtConfig {
                min_points: 2,
                ..cfg()
            },
            NdtConfig {
                eig_ratio_floor: 0.0,
                ..cfg()
            },
            NdtConfig {
                eig_ratio_floor: 1.5,
                ..cfg()
            },
            NdtConfig {
                outlier_ratio: 1.0,
                ..cfg()
            },
            NdtConfig {
                outlier_ratio: 0.0,
                ..cfg()
            },
            NdtConfig {
                max_iterations: 0,
                ..cfg()
            },
            NdtConfig {
                lm_lambda0: 0.0,
                ..cfg()
            },
        ] {
            assert!(
                matches!(bad.validate(), Err(SlamError::InvalidParameter { .. })),
                "{bad:?}"
            );
            assert!(NdtMap::from_points(&line_points(), &bad).is_err());
            assert!(register(&src, &map, Se2::IDENTITY, &bad).is_err());
        }
        assert!(cfg().validate().is_ok());
    }

    #[test]
    fn nothing_nearby_is_not_called_converged() {
        let map = NdtMap::from_points(&line_points(), &cfg()).unwrap();
        let src = [Vector2::new(5.0, 5.0)];
        let r = register(&src, &map, Se2::IDENTITY, &cfg()).unwrap();
        assert!(!r.converged);
        assert_eq!(r.matched_points, 0);
        assert_eq!(r.pose, Se2::IDENTITY);
    }
}
