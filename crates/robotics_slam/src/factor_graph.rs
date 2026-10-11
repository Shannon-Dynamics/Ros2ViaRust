//! A factor graph over SE(2) poses and 2-D point landmarks, and its optimizer.
//!
//! The unknowns are [`Values`]: poses (tangent dimension 3) and landmarks (dimension 2). A
//! [`Factor`] constrains one or two of them: a [`FactorKind::Prior`] on a pose (the gauge), a
//! [`FactorKind::Between`] relative-pose measurement (odometry, scan matching, loop closure),
//! or a [`FactorKind::RangeBearing`] observation of a landmark from a pose. Every factor has an
//! information matrix and a [`RobustKernel`].
//!
//! [`FactorGraph::optimize`] solves the maximum-a-posteriori problem by Gauss-Newton or
//! Levenberg-Marquardt (decision DS9):
//!
//! * `(H + λ·diag H) Δ = −b`, accepted when the cost falls, `λ ← λ·max(1/3, 1 − (2ρ−1)³)` on
//!   the gain ratio `ρ` and `λ ← 10λ` on rejection;
//! * a kernel enters by IRLS: the factor's contribution to `H` and to `b` is multiplied by
//!   `w(e)`, `e` the norm of the whitened residual, and the accepted cost is `Σ ρ(e)`;
//! * the gauge is fixed by a prior factor. If the *undamped* normal equations cannot be
//!   factorized the optimizer returns [`SlamError::SingularSystem`]; it does not nudge the
//!   diagonal until it succeeds, because a missing prior is a bug in the caller;
//! * the linear solve is either a dense `nalgebra` Cholesky (the oracle) or the sparse `faer`
//!   LLT, whose symbolic factorization is built once per optimization. The two agree to 1e-9
//!   on every test graph.
//!
//! [`FactorGraph::marginal_covariance`] returns the joint covariance block of chosen variables
//! (the inverse of the undamped normal equations, restricted), which is what the loop gate of
//! [`crate::pose_graph`] needs and what a compounded-noise approximation only imitates.

use crate::geom::relative_pose_residual;
use crate::kernel::RobustKernel;
use crate::SlamError;
use faer::linalg::solvers::Solve;
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::{Argsort, Pair, SparseColMat, SymbolicSparseColMat};
use faer::{Mat, Side};
use nalgebra::{DMatrix, DVector, Matrix2, Matrix3, Vector2};
use robotics_core::angle::normalize_angle;
use robotics_core::{Manifold, Se2};

/// A variable of the graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum VarKey {
    /// The pose with this index (tangent dimension 3).
    Pose(usize),
    /// The landmark with this index (tangent dimension 2).
    Landmark(usize),
}

/// The current estimate of every variable.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Values {
    /// Pose estimates.
    pub poses: Vec<Se2>,
    /// Landmark estimates.
    pub landmarks: Vec<Vector2<f64>>,
}

impl Values {
    /// Total tangent dimension.
    pub fn dim(&self) -> usize {
        3 * self.poses.len() + 2 * self.landmarks.len()
    }

    fn offset(&self, key: VarKey) -> Result<usize, SlamError> {
        match key {
            VarKey::Pose(i) if i < self.poses.len() => Ok(3 * i),
            VarKey::Landmark(l) if l < self.landmarks.len() => Ok(3 * self.poses.len() + 2 * l),
            other => Err(SlamError::UnknownVariable(format!("{other:?}"))),
        }
    }

    fn retract(&mut self, delta: &DVector<f64>) {
        for (i, p) in self.poses.iter_mut().enumerate() {
            *p = p.boxplus(&nalgebra::Vector3::new(
                delta[3 * i],
                delta[3 * i + 1],
                delta[3 * i + 2],
            ));
        }
        let base = 3 * self.poses.len();
        for (l, p) in self.landmarks.iter_mut().enumerate() {
            p.x += delta[base + 2 * l];
            p.y += delta[base + 2 * l + 1];
        }
    }
}

/// A whitened residual and the whitened Jacobian block of each variable it touches.
type Linearization = (DVector<f64>, Vec<(VarKey, DMatrix<f64>)>);

/// What a factor measures.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FactorKind {
    /// The pose is `measurement`: residual `log(Z⁻¹ X)`.
    Prior {
        /// Pose index.
        pose: usize,
        /// The prior pose `Z`.
        measurement: Se2,
    },
    /// The pose `to` seen from `from` is `measurement`: residual `log(Z⁻¹ X_from⁻¹ X_to)`.
    Between {
        /// Pose index of the frame the measurement is expressed in.
        from: usize,
        /// Pose index of the measured pose.
        to: usize,
        /// The relative pose `Z`.
        measurement: Se2,
    },
    /// Range and bearing of a landmark from a pose: residual `(r̂ − r, wrap(β̂ − β))`.
    RangeBearing {
        /// Observing pose index.
        pose: usize,
        /// Observed landmark index.
        landmark: usize,
        /// Measured range (m).
        range: f64,
        /// Measured bearing in the robot frame (rad).
        bearing: f64,
    },
}

/// A constraint with its noise model and kernel.
#[derive(Debug, Clone, PartialEq)]
pub struct Factor {
    kind: FactorKind,
    /// `Lᵀ` with `Λ = L Lᵀ`: whitens a residual.
    whitener: DMatrix<f64>,
    kernel: RobustKernel,
}

fn whitener(info: &DMatrix<f64>) -> Result<DMatrix<f64>, SlamError> {
    if info.iter().any(|v| !v.is_finite()) {
        return Err(SlamError::NonFinite("factor information"));
    }
    let l = info
        .clone()
        .cholesky()
        .ok_or(SlamError::SingularSystem(
            "factor information is not positive definite",
        ))?
        .l();
    Ok(l.transpose())
}

impl Factor {
    /// A prior on a pose with information `info` (the inverse covariance in the tangent at
    /// the measurement).
    pub fn prior(
        pose: usize,
        measurement: Se2,
        info: &Matrix3<f64>,
        kernel: RobustKernel,
    ) -> Result<Self, SlamError> {
        kernel.validate()?;
        Ok(Self {
            kind: FactorKind::Prior { pose, measurement },
            whitener: whitener(&DMatrix::from_column_slice(3, 3, info.as_slice()))?,
            kernel,
        })
    }

    /// A relative-pose measurement of `to` seen from `from`.
    pub fn between(
        from: usize,
        to: usize,
        measurement: Se2,
        info: &Matrix3<f64>,
        kernel: RobustKernel,
    ) -> Result<Self, SlamError> {
        kernel.validate()?;
        if from == to {
            return Err(SlamError::InvalidParameter {
                what: "between factor joins a pose to itself",
                value: from as f64,
            });
        }
        Ok(Self {
            kind: FactorKind::Between {
                from,
                to,
                measurement,
            },
            whitener: whitener(&DMatrix::from_column_slice(3, 3, info.as_slice()))?,
            kernel,
        })
    }

    /// A range-bearing observation of `landmark` from `pose`, with information `info` in
    /// `(range, bearing)` order.
    pub fn range_bearing(
        pose: usize,
        landmark: usize,
        range: f64,
        bearing: f64,
        info: &Matrix2<f64>,
        kernel: RobustKernel,
    ) -> Result<Self, SlamError> {
        kernel.validate()?;
        if !(range.is_finite() && bearing.is_finite()) {
            return Err(SlamError::NonFinite("range-bearing measurement"));
        }
        Ok(Self {
            kind: FactorKind::RangeBearing {
                pose,
                landmark,
                range,
                bearing,
            },
            whitener: whitener(&DMatrix::from_column_slice(2, 2, info.as_slice()))?,
            kernel,
        })
    }

    /// What the factor measures.
    pub fn kind(&self) -> &FactorKind {
        &self.kind
    }

    /// The kernel.
    pub fn kernel(&self) -> RobustKernel {
        self.kernel
    }

    /// Whitened residual and the Jacobian blocks (also whitened) for each variable it touches.
    fn linearize(&self, values: &Values) -> Result<Linearization, SlamError> {
        let pose = |i: usize| {
            values
                .poses
                .get(i)
                .copied()
                .ok_or_else(|| SlamError::UnknownVariable(format!("Pose({i})")))
        };
        let (e, blocks): Linearization = match self.kind {
            FactorKind::Prior {
                pose: i,
                measurement,
            } => {
                let x = pose(i)?;
                let r = relative_pose_residual(&Se2::IDENTITY, &x, &measurement);
                // log(Z⁻¹ X) depends on X alone: the `j` block of the relative residual.
                (
                    DVector::from_column_slice(r.error.as_slice()),
                    vec![(
                        VarKey::Pose(i),
                        DMatrix::from_column_slice(3, 3, r.jac_j.as_slice()),
                    )],
                )
            }
            FactorKind::Between {
                from,
                to,
                measurement,
            } => {
                let r = relative_pose_residual(&pose(from)?, &pose(to)?, &measurement);
                (
                    DVector::from_column_slice(r.error.as_slice()),
                    vec![
                        (
                            VarKey::Pose(from),
                            DMatrix::from_column_slice(3, 3, r.jac_i.as_slice()),
                        ),
                        (
                            VarKey::Pose(to),
                            DMatrix::from_column_slice(3, 3, r.jac_j.as_slice()),
                        ),
                    ],
                )
            }
            FactorKind::RangeBearing {
                pose: i,
                landmark,
                range,
                bearing,
            } => {
                let x = pose(i)?;
                let l =
                    values.landmarks.get(landmark).copied().ok_or_else(|| {
                        SlamError::UnknownVariable(format!("Landmark({landmark})"))
                    })?;
                let loc = x.act_inverse(&l);
                let r = loc.norm();
                if r < 1e-9 {
                    return Err(SlamError::SingularSystem(
                        "landmark coincides with its observer",
                    ));
                }
                let e = DVector::from_vec(vec![
                    r - range,
                    normalize_angle(loc.y.atan2(loc.x) - bearing),
                ]);
                // d(range, bearing)/d(loc), then d(loc)/d(pose tangent) and d(loc)/d(landmark).
                let d_polar = DMatrix::from_row_slice(
                    2,
                    2,
                    &[loc.x / r, loc.y / r, -loc.y / (r * r), loc.x / (r * r)],
                );
                let d_loc_pose =
                    DMatrix::from_row_slice(2, 3, &[-1.0, 0.0, loc.y, 0.0, -1.0, -loc.x]);
                let rt = x.rotation().inverse();
                let d_loc_l = DMatrix::from_row_slice(
                    2,
                    2,
                    &[rt[(0, 0)], rt[(0, 1)], rt[(1, 0)], rt[(1, 1)]],
                );
                (
                    e,
                    vec![
                        (VarKey::Pose(i), &d_polar * d_loc_pose),
                        (VarKey::Landmark(landmark), &d_polar * d_loc_l),
                    ],
                )
            }
        };
        let blocks = blocks
            .into_iter()
            .map(|(k, j)| (k, &self.whitener * j))
            .collect();
        Ok((&self.whitener * e, blocks))
    }
}

/// Which linear solver the optimizer uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LinearSolver {
    /// Dense `nalgebra` Cholesky: the oracle, `O(n³)`.
    Dense,
    /// Sparse `faer` LLT with the symbolic factorization built once per optimization.
    #[default]
    Sparse,
}

/// Gauss-Newton or Levenberg-Marquardt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Method {
    /// Full steps, no acceptance test.
    GaussNewton,
    /// Damped steps accepted when the cost falls.
    #[default]
    LevenbergMarquardt,
}

/// Optimizer parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OptimizerConfig {
    /// The update rule.
    pub method: Method,
    /// The linear solver.
    pub solver: LinearSolver,
    /// Iteration limit (accepted or rejected trial steps both count).
    pub max_iterations: usize,
    /// Converged when an accepted step lowers the cost by less than this fraction.
    pub rel_cost_tol: f64,
    /// Converged when the largest tangent step is below this.
    pub step_tol: f64,
    /// Initial `λ`.
    pub lambda0: f64,
    /// `λ` above this means no step helps: stop.
    pub lambda_max: f64,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self {
            method: Method::LevenbergMarquardt,
            solver: LinearSolver::Sparse,
            max_iterations: 100,
            rel_cost_tol: 1e-10,
            step_tol: 1e-9,
            lambda0: 1e-3,
            lambda_max: 1e12,
        }
    }
}

/// What an optimization did.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OptimizeReport {
    /// Trial steps taken.
    pub iterations: usize,
    /// Steps that were accepted.
    pub accepted_steps: usize,
    /// Cost `Σ ρ(e)` before.
    pub initial_cost: f64,
    /// Cost after.
    pub final_cost: f64,
    /// A tolerance was met (as opposed to the iteration limit or `λ_max`).
    pub converged: bool,
    /// Final damping.
    pub lambda: f64,
}

/// The normal equations at one linearization point.
struct Assembly {
    /// Lower-triangle entries `(row, col)` in a fixed order, and their values.
    pairs: Vec<Pair<usize, usize>>,
    vals: Vec<f64>,
    /// `Σ w Jᵀ e`.
    gradient: DVector<f64>,
    /// Diagonal of `H`.
    diag: DVector<f64>,
    cost: f64,
}

/// A set of factors. Variables live in [`Values`], so a graph can be evaluated at any estimate.
#[derive(Debug, Clone, Default)]
pub struct FactorGraph {
    factors: Vec<Factor>,
}

impl FactorGraph {
    /// An empty graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a factor.
    pub fn add(&mut self, factor: Factor) {
        self.factors.push(factor);
    }

    /// The factors, in insertion order.
    pub fn factors(&self) -> &[Factor] {
        &self.factors
    }

    /// Remove the last factor added (used to roll back a rejected candidate).
    pub fn pop(&mut self) -> Option<Factor> {
        self.factors.pop()
    }

    /// `Σ ρ(e)` at `values`.
    pub fn cost(&self, values: &Values) -> Result<f64, SlamError> {
        let mut c = 0.0;
        for f in &self.factors {
            let (e, _) = f.linearize(values)?;
            c += f.kernel.cost(e.norm());
        }
        Ok(c)
    }

    fn assemble(&self, values: &Values) -> Result<Assembly, SlamError> {
        let n = values.dim();
        let mut pairs = Vec::new();
        let mut vals = Vec::new();
        let mut gradient = DVector::<f64>::zeros(n);
        let mut diag = DVector::<f64>::zeros(n);
        let mut cost = 0.0_f64;
        for f in &self.factors {
            let (e, blocks) = f.linearize(values)?;
            let en = e.norm();
            cost += f.kernel.cost(en);
            let w = f.kernel.weight(en);
            for (ka, ja) in &blocks {
                let oa = values.offset(*ka)?;
                let g = ja.transpose() * &e * w;
                for r in 0..ja.ncols() {
                    gradient[oa + r] += g[r];
                }
                for (kb, jb) in &blocks {
                    let ob = values.offset(*kb)?;
                    if oa < ob {
                        continue;
                    }
                    let h = ja.transpose() * jb * w;
                    for r in 0..h.nrows() {
                        for c in 0..h.ncols() {
                            let (row, col) = (oa + r, ob + c);
                            if row < col {
                                continue;
                            }
                            pairs.push(Pair::new(row, col));
                            vals.push(h[(r, c)]);
                            if row == col {
                                diag[row] += h[(r, c)];
                            }
                        }
                    }
                }
            }
        }
        if !(cost.is_finite() && gradient.iter().all(|v| v.is_finite())) {
            return Err(SlamError::NonFinite("factor graph linearization"));
        }
        Ok(Assembly {
            pairs,
            vals,
            gradient,
            diag,
            cost,
        })
    }

    fn dense_matrix(a: &Assembly, n: usize, lambda: f64) -> DMatrix<f64> {
        let mut h = DMatrix::zeros(n, n);
        for (p, v) in a.pairs.iter().zip(&a.vals) {
            h[(p.row, p.col)] += v;
            if p.row != p.col {
                h[(p.col, p.row)] += v;
            }
        }
        for i in 0..n {
            h[(i, i)] += lambda * a.diag[i].max(1e-9);
        }
        h
    }

    /// Optimize `values` in place.
    pub fn optimize(
        &self,
        values: &mut Values,
        cfg: &OptimizerConfig,
    ) -> Result<OptimizeReport, SlamError> {
        if self.factors.is_empty() || values.dim() == 0 {
            return Err(SlamError::EmptyInput("factor graph"));
        }
        let n = values.dim();
        let mut a = self.assemble(values)?;
        let initial_cost = a.cost;
        let mut solver = Solver::new(cfg.solver, &a, n)?;
        // The undamped system must be solvable: a missing prior is a bug, not something to nudge.
        solver.factor(&a, 0.0)?;
        let mut lambda = match cfg.method {
            Method::GaussNewton => 0.0,
            Method::LevenbergMarquardt => cfg.lambda0,
        };
        let (mut iterations, mut accepted, mut converged) = (0, 0, false);
        while iterations < cfg.max_iterations {
            iterations += 1;
            if solver.factor(&a, lambda).is_err() {
                if cfg.method == Method::GaussNewton {
                    return Err(SlamError::SingularSystem("Gauss-Newton step"));
                }
                lambda *= 10.0;
                if lambda > cfg.lambda_max {
                    break;
                }
                continue;
            }
            let delta = solver.solve(&(-&a.gradient));
            if delta.iter().any(|v| !v.is_finite()) {
                return Err(SlamError::NonFinite("optimizer step"));
            }
            let step_max = delta.amax();
            let mut trial = values.clone();
            trial.retract(&delta);
            let next = self.assemble(&trial)?;
            match cfg.method {
                Method::GaussNewton => {
                    let drop = a.cost - next.cost;
                    *values = trial;
                    a = next;
                    accepted += 1;
                    if step_max < cfg.step_tol
                        || drop.abs() <= cfg.rel_cost_tol * a.cost.max(1e-300)
                    {
                        converged = true;
                        break;
                    }
                }
                Method::LevenbergMarquardt => {
                    let predicted = 0.5
                        * (lambda
                            * delta
                                .iter()
                                .zip(a.diag.iter())
                                .map(|(d, h)| d * d * h.max(1e-9))
                                .sum::<f64>()
                            - a.gradient.dot(&delta));
                    let actual = a.cost - next.cost;
                    if actual > 0.0 && predicted > 0.0 {
                        let rho = actual / predicted;
                        lambda *= (1.0f64 / 3.0).max(1.0 - (2.0 * rho - 1.0).powi(3));
                        lambda = lambda.max(1e-12);
                        *values = trial;
                        let old = a.cost;
                        a = next;
                        accepted += 1;
                        if actual <= cfg.rel_cost_tol * old.max(1e-300) || step_max < cfg.step_tol {
                            converged = true;
                            break;
                        }
                    } else {
                        if step_max < cfg.step_tol {
                            converged = true;
                            break;
                        }
                        lambda *= 10.0;
                        if lambda > cfg.lambda_max {
                            break;
                        }
                    }
                }
            }
        }
        Ok(OptimizeReport {
            iterations,
            accepted_steps: accepted,
            initial_cost,
            final_cost: a.cost,
            converged,
            lambda,
        })
    }

    /// The joint marginal covariance of `keys`, in the order given: the inverse of the undamped
    /// normal equations at `values`, restricted to those variables. Only meaningful at an
    /// optimum, and only if the system is solvable (a prior fixes the gauge).
    pub fn marginal_covariance(
        &self,
        values: &Values,
        keys: &[VarKey],
        solver_kind: LinearSolver,
    ) -> Result<DMatrix<f64>, SlamError> {
        let n = values.dim();
        let a = self.assemble(values)?;
        let mut solver = Solver::new(solver_kind, &a, n)?;
        solver.factor(&a, 0.0)?;
        let mut idx = Vec::new();
        for k in keys {
            let o = values.offset(*k)?;
            let d = match k {
                VarKey::Pose(_) => 3,
                VarKey::Landmark(_) => 2,
            };
            idx.extend(o..o + d);
        }
        let m = idx.len();
        let mut cov = DMatrix::zeros(m, m);
        for (c, &col) in idx.iter().enumerate() {
            let mut e = DVector::zeros(n);
            e[col] = 1.0;
            let x = solver.solve(&e);
            for (r, &row) in idx.iter().enumerate() {
                cov[(r, c)] = x[row];
            }
        }
        // Remove the asymmetry of rounding.
        Ok((&cov + cov.transpose()) * 0.5)
    }
}

enum Solver {
    Dense(Option<nalgebra::Cholesky<f64, nalgebra::Dyn>>, usize),
    Sparse {
        n: usize,
        symbolic: SymbolicSparseColMat<usize>,
        argsort: Argsort<usize>,
        sym_llt: SymbolicLlt<usize>,
        llt: Option<Llt<usize, f64>>,
        diag_slots: usize,
    },
}

impl Solver {
    fn new(kind: LinearSolver, a: &Assembly, n: usize) -> Result<Self, SlamError> {
        match kind {
            LinearSolver::Dense => Ok(Solver::Dense(None, n)),
            LinearSolver::Sparse => {
                // The structure is the pairs of the assembly plus every diagonal entry, so
                // damping never changes it.
                let mut pairs = a.pairs.clone();
                pairs.extend((0..n).map(|i| Pair::new(i, i)));
                let (symbolic, argsort) = SymbolicSparseColMat::try_new_from_indices(n, n, &pairs)
                    .map_err(|_| SlamError::SingularSystem("sparse structure"))?;
                let sym_llt = SymbolicLlt::try_new(symbolic.as_ref(), Side::Lower)
                    .map_err(|_| SlamError::SingularSystem("symbolic Cholesky"))?;
                Ok(Solver::Sparse {
                    n,
                    symbolic,
                    argsort,
                    sym_llt,
                    llt: None,
                    diag_slots: n,
                })
            }
        }
    }

    fn factor(&mut self, a: &Assembly, lambda: f64) -> Result<(), SlamError> {
        match self {
            Solver::Dense(chol, n) => {
                *chol = None;
                let h = FactorGraph::dense_matrix(a, *n, lambda);
                let c = h.cholesky().ok_or(SlamError::SingularSystem(
                    "normal equations (add a prior factor?)",
                ))?;
                *chol = Some(c);
                Ok(())
            }
            Solver::Sparse {
                n,
                symbolic,
                argsort,
                sym_llt,
                llt,
                diag_slots,
            } => {
                *llt = None;
                let mut vals = a.vals.clone();
                debug_assert_eq!(*diag_slots, *n);
                vals.extend((0..*n).map(|i| lambda * a.diag[i].max(1e-9)));
                let m =
                    SparseColMat::<usize, f64>::new_from_argsort(symbolic.clone(), argsort, &vals)
                        .map_err(|_| SlamError::SingularSystem("sparse assembly"))?;
                let f = Llt::try_new_with_symbolic(sym_llt.clone(), m.as_ref(), Side::Lower)
                    .map_err(|_| {
                        SlamError::SingularSystem("normal equations (add a prior factor?)")
                    })?;
                *llt = Some(f);
                Ok(())
            }
        }
    }

    fn solve(&self, rhs: &DVector<f64>) -> DVector<f64> {
        match self {
            Solver::Dense(Some(c), _) => c.solve(rhs),
            Solver::Sparse {
                n, llt: Some(l), ..
            } => {
                let mut b = Mat::<f64>::from_fn(*n, 1, |i, _| rhs[i]);
                l.solve_in_place(b.as_mut());
                DVector::from_fn(*n, |i, _| b[(i, 0)])
            }
            _ => DVector::from_element(rhs.len(), f64::NAN),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;
    use robotics_core::{seeded_rng, Gaussian};

    fn info3(sx: f64, sth: f64) -> Matrix3<f64> {
        Matrix3::from_diagonal(&Vector3::new(
            1.0 / (sx * sx),
            1.0 / (sx * sx),
            1.0 / (sth * sth),
        ))
    }

    /// `n` poses on a circle of radius 5 heading along the tangent.
    fn ring_truth(n: usize) -> Vec<Se2> {
        (0..n)
            .map(|k| {
                let a = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
                Se2::new(
                    5.0 * a.cos(),
                    5.0 * a.sin(),
                    a + std::f64::consts::FRAC_PI_2,
                )
            })
            .collect()
    }

    /// A ring graph: a prior on pose 0, noisy odometry around the ring, one loop closure.
    /// Returns the graph, dead-reckoned initial values and the truth.
    fn ring(n: usize, seed: u64, kernel: RobustKernel) -> (FactorGraph, Values, Vec<Se2>) {
        let truth = ring_truth(n);
        let mut rng = seeded_rng(seed);
        let (sx, sth) = (0.02, 0.01);
        let noise = Gaussian::<3>::new(
            Vector3::zeros(),
            Matrix3::from_diagonal(&Vector3::new(sx * sx, sx * sx, sth * sth)),
        )
        .unwrap();
        let mut g = FactorGraph::new();
        g.add(Factor::prior(0, truth[0], &info3(1e-3, 1e-3), RobustKernel::L2).unwrap());
        let mut init = vec![truth[0]];
        for k in 0..n - 1 {
            let z = truth[k]
                .between(&truth[k + 1])
                .boxplus(&noise.sample(&mut rng));
            g.add(Factor::between(k, k + 1, z, &info3(sx, sth), kernel).unwrap());
            let next = init[k].compose(&z);
            init.push(next);
        }
        let z = truth[n - 1]
            .between(&truth[0])
            .boxplus(&noise.sample(&mut rng));
        g.add(Factor::between(n - 1, 0, z, &info3(sx, sth), kernel).unwrap());
        (
            g,
            Values {
                poses: init,
                landmarks: vec![],
            },
            truth,
        )
    }

    fn rms_position_error(v: &Values, truth: &[Se2]) -> f64 {
        let s: f64 = v
            .poses
            .iter()
            .zip(truth)
            .map(|(a, b)| a.distance_to(b).powi(2))
            .sum();
        (s / truth.len() as f64).sqrt()
    }

    fn numeric_whitened_jacobian(f: &Factor, values: &Values, key: VarKey) -> DMatrix<f64> {
        let (dim, h) = match key {
            VarKey::Pose(_) => (3, 1e-6),
            VarKey::Landmark(_) => (2, 1e-6),
        };
        let eval = |d: &[f64]| {
            let mut v = values.clone();
            match key {
                VarKey::Pose(i) => v.poses[i] = v.poses[i].boxplus(&Vector3::new(d[0], d[1], d[2])),
                VarKey::Landmark(l) => {
                    v.landmarks[l] += Vector2::new(d[0], d[1]);
                }
            }
            f.linearize(&v).unwrap().0
        };
        let rows = f.linearize(values).unwrap().0.len();
        let mut j = DMatrix::zeros(rows, dim);
        for c in 0..dim {
            let (mut p, mut m) = (vec![0.0; dim], vec![0.0; dim]);
            p[c] = h;
            m[c] = -h;
            j.set_column(c, &((eval(&p) - eval(&m)) / (2.0 * h)));
        }
        j
    }

    #[test]
    fn analytic_jacobians_of_every_factor_match_central_differences() {
        let values = Values {
            poses: vec![Se2::new(1.0, 0.5, 0.4), Se2::new(2.5, -1.0, 1.1)],
            landmarks: vec![Vector2::new(3.0, 2.0)],
        };
        let info2 = Matrix2::new(40.0, 3.0, 3.0, 90.0);
        let factors = [
            Factor::prior(
                1,
                Se2::new(2.0, -0.7, 0.9),
                &info3(0.1, 0.05),
                RobustKernel::L2,
            )
            .unwrap(),
            Factor::between(
                0,
                1,
                Se2::new(1.2, -1.4, 0.5),
                &info3(0.1, 0.05),
                RobustKernel::L2,
            )
            .unwrap(),
            Factor::range_bearing(0, 0, 2.7, 0.3, &info2, RobustKernel::L2).unwrap(),
        ];
        for f in &factors {
            let (_, blocks) = f.linearize(&values).unwrap();
            for (key, j) in blocks {
                let n = numeric_whitened_jacobian(f, &values, key);
                assert!((j - n).abs().max() < 1e-5, "{:?} {key:?}", f.kind());
            }
        }
    }

    #[test]
    fn a_consistent_graph_is_a_zero_of_the_cost() {
        let truth = ring_truth(12);
        let mut g = FactorGraph::new();
        g.add(Factor::prior(0, truth[0], &info3(0.01, 0.01), RobustKernel::L2).unwrap());
        for k in 0..11 {
            g.add(
                Factor::between(
                    k,
                    k + 1,
                    truth[k].between(&truth[k + 1]),
                    &info3(0.02, 0.01),
                    RobustKernel::L2,
                )
                .unwrap(),
            );
        }
        let v = Values {
            poses: truth.clone(),
            landmarks: vec![],
        };
        assert!(g.cost(&v).unwrap() < 1e-20);
    }

    #[test]
    fn optimization_lowers_the_cost_and_the_trajectory_error() {
        let (g, mut v, truth) = ring(40, 3, RobustKernel::L2);
        let before = rms_position_error(&v, &truth);
        let rep = g.optimize(&mut v, &OptimizerConfig::default()).unwrap();
        let after = rms_position_error(&v, &truth);
        assert!(rep.converged);
        assert!(rep.final_cost < rep.initial_cost);
        assert!(after < before, "{after} {before}");
        assert_relative_eq!(rep.final_cost, g.cost(&v).unwrap(), max_relative = 1e-12);
    }

    #[test]
    fn dense_and_sparse_solvers_agree() {
        let (g, v0, _) = ring(60, 5, RobustKernel::Huber { k: 2.0 });
        let run = |solver| {
            let mut v = v0.clone();
            let cfg = OptimizerConfig {
                solver,
                ..Default::default()
            };
            let rep = g.optimize(&mut v, &cfg).unwrap();
            (v, rep)
        };
        let (vd, rd) = run(LinearSolver::Dense);
        let (vs, rs) = run(LinearSolver::Sparse);
        for (a, b) in vd.poses.iter().zip(&vs.poses) {
            assert!(a.between(b).log().norm() < 1e-9, "{a:?} {b:?}");
        }
        assert_relative_eq!(rd.final_cost, rs.final_cost, max_relative = 1e-9);
        assert_eq!(rd.iterations, rs.iterations);
    }

    #[test]
    fn gauss_newton_and_levenberg_marquardt_reach_the_same_optimum() {
        let (g, v0, _) = ring(30, 8, RobustKernel::L2);
        let go = |method| {
            let mut v = v0.clone();
            let cfg = OptimizerConfig {
                method,
                ..Default::default()
            };
            g.optimize(&mut v, &cfg).unwrap();
            v
        };
        let (a, b) = (go(Method::GaussNewton), go(Method::LevenbergMarquardt));
        for (x, y) in a.poses.iter().zip(&b.poses) {
            assert!(x.between(y).log().norm() < 1e-6);
        }
    }

    #[test]
    fn the_same_graph_gives_the_same_answer_bit_for_bit() {
        let (g, v0, _) = ring(30, 9, RobustKernel::Cauchy { c: 2.0 });
        let run = || {
            let mut v = v0.clone();
            g.optimize(&mut v, &OptimizerConfig::default()).unwrap();
            v.poses
                .iter()
                .flat_map(|p| [p.x().to_bits(), p.y().to_bits(), p.theta().to_bits()])
                .collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn a_missing_prior_is_an_error_not_a_nudge() {
        let truth = ring_truth(6);
        let mut g = FactorGraph::new();
        for k in 0..5 {
            g.add(
                Factor::between(
                    k,
                    k + 1,
                    truth[k].between(&truth[k + 1]),
                    &info3(0.02, 0.01),
                    RobustKernel::L2,
                )
                .unwrap(),
            );
        }
        for solver in [LinearSolver::Dense, LinearSolver::Sparse] {
            let mut v = Values {
                poses: truth.clone(),
                landmarks: vec![],
            };
            let cfg = OptimizerConfig {
                solver,
                ..Default::default()
            };
            assert!(
                matches!(g.optimize(&mut v, &cfg), Err(SlamError::SingularSystem(_))),
                "{solver:?}"
            );
        }
    }

    #[test]
    fn a_robust_kernel_limits_what_one_false_loop_closure_does() {
        // Exact odometry around a ring, six exact chords that make it rigid, and one loop edge
        // that lies by 1 m. Without the chords the chain is softer than the lie and every
        // kernel ends up in its quadratic zone, which proves nothing.
        let n = 30;
        let truth = ring_truth(n);
        let lie = truth[0]
            .between(&truth[15])
            .boxplus(&Vector3::new(1.0, 0.0, 0.0));
        let solve = |kernel: RobustKernel| {
            let mut g = FactorGraph::new();
            g.add(Factor::prior(0, truth[0], &info3(1e-3, 1e-3), RobustKernel::L2).unwrap());
            for k in 0..n - 1 {
                g.add(
                    Factor::between(
                        k,
                        k + 1,
                        truth[k].between(&truth[k + 1]),
                        &info3(0.05, 0.02),
                        RobustKernel::L2,
                    )
                    .unwrap(),
                );
            }
            for (a, b) in [(0, 10), (10, 20), (5, 25), (8, 22), (3, 17), (12, 27)] {
                g.add(
                    Factor::between(
                        a,
                        b,
                        truth[a].between(&truth[b]),
                        &info3(0.05, 0.02),
                        RobustKernel::L2,
                    )
                    .unwrap(),
                );
            }
            g.add(Factor::between(0, 15, lie, &info3(0.05, 0.02), kernel).unwrap());
            let mut v = Values {
                poses: truth.clone(),
                landmarks: vec![],
            };
            g.optimize(&mut v, &OptimizerConfig::default()).unwrap();
            rms_position_error(&v, &truth)
        };
        let (l2, huber, cauchy) = (
            solve(RobustKernel::L2),
            solve(RobustKernel::Huber { k: 2.796 }),
            solve(RobustKernel::Cauchy { c: 2.796 }),
        );
        assert!(huber < l2, "huber {huber} l2 {l2}");
        assert!(cauchy < huber, "cauchy {cauchy} huber {huber}");
    }

    #[test]
    fn a_landmark_is_recovered_from_range_bearing_observations() {
        let truth_l = Vector2::new(3.0, 2.0);
        let poses = vec![
            Se2::new(0.0, 0.0, 0.0),
            Se2::new(1.0, -1.0, 0.4),
            Se2::new(2.0, -1.5, 1.2),
            Se2::new(-1.0, 0.5, -0.3),
        ];
        let mut g = FactorGraph::new();
        for (i, p) in poses.iter().enumerate() {
            g.add(Factor::prior(i, *p, &info3(1e-3, 1e-3), RobustKernel::L2).unwrap());
            let loc = p.act_inverse(&truth_l);
            g.add(
                Factor::range_bearing(
                    i,
                    0,
                    loc.norm(),
                    loc.y.atan2(loc.x),
                    &Matrix2::from_diagonal(&nalgebra::Vector2::new(400.0, 1600.0)),
                    RobustKernel::L2,
                )
                .unwrap(),
            );
        }
        let mut v = Values {
            poses,
            landmarks: vec![Vector2::new(2.0, 3.5)],
        };
        let rep = g.optimize(&mut v, &OptimizerConfig::default()).unwrap();
        assert!(rep.converged);
        assert!(
            (v.landmarks[0] - truth_l).norm() < 1e-6,
            "{:?}",
            v.landmarks[0]
        );
    }

    #[test]
    fn marginal_covariance_of_a_prior_and_one_between_is_the_closed_form() {
        let x0 = Se2::new(1.0, 2.0, 0.6);
        let z = Se2::new(1.5, -0.4, 0.7);
        let (p, b) = (info3(0.1, 0.05), info3(0.2, 0.08));
        let mut g = FactorGraph::new();
        g.add(Factor::prior(0, x0, &p, RobustKernel::L2).unwrap());
        g.add(Factor::between(0, 1, z, &b, RobustKernel::L2).unwrap());
        let v = Values {
            poses: vec![x0, x0.compose(&z)],
            landmarks: vec![],
        };
        let p_inv = p.try_inverse().unwrap();
        let b_inv = b.try_inverse().unwrap();
        let ad = z.inverse().adjoint();
        let s00 = p_inv;
        let s01 = p_inv * ad.transpose();
        let s11 = ad * p_inv * ad.transpose() + b_inv;
        for solver in [LinearSolver::Dense, LinearSolver::Sparse] {
            let c = g
                .marginal_covariance(&v, &[VarKey::Pose(0), VarKey::Pose(1)], solver)
                .unwrap();
            let block = |r: usize, c0: usize| Matrix3::from_fn(|i, j| c[(3 * r + i, 3 * c0 + j)]);
            assert!((block(0, 0) - s00).abs().max() < 1e-9, "{solver:?}");
            assert!((block(0, 1) - s01).abs().max() < 1e-9, "{solver:?}");
            assert!((block(1, 1) - s11).abs().max() < 1e-9, "{solver:?}");
        }
    }

    #[test]
    fn unknown_variables_and_empty_graphs_are_typed_errors() {
        let mut g = FactorGraph::new();
        let mut v = Values::default();
        assert!(matches!(
            g.optimize(&mut v, &OptimizerConfig::default()),
            Err(SlamError::EmptyInput(_))
        ));
        g.add(Factor::prior(3, Se2::IDENTITY, &info3(0.1, 0.1), RobustKernel::L2).unwrap());
        v.poses.push(Se2::IDENTITY);
        assert!(matches!(g.cost(&v), Err(SlamError::UnknownVariable(_))));
        assert!(Factor::between(1, 1, Se2::IDENTITY, &info3(0.1, 0.1), RobustKernel::L2).is_err());
        assert!(Factor::prior(0, Se2::IDENTITY, &Matrix3::zeros(), RobustKernel::L2).is_err());
    }
}
