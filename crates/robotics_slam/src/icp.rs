//! Iterative closest point in the plane, and the information matrix of its result.
//!
//! [`register`] finds the pose `T` that maps source points `p` (the source scan's sensor
//! frame) onto the target map (`q ≈ T·p`), starting from a guess. Correspondences come from a
//! [`VoxelMap`] and are found again every iteration. Those farther than the threshold `τ` are
//! dropped; the rest are weighted by Geman-McClure with scale `τ/3`, so the weight at the
//! rejection boundary is one per cent and the cost has no cliff there (decision DS4).
//!
//! * [`IcpMetric::PointToPlane`] minimizes `Σ w (n · (T p − q))²` by Gauss-Newton on the SE(2)
//!   tangent (`T ⊞ δ = T·exp(δ)`, order `(v_x, v_y, ω)`), with the normal `n` from a PCA of the
//!   target points around `q`. A point whose neighbourhood is not a line (too few points, or
//!   the smaller eigenvalue above `planarity_ratio` of the larger: a corner, a pillar edge)
//!   falls back to a point-to-point residual for that correspondence; a normal is never guessed.
//! * [`IcpMetric::PointToPoint`] is the closed-form weighted alignment on centred clouds
//!   (`θ = atan2(Σ w a×b, Σ w a·b)`; in 2-D there is no SVD to take), iterated.
//!
//! The normal equations carry a ridge proportional to their trace, never an absolute one: an
//! absolute ridge slides the scan along an unconstrained direction.
//!
//! [`IcpResult::information`] is `Λ = (1/σ²) Σ w JᵀJ` at the final correspondences, in the
//! tangent at `T`, which is what a between factor needs. For point-to-plane it is built from the
//! correspondences that have a normal only (a corner, or a wall sampled too sparsely for a
//! normal, steers the solve and is not counted: along a sparse wall it would invent a constraint
//! along the wall and a corridor would not be seen to be degenerate). It is **overconfident**
//! (the beams are not independent, and it ignores correspondence error); claim S2 measures by
//! how much, and `info_inflation` is the one named number that scales it down.

use crate::voxel::VoxelMap;
use crate::SlamError;
use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use robotics_core::{LaserScan, Manifold, Se2};

/// Which residual is minimized.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IcpMetric {
    /// Distance between a point and its correspondence.
    PointToPoint,
    /// Distance between a point and the line through its correspondence.
    #[default]
    PointToPlane,
}

/// ICP parameters. Defaults are initial values, frozen after the development run (DS18).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IcpConfig {
    /// The residual.
    pub metric: IcpMetric,
    /// Correspondence threshold `τ` (m). Normally supplied per scan by the adaptive threshold.
    pub threshold: f64,
    /// Geman-McClure scale as a fraction of `τ` (derived: weight 0.01 at the boundary).
    pub kernel_ratio: f64,
    /// Iteration limit; stopping on it reports `converged = false`.
    pub max_iterations: usize,
    /// Convergence: translation step below this (m). Derived: a fifth of the lidar noise.
    pub trans_tol: f64,
    /// Convergence: rotation step below this (rad).
    pub rot_tol: f64,
    /// Radius of the neighbourhood the normal is fitted to (m). Derived: ±0.2 m of a wall.
    pub normal_radius: f64,
    /// Fewest neighbours for a normal.
    pub normal_min_points: usize,
    /// Largest `λ_min/λ_max` of the neighbourhood for it to count as a line.
    pub planarity_ratio: f64,
    /// Floor on the residual standard deviation used for the information matrix (m): the lidar
    /// noise, so a perfect synthetic fit does not report infinite certainty.
    pub sigma_floor: f64,
    /// Divides the information matrix. 1.0 until claim S2 has been measured, then frozen.
    pub info_inflation: f64,
    /// Ridge of the normal equations and of the information matrix, relative to their trace.
    pub ridge: f64,
    /// Smaller over larger eigenvalue of the translation block of `Λ` below which the
    /// registration is flagged degenerate (a corridor, or one wall in view).
    pub degeneracy_ratio: f64,
    /// Fewest correspondences for a result.
    pub min_correspondences: usize,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            metric: IcpMetric::PointToPlane,
            threshold: 0.5,
            kernel_ratio: 1.0 / 3.0,
            max_iterations: 50,
            trans_tol: 1e-4,
            rot_tol: 1e-4,
            normal_radius: 0.2,
            normal_min_points: 3,
            planarity_ratio: 0.1,
            sigma_floor: 0.015,
            info_inflation: 1.0,
            ridge: 1e-6,
            degeneracy_ratio: 0.02,
            min_correspondences: 10,
        }
    }
}

impl IcpConfig {
    /// Check every parameter.
    pub fn validate(&self) -> Result<(), SlamError> {
        let pos = |what: &'static str, v: f64| {
            if v.is_finite() && v > 0.0 {
                Ok(())
            } else {
                Err(SlamError::InvalidParameter { what, value: v })
            }
        };
        pos("threshold", self.threshold)?;
        pos("kernel_ratio", self.kernel_ratio)?;
        pos("trans_tol", self.trans_tol)?;
        pos("rot_tol", self.rot_tol)?;
        pos("normal_radius", self.normal_radius)?;
        pos("planarity_ratio", self.planarity_ratio)?;
        pos("sigma_floor", self.sigma_floor)?;
        pos("info_inflation", self.info_inflation)?;
        pos("degeneracy_ratio", self.degeneracy_ratio)?;
        if !(self.ridge.is_finite() && self.ridge >= 0.0) {
            return Err(SlamError::InvalidParameter {
                what: "ridge",
                value: self.ridge,
            });
        }
        if self.max_iterations == 0 {
            return Err(SlamError::InvalidParameter {
                what: "max_iterations",
                value: 0.0,
            });
        }
        if self.normal_min_points < 3 {
            return Err(SlamError::InvalidParameter {
                what: "normal_min_points",
                value: self.normal_min_points as f64,
            });
        }
        Ok(())
    }
}

/// The result of a registration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IcpResult {
    /// The estimated pose: source frame into target frame.
    pub pose: Se2,
    /// Stopped on the step tolerance, not on the iteration limit, with enough correspondences.
    pub converged: bool,
    /// The last step was under ten times the tolerance, with enough correspondences. Nearest
    /// neighbours flip between iterations on a discrete map, so a registration that has reached
    /// the answer can jitter by a fraction of a millimetre for ever; `settled` is what the front
    /// end and the loop verifier believe, `converged` is the strict statement.
    pub settled: bool,
    /// Translation of the last step (m).
    pub final_step_trans: f64,
    /// Rotation of the last step (rad).
    pub final_step_rot: f64,
    /// Iterations performed.
    pub iterations: usize,
    /// Correspondences at the final pose.
    pub correspondences: usize,
    /// Correspondences over source points.
    pub inlier_fraction: f64,
    /// Root-mean-square residual over the final correspondences (m).
    pub rmse: f64,
    /// `Λ` in the tangent at `pose`, scaled by `1/info_inflation`.
    pub information: Matrix3<f64>,
    /// The translation block of `Λ` is nearly rank one.
    pub degenerate: bool,
}

/// Source points of a scan in its sensor frame. A return at or beyond `range_max` is dropped,
/// not clamped: it says nothing about where a surface is.
pub fn scan_points(scan: &LaserScan) -> Vec<Vector2<f64>> {
    let limit = scan.range_max - 1e-6;
    scan.valid_beams()
        .filter(|&(_, r)| r < limit)
        .map(|(b, r)| Vector2::new(r * b.cos(), r * b.sin()))
        .collect()
}

/// The fraction of `source` points that, moved by `pose`, have a target point within `radius`.
/// A hypothesis's *support*: unlike the RMS of the matched points, it is charged for the points
/// a pose fails to explain.
pub fn support_fraction(
    source: &[Vector2<f64>],
    target: &VoxelMap,
    pose: &Se2,
    radius: f64,
) -> f64 {
    if source.is_empty() {
        return 0.0;
    }
    let n = source
        .iter()
        .filter(|p| target.nearest(&pose.act(p), radius).is_some())
        .count();
    n as f64 / source.len() as f64
}

struct Corr {
    p: Vector2<f64>,
    q: Vector2<f64>,
    normal: Option<Vector2<f64>>,
}

fn normal_at(q: &Vector2<f64>, target: &VoxelMap, cfg: &IcpConfig) -> Option<Vector2<f64>> {
    let nb = target.within_unsorted(q, cfg.normal_radius);
    if nb.len() < cfg.normal_min_points {
        return None;
    }
    let mean = nb.iter().sum::<Vector2<f64>>() / nb.len() as f64;
    let mut cov = Matrix2::zeros();
    for p in &nb {
        let d = p - mean;
        cov += d * d.transpose();
    }
    cov /= nb.len() as f64;
    let eig = cov.symmetric_eigen();
    let (imin, imax) = if eig.eigenvalues[0] <= eig.eigenvalues[1] {
        (0, 1)
    } else {
        (1, 0)
    };
    let (lmin, lmax) = (eig.eigenvalues[imin], eig.eigenvalues[imax]);
    if !lmax.is_finite() || lmax <= 0.0 || lmin / lmax > cfg.planarity_ratio {
        return None;
    }
    Some(eig.eigenvectors.column(imin).into_owned())
}

fn associate(source: &[Vector2<f64>], target: &VoxelMap, t: &Se2, cfg: &IcpConfig) -> Vec<Corr> {
    let mut out = Vec::with_capacity(source.len());
    for p in source {
        let x = t.act(p);
        if let Some((q, _)) = target.nearest(&x, cfg.threshold) {
            let normal = match cfg.metric {
                IcpMetric::PointToPlane => normal_at(&q, target, cfg),
                IcpMetric::PointToPoint => None,
            };
            out.push(Corr { p: *p, q, normal });
        }
    }
    out
}

fn gm_weight(e: f64, k: f64) -> f64 {
    let k2 = k * k;
    let d = k2 + e * e;
    k2 * k2 / (d * d)
}

/// Rows of the residual Jacobian `∂r/∂δ` at `t` for the right perturbation, as a list of
/// `(row, residual)` pairs: one row for a point-to-plane term, two for a point-to-point term.
fn rows(c: &Corr, t: &Se2, point_to_point_metric: bool) -> (Vec<Vector3<f64>>, Vec<f64>) {
    let x = t.act(&c.p);
    let rot = t.rotation();
    let jp = Vector2::new(-c.p.y, c.p.x);
    match (&c.normal, point_to_point_metric) {
        (Some(n), false) => {
            let m = rot.inverse() * n;
            (
                vec![Vector3::new(m.x, m.y, m.x * jp.x + m.y * jp.y)],
                vec![n.dot(&(x - c.q))],
            )
        }
        _ => {
            let r = x - c.q;
            let a0 = rot * Vector2::new(1.0, 0.0);
            let a1 = rot * Vector2::new(0.0, 1.0);
            let a2 = rot * jp;
            (
                vec![
                    Vector3::new(a0.x, a1.x, a2.x),
                    Vector3::new(a0.y, a1.y, a2.y),
                ],
                vec![r.x, r.y],
            )
        }
    }
}

fn solve_point_to_point(corr: &[Corr], t: &Se2, k: f64) -> Se2 {
    let w: Vec<f64> = corr
        .iter()
        .map(|c| gm_weight((t.act(&c.p) - c.q).norm(), k))
        .collect();
    let sw: f64 = w.iter().sum();
    let pbar = corr
        .iter()
        .zip(&w)
        .map(|(c, w)| c.p * *w)
        .sum::<Vector2<f64>>()
        / sw;
    let qbar = corr
        .iter()
        .zip(&w)
        .map(|(c, w)| c.q * *w)
        .sum::<Vector2<f64>>()
        / sw;
    let (mut dot, mut cross) = (0.0, 0.0);
    for (c, w) in corr.iter().zip(&w) {
        let a = c.p - pbar;
        let b = c.q - qbar;
        dot += w * a.dot(&b);
        cross += w * (a.x * b.y - a.y * b.x);
    }
    let theta = cross.atan2(dot);
    let (s, co) = theta.sin_cos();
    let tr = qbar - Vector2::new(co * pbar.x - s * pbar.y, s * pbar.x + co * pbar.y);
    Se2::new(tr.x, tr.y, theta)
}

fn solve_point_to_plane(corr: &[Corr], t: &Se2, k: f64, cfg: &IcpConfig) -> Result<Se2, SlamError> {
    let mut h = Matrix3::zeros();
    let mut g = Vector3::zeros();
    for c in corr {
        let (js, rs) = rows(c, t, false);
        let e = rs.iter().map(|r| r * r).sum::<f64>().sqrt();
        let w = gm_weight(e, k);
        for (j, r) in js.iter().zip(&rs) {
            h += w * j * j.transpose();
            g += w * *r * j;
        }
    }
    let ridge = cfg.ridge * h.trace() / 3.0;
    if h.trace().partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return Err(SlamError::SingularSystem("ICP normal equations are empty"));
    }
    let h = h + Matrix3::identity() * ridge;
    let delta = h
        .cholesky()
        .ok_or(SlamError::SingularSystem("ICP normal equations"))?
        .solve(&(-g));
    Ok(t.boxplus(&delta))
}

fn information_and_rmse(
    corr: &[Corr],
    t: &Se2,
    k: f64,
    cfg: &IcpConfig,
) -> (Matrix3<f64>, f64, bool) {
    let mut h = Matrix3::zeros();
    let (mut sw_r2, mut sw_dof, mut sum_r2, mut n_res) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    let point_to_point_metric = cfg.metric == IcpMetric::PointToPoint;
    for c in corr {
        let (js, rs) = rows(c, t, point_to_point_metric);
        let e = rs.iter().map(|r| r * r).sum::<f64>().sqrt();
        let w = gm_weight(e, k);
        sum_r2 += e * e;
        n_res += 1.0;
        // Point-to-plane: only line features carry information. A correspondence without a
        // normal (a corner, or a wall sampled too sparsely for a normal) steers the solve but
        // is not counted: along a sparse wall it would invent a constraint along the wall, and
        // a corridor would not be seen to be degenerate.
        if !point_to_point_metric && c.normal.is_none() {
            continue;
        }
        for j in &js {
            h += w * j * j.transpose();
        }
        sw_r2 += w * e * e;
        sw_dof += w * rs.len() as f64;
    }
    let sigma2 = (sw_r2 / sw_dof.max(1e-12)).max(cfg.sigma_floor * cfg.sigma_floor);
    let mut lambda = h / sigma2;
    let tb = lambda.fixed_view::<2, 2>(0, 0).into_owned();
    let eig = tb.symmetric_eigen();
    let (lo, hi) = (
        eig.eigenvalues[0].min(eig.eigenvalues[1]),
        eig.eigenvalues[0].max(eig.eigenvalues[1]),
    );
    let degenerate = !hi.is_finite() || hi <= 0.0 || lo / hi < cfg.degeneracy_ratio;
    lambda += Matrix3::identity() * (cfg.ridge * lambda.trace() / 3.0);
    (
        lambda / cfg.info_inflation,
        (sum_r2 / n_res.max(1.0)).sqrt(),
        degenerate,
    )
}

/// Register `source` against `target` starting from `init`.
///
/// Errors on an invalid configuration, an empty source or target, or a non-finite initial pose.
/// A registration that finds too few correspondences, or that stops on the iteration limit,
/// is returned with `converged = false` and the best pose reached: the caller decides.
pub fn register(
    source: &[Vector2<f64>],
    target: &VoxelMap,
    init: Se2,
    cfg: &IcpConfig,
) -> Result<IcpResult, SlamError> {
    cfg.validate()?;
    if source.is_empty() {
        return Err(SlamError::EmptyInput("ICP source"));
    }
    if target.is_empty() {
        return Err(SlamError::EmptyInput("ICP target"));
    }
    if !init.is_finite() {
        return Err(SlamError::NonFinite("ICP initial pose"));
    }
    let k = cfg.threshold * cfg.kernel_ratio;
    let mut t = init;
    let mut converged = false;
    let mut iterations = 0;
    let (mut last_trans, mut last_rot) = (f64::INFINITY, f64::INFINITY);
    for it in 0..cfg.max_iterations {
        iterations = it + 1;
        let corr = associate(source, target, &t, cfg);
        if corr.len() < cfg.min_correspondences {
            break;
        }
        let next = match cfg.metric {
            IcpMetric::PointToPoint => solve_point_to_point(&corr, &t, k),
            IcpMetric::PointToPlane => solve_point_to_plane(&corr, &t, k, cfg)?,
        };
        let step = t.between(&next).log();
        t = next;
        last_trans = step.xy().norm();
        last_rot = step.z.abs();
        if last_trans < cfg.trans_tol && last_rot < cfg.rot_tol {
            converged = true;
            break;
        }
    }
    let corr = associate(source, target, &t, cfg);
    let enough = corr.len() >= cfg.min_correspondences;
    let (information, rmse, degenerate) = if enough {
        information_and_rmse(&corr, &t, k, cfg)
    } else {
        (Matrix3::zeros(), f64::INFINITY, true)
    };
    Ok(IcpResult {
        pose: t,
        converged: converged && enough,
        settled: enough && last_trans < 10.0 * cfg.trans_tol && last_rot < 10.0 * cfg.rot_tol,
        final_step_trans: last_trans,
        final_step_rot: last_rot,
        iterations,
        correspondences: corr.len(),
        inlier_fraction: corr.len() as f64 / source.len() as f64,
        rmse,
        information,
        degenerate,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn line(a: Vector2<f64>, b: Vector2<f64>, n: usize) -> Vec<Vector2<f64>> {
        (0..n)
            .map(|i| a + (b - a) * (i as f64 / (n - 1) as f64))
            .collect()
    }

    /// An L-shaped, asymmetric set of two walls plus a short stub.
    fn corner_cloud() -> Vec<Vector2<f64>> {
        let mut pts = line(Vector2::new(0.0, 0.0), Vector2::new(3.0, 0.0), 120);
        pts.extend(line(Vector2::new(0.0, 0.0), Vector2::new(0.0, 2.0), 80));
        pts.extend(line(Vector2::new(1.0, 0.0), Vector2::new(1.0, 0.5), 20));
        pts
    }

    fn map_of(points: &[Vector2<f64>]) -> VoxelMap {
        let mut m = VoxelMap::new(0.1, 8).unwrap();
        m.insert_all(points);
        m
    }

    #[test]
    fn closed_form_point_to_point_is_exact_for_a_rigid_transform() {
        let truth = Se2::new(0.07, -0.04, 0.12);
        let target = corner_cloud();
        let source: Vec<_> = target.iter().map(|q| truth.act_inverse(q)).collect();
        let corr: Vec<_> = source
            .iter()
            .zip(&target)
            .map(|(p, q)| Corr {
                p: *p,
                q: *q,
                normal: None,
            })
            .collect();
        let t = solve_point_to_point(&corr, &Se2::IDENTITY, 1.0);
        assert_relative_eq!(t.x(), truth.x(), epsilon = 1e-9);
        assert_relative_eq!(t.y(), truth.y(), epsilon = 1e-9);
        assert_relative_eq!(t.theta(), truth.theta(), epsilon = 1e-9);
    }

    #[test]
    fn both_metrics_recover_a_small_offset_on_a_noiseless_corner() {
        let truth = Se2::new(0.05, 0.03, 0.04);
        let target = corner_cloud();
        let source: Vec<_> = target.iter().map(|q| truth.act_inverse(q)).collect();
        let map = map_of(&target);
        // Point-to-plane has no floor from the sampling of the walls; point-to-point cannot
        // beat the spacing of the target points (about 2.5 cm here), so it is held to that.
        for (metric, tol) in [
            (IcpMetric::PointToPlane, 2e-3),
            (IcpMetric::PointToPoint, 2e-2),
        ] {
            let cfg = IcpConfig {
                metric,
                threshold: 0.3,
                ..Default::default()
            };
            let r = register(&source, &map, Se2::IDENTITY, &cfg).unwrap();
            assert!(r.converged, "{metric:?}");
            let err = truth.between(&r.pose).log();
            assert!(
                err.xy().norm() < tol && err.z.abs() < tol,
                "{metric:?} {err:?}"
            );
        }
    }

    #[test]
    fn the_translation_block_of_a_single_wall_is_flagged_degenerate() {
        let wall = line(Vector2::new(-2.0, 1.0), Vector2::new(2.0, 1.0), 200);
        let map = map_of(&wall);
        let cfg = IcpConfig {
            threshold: 0.3,
            ..Default::default()
        };
        let r = register(&wall, &map, Se2::IDENTITY, &cfg).unwrap();
        assert!(r.degenerate);
        let corner = corner_cloud();
        let r = register(&corner, &map_of(&corner), Se2::IDENTITY, &cfg).unwrap();
        assert!(!r.degenerate);
    }

    #[test]
    fn a_threshold_smaller_than_the_offset_finds_nothing_to_match() {
        let target = corner_cloud();
        let truth = Se2::new(0.4, 0.4, 0.0);
        let source: Vec<_> = target.iter().map(|q| truth.act_inverse(q)).collect();
        let cfg = IcpConfig {
            threshold: 0.02,
            ..Default::default()
        };
        let r = register(&source, &map_of(&target), Se2::IDENTITY, &cfg).unwrap();
        assert!(!r.converged);
    }

    #[test]
    fn errors_are_typed() {
        let map = map_of(&corner_cloud());
        let cfg = IcpConfig::default();
        assert!(matches!(
            register(&[], &map, Se2::IDENTITY, &cfg),
            Err(SlamError::EmptyInput(_))
        ));
        let empty = VoxelMap::new(0.1, 4).unwrap();
        assert!(matches!(
            register(&corner_cloud(), &empty, Se2::IDENTITY, &cfg),
            Err(SlamError::EmptyInput(_))
        ));
        let bad = IcpConfig {
            threshold: -1.0,
            ..cfg
        };
        assert!(register(&corner_cloud(), &map, Se2::IDENTITY, &bad).is_err());
        assert!(matches!(
            register(&corner_cloud(), &map, Se2::new(f64::NAN, 0.0, 0.0), &cfg),
            Err(SlamError::NonFinite(_))
        ));
    }
}
