//! Unscented Kalman filter and the unscented transform, generic over the state manifold.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 7 (`UtParams`, `sigma_points`,
//! `manifold_mean`, `unscented_transform`); Thrun, Burgard & Fox, Table 3.4; Julier &
//! Uhlmann's scaled unscented transform.
//!
//! Sigma points are drawn in the tangent space and retracted: `X₀ = μ`,
//! `X_{±i} = μ ⊞ (±√((n+λ)P))_i`, with `λ = α²(n+κ) − n`, weights
//! `w_m⁰ = λ/(n+λ)`, `w_c⁰ = w_m⁰ + (1 − α² + β)`, `w_i = 1/(2(n+λ))`.
//! Means of manifold-valued points are computed by the fixed-point iteration
//! `μ ← μ ⊞ Σ w_i (X_i ⊟ μ)`; for vector spaces it converges in one step.
//!
//! The machinery (sigma points, weights, means) is independent of any robot model and is
//! reused by later phases (e.g. UKF localization).

use crate::ekf::{MeasurementModel, ProcessModel};
use crate::update::{factor_innovation, symmetrize, validate_covariance, Tolerances};
use crate::FilterError;
use nalgebra::{linalg::Cholesky, SMatrix, SVector};
use robotics_core::Manifold;

/// Scaled unscented-transform parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UtParams {
    /// Spread of the sigma points (`0 < α ≤ 1` typical).
    pub alpha: f64,
    /// Prior-distribution knowledge (`β = 2` optimal for Gaussians).
    pub beta: f64,
    /// Secondary scaling (`κ = 3 − n` is Julier's heuristic, `0` is common).
    pub kappa: f64,
}

impl UtParams {
    /// `α = 1, β = 2, κ = 3 − n` (Julier's heuristic; the book's default).
    pub fn julier(n: usize) -> Self {
        Self {
            alpha: 1.0,
            beta: 2.0,
            kappa: 3.0 - n as f64,
        }
    }

    /// Weights `(λ, w_m⁰, w_c⁰, w_i)` for an `n`-dimensional state, validated.
    pub fn weights(&self, n: usize) -> Result<(f64, f64, f64, f64), FilterError> {
        for (what, v) in [
            ("alpha", self.alpha),
            ("beta", self.beta),
            ("kappa", self.kappa),
        ] {
            if !v.is_finite() {
                return Err(FilterError::InvalidNoiseParameter { what, value: v });
            }
        }
        if self.alpha <= 0.0 {
            return Err(FilterError::InvalidNoiseParameter {
                what: "alpha",
                value: self.alpha,
            });
        }
        let nf = n as f64;
        let lambda = self.alpha * self.alpha * (nf + self.kappa) - nf;
        if nf + lambda <= 0.0 {
            return Err(FilterError::InvalidNoiseParameter {
                what: "n + lambda",
                value: nf + lambda,
            });
        }
        let wm0 = lambda / (nf + lambda);
        let wc0 = wm0 + (1.0 - self.alpha * self.alpha + self.beta);
        let wi = 0.5 / (nf + lambda);
        Ok((lambda, wm0, wc0, wi))
    }
}

/// `2n + 1` sigma points with their mean and covariance weights.
#[derive(Debug, Clone)]
pub struct SigmaPoints<X> {
    /// The points; index 0 is the mean.
    pub points: Vec<X>,
    /// Mean weights.
    pub wm: Vec<f64>,
    /// Covariance weights.
    pub wc: Vec<f64>,
}

/// Generate sigma points around `mean` with tangent covariance `cov`.
pub fn sigma_points<X: Manifold<N>, const N: usize>(
    mean: &X,
    cov: &SMatrix<f64, N, N>,
    params: &UtParams,
) -> Result<SigmaPoints<X>, FilterError> {
    let (lambda, wm0, wc0, wi) = params.weights(N)?;
    let scaled = symmetrize(cov) * (N as f64 + lambda);
    // Allow exactly-singular (e.g. zero) covariances by a tiny diagonal loading.
    let scale = 1.0 + scaled.abs().max();
    let chol = Cholesky::new(scaled)
        .or_else(|| Cholesky::new(scaled + SMatrix::<f64, N, N>::identity() * 1e-12 * scale))
        .ok_or(FilterError::InvalidCovariance(
            "sigma-point square root failed",
        ))?;
    let l = chol.l();
    let mut points = Vec::with_capacity(2 * N + 1);
    points.push(mean.clone());
    for i in 0..N {
        let col: SVector<f64, N> = l.column(i).into();
        points.push(mean.boxplus(&col));
    }
    for i in 0..N {
        let col: SVector<f64, N> = l.column(i).into();
        points.push(mean.boxplus(&(-col)));
    }
    let mut wm = vec![wi; 2 * N + 1];
    let mut wc = vec![wi; 2 * N + 1];
    wm[0] = wm0;
    wc[0] = wc0;
    Ok(SigmaPoints { points, wm, wc })
}

/// Weighted mean on a manifold by fixed-point iteration, starting from `init`.
pub fn manifold_mean<X: Manifold<N>, const N: usize>(
    points: &[X],
    weights: &[f64],
    init: &X,
    max_iters: usize,
    tol: f64,
) -> X {
    let mut mu = init.clone();
    for _ in 0..max_iters.max(1) {
        let step: SVector<f64, N> = points
            .iter()
            .zip(weights)
            .map(|(p, w)| p.boxminus(&mu) * *w)
            .fold(SVector::zeros(), |a, b| a + b);
        mu = mu.boxplus(&step);
        if step.norm() < tol {
            break;
        }
    }
    mu
}

/// Unscented transform of a set of already-propagated vector samples: weighted mean (via
/// `residual`, so angular components average correctly) and covariance.
pub fn vector_mean_cov<const M: usize>(
    samples: &[SVector<f64, M>],
    sp_wm: &[f64],
    sp_wc: &[f64],
    residual: impl Fn(&SVector<f64, M>, &SVector<f64, M>) -> SVector<f64, M>,
) -> (SVector<f64, M>, SMatrix<f64, M, M>) {
    let mut mean = samples[0];
    for _ in 0..5 {
        let step = samples
            .iter()
            .zip(sp_wm)
            .map(|(s, w)| residual(s, &mean) * *w)
            .fold(SVector::<f64, M>::zeros(), |a, b| a + b);
        mean += step;
        if step.norm() < 1e-12 {
            break;
        }
    }
    let cov = samples
        .iter()
        .zip(sp_wc)
        .map(|(s, w)| {
            let d = residual(s, &mean);
            d * d.transpose() * *w
        })
        .fold(SMatrix::<f64, M, M>::zeros(), |a, b| a + b);
    (mean, symmetrize(&cov))
}

/// Diagnostics of a UKF correction.
#[derive(Debug, Clone, PartialEq)]
pub struct UkfCorrection<const N: usize, const M: usize> {
    /// Innovation `ν = z ⊟ ẑ`.
    pub innovation: SVector<f64, M>,
    /// Predicted measurement mean `ẑ`.
    pub predicted: SVector<f64, M>,
    /// Innovation covariance `S`.
    pub innovation_cov: SMatrix<f64, M, M>,
    /// State–measurement cross covariance `P_xz`.
    pub cross_cov: SMatrix<f64, N, M>,
    /// Gain `K = P_xz S⁻¹`.
    pub gain: SMatrix<f64, N, M>,
    /// `νᵀ S⁻¹ ν`.
    pub nis: f64,
}

/// Unscented Kalman filter over a manifold state.
#[derive(Debug, Clone)]
pub struct Ukf<X, const N: usize> {
    x: X,
    p: SMatrix<f64, N, N>,
    params: UtParams,
    tol: Tolerances,
    mean_iters: usize,
}

impl<X: Manifold<N>, const N: usize> Ukf<X, N> {
    /// Create the filter; parameters and covariance are validated.
    pub fn new(x: X, p: SMatrix<f64, N, N>, params: UtParams) -> Result<Self, FilterError> {
        let tol = Tolerances::default();
        params.weights(N)?;
        let p = validate_covariance(&p, &tol)?;
        Ok(Self {
            x,
            p,
            params,
            tol,
            mean_iters: 10,
        })
    }

    /// Current estimate.
    #[inline]
    pub fn state(&self) -> &X {
        &self.x
    }
    /// Current covariance.
    #[inline]
    pub fn covariance(&self) -> &SMatrix<f64, N, N> {
        &self.p
    }
    /// UT parameters.
    #[inline]
    pub fn params(&self) -> &UtParams {
        &self.params
    }

    /// Prediction through the sigma points. On error the filter is unchanged.
    pub fn predict<U, P: ProcessModel<X, U, N>>(
        &mut self,
        model: &P,
        u: &U,
    ) -> Result<(), FilterError> {
        let sp = sigma_points(&self.x, &self.p, &self.params)?;
        let propagated: Vec<X> = sp.points.iter().map(|p| model.propagate(p, u)).collect();
        let mean = manifold_mean(&propagated, &sp.wm, &propagated[0], self.mean_iters, 1e-12);
        let mut cov = SMatrix::<f64, N, N>::zeros();
        for (p, w) in propagated.iter().zip(&sp.wc) {
            let d = p.boxminus(&mean);
            if d.iter().any(|v| !v.is_finite()) {
                return Err(FilterError::NonFinite("propagated sigma point"));
            }
            cov += d * d.transpose() * *w;
        }
        let q = validate_covariance(&model.process_noise(&mean, u), &self.tol)?;
        let cov = validate_covariance(&symmetrize(&(cov + q)), &self.tol)?;
        self.x = mean;
        self.p = cov;
        Ok(())
    }

    /// Correction with measurement `z`. On error the filter is unchanged.
    pub fn correct<const M: usize, H: MeasurementModel<X, N, M>>(
        &mut self,
        model: &H,
        z: &SVector<f64, M>,
    ) -> Result<UkfCorrection<N, M>, FilterError> {
        if z.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("measurement"));
        }
        let sp = sigma_points(&self.x, &self.p, &self.params)?;
        let zs: Vec<SVector<f64, M>> = sp.points.iter().map(|p| model.predict(p)).collect();
        if zs.iter().any(|v| v.iter().any(|c| !c.is_finite())) {
            return Err(FilterError::NonFinite("predicted measurement"));
        }
        let (z_hat, s_z) = vector_mean_cov(&zs, &sp.wm, &sp.wc, |a, b| model.residual(a, b));
        let r = validate_covariance(&model.measurement_noise(&self.x), &self.tol)?;
        let s = symmetrize(&(s_z + r));
        let mut pxz = SMatrix::<f64, N, M>::zeros();
        for ((p, zi), w) in sp.points.iter().zip(&zs).zip(&sp.wc) {
            pxz += p.boxminus(&self.x) * model.residual(zi, &z_hat).transpose() * *w;
        }
        let chol = factor_innovation(&s, &self.tol)?;
        let gain = chol.solve(&pxz.transpose()).transpose();
        let nu = model.residual(z, &z_hat);
        let p_new = symmetrize(&(self.p - gain * s * gain.transpose()));
        let p_new = validate_covariance(&p_new, &self.tol)?;
        let nis = nu.dot(&chol.solve(&nu));
        self.x = self.x.boxplus(&(gain * nu));
        self.p = p_new;
        Ok(UkfCorrection {
            innovation: nu,
            predicted: z_hat,
            innovation_cov: s,
            cross_cov: pxz,
            gain,
            nis,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ekf::MeasurementJacobian;
    use crate::Ekf;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix1, Matrix2, Vector1, Vector2};
    use robotics_core::{angle::angle_diff, Se2, Tangent2};
    use std::f64::consts::FRAC_PI_2;

    struct PolarToCartesian;
    impl ProcessModel<Vector2<f64>, (), 2> for PolarToCartesian {
        fn propagate(&self, x: &Vector2<f64>, _: &()) -> Vector2<f64> {
            Vector2::new(x[0] * x[1].cos(), x[0] * x[1].sin())
        }
        fn process_noise(&self, _: &Vector2<f64>, _: &()) -> Matrix2<f64> {
            Matrix2::zeros()
        }
    }

    /// Chapter 7 worked example: UT(α=1, β=0, κ=1) of the polar → Cartesian map.
    /// λ = 1, weights 1/3 and 1/6; mean y = ⅓·1 + ⅙(1.034641 + 0.965359) + ⅙(2·0.898941)
    /// = 0.966314 against the true 0.966311 (EKF: 1.0); σ_y = 0.051667, σ_x = 0.252919.
    #[test]
    fn worked_example_ch7_unscented_polar() {
        let sig_th = 15f64.to_radians();
        let params = UtParams {
            alpha: 1.0,
            beta: 0.0,
            kappa: 1.0,
        };
        let (lambda, wm0, wc0, wi) = params.weights(2).unwrap();
        assert_relative_eq!(lambda, 1.0, epsilon = 1e-15);
        assert_relative_eq!(wm0, 1.0 / 3.0, epsilon = 1e-15);
        assert_relative_eq!(wc0, 1.0 / 3.0, epsilon = 1e-15);
        assert_relative_eq!(wi, 1.0 / 6.0, epsilon = 1e-15);
        let mut ukf = Ukf::new(
            Vector2::new(1.0, FRAC_PI_2),
            Matrix2::new(0.02 * 0.02, 0.0, 0.0, sig_th * sig_th),
            params,
        )
        .unwrap();
        ukf.predict(&PolarToCartesian, &()).unwrap();
        assert_relative_eq!(ukf.state()[0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(ukf.state()[1], 0.966314, epsilon = 1e-5);
        assert_relative_eq!(ukf.covariance()[(1, 1)].sqrt(), 0.051667, epsilon = 1e-5);
        assert_relative_eq!(ukf.covariance()[(0, 0)].sqrt(), 0.252919, epsilon = 1e-4);
    }

    /// Book check #11: the UT is exact for affine maps.
    #[test]
    fn sigma_points_reconstruct_mean_and_covariance() {
        let mean = Vector2::new(0.3, -0.7);
        let cov = Matrix2::new(0.4, 0.1, 0.1, 0.25);
        let sp = sigma_points(&mean, &cov, &UtParams::julier(2)).unwrap();
        assert_eq!(sp.points.len(), 5);
        let a = Matrix2::new(2.0, -1.0, 0.5, 3.0);
        let b = Vector2::new(1.0, -2.0);
        let ys: Vec<Vector2<f64>> = sp.points.iter().map(|p| a * p + b).collect();
        let (m, c) = vector_mean_cov(&ys, &sp.wm, &sp.wc, |x, y| x - y);
        assert!((m - (a * mean + b)).norm() < 1e-9);
        assert!((c - a * cov * a.transpose()).norm() < 1e-9);
        assert_relative_eq!(sp.wm.iter().sum::<f64>(), 1.0, epsilon = 1e-12);
    }

    struct PosSensor;
    impl MeasurementModel<Vector2<f64>, 2, 1> for PosSensor {
        fn predict(&self, x: &Vector2<f64>) -> Vector1<f64> {
            Vector1::new(x[0])
        }
        fn measurement_noise(&self, _: &Vector2<f64>) -> Matrix1<f64> {
            Matrix1::new(0.25)
        }
    }
    impl MeasurementJacobian<Vector2<f64>, 2, 1> for PosSensor {
        fn jacobian(&self, _: &Vector2<f64>) -> SMatrix<f64, 1, 2> {
            SMatrix::<f64, 1, 2>::new(1.0, 0.0)
        }
    }

    /// For a linear measurement the UKF update equals the (E)KF update.
    #[test]
    fn linear_measurement_update_matches_ekf() {
        let x0 = Vector2::new(0.0, 1.0);
        let p0 = Matrix2::new(1.0, 0.2, 0.2, 0.5);
        let mut ukf = Ukf::new(x0, p0, UtParams::julier(2)).unwrap();
        let mut ekf = Ekf::new(x0, p0).unwrap();
        let c = ukf.correct(&PosSensor, &Vector1::new(0.3)).unwrap();
        let e = ekf.correct(&PosSensor, &Vector1::new(0.3)).unwrap();
        assert!((ukf.state() - ekf.state()).norm() < 1e-10);
        assert!((ukf.covariance() - ekf.covariance()).norm() < 1e-10);
        assert_relative_eq!(c.innovation_cov[(0, 0)], 1.25, epsilon = 1e-10);
        assert_relative_eq!(c.cross_cov[(0, 0)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(c.cross_cov[(1, 0)], 0.2, epsilon = 1e-10);
        assert_relative_eq!(c.nis, e.nis, epsilon = 1e-10);
    }

    /// Range-bearing measurement with a wrapped bearing residual on an SE(2) state.
    struct Bearing {
        landmark: Vector2<f64>,
    }
    impl MeasurementModel<Se2, 3, 1> for Bearing {
        fn predict(&self, x: &Se2) -> Vector1<f64> {
            let d = x.act_inverse(&self.landmark);
            Vector1::new(d.y.atan2(d.x))
        }
        fn measurement_noise(&self, _: &Se2) -> Matrix1<f64> {
            Matrix1::new(0.01 * 0.01)
        }
        fn residual(&self, z: &Vector1<f64>, zh: &Vector1<f64>) -> Vector1<f64> {
            Vector1::new(angle_diff(z[0], zh[0]))
        }
    }

    #[test]
    fn se2_ukf_handles_bearing_across_pi() {
        // Landmark straight behind the robot: bearing ≈ ±π, the worst case for wrapping.
        let truth = Se2::new(0.0, 0.0, 0.05);
        let lm = Vector2::new(-5.0, 0.0);
        let z = Bearing { landmark: lm }.predict(&truth);
        let mut ukf = Ukf::new(
            Se2::new(0.0, 0.0, -0.05),
            SMatrix::<f64, 3, 3>::identity() * 0.01,
            UtParams::julier(3),
        )
        .unwrap();
        for _ in 0..5 {
            ukf.correct(&Bearing { landmark: lm }, &z).unwrap();
        }
        assert!(angle_diff(ukf.state().theta(), truth.theta()).abs() < 0.02);
        let tangent: Tangent2 = ukf.state().boxminus(&truth);
        assert!(tangent.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn rejects_invalid_parameters() {
        assert!(UtParams {
            alpha: 0.0,
            beta: 2.0,
            kappa: 0.0
        }
        .weights(2)
        .is_err());
        assert!(UtParams {
            alpha: 1.0,
            beta: 2.0,
            kappa: -3.0
        }
        .weights(2)
        .is_err());
        assert!(Ukf::new(
            Vector2::zeros(),
            Matrix2::identity(),
            UtParams {
                alpha: f64::NAN,
                beta: 2.0,
                kappa: 0.0
            }
        )
        .is_err());
    }
}
