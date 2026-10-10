//! Extended Kalman filter, generic over the state manifold.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 7 — Beyond Linearity (a generic
//! EKF over a `Manifold` trait); Thrun, Burgard & Fox, Table 3.3.
//!
//! Model: `x_k = f(x_{k−1}, u_k) ⊞ w`, `z_k = h(x_k) + v`, with `w ~ N(0, Q)` in the tangent
//! space at the predicted state and `v ~ N(0, R)`. Jacobians are taken with respect to the
//! tangent-space error (`F = ∂(f(x̂ ⊞ δ) ⊟ f(x̂))/∂δ`, `H = ∂h(x̂ ⊞ δ)/∂δ`). For a plain vector
//! state (`SVector<f64, N>` implements [`Manifold`]) these are the ordinary Jacobians and
//! the filter is the textbook EKF.
//!
//! Naming: the traits call the covariances `process_noise` and `measurement_noise` so the
//! Thrun (`R` motion / `Q` measurement) versus control-theory (`Q` / `R`) clash never
//! appears in code.

use crate::update::{
    joseph_correction, symmetrize, validate_covariance, GaussianCorrection, Tolerances,
};
use crate::FilterError;
use nalgebra::{SMatrix, SVector};
use robotics_core::Manifold;

/// State transition `x_k = f(x_{k−1}, u)` with its process noise.
pub trait ProcessModel<X, U, const N: usize> {
    /// Deterministic propagation.
    fn propagate(&self, x: &X, u: &U) -> X;
    /// Process-noise covariance in the tangent space at the propagated state.
    fn process_noise(&self, x: &X, u: &U) -> SMatrix<f64, N, N>;
}

/// A process model that also provides its error-state Jacobian (needed by the EKF).
pub trait ProcessJacobian<X, U, const N: usize>: ProcessModel<X, U, N> {
    /// `F = ∂f/∂δ` evaluated at `x` (tangent-space error of the input state).
    fn jacobian(&self, x: &X, u: &U) -> SMatrix<f64, N, N>;
}

/// Measurement function `z = h(x) + v` with `M` components.
pub trait MeasurementModel<X, const N: usize, const M: usize> {
    /// Predicted measurement `h(x)`.
    fn predict(&self, x: &X) -> SVector<f64, M>;
    /// Measurement-noise covariance.
    fn measurement_noise(&self, x: &X) -> SMatrix<f64, M, M>;
    /// Residual `z ⊟ ẑ`; override to wrap angular components.
    fn residual(&self, z: &SVector<f64, M>, z_hat: &SVector<f64, M>) -> SVector<f64, M> {
        z - z_hat
    }
}

/// A measurement model that also provides its Jacobian (needed by the EKF).
pub trait MeasurementJacobian<X, const N: usize, const M: usize>:
    MeasurementModel<X, N, M>
{
    /// `H = ∂h/∂δ` evaluated at `x`.
    fn jacobian(&self, x: &X) -> SMatrix<f64, M, N>;
}

/// Extended Kalman filter over a state `X` living on an `N`-dimensional manifold.
#[derive(Debug, Clone)]
pub struct Ekf<X, const N: usize> {
    x: X,
    p: SMatrix<f64, N, N>,
    tol: Tolerances,
}

impl<X: Manifold<N>, const N: usize> Ekf<X, N> {
    /// Create a filter from an initial estimate and covariance (validated).
    pub fn new(x: X, p: SMatrix<f64, N, N>) -> Result<Self, FilterError> {
        Self::with_tolerances(x, p, Tolerances::default())
    }

    /// Create a filter with explicit numerical tolerances.
    pub fn with_tolerances(
        x: X,
        p: SMatrix<f64, N, N>,
        tol: Tolerances,
    ) -> Result<Self, FilterError> {
        let p = validate_covariance(&p, &tol)?;
        Ok(Self { x, p, tol })
    }

    /// Current estimate.
    #[inline]
    pub fn state(&self) -> &X {
        &self.x
    }
    /// Current covariance (tangent space of the estimate).
    #[inline]
    pub fn covariance(&self) -> &SMatrix<f64, N, N> {
        &self.p
    }
    /// Tolerances in use.
    #[inline]
    pub fn tolerances(&self) -> &Tolerances {
        &self.tol
    }

    /// Prediction: `x̂ ← f(x̂, u)`, `P ← F P Fᵀ + Q`. On error the filter is unchanged.
    pub fn predict<U, P>(&mut self, model: &P, u: &U) -> Result<(), FilterError>
    where
        P: ProcessJacobian<X, U, N>,
    {
        let f = model.jacobian(&self.x, u);
        if f.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("process Jacobian"));
        }
        let x_next = model.propagate(&self.x, u);
        // `x ⊟ x` of a corrupt (NaN) state is NaN; a healthy state gives exactly zero.
        if x_next.boxminus(&x_next).iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("propagated state"));
        }
        let q = validate_covariance(&model.process_noise(&x_next, u), &self.tol)?;
        let p = symmetrize(&(f * self.p * f.transpose() + q));
        self.p = validate_covariance(&p, &self.tol)?;
        self.x = x_next;
        Ok(())
    }

    /// Correction with measurement `z`; returns the correction record (innovation, `S`,
    /// gain, NIS, log-likelihood). On error the filter is unchanged.
    pub fn correct<const M: usize, H>(
        &mut self,
        model: &H,
        z: &SVector<f64, M>,
    ) -> Result<GaussianCorrection<N, M>, FilterError>
    where
        H: MeasurementJacobian<X, N, M>,
    {
        if z.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("measurement"));
        }
        let z_hat = model.predict(&self.x);
        let nu = model.residual(z, &z_hat);
        let h = model.jacobian(&self.x);
        let r = model.measurement_noise(&self.x);
        let c = joseph_correction(&self.p, &h, &r, &nu, &self.tol)?;
        self.x = self.x.boxplus(&c.delta);
        self.p = c.covariance;
        Ok(c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BayesFilter, KalmanFilter};
    use approx::assert_relative_eq;
    use nalgebra::{Matrix1, Matrix2, Vector1, Vector2};
    use robotics_core::Gaussian;

    struct Linear {
        a: Matrix2<f64>,
        q: Matrix2<f64>,
    }
    impl ProcessModel<Vector2<f64>, (), 2> for Linear {
        fn propagate(&self, x: &Vector2<f64>, _: &()) -> Vector2<f64> {
            self.a * x
        }
        fn process_noise(&self, _: &Vector2<f64>, _: &()) -> Matrix2<f64> {
            self.q
        }
    }
    impl ProcessJacobian<Vector2<f64>, (), 2> for Linear {
        fn jacobian(&self, _: &Vector2<f64>, _: &()) -> Matrix2<f64> {
            self.a
        }
    }

    struct PosSensor {
        r: f64,
    }
    impl MeasurementModel<Vector2<f64>, 2, 1> for PosSensor {
        fn predict(&self, x: &Vector2<f64>) -> Vector1<f64> {
            Vector1::new(x[0])
        }
        fn measurement_noise(&self, _: &Vector2<f64>) -> Matrix1<f64> {
            Matrix1::new(self.r)
        }
    }
    impl MeasurementJacobian<Vector2<f64>, 2, 1> for PosSensor {
        fn jacobian(&self, _: &Vector2<f64>) -> SMatrix<f64, 1, 2> {
            SMatrix::<f64, 1, 2>::new(1.0, 0.0)
        }
    }

    /// Book check #9: on a linear system the EKF reproduces the Kalman filter.
    #[test]
    fn linear_system_matches_kalman_filter() {
        let a = Matrix2::new(1.0, 0.1, 0.0, 1.0);
        let q = Matrix2::new(0.01, 0.0, 0.0, 0.02);
        let x0 = Vector2::new(0.0, 1.0);
        let mut ekf = Ekf::new(x0, Matrix2::identity()).unwrap();
        let mut kf = KalmanFilter::<2, 1, 1>::new(
            a,
            SMatrix::<f64, 2, 1>::zeros(),
            SMatrix::<f64, 1, 2>::new(1.0, 0.0),
            q,
            Matrix1::new(0.25),
            Gaussian::new(x0, Matrix2::identity()).unwrap(),
        )
        .unwrap();
        let model = Linear { a, q };
        let sensor = PosSensor { r: 0.25 };
        for (k, z) in [0.3, 0.45, 0.61, 0.7].into_iter().enumerate() {
            if k > 0 {
                ekf.predict(&model, &()).unwrap();
                kf.predict(&Vector1::new(0.0)).unwrap();
            }
            let c = ekf.correct(&sensor, &Vector1::new(z)).unwrap();
            kf.correct(&Vector1::new(z)).unwrap();
            let up = kf.last_update().unwrap();
            assert!((ekf.state() - kf.mean()).norm() < 1e-12);
            assert!((ekf.covariance() - kf.cov()).norm() < 1e-12);
            assert_relative_eq!(c.innovation_cov[(0, 0)], up.s[(0, 0)], epsilon = 1e-12);
            assert_relative_eq!(c.nis, up.nis, epsilon = 1e-12);
        }
    }

    /// Chapter 7 worked example (polar → Cartesian): r ~ N(1, 0.02²), θ ~ N(π/2, (15°)²).
    /// Linearization puts the mean at (0, 1) with σ_x = r·σ_θ = 0.261799 and σ_y = 0.02 —
    /// the EKF's well-known bias (the true E[y] is 0.966311).
    struct PolarToCartesian;
    impl ProcessModel<Vector2<f64>, (), 2> for PolarToCartesian {
        fn propagate(&self, x: &Vector2<f64>, _: &()) -> Vector2<f64> {
            Vector2::new(x[0] * x[1].cos(), x[0] * x[1].sin())
        }
        fn process_noise(&self, _: &Vector2<f64>, _: &()) -> Matrix2<f64> {
            Matrix2::zeros()
        }
    }
    impl ProcessJacobian<Vector2<f64>, (), 2> for PolarToCartesian {
        fn jacobian(&self, x: &Vector2<f64>, _: &()) -> Matrix2<f64> {
            let (s, c) = x[1].sin_cos();
            Matrix2::new(c, -x[0] * s, s, x[0] * c)
        }
    }

    #[test]
    fn worked_example_ch7_polar_linearization() {
        let sig_th = 15f64.to_radians();
        let mut ekf = Ekf::new(
            Vector2::new(1.0, std::f64::consts::FRAC_PI_2),
            Matrix2::new(0.02 * 0.02, 0.0, 0.0, sig_th * sig_th),
        )
        .unwrap();
        ekf.predict(&PolarToCartesian, &()).unwrap();
        assert_relative_eq!(ekf.state()[0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(ekf.state()[1], 1.0, epsilon = 1e-12);
        assert_relative_eq!(ekf.covariance()[(0, 0)].sqrt(), 0.261799, epsilon = 1e-6);
        assert_relative_eq!(ekf.covariance()[(1, 1)].sqrt(), 0.02, epsilon = 1e-12);
    }

    #[test]
    fn covariance_stays_symmetric_and_finite_over_many_steps() {
        let model = Linear {
            a: Matrix2::new(1.0, 0.05, -0.02, 0.99),
            q: Matrix2::new(1e-4, 0.0, 0.0, 1e-4),
        };
        let sensor = PosSensor { r: 1e-2 };
        let mut ekf = Ekf::new(Vector2::zeros(), Matrix2::identity() * 10.0).unwrap();
        for k in 0..2000 {
            ekf.predict(&model, &()).unwrap();
            ekf.correct(&sensor, &Vector1::new((k as f64 * 0.01).sin()))
                .unwrap();
            let p = ekf.covariance();
            assert!(p.iter().all(|v| v.is_finite()));
            assert_eq!((p - p.transpose()).abs().max(), 0.0);
            assert!(p[(0, 0)] > 0.0 && p[(1, 1)] > 0.0);
        }
    }

    #[test]
    fn rejects_bad_inputs_without_touching_the_state() {
        let mut ekf = Ekf::new(Vector2::new(1.0, 2.0), Matrix2::identity()).unwrap();
        let before = (*ekf.state(), *ekf.covariance());
        let e = ekf.correct(&PosSensor { r: 0.1 }, &Vector1::new(f64::INFINITY));
        assert_eq!(e.unwrap_err(), FilterError::NonFinite("measurement"));
        let e = ekf.correct(&PosSensor { r: -1.0 }, &Vector1::new(0.0));
        assert!(matches!(e, Err(FilterError::InvalidCovariance(_))));
        assert_eq!((*ekf.state(), *ekf.covariance()), before);
        assert!(Ekf::new(Vector2::zeros(), Matrix2::new(1.0, 0.0, 0.0, -1.0)).is_err());
    }

    #[test]
    fn tiny_noise_is_handled() {
        let mut ekf = Ekf::new(Vector2::zeros(), Matrix2::identity() * 1e-10).unwrap();
        let c = ekf
            .correct(&PosSensor { r: 1e-10 }, &Vector1::new(1e-5))
            .unwrap();
        assert_relative_eq!(ekf.state()[0], 0.5e-5, epsilon = 1e-15);
        assert!(c.nis.is_finite());
    }
}
