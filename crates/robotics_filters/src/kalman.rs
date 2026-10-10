//! Linear Kalman filter in Joseph form.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 6 — Kalman Filters: The
//! Linear-Gaussian World (`Kf<N, U, M>`); Thrun, Burgard & Fox, Table 3.1.
//!
//! Naming follows Thrun: `A` state transition, `B` control, `C` measurement,
//! **`R` motion noise, `Q` measurement noise**.

use crate::bayes::{BayesFilter, FilterError};
use nalgebra::{linalg::Cholesky, SMatrix, SVector};
use robotics_core::Gaussian;
use std::f64::consts::PI;

/// Diagnostics of the most recent correction step.
#[derive(Debug, Clone, PartialEq)]
pub struct KalmanUpdate<const N: usize, const M: usize> {
    /// Innovation `ν = z − C μ̄`.
    pub innovation: SVector<f64, M>,
    /// Innovation covariance `S = C Σ̄ Cᵀ + Q`.
    pub s: SMatrix<f64, M, M>,
    /// Kalman gain `K = Σ̄ Cᵀ S⁻¹`.
    pub gain: SMatrix<f64, N, M>,
    /// Normalized innovation squared `νᵀ S⁻¹ ν` (χ² with `M` degrees of freedom when consistent).
    pub nis: f64,
    /// `ln p(z | z_{1:t−1})`.
    pub log_evidence: f64,
}

/// `KalmanFilter<N, U, M>`: `N` states, `U` controls, `M` measurements.
///
/// Prediction: `μ̄ = A μ + B u`, `Σ̄ = A Σ Aᵀ + R`.
/// Correction: `S = C Σ̄ Cᵀ + Q`, `K = Σ̄ Cᵀ S⁻¹`, `μ = μ̄ + K(z − C μ̄)`,
/// `Σ = (I − K C) Σ̄ (I − K C)ᵀ + K Q Kᵀ` (Joseph form, symmetric by construction).
#[derive(Debug, Clone, PartialEq)]
pub struct KalmanFilter<const N: usize, const U: usize, const M: usize> {
    a: SMatrix<f64, N, N>,
    b: SMatrix<f64, N, U>,
    c: SMatrix<f64, M, N>,
    r: SMatrix<f64, N, N>,
    q: SMatrix<f64, M, M>,
    belief: Gaussian<N>,
    last_update: Option<KalmanUpdate<N, M>>,
}

impl<const N: usize, const U: usize, const M: usize> KalmanFilter<N, U, M> {
    /// Build a filter from its system matrices and an initial belief.
    pub fn new(
        a: SMatrix<f64, N, N>,
        b: SMatrix<f64, N, U>,
        c: SMatrix<f64, M, N>,
        r: SMatrix<f64, N, N>,
        q: SMatrix<f64, M, M>,
        initial: Gaussian<N>,
    ) -> Result<Self, FilterError> {
        for (name, ok) in [
            ("A", a.iter().all(|v| v.is_finite())),
            ("B", b.iter().all(|v| v.is_finite())),
            ("C", c.iter().all(|v| v.is_finite())),
            ("R", r.iter().all(|v| v.is_finite())),
            ("Q", q.iter().all(|v| v.is_finite())),
        ] {
            if !ok {
                return Err(FilterError::NonFinite(name));
            }
        }
        Ok(Self {
            a,
            b,
            c,
            r,
            q,
            belief: initial,
            last_update: None,
        })
    }

    /// Replace the measurement model (e.g. when a different sensor reports).
    pub fn set_measurement_model(&mut self, c: SMatrix<f64, M, N>, q: SMatrix<f64, M, M>) {
        self.c = c;
        self.q = q;
    }

    /// Diagnostics of the last correction, if any.
    #[inline]
    pub fn last_update(&self) -> Option<&KalmanUpdate<N, M>> {
        self.last_update.as_ref()
    }

    /// Mean of the current belief.
    #[inline]
    pub fn mean(&self) -> &SVector<f64, N> {
        self.belief.mean()
    }

    /// Covariance of the current belief.
    #[inline]
    pub fn cov(&self) -> &SMatrix<f64, N, N> {
        self.belief.cov()
    }
}

impl<const N: usize, const U: usize, const M: usize> BayesFilter for KalmanFilter<N, U, M> {
    type Belief = Gaussian<N>;
    type Control = SVector<f64, U>;
    type Measurement = SVector<f64, M>;

    fn predict(&mut self, u: &SVector<f64, U>) -> Result<(), FilterError> {
        if u.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("control"));
        }
        let mean = self.a * self.belief.mean() + self.b * u;
        let cov = self.a * self.belief.cov() * self.a.transpose() + self.r;
        self.belief = Gaussian::new(mean, cov)?;
        Ok(())
    }

    fn correct(&mut self, z: &SVector<f64, M>) -> Result<f64, FilterError> {
        if z.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("measurement"));
        }
        let (mu, sigma) = (self.belief.mean(), self.belief.cov());
        let innovation = z - self.c * mu;
        let s = self.c * sigma * self.c.transpose() + self.q;
        let chol = Cholesky::new(s).ok_or(FilterError::SingularInnovation)?;
        let gain = sigma * self.c.transpose() * chol.inverse();
        let mean = mu + gain * innovation;
        let ikc = SMatrix::<f64, N, N>::identity() - gain * self.c;
        let cov = ikc * sigma * ikc.transpose() + gain * self.q * gain.transpose();
        self.belief = Gaussian::new(mean, cov)?;

        let nis = innovation.dot(&chol.solve(&innovation));
        let ln_det_s = 2.0 * (0..M).map(|i| chol.l()[(i, i)].ln()).sum::<f64>();
        let log_evidence = -0.5 * (nis + ln_det_s + (M as f64) * (2.0 * PI).ln());
        self.last_update = Some(KalmanUpdate {
            innovation,
            s,
            gain,
            nis,
            log_evidence,
        });
        Ok(log_evidence.exp())
    }

    fn belief(&self) -> &Gaussian<N> {
        &self.belief
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix1, Matrix2, Vector1, Vector2};

    /// Chapter 6 worked example (1-D cart, A = B = C = 1, R = 0.25, Q = 0.5, prior N(0, 1)).
    #[test]
    fn worked_example_1d_cart() {
        let one = Matrix1::new(1.0);
        let prior = Gaussian::new(Vector1::new(0.0), Matrix1::new(1.0)).unwrap();
        let mut kf = KalmanFilter::<1, 1, 1>::new(
            one,
            one,
            one,
            Matrix1::new(0.25),
            Matrix1::new(0.5),
            prior,
        )
        .unwrap();

        kf.predict(&Vector1::new(1.0)).unwrap();
        assert_relative_eq!(kf.mean()[0], 1.0, epsilon = 1e-12);
        assert_relative_eq!(kf.cov()[(0, 0)], 1.25, epsilon = 1e-12);
        kf.correct(&Vector1::new(1.7)).unwrap();
        let up = kf.last_update().unwrap();
        assert_relative_eq!(up.s[(0, 0)], 1.75, epsilon = 1e-12);
        assert_relative_eq!(up.gain[(0, 0)], 5.0 / 7.0, epsilon = 1e-12);
        assert_relative_eq!(kf.mean()[0], 1.5, epsilon = 1e-12);
        assert_relative_eq!(kf.cov()[(0, 0)], 2.5 / 7.0, epsilon = 1e-12);

        kf.predict(&Vector1::new(1.0)).unwrap();
        assert_relative_eq!(kf.mean()[0], 2.5, epsilon = 1e-12);
        assert_relative_eq!(kf.cov()[(0, 0)], 4.25 / 7.0, epsilon = 1e-12);
        kf.correct(&Vector1::new(2.1)).unwrap();
        let up = kf.last_update().unwrap();
        assert_relative_eq!(up.s[(0, 0)], 7.75 / 7.0, epsilon = 1e-12);
        assert_relative_eq!(up.gain[(0, 0)], 4.25 / 7.75, epsilon = 1e-12);
        assert_relative_eq!(kf.mean()[0], 2.280645161290323, epsilon = 1e-12);
        assert_relative_eq!(kf.cov()[(0, 0)], 0.2741935483870968, epsilon = 1e-12);
    }

    /// Book check #8: prior N(0,1), z = 1 with H = 1, R = 1 → K = 0.5, x = 0.5; then a
    /// predict with F = 1, Q = 1 gives P = 1.5.
    #[test]
    fn scalar_update_then_predict() {
        let one = Matrix1::new(1.0);
        let prior = Gaussian::new(Vector1::new(0.0), one).unwrap();
        let mut kf = KalmanFilter::<1, 1, 1>::new(one, one, one, one, one, prior).unwrap();
        let ev = kf.correct(&Vector1::new(1.0)).unwrap();
        assert_relative_eq!(kf.last_update().unwrap().gain[(0, 0)], 0.5, epsilon = 1e-12);
        assert_relative_eq!(kf.mean()[0], 0.5, epsilon = 1e-12);
        // evidence = N(1; 0, 2)
        assert_relative_eq!(
            ev,
            (-0.25f64).exp() / (2.0 * 2.0 * PI).sqrt(),
            epsilon = 1e-12
        );
        kf.predict(&Vector1::new(0.0)).unwrap();
        assert_relative_eq!(kf.cov()[(0, 0)], 1.5, epsilon = 1e-12);
    }

    #[test]
    fn two_state_tracker_stays_consistent() {
        let a = Matrix2::new(1.0, 0.1, 0.0, 1.0);
        let b = SMatrix::<f64, 2, 1>::zeros();
        let c = SMatrix::<f64, 1, 2>::new(1.0, 0.0);
        let r = Matrix2::new(0.01, 0.0, 0.0, 0.02);
        let q = Matrix1::new(0.25);
        let prior = Gaussian::new(Vector2::new(0.0, 1.0), Matrix2::identity()).unwrap();
        let mut kf = KalmanFilter::<2, 1, 1>::new(a, b, c, r, q, prior).unwrap();
        kf.correct(&Vector1::new(0.3)).unwrap();
        let cov = *kf.cov();
        assert!((cov - cov.transpose()).abs().max() < 1e-15);
        // Only position is observed and the prior is uncorrelated, so position variance
        // shrinks while velocity variance is untouched.
        assert!(cov[(0, 0)] < 1.0);
        assert_relative_eq!(cov[(1, 1)], 1.0, epsilon = 1e-12);
        assert!(kf.last_update().unwrap().nis >= 0.0);
    }

    #[test]
    fn rejects_non_finite_measurement() {
        let one = Matrix1::new(1.0);
        let prior = Gaussian::new(Vector1::new(0.0), one).unwrap();
        let mut kf = KalmanFilter::<1, 1, 1>::new(one, one, one, one, one, prior).unwrap();
        assert_eq!(
            kf.correct(&Vector1::new(f64::NAN)),
            Err(FilterError::NonFinite("measurement"))
        );
    }
}
