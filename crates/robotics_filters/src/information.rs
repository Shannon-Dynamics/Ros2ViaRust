//! Linear information filter: the Kalman filter with the belief stored in canonical form.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 6 (`InfoFilter<N, U, M>`);
//! Thrun, Burgard & Fox, Table 3.4. Naming follows Thrun: `A`, `B`, `C`, **`R` motion
//! noise, `Q` measurement noise**.
//!
//! Belief `(ξ, Ω)` with `Ω = Σ⁻¹`, `ξ = Σ⁻¹ μ`.
//!
//! * **Predict** (Table 3.4 lines 2–3, the expensive step here):
//!   `Σ = Ω⁻¹`, `μ = Ω⁻¹ ξ`, `Σ̄ = A Σ Aᵀ + R`, `Ω̄ = Σ̄⁻¹`, `ξ̄ = Ω̄ (A μ + B u)`.
//! * **Correct** (lines 4–5, the cheap step here): `Ω = Ω̄ + Cᵀ Q⁻¹ C`,
//!   `ξ = ξ̄ + Cᵀ Q⁻¹ z`.
//!
//! Numerics: no explicit inverse is formed for solves. `μ` comes from a Cholesky solve
//! `Ω μ = ξ`; `Q⁻¹ C` and `Q⁻¹ z` from Cholesky solves with `Q`; `ξ̄` from the solve
//! `Σ̄ ξ̄ = μ̄`. The two matrices the algorithm *is* defined by — `Σ = Ω⁻¹` and
//! `Ω̄ = Σ̄⁻¹` — come from their Cholesky factors. Every factorization is checked for
//! near-singularity with the same pivot-ratio test the EKF uses ([`Tolerances`]).
//!
//! The canonical form can represent **no information** (`Ω = 0`): corrections may start
//! from an uninformative prior, while prediction and moment recovery need a positive
//! definite `Ω` and report [`FilterError::SingularInnovation`] otherwise.

use crate::bayes::{BayesFilter, FilterError};
use crate::update::{factor_innovation, symmetrize, Tolerances};
use nalgebra::{SMatrix, SVector};
use robotics_core::{Canonical, Gaussian};
use std::f64::consts::PI;

/// `InformationFilter<N, U, M>`: `N` states, `U` controls, `M` measurements.
#[derive(Debug, Clone, PartialEq)]
pub struct InformationFilter<const N: usize, const U: usize, const M: usize> {
    a: SMatrix<f64, N, N>,
    b: SMatrix<f64, N, U>,
    c: SMatrix<f64, M, N>,
    r: SMatrix<f64, N, N>,
    q: SMatrix<f64, M, M>,
    belief: Canonical<N>,
    tol: Tolerances,
}

fn check_finite<const R: usize, const C: usize>(
    m: &SMatrix<f64, R, C>,
    what: &'static str,
) -> Result<(), FilterError> {
    if m.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(FilterError::NonFinite(what))
    }
}

impl<const N: usize, const U: usize, const M: usize> InformationFilter<N, U, M> {
    /// Build from system matrices and a canonical-form prior (which may be uninformative).
    ///
    /// `Q` must be positive definite (it is inverted in every correction); `R` must be a
    /// valid (symmetric positive semi-definite) covariance.
    pub fn new(
        a: SMatrix<f64, N, N>,
        b: SMatrix<f64, N, U>,
        c: SMatrix<f64, M, N>,
        r: SMatrix<f64, N, N>,
        q: SMatrix<f64, M, M>,
        prior: Canonical<N>,
    ) -> Result<Self, FilterError> {
        check_finite(&a, "A")?;
        check_finite(&b, "B")?;
        check_finite(&c, "C")?;
        check_finite(&r, "R")?;
        check_finite(&q, "Q")?;
        check_finite(prior.omega(), "information matrix")?;
        check_finite(prior.xi(), "information vector")?;
        let tol = Tolerances::default();
        let r = crate::update::validate_covariance(&r, &tol)?;
        factor_innovation(&q, &tol)
            .map_err(|_| FilterError::InvalidCovariance("Q must be positive definite"))?;
        Ok(Self {
            a,
            b,
            c,
            r,
            q: symmetrize(&q),
            belief: prior,
            tol,
        })
    }

    /// Build from a moments-form prior (`Ω = Σ⁻¹`, `ξ = Σ⁻¹ μ`).
    pub fn from_gaussian(
        a: SMatrix<f64, N, N>,
        b: SMatrix<f64, N, U>,
        c: SMatrix<f64, M, N>,
        r: SMatrix<f64, N, N>,
        q: SMatrix<f64, M, M>,
        prior: &Gaussian<N>,
    ) -> Result<Self, FilterError> {
        Self::new(a, b, c, r, q, to_information(prior)?)
    }

    /// Information vector `ξ`.
    #[inline]
    pub fn xi(&self) -> &SVector<f64, N> {
        self.belief.xi()
    }

    /// Information matrix `Ω`.
    #[inline]
    pub fn omega(&self) -> &SMatrix<f64, N, N> {
        self.belief.omega()
    }

    /// Recover the mean `μ = Ω⁻¹ ξ` (Cholesky solve).
    pub fn mean(&self) -> Result<SVector<f64, N>, FilterError> {
        let chol = factor_innovation(self.belief.omega(), &self.tol)?;
        Ok(chol.solve(self.belief.xi()))
    }

    /// Recover the covariance `Σ = Ω⁻¹`.
    pub fn covariance(&self) -> Result<SMatrix<f64, N, N>, FilterError> {
        let chol = factor_innovation(self.belief.omega(), &self.tol)?;
        Ok(symmetrize(&chol.inverse()))
    }

    /// The belief in moments form.
    pub fn to_gaussian(&self) -> Result<Gaussian<N>, FilterError> {
        to_moments(&self.belief, &self.tol)
    }

    /// Add the information of one measurement `z = C x + δ`, `δ ~ N(0, Q)` of any
    /// dimension `K` (Table 3.4 lines 4–5 with a sensor-specific `C`, `Q`). Fusing several
    /// sensors in information form is a sum, which is why this is exposed separately.
    pub fn add_information<const K: usize>(
        &mut self,
        z: &SVector<f64, K>,
        c: &SMatrix<f64, K, N>,
        q: &SMatrix<f64, K, K>,
    ) -> Result<(), FilterError> {
        if z.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("measurement"));
        }
        check_finite(c, "C")?;
        check_finite(q, "Q")?;
        let chol_q = factor_innovation(q, &self.tol)?;
        // Cᵀ Q⁻¹ C and Cᵀ Q⁻¹ z via solves with Q (no explicit Q⁻¹).
        let qinv_c = chol_q.solve(c);
        let qinv_z = chol_q.solve(z);
        let omega = symmetrize(&(self.belief.omega() + c.transpose() * qinv_c));
        let xi = self.belief.xi() + c.transpose() * qinv_z;
        check_finite(&omega, "information matrix")?;
        self.belief = Canonical::new_unchecked(xi, omega);
        Ok(())
    }
}

/// Moments → canonical form, `Ω = Σ⁻¹` (from the Cholesky factor), `ξ = Σ⁻¹ μ` (solve).
pub fn to_information<const N: usize>(g: &Gaussian<N>) -> Result<Canonical<N>, FilterError> {
    let tol = Tolerances::default();
    let chol = factor_innovation(g.cov(), &tol)?;
    let omega = symmetrize(&chol.inverse());
    let xi = chol.solve(g.mean());
    Ok(Canonical::new_unchecked(xi, omega))
}

/// Canonical → moments form; fails for a singular or near-singular `Ω`.
pub fn to_moments<const N: usize>(
    c: &Canonical<N>,
    tol: &Tolerances,
) -> Result<Gaussian<N>, FilterError> {
    let chol = factor_innovation(c.omega(), tol)?;
    Ok(Gaussian::new(
        chol.solve(c.xi()),
        symmetrize(&chol.inverse()),
    )?)
}

impl<const N: usize, const U: usize, const M: usize> BayesFilter for InformationFilter<N, U, M> {
    type Belief = Canonical<N>;
    type Control = SVector<f64, U>;
    type Measurement = SVector<f64, M>;

    fn predict(&mut self, u: &SVector<f64, U>) -> Result<(), FilterError> {
        if u.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("control"));
        }
        let chol = factor_innovation(self.belief.omega(), &self.tol)?;
        let sigma = symmetrize(&chol.inverse());
        let mu = chol.solve(self.belief.xi());
        let sigma_bar = symmetrize(&(self.a * sigma * self.a.transpose() + self.r));
        let mu_bar = self.a * mu + self.b * u;
        let chol_bar = factor_innovation(&sigma_bar, &self.tol)?;
        let omega_bar = symmetrize(&chol_bar.inverse());
        let xi_bar = chol_bar.solve(&mu_bar);
        check_finite(&omega_bar, "information matrix")?;
        self.belief = Canonical::new_unchecked(xi_bar, omega_bar);
        Ok(())
    }

    /// Adds this filter's own sensor (`C`, `Q`). Returns the evidence
    /// `N(z; C μ̄, C Σ̄ Cᵀ + Q)` when the prior is proper; for an improper prior (`Ω̄`
    /// singular, e.g. the uninformative start) the evidence density is `0` — the limit of
    /// the proper case as `Σ̄ → ∞` — and the information is still added.
    fn correct(&mut self, z: &SVector<f64, M>) -> Result<f64, FilterError> {
        if z.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("measurement"));
        }
        let evidence = match factor_innovation(self.belief.omega(), &self.tol) {
            Ok(chol) => {
                let sigma = chol.inverse();
                let mu = chol.solve(self.belief.xi());
                let nu = z - self.c * mu;
                let s = symmetrize(&(self.c * sigma * self.c.transpose() + self.q));
                let chol_s = factor_innovation(&s, &self.tol)?;
                let nis = nu.dot(&chol_s.solve(&nu));
                let ln_det = 2.0 * (0..M).map(|i| chol_s.l()[(i, i)].ln()).sum::<f64>();
                (-0.5 * (nis + ln_det + M as f64 * (2.0 * PI).ln())).exp()
            }
            Err(_) => 0.0,
        };
        let (c, q) = (self.c, self.q);
        self.add_information(z, &c, &q)?;
        Ok(evidence)
    }

    fn belief(&self) -> &Canonical<N> {
        &self.belief
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::KalmanFilter;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix1, Matrix2, Vector1, Vector2};

    fn cart_if() -> InformationFilter<1, 1, 1> {
        let one = Matrix1::new(1.0);
        let prior = Gaussian::new(Vector1::new(0.0), one).unwrap();
        InformationFilter::from_gaussian(
            one,
            one,
            one,
            Matrix1::new(0.25),
            Matrix1::new(0.5),
            &prior,
        )
        .unwrap()
    }

    /// Chapter 6: the 1-D cart in canonical coordinates lands on the Kalman posterior
    /// `μ = 2.280645161290323`, `Σ = 0.2741935483870968` after two steps.
    #[test]
    fn worked_example_1d_cart_agrees_with_kalman() {
        let mut inf = cart_if();
        inf.predict(&Vector1::new(1.0)).unwrap();
        assert_relative_eq!(inf.omega()[(0, 0)], 1.0 / 1.25, epsilon = 1e-12);
        assert_relative_eq!(inf.xi()[0], 1.0 / 1.25, epsilon = 1e-12);
        let ev = inf.correct(&Vector1::new(1.7)).unwrap();
        // Evidence N(1.7; 1, 1.75).
        assert_relative_eq!(
            ev,
            (-0.5 * 0.49 / 1.75f64).exp() / (2.0 * PI * 1.75).sqrt(),
            epsilon = 1e-12
        );
        assert_relative_eq!(inf.mean().unwrap()[0], 1.5, epsilon = 1e-12);
        assert_relative_eq!(
            inf.covariance().unwrap()[(0, 0)],
            2.5 / 7.0,
            epsilon = 1e-12
        );
        inf.predict(&Vector1::new(1.0)).unwrap();
        inf.correct(&Vector1::new(2.1)).unwrap();
        assert_relative_eq!(inf.mean().unwrap()[0], 2.280645161290323, epsilon = 1e-12);
        assert_relative_eq!(
            inf.covariance().unwrap()[(0, 0)],
            0.2741935483870968,
            epsilon = 1e-12
        );
    }

    #[test]
    fn moments_information_round_trip() {
        let g = Gaussian::new(Vector2::new(1.5, -0.25), Matrix2::new(2.0, 0.6, 0.6, 0.5)).unwrap();
        let c = to_information(&g).unwrap();
        // Ω Σ = I and ξ = Ω μ.
        assert!((c.omega() * g.cov() - Matrix2::identity()).abs().max() < 1e-12);
        assert!((c.xi() - c.omega() * g.mean()).abs().max() < 1e-12);
        let back = to_moments(&c, &Tolerances::default()).unwrap();
        assert!((back.mean() - g.mean()).abs().max() < 1e-12);
        assert!((back.cov() - g.cov()).abs().max() < 1e-12);
    }

    #[test]
    fn agrees_with_kalman_on_a_two_state_tracker() {
        let a = Matrix2::new(1.0, 0.1, 0.0, 1.0);
        let b = SMatrix::<f64, 2, 1>::new(0.005, 0.1);
        let c = SMatrix::<f64, 1, 2>::new(1.0, 0.0);
        let r = Matrix2::new(1e-4, 0.0, 0.0, 4e-3);
        let q = Matrix1::new(0.04);
        let prior =
            Gaussian::new(Vector2::new(0.0, 0.5), Matrix2::new(0.5, 0.1, 0.1, 0.3)).unwrap();
        let mut kf = KalmanFilter::<2, 1, 1>::new(a, b, c, r, q, prior.clone()).unwrap();
        let mut inf = InformationFilter::<2, 1, 1>::from_gaussian(a, b, c, r, q, &prior).unwrap();
        for k in 0..50 {
            let u = Vector1::new((k as f64 * 0.3).sin());
            let z = Vector1::new(0.05 * k as f64 + 0.1 * (k as f64 * 1.7).cos());
            kf.predict(&u).unwrap();
            inf.predict(&u).unwrap();
            let ek = kf.correct(&z).unwrap();
            let ei = inf.correct(&z).unwrap();
            assert_relative_eq!(ek, ei, epsilon = 1e-9, max_relative = 1e-9);
            let g = inf.to_gaussian().unwrap();
            assert!((g.mean() - kf.mean()).abs().max() < 1e-9, "step {k}");
            assert!((g.cov() - kf.cov()).abs().max() < 1e-9, "step {k}");
        }
    }

    #[test]
    fn sequential_updates_equal_one_stacked_update() {
        // Two scalar sensors observing different components, fused one after the other,
        // must equal a single 2-D measurement with block-diagonal Q (information adds).
        let prior = Gaussian::new(Vector2::new(0.0, 0.0), Matrix2::identity() * 4.0).unwrap();
        let id = Matrix2::identity();
        let mut seq = InformationFilter::<2, 1, 2>::from_gaussian(
            id,
            SMatrix::zeros(),
            id,
            Matrix2::identity() * 0.01,
            Matrix2::new(0.5, 0.0, 0.0, 0.2),
            &prior,
        )
        .unwrap();
        let mut stacked = seq.clone();
        seq.add_information(
            &Vector1::new(1.0),
            &SMatrix::<f64, 1, 2>::new(1.0, 0.0),
            &Matrix1::new(0.5),
        )
        .unwrap();
        seq.add_information(
            &Vector1::new(-2.0),
            &SMatrix::<f64, 1, 2>::new(0.0, 1.0),
            &Matrix1::new(0.2),
        )
        .unwrap();
        stacked.correct(&Vector2::new(1.0, -2.0)).unwrap();
        assert!((seq.omega() - stacked.omega()).abs().max() < 1e-12);
        assert!((seq.xi() - stacked.xi()).abs().max() < 1e-12);
        // Covariance/information consistency: Σ Ω = I.
        let sigma = seq.covariance().unwrap();
        assert!((sigma * seq.omega() - Matrix2::identity()).abs().max() < 1e-12);
        // Scalar check of the first component: 1/(1/4 + 1/0.5) = 0.444…
        assert_relative_eq!(sigma[(0, 0)], 1.0 / (0.25 + 2.0), epsilon = 1e-12);
    }

    #[test]
    fn uninformative_prior_is_representable_and_singularity_is_reported() {
        let id = Matrix2::identity();
        let mut f = InformationFilter::<2, 1, 1>::new(
            id,
            SMatrix::zeros(),
            SMatrix::<f64, 1, 2>::new(1.0, 0.0),
            id * 0.1,
            Matrix1::new(0.5),
            Canonical::uninformative(),
        )
        .unwrap();
        // Moments do not exist yet.
        assert_eq!(f.mean().unwrap_err(), FilterError::SingularInnovation);
        assert!(f.predict(&Vector1::new(0.0)).is_err());
        // Observing x only: still singular in y.
        assert_eq!(f.correct(&Vector1::new(3.0)).unwrap(), 0.0);
        assert!(f.covariance().is_err());
        // Observing y makes it proper.
        f.add_information(
            &Vector1::new(-1.0),
            &SMatrix::<f64, 1, 2>::new(0.0, 1.0),
            &Matrix1::new(0.25),
        )
        .unwrap();
        let m = f.mean().unwrap();
        assert_relative_eq!(m[0], 3.0, epsilon = 1e-12);
        assert_relative_eq!(m[1], -1.0, epsilon = 1e-12);

        // Near-singular information (pivot ratio ~1e-15, below the 1e-14 guard) is refused
        // rather than inverted into garbage.
        let near = Canonical::new_unchecked(
            Vector2::new(1.0, 1.0),
            Matrix2::new(1.0, 1.0, 1.0, 1.0 + 1e-15),
        );
        assert!(to_moments(&near, &Tolerances::default()).is_err());
    }

    #[test]
    fn non_finite_inputs_are_rejected() {
        let mut f = cart_if();
        let before = f.clone();
        assert_eq!(
            f.correct(&Vector1::new(f64::NAN)),
            Err(FilterError::NonFinite("measurement"))
        );
        assert_eq!(
            f.predict(&Vector1::new(f64::INFINITY)),
            Err(FilterError::NonFinite("control"))
        );
        assert_eq!(f, before, "rejected inputs leave the belief untouched");
        let one = Matrix1::new(1.0);
        assert!(InformationFilter::<1, 1, 1>::new(
            one,
            one,
            one,
            one,
            Matrix1::new(0.0),
            Canonical::uninformative()
        )
        .is_err());
        assert!(InformationFilter::<1, 1, 1>::new(
            Matrix1::new(f64::NAN),
            one,
            one,
            one,
            one,
            Canonical::uninformative()
        )
        .is_err());
    }
}
