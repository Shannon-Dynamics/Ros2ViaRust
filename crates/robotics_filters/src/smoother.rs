//! Rauch–Tung–Striebel fixed-interval smoother for linear-Gaussian systems.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 6 (`rts_smooth`, derived from
//! the conditional Gaussian `p(x_t | x_{t+1}, z_{1:t})`); Rauch, Tung & Striebel (1965).
//!
//! A forward Kalman pass keeps, for every step `t = 1…T`, the prediction
//! `(μ̄_t, Σ̄_t)`, the filtered belief `(μ_t, Σ_t)` and the transition `A_t` that produced
//! the prediction from `t − 1`. One backward pass then conditions every state on all
//! measurements:
//!
//! ```text
//! L_t       = Σ_t A_{t+1}ᵀ Σ̄_{t+1}⁻¹                      (smoother gain)
//! μ_{t|T}   = μ_t + L_t (μ_{t+1|T} − μ̄_{t+1})
//! Σ_{t|T}   = Σ_t + L_t (Σ_{t+1|T} − Σ̄_{t+1}) L_tᵀ
//! ```
//!
//! starting from `(μ_{T|T}, Σ_{T|T}) = (μ_T, Σ_T)`. `L_t` is computed with a Cholesky
//! solve (`L_tᵀ = Σ̄_{t+1}⁻¹ A_{t+1} Σ_t`), never an explicit inverse. Since
//! `Σ_{t+1|T} ⪯ Σ̄_{t+1}`, smoothing never increases a covariance.
//!
//! The smoother is pure data in, data out: it never sees a ROS message and never runs
//! inside a node callback.

use crate::bayes::FilterError;
use crate::update::{factor_innovation, symmetrize, Tolerances};
use nalgebra::SMatrix;
use robotics_core::{CoreError, Gaussian};

/// One stored forward step `t ≥ 1`.
#[derive(Debug, Clone, PartialEq)]
pub struct ForwardStep<const N: usize> {
    /// `A_t`, the transition that produced `predicted` from the belief at `t − 1`.
    pub transition: SMatrix<f64, N, N>,
    /// `(μ̄_t, Σ̄_t)`, the belief after `predict`, before `correct`.
    pub predicted: Gaussian<N>,
    /// `(μ_t, Σ_t)`, the belief after `correct` (equal to `predicted` if no measurement
    /// arrived at step `t`).
    pub filtered: Gaussian<N>,
}

/// Recorder for a forward pass plus the backward smoothing pass.
#[derive(Debug, Clone, PartialEq)]
pub struct RtsSmoother<const N: usize> {
    initial: Gaussian<N>,
    steps: Vec<ForwardStep<N>>,
    tol: Tolerances,
}

impl<const N: usize> RtsSmoother<N> {
    /// Start a run from the belief at `t = 0` (the prior).
    pub fn new(initial: Gaussian<N>) -> Self {
        Self {
            initial,
            steps: Vec::new(),
            tol: Tolerances::default(),
        }
    }

    /// Record step `t`: the transition used by `predict`, the predicted belief, and the
    /// filtered belief.
    pub fn push(
        &mut self,
        transition: SMatrix<f64, N, N>,
        predicted: Gaussian<N>,
        filtered: Gaussian<N>,
    ) -> Result<(), FilterError> {
        if transition.iter().any(|v| !v.is_finite()) {
            return Err(FilterError::NonFinite("transition"));
        }
        self.steps.push(ForwardStep {
            transition,
            predicted,
            filtered,
        });
        Ok(())
    }

    /// Number of stored beliefs (`T + 1`, including the prior).
    pub fn len(&self) -> usize {
        self.steps.len() + 1
    }

    /// Never empty: the prior is always stored.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// Filtered beliefs `t = 0…T` (the forward pass as recorded).
    pub fn filtered(&self) -> Vec<Gaussian<N>> {
        std::iter::once(self.initial.clone())
            .chain(self.steps.iter().map(|s| s.filtered.clone()))
            .collect()
    }

    /// Smoothed beliefs `p(x_t | z_{1:T})` for `t = 0…T`.
    pub fn smooth(&self) -> Result<Vec<Gaussian<N>>, FilterError> {
        let filtered = self.filtered();
        let mut out = filtered.clone();
        for t in (0..self.steps.len()).rev() {
            let next = &self.steps[t];
            let (mu, sigma) = (filtered[t].mean(), filtered[t].cov());
            let chol = factor_innovation(next.predicted.cov(), &self.tol)?;
            // Lᵀ = Σ̄⁻¹ A Σ  (Σ̄ and Σ symmetric).
            let gain = chol.solve(&(next.transition * sigma)).transpose();
            let mean = mu + gain * (out[t + 1].mean() - next.predicted.mean());
            let cov = symmetrize(
                &(sigma + gain * (out[t + 1].cov() - next.predicted.cov()) * gain.transpose()),
            );
            out[t] = Gaussian::new(mean, cov)?;
        }
        Ok(out)
    }
}

/// Stateless form: `filtered` has `T + 1` entries (`t = 0…T`), `predicted` and
/// `transitions` have `T` entries (`t = 1…T`).
pub fn rts_smooth<const N: usize>(
    filtered: &[Gaussian<N>],
    predicted: &[Gaussian<N>],
    transitions: &[SMatrix<f64, N, N>],
) -> Result<Vec<Gaussian<N>>, FilterError> {
    let t = filtered.len();
    if t == 0 {
        return Err(FilterError::Core(CoreError::DimensionMismatch {
            expected: 1,
            got: 0,
        }));
    }
    for got in [predicted.len(), transitions.len()] {
        if got != t - 1 {
            return Err(FilterError::Core(CoreError::DimensionMismatch {
                expected: t - 1,
                got,
            }));
        }
    }
    let mut s = RtsSmoother::new(filtered[0].clone());
    for k in 1..t {
        s.push(
            transitions[k - 1],
            predicted[k - 1].clone(),
            filtered[k].clone(),
        )?;
    }
    s.smooth()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BayesFilter, KalmanFilter};
    use approx::assert_relative_eq;
    use nalgebra::{Matrix1, Matrix2, SMatrix, Vector1, Vector2};
    use rand_distr::{Distribution, Normal};
    use robotics_core::seeded_rng;

    fn cart() -> KalmanFilter<1, 1, 1> {
        let one = Matrix1::new(1.0);
        KalmanFilter::new(
            one,
            one,
            one,
            Matrix1::new(0.25),
            Matrix1::new(0.5),
            Gaussian::new(Vector1::new(0.0), one).unwrap(),
        )
        .unwrap()
    }

    /// Chapter 6's 1-D cart (u = 1, z = 1.7, 2.1), smoothed by hand:
    /// `L₁ = (5/14)/(17/28) = 10/17`, `μ₁|₂ = 1.5 + (10/17)(2.280645 − 2.5) = 85/62`,
    /// `Σ₁|₂ = 5/14 + (10/17)²(17/62 − 17/28) = 15/62`; `L₀ = 1/1.25 = 0.8`,
    /// `μ₀|₂ = 0.8(85/62 − 1) = 18.4/62`, `Σ₀|₂ = 1 + 0.64(15/62 − 1.25) = 22/62`.
    #[test]
    fn worked_example_1d_cart() {
        let mut kf = cart();
        let mut rts = RtsSmoother::new(kf.belief().clone());
        for (u, z) in [(1.0, 1.7), (1.0, 2.1)] {
            kf.predict(&Vector1::new(u)).unwrap();
            let predicted = kf.belief().clone();
            kf.correct(&Vector1::new(z)).unwrap();
            rts.push(Matrix1::new(1.0), predicted, kf.belief().clone())
                .unwrap();
        }
        let s = rts.smooth().unwrap();
        assert_eq!(s.len(), 3);
        // The last smoothed belief is the last filtered one.
        assert_relative_eq!(s[2].mean()[0], 2.280645161290323, epsilon = 1e-12);
        assert_relative_eq!(s[2].cov()[(0, 0)], 17.0 / 62.0, epsilon = 1e-12);
        assert_relative_eq!(s[1].mean()[0], 85.0 / 62.0, epsilon = 1e-12);
        assert_relative_eq!(s[1].cov()[(0, 0)], 15.0 / 62.0, epsilon = 1e-12);
        assert_relative_eq!(s[0].mean()[0], 18.4 / 62.0, epsilon = 1e-12);
        assert_relative_eq!(s[0].cov()[(0, 0)], 22.0 / 62.0, epsilon = 1e-12);
    }

    struct Run {
        truth: Vec<Vector2<f64>>,
        filtered: Vec<Gaussian<2>>,
        smoothed: Vec<Gaussian<2>>,
    }

    /// Constant-velocity target, position measured, seeded noise.
    fn tracker_run(seed: u64, steps: usize) -> Run {
        let dt: f64 = 0.1;
        let a = Matrix2::new(1.0, dt, 0.0, 1.0);
        let b = SMatrix::<f64, 2, 1>::zeros();
        let c = SMatrix::<f64, 1, 2>::new(1.0, 0.0);
        let qa = 0.2; // acceleration noise density
        let r = Matrix2::new(
            qa * dt.powi(3) / 3.0,
            qa * dt.powi(2) / 2.0,
            qa * dt.powi(2) / 2.0,
            qa * dt,
        );
        let q = Matrix1::new(0.25);
        let prior = Gaussian::new(Vector2::new(0.0, 1.0), Matrix2::identity()).unwrap();
        let mut kf = KalmanFilter::<2, 1, 1>::new(a, b, c, r, q, prior.clone()).unwrap();
        let mut rts = RtsSmoother::new(prior);
        let mut rng = seeded_rng(seed);
        let process = Gaussian::new(Vector2::zeros(), r).unwrap();
        let meas = Normal::new(0.0, 0.5).unwrap();
        let mut x = Vector2::new(0.0, 1.0);
        let mut truth = vec![x];
        for _ in 0..steps {
            x = a * x + process.sample(&mut rng);
            truth.push(x);
            kf.predict(&Vector1::zeros()).unwrap();
            let predicted = kf.belief().clone();
            kf.correct(&Vector1::new(x[0] + meas.sample(&mut rng)))
                .unwrap();
            rts.push(a, predicted, kf.belief().clone()).unwrap();
        }
        Run {
            truth,
            filtered: rts.filtered(),
            smoothed: rts.smooth().unwrap(),
        }
    }

    #[test]
    fn final_smoothed_state_equals_final_filtered_state() {
        let run = tracker_run(3, 100);
        let (f, s) = (run.filtered.last().unwrap(), run.smoothed.last().unwrap());
        assert_eq!(f.mean(), s.mean());
        assert_eq!(f.cov(), s.cov());
    }

    #[test]
    fn smoothing_never_increases_uncertainty() {
        let run = tracker_run(5, 200);
        for (f, s) in run.filtered.iter().zip(&run.smoothed) {
            let d = f.cov() - s.cov();
            // Σ_filtered − Σ_smoothed ⪰ 0: both eigenvalues non-negative.
            let eig = d.symmetric_eigenvalues();
            assert!(eig.iter().all(|e| *e > -1e-12), "eigenvalues {eig}");
            assert!(s.cov()[(0, 0)] <= f.cov()[(0, 0)] + 1e-12);
        }
    }

    #[test]
    fn smoother_beats_filter_on_a_deterministic_trajectory() {
        let rmse = |est: &[Gaussian<2>], truth: &[Vector2<f64>]| {
            let n = truth.len() as f64;
            (est.iter()
                .zip(truth)
                .map(|(e, t)| (e.mean()[0] - t[0]).powi(2))
                .sum::<f64>()
                / n)
                .sqrt()
        };
        let a = tracker_run(11, 300);
        let b = tracker_run(11, 300);
        // Seeded: bit-identical reruns.
        assert_eq!(a.smoothed, b.smoothed);
        let (rf, rs) = (rmse(&a.filtered, &a.truth), rmse(&a.smoothed, &a.truth));
        assert!(rs < rf, "smoothed {rs} vs filtered {rf}");
        // With σ_z = 0.5 m, both are well below the raw measurement noise.
        assert!(rf < 0.5 && rs < 0.5);
    }

    #[test]
    fn stateless_form_matches_and_validates_lengths() {
        let run = tracker_run(2, 20);
        // Rebuild the inputs from a recorded run and compare.
        let mut kf_prior = run.filtered[0].clone();
        let dt: f64 = 0.1;
        let a = Matrix2::new(1.0, dt, 0.0, 1.0);
        let mut predicted = Vec::new();
        let r = Matrix2::new(
            0.2 * dt.powi(3) / 3.0,
            0.2 * dt.powi(2) / 2.0,
            0.2 * dt.powi(2) / 2.0,
            0.2 * dt,
        );
        for f in &run.filtered[1..] {
            predicted.push(
                Gaussian::new(a * kf_prior.mean(), a * kf_prior.cov() * a.transpose() + r).unwrap(),
            );
            kf_prior = f.clone();
        }
        let s = rts_smooth(&run.filtered, &predicted, &vec![a; 20]).unwrap();
        for (x, y) in s.iter().zip(&run.smoothed) {
            assert!((x.mean() - y.mean()).abs().max() < 1e-12);
        }
        assert!(rts_smooth(&run.filtered, &predicted[1..], &vec![a; 20]).is_err());
        assert!(rts_smooth::<2>(&[], &[], &[]).is_err());
        let mut sm = RtsSmoother::new(run.filtered[0].clone());
        assert!(sm
            .push(
                Matrix2::new(f64::NAN, 0.0, 0.0, 1.0),
                predicted[0].clone(),
                run.filtered[1].clone()
            )
            .is_err());
        assert_eq!(sm.len(), 1);
    }
}
