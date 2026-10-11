//! EKF-SLAM: one Gaussian over the robot pose and every confirmed landmark, with provisional
//! landmarks. A pedagogical back end that sits beside the factor-graph optimizer.
//!
//! # State convention
//! The state is `[x, y, θ, l1x, l1y, l2x, l2y, …]`, dimension `3 + 2N` for `N` landmarks, all in
//! world coordinates, and the covariance is expressed in those same coordinates. This differs
//! on purpose from the SE(2)-tangent covariance convention of the rest of the repository
//! (`robotics_filters`, the factor graph): a landmark is a world-frame point, so a plain
//! Cartesian chart for the pose keeps the pose-landmark Jacobians simple. The [`Se2`] view of
//! the robot is built from the first three entries ([`EkfSlam::pose`]).
//!
//! # Cycle
//! * [`EkfSlam::predict`] moves the pose by a body-frame odometry increment and propagates the
//!   covariance without ever forming the full `(3 + 2N)²` Jacobian: only the robot block and the
//!   pose-landmark strip change, the map-map block is untouched.
//! * [`EkfSlam::update`] takes range-bearing measurements of landmark centres (the caller does
//!   the detection) and processes them one at a time. Each is gated against every confirmed
//!   landmark by its Mahalanobis distance. Inside the gate it updates the nearest landmark.
//!   Between the gate and the larger new-landmark gate it is discarded (a deliberate dead band:
//!   too far to trust as a match, too near to spawn a landmark). Beyond both it feeds a
//!   provisional candidate, and a candidate seen `promote_after` times becomes a landmark.
//!   Provisional candidates are plain averaged world points and are never touched by Kalman
//!   updates.
//! * [`EkfSlam::update_known`] applies measurements with an externally given association.
//!
//! # Limits (stated, not hidden)
//! There is no existence log-odds and no landmark retirement: a landmark created from a
//! spurious cluster of detections stays in the state forever. The filter linearizes once per
//! update, so over long loops with few re-observations it becomes over-confident; the
//! consistency tests in `tests/ekf_slam_it.rs` measure the horizon over which it holds.
//! The cost of a measurement update is `O((3 + 2N)²)` for the covariance, which is acceptable
//! for the few tens of landmarks this back end is meant for.

use crate::SlamError;
use nalgebra::{DMatrix, DVector, Matrix2, Matrix3, SMatrix, Vector2};
use robotics_core::angle::normalize_angle;
use robotics_core::prob::chi2_quantile;
use robotics_core::Se2;

/// The 2×5 measurement Jacobian of one landmark: columns are `(p_x, p_y, p_θ | l_x, l_y)`.
pub type MeasurementJacobian = SMatrix<f64, 2, 5>;

/// A range-bearing measurement of a landmark centre.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RangeBearing {
    /// Distance from the robot to the landmark centre (m), non-negative.
    pub range: f64,
    /// Direction to the landmark relative to the robot heading (rad).
    pub bearing: f64,
}

impl RangeBearing {
    /// The noise-free measurement of `landmark` from `pose`:
    /// `h(p, l) = (‖l − p‖, atan2(l_y − p_y, l_x − p_x) − θ)`, bearing wrapped to `(−π, π]`.
    pub fn observe(pose: &Se2, landmark: &Vector2<f64>) -> Self {
        let d = landmark - pose.translation();
        Self {
            range: d.norm(),
            bearing: normalize_angle(d.y.atan2(d.x) - pose.theta()),
        }
    }

    fn validate(&self) -> Result<(), SlamError> {
        if !(self.range.is_finite() && self.bearing.is_finite()) {
            return Err(SlamError::NonFinite("range-bearing observation"));
        }
        if self.range < 0.0 {
            return Err(SlamError::InvalidParameter {
                what: "observation range",
                value: self.range,
            });
        }
        Ok(())
    }
}

/// The exact 2×5 Jacobian of `h(p, l)` with respect to `(p_x, p_y, p_θ, l_x, l_y)`.
///
/// With `δ = l − p` and `q = ‖δ‖²` the range row is `(−δx/√q, −δy/√q, 0 | δx/√q, δy/√q)` and the
/// bearing row is `(δy/q, −δx/q, −1 | −δy/q, δx/q)`. Fails with
/// [`SlamError::SingularSystem`] when the landmark coincides with the robot, where the bearing
/// is undefined.
pub fn measurement_jacobian(
    pose: &Se2,
    landmark: &Vector2<f64>,
) -> Result<MeasurementJacobian, SlamError> {
    let d = landmark - pose.translation();
    let q = d.norm_squared();
    if q.is_nan() || q <= 1e-18 {
        return Err(SlamError::SingularSystem(
            "landmark coincides with the robot position",
        ));
    }
    let r = q.sqrt();
    Ok(MeasurementJacobian::new(
        -d.x / r,
        -d.y / r,
        0.0,
        d.x / r,
        d.y / r,
        d.y / q,
        -d.x / q,
        -1.0,
        -d.y / q,
        d.x / q,
    ))
}

/// Parameters of [`EkfSlam`].
#[derive(Debug, Clone, PartialEq)]
pub struct EkfSlamConfig {
    /// Standard deviation of the range measurement (m).
    pub sigma_range: f64,
    /// Standard deviation of the bearing measurement (rad).
    pub sigma_bearing: f64,
    /// Probability mass of the χ²(2) association gate; an observation closer than this
    /// quantile to a landmark updates it.
    pub gate_quantile: f64,
    /// Probability mass of the larger χ²(2) gate; beyond it an observation may start or feed a
    /// provisional landmark. Between the two gates it is discarded.
    pub new_gate_quantile: f64,
    /// World-frame radius (m) within which provisional sightings are merged.
    pub candidate_radius: f64,
    /// Number of sightings after which a provisional landmark is promoted.
    pub promote_after: usize,
    /// Covariance of the initial pose in `(x, y, θ)` world coordinates. Zero by default: the
    /// starting pose is the gauge.
    pub initial_pose_cov: Matrix3<f64>,
}

impl Default for EkfSlamConfig {
    fn default() -> Self {
        Self {
            sigma_range: 0.05,
            sigma_bearing: 0.02,
            gate_quantile: 0.99,
            new_gate_quantile: 0.999,
            candidate_radius: 0.35,
            promote_after: 3,
            initial_pose_cov: Matrix3::zeros(),
        }
    }
}

impl EkfSlamConfig {
    /// Check every parameter; the filter constructor calls this.
    pub fn validate(&self) -> Result<(), SlamError> {
        positive("sigma_range", self.sigma_range)?;
        positive("sigma_bearing", self.sigma_bearing)?;
        positive("candidate_radius", self.candidate_radius)?;
        let open_unit = |v: f64| v > 0.0 && v < 1.0;
        if !open_unit(self.gate_quantile) {
            return Err(SlamError::InvalidParameter {
                what: "gate_quantile",
                value: self.gate_quantile,
            });
        }
        if !(open_unit(self.new_gate_quantile) && self.new_gate_quantile >= self.gate_quantile) {
            return Err(SlamError::InvalidParameter {
                what: "new_gate_quantile",
                value: self.new_gate_quantile,
            });
        }
        if self.promote_after == 0 {
            return Err(SlamError::InvalidParameter {
                what: "promote_after",
                value: 0.0,
            });
        }
        check_covariance(&self.initial_pose_cov, "initial_pose_cov")
    }
}

fn positive(what: &'static str, value: f64) -> Result<(), SlamError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(SlamError::InvalidParameter { what, value })
    }
}

/// Finite, symmetric and positive semi-definite up to rounding.
fn check_covariance(m: &Matrix3<f64>, what: &'static str) -> Result<(), SlamError> {
    if m.iter().any(|v| !v.is_finite()) {
        return Err(SlamError::NonFinite(what));
    }
    let scale = 1.0 + m.abs().max();
    let asymmetry = (m - m.transpose()).abs().max();
    if asymmetry > 1e-9 * scale {
        return Err(SlamError::InvalidParameter {
            what,
            value: asymmetry,
        });
    }
    let sym = 0.5 * (m + m.transpose());
    let min_eig = sym.symmetric_eigen().eigenvalues.min();
    if min_eig < -1e-9 * scale {
        return Err(SlamError::InvalidParameter {
            what,
            value: min_eig,
        });
    }
    Ok(())
}

/// What one call to [`EkfSlam::update`] or [`EkfSlam::update_known`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct UpdateReport {
    /// Observations that updated a confirmed landmark.
    pub updated: usize,
    /// Observations discarded in the dead band between the two gates.
    pub discarded: usize,
    /// Provisional candidates started.
    pub provisional_started: usize,
    /// Observations merged into an existing provisional candidate.
    pub provisional_seen: usize,
    /// Candidates promoted to confirmed landmarks.
    pub promoted: usize,
}

/// A provisional landmark: an averaged world point and its sighting count.
#[derive(Debug, Clone, Copy)]
struct Candidate {
    position: Vector2<f64>,
    sightings: usize,
}

/// The EKF-SLAM filter; see the [module documentation](self).
#[derive(Debug, Clone)]
pub struct EkfSlam {
    config: EkfSlamConfig,
    gate: f64,
    new_gate: f64,
    meas_cov: Matrix2<f64>,
    x: DVector<f64>,
    p: DMatrix<f64>,
    candidates: Vec<Candidate>,
}

impl EkfSlam {
    /// Start at `initial` with covariance `config.initial_pose_cov` and no landmarks.
    pub fn new(initial: Se2, config: EkfSlamConfig) -> Result<Self, SlamError> {
        config.validate()?;
        let quantile = |q: f64, what: &'static str| {
            chi2_quantile(2, q).map_err(|_| SlamError::InvalidParameter { what, value: q })
        };
        let gate = quantile(config.gate_quantile, "gate_quantile")?;
        let new_gate = quantile(config.new_gate_quantile, "new_gate_quantile")?;
        let meas_cov = Matrix2::from_diagonal(&nalgebra::Vector2::new(
            config.sigma_range.powi(2),
            config.sigma_bearing.powi(2),
        ));
        if !initial.is_finite() {
            return Err(SlamError::NonFinite("initial pose"));
        }
        let mut p = DMatrix::zeros(3, 3);
        p.view_mut((0, 0), (3, 3))
            .copy_from(&(0.5 * (config.initial_pose_cov + config.initial_pose_cov.transpose())));
        Ok(Self {
            config,
            gate,
            new_gate,
            meas_cov,
            x: DVector::from_vec(vec![initial.x(), initial.y(), initial.theta()]),
            p,
            candidates: Vec::new(),
        })
    }

    /// The robot pose.
    pub fn pose(&self) -> Se2 {
        Se2::new(self.x[0], self.x[1], self.x[2])
    }

    /// The 3×3 `(x, y, θ)` block of the covariance.
    pub fn pose_covariance(&self) -> Matrix3<f64> {
        self.p.fixed_view::<3, 3>(0, 0).into_owned()
    }

    /// Number of confirmed landmarks `N`.
    pub fn landmark_count(&self) -> usize {
        (self.x.len() - 3) / 2
    }

    /// Number of provisional candidates not yet promoted.
    pub fn provisional_count(&self) -> usize {
        self.candidates.len()
    }

    /// World position of landmark `i`.
    ///
    /// # Panics
    /// If `i >= landmark_count()`, like slice indexing.
    pub fn landmark(&self, i: usize) -> Vector2<f64> {
        let k = self.landmark_offset(i);
        Vector2::new(self.x[k], self.x[k + 1])
    }

    /// 2×2 covariance of landmark `i`.
    ///
    /// # Panics
    /// If `i >= landmark_count()`, like slice indexing.
    pub fn landmark_covariance(&self, i: usize) -> Matrix2<f64> {
        let k = self.landmark_offset(i);
        self.p.fixed_view::<2, 2>(k, k).into_owned()
    }

    /// The full state `[x, y, θ, l1x, l1y, …]`.
    pub fn state(&self) -> &DVector<f64> {
        &self.x
    }

    /// The full `(3 + 2N)²` covariance in the coordinates of [`state`](Self::state).
    pub fn covariance(&self) -> &DMatrix<f64> {
        &self.p
    }

    /// Mahalanobis distance `νᵀ S⁻¹ ν` of `obs` to confirmed landmark `k`, the quantity the
    /// association gates are tested against.
    pub fn mahalanobis2(&self, obs: &RangeBearing, k: usize) -> Result<f64, SlamError> {
        obs.validate()?;
        if k >= self.landmark_count() {
            return Err(SlamError::InvalidParameter {
                what: "landmark index",
                value: k as f64,
            });
        }
        self.gate_distance(obs, k).ok_or(SlamError::SingularSystem(
            "innovation covariance of the landmark",
        ))
    }

    /// Propagate the pose by the body-frame odometry increment `delta` (`pose ← pose · delta`)
    /// with noise covariance `r`, given in the body frame in `(x, y, θ)` order.
    ///
    /// In world `(x, y, θ)` coordinates the Jacobians are
    /// `G_x = [[1, 0, −(s δx + c δy)], [0, 1, c δx − s δy], [0, 0, 1]]` with respect to the pose
    /// and `G_u = Rot(θ)` (block-diagonal with a one for the heading) with respect to the
    /// increment, so `G_u R G_uᵀ` is `r` rotated into the world frame. The robot block becomes
    /// `G_x Σ_xx G_xᵀ + G_u R G_uᵀ` and the pose-landmark strip `G_x Σ_xm`.
    pub fn predict(&mut self, delta: &Se2, r: &Matrix3<f64>) -> Result<(), SlamError> {
        if !delta.is_finite() {
            return Err(SlamError::NonFinite("odometry increment"));
        }
        check_covariance(r, "odometry covariance")?;
        let r = 0.5 * (r + r.transpose());
        let pose = self.pose();
        let (s, c) = pose.theta().sin_cos();
        let (dx, dy) = (delta.x(), delta.y());
        let gx = Matrix3::new(
            1.0,
            0.0,
            -(s * dx + c * dy),
            0.0,
            1.0,
            c * dx - s * dy,
            0.0,
            0.0,
            1.0,
        );
        let gu = Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);

        let spp = self.pose_covariance();
        let new_pp = gx * spp * gx.transpose() + gu * r * gu.transpose();
        let new_pp = 0.5 * (new_pp + new_pp.transpose());
        let m = self.x.len() - 3;
        if m > 0 {
            let strip = self.p.view((0, 3), (3, m)).into_owned();
            let new_strip = DMatrix::from_fn(3, m, |i, j| {
                (0..3).map(|k| gx[(i, k)] * strip[(k, j)]).sum::<f64>()
            });
            self.p.view_mut((0, 3), (3, m)).copy_from(&new_strip);
            self.p
                .view_mut((3, 0), (m, 3))
                .copy_from(&new_strip.transpose());
        }
        self.p.view_mut((0, 0), (3, 3)).copy_from(&new_pp);

        let next = pose.compose(delta);
        self.x[0] = next.x();
        self.x[1] = next.y();
        self.x[2] = next.theta();
        Ok(())
    }

    /// Process the measurements one at a time in the given order: gate, then update, discard,
    /// or feed the provisional candidates (see the [module documentation](self)).
    ///
    /// The call is atomic: on error the filter is unchanged.
    pub fn update(&mut self, obs: &[RangeBearing]) -> Result<UpdateReport, SlamError> {
        for z in obs {
            z.validate()?;
        }
        let mut work = self.clone();
        let mut report = UpdateReport::default();
        for z in obs {
            work.process(z, &mut report)?;
        }
        *self = work;
        Ok(report)
    }

    /// Apply measurements with a given association: each pair is `(index of a confirmed
    /// landmark, measurement)`. No gating, no provisional handling. Atomic like
    /// [`update`](Self::update).
    pub fn update_known(
        &mut self,
        obs: &[(usize, RangeBearing)],
    ) -> Result<UpdateReport, SlamError> {
        for (k, z) in obs {
            if *k >= self.landmark_count() {
                return Err(SlamError::InvalidParameter {
                    what: "landmark index",
                    value: *k as f64,
                });
            }
            z.validate()?;
        }
        let mut work = self.clone();
        let mut report = UpdateReport::default();
        for (k, z) in obs {
            work.update_landmark(*k, z)?;
            report.updated += 1;
        }
        *self = work;
        Ok(report)
    }

    fn landmark_offset(&self, i: usize) -> usize {
        assert!(i < self.landmark_count(), "landmark index {i} out of range");
        3 + 2 * i
    }

    /// Indices of the pose and landmark `k` entries in the state.
    fn indices(k: usize) -> [usize; 5] {
        [0, 1, 2, 3 + 2 * k, 4 + 2 * k]
    }

    /// Innovation `z − h(p, l_k)` with the bearing wrapped, and the 2×5 Jacobian.
    fn linearize(
        &self,
        z: &RangeBearing,
        k: usize,
    ) -> Result<(Vector2<f64>, MeasurementJacobian), SlamError> {
        let l = self.landmark(k);
        let pose = self.pose();
        let jac = measurement_jacobian(&pose, &l)?;
        let h = RangeBearing::observe(&pose, &l);
        let nu = Vector2::new(z.range - h.range, normalize_angle(z.bearing - h.bearing));
        Ok((nu, jac))
    }

    /// `S = J C Jᵀ + Q` from the 5×5 covariance sub-block of the pose and landmark `k`.
    fn innovation_covariance(&self, k: usize, jac: &MeasurementJacobian) -> Matrix2<f64> {
        let idx = Self::indices(k);
        let c = SMatrix::<f64, 5, 5>::from_fn(|a, b| self.p[(idx[a], idx[b])]);
        let s = jac * c * jac.transpose() + self.meas_cov;
        0.5 * (s + s.transpose())
    }

    /// `νᵀ S⁻¹ ν` for landmark `k`, or `None` when the geometry is degenerate.
    fn gate_distance(&self, z: &RangeBearing, k: usize) -> Option<f64> {
        let (nu, jac) = self.linearize(z, k).ok()?;
        let s_inv = self.innovation_covariance(k, &jac).try_inverse()?;
        let d = (nu.transpose() * s_inv * nu)[(0, 0)];
        d.is_finite().then_some(d)
    }

    fn process(&mut self, z: &RangeBearing, report: &mut UpdateReport) -> Result<(), SlamError> {
        let mut best: Option<(usize, f64)> = None;
        for k in 0..self.landmark_count() {
            if let Some(d) = self.gate_distance(z, k) {
                if best.is_none_or(|(_, bd)| d < bd) {
                    best = Some((k, d));
                }
            }
        }
        match best {
            Some((j, d)) if d < self.gate => {
                self.update_landmark(j, z)?;
                report.updated += 1;
            }
            Some((_, d)) if d < self.new_gate => report.discarded += 1,
            _ => self.observe_provisionally(z, report),
        }
        Ok(())
    }

    /// One EKF measurement update of landmark `k`: dense gain and state update, Joseph-form
    /// covariance update.
    fn update_landmark(&mut self, k: usize, z: &RangeBearing) -> Result<(), SlamError> {
        let (nu, jac) = self.linearize(z, k)?;
        let s = self.innovation_covariance(k, &jac);
        let s_inv = s
            .try_inverse()
            .ok_or(SlamError::SingularSystem("innovation covariance"))?;
        let n = self.x.len();
        let idx = Self::indices(k);

        // K = Σ Hᵀ S⁻¹, where H is zero outside the five columns in `idx`.
        let sht = DMatrix::from_fn(n, 2, |r, c| {
            (0..5)
                .map(|a| self.p[(r, idx[a])] * jac[(c, a)])
                .sum::<f64>()
        });
        let gain = DMatrix::from_fn(n, 2, |r, c| {
            sht[(r, 0)] * s_inv[(0, c)] + sht[(r, 1)] * s_inv[(1, c)]
        });
        if gain.iter().any(|v| !v.is_finite()) {
            return Err(SlamError::NonFinite("Kalman gain"));
        }

        for r in 0..n {
            self.x[r] += gain[(r, 0)] * nu[0] + gain[(r, 1)] * nu[1];
        }
        self.x[2] = normalize_angle(self.x[2]);

        // Joseph form: (I − K H) Σ (I − K H)ᵀ + K Q Kᵀ.
        let mut a = DMatrix::<f64>::identity(n, n);
        for (c, &col) in idx.iter().enumerate() {
            for r in 0..n {
                a[(r, col)] -= gain[(r, 0)] * jac[(0, c)] + gain[(r, 1)] * jac[(1, c)];
            }
        }
        let q = DMatrix::from_fn(2, 2, |i, j| self.meas_cov[(i, j)]);
        let joseph = &a * &self.p * a.transpose() + &gain * q * gain.transpose();
        self.p = 0.5 * (&joseph + joseph.transpose());
        Ok(())
    }

    /// Start or feed a provisional candidate; promote it when seen often enough.
    fn observe_provisionally(&mut self, z: &RangeBearing, report: &mut UpdateReport) {
        let pose = self.pose();
        let alpha = pose.theta() + z.bearing;
        let world = pose.translation() + z.range * Vector2::new(alpha.cos(), alpha.sin());

        let mut nearest: Option<(usize, f64)> = None;
        for (i, cand) in self.candidates.iter().enumerate() {
            let d = (cand.position - world).norm();
            if d < self.config.candidate_radius && nearest.is_none_or(|(_, nd)| d < nd) {
                nearest = Some((i, d));
            }
        }
        let i = match nearest {
            Some((i, _)) => {
                let cand = &mut self.candidates[i];
                cand.position = 0.5 * cand.position + 0.5 * world;
                cand.sightings += 1;
                report.provisional_seen += 1;
                i
            }
            None => {
                self.candidates.push(Candidate {
                    position: world,
                    sightings: 1,
                });
                report.provisional_started += 1;
                self.candidates.len() - 1
            }
        };
        if self.candidates[i].sightings >= self.config.promote_after {
            self.candidates.remove(i);
            self.add_landmark(z);
            report.promoted += 1;
        }
    }

    /// Append a landmark initialized from the measurement `z` at the current pose.
    ///
    /// Mean `l = p_xy + r (cos α, sin α)`, `α = θ + φ`; `Σ_ll = G_x Σ_pp G_xᵀ + G_z Q G_zᵀ`
    /// and the cross-covariance with every existing entry is `G_x Σ_p•`.
    fn add_landmark(&mut self, z: &RangeBearing) {
        let n = self.x.len();
        let (sa, ca) = (self.x[2] + z.bearing).sin_cos();
        let gx = SMatrix::<f64, 2, 3>::new(1.0, 0.0, -z.range * sa, 0.0, 1.0, z.range * ca);
        let gz = Matrix2::new(ca, -z.range * sa, sa, z.range * ca);
        let sll =
            gx * self.pose_covariance() * gx.transpose() + gz * self.meas_cov * gz.transpose();
        let sl = DMatrix::from_fn(2, n, |i, j| {
            (0..3).map(|k| gx[(i, k)] * self.p[(k, j)]).sum::<f64>()
        });

        let mut p = DMatrix::zeros(n + 2, n + 2);
        p.view_mut((0, 0), (n, n)).copy_from(&self.p);
        p.view_mut((n, 0), (2, n)).copy_from(&sl);
        p.view_mut((0, n), (n, 2)).copy_from(&sl.transpose());
        p.view_mut((n, n), (2, 2))
            .copy_from(&(0.5 * (sll + sll.transpose())));
        self.p = p;

        let mean = Vector2::new(self.x[0] + z.range * ca, self.x[1] + z.range * sa);
        self.x = self.x.clone().insert_rows(n, 2, 0.0);
        self.x[n] = mean.x;
        self.x[n + 1] = mean.y;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn cfg() -> EkfSlamConfig {
        EkfSlamConfig::default()
    }

    fn odom_cov() -> Matrix3<f64> {
        Matrix3::from_diagonal(&nalgebra::Vector3::new(1e-4, 4e-5, 1e-4))
    }

    /// A filter with two landmarks and a dense, correlated covariance.
    fn two_landmark_filter() -> EkfSlam {
        let mut c = cfg();
        c.initial_pose_cov =
            Matrix3::new(0.02, 0.005, 0.001, 0.005, 0.03, -0.002, 0.001, -0.002, 0.01);
        let mut f = EkfSlam::new(Se2::new(0.3, -0.2, 0.7), c).unwrap();
        f.predict(&Se2::new(0.2, 0.05, 0.1), &odom_cov()).unwrap();
        f.add_landmark(&RangeBearing {
            range: 2.0,
            bearing: 0.4,
        });
        f.predict(&Se2::new(0.15, -0.03, 0.2), &odom_cov()).unwrap();
        f.add_landmark(&RangeBearing {
            range: 1.5,
            bearing: -1.1,
        });
        f.predict(&Se2::new(0.1, 0.02, -0.1), &odom_cov()).unwrap();
        assert_eq!(f.landmark_count(), 2);
        f
    }

    fn assert_symmetric_psd(p: &DMatrix<f64>) {
        let scale = 1.0 + p.abs().max();
        assert!(
            (p - p.transpose()).abs().max() <= 1e-12 * scale,
            "not symmetric"
        );
        let min_eig = p.clone().symmetric_eigen().eigenvalues.min();
        assert!(
            min_eig >= -1e-10 * scale,
            "not PSD: min eigenvalue {min_eig}"
        );
    }

    #[test]
    fn measurement_jacobian_matches_central_differences() {
        let pose = Se2::new(0.4, -0.7, 2.9);
        let l = Vector2::new(-1.3, 0.8);
        let jac = measurement_jacobian(&pose, &l).unwrap();
        let eval = |v: [f64; 5]| {
            let m = RangeBearing::observe(&Se2::new(v[0], v[1], v[2]), &Vector2::new(v[3], v[4]));
            Vector2::new(m.range, m.bearing)
        };
        let base = [pose.x(), pose.y(), pose.theta(), l.x, l.y];
        let h = 1e-6;
        for c in 0..5 {
            let (mut up, mut dn) = (base, base);
            up[c] += h;
            dn[c] -= h;
            let mut fd = (eval(up) - eval(dn)) / (2.0 * h);
            // The bearing is only defined modulo 2π; a difference across the branch cut is
            // brought back by the wrap.
            fd[1] = normalize_angle(eval(up)[1] - eval(dn)[1]) / (2.0 * h);
            assert_abs_diff_eq!(jac[(0, c)], fd[0], epsilon = 1e-8);
            assert_abs_diff_eq!(jac[(1, c)], fd[1], epsilon = 1e-8);
        }
        // Landmark on top of the robot has no defined bearing.
        assert!(measurement_jacobian(&pose, &pose.translation()).is_err());
    }

    #[test]
    fn landmark_initialization_and_second_observation_worked_example() {
        let mut c = cfg();
        c.sigma_range = 0.1;
        c.sigma_bearing = 0.05;
        let mut f = EkfSlam::new(Se2::IDENTITY, c).unwrap();
        let z = RangeBearing {
            range: 2.0,
            bearing: 0.0,
        };
        f.add_landmark(&z);
        assert_abs_diff_eq!(f.landmark(0), Vector2::new(2.0, 0.0), epsilon = 1e-15);
        let expected = Matrix2::new(0.01, 0.0, 0.0, 4.0 * 0.05 * 0.05);
        assert_abs_diff_eq!(f.landmark_covariance(0), expected, epsilon = 1e-12);
        assert_abs_diff_eq!(f.landmark_covariance(0)[(1, 1)], 0.01, epsilon = 1e-12);

        let report = f.update_known(&[(0, z)]).unwrap();
        assert_eq!(report.updated, 1);
        let halved = Matrix2::from_diagonal(&nalgebra::Vector2::new(0.005, 0.005));
        assert_abs_diff_eq!(f.landmark_covariance(0), halved, epsilon = 1e-9);
        // The pose was known exactly and stays exactly known.
        assert_abs_diff_eq!(f.pose_covariance(), Matrix3::zeros(), epsilon = 1e-15);
        assert_abs_diff_eq!(f.landmark(0), Vector2::new(2.0, 0.0), epsilon = 1e-12);
    }

    #[test]
    fn initialization_correlates_the_landmark_with_the_pose() {
        let f = two_landmark_filter();
        let p = f.covariance();
        // Σ_l• = G_x Σ_p• at creation; after that the pose-landmark strip is nonzero.
        let strip = p.view((0, 3), (3, 4));
        assert!(strip.abs().max() > 1e-4);
        assert_eq!(p.nrows(), 7);
        assert_symmetric_psd(p);
    }

    #[test]
    fn predict_matches_the_brute_force_full_jacobian() {
        let f0 = two_landmark_filter();
        let delta = Se2::new(0.23, -0.07, 0.31);
        let r = Matrix3::new(2e-4, 1e-5, 0.0, 1e-5, 1e-4, 2e-6, 0.0, 2e-6, 3e-4);
        let mut f = f0.clone();
        f.predict(&delta, &r).unwrap();

        // Full (3+2N)² Jacobians, formed explicitly from first principles.
        let n = f0.state().len();
        let th = f0.state()[2];
        let (s, c) = th.sin_cos();
        let mut big_f = DMatrix::<f64>::identity(n, n);
        big_f[(0, 2)] = -(s * delta.x() + c * delta.y());
        big_f[(1, 2)] = c * delta.x() - s * delta.y();
        let mut big_g = DMatrix::<f64>::zeros(n, 3);
        big_g[(0, 0)] = c;
        big_g[(0, 1)] = -s;
        big_g[(1, 0)] = s;
        big_g[(1, 1)] = c;
        big_g[(2, 2)] = 1.0;
        let rd = DMatrix::from_fn(3, 3, |i, j| r[(i, j)]);
        let expected =
            &big_f * f0.covariance() * big_f.transpose() + &big_g * rd * big_g.transpose();
        assert_abs_diff_eq!(f.covariance(), &expected, epsilon = 1e-10);

        // The analytic G_x agrees with differentiating the motion model.
        let eval = |v: [f64; 3]| Se2::new(v[0], v[1], v[2]).compose(&delta).xytheta();
        let base = [f0.state()[0], f0.state()[1], th];
        let h = 1e-6;
        for col in 0..3 {
            let (mut up, mut dn) = (base, base);
            up[col] += h;
            dn[col] -= h;
            let fd = (eval(up) - eval(dn)) / (2.0 * h);
            for row in 0..2 {
                assert_abs_diff_eq!(big_f[(row, col)], fd[row], epsilon = 1e-8);
            }
        }

        // The state moved by the composition, landmarks untouched, map-map block untouched.
        assert_abs_diff_eq!(
            f.pose().xytheta(),
            f0.pose().compose(&delta).xytheta(),
            epsilon = 1e-15
        );
        assert_eq!(f.landmark(1), f0.landmark(1));
        assert_eq!(
            f.covariance().view((3, 3), (4, 4)),
            f0.covariance().view((3, 3), (4, 4))
        );
        assert_symmetric_psd(f.covariance());
    }

    #[test]
    fn updates_with_known_association_never_grow_the_pose_block() {
        let mut f = two_landmark_filter();
        let truth_pose = f.pose();
        let obs = [
            (0, RangeBearing::observe(&truth_pose, &f.landmark(0))),
            (1, RangeBearing::observe(&truth_pose, &f.landmark(1))),
            (
                0,
                RangeBearing {
                    range: 1.97,
                    bearing: 0.45,
                },
            ),
        ];
        for (k, z) in obs {
            let before = f.pose_covariance().trace();
            let landmark_before = f.landmark_covariance(k).trace();
            f.update_known(&[(k, z)]).unwrap();
            assert!(f.pose_covariance().trace() <= before + 1e-12);
            assert!(f.landmark_covariance(k).trace() <= landmark_before + 1e-12);
            assert_symmetric_psd(f.covariance());
        }
        // And through predict/update alternation.
        for _ in 0..5 {
            f.predict(&Se2::new(0.05, 0.0, 0.02), &odom_cov()).unwrap();
            let z = RangeBearing::observe(&f.pose(), &f.landmark(1));
            let before = f.pose_covariance().trace();
            f.update_known(&[(1, z)]).unwrap();
            assert!(f.pose_covariance().trace() <= before + 1e-12);
            assert_symmetric_psd(f.covariance());
        }
    }

    #[test]
    fn dead_band_discards_between_the_two_gates() {
        let mut f = two_landmark_filter();
        let pose = f.pose();
        let lm = f.landmark(0);
        let exact = RangeBearing::observe(&pose, &lm);
        assert!(f.mahalanobis2(&exact, 0).unwrap() < 1e-12);

        // Displace the bearing until the distance lands between the gates.
        let mut z = exact;
        let mut found = false;
        for i in 0..20_000 {
            z.bearing = exact.bearing + 1e-4 * i as f64;
            let d = f.mahalanobis2(&z, 0).unwrap();
            if d >= f.gate && d < f.new_gate {
                found = true;
                break;
            }
            assert!(d < f.new_gate, "stepped over the dead band");
        }
        assert!(found);
        let (x0, p0) = (f.state().clone(), f.covariance().clone());
        let report = f.update(&[z]).unwrap();
        assert_eq!(
            report,
            UpdateReport {
                discarded: 1,
                ..UpdateReport::default()
            }
        );
        assert_eq!(f.state(), &x0);
        assert_eq!(f.covariance(), &p0);
        assert_eq!(f.provisional_count(), 0);

        // Just inside the gate it updates; far outside both it starts a candidate.
        let report = f.update(&[exact]).unwrap();
        assert_eq!(report.updated, 1);
        let far = RangeBearing {
            range: exact.range,
            bearing: exact.bearing + 1.0,
        };
        let report = f.update(&[far]).unwrap();
        assert_eq!(report.provisional_started, 1);
        assert_eq!(f.provisional_count(), 1);
    }

    #[test]
    fn candidates_are_promoted_on_the_third_sighting() {
        let mut f = EkfSlam::new(Se2::IDENTITY, cfg()).unwrap();
        let z = RangeBearing {
            range: 3.0,
            bearing: 0.5,
        };
        let r1 = f.update(&[z]).unwrap();
        assert_eq!((r1.provisional_started, f.landmark_count()), (1, 0));
        let r2 = f.update(&[z]).unwrap();
        assert_eq!((r2.provisional_seen, f.landmark_count()), (1, 0));
        let r3 = f.update(&[z]).unwrap();
        assert_eq!((r3.provisional_seen, r3.promoted), (1, 1));
        assert_eq!((f.landmark_count(), f.provisional_count()), (1, 0));
        // A fourth sighting now updates the landmark.
        assert_eq!(f.update(&[z]).unwrap().updated, 1);
    }

    #[test]
    fn bad_inputs_are_errors_and_leave_the_filter_unchanged() {
        let bad = |f: &dyn Fn(&mut EkfSlamConfig)| {
            let mut c = cfg();
            f(&mut c);
            EkfSlam::new(Se2::IDENTITY, c).unwrap_err()
        };
        assert!(matches!(
            bad(&|c| c.sigma_range = 0.0),
            SlamError::InvalidParameter {
                what: "sigma_range",
                ..
            }
        ));
        assert!(matches!(
            bad(&|c| c.sigma_bearing = f64::NAN),
            SlamError::InvalidParameter {
                what: "sigma_bearing",
                ..
            }
        ));
        assert!(matches!(
            bad(&|c| c.gate_quantile = 1.0),
            SlamError::InvalidParameter {
                what: "gate_quantile",
                ..
            }
        ));
        assert!(matches!(
            bad(&|c| c.new_gate_quantile = 0.9),
            SlamError::InvalidParameter {
                what: "new_gate_quantile",
                ..
            }
        ));
        assert!(matches!(
            bad(&|c| c.candidate_radius = -1.0),
            SlamError::InvalidParameter {
                what: "candidate_radius",
                ..
            }
        ));
        assert!(matches!(
            bad(&|c| c.promote_after = 0),
            SlamError::InvalidParameter {
                what: "promote_after",
                ..
            }
        ));
        assert!(matches!(
            bad(&|c| c.initial_pose_cov[(0, 0)] = -1.0),
            SlamError::InvalidParameter {
                what: "initial_pose_cov",
                ..
            }
        ));
        assert!(matches!(
            bad(&|c| c.initial_pose_cov[(0, 1)] = 0.5),
            SlamError::InvalidParameter {
                what: "initial_pose_cov",
                ..
            }
        ));
        assert!(matches!(
            bad(&|c| c.initial_pose_cov[(1, 1)] = f64::INFINITY),
            SlamError::NonFinite("initial_pose_cov")
        ));
        assert!(EkfSlam::new(Se2::new(f64::NAN, 0.0, 0.0), cfg()).is_err());

        let mut f = two_landmark_filter();
        let (x0, p0) = (f.state().clone(), f.covariance().clone());
        let good = RangeBearing {
            range: 1.0,
            bearing: 0.1,
        };
        for z in [
            RangeBearing {
                range: f64::NAN,
                bearing: 0.0,
            },
            RangeBearing {
                range: 1.0,
                bearing: f64::INFINITY,
            },
        ] {
            assert_eq!(
                f.update(&[good, z]).unwrap_err(),
                SlamError::NonFinite("range-bearing observation")
            );
            assert_eq!(
                f.update_known(&[(0, z)]).unwrap_err(),
                SlamError::NonFinite("range-bearing observation")
            );
        }
        assert!(matches!(
            f.update(&[RangeBearing {
                range: -1.0,
                bearing: 0.0
            }])
            .unwrap_err(),
            SlamError::InvalidParameter {
                what: "observation range",
                ..
            }
        ));
        assert!(matches!(
            f.update_known(&[(0, good), (2, good)]).unwrap_err(),
            SlamError::InvalidParameter {
                what: "landmark index",
                ..
            }
        ));
        assert!(f.mahalanobis2(&good, 7).is_err());
        assert!(f
            .predict(&Se2::new(f64::NAN, 0.0, 0.0), &odom_cov())
            .is_err());
        let mut asym = odom_cov();
        asym[(0, 1)] = 0.1;
        assert!(f.predict(&Se2::IDENTITY, &asym).is_err());
        assert!(f.predict(&Se2::IDENTITY, &(-odom_cov())).is_err());
        let mut nan = odom_cov();
        nan[(2, 2)] = f64::NAN;
        assert_eq!(
            f.predict(&Se2::IDENTITY, &nan).unwrap_err(),
            SlamError::NonFinite("odometry covariance")
        );
        // Nothing above changed the filter.
        assert_eq!(f.state(), &x0);
        assert_eq!(f.covariance(), &p0);
    }

    #[test]
    fn gates_come_from_the_chi_square_quantiles() {
        let f = EkfSlam::new(Se2::IDENTITY, cfg()).unwrap();
        assert_abs_diff_eq!(f.gate, 9.21, epsilon = 5e-3);
        assert_abs_diff_eq!(f.new_gate, 13.82, epsilon = 5e-3);
    }

    #[test]
    fn identical_inputs_give_bit_identical_results() {
        let run = || {
            let mut f = two_landmark_filter();
            for i in 0..10 {
                f.predict(&Se2::new(0.05, 0.0, 0.03), &odom_cov()).unwrap();
                let pose = f.pose();
                let mut obs = vec![RangeBearing::observe(&pose, &f.landmark(0))];
                obs.push(RangeBearing {
                    range: 2.5 + 0.01 * i as f64,
                    bearing: -0.3,
                });
                f.update(&obs).unwrap();
            }
            f
        };
        let (a, b) = (run(), run());
        let bits = |f: &EkfSlam| {
            f.state()
                .iter()
                .chain(f.covariance().iter())
                .map(|v| v.to_bits())
                .collect::<Vec<_>>()
        };
        assert_eq!(bits(&a), bits(&b));
    }
}
