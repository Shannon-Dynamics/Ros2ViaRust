//! Asynchronous wheel-odometry + gyro fusion around [`EskfSe2`].
//!
//! This is the ROS-independent core of `state_estimator_node`: the node only converts
//! messages into [`ImuMeasurement`] / wheel twists and forwards them here. All timing
//! policy lives in this module so it can be unit-tested without ROS.
//!
//! # Event model
//!
//! Measurements are processed in arrival order but **integrated by their own
//! timestamps**, never by callback or timer frequency. The filter keeps a single time
//! `t_f`. For a measurement stamped `t`:
//!
//! | condition | policy |
//! |---|---|
//! | `t == 0` or non-finite values | rejected (`InvalidTimestamp` / `NonFinite`), counted |
//! | `t < t_f − max_measurement_delay` | rejected (`OutOfOrderMeasurement`), counted |
//! | `t_f − max_measurement_delay ≤ t < t_f` | applied at `t_f` without propagation (late), counted |
//! | `t == t_f` | no propagation; IMU duplicates only refresh the held rate |
//! | `0 < t − t_f ≤ max_dt` | propagate to `t`, then apply |
//! | `t − t_f > max_dt` | gap: state not integrated across it, covariance inflated by `max_dt` of process noise, time re-anchored to `t`, reported (`ExcessiveDt`) |
//!
//! A gyro sample is the **average** rate over the interval ending at its stamp, so the
//! propagation *to* an IMU stamp uses that very sample; events between IMU samples
//! extrapolate with the latest sample (zero-order hold). Without a gyro fresher than
//! `imu_timeout`, propagation uses the wheel yaw rate (wheel-only mode, bias frozen).
//!
//! # Measurements
//!
//! * IMU: provides the propagation input `ω_m`; it is not an ESKF update by itself.
//! * Wheel twist `(v_w, ω_w)`: ESKF update `z = (v_w, ω_w − ω̄_m)` observing speed and gyro
//!   bias (see [`EskfSe2::update_wheel_with_gyro`]). The wheel twist is the average over the
//!   interval `(t_prev, t]` since the previous wheel sample, so `ω̄_m` is the average of the
//!   gyro samples **of that same interval**, taken from a short history buffer. If the
//!   interval is not yet covered by gyro samples (the wheel message overtook the IMU on the
//!   wire), or no gyro is fresh, the update observes speed only. Pairing the wheel rate with
//!   the latest gyro sample instead would inject a bias error at every turn-rate step
//!   whenever messages arrive out of order.
//!
//! # Wheel yaw-rate noise
//!
//! The wheel yaw rate `(Δs_R − Δs_L)/(ℓ Δt)` is corrupted by random slip *and* by
//! calibration errors (wheel radius mismatch, effective track width) that scale with the
//! motion. Its variance is modelled as
//! `σ² = σ₀² + (k_v v)² + (k_ω ω)²` with `σ₀ = wheel_yaw_rate_sigma`,
//! `k_v = wheel_yaw_rate_per_speed` (rad/m) and `k_ω = wheel_yaw_rate_per_turn`. At
//! standstill or in slow motion the wheel rate pins the gyro bias tightly; while driving,
//! the gyro dominates heading. Without the motion terms a radius error of a fraction of a
//! percent is absorbed into the bias estimate and the fused heading inherits the wheel
//! drift.
//!
//! Calibration error is *correlated* over many seconds, which no white-noise variance can
//! represent, so the wheel/gyro yaw-rate difference is only used to update the bias while
//! `|v| ≤ bias_update_max_speed` (standstill and turns in place: a zero-velocity update).
//! Above that speed the wheel update observes speed only. Set the threshold to `+∞` to
//! always use the wheel yaw rate.

use crate::eskf_se2::{idx, EskfCov, EskfError, EskfSe2, PlanarState, ProcessNoise};
use crate::FilterError;
use nalgebra::Matrix3;
use robotics_core::{Cov3, ImuMeasurement, Se2, Timestamp, Twist2};

/// Configuration of the fusion front-end.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FusionConfig {
    /// ESKF process-noise densities.
    pub process: ProcessNoise,
    /// Wheel forward-speed noise σ (m/s).
    pub wheel_speed_sigma: f64,
    /// Wheel yaw-rate noise σ₀ at standstill (rad/s).
    pub wheel_yaw_rate_sigma: f64,
    /// Wheel yaw-rate calibration uncertainty per unit forward speed `k_v` (rad/m), e.g.
    /// 0.03 ≈ ±0.5 % wheel-radius mismatch on a 0.16 m track.
    pub wheel_yaw_rate_per_speed: f64,
    /// Wheel yaw-rate calibration uncertainty relative to the yaw rate `k_ω` (fraction),
    /// e.g. 0.02 ≈ ±2 % effective track width.
    pub wheel_yaw_rate_per_turn: f64,
    /// Largest |forward speed| (m/s) at which the wheel yaw rate updates the gyro bias.
    pub bias_update_max_speed: f64,
    /// Gyro per-sample noise σ (rad/s) used when the message reports no covariance.
    pub gyro_sigma: f64,
    /// Yaw-rate variance density (rad²/s) used to propagate in wheel-only mode.
    pub wheel_only_yaw_var_per_s: f64,
    /// Initial position σ (m).
    pub initial_sigma_xy: f64,
    /// Initial heading σ (rad).
    pub initial_sigma_theta: f64,
    /// Initial speed σ (m/s).
    pub initial_sigma_speed: f64,
    /// Initial gyro-bias σ (rad/s).
    pub initial_sigma_bias: f64,
    /// Largest time step integrated in one go (s).
    pub max_dt: f64,
    /// Largest accepted lateness of a measurement behind the filter time (s).
    pub max_measurement_delay: f64,
    /// Gyro samples older than this are not used for propagation / health (s).
    pub imu_timeout: f64,
    /// Wheel data older than this marks the estimator degraded (s).
    pub wheel_timeout: f64,
    /// How long a transient fault keeps the health `Degraded` (s).
    pub fault_hold: f64,
}

impl Default for FusionConfig {
    fn default() -> Self {
        Self {
            process: ProcessNoise::default(),
            wheel_speed_sigma: 0.02,
            // Derivation for a 0.16 m track, 4096-tick encoders, 50 Hz odometry:
            // one encoder tick per sample ≈ 0.016 rad/s, so quantization contributes
            // ≈ 0.005 rad/s rms while moving (and nothing at standstill); 2 % random
            // slip per wheel contributes ≈ 0.09·v rad/s; ±0.5 % radius mismatch another
            // 0.03·v. k_v = 0.1 rad/m covers both speed-proportional terms.
            wheel_yaw_rate_sigma: 0.005,
            wheel_yaw_rate_per_speed: 0.1,
            wheel_yaw_rate_per_turn: 0.02,
            bias_update_max_speed: 0.05,
            gyro_sigma: 0.005,
            wheel_only_yaw_var_per_s: 0.01,
            initial_sigma_xy: 1e-3,
            initial_sigma_theta: 1e-3,
            initial_sigma_speed: 0.5,
            initial_sigma_bias: 0.02,
            max_dt: 0.5,
            max_measurement_delay: 0.05,
            imu_timeout: 0.2,
            wheel_timeout: 0.5,
            fault_hold: 2.0,
        }
    }
}

impl FusionConfig {
    /// Validate every parameter.
    pub fn validated(self) -> Result<Self, FilterError> {
        self.process.validated()?;
        for (what, v) in [
            ("wheel_speed_sigma", self.wheel_speed_sigma),
            ("wheel_yaw_rate_sigma", self.wheel_yaw_rate_sigma),
            ("gyro_sigma", self.gyro_sigma),
            ("initial_sigma_xy", self.initial_sigma_xy),
            ("initial_sigma_theta", self.initial_sigma_theta),
            ("initial_sigma_speed", self.initial_sigma_speed),
            ("initial_sigma_bias", self.initial_sigma_bias),
            ("max_dt", self.max_dt),
            ("imu_timeout", self.imu_timeout),
            ("wheel_timeout", self.wheel_timeout),
        ] {
            FilterError::check_positive(what, v)?;
        }
        FilterError::check_non_negative("wheel_only_yaw_var_per_s", self.wheel_only_yaw_var_per_s)?;
        FilterError::check_non_negative("wheel_yaw_rate_per_speed", self.wheel_yaw_rate_per_speed)?;
        FilterError::check_non_negative("wheel_yaw_rate_per_turn", self.wheel_yaw_rate_per_turn)?;
        if self.bias_update_max_speed.is_nan() || self.bias_update_max_speed < 0.0 {
            return Err(FilterError::InvalidNoiseParameter {
                what: "bias_update_max_speed",
                value: self.bias_update_max_speed,
            });
        }
        FilterError::check_non_negative("max_measurement_delay", self.max_measurement_delay)?;
        FilterError::check_non_negative("fault_hold", self.fault_hold)?;
        Ok(self)
    }

    /// Variance of a wheel yaw-rate measurement for the given twist (see module docs).
    pub fn wheel_yaw_rate_var(&self, twist: &Twist2) -> f64 {
        self.wheel_yaw_rate_sigma.powi(2)
            + (self.wheel_yaw_rate_per_speed * twist.vx).powi(2)
            + (self.wheel_yaw_rate_per_turn * twist.omega).powi(2)
    }

    fn initial_covariance(&self) -> EskfCov {
        EskfCov::from_diagonal(&EskfError::new(
            self.initial_sigma_xy.powi(2),
            self.initial_sigma_xy.powi(2),
            self.initial_sigma_theta.powi(2),
            self.initial_sigma_speed.powi(2),
            self.initial_sigma_bias.powi(2),
        ))
    }
}

/// Why the estimator is degraded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DegradedReason {
    /// No gyro sample within `imu_timeout`; propagating from wheel yaw rate.
    ImuTimeout,
    /// No wheel data within `wheel_timeout`; speed is held and uncertainty grows.
    WheelTimeout,
    /// A measurement with non-finite values or invalid covariance was rejected recently.
    InvalidMeasurement,
    /// A measurement with a zero or out-of-order timestamp was rejected recently.
    InvalidTimestamp,
    /// A time gap larger than `max_dt` was bridged recently.
    ExcessiveDt,
    /// The filter rejected an update for numerical reasons (singular innovation, invalid
    /// covariance) recently.
    NumericalFailure,
}

impl DegradedReason {
    /// Short machine-readable label.
    pub fn as_str(&self) -> &'static str {
        match self {
            DegradedReason::ImuTimeout => "imu_timeout",
            DegradedReason::WheelTimeout => "wheel_timeout",
            DegradedReason::InvalidMeasurement => "invalid_measurement",
            DegradedReason::InvalidTimestamp => "invalid_timestamp",
            DegradedReason::ExcessiveDt => "excessive_dt",
            DegradedReason::NumericalFailure => "numerical_failure",
        }
    }
}

/// Estimator health.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Health {
    /// No measurement yet, or still waiting for the second sensor within the grace period.
    Initializing,
    /// Both sensors fresh, no recent faults.
    Running,
    /// Operating with the listed problems.
    Degraded(Vec<DegradedReason>),
}

impl Health {
    /// `INITIALIZING` / `RUNNING` / `DEGRADED`.
    pub fn label(&self) -> &'static str {
        match self {
            Health::Initializing => "INITIALIZING",
            Health::Running => "RUNNING",
            Health::Degraded(_) => "DEGRADED",
        }
    }
}

/// What happened to an accepted measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    /// First measurement: the filter was initialized at its stamp.
    Initialized,
    /// Propagated to the measurement time and applied.
    Applied,
    /// Slightly late (within `max_measurement_delay`): applied at the filter time.
    AppliedLate,
    /// Same timestamp as the filter time: no propagation.
    SameTime,
    /// A gap larger than `max_dt` was bridged before applying.
    AfterGap {
        /// The gap length (s).
        dt: f64,
    },
}

/// Counters for diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FusionStats {
    /// Accepted IMU samples.
    pub imu_accepted: u64,
    /// Accepted wheel samples.
    pub wheel_accepted: u64,
    /// Rejected for non-finite values / invalid covariance.
    pub rejected_invalid: u64,
    /// Rejected for zero timestamps.
    pub rejected_timestamp: u64,
    /// Rejected as too old.
    pub rejected_out_of_order: u64,
    /// Applied late within tolerance.
    pub applied_late: u64,
    /// Gaps larger than `max_dt`.
    pub gaps: u64,
    /// Numerical failures inside the filter.
    pub numerical_failures: u64,
    /// Wheel updates performed without a fresh gyro (wheel-only mode).
    pub wheel_only_updates: u64,
}

/// A snapshot for publishing.
#[derive(Debug, Clone, PartialEq)]
pub struct FusedEstimate {
    /// Filter time.
    pub stamp: Timestamp,
    /// Pose of `base_link` in `odom`.
    pub pose: Se2,
    /// Pose covariance with the position block in the `odom` frame (ROS convention).
    pub pose_cov: Cov3,
    /// Body twist `(v, 0, ω)` with `ω = ω_gyro − b̂` (or wheel yaw rate without IMU).
    pub twist: Twist2,
    /// Twist covariance.
    pub twist_cov: Cov3,
    /// Estimated gyro bias (rad/s).
    pub gyro_bias: f64,
    /// σ of the gyro-bias estimate (rad/s).
    pub gyro_bias_sigma: f64,
}

/// Longest gyro history kept for interval averaging (samples).
const GYRO_HISTORY: usize = 512;

#[derive(Debug, Clone, Copy)]
struct HeldRate {
    stamp: Timestamp,
    rate: f64,
    var: f64,
}

/// The wheel + gyro fusion estimator.
#[derive(Debug, Clone)]
pub struct OdomImuFusion {
    cfg: FusionConfig,
    eskf: Option<EskfSe2>,
    time: Option<Timestamp>,
    init_time: Option<Timestamp>,
    gyro: Option<HeldRate>,
    gyro_history: std::collections::VecDeque<HeldRate>,
    wheel: Option<HeldRate>,
    last_fault: Option<(Timestamp, DegradedReason)>,
    stats: FusionStats,
    last_nis: Option<(f64, usize)>,
}

impl OdomImuFusion {
    /// Create an uninitialized estimator with a validated configuration.
    pub fn new(cfg: FusionConfig) -> Result<Self, FilterError> {
        Ok(Self {
            cfg: cfg.validated()?,
            eskf: None,
            time: None,
            init_time: None,
            gyro: None,
            gyro_history: std::collections::VecDeque::with_capacity(GYRO_HISTORY),
            wheel: None,
            last_fault: None,
            stats: FusionStats::default(),
            last_nis: None,
        })
    }

    /// NIS of the most recent wheel correction and its degrees of freedom (2 for the
    /// speed + wheel/gyro-difference update, 1 for a speed-only update). `None` before the
    /// first correction. For consistency monitoring (see [`crate::consistency`]).
    pub fn last_nis(&self) -> Option<(f64, usize)> {
        self.last_nis
    }

    /// The configuration.
    pub fn config(&self) -> &FusionConfig {
        &self.cfg
    }
    /// Diagnostic counters.
    pub fn stats(&self) -> &FusionStats {
        &self.stats
    }
    /// The underlying filter once initialized.
    pub fn filter(&self) -> Option<&EskfSe2> {
        self.eskf.as_ref()
    }
    /// Filter time.
    pub fn time(&self) -> Option<Timestamp> {
        self.time
    }

    fn fault(&mut self, at: Timestamp, reason: DegradedReason, err: FilterError) -> FilterError {
        match reason {
            DegradedReason::InvalidMeasurement => self.stats.rejected_invalid += 1,
            DegradedReason::InvalidTimestamp => {
                if matches!(err, FilterError::OutOfOrderMeasurement { .. }) {
                    self.stats.rejected_out_of_order += 1
                } else {
                    self.stats.rejected_timestamp += 1
                }
            }
            DegradedReason::NumericalFailure => self.stats.numerical_failures += 1,
            DegradedReason::ExcessiveDt => self.stats.gaps += 1,
            _ => {}
        }
        let at = if at == Timestamp::ZERO {
            self.time.unwrap_or(at)
        } else {
            at
        };
        self.last_fault = Some((at, reason));
        err
    }

    fn classify(err: &FilterError) -> DegradedReason {
        match err {
            FilterError::NonFinite(_) | FilterError::InvalidNoiseParameter { .. } => {
                DegradedReason::InvalidMeasurement
            }
            FilterError::InvalidTimestamp | FilterError::OutOfOrderMeasurement { .. } => {
                DegradedReason::InvalidTimestamp
            }
            FilterError::InvalidTimeDelta { .. } => DegradedReason::ExcessiveDt,
            _ => DegradedReason::NumericalFailure,
        }
    }

    fn ensure_initialized(
        &mut self,
        stamp: Timestamp,
        speed: Option<f64>,
    ) -> Result<bool, FilterError> {
        if self.eskf.is_some() {
            return Ok(false);
        }
        let x = PlanarState {
            pose: Se2::IDENTITY,
            speed: speed.unwrap_or(0.0),
            gyro_bias: 0.0,
        };
        self.eskf = Some(EskfSe2::new(
            x,
            self.cfg.initial_covariance(),
            self.cfg.process,
        )?);
        self.time = Some(stamp);
        self.init_time = Some(stamp);
        Ok(true)
    }

    fn gyro_fresh_at(&self, t: Timestamp) -> Option<HeldRate> {
        self.gyro
            .filter(|g| (t - g.stamp).abs() <= self.cfg.imu_timeout)
    }

    /// Insert a gyro sample into the stamp-ordered history (late samples are placed by
    /// stamp, duplicates replace the earlier entry).
    fn record_gyro(&mut self, g: HeldRate) {
        let pos = self.gyro_history.partition_point(|h| h.stamp < g.stamp);
        if self
            .gyro_history
            .get(pos)
            .map(|h| h.stamp == g.stamp)
            .unwrap_or(false)
        {
            self.gyro_history[pos] = g;
        } else {
            self.gyro_history.insert(pos, g);
        }
        while self.gyro_history.len() > GYRO_HISTORY {
            self.gyro_history.pop_front();
        }
    }

    /// Average gyro rate over `(start, end]` and the variance of that average, if the
    /// history covers the interval: at least one sample in it and a sample at or after
    /// `end − ½(end − start)`, so the tail of the interval is not extrapolated.
    fn gyro_average(&self, start: Timestamp, end: Timestamp) -> Option<(f64, f64)> {
        let span = end - start;
        if span <= 0.0 {
            return None;
        }
        let (mut sum, mut var_sum, mut n, mut latest) = (0.0, 0.0, 0usize, f64::NEG_INFINITY);
        for h in &self.gyro_history {
            let rel = h.stamp - start;
            if rel > 0.0 && h.stamp <= end {
                sum += h.rate;
                var_sum += h.var;
                n += 1;
                latest = latest.max(rel);
            }
        }
        if n == 0 || latest < 0.5 * span {
            return None;
        }
        let nf = n as f64;
        Some((sum / nf, var_sum / (nf * nf)))
    }

    /// Move the filter time to `t` (see the module-level policy table).
    fn advance_to(&mut self, t: Timestamp) -> Result<Outcome, FilterError> {
        let tf = self.time.ok_or(FilterError::NotInitialized)?;
        let dt = t - tf;
        if dt < 0.0 {
            if -dt <= self.cfg.max_measurement_delay {
                self.stats.applied_late += 1;
                return Ok(Outcome::AppliedLate);
            }
            return Err(FilterError::OutOfOrderMeasurement { age: -dt });
        }
        if dt == 0.0 {
            return Ok(Outcome::SameTime);
        }
        let eskf = self.eskf.as_mut().ok_or(FilterError::NotInitialized)?;
        if dt > self.cfg.max_dt {
            eskf.inflate(self.cfg.max_dt)?;
            self.time = Some(t);
            self.stats.gaps += 1;
            self.last_fault = Some((t, DegradedReason::ExcessiveDt));
            return Ok(Outcome::AfterGap { dt });
        }
        let gyro = self
            .gyro
            .filter(|g| (t - g.stamp).abs() <= self.cfg.imu_timeout);
        match (gyro, self.wheel) {
            (Some(g), _) => eskf.predict_gyro(g.rate, dt)?,
            (None, Some(w)) => {
                eskf.predict_yaw_rate(w.rate, self.cfg.wheel_only_yaw_var_per_s, dt)?
            }
            (None, None) => eskf.predict_yaw_rate(0.0, self.cfg.wheel_only_yaw_var_per_s, dt)?,
        }
        self.time = Some(t);
        Ok(Outcome::Applied)
    }

    /// Process one IMU sample.
    pub fn on_imu(&mut self, m: &ImuMeasurement) -> Result<Outcome, FilterError> {
        if m.stamp == Timestamp::ZERO {
            return Err(self.fault(
                m.stamp,
                DegradedReason::InvalidTimestamp,
                FilterError::InvalidTimestamp,
            ));
        }
        if let Err(e) = m.validate() {
            return Err(self.fault(m.stamp, DegradedReason::InvalidMeasurement, e.into()));
        }
        let var = m
            .yaw_rate_var
            .unwrap_or(self.cfg.gyro_sigma * self.cfg.gyro_sigma);
        if self.ensure_initialized(m.stamp, None)? {
            self.gyro = Some(HeldRate {
                stamp: m.stamp,
                rate: m.yaw_rate,
                var,
            });
            self.record_gyro(HeldRate {
                stamp: m.stamp,
                rate: m.yaw_rate,
                var,
            });
            self.stats.imu_accepted += 1;
            return Ok(Outcome::Initialized);
        }
        // The sample is the average rate over the interval ending at its stamp: install it
        // *before* propagating so that interval is integrated with it. An older (late)
        // sample never replaces a newer held one.
        let prev = self.gyro;
        if prev.map(|g| m.stamp >= g.stamp).unwrap_or(true) {
            self.gyro = Some(HeldRate {
                stamp: m.stamp,
                rate: m.yaw_rate,
                var,
            });
        }
        let outcome = match self.advance_to(m.stamp) {
            Ok(o) => o,
            Err(e) => {
                self.gyro = prev;
                let r = Self::classify(&e);
                return Err(self.fault(m.stamp, r, e));
            }
        };
        self.record_gyro(HeldRate {
            stamp: m.stamp,
            rate: m.yaw_rate,
            var,
        });
        self.stats.imu_accepted += 1;
        Ok(outcome)
    }

    /// Process one wheel-odometry twist stamped `stamp` (the average velocity over the
    /// interval ending at `stamp`).
    pub fn on_wheel(&mut self, stamp: Timestamp, twist: &Twist2) -> Result<Outcome, FilterError> {
        if stamp == Timestamp::ZERO {
            return Err(self.fault(
                stamp,
                DegradedReason::InvalidTimestamp,
                FilterError::InvalidTimestamp,
            ));
        }
        if !twist.is_finite() {
            return Err(self.fault(
                stamp,
                DegradedReason::InvalidMeasurement,
                FilterError::NonFinite("wheel twist"),
            ));
        }
        let wheel = HeldRate {
            stamp,
            rate: twist.omega,
            var: self.cfg.wheel_yaw_rate_var(twist),
        };
        let initialized = self.ensure_initialized(stamp, Some(twist.vx))?;
        // In wheel-only mode the interval ending at `stamp` is propagated with this very
        // twist's yaw rate, the average over that interval.
        let prev_wheel = self.wheel.replace(wheel);
        let outcome = if initialized {
            Outcome::Initialized
        } else {
            match self.advance_to(stamp) {
                Ok(o) => o,
                Err(e) => {
                    self.wheel = prev_wheel;
                    let r = Self::classify(&e);
                    return Err(self.fault(stamp, r, e));
                }
            }
        };
        let var_v = self.cfg.wheel_speed_sigma * self.cfg.wheel_speed_sigma;
        let gyro = self.gyro_fresh_at(stamp);
        // Gyro average over the same interval as the wheel twist (see module docs).
        let interval_gyro = prev_wheel
            .filter(|p| stamp - p.stamp <= self.cfg.max_dt)
            .and_then(|p| self.gyro_average(p.stamp, stamp));
        let eskf = self.eskf.as_mut().ok_or(FilterError::NotInitialized)?;
        let res = match (gyro, interval_gyro) {
            (Some(_), Some((g_rate, g_var)))
                if twist.vx.abs() <= self.cfg.bias_update_max_speed =>
            {
                eskf.update_wheel_with_gyro(twist.vx, twist.omega, g_rate, var_v, wheel.var, g_var)
                    .map(|c| (c.nis, 2))
            }
            (Some(_), _) => eskf.update_speed(twist.vx, var_v).map(|c| (c.nis, 1)),
            (None, _) => {
                self.stats.wheel_only_updates += 1;
                eskf.update_speed(twist.vx, var_v).map(|c| (c.nis, 1))
            }
        };
        match res {
            Ok(nis) => self.last_nis = Some(nis),
            Err(e) => {
                let r = Self::classify(&e);
                return Err(self.fault(stamp, r, e));
            }
        }
        self.stats.wheel_accepted += 1;
        Ok(outcome)
    }

    /// Health at time `now` (use the same clock as the measurement stamps).
    pub fn health(&self, now: Timestamp) -> Health {
        let Some(init) = self.init_time else {
            return Health::Initializing;
        };
        let grace = now - init < self.cfg.imu_timeout.max(self.cfg.wheel_timeout) * 2.0;
        let imu_seen = self.gyro.is_some();
        let wheel_seen = self.wheel.is_some();
        if grace && (!imu_seen || !wheel_seen) {
            return Health::Initializing;
        }
        let mut reasons = Vec::new();
        if self
            .gyro
            .map(|g| now - g.stamp > self.cfg.imu_timeout)
            .unwrap_or(true)
        {
            reasons.push(DegradedReason::ImuTimeout);
        }
        if self
            .wheel
            .map(|w| now - w.stamp > self.cfg.wheel_timeout)
            .unwrap_or(true)
        {
            reasons.push(DegradedReason::WheelTimeout);
        }
        if let Some((t, r)) = self.last_fault {
            if (now - t).abs() <= self.cfg.fault_hold && !reasons.contains(&r) {
                reasons.push(r);
            }
        }
        if reasons.is_empty() {
            Health::Running
        } else {
            Health::Degraded(reasons)
        }
    }

    /// Current estimate, if initialized.
    pub fn estimate(&self) -> Option<FusedEstimate> {
        let eskf = self.eskf.as_ref()?;
        let t = self.time?;
        let x = eskf.state();
        let p = eskf.covariance();
        let (omega, omega_var) = match self.gyro_fresh_at(t) {
            Some(g) => (g.rate - x.gyro_bias, g.var + p[(idx::B, idx::B)]),
            None => self
                .wheel
                .map(|w| (w.rate, w.var))
                .unwrap_or((0.0, self.cfg.wheel_only_yaw_var_per_s)),
        };
        let mut twist_cov = Matrix3::zeros();
        twist_cov[(0, 0)] = p[(idx::V, idx::V)];
        twist_cov[(1, 1)] = self.cfg.process.slip_diffusion.powi(2);
        twist_cov[(2, 2)] = omega_var;
        Some(FusedEstimate {
            stamp: t,
            pose: x.pose,
            pose_cov: eskf.pose_covariance_world(),
            twist: Twist2::unicycle(x.speed, omega),
            twist_cov,
            gyro_bias: x.gyro_bias,
            gyro_bias_sigma: p[(idx::B, idx::B)].sqrt(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use robotics_core::Manifold;

    fn t(s: f64) -> Timestamp {
        Timestamp::from_secs_f64(1000.0 + s)
    }
    fn imu(s: f64, w: f64) -> ImuMeasurement {
        ImuMeasurement::yaw_rate(t(s), "imu", w, None).unwrap()
    }
    fn fusion() -> OdomImuFusion {
        OdomImuFusion::new(FusionConfig::default()).unwrap()
    }

    #[test]
    fn asynchronous_rates_track_a_circle() {
        // IMU at 200 Hz, wheels at 20 Hz, perfect sensors: fused pose == exact arc.
        let (v, w) = (0.4, 0.3);
        let mut f = fusion();
        let mut k_imu = 0;
        for k in 0..=400 {
            let s = k as f64 * 0.05;
            while (k_imu as f64) * 0.005 <= s + 1e-12 {
                f.on_imu(&imu(k_imu as f64 * 0.005, w)).unwrap();
                k_imu += 1;
            }
            f.on_wheel(t(s), &Twist2::unicycle(v, w)).unwrap();
        }
        let e = f.estimate().unwrap();
        // The last IMU sample (t = 20.0) and wheel sample coincide: filter time is 20 s.
        assert_relative_eq!(e.stamp - t(0.0), 20.0, epsilon = 1e-6);
        let truth = Se2::exp(&robotics_core::Tangent2::new(v * 20.0, 0.0, w * 20.0));
        let err = e.pose.boxminus(&truth);
        assert!(err.norm() < 0.02, "err {err}");
        assert_relative_eq!(e.twist.vx, v, epsilon = 1e-3);
        assert_relative_eq!(e.twist.omega, w, epsilon = 1e-3);
        assert_eq!(f.health(t(20.0)), Health::Running);
    }

    #[test]
    fn timestamp_policies() {
        let mut f = fusion();
        assert_eq!(f.on_imu(&imu(0.0, 0.0)).unwrap(), Outcome::Initialized);
        assert_eq!(f.on_imu(&imu(0.01, 0.0)).unwrap(), Outcome::Applied);
        // Duplicate timestamp.
        assert_eq!(f.on_imu(&imu(0.01, 0.0)).unwrap(), Outcome::SameTime);
        // Slightly late wheel sample: applied without rewinding.
        assert_eq!(
            f.on_wheel(t(0.0), &Twist2::unicycle(0.1, 0.0)).unwrap(),
            Outcome::AppliedLate
        );
        // Far too old (0.1 s behind the filter time, tolerance 0.05 s): rejected.
        assert_eq!(f.on_imu(&imu(0.3, 0.0)).unwrap(), Outcome::Applied);
        let e = f.on_wheel(t(0.2), &Twist2::unicycle(0.1, 0.0)).unwrap_err();
        assert!(matches!(e, FilterError::OutOfOrderMeasurement { .. }));
        // Zero timestamp.
        let z = f
            .on_wheel(Timestamp::ZERO, &Twist2::unicycle(0.1, 0.0))
            .unwrap_err();
        assert_eq!(z, FilterError::InvalidTimestamp);
        assert_eq!(f.stats().rejected_timestamp, 1);
        assert_eq!(f.stats().rejected_out_of_order, 1);
        assert_eq!(f.stats().applied_late, 1);
    }

    #[test]
    fn large_gap_is_bridged_not_integrated() {
        let mut f = fusion();
        f.on_wheel(t(0.0), &Twist2::unicycle(1.0, 0.0)).unwrap();
        f.on_imu(&imu(0.01, 0.0)).unwrap();
        let before = f.estimate().unwrap();
        let tr = f.filter().unwrap().covariance().trace();
        let o = f.on_imu(&imu(5.0, 0.0)).unwrap();
        assert!(
            matches!(o, Outcome::AfterGap { dt } if (dt - 4.99).abs() < 1e-6),
            "{o:?}"
        );
        let after = f.estimate().unwrap();
        assert_eq!(after.pose, before.pose, "a gap must not be integrated");
        assert!(f.filter().unwrap().covariance().trace() > tr);
        assert_eq!(f.stats().gaps, 1);
        match f.health(t(5.0)) {
            Health::Degraded(r) => assert!(r.contains(&DegradedReason::ExcessiveDt)),
            h => panic!("expected degraded, got {h:?}"),
        }
    }

    #[test]
    fn non_finite_measurements_are_rejected_and_counted() {
        let mut f = fusion();
        f.on_imu(&imu(0.0, 0.0)).unwrap();
        let mut bad = imu(0.01, 0.0);
        bad.yaw_rate = f64::NAN;
        assert!(f.on_imu(&bad).is_err());
        assert!(f
            .on_wheel(t(0.02), &Twist2::unicycle(f64::INFINITY, 0.0))
            .is_err());
        assert_eq!(f.stats().rejected_invalid, 2);
        assert!(f.estimate().unwrap().pose.is_finite());
    }

    #[test]
    fn imu_timeout_falls_back_to_wheel_only() {
        let mut f = fusion();
        f.on_imu(&imu(0.0, 0.5)).unwrap();
        for k in 1..=40 {
            f.on_wheel(t(k as f64 * 0.05), &Twist2::unicycle(0.5, 0.5))
                .unwrap();
        }
        // IMU stopped at t=0: after 0.2 s every update is wheel-only.
        assert!(f.stats().wheel_only_updates >= 36);
        let e = f.estimate().unwrap();
        let truth = Se2::exp(&robotics_core::Tangent2::new(0.5 * 2.0, 0.0, 0.5 * 2.0));
        assert!(e.pose.boxminus(&truth).norm() < 0.05);
        match f.health(t(2.0)) {
            Health::Degraded(r) => assert_eq!(r, vec![DegradedReason::ImuTimeout]),
            h => panic!("{h:?}"),
        }
    }

    #[test]
    fn health_transitions() {
        let mut f = fusion();
        assert_eq!(f.health(t(0.0)), Health::Initializing);
        f.on_imu(&imu(0.0, 0.0)).unwrap();
        assert_eq!(f.health(t(0.1)), Health::Initializing);
        f.on_wheel(t(0.05), &Twist2::unicycle(0.0, 0.0)).unwrap();
        f.on_imu(&imu(0.1, 0.0)).unwrap();
        assert_eq!(f.health(t(0.1)), Health::Running);
        match f.health(t(3.0)) {
            Health::Degraded(r) => {
                assert!(r.contains(&DegradedReason::ImuTimeout));
                assert!(r.contains(&DegradedReason::WheelTimeout));
            }
            h => panic!("{h:?}"),
        }
        assert_eq!(Health::Running.label(), "RUNNING");
    }

    /// Wheel messages that arrive after later IMU samples (a normal occurrence over DDS)
    /// must not corrupt the bias at turn-rate steps: the wheel rate is compared with the gyro
    /// average of its own interval, never with the newest held sample.
    #[test]
    fn late_wheel_messages_do_not_bias_the_gyro_at_rate_steps() {
        let run = |wheel_lag_samples: usize| {
            let mut f = fusion();
            let rate = |s: f64| if (s % 4.0) < 2.0 { 0.0 } else { 0.9 }; // spin on/off
            let mut pending: std::collections::VecDeque<f64> = Default::default();
            for k in 0..=6000 {
                let s = k as f64 * 0.005;
                if k % 2 == 0 {
                    f.on_imu(&imu(s, rate(s - 0.005))).unwrap();
                }
                if k % 4 == 0 {
                    pending.push_back(s);
                }
                // Deliver wheel samples `wheel_lag_samples` IMU periods late.
                while pending
                    .front()
                    .map(|t0| s - t0 >= wheel_lag_samples as f64 * 0.01 - 1e-9)
                    .unwrap_or(false)
                {
                    let t0 = pending.pop_front().unwrap();
                    let _ = f.on_wheel(t(t0), &Twist2::unicycle(0.0, rate(t0 - 0.01)));
                }
            }
            f.estimate().unwrap().gyro_bias
        };
        let on_time = run(0);
        let late = run(3);
        assert!(
            on_time.abs() < 1e-3,
            "bias with in-order delivery {on_time}"
        );
        assert!(late.abs() < 1e-3, "bias with late wheel delivery {late}");
    }

    #[test]
    fn invalid_config_is_rejected() {
        let cfg = FusionConfig {
            max_dt: 0.0,
            ..Default::default()
        };
        assert!(OdomImuFusion::new(cfg).is_err());
    }
}
