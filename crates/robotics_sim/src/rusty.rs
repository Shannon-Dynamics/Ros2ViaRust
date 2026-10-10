//! The Rusty differential-drive simulator.

use crate::lidar::{Lidar, LidarConfig};
use crate::trajectory::Trajectory;
use crate::world::World;
use crate::SimError;
use rand::rngs::SmallRng;
use rand_distr::{Distribution, Normal};
use robotics_core::{
    seeded_rng, FrameId, ImuMeasurement, LaserScan, Manifold, Se2, Tangent2, Timestamp, Twist2,
};
use robotics_models::sensor::gyro::GyroNoise;
use robotics_models::{DiffDriveParams, EncoderTicks};
use std::f64::consts::TAU;

/// Simulator configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct RustyConfig {
    /// Kinematic parameters (nominal, what the robot's software believes).
    pub params: DiffDriveParams,
    /// Fixed physics step (s).
    pub physics_dt: f64,
    /// Encoder (joint state) publication rate (Hz); must divide the physics rate.
    pub wheel_rate_hz: f64,
    /// IMU rate (Hz); must divide the physics rate.
    pub imu_rate_hz: f64,
    /// Encoder resolution (ticks per wheel revolution).
    pub ticks_per_rev: u32,
    /// Per-wheel multiplicative slip σ per physics step (dimensionless).
    pub slip_sigma: f64,
    /// Systematic right-wheel radius error `δ_R` (dimensionless, e.g. 0.002 = 0.2 %).
    pub right_radius_error: f64,
    /// Gyro noise model.
    pub gyro: GyroNoise,
    /// Initial gyro bias (rad/s).
    pub gyro_initial_bias: f64,
    /// Master seed for all noise streams.
    pub seed: u64,
    /// Timestamp of simulation time zero.
    pub start: Timestamp,
    /// Frame id of the IMU samples.
    pub imu_frame: FrameId,
    /// LiDAR, if the robot carries one (its rate must divide the physics rate).
    pub lidar: Option<LidarConfig>,
}

impl Default for RustyConfig {
    fn default() -> Self {
        Self {
            params: DiffDriveParams::RUSTY,
            physics_dt: 0.005,
            wheel_rate_hz: 50.0,
            imu_rate_hz: 100.0,
            ticks_per_rev: 4096,
            slip_sigma: 0.02,
            right_radius_error: 0.002,
            gyro: GyroNoise::default(),
            gyro_initial_bias: 0.005,
            seed: robotics_core::BOOK_SEED,
            start: Timestamp::from_secs_f64(1.0),
            imu_frame: FrameId::new("imu_link"),
            lidar: Some(LidarConfig::default()),
        }
    }
}

impl RustyConfig {
    /// A configuration with every noise source disabled (for exactness tests).
    pub fn noiseless() -> Self {
        Self {
            slip_sigma: 0.0,
            right_radius_error: 0.0,
            gyro: GyroNoise {
                noise_density: 0.0,
                bias_random_walk: 0.0,
            },
            gyro_initial_bias: 0.0,
            ticks_per_rev: 1 << 26,
            lidar: Some(LidarConfig::default().noiseless()),
            ..Self::default()
        }
    }

    fn divisor(&self, rate_hz: f64) -> Result<u64, SimError> {
        let physics_hz = 1.0 / self.physics_dt;
        if !(rate_hz.is_finite() && rate_hz > 0.0) {
            return Err(SimError::InvalidParameter {
                what: "sensor rate",
                value: rate_hz,
            });
        }
        let k = physics_hz / rate_hz;
        let kr = k.round();
        if kr < 1.0 || (k - kr).abs() > 1e-6 {
            return Err(SimError::RateMismatch {
                rate_hz,
                physics_hz,
            });
        }
        Ok(kr as u64)
    }

    fn validate(&self) -> Result<(u64, u64, Option<u64>), SimError> {
        self.params.validated()?;
        self.gyro.validated()?;
        if !(self.physics_dt.is_finite() && self.physics_dt > 0.0) {
            return Err(SimError::InvalidParameter {
                what: "physics_dt",
                value: self.physics_dt,
            });
        }
        for (what, v) in [
            ("slip_sigma", self.slip_sigma),
            ("right_radius_error", self.right_radius_error.abs()),
            ("gyro_initial_bias", self.gyro_initial_bias.abs()),
        ] {
            if !v.is_finite() || v < 0.0 {
                return Err(SimError::InvalidParameter { what, value: v });
            }
        }
        if self.ticks_per_rev == 0 {
            return Err(SimError::InvalidParameter {
                what: "ticks_per_rev",
                value: 0.0,
            });
        }
        let lidar_every = match &self.lidar {
            Some(l) => {
                l.validate()?;
                Some(self.divisor(l.rate_hz)?)
            }
            None => None,
        };
        Ok((
            self.divisor(self.wheel_rate_hz)?,
            self.divisor(self.imu_rate_hz)?,
            lidar_every,
        ))
    }
}

/// True state at a physics step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundTruth {
    /// Time.
    pub stamp: Timestamp,
    /// True pose in the world frame (identical to `odom` at start).
    pub pose: Se2,
    /// True body twist over the last step.
    pub twist: Twist2,
    /// True gyro bias (rad/s).
    pub gyro_bias: f64,
}

/// An encoder reading (what a base driver publishes as joint states).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSample {
    /// Sample time.
    pub stamp: Timestamp,
    /// Cumulative quantized encoder counts.
    pub ticks: EncoderTicks,
    /// Left wheel angle derived from the ticks (rad).
    pub left_angle: f64,
    /// Right wheel angle derived from the ticks (rad).
    pub right_angle: f64,
    /// Commanded left wheel rate (rad/s).
    pub left_rate: f64,
    /// Commanded right wheel rate (rad/s).
    pub right_rate: f64,
}

/// Everything produced by one physics step.
#[derive(Debug, Clone, PartialEq)]
pub struct SimStep {
    /// Ground truth after the step.
    pub truth: GroundTruth,
    /// Encoder sample, when due.
    pub wheel: Option<WheelSample>,
    /// IMU sample, when due.
    pub imu: Option<ImuMeasurement>,
    /// LiDAR scan taken at the pose in `truth`, when due.
    pub scan: Option<LaserScan>,
}

/// The simulator.
#[derive(Debug, Clone)]
pub struct RustySim {
    cfg: RustyConfig,
    trajectory: Trajectory,
    world: World,
    lidar: Option<(Lidar, u64)>,
    wheel_every: u64,
    imu_every: u64,
    step: u64,
    pose: Se2,
    wheel_angles: (f64, f64),
    gyro_bias: f64,
    yaw_accum: f64,
    yaw_accum_steps: u64,
    slip_rng: SmallRng,
    gyro_rng: SmallRng,
    bias_rng: SmallRng,
    override_cmd: Option<Twist2>,
}

/// Derive an independent stream seed (SplitMix64 finalizer).
fn stream_seed(seed: u64, stream: u64) -> u64 {
    let mut z = seed ^ stream.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn normal(sigma: f64, rng: &mut SmallRng) -> f64 {
    if sigma > 0.0 {
        Normal::new(0.0, sigma)
            .map(|n| n.sample(rng))
            .unwrap_or(0.0)
    } else {
        0.0
    }
}

impl RustySim {
    /// Create a simulator at the origin of [`World::rusty_arena`].
    pub fn new(cfg: RustyConfig, trajectory: Trajectory) -> Result<Self, SimError> {
        Self::with_world(cfg, trajectory, World::rusty_arena())
    }

    /// Create a simulator at the origin of `world`.
    ///
    /// RNG streams: 1 = wheel slip, 2 = gyro white noise, 3 = gyro bias walk, 4 = LiDAR.
    /// Each is derived from `cfg.seed` independently, so adding or reconfiguring the LiDAR
    /// never changes the wheel or IMU data of a given seed.
    pub fn with_world(
        cfg: RustyConfig,
        trajectory: Trajectory,
        world: World,
    ) -> Result<Self, SimError> {
        let (wheel_every, imu_every, lidar_every) = cfg.validate()?;
        let lidar = match (&cfg.lidar, lidar_every) {
            (Some(l), Some(every)) => {
                Some((Lidar::new(l.clone(), stream_seed(cfg.seed, 4))?, every))
            }
            _ => None,
        };
        Ok(Self {
            slip_rng: seeded_rng(stream_seed(cfg.seed, 1)),
            gyro_rng: seeded_rng(stream_seed(cfg.seed, 2)),
            bias_rng: seeded_rng(stream_seed(cfg.seed, 3)),
            gyro_bias: cfg.gyro_initial_bias,
            cfg,
            trajectory,
            world,
            lidar,
            wheel_every,
            imu_every,
            step: 0,
            pose: Se2::IDENTITY,
            wheel_angles: (0.0, 0.0),
            yaw_accum: 0.0,
            yaw_accum_steps: 0,
            override_cmd: None,
        })
    }

    /// Configuration.
    pub fn config(&self) -> &RustyConfig {
        &self.cfg
    }
    /// Simulation time (s since start).
    pub fn time(&self) -> f64 {
        self.step as f64 * self.cfg.physics_dt
    }
    /// Current stamp.
    pub fn stamp(&self) -> Timestamp {
        self.cfg.start.plus_secs_f64(self.time())
    }
    /// Current true pose.
    pub fn pose(&self) -> Se2 {
        self.pose
    }
    /// The static world the LiDAR sees.
    pub fn world(&self) -> &World {
        &self.world
    }
    /// The LiDAR, if configured.
    pub fn lidar(&self) -> Option<&Lidar> {
        self.lidar.as_ref().map(|(l, _)| l)
    }
    /// The encoder reading at time zero (to initialize odometry before the first step).
    pub fn initial_wheel_sample(&self) -> WheelSample {
        self.wheel_sample(0.0, 0.0)
    }

    /// Replace the scripted command with an external one (e.g. from `/cmd_vel`); `None`
    /// returns to the script.
    pub fn set_command_override(&mut self, cmd: Option<Twist2>) {
        self.override_cmd = cmd.filter(|c| c.is_finite());
    }

    fn quantize(&self, angle: f64) -> i64 {
        (angle * self.cfg.ticks_per_rev as f64 / TAU).round() as i64
    }

    fn wheel_sample(&self, left_rate: f64, right_rate: f64) -> WheelSample {
        let ticks = EncoderTicks {
            left: self.quantize(self.wheel_angles.0),
            right: self.quantize(self.wheel_angles.1),
        };
        let per_tick = TAU / self.cfg.ticks_per_rev as f64;
        WheelSample {
            stamp: self.stamp(),
            ticks,
            left_angle: ticks.left as f64 * per_tick,
            right_angle: ticks.right as f64 * per_tick,
            left_rate,
            right_rate,
        }
    }

    /// Advance one physics step.
    pub fn step(&mut self) -> SimStep {
        let dt = self.cfg.physics_dt;
        let cmd = self
            .override_cmd
            .unwrap_or_else(|| self.trajectory.command(self.time()));
        let p = self.cfg.params;
        let w = p.inverse_kinematics(cmd);
        self.wheel_angles.0 += w.left * dt;
        self.wheel_angles.1 += w.right * dt;

        let eps_l = normal(self.cfg.slip_sigma, &mut self.slip_rng);
        let eps_r = normal(self.cfg.slip_sigma, &mut self.slip_rng);
        let ds_l = p.wheel_radius * w.left * dt * (1.0 + eps_l);
        let ds_r =
            p.wheel_radius * (1.0 + self.cfg.right_radius_error) * w.right * dt * (1.0 + eps_r);
        let tau: Tangent2 = p.wheel_arcs_to_tangent(ds_l, ds_r);
        self.pose = self.pose.boxplus(&tau);
        let twist = Twist2::unicycle(tau[0] / dt, tau[2] / dt);

        self.gyro_bias += normal(self.cfg.gyro.bias_step_sigma(dt), &mut self.bias_rng);
        self.yaw_accum += twist.omega;
        self.yaw_accum_steps += 1;
        self.step += 1;

        let wheel = (self.step % self.wheel_every == 0).then(|| self.wheel_sample(w.left, w.right));
        let imu = if self.step % self.imu_every == 0 {
            let mean_rate = self.yaw_accum / self.yaw_accum_steps as f64;
            self.yaw_accum = 0.0;
            self.yaw_accum_steps = 0;
            let noise = normal(
                self.cfg.gyro.sample_sigma(self.cfg.imu_rate_hz),
                &mut self.gyro_rng,
            );
            let sigma = self.cfg.gyro.sample_sigma(self.cfg.imu_rate_hz);
            ImuMeasurement::yaw_rate(
                self.stamp(),
                self.cfg.imu_frame.clone(),
                mean_rate + self.gyro_bias + noise,
                Some(sigma * sigma),
            )
            .ok()
        } else {
            None
        };
        let stamp = self.stamp();
        let scan = match &mut self.lidar {
            Some((lidar, every)) if self.step % *every == 0 => {
                Some(lidar.scan(&self.world, &self.pose, stamp))
            }
            _ => None,
        };
        SimStep {
            truth: GroundTruth {
                stamp,
                pose: self.pose,
                twist,
                gyro_bias: self.gyro_bias,
            },
            wheel,
            imu,
            scan,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robotics_models::{OdometryNoise, WheelOdometry};

    fn run(cfg: RustyConfig, secs: f64) -> Vec<SimStep> {
        let mut sim = RustySim::new(cfg, Trajectory::mixed(false)).unwrap();
        let n = (secs / sim.config().physics_dt).round() as usize;
        (0..n).map(|_| sim.step()).collect()
    }

    #[test]
    fn deterministic_replay() {
        let a = run(RustyConfig::default(), 20.0);
        let b = run(RustyConfig::default(), 20.0);
        assert_eq!(a, b, "same seed must replay bit-for-bit");
        let c = run(
            RustyConfig {
                seed: 99,
                ..RustyConfig::default()
            },
            20.0,
        );
        assert_ne!(a.last(), c.last());
    }

    #[test]
    fn sensor_rates() {
        let steps = run(RustyConfig::default(), 2.0);
        assert_eq!(steps.len(), 400);
        assert_eq!(steps.iter().filter(|s| s.wheel.is_some()).count(), 100);
        assert_eq!(steps.iter().filter(|s| s.imu.is_some()).count(), 200);
        let imu_stamps: Vec<_> = steps
            .iter()
            .filter_map(|s| s.imu.as_ref().map(|m| m.stamp))
            .collect();
        assert!(imu_stamps
            .windows(2)
            .all(|w| (w[1] - w[0] - 0.01).abs() < 1e-9));
    }

    #[test]
    fn noiseless_encoders_reproduce_ground_truth() {
        let cfg = RustyConfig::noiseless();
        let mut sim = RustySim::new(cfg.clone(), Trajectory::mixed(false)).unwrap();
        let mut odom = WheelOdometry::new(cfg.params, OdometryNoise::default()).unwrap();
        let w0 = sim.initial_wheel_sample();
        odom.update_from_wheel_angles(w0.left_angle, w0.right_angle, w0.stamp);
        let mut last = None;
        for _ in 0..(60.0 / cfg.physics_dt) as usize {
            let s = sim.step();
            if let Some(w) = s.wheel {
                odom.update_from_wheel_angles(w.left_angle, w.right_angle, w.stamp);
            }
            if let Some(m) = &s.imu {
                assert!((m.yaw_rate - s.truth.twist.omega).abs() < 1e-9);
            }
            last = Some(s.truth);
        }
        let truth = last.unwrap();
        // Remaining error comes from 2^26-tick quantization and wheel samples coinciding
        // with the physics grid only every 4 steps.
        assert!(odom.pose().boxminus(&truth.pose).norm() < 1e-4);
    }

    #[test]
    fn rejects_bad_configuration() {
        let bad = RustyConfig {
            imu_rate_hz: 70.0,
            ..RustyConfig::default()
        };
        assert!(matches!(
            RustySim::new(bad, Trajectory::mixed(false)),
            Err(SimError::RateMismatch { .. })
        ));
        let bad = RustyConfig {
            slip_sigma: -0.1,
            ..RustyConfig::default()
        };
        assert!(RustySim::new(bad, Trajectory::mixed(false)).is_err());
    }

    #[test]
    fn scans_are_timed_and_taken_at_the_true_pose() {
        let steps = run(RustyConfig::default(), 2.0);
        let scans: Vec<_> = steps.iter().filter(|s| s.scan.is_some()).collect();
        assert_eq!(scans.len(), 20, "10 Hz over 2 s");
        for s in &scans {
            let scan = s.scan.as_ref().unwrap();
            assert_eq!(scan.stamp, s.truth.stamp, "scan stamp = ground-truth stamp");
            assert_eq!(scan.ranges.len(), 360);
            assert!(scan.validate().is_ok());
        }
        // Noise-free: each scan equals the ideal ray cast from the pose in the same step.
        let mut sim = RustySim::new(RustyConfig::noiseless(), Trajectory::mixed(false)).unwrap();
        for _ in 0..400 {
            let s = sim.step();
            if let Some(scan) = s.scan {
                let ideal = sim
                    .lidar()
                    .unwrap()
                    .ideal_ranges(sim.world(), &s.truth.pose);
                for (r, z) in scan.ranges.iter().zip(ideal) {
                    assert_eq!(*r, z.map_or(f32::INFINITY, |z| z as f32));
                }
            }
        }
    }

    #[test]
    fn lidar_does_not_perturb_the_other_noise_streams() {
        let with = run(RustyConfig::default(), 10.0);
        let without = run(
            RustyConfig {
                lidar: None,
                ..RustyConfig::default()
            },
            10.0,
        );
        for (a, b) in with.iter().zip(&without) {
            assert_eq!(a.truth, b.truth);
            assert_eq!(a.wheel, b.wheel);
            assert_eq!(a.imu, b.imu);
            assert!(b.scan.is_none());
        }
        let bad = RustyConfig {
            lidar: Some(LidarConfig {
                rate_hz: 7.0,
                ..LidarConfig::default()
            }),
            ..RustyConfig::default()
        };
        assert!(matches!(
            RustySim::new(bad, Trajectory::mixed(false)),
            Err(SimError::RateMismatch { .. })
        ));
    }

    #[test]
    fn rusty_arena_is_clear_of_the_trajectory() {
        // 20 repeated laps (~20 min) noise-free, plus 10 min with default noise: the
        // robot never comes within 0.5 m of any wall or obstacle (the world is not a
        // collision model, so this is a design check of the arena).
        for cfg in [RustyConfig::noiseless(), RustyConfig::default()] {
            let secs = if cfg.slip_sigma == 0.0 { 1200.0 } else { 600.0 };
            let mut sim =
                RustySim::new(RustyConfig { lidar: None, ..cfg }, Trajectory::mixed(true)).unwrap();
            let world = World::rusty_arena();
            let mut min_clear = f64::INFINITY;
            for _ in 0..(secs / 0.005) as usize {
                let p = sim.step().truth.pose;
                min_clear = min_clear.min(world.clearance(&p.translation()));
            }
            assert!(min_clear > 0.5, "min clearance {min_clear}");
        }
    }

    #[test]
    fn command_override() {
        let mut sim = RustySim::new(RustyConfig::noiseless(), Trajectory::mixed(false)).unwrap();
        sim.set_command_override(Some(Twist2::unicycle(1.0, 0.0)));
        for _ in 0..200 {
            sim.step();
        }
        assert!((sim.pose().x() - 1.0).abs() < 1e-9);
    }
}
