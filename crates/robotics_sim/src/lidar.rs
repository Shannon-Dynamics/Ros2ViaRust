//! Simulated planar LiDAR: exact ray casting against a [`World`] plus seeded noise.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 4 — the LiDAR forward model and
//! `raycast_scan(m, x_t, lidar)`:
//!
//! ```text
//! T = x_t ∘ T_base_laser                                  (sensor pose)
//! for each beam k:  z*_k = first-hit distance from T along φ_k, capped at z_max
//!                   draw u ~ U[0,1) and ε ~ N(0, σ_r²)     (both, always)
//!                   if u < p_drop or no hit:  z_k = no-return
//!                   else                     z_k = z*_k + ε  (then range policy)
//! ```
//!
//! Drawing both random numbers for every beam, whether or not a branch uses them, keeps
//! the noise on surviving beams identical when the dropout rate changes — two runs that
//! differ in one parameter differ only in that parameter.
//!
//! # Range encoding ([`NoReturnPolicy`])
//! * `Infinity` (default, REP 117): no return / dropout / noisy range above `range_max`
//!   → `+∞`; a return closer than `range_min` → `−∞`.
//! * `MaxRange` (the book's model): no return → `range_max`; ranges clamped to
//!   `[0, range_max]`.
//!
//! The sweep is **instantaneous**: all beams see the pose at the scan stamp
//! (`time_increment = 0`), i.e. there is no motion distortion to undo.

use crate::world::World;
use crate::SimError;
use nalgebra::Vector2;
use rand::rngs::SmallRng;
use rand::RngExt;
use rand_distr::StandardNormal;
use robotics_core::{seeded_rng, FrameId, LaserScan, Se2, Timestamp};
use std::f64::consts::PI;

/// How ranges without a valid return are encoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoReturnPolicy {
    /// REP 117: `+∞` for no return, `−∞` for too close.
    Infinity,
    /// The book's simulator: report `range_max`, clamp to `[0, range_max]`.
    MaxRange,
}

/// LiDAR parameters.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarConfig {
    /// Sensor frame id (`laser`).
    pub frame: FrameId,
    /// Mounting pose `T_base_laser`.
    pub mount: Se2,
    /// Bearing of the first beam (rad, sensor frame).
    pub angle_min: f64,
    /// Bearing of the last beam (rad), inclusive.
    pub angle_max: f64,
    /// Number of beams (≥ 1).
    pub beams: usize,
    /// Minimum valid range (m).
    pub range_min: f64,
    /// Maximum range (m).
    pub range_max: f64,
    /// Scan rate (Hz); must divide the physics rate.
    pub rate_hz: f64,
    /// Range noise σ (m).
    pub range_sigma: f64,
    /// Probability that a beam returns nothing.
    pub dropout_probability: f64,
    /// Encoding of missing returns.
    pub no_return: NoReturnPolicy,
}

impl Default for LidarConfig {
    /// The book's Rusty LiDAR (360 beams over the full circle, 8 m, σ = 2 cm, 1 %
    /// dropout) at 10 Hz, mounted 5 cm ahead of the base.
    fn default() -> Self {
        let beams = 360;
        Self {
            frame: FrameId::new("laser"),
            mount: Se2::new(0.05, 0.0, 0.0),
            angle_min: -PI,
            angle_max: -PI + 2.0 * PI * (beams - 1) as f64 / beams as f64,
            beams,
            range_min: 0.12,
            range_max: 8.0,
            rate_hz: 10.0,
            range_sigma: 0.02,
            dropout_probability: 0.01,
            no_return: NoReturnPolicy::Infinity,
        }
    }
}

impl LidarConfig {
    /// A noise-free copy (σ = 0, no dropout) for exactness tests.
    pub fn noiseless(mut self) -> Self {
        self.range_sigma = 0.0;
        self.dropout_probability = 0.0;
        self
    }

    /// Angular step between beams (0 for a single beam).
    pub fn angle_increment(&self) -> f64 {
        if self.beams > 1 {
            (self.angle_max - self.angle_min) / (self.beams - 1) as f64
        } else {
            0.0
        }
    }

    /// Validate the parameters.
    pub fn validate(&self) -> Result<(), SimError> {
        let bad = |what: &'static str, value: f64| Err(SimError::InvalidParameter { what, value });
        if self.frame.is_empty() {
            return bad("lidar frame", f64::NAN);
        }
        if !self.mount.is_finite() {
            return bad("lidar mount", f64::NAN);
        }
        if self.beams == 0 {
            return bad("lidar beams", 0.0);
        }
        if !(self.angle_min.is_finite() && self.angle_max.is_finite()) {
            return bad("lidar angle range", f64::NAN);
        }
        if self.beams > 1 && self.angle_max <= self.angle_min {
            return bad("lidar angle_max", self.angle_max);
        }
        if self.angle_max - self.angle_min > 2.0 * PI + 1e-9 {
            return bad("lidar field of view", self.angle_max - self.angle_min);
        }
        if !(self.range_min.is_finite() && self.range_min >= 0.0) {
            return bad("lidar range_min", self.range_min);
        }
        if !(self.range_max.is_finite() && self.range_max > self.range_min) {
            return bad("lidar range_max", self.range_max);
        }
        if !(self.rate_hz.is_finite() && self.rate_hz > 0.0) {
            return bad("lidar rate", self.rate_hz);
        }
        if !(self.range_sigma.is_finite() && self.range_sigma >= 0.0) {
            return bad("lidar range_sigma", self.range_sigma);
        }
        if !(0.0..=1.0).contains(&self.dropout_probability) {
            return bad("lidar dropout_probability", self.dropout_probability);
        }
        Ok(())
    }
}

/// A LiDAR with its own noise stream.
#[derive(Debug, Clone)]
pub struct Lidar {
    cfg: LidarConfig,
    rng: SmallRng,
}

impl Lidar {
    /// Create a LiDAR whose noise is drawn from `seed`.
    pub fn new(cfg: LidarConfig, seed: u64) -> Result<Self, SimError> {
        cfg.validate()?;
        Ok(Self {
            cfg,
            rng: seeded_rng(seed),
        })
    }

    /// Configuration.
    pub fn config(&self) -> &LidarConfig {
        &self.cfg
    }

    /// Sensor pose in the world for a robot at `body`.
    pub fn sensor_pose(&self, body: &Se2) -> Se2 {
        body.compose(&self.cfg.mount)
    }

    /// Noise-free first-hit distances `z*_k` (`None` = nothing within `range_max`).
    pub fn ideal_ranges(&self, world: &World, body: &Se2) -> Vec<Option<f64>> {
        let s = self.sensor_pose(body);
        let origin = Vector2::new(s.x(), s.y());
        let inc = self.cfg.angle_increment();
        (0..self.cfg.beams)
            .map(|k| {
                let bearing = self.cfg.angle_min + k as f64 * inc;
                world.raycast(&origin, s.theta() + bearing, self.cfg.range_max)
            })
            .collect()
    }

    /// One sweep from a robot at `body`, stamped `stamp`.
    pub fn scan(&mut self, world: &World, body: &Se2, stamp: Timestamp) -> LaserScan {
        let ideal = self.ideal_ranges(world, body);
        let c = &self.cfg;
        let mut ranges = Vec::with_capacity(c.beams);
        for z_star in ideal {
            let u: f64 = self.rng.random::<f64>();
            let n: f64 = self.rng.sample(StandardNormal);
            let dropped = u < c.dropout_probability;
            let z = match (dropped, z_star) {
                (false, Some(z)) => Some(z + c.range_sigma * n),
                _ => None,
            };
            let r = match (c.no_return, z) {
                (NoReturnPolicy::Infinity, None) => f64::INFINITY,
                (NoReturnPolicy::Infinity, Some(z)) if z > c.range_max => f64::INFINITY,
                (NoReturnPolicy::Infinity, Some(z)) if z < c.range_min => f64::NEG_INFINITY,
                (NoReturnPolicy::Infinity, Some(z)) => z,
                (NoReturnPolicy::MaxRange, None) => c.range_max,
                (NoReturnPolicy::MaxRange, Some(z)) => z.clamp(0.0, c.range_max),
            };
            ranges.push(r as f32);
        }
        LaserScan {
            stamp,
            frame: c.frame.clone(),
            angle_min: c.angle_min,
            angle_increment: c.angle_increment(),
            time_increment: 0.0,
            scan_time: 1.0 / c.rate_hz,
            range_min: c.range_min,
            range_max: c.range_max,
            ranges,
            intensities: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::FRAC_PI_2;

    /// Room [−2, 4] × [−1, 3]; four beams at 0, π/2, π, −π/2 on a sensor at the robot
    /// origin.
    fn room() -> World {
        let mut w = World::empty();
        w.add_rectangle(Vector2::new(-2.0, -1.0), Vector2::new(4.0, 3.0))
            .unwrap();
        w
    }

    fn four_beams() -> LidarConfig {
        LidarConfig {
            mount: Se2::IDENTITY,
            angle_min: -PI,
            angle_max: FRAC_PI_2,
            beams: 4,
            range_max: 10.0,
            ..LidarConfig::default()
        }
        .noiseless()
    }

    #[test]
    fn centered_robot_sees_every_wall() {
        let mut l = Lidar::new(four_beams(), 1).unwrap();
        let s = l.scan(&room(), &Se2::IDENTITY, Timestamp::from_secs_f64(2.0));
        assert!(s.validate().is_ok());
        assert_eq!(s.ranges.len(), 4);
        // Beams at −π (rear), −π/2 (right), 0 (front), π/2 (left).
        assert_relative_eq!(s.ranges[0] as f64, 2.0, epsilon = 1e-6);
        assert_relative_eq!(s.ranges[1] as f64, 1.0, epsilon = 1e-6);
        assert_relative_eq!(s.ranges[2] as f64, 4.0, epsilon = 1e-6);
        assert_relative_eq!(s.ranges[3] as f64, 3.0, epsilon = 1e-6);
        assert_relative_eq!(s.angle_max(), FRAC_PI_2, epsilon = 1e-12);
        assert_eq!(s.stamp, Timestamp::from_secs_f64(2.0));
        assert_eq!(s.frame.as_str(), "laser");
    }

    #[test]
    fn rotated_and_mounted_sensor() {
        let world = room();
        // Robot at (1, 0) facing +y: its front beam hits y = 3 at 3 m, its rear y = −1.
        let mut l = Lidar::new(four_beams(), 1).unwrap();
        let s = l.scan(&world, &Se2::new(1.0, 0.0, FRAC_PI_2), Timestamp::ZERO);
        assert_relative_eq!(s.ranges[2] as f64, 3.0, epsilon = 1e-6); // front
        assert_relative_eq!(s.ranges[0] as f64, 1.0, epsilon = 1e-6); // rear
        assert_relative_eq!(s.ranges[1] as f64, 3.0, epsilon = 1e-6); // right → x = 4
        assert_relative_eq!(s.ranges[3] as f64, 3.0, epsilon = 1e-6); // left → x = −2
                                                                      // A sensor mounted 0.5 m ahead (in the robot's x) sees the front wall 0.5 m closer.
        let mounted = LidarConfig {
            mount: Se2::new(0.5, 0.0, 0.0),
            ..four_beams()
        };
        let mut l = Lidar::new(mounted, 1).unwrap();
        let s = l.scan(&world, &Se2::new(1.0, 0.0, FRAC_PI_2), Timestamp::ZERO);
        assert_relative_eq!(s.ranges[2] as f64, 2.5, epsilon = 1e-6);
        assert_relative_eq!(s.ranges[0] as f64, 1.5, epsilon = 1e-6);
    }

    #[test]
    fn no_hit_is_a_max_range_return() {
        let short = LidarConfig {
            range_max: 2.5,
            ..four_beams()
        };
        let mut l = Lidar::new(short.clone(), 1).unwrap();
        let s = l.scan(&room(), &Se2::IDENTITY, Timestamp::ZERO);
        // front (4 m) and left (3 m) are beyond 2.5 m.
        assert_eq!(s.ranges[2], f32::INFINITY);
        assert_eq!(s.ranges[3], f32::INFINITY);
        assert_eq!(s.valid_beams().count(), 2);
        let book = LidarConfig {
            no_return: NoReturnPolicy::MaxRange,
            ..short
        };
        let mut l = Lidar::new(book, 1).unwrap();
        let s = l.scan(&room(), &Se2::IDENTITY, Timestamp::ZERO);
        assert_eq!(s.ranges[2], 2.5);
        // Empty world: everything is a no-return.
        let mut l = Lidar::new(four_beams(), 1).unwrap();
        let s = l.scan(&World::empty(), &Se2::IDENTITY, Timestamp::ZERO);
        assert!(s.ranges.iter().all(|r| *r == f32::INFINITY));
    }

    #[test]
    fn too_close_returns_are_negative_infinity() {
        let mut w = World::empty();
        w.add_segment(Vector2::new(0.05, -1.0), Vector2::new(0.05, 1.0))
            .unwrap();
        let mut l = Lidar::new(four_beams(), 1).unwrap();
        let s = l.scan(&w, &Se2::IDENTITY, Timestamp::ZERO);
        assert_eq!(s.ranges[2], f32::NEG_INFINITY);
        assert!(!s.is_valid(2));
    }

    #[test]
    fn noise_free_default_scan_is_exact() {
        let world = World::rusty_arena();
        let cfg = LidarConfig::default().noiseless();
        let mut l = Lidar::new(cfg, 1).unwrap();
        let pose = Se2::new(0.7, 1.3, 0.4);
        let s = l.scan(&world, &pose, Timestamp::ZERO);
        let ideal = l.ideal_ranges(&world, &pose);
        assert_eq!(s.ranges.len(), 360);
        assert_relative_eq!(s.angle_increment, PI / 180.0, epsilon = 1e-12);
        for (r, z) in s.ranges.iter().zip(ideal) {
            match z {
                Some(z) => assert_eq!(*r, z as f32),
                None => assert_eq!(*r, f32::INFINITY),
            }
        }
        // Every valid beam endpoint lies on a wall.
        let sensor = l.sensor_pose(&pose);
        for (b, r) in s.valid_beams() {
            let p = sensor.act(&Vector2::new(r * b.cos(), r * b.sin()));
            assert!(world.clearance(&p) < 1e-5);
        }
    }

    #[test]
    fn seeded_noise_is_reproducible_and_calibrated() {
        let world = room();
        let cfg = LidarConfig {
            range_max: 10.0,
            mount: Se2::IDENTITY,
            ..LidarConfig::default()
        };
        let scan = |seed| {
            Lidar::new(cfg.clone(), seed)
                .unwrap()
                .scan(&world, &Se2::IDENTITY, Timestamp::ZERO)
        };
        assert_eq!(scan(5), scan(5));
        assert_ne!(scan(5), scan(6));
        // Residuals against the ideal ranges: zero mean, σ ≈ 0.02, ~1 % dropouts.
        let ideal = Lidar::new(cfg.clone(), 0)
            .unwrap()
            .ideal_ranges(&world, &Se2::IDENTITY);
        let (mut n, mut s1, mut s2, mut drops, mut total) = (0.0, 0.0, 0.0, 0, 0);
        for seed in 0..100 {
            for (r, z) in scan(seed).ranges.iter().zip(&ideal) {
                total += 1;
                if !r.is_finite() {
                    drops += 1;
                    continue;
                }
                let e = *r as f64 - z.unwrap();
                n += 1.0;
                s1 += e;
                s2 += e * e;
            }
        }
        let mean = s1 / n;
        let sd = (s2 / n - mean * mean).sqrt();
        assert!(mean.abs() < 1e-3, "mean {mean}");
        assert!((sd - 0.02).abs() < 1e-3, "sd {sd}");
        let rate = drops as f64 / total as f64;
        assert!((rate - 0.01).abs() < 0.003, "dropout rate {rate}");
    }

    #[test]
    fn dropout_does_not_reshuffle_surviving_noise() {
        let world = room();
        let base = LidarConfig {
            range_max: 10.0,
            ..LidarConfig::default()
        };
        let a = Lidar::new(
            LidarConfig {
                dropout_probability: 0.0,
                ..base.clone()
            },
            3,
        )
        .unwrap()
        .scan(&world, &Se2::IDENTITY, Timestamp::ZERO);
        let b = Lidar::new(
            LidarConfig {
                dropout_probability: 0.3,
                ..base
            },
            3,
        )
        .unwrap()
        .scan(&world, &Se2::IDENTITY, Timestamp::ZERO);
        let mut survivors = 0;
        for (x, y) in a.ranges.iter().zip(&b.ranges) {
            if y.is_finite() {
                assert_eq!(x, y);
                survivors += 1;
            }
        }
        assert!(survivors > 200 && survivors < 300);
    }

    #[test]
    fn invalid_configurations_are_rejected() {
        let d = LidarConfig::default();
        for bad in [
            LidarConfig {
                beams: 0,
                ..d.clone()
            },
            LidarConfig {
                range_max: 0.1,
                ..d.clone()
            },
            LidarConfig {
                range_min: -1.0,
                ..d.clone()
            },
            LidarConfig {
                angle_max: -4.0,
                ..d.clone()
            },
            LidarConfig {
                angle_min: -4.0,
                angle_max: 4.0,
                ..d.clone()
            },
            LidarConfig {
                rate_hz: 0.0,
                ..d.clone()
            },
            LidarConfig {
                range_sigma: -0.1,
                ..d.clone()
            },
            LidarConfig {
                dropout_probability: 1.5,
                ..d.clone()
            },
            LidarConfig {
                frame: FrameId::default(),
                ..d.clone()
            },
        ] {
            assert!(Lidar::new(bad.clone(), 0).is_err(), "{bad:?}");
        }
    }
}
