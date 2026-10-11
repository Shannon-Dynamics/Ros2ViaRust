//! Fixtures shared by the integration tests: worlds, a lidar shaped like the Webots LDS-01
//! (360 beams, 0.12 to 3.5 m, about 15 mm of range noise, no dropout), and routes.
#![allow(dead_code)]

use nalgebra::Vector2;
use robotics_core::{LaserScan, Se2, Timestamp};
use robotics_sim::{Lidar, LidarConfig, World};

fn v(x: f64, y: f64) -> Vector2<f64> {
    Vector2::new(x, y)
}

/// The Phase 3 and 4 room: walls at ±3 m and four 12-sided pillars of radius 0.15 m.
pub fn pillar_room() -> World {
    let mut w = square_room();
    for (cx, cy) in [(1.8, 1.2), (-1.4, 2.0), (-2.0, -1.0), (0.9, -2.1)] {
        let ring: Vec<_> = (0..12)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 12.0;
                v(cx + 0.15 * a.cos(), cy + 0.15 * a.sin())
            })
            .collect();
        w.add_polyline(&ring, true).unwrap();
    }
    w
}

/// An empty square room, fourfold symmetric about its centre.
pub fn square_room() -> World {
    let mut w = World::empty();
    w.add_rectangle(v(-3.0, -3.0), v(3.0, 3.0)).unwrap();
    w
}

/// A room with no symmetry: unequal sides and two boxes at awkward angles.
pub fn asymmetric_room() -> World {
    let mut w = World::empty();
    w.add_rectangle(v(-3.0, -2.5), v(3.2, 2.5)).unwrap();
    w.add_box(v(1.0, 1.0), v(0.4, 0.25), 0.6).unwrap();
    w.add_box(v(-1.5, -0.8), v(0.2, 0.5), -0.3).unwrap();
    w
}

/// A long hall: two parallel walls 2 m apart, 40 m long, closed at the ends.
pub fn corridor() -> World {
    let mut w = World::empty();
    w.add_rectangle(v(-20.0, -1.0), v(20.0, 1.0)).unwrap();
    w
}

/// A lidar like the Webots one.
pub fn lidar(seed: u64, noisy: bool) -> Lidar {
    let mut cfg = LidarConfig {
        mount: Se2::IDENTITY,
        range_min: 0.12,
        range_max: 3.5,
        range_sigma: 0.015,
        dropout_probability: 0.0,
        rate_hz: 5.0,
        ..LidarConfig::default()
    };
    if !noisy {
        cfg = cfg.noiseless();
    }
    Lidar::new(cfg, seed).unwrap()
}

/// One scan from `pose` (the sensor is at the pose: the mount is the identity).
pub fn scan_at(lidar: &mut Lidar, world: &World, pose: &Se2, k: usize) -> LaserScan {
    lidar.scan(world, pose, Timestamp::from_secs_f64(0.2 * k as f64))
}

/// Poses along a circle of `radius` about the origin, heading along the tangent, one every
/// `step` metres of arc, for `laps` laps. The first pose is at angle 0 and the last is one step
/// short of closing the final lap.
pub fn circle_route(radius: f64, step: f64, laps: f64) -> Vec<Se2> {
    let n = (std::f64::consts::TAU * radius * laps / step).round() as usize;
    (0..n)
        .map(|k| {
            let a = k as f64 * step / radius;
            Se2::new(
                radius * a.cos(),
                radius * a.sin(),
                a + std::f64::consts::FRAC_PI_2,
            )
        })
        .collect()
}
