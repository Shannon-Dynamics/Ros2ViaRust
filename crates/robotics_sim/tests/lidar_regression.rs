//! LiDAR numerical regression in the Phase 2 arena, without ROS.
//!
//! The arena's outer walls are x = −5.5, x = 7.5, y = −4, y = 9; the default LiDAR has 360
//! beams from −π in 1° steps (beam 0 = rear, 90 = right, 180 = front, 270 = left) and is
//! mounted 0.05 m ahead of the base.

use robotics_core::{Se2, Timestamp};
use robotics_sim::{Lidar, LidarConfig, RustyConfig, RustySim, Trajectory, World};
use std::f64::consts::FRAC_PI_2;

fn ideal_scan(pose: Se2) -> Vec<f32> {
    let mut l = Lidar::new(LidarConfig::default().noiseless(), 0).unwrap();
    l.scan(&World::rusty_arena(), &pose, Timestamp::ZERO).ranges
}

#[test]
fn start_pose_wall_distances() {
    let r = ideal_scan(Se2::IDENTITY);
    assert!((r[180] - 7.45).abs() < 1e-5, "front {}", r[180]); // 7.5 − 0.05
    assert!((r[0] - 5.55).abs() < 1e-5, "rear {}", r[0]); // 5.5 + 0.05
    assert!((r[90] - 4.0).abs() < 1e-5, "right {}", r[90]);
    assert_eq!(
        r[270],
        f32::INFINITY,
        "left wall at 9 m is beyond the 8 m range"
    );
}

#[test]
fn rotated_pose_wall_distances() {
    // Facing +y from (1, 2): sensor at (1, 2.05). Front → y = 9 (6.95 m), rear → y = −4
    // (6.05 m), right → x = 7.5 (6.5 m), left → x = −5.5 (6.5 m).
    let r = ideal_scan(Se2::new(1.0, 2.0, FRAC_PI_2));
    for (i, expect) in [(180, 6.95), (0, 6.05), (90, 6.5), (270, 6.5)] {
        assert!(
            (r[i] as f64 - expect).abs() < 1e-5,
            "beam {i}: {} vs {expect}",
            r[i]
        );
    }
}

#[test]
fn simulator_scans_replay_bit_exactly() {
    let run = |seed| {
        let mut sim = RustySim::new(
            RustyConfig {
                seed,
                ..RustyConfig::default()
            },
            Trajectory::mixed(true),
        )
        .unwrap();
        (0..4000)
            .filter_map(|_| sim.step().scan)
            .collect::<Vec<_>>()
    };
    let a = run(7);
    assert_eq!(a.len(), 200, "20 s at 10 Hz");
    assert_eq!(a, run(7));
    assert_ne!(a, run(8));
    // Every scan has the documented structure and plenty of valid returns in the arena
    // (walls farther than 8 m and 1 % dropouts are +inf).
    for s in &a {
        assert!(s.validate().is_ok());
        assert!(s.valid_beams().count() > 150, "{}", s.valid_beams().count());
    }
}
