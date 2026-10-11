//! Scan matching and the LiDAR odometry front end on seeded Rusty scans with exact ground
//! truth. These are tests with their own thresholds, not the claims S1 to S6 (those are graded
//! by `scripts/phase5_verdict.py` on a metrics file); the thresholds say what a correct
//! implementation must do on a model-level scene, each chosen after reading the measured
//! numbers a first run printed and noted in the comment next to it.

mod common;

use common::*;
use nalgebra::Vector2;
use robotics_core::Se2;
use robotics_slam::icp::{register, scan_points, IcpConfig, IcpMetric};
use robotics_slam::odometry::{LidarOdometry, OdometryConfig, TrackingStatus};
use robotics_slam::voxel::{downsample, VoxelMap};

fn pts(scan: &robotics_core::LaserScan) -> Vec<Vector2<f64>> {
    downsample(&scan_points(scan), 0.05).unwrap()
}

fn target_map(points: &[Vector2<f64>]) -> VoxelMap {
    let mut m = VoxelMap::new(0.1, 8).unwrap();
    m.insert_all(points);
    m
}

#[test]
fn point_to_plane_recovers_the_relative_pose_of_two_noiseless_scans() {
    let world = asymmetric_room();
    let mut lid = lidar(1, false);
    let (a, b) = (Se2::new(0.2, 0.1, 0.3), Se2::new(0.45, 0.2, 0.42));
    let (sa, sb) = (
        scan_at(&mut lid, &world, &a, 0),
        scan_at(&mut lid, &world, &b, 1),
    );
    let truth = a.between(&b);
    for metric in [IcpMetric::PointToPlane, IcpMetric::PointToPoint] {
        let cfg = IcpConfig {
            metric,
            threshold: 0.4,
            ..Default::default()
        };
        let r = register(&pts(&sb), &target_map(&pts(&sa)), Se2::IDENTITY, &cfg).unwrap();
        assert!(r.converged, "{metric:?}");
        let e = truth.between(&r.pose).log();
        let tol = if metric == IcpMetric::PointToPlane {
            3e-3
        } else {
            2e-2
        };
        assert!(e.xy().norm() < tol && e.z.abs() < tol, "{metric:?} {e:?}");
    }
}

#[test]
fn point_to_plane_needs_fewer_iterations_than_point_to_point() {
    let world = asymmetric_room();
    let mut lid = lidar(2, false);
    let (a, b) = (Se2::new(0.0, 0.0, 0.0), Se2::new(0.3, 0.1, 0.25));
    let (sa, sb) = (
        scan_at(&mut lid, &world, &a, 0),
        scan_at(&mut lid, &world, &b, 1),
    );
    let iters = |metric| {
        let cfg = IcpConfig {
            metric,
            threshold: 0.5,
            ..Default::default()
        };
        register(&pts(&sb), &target_map(&pts(&sa)), Se2::IDENTITY, &cfg)
            .unwrap()
            .iterations
    };
    let (plane, point) = (
        iters(IcpMetric::PointToPlane),
        iters(IcpMetric::PointToPoint),
    );
    assert!(plane < point, "plane {plane} point {point}");
}

#[test]
fn the_information_matrix_is_full_rank_in_a_room_and_degenerate_in_a_corridor() {
    let cfg = IcpConfig {
        threshold: 0.5,
        ..Default::default()
    };
    let room = asymmetric_room();
    let mut lid = lidar(3, true);
    let (a, b) = (Se2::new(0.0, 0.0, 0.0), Se2::new(0.1, 0.05, 0.05));
    let (sa, sb) = (
        scan_at(&mut lid, &room, &a, 0),
        scan_at(&mut lid, &room, &b, 1),
    );
    let r = register(&pts(&sb), &target_map(&pts(&sa)), Se2::IDENTITY, &cfg).unwrap();
    assert!(!r.degenerate);
    let eig = r.information.symmetric_eigen().eigenvalues;
    assert!(eig.min() > 0.0);

    let hall = corridor();
    let (sa, sb) = (
        scan_at(&mut lid, &hall, &a, 2),
        scan_at(&mut lid, &hall, &b, 3),
    );
    let r = register(&pts(&sb), &target_map(&pts(&sa)), Se2::IDENTITY, &cfg).unwrap();
    assert!(r.degenerate);
}

#[test]
fn registration_is_bit_identical_on_a_second_run() {
    let world = pillar_room();
    let mut lid = lidar(4, true);
    let (a, b) = (Se2::new(0.0, 0.0, 0.0), Se2::new(0.2, 0.0, 0.1));
    let (sa, sb) = (
        scan_at(&mut lid, &world, &a, 0),
        scan_at(&mut lid, &world, &b, 1),
    );
    let cfg = IcpConfig::default();
    let run = || register(&pts(&sb), &target_map(&pts(&sa)), Se2::IDENTITY, &cfg).unwrap();
    let (x, y) = (run(), run());
    assert_eq!(x.pose.x().to_bits(), y.pose.x().to_bits());
    assert_eq!(x.pose.theta().to_bits(), y.pose.theta().to_bits());
    assert_eq!(x.information, y.information);
}

fn run_front_end(world: &robotics_sim::World, route: &[Se2], seed: u64) -> (Vec<Se2>, usize) {
    let mut lid = lidar(seed, true);
    let mut odo = LidarOdometry::new(OdometryConfig::default()).unwrap();
    let origin = route[0];
    let mut est = Vec::new();
    let mut predicted = 0;
    for (k, p) in route.iter().enumerate() {
        let scan = scan_at(&mut lid, world, p, k);
        let r = odo.step(&scan).unwrap();
        if r.status == TrackingStatus::Predicted {
            predicted += 1;
        }
        // The estimate is in the frame of the first sensor pose; bring it to the world.
        est.push(origin.compose(&r.pose));
    }
    (est, predicted)
}

#[test]
fn the_front_end_follows_two_laps_of_the_pillar_room() {
    let world = pillar_room();
    let route = circle_route(1.4, 0.06, 2.0);
    let (est, predicted) = run_front_end(&world, &route, 7);
    let end = est.last().unwrap().distance_to(route.last().unwrap());
    let worst = est
        .iter()
        .zip(&route)
        .map(|(a, b)| a.distance_to(b))
        .fold(0.0, f64::max);
    println!(
        "front end: {} scans, predicted {predicted}, end error {end:.4} m, worst {worst:.4} m",
        route.len()
    );
    assert_eq!(predicted, 0, "a registration was lost");
    assert!(worst < 0.10, "worst error {worst}");
}

#[test]
fn the_front_end_is_deterministic() {
    let world = pillar_room();
    let route = circle_route(1.4, 0.1, 0.6);
    let bits = |seed| {
        run_front_end(&world, &route, seed)
            .0
            .iter()
            .flat_map(|p| [p.x().to_bits(), p.y().to_bits(), p.theta().to_bits()])
            .collect::<Vec<_>>()
    };
    assert_eq!(bits(5), bits(5));
    assert_ne!(bits(5), bits(6));
}
