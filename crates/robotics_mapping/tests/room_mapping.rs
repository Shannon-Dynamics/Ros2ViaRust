//! The mapper end to end on a synthetic copy of the Phase 3 room, with poses given.
//!
//! The world is the room of `sim/webots/config/landmarks.yaml` rebuilt in `robotics_sim`
//! (inner faces at +/- 3 m, four cylinders as 64-gons), the lidar mimics the Webots LDS-01 as
//! measured in `evidence/phase-4-mapping/2026-10-10-scan-skew-probe` (360 beams over the full
//! circle, 3.5 m, `inf` for no return), and the robot drives two loops. No Webots, no ROS and
//! no estimator: this checks that the mapper is *implemented* correctly (claim M3 and the
//! library-level part of M1) and that it is deterministic (claim M2). It says nothing about
//! the real simulator; that is what the evidence runs are for.

use robotics_core::nalgebra::Vector2;
use robotics_core::{FrameId, Se2, Timestamp};
use robotics_mapping::{
    evaluate, grid_entropy, signed_esdf, CellState, EvalParams, GridGeometry, GroundTruth,
    NoReturn, OccupancyGrid, OccupancyParams, RayTracer, ScanIntegrator,
};
use robotics_sim::lidar::{Lidar, LidarConfig, NoReturnPolicy};
use robotics_sim::World;
use std::f64::consts::{PI, TAU};

const LANDMARKS: &str = include_str!("../../../sim/webots/config/landmarks.yaml");

fn room_world(truth: &[(f64, f64, f64)]) -> World {
    let mut w = World::empty();
    w.add_rectangle(Vector2::new(-3.0, -3.0), Vector2::new(3.0, 3.0))
        .unwrap();
    for &(x, y, r) in truth {
        let pts: Vec<_> = (0..64)
            .map(|k| {
                let a = TAU * k as f64 / 64.0;
                Vector2::new(x + r * a.cos(), y + r * a.sin())
            })
            .collect();
        w.add_polyline(&pts, true).unwrap();
    }
    w
}

fn lidar(seed: u64) -> Lidar {
    let cfg = LidarConfig {
        frame: FrameId::new("LDS-01"),
        mount: Se2::new(-0.03, 0.0, 0.0),
        angle_min: -PI,
        angle_max: PI,
        beams: 360,
        range_min: 0.12,
        range_max: 3.5,
        rate_hz: 5.0,
        range_sigma: 0.007,
        dropout_probability: 0.0,
        no_return: NoReturnPolicy::Infinity,
    };
    Lidar::new(cfg, seed).unwrap()
}

/// Two loops, radius 1.0 m and 1.7 m, at 0.18 m/s with a scan every 0.2 s, heading along the
/// loop. Bodies only; the sensor pose is the body composed with the mount.
fn bodies() -> Vec<Se2> {
    let mut out = Vec::new();
    for radius in [1.0, 1.7] {
        let steps = (TAU * radius / (0.18 * 0.2)).round() as usize;
        for k in 0..steps {
            let a = TAU * k as f64 / steps as f64;
            out.push(Se2::new(
                radius * a.cos(),
                radius * a.sin(),
                a + std::f64::consts::FRAC_PI_2,
            ));
        }
    }
    out
}

fn geometry() -> GridGeometry {
    GridGeometry {
        width: 140,
        height: 140,
        resolution: 0.05,
        origin: Vector2::new(-3.5, -3.5),
    }
}

/// Run the mapper over the loops with the given scan noise seed.
fn map_room(seed: u64, tracer: RayTracer) -> (OccupancyGrid, Vec<Vector2<f64>>) {
    let truth = GroundTruth::from_landmarks_yaml(LANDMARKS).unwrap();
    assert_eq!(truth.pillar_count(), 4);
    let pillars: Vec<(f64, f64, f64)> = vec![
        (1.8, 1.2, 0.15),
        (-1.4, 2.0, 0.15),
        (-2.0, -1.0, 0.15),
        (0.9, -2.1, 0.15),
    ];
    let world = room_world(&pillars);
    let mut l = lidar(seed);
    let mut grid = OccupancyGrid::new(geometry(), OccupancyParams::default()).unwrap();
    let mut integ = ScanIntegrator::new(NoReturn::ClearToMaxRange, tracer);
    let mut positions = Vec::new();
    for (k, body) in bodies().iter().enumerate() {
        let scan = l.scan(&world, body, Timestamp::from_secs_f64(0.2 * k as f64));
        let sensor = l.sensor_pose(body);
        integ.integrate(&mut grid, &sensor, &scan).unwrap();
        positions.push(Vector2::new(sensor.x(), sensor.y()));
    }
    (grid, positions)
}

fn eval_params(tau_cells: f64) -> EvalParams {
    EvalParams {
        tau: tau_cells * 0.05,
        sample_spacing: 0.01,
        max_range: 3.5,
        max_incidence: robotics_mapping::DEFAULT_MAX_INCIDENCE_DEG.to_radians(),
        coverage_stride: 10,
    }
}

/// M2, the blocking part: the same scans and poses, integrated twice, give grids that are
/// identical bit for bit, and a grid survives its own text form unchanged.
#[test]
fn the_mapper_is_deterministic_and_its_text_form_loses_nothing() {
    let (a, _) = map_room(1001, RayTracer::Supercover);
    let (b, _) = map_room(1001, RayTracer::Supercover);
    assert!(a.bit_identical(&b));
    let back = OccupancyGrid::from_text(&a.to_text()).unwrap();
    assert!(a.bit_identical(&back));
    // A different noise seed is a different measurement, so a different grid: the check above
    // is not vacuous.
    let (c, _) = map_room(1002, RayTracer::Supercover);
    assert!(!a.bit_identical(&c));
}

/// The library-level part of M1: with exact poses the map of an easy room is accurate. The
/// numbers asserted are those a correct mapper must meet on this world by construction (the
/// scan noise is 7 mm and a cell is 50 mm), set before the first run and not adjusted to it.
#[test]
fn with_exact_poses_the_room_is_mapped_accurately() {
    let truth = GroundTruth::from_landmarks_yaml(LANDMARKS).unwrap();
    let (grid, positions) = map_room(1001, RayTracer::Supercover);
    let m = evaluate(&grid, &truth, &positions, &eval_params(1.0)).unwrap();
    println!("{}", m.to_yaml());
    let e = m.surface_error.expect("some cells are occupied");
    // Every occupied cell is within one cell of a surface, and on average within half a cell.
    assert!(e.max_m <= 0.05 + 1e-9, "max surface error {}", e.max_m);
    assert!(e.mean_m <= 0.025, "mean surface error {}", e.mean_m);
    assert_eq!(m.false_occupied_cells, 0);
    // Nothing the robot saw on a flat wall is called free. What is left is the silhouette of
    // the pillars: a cell the edge of a 0.15 m cylinder passes through is mostly free space
    // that many grazing beams cross, and a mapper that updates by beam cannot call it
    // occupied. That is a discretization effect, bounded by the pillars' share of the
    // surface samples (about 14 per cent), and the bound asserted is five per cent of them all.
    let walls_only = GroundTruth::room(3.0, 3.0, &[]).unwrap();
    let w = evaluate(&grid, &walls_only, &positions, &eval_params(1.0)).unwrap();
    assert_eq!(
        w.false_free_samples, 0,
        "a flat wall sample was called free"
    );
    assert!(
        m.false_free_fraction.unwrap() <= 0.05,
        "false free {:?}",
        m.false_free_fraction
    );
    // And almost everything it saw is on the map.
    assert!(
        m.completeness.unwrap() >= 0.95,
        "completeness {:?}",
        m.completeness
    );
    assert!(m.coverage.unwrap() >= 0.9, "coverage {:?}", m.coverage);
    assert!(
        m.observable_fraction > 0.8,
        "observable {}",
        m.observable_fraction
    );
    assert_eq!(m.samples_outside_grid, 0);
}

/// The inverse model never writes behind a wall: a cell outside the room, which no beam can
/// reach, stays unknown.
#[test]
fn nothing_is_written_outside_the_walls() {
    let (grid, _) = map_room(1001, RayTracer::Supercover);
    for j in 0..140 {
        for i in 0..140 {
            let c = grid.cell_to_world(i, j);
            if c.x.abs() > 3.06 || c.y.abs() > 3.06 {
                assert_eq!(
                    grid.state(i, j),
                    Some(CellState::Unknown),
                    "cell ({i}, {j})"
                );
            }
        }
    }
}

/// Entropy falls as the room is explored, and the decided cells account for the fall.
#[test]
fn entropy_falls_with_exploration() {
    let truth_poses = bodies();
    let pillars = [
        (1.8, 1.2, 0.15),
        (-1.4, 2.0, 0.15),
        (-2.0, -1.0, 0.15),
        (0.9, -2.1, 0.15),
    ];
    let world = room_world(&pillars);
    let mut l = lidar(7);
    let mut grid = OccupancyGrid::new(geometry(), OccupancyParams::default()).unwrap();
    let mut integ = ScanIntegrator::default();
    let fresh = grid_entropy(&grid);
    assert_eq!(fresh.total_bits, 19_600.0);
    let mut last = fresh.total_bits;
    for (k, body) in truth_poses.iter().enumerate().take(60) {
        let scan = l.scan(&world, body, Timestamp::from_secs_f64(0.2 * k as f64));
        integ
            .integrate(&mut grid, &l.sensor_pose(body), &scan)
            .unwrap();
        let e = grid_entropy(&grid).total_bits;
        assert!(e <= last + 1e-6, "entropy rose at scan {k}: {last} -> {e}");
        last = e;
    }
    assert!(last < fresh.total_bits * 0.9);
}

/// The ESDF of the finished map: zero level at the walls, positive across the room, negative
/// in the wall cells, and agreeing with the analytic distance to the nearest surface in the
/// middle of the room to within a cell.
#[test]
fn the_esdf_of_the_room_agrees_with_the_analytic_distance() {
    let truth = GroundTruth::from_landmarks_yaml(LANDMARKS).unwrap();
    let (grid, _) = map_room(1001, RayTracer::Supercover);
    let esdf = signed_esdf(&grid, 5.0).unwrap();
    // The field is a field over cell centres, and the zero level sits on the faces of the
    // occupied cells, which contain the hits and so reach up to one cell towards the sensor
    // from the true surface. So at a cell centre it may read less than the analytic distance
    // by at most one cell, and more by at most the noise-sized overshoot of a half cell.
    let p = grid.cell_to_world(70, 70);
    let want = truth.distance_to_surface(&p);
    let got = esdf.at(&p).unwrap() as f64;
    assert!(
        got - want <= 0.025 && want - got <= 0.05,
        "esdf {got} vs analytic {want}"
    );
    // Half a metre from the east wall the field reads about half a metre, one cell less at
    // most: the wall lies on a lattice line, so its hits split between two cells and it is
    // mapped two cells thick, which moves the zero level a cell toward the sensor.
    let c = grid.cell_to_world(
        grid.world_to_cell(&Vector2::new(2.5, 0.0)).unwrap().0,
        grid.world_to_cell(&Vector2::new(2.5, 0.0)).unwrap().1,
    );
    let want = 3.0 - c.x;
    let near = esdf.at(&c).unwrap() as f64;
    assert!(
        near <= want + 1e-6 && want - near <= 0.05 + 1e-6,
        "esdf {near} vs {want}"
    );
    let lowest = (0..=40)
        .map(|k| esdf.at(&Vector2::new(2.9 + 0.005 * k as f64, 0.0)).unwrap())
        .fold(f32::INFINITY, f32::min);
    assert!(
        lowest < 0.0,
        "no negative cell across the east wall, lowest {lowest}"
    );
}
