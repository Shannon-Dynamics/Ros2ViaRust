//! Integration tests of the 2-D NDT registration on synthetic lidar scans.
//!
//! A target scan is taken from the body pose `A`, a source scan from `B = A ⊞ (0.15, 0.10, 0.1)`
//! in an asymmetric arena. The map is built from the target points in `A`'s sensor frame, the
//! source points are in `B`'s sensor frame, and the expected registration is
//! `T = S_A⁻¹ · S_B` for the sensor poses `S = body · mount`, so that `q = T·p` maps a source
//! point into the map frame.

use nalgebra::{SymmetricEigen, Vector2, Vector3};
use robotics_core::{Manifold, Se2, Timestamp};
use robotics_sim::{Lidar, LidarConfig, World};
use robotics_slam::ndt::{register, NdtConfig, NdtMap};
use robotics_slam::SlamError;

/// Sensor-frame endpoints of the valid beams of one sweep from `body`.
fn scan_points(world: &World, lidar: &mut Lidar, body: &Se2) -> Vec<Vector2<f64>> {
    let scan = lidar.scan(world, body, Timestamp::ZERO);
    scan.valid_beams()
        .map(|(b, r)| Vector2::new(r * b.cos(), r * b.sin()))
        .collect()
}

/// Target points, source points and the true registration for the body poses `a` and
/// `a ⊞ rel`.
struct Pair {
    target: Vec<Vector2<f64>>,
    source: Vec<Vector2<f64>>,
    truth: Se2,
}

fn make_pair(world: &World, lidar: &mut Lidar, a: Se2, rel: Vector3<f64>) -> Pair {
    let b = a.boxplus(&rel);
    let target = scan_points(world, lidar, &a);
    let source = scan_points(world, lidar, &b);
    let truth = lidar.sensor_pose(&a).between(&lidar.sensor_pose(&b));
    Pair {
        target,
        source,
        truth,
    }
}

fn rel() -> Vector3<f64> {
    Vector3::new(0.15, 0.10, 0.1)
}

fn noiseless_lidar() -> Lidar {
    Lidar::new(LidarConfig::default().noiseless(), 1).unwrap()
}

/// The asymmetric arena: the Phase 2 room with its obstacles and a skewed box added.
fn arena() -> World {
    let mut w = World::rusty_arena();
    w.add_box(Vector2::new(2.5, 3.0), Vector2::new(0.6, 0.3), 0.4)
        .unwrap();
    w
}

/// Pose error `(translation m, rotation rad)` of `pose` against `truth`.
fn error(pose: &Se2, truth: &Se2) -> (f64, f64) {
    let e = pose.boxminus(truth);
    (e.xy().norm(), e[2].abs())
}

fn standard_pair() -> Pair {
    make_pair(
        &arena(),
        &mut noiseless_lidar(),
        Se2::new(1.0, 0.5, 0.3),
        rel(),
    )
}

#[test]
fn gradient_matches_central_differences_on_a_room_scan() {
    let pair = standard_pair();
    let cfg = NdtConfig::default();
    let map = NdtMap::from_points(&pair.target, &cfg).unwrap();
    // Off the optimum, with a nonzero rotation error.
    let pose = pair.truth.boxplus(&Vector3::new(0.031, -0.023, 0.037));
    let ev = map.evaluate(&pair.source, &pose);
    assert!(ev.matched_points > 100);
    let h = 1e-6;
    for k in 0..3 {
        let mut d = Vector3::zeros();
        d[k] = h;
        let hi = map.evaluate(&pair.source, &pose.boxplus(&d)).score;
        let lo = map.evaluate(&pair.source, &pose.boxplus(&-d)).score;
        let num = (hi - lo) / (2.0 * h);
        let scale = num.abs().max(ev.gradient.norm()).max(1.0);
        assert!(
            (ev.gradient[k] - num).abs() < 1e-5 * scale,
            "k = {k}: {} vs {num}",
            ev.gradient[k]
        );
    }
}

#[test]
fn hessian_is_symmetric_and_positive_semidefinite() {
    let pair = standard_pair();
    let map = NdtMap::from_points(&pair.target, &NdtConfig::default()).unwrap();
    for off in [
        Vector3::new(0.031, -0.023, 0.037),
        Vector3::new(-0.06, 0.04, -0.08),
        Vector3::zeros(),
    ] {
        let ev = map.evaluate(&pair.source, &pair.truth.boxplus(&off));
        assert_eq!(ev.hessian, ev.hessian.transpose());
        let min = SymmetricEigen::new(ev.hessian).eigenvalues.min();
        assert!(min >= -1e-9, "smallest eigenvalue {min}");
    }
}

#[test]
fn registration_recovers_the_relative_pose_without_noise() {
    let pair = standard_pair();
    let cfg = NdtConfig::default();
    let map = NdtMap::from_points(&pair.target, &cfg).unwrap();
    assert!(map.cell_count() >= 10);
    // Identity-ish starts: from a few centimeters and a few hundredths of a radian off.
    for off in [
        Vector3::new(0.05, -0.04, 0.03),
        Vector3::new(-0.06, 0.05, -0.04),
        Vector3::new(0.0, 0.0, 0.05),
    ] {
        let r = register(&pair.source, &map, pair.truth.boxplus(&off), &cfg).unwrap();
        let (et, er) = error(&r.pose, &pair.truth);
        assert!(r.converged, "{r:?}");
        assert!(et < 5e-3 && er < 3e-3, "{off:?}: {et} m, {er} rad");
        assert!(r.iterations < cfg.max_iterations);
    }
}

#[test]
fn registration_from_the_identity_start() {
    // The identity is 0.18 m and 0.1 rad from the truth, inside the basin measured below (in this
    // scene; the basin is direction- and scene-dependent).
    let pair = standard_pair();
    let cfg = NdtConfig::default();
    let map = NdtMap::from_points(&pair.target, &cfg).unwrap();
    let r = register(&pair.source, &map, Se2::IDENTITY, &cfg).unwrap();
    let (et, er) = error(&r.pose, &pair.truth);
    let (et0, er0) = error(&Se2::IDENTITY, &pair.truth);
    assert!(
        et0 > 0.15 && er0 > 0.09,
        "the start is not the truth: {et0} {er0}"
    );
    assert!(
        et < 5e-3 && er < 3e-3,
        "identity start ended {et} m, {er} rad"
    );
}

#[test]
fn registration_with_the_seeded_noisy_lidar() {
    // Default lidar noise (σ = 2 cm, 1 % dropout), seeds 1 to 5 for the one noise stream that
    // draws the target and then the source sweep.
    let world = arena();
    let cfg = NdtConfig::default();
    for seed in 1..=5u64 {
        let mut lidar = Lidar::new(LidarConfig::default(), seed).unwrap();
        let pair = make_pair(&world, &mut lidar, Se2::new(1.0, 0.5, 0.3), rel());
        let map = NdtMap::from_points(&pair.target, &cfg).unwrap();
        let init = pair.truth.boxplus(&Vector3::new(0.06, -0.05, 0.04));
        let r = register(&pair.source, &map, init, &cfg).unwrap();
        let (et, er) = error(&r.pose, &pair.truth);
        assert!(et < 1.5e-2 && er < 1e-2, "seed {seed}: {et} m, {er} rad");
    }
}

#[test]
fn convergence_basin_is_what_was_measured() {
    // Measured with the noiseless lidar and the default config, in 8 directions each. Scene 1
    // (body pose A = (1, 0.5, 0.3)): every start up to 0.30 m of translation error, with or
    // without a 0.2 rad rotation error of either sign, reaches the truth; 0.4 m fails in two of
    // eight directions and 0.4 rad of rotation fails. Scene 2 (A = (-2, 6, 2)) is narrower:
    // translation errors up to 0.20 m and rotation errors up to 0.20 rad succeed, 0.30 m with
    // 0.20 rad succeeds in 11 of 16 starts. The basin depends on how much structure is near;
    // only the measured ranges are asserted. (In the bare `rusty_arena` without the extra box
    // the same test succeeds only up to 0.10 m in every direction.)
    let world = arena();
    let cfg = NdtConfig::default();
    let mut lidar = noiseless_lidar();
    let cases = [
        (Se2::new(1.0, 0.5, 0.3), 0.3, 0.2),
        (Se2::new(-2.0, 6.0, 2.0), 0.2, 0.2),
    ];
    for (a, max_trans, max_rot) in cases {
        let pair = make_pair(&world, &mut lidar, a, rel());
        let map = NdtMap::from_points(&pair.target, &cfg).unwrap();
        let t = pair.truth;
        let reaches = |init: Se2| {
            let r = register(&pair.source, &map, init, &cfg).unwrap();
            let (et, er) = error(&r.pose, &t);
            et < 5e-3 && er < 3e-3
        };
        for k in 0..8 {
            let an = k as f64 * std::f64::consts::FRAC_PI_4;
            let (dx, dy) = (max_trans * an.cos(), max_trans * an.sin());
            assert!(
                reaches(Se2::new(t.x() + dx, t.y() + dy, t.theta())),
                "{a:?} {max_trans} m, dir {k}"
            );
            // Rotation together with the large translation only where it was measured to work.
            for sign in [1.0, -1.0] {
                let (dx, dy) = (0.1 * an.cos(), 0.1 * an.sin());
                assert!(
                    reaches(Se2::new(t.x() + dx, t.y() + dy, t.theta() + sign * 0.1)),
                    "{a:?} dir {k}"
                );
                if max_trans > 0.25 {
                    let init = Se2::new(
                        t.x() + max_trans * an.cos(),
                        t.y() + max_trans * an.sin(),
                        t.theta() + sign * max_rot,
                    );
                    assert!(
                        reaches(init),
                        "{a:?} {max_trans} m and {} rad, dir {k}",
                        sign * max_rot
                    );
                }
            }
        }
        for sign in [1.0, -1.0] {
            assert!(
                reaches(Se2::new(t.x(), t.y(), t.theta() + sign * max_rot)),
                "{a:?} rotation"
            );
        }
    }
}

#[test]
fn final_score_beats_the_start_and_matches_the_true_pose() {
    let pair = standard_pair();
    let cfg = NdtConfig::default();
    let map = NdtMap::from_points(&pair.target, &cfg).unwrap();
    let init = pair.truth.boxplus(&Vector3::new(0.08, -0.06, 0.05));
    let s0 = map.evaluate(&pair.source, &init).score;
    let r = register(&pair.source, &map, init, &cfg).unwrap();
    let s_true = map.evaluate(&pair.source, &pair.truth).score;
    assert!(s0 < 0.0 && r.score < 0.0);
    assert!(r.score < s0, "{} vs initial {s0}", r.score);
    assert!(
        r.score <= s_true + 1e-6 * s_true.abs(),
        "{} vs true {s_true}",
        r.score
    );
    assert_eq!(r.score, map.evaluate(&pair.source, &r.pose).score);
    assert_eq!(
        r.matched_points,
        map.evaluate(&pair.source, &r.pose).matched_points
    );
}

#[test]
fn registration_is_bit_reproducible() {
    let pair = standard_pair();
    let cfg = NdtConfig::default();
    let init = pair.truth.boxplus(&Vector3::new(0.08, -0.06, 0.05));
    let run = || {
        let map = NdtMap::from_points(&pair.target, &cfg).unwrap();
        register(&pair.source, &map, init, &cfg).unwrap()
    };
    let (a, b) = (run(), run());
    assert_eq!(a.pose.x().to_bits(), b.pose.x().to_bits());
    assert_eq!(a.pose.y().to_bits(), b.pose.y().to_bits());
    assert_eq!(a.pose.theta().to_bits(), b.pose.theta().to_bits());
    assert_eq!(a.score.to_bits(), b.score.to_bits());
    assert_eq!(a.iterations, b.iterations);
}

#[test]
fn bad_inputs_return_the_matching_errors() {
    let pair = standard_pair();
    let cfg = NdtConfig::default();
    let map = NdtMap::from_points(&pair.target, &cfg).unwrap();
    assert!(matches!(
        register(&[], &map, Se2::IDENTITY, &cfg),
        Err(SlamError::EmptyInput(_))
    ));
    assert!(matches!(
        NdtMap::from_points(&[], &cfg),
        Err(SlamError::EmptyInput(_))
    ));
    // Points spread too thin for any cell to reach min_points.
    let sparse: Vec<_> = (0..50).map(|k| Vector2::new(k as f64, 0.0)).collect();
    assert!(matches!(
        NdtMap::from_points(&sparse, &cfg),
        Err(SlamError::EmptyInput(_))
    ));
    let mut nan = pair.source.clone();
    nan[3].y = f64::NAN;
    assert!(matches!(
        register(&nan, &map, Se2::IDENTITY, &cfg),
        Err(SlamError::NonFinite(_))
    ));
    nan = pair.target.clone();
    nan[0].x = f64::INFINITY;
    assert!(matches!(
        NdtMap::from_points(&nan, &cfg),
        Err(SlamError::NonFinite(_))
    ));
    for (bad, what) in [
        (
            NdtConfig {
                cell: 0.0,
                ..cfg.clone()
            },
            "ndt cell",
        ),
        (
            NdtConfig {
                cell: -1.0,
                ..cfg.clone()
            },
            "ndt cell",
        ),
        (
            NdtConfig {
                min_points: 2,
                ..cfg.clone()
            },
            "ndt min_points",
        ),
        (
            NdtConfig {
                eig_ratio_floor: 0.0,
                ..cfg.clone()
            },
            "ndt eig_ratio_floor",
        ),
        (
            NdtConfig {
                eig_ratio_floor: 1.01,
                ..cfg.clone()
            },
            "ndt eig_ratio_floor",
        ),
        (
            NdtConfig {
                outlier_ratio: 0.0,
                ..cfg.clone()
            },
            "ndt outlier_ratio",
        ),
        (
            NdtConfig {
                outlier_ratio: 1.0,
                ..cfg.clone()
            },
            "ndt outlier_ratio",
        ),
    ] {
        let want =
            |e: SlamError| matches!(e, SlamError::InvalidParameter { what: w, .. } if w == what);
        assert!(want(bad.validate().unwrap_err()));
        assert!(want(NdtMap::from_points(&pair.target, &bad).unwrap_err()));
        assert!(want(
            register(&pair.source, &map, Se2::IDENTITY, &bad).unwrap_err()
        ));
    }
}

#[test]
fn a_corridor_does_not_constrain_the_translation_along_it() {
    // Two parallel walls at y = ±1 and nothing else in range. Translation along x leaves every
    // point on the same walls, so the Hessian has a direction with (almost) no curvature. This
    // documents the information structure; it does not claim a corridor registration works.
    let mut world = World::empty();
    world
        .add_segment(Vector2::new(-30.0, 1.0), Vector2::new(30.0, 1.0))
        .unwrap();
    world
        .add_segment(Vector2::new(-30.0, -1.0), Vector2::new(30.0, -1.0))
        .unwrap();
    let cfg = LidarConfig {
        beams: 720,
        mount: Se2::IDENTITY,
        ..LidarConfig::default()
    };
    let mut lidar = Lidar::new(cfg.noiseless(), 1).unwrap();
    let pair = make_pair(
        &world,
        &mut lidar,
        Se2::new(0.0, 0.0, 0.0),
        Vector3::new(0.3, 0.0, 0.0),
    );
    let map = NdtMap::from_points(&pair.target, &NdtConfig::default()).unwrap();
    let ev = map.evaluate(&pair.source, &pair.truth);
    let eig = SymmetricEigen::new(ev.hessian);
    let (lo, hi) = (eig.eigenvalues.min(), eig.eigenvalues.max());
    assert!(lo >= -1e-9, "{lo}");
    assert!(lo < 1e-2 * hi, "smallest {lo}, largest {hi}");
    // The weak direction is the along-corridor translation.
    let k = eig.eigenvalues.imin();
    let weak: Vector3<f64> = eig.eigenvectors.column(k).into();
    assert!(weak[0].abs() > 0.99, "weak direction {weak:?}");
    // The arena, in contrast, constrains every direction.
    let room = standard_pair();
    let room_map = NdtMap::from_points(&room.target, &NdtConfig::default()).unwrap();
    let room_eig = SymmetricEigen::new(room_map.evaluate(&room.source, &room.truth).hessian);
    let room_ratio = room_eig.eigenvalues.min() / room_eig.eigenvalues.max();
    assert!(
        room_ratio > 5.0 * (lo / hi),
        "arena {room_ratio}, corridor {}",
        lo / hi
    );
}
