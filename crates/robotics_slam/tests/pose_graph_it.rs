//! The pose graph and its loop-closure pipeline on seeded Rusty scans with exact ground truth.
//!
//! Two kinds of test. The *drift* test drives two laps, hands the graph odometry edges that
//! carry a known systematic yaw error (the error is injected into the edges, not into the
//! sensor, and the edge information is set to what that error warrants), and checks that
//! verified loop closures reduce the trajectory error and that none of the accepted ones is
//! false. The *rejection* tests each build one candidate designed to be refused for one reason
//! and check that it is refused for that reason, next to a true loop that must be accepted.
//! These are tests with their own thresholds; claim S5 is graded on a metrics file.

mod common;

use common::*;
use nalgebra::{Matrix3, Vector3};
use robotics_core::{LaserScan, Manifold, Se2};
use robotics_sim::World;
use robotics_slam::icp::scan_points;
use robotics_slam::odometry::{EdgeAccumulator, LidarOdometry, OdometryConfig};
use robotics_slam::pose_graph::{LoopCandidate, PoseGraph, PoseGraphConfig, Reason, Verification};
use robotics_slam::voxel::downsample;
use robotics_slam::SlamError;

fn points(scan: &LaserScan) -> Vec<nalgebra::Vector2<f64>> {
    downsample(&scan_points(scan), 0.05).unwrap()
}

fn info(sx: f64, sth: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(
        1.0 / (sx * sx),
        1.0 / (sx * sx),
        1.0 / (sth * sth),
    ))
}

struct Drive {
    graph: PoseGraph,
    /// Ground-truth sensor pose of each keyframe in the frame of the first one.
    truth: Vec<Se2>,
    /// Dead-reckoned keyframe poses (the graph before any loop edge).
    dead: Vec<Se2>,
    /// Every loop outcome seen, with the keyframe pair.
    outcomes: Vec<robotics_slam::pose_graph::LoopOutcome>,
}

fn drive(laps: f64, bias_per_keyframe: f64, seed: u64, with_loops: bool) -> Drive {
    let world = pillar_room();
    let route = circle_route(1.4, 0.06, laps);
    let mut lid = lidar(seed, true);
    let mut odo = LidarOdometry::new(OdometryConfig::default()).unwrap();
    let mut graph = PoseGraph::new(PoseGraphConfig::default()).unwrap();
    let mut acc = EdgeAccumulator::new();
    let origin = route[0];
    let mut truth = Vec::new();
    let mut outcomes = Vec::new();
    for (k, p) in route.iter().enumerate() {
        let scan = scan_at(&mut lid, &world, p, k);
        let step = odo.step(&scan).unwrap();
        if k == 0 {
            graph
                .add_keyframe(points(&scan), &Se2::IDENTITY, &info(0.03, 0.01))
                .unwrap();
            truth.push(origin.between(p));
            continue;
        }
        acc.push(&step.delta, &step.information).unwrap();
        if graph.wants_keyframe(&acc.delta()) {
            let (delta, _chain_info) = acc.take().unwrap();
            // The injected error, and the information it warrants (the ICP chain information
            // is far tighter than a systematic bias allows: that overconfidence is claim S2).
            let biased = delta.boxplus(&Vector3::new(0.0, 0.0, bias_per_keyframe));
            let id = graph
                .add_keyframe(points(&scan), &biased, &info(0.03, 0.01))
                .unwrap();
            truth.push(origin.between(p));
            if with_loops {
                outcomes.extend(graph.process_loops(id).unwrap());
            }
        }
    }
    let dead = {
        let mut v = vec![Se2::IDENTITY];
        // Re-derive dead reckoning from the odometry edges only.
        for f in graph.factor_graph().factors().iter().skip(1) {
            if let robotics_slam::factor_graph::FactorKind::Between {
                from,
                to,
                measurement,
            } = f.kind()
            {
                if *to == *from + 1 {
                    let next = v[*from].compose(measurement);
                    if v.len() == *to {
                        v.push(next);
                    }
                }
            }
        }
        v
    };
    Drive {
        graph,
        truth,
        dead,
        outcomes,
    }
}

fn ate(poses: &[Se2], truth: &[Se2]) -> f64 {
    let s: f64 = poses
        .iter()
        .zip(truth)
        .map(|(a, b)| a.distance_to(b).powi(2))
        .sum();
    (s / truth.len() as f64).sqrt()
}

#[test]
fn verified_loop_closures_reduce_the_error_and_none_is_false() {
    let d = drive(2.0, 0.004, 21, true);
    let accepted: Vec<_> = d
        .outcomes
        .iter()
        .filter(|o| matches!(o.verification, Verification::Accepted { .. }))
        .collect();
    let (before, after) = (ate(&d.dead, &d.truth), ate(d.graph.poses(), &d.truth));
    let rejected_reasons: Vec<_> = d
        .outcomes
        .iter()
        .filter_map(|o| match o.verification {
            Verification::Rejected { reason, .. } => Some(reason),
            _ => None,
        })
        .collect();
    println!(
        "keyframes {}, candidates {}, accepted {}, ATE before {before:.4} m after {after:.4} m, rejected {rejected_reasons:?}",
        d.graph.keyframes().len(),
        d.outcomes.len(),
        accepted.len()
    );
    assert!(!accepted.is_empty(), "no loop was accepted");
    // The harm of a false accept is a wrong constraint, so the label is the error of the
    // accepted measurement against the ground-truth relative pose, not how far apart the
    // keyframes were.
    for o in &accepted {
        let Verification::Accepted { measurement, .. } = o.verification else {
            unreachable!()
        };
        let gt = d.truth[o.candidate.from].between(&d.truth[o.candidate.to]);
        let e = gt.between(&measurement).log();
        assert!(
            e.xy().norm() < 0.05 && e.z.abs() < 0.03,
            "wrong constraint {e:?}"
        );
    }
    assert!(after < 0.6 * before, "ATE {before} -> {after}");
}

#[test]
fn without_loops_the_graph_is_the_odometry_chain() {
    let d = drive(1.0, 0.004, 22, false);
    for (a, b) in d.graph.poses().iter().zip(&d.dead) {
        assert!(a.between(b).log().norm() < 1e-12);
    }
    assert_eq!(d.graph.loop_edge_count(), 0);
}

/// A two-keyframe graph: scan `sa` at `pose_a` in `world_a`, scan `sb` at `pose_b` in
/// `world_b`, joined by an odometry edge that says `pose_b` is `believed` from `pose_a` with
/// the given sigmas. Returns the graph and the candidate between them.
fn pair(
    world_a: &World,
    pose_a: &Se2,
    world_b: &World,
    pose_b: &Se2,
    believed: &Se2,
    sx: f64,
    cfg: PoseGraphConfig,
) -> (PoseGraph, LoopCandidate) {
    let mut lid = lidar(31, true);
    let (sa, sb) = (
        scan_at(&mut lid, world_a, pose_a, 0),
        scan_at(&mut lid, world_b, pose_b, 1),
    );
    let mut g = PoseGraph::new(cfg).unwrap();
    g.add_keyframe(points(&sa), &Se2::IDENTITY, &info(sx, 0.01))
        .unwrap();
    g.add_keyframe(points(&sb), believed, &info(sx, 0.01))
        .unwrap();
    let cand = LoopCandidate {
        from: 0,
        to: 1,
        predicted: g.poses()[0].between(&g.poses()[1]),
        distance: believed.translation().norm(),
        sigma_position: sx,
    };
    (g, cand)
}

fn reason(v: &Verification) -> Option<Reason> {
    match v {
        Verification::Rejected { reason, .. } => Some(*reason),
        Verification::Accepted { .. } => None,
    }
}

#[test]
fn a_true_loop_is_accepted() {
    let w = pillar_room();
    let (a, b) = (Se2::new(0.5, 0.2, 0.3), Se2::new(0.58, 0.27, 0.34));
    let (g, c) = pair(
        &w,
        &a,
        &w,
        &b,
        &a.between(&b),
        0.05,
        PoseGraphConfig::default(),
    );
    match g.verify(&c).unwrap() {
        Verification::Accepted {
            measurement, chi2, ..
        } => {
            let e = a.between(&b).between(&measurement).log();
            assert!(e.xy().norm() < 0.02 && e.z.abs() < 0.02, "{e:?}");
            assert!(chi2 < 7.815, "{chi2}");
        }
        v => panic!("rejected: {v:?}"),
    }
}

#[test]
fn a_candidate_whose_views_do_not_overlap_is_rejected() {
    let big = square_room();
    let mut small = World::empty();
    small
        .add_rectangle(
            nalgebra::Vector2::new(-1.2, -1.2),
            nalgebra::Vector2::new(1.2, 1.2),
        )
        .unwrap();
    let p = Se2::new(0.3, 0.1, 0.2);
    let (g, c) = pair(
        &big,
        &p,
        &small,
        &p,
        &Se2::IDENTITY,
        0.05,
        PoseGraphConfig::default(),
    );
    let v = g.verify(&c).unwrap();
    assert_eq!(reason(&v), Some(Reason::NoOverlap), "{v:?}");
}

#[test]
fn a_corridor_is_rejected_as_degenerate_however_well_it_fits() {
    let hall = corridor();
    let (a, b) = (Se2::new(0.0, 0.0, 0.0), Se2::new(2.0, 0.0, 0.0));
    let (g, c) = pair(
        &hall,
        &a,
        &hall,
        &b,
        &a.between(&b),
        0.05,
        PoseGraphConfig::default(),
    );
    assert_eq!(reason(&g.verify(&c).unwrap()), Some(Reason::Degenerate));
}

#[test]
fn a_true_loop_displaced_beyond_the_graphs_covariance_fails_the_gate() {
    let w = pillar_room();
    let (a, b) = (Se2::new(0.5, 0.2, 0.3), Se2::new(0.58, 0.27, 0.34));
    // The graph believes b is 0.3 m from where it is, and is sure of it (sigma 2 cm).
    let believed = a.between(&b).boxplus(&Vector3::new(0.3, 0.0, 0.0));
    let (g, c) = pair(&w, &a, &w, &b, &believed, 0.02, PoseGraphConfig::default());
    match reason(&g.verify(&c).unwrap()) {
        Some(Reason::GateExceeded { chi2 }) => assert!(chi2 > 7.815),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_ninety_degree_alias_in_a_symmetric_room_is_rejected_as_ambiguous() {
    // At the centre of an empty square room, a scan taken facing east and one facing north
    // are the same scan: ICP cannot tell a quarter turn from none, and a verifier that stops
    // at fit and chi-squared accepts the alias with a perfect fit.
    let w = square_room();
    let (a, b) = (
        Se2::new(0.0, 0.0, 0.0),
        Se2::new(0.0, 0.0, std::f64::consts::FRAC_PI_2),
    );
    let (g, c) = pair(
        &w,
        &a,
        &w,
        &b,
        &a.between(&b),
        0.05,
        PoseGraphConfig::default(),
    );
    assert_eq!(reason(&g.verify(&c).unwrap()), Some(Reason::Ambiguous));
}

#[test]
fn the_same_alias_is_accepted_once_the_room_has_a_landmark_that_breaks_the_symmetry() {
    let mut w = square_room();
    w.add_box(
        nalgebra::Vector2::new(1.5, 0.9),
        nalgebra::Vector2::new(0.6, 0.4),
        0.4,
    )
    .unwrap();
    let (a, b) = (
        Se2::new(0.0, 0.0, 0.0),
        Se2::new(0.05, 0.02, std::f64::consts::FRAC_PI_2),
    );
    let (g, c) = pair(
        &w,
        &a,
        &w,
        &b,
        &a.between(&b),
        0.05,
        PoseGraphConfig::default(),
    );
    let v = g.verify(&c).unwrap();
    assert!(matches!(v, Verification::Accepted { .. }), "{v:?}");
}

#[test]
fn a_registration_that_cannot_settle_is_not_believed() {
    let w = pillar_room();
    let (a, b) = (Se2::new(0.5, 0.2, 0.3), Se2::new(0.58, 0.27, 0.34));
    let mut cfg = PoseGraphConfig::default();
    cfg.loops.icp.max_iterations = 1;
    let believed = a.between(&b).boxplus(&Vector3::new(0.25, 0.1, 0.0));
    let (g, c) = pair(&w, &a, &w, &b, &believed, 0.3, cfg);
    assert_eq!(reason(&g.verify(&c).unwrap()), Some(Reason::NotConverged));
}

/// A wall now hides the east side of the view from the robot at the origin.
fn partly_occluded_pair(cfg: PoseGraphConfig) -> (PoseGraph, LoopCandidate, Se2) {
    let w = pillar_room();
    let mut changed = pillar_room();
    changed
        .add_segment(
            nalgebra::Vector2::new(1.2, -3.0),
            nalgebra::Vector2::new(1.2, 3.0),
        )
        .unwrap();
    let (a, b) = (Se2::new(0.0, 0.0, 0.0), Se2::new(0.05, 0.0, 0.02));
    let truth = a.between(&b);
    let (g, c) = pair(&w, &a, &changed, &b, &truth, 0.05, cfg);
    (g, c, truth)
}

#[test]
fn a_partly_changed_scene_is_still_a_loop_when_most_of_it_matches() {
    let (g, c, truth) = partly_occluded_pair(PoseGraphConfig::default());
    match g.verify(&c).unwrap() {
        Verification::Accepted { measurement, .. } => {
            let e = truth.between(&measurement).log();
            assert!(e.xy().norm() < 0.03 && e.z.abs() < 0.03, "{e:?}");
        }
        v => panic!("{v:?}"),
    }
}

#[test]
fn the_same_scene_is_refused_as_a_poor_fit_when_the_inlier_floor_is_high() {
    let mut cfg = PoseGraphConfig::default();
    cfg.loops.fit_inlier_fraction_min = 0.95;
    let (g, c, _) = partly_occluded_pair(cfg);
    assert_eq!(reason(&g.verify(&c).unwrap()), Some(Reason::PoorFit));
}

#[test]
fn a_rejected_verification_cannot_be_added_to_the_graph() {
    let hall = corridor();
    let (a, b) = (Se2::new(0.0, 0.0, 0.0), Se2::new(2.0, 0.0, 0.0));
    let (mut g, c) = pair(
        &hall,
        &a,
        &hall,
        &b,
        &a.between(&b),
        0.05,
        PoseGraphConfig::default(),
    );
    let v = g.verify(&c).unwrap();
    assert!(matches!(
        g.add_loop(&c, &v),
        Err(SlamError::InvalidParameter { .. })
    ));
    assert_eq!(g.loop_edge_count(), 0);
}

#[test]
fn the_pipeline_is_deterministic() {
    let run = || {
        let d = drive(1.6, 0.004, 23, true);
        d.graph
            .poses()
            .iter()
            .flat_map(|p| [p.x().to_bits(), p.y().to_bits(), p.theta().to_bits()])
            .collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}
