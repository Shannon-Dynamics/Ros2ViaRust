//! Integration tests of the EKF-SLAM back end on a seeded synthetic world.
//!
//! The world is the four pillar centres of the Webots room, treated as ideal point landmarks.
//! Sensor: range limit 3.5 m, full-circle field of view, so a pillar is seen whenever it is
//! within 3.5 m. The robot drives a circle of radius 1.2 m about the room centre: forward
//! 0.12 m and a left turn of 0.1 rad per step (63 steps per lap), starting at (1.2, 0) heading
//! north. The filter is told the start pose exactly (the gauge). Odometry increments carry
//! Gaussian noise in the body frame (σ = 1 cm along, 0.5 cm across, 0.01 rad of heading per
//! step) and the filter is given exactly that covariance; measurements carry the configured
//! noise (σ_range = 5 cm, σ_bearing = 0.02 rad). All randomness is `seeded_rng(seed)`.

use robotics_core::angle::normalize_angle;
use robotics_core::nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use robotics_core::{seeded_rng, Gaussian, Se2};
use robotics_filters::consistency::{nees, ConsistencyStats};
use robotics_slam::ekf_slam::{EkfSlam, EkfSlamConfig, RangeBearing, UpdateReport};

const PILLARS: [(f64, f64); 4] = [(1.8, 1.2), (-1.4, 2.0), (-2.0, -1.0), (0.9, -2.1)];
const RANGE_LIMIT: f64 = 3.5;
const LAP: usize = 63;

/// Noise levels and sensing schedule of one synthetic run.
#[derive(Clone, Copy)]
struct Scenario {
    odom_sigma: [f64; 3],
    sigma_range: f64,
    sigma_bearing: f64,
    /// The sensor fires every `observe_every` steps.
    observe_every: usize,
}

const NOMINAL: Scenario = Scenario {
    odom_sigma: [0.01, 0.005, 0.01],
    sigma_range: 0.05,
    sigma_bearing: 0.02,
    observe_every: 1,
};

/// One step of a run, handed to the visitor.
struct Step<'a> {
    index: usize,
    filter: &'a EkfSlam,
    truth: Se2,
    report: UpdateReport,
}

fn run(seed: u64, steps: usize, sc: Scenario, mut visit: impl FnMut(Step<'_>)) -> EkfSlam {
    let r = Matrix3::from_diagonal(&Vector3::new(
        sc.odom_sigma[0].powi(2),
        sc.odom_sigma[1].powi(2),
        sc.odom_sigma[2].powi(2),
    ));
    let odometry_noise = Gaussian::new(Vector3::zeros(), r).unwrap();
    let measurement_noise = Gaussian::new(
        Vector2::zeros(),
        Matrix2::from_diagonal(&Vector2::new(
            sc.sigma_range.powi(2),
            sc.sigma_bearing.powi(2),
        )),
    )
    .unwrap();
    let mut rng = seeded_rng(seed);

    let start = Se2::new(1.2, 0.0, std::f64::consts::FRAC_PI_2);
    let config = EkfSlamConfig {
        sigma_range: sc.sigma_range,
        sigma_bearing: sc.sigma_bearing,
        ..EkfSlamConfig::default()
    };
    let mut filter = EkfSlam::new(start, config).unwrap();
    let mut truth = start;
    let motion = Se2::new(0.12, 0.0, 0.1);
    for index in 0..steps {
        truth = truth.compose(&motion);
        let n = odometry_noise.sample(&mut rng);
        let noisy = Se2::new(motion.x() + n[0], motion.y() + n[1], motion.theta() + n[2]);
        filter.predict(&noisy, &r).unwrap();

        let mut obs = Vec::new();
        if index % sc.observe_every == 0 {
            for &(px, py) in &PILLARS {
                let m = RangeBearing::observe(&truth, &Vector2::new(px, py));
                if m.range <= RANGE_LIMIT {
                    let e = measurement_noise.sample(&mut rng);
                    obs.push(RangeBearing {
                        range: m.range + e[0],
                        bearing: m.bearing + e[1],
                    });
                }
            }
        }
        let report = filter.update(&obs).unwrap();
        visit(Step {
            index,
            filter: &filter,
            truth,
            report,
        });
    }
    filter
}

/// NEES of the pose block against ground truth, in the filter's own (x, y, θ) coordinates.
fn pose_nees(filter: &EkfSlam, truth: &Se2) -> f64 {
    let est = filter.pose();
    let e = Vector3::new(
        truth.x() - est.x(),
        truth.y() - est.y(),
        normalize_angle(truth.theta() - est.theta()),
    );
    nees(&e, &filter.pose_covariance()).unwrap()
}

/// Distance from landmark `i` to its nearest pillar, and which pillar that is.
fn nearest_pillar(filter: &EkfSlam, i: usize) -> (usize, f64) {
    let l = filter.landmark(i);
    (0..PILLARS.len())
        .map(|k| (k, (l - Vector2::new(PILLARS[k].0, PILLARS[k].1)).norm()))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap()
}

#[test]
fn association_creates_exactly_four_landmarks_over_eight_seeds() {
    let mut worst = 0.0f64;
    for seed in 0..8 {
        let mut after_first_lap = None;
        let mut duplicates_seen = 0;
        let end = run(seed, 2 * LAP, NOMINAL, |s| {
            if s.index + 1 == LAP {
                after_first_lap = Some(s.filter.landmark_count());
            }
            if s.filter.landmark_count() > 4 {
                duplicates_seen += 1;
            }
        });
        assert_eq!(after_first_lap, Some(4), "seed {seed}: count after one lap");
        assert_eq!(end.landmark_count(), 4, "seed {seed}: count after two laps");
        assert_eq!(duplicates_seen, 0, "seed {seed}: a duplicate appeared");
        // Each landmark sits on a different pillar.
        let mut owners: Vec<usize> = (0..4).map(|i| nearest_pillar(&end, i).0).collect();
        owners.sort_unstable();
        assert_eq!(
            owners,
            vec![0, 1, 2, 3],
            "seed {seed}: pillars not one-to-one"
        );
        for i in 0..4 {
            worst = worst.max(nearest_pillar(&end, i).1);
        }
    }
    println!("worst landmark position error after two laps over 8 seeds: {worst:.4} m");
    assert!(worst < 0.25, "worst landmark error {worst} m");
}

#[test]
fn promotion_waits_for_three_sightings() {
    // With the sensor firing every step, the first pillar in view is promoted on step three.
    let mut first_promotion = None;
    run(3, 20, NOMINAL, |s| {
        if s.report.promoted > 0 && first_promotion.is_none() {
            first_promotion = Some(s.index);
        }
    });
    assert_eq!(first_promotion, Some(2));
}

#[test]
fn nees_is_consistent_over_a_short_horizon() {
    const SEEDS: usize = 25;
    const HORIZON: usize = 100;
    let mut sums = vec![0.0; HORIZON];
    let mut per_seed_mean = Vec::new();
    for seed in 0..SEEDS as u64 {
        let mut total = 0.0;
        run(100 + seed, HORIZON, NOMINAL, |s| {
            let e = pose_nees(s.filter, &s.truth);
            sums[s.index] += e;
            total += e;
        });
        per_seed_mean.push(total / HORIZON as f64);
    }

    // At a fixed step the NEES of independent seeds are independent χ²(3) draws, so the
    // seed average must lie in the χ²(3·SEEDS)/SEEDS interval.
    let (lo, hi) = robotics_core::prob::chi2_mean_interval(3, SEEDS, 0.01).unwrap();
    let mut outside = 0;
    for (k, total) in sums.iter().enumerate() {
        let m = total / SEEDS as f64;
        if m <= lo || m >= hi {
            outside += 1;
        }
        if [9, 24, 49, 74, 99].contains(&k) {
            println!(
                "step {:>3}: mean NEES over {SEEDS} seeds {m:.3}, 99% bounds [{lo:.3}, {hi:.3}]",
                k + 1
            );
            assert!(m > lo && m < hi, "step {}: {m} outside [{lo}, {hi}]", k + 1);
        }
    }
    println!("{outside} of {HORIZON} per-step seed averages outside the 99% interval");

    // Successive steps of one run are correlated, so the time average is judged across
    // seeds, which are independent: a t-interval from the spread of the per-seed averages.
    let n = SEEDS as f64;
    let mean = per_seed_mean.iter().sum::<f64>() / n;
    let var = per_seed_mean
        .iter()
        .map(|v| (v - mean).powi(2))
        .sum::<f64>()
        / (n - 1.0);
    let stderr = (var / n).sqrt();
    println!(
        "time-averaged NEES {mean:.3} +- {stderr:.3} (1 s.e.) over {SEEDS} seeds x {HORIZON} steps"
    );
    // 2.797 is the two-sided 99% quantile of Student's t with 24 degrees of freedom.
    assert!(
        (mean - 3.0).abs() < 2.797 * stderr,
        "mean {mean} vs 3 with s.e. {stderr}"
    );
}

#[test]
fn nees_turns_optimistic_on_a_long_loop_with_sharp_sensing_and_a_noisy_heading() {
    // The documented failure of EKF-SLAM: the linearization points are the estimates, so the
    // heading error leaks into the map and the filter becomes over-confident. A heading
    // noise of 0.03 rad per step with a sharp sensor (2 cm, 0.005 rad) shows it clearly.
    const SEEDS: usize = 20;
    const HORIZON: usize = 200;
    let sc = Scenario {
        odom_sigma: [0.01, 0.005, 0.03],
        sigma_range: 0.02,
        sigma_bearing: 0.005,
        observe_every: 1,
    };
    let mut stats = ConsistencyStats::new(3).unwrap();
    for seed in 0..SEEDS as u64 {
        run(500 + seed, HORIZON, sc, |s| {
            stats.push(pose_nees(s.filter, &s.truth)).unwrap();
        });
    }
    let report = stats.report(0.01).unwrap();
    println!(
        "long loop, noisy heading: mean NEES {:.2} over {SEEDS} seeds x {HORIZON} steps, upper bound {:.3}",
        report.mean, report.upper
    );
    assert!(report.is_overconfident(), "{report:?}");
}

#[test]
fn same_seed_gives_bit_identical_state_and_covariance() {
    let bits = |f: &EkfSlam| -> Vec<u64> {
        f.state()
            .iter()
            .chain(f.covariance().iter())
            .map(|v| v.to_bits())
            .collect()
    };
    let a = run(7, 80, NOMINAL, |_| {});
    let b = run(7, 80, NOMINAL, |_| {});
    assert_eq!(bits(&a), bits(&b));
    let c = run(8, 80, NOMINAL, |_| {});
    assert_ne!(bits(&a), bits(&c));
}
