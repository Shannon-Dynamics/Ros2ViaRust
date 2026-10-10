//! `mcl_node`: particle filter (MCL) localization against the known pillar map (Phase 3).
//!
//! ```text
//!   sensor_msgs/LaserScan (topics.scan) ─► pillar detector ─► range/bearing observations ─┐
//!   nav_msgs/Odometry     (topics.odom) ─► two consecutive odometry poses ──────────────┐  │
//!                                                                                       ▼  ▼
//!                                                          robotics_localization::McLocalizer
//!                                                                      │
//!              geometry_msgs/PoseWithCovarianceStamped (topics.pose, frame `frames.map`)
//!                                                                    + /diagnostics
//! ```
//!
//! Deliberately the same inputs, the same map, the same pillar detector and the same
//! landmark sensor model as `localization_node`. The only thing that differs between the
//! two binaries is the representation of the belief and what each one does with it; see
//! `robotics_localization::mcl` for why that is the whole point of having both.
//!
//! This node never subscribes to ground truth, for the same reason `localization_node`
//! does not: `/ground_truth/pose` exists for `localization_evaluator` and nobody else,
//! which is what makes a kidnap recovery claim about this node meaningful rather than
//! assumed.
//!
//! Two ways to start, chosen by `initial_pose.mode`:
//!
//! * `known` (the default): the pose is known to a few centimetres, from the map file's spawn
//!   pose or from `initial_pose.{x,y,theta}`, and the particles are spread around it. This is
//!   position tracking (P1) and, by the kidnap trigger, kidnap recovery.
//! * `uniform_prior`: the robot has no idea where it is. Particles are drawn uniformly over
//!   the free space of the map and the filter searches until its own convergence criterion
//!   declares success (`robotics_localization::global`). The spawn pose is ignored. While
//!   searching, `/diagnostics` reports `SEARCHING`, the published covariance is the real,
//!   wide one, and the kidnap trigger is inert; on convergence the node logs one
//!   `GLOBAL_LOCALIZATION_CONVERGED` line carrying the timestamps the experiments read.
//!
//! No TF is published here. `localization_node` is the sole owner of `map -> odom`
//! (REP 105; see its own header), and running both nodes in the same launch to compare
//! them head to head, which `scripts/run_phase3_correctness.sh` does for the kidnap
//! scenario, would otherwise make two nodes broadcast the same edge, a TF authority
//! conflict `robotics_core::tf::TransformBuffer` exists to detect, not to be walked into.
//! The comparison this phase needs is on `/mcl/pose` against `/localization/pose`, both
//! graded against the same ground truth by two instances of `localization_evaluator`.

use rand::rngs::SmallRng;
use rclrs::*;
use robotics_core::tf::{LookupTime, TransformBuffer, TransformBufferConfig};
use robotics_core::{seeded_rng, Cov3, FrameId, Se2, Timestamp};
use robotics_localization::{
    detect, GlobalLocalizationConfig, LocalizationPhase, MapFile, McLocalizer, MclConfig,
    PillarDetector,
};
use robotics_ros_bridge::convert::{
    laser_scan_from_ros, now, odometry_from_ros, pose_with_covariance_stamped_to_ros,
};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::tf::TfListener;
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::{geometry_msgs, nav_msgs, sensor_msgs};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// How the filter starts; see the header of this file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InitialMode {
    Known,
    UniformPrior,
}

/// Everything the callbacks share.
struct State {
    cfg: MclConfig,
    detector: PillarDetector,
    map: robotics_models::sensor::landmark::LandmarkMap,
    initial_pose: Se2,
    /// Built lazily, once the laser mount is known (from TF or from parameters), exactly
    /// as `localization_node::State::filter` is.
    filter: Option<McLocalizer>,
    rng: SmallRng,
    last_odom: Option<Se2>,
    map_frame: FrameId,
    base_frame: FrameId,
    publisher: Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>,
    min_publish_interval: f64,
    last_publish: Option<Timestamp>,
    published: u64,
    scans: u64,
    detections: u64,
    rejects: u64,
    last_error: String,
    sensor_pose_source: &'static str,
    last_injected_at: Option<Timestamp>,
    mode: InitialMode,
    global_cfg: GlobalLocalizationConfig,
    /// Stamp of the first scan the filter processed; the origin of a convergence time.
    started_at: Option<Timestamp>,
    converged_at: Option<Timestamp>,
    observe_ms_last: f64,
    observe_ms_max: f64,
}

impl State {
    fn ensure_filter(&mut self, node: &Node, sensor_pose: Se2, source: &'static str) {
        if self.filter.is_some() {
            return;
        }
        let cfg = MclConfig {
            sensor_pose,
            ..self.cfg
        };
        let built = match self.mode {
            InitialMode::Known => {
                McLocalizer::new_at(&cfg, self.map.clone(), self.initial_pose, &mut self.rng)
            }
            InitialMode::UniformPrior => McLocalizer::new_uniform_prior(
                &cfg,
                &self.global_cfg,
                self.map.clone(),
                &mut self.rng,
            ),
        };
        match built {
            Ok(f) => {
                self.cfg = cfg;
                self.sensor_pose_source = source;
                let origin = match self.mode {
                    InitialMode::Known => format!(
                        "at ({:.3}, {:.3}, {:.3})",
                        self.initial_pose.x(),
                        self.initial_pose.y(),
                        self.initial_pose.theta()
                    ),
                    InitialMode::UniformPrior => {
                        "from a UNIFORM PRIOR over the free space (no initial pose)".to_string()
                    }
                };
                log_info!(
                    node.logger(),
                    "MCL started {} with {} particles, {} landmarks, \
                     laser at ({:.3}, {:.3}, {:.3}) from {}, fit threshold {:.2} nats/obs",
                    origin,
                    f.len(),
                    self.map.len(),
                    sensor_pose.x(),
                    sensor_pose.y(),
                    sensor_pose.theta(),
                    source,
                    f.fit_log_threshold()
                );
                self.filter = Some(f);
            }
            Err(e) => {
                self.last_error = format!("MCL: {e}");
                log_error!(node.logger(), "cannot build the MCL filter: {e}");
            }
        }
    }

    fn publish(&mut self, node: &Node, stamp: Timestamp) {
        let Some(f) = &self.filter else { return };
        if let Some(last) = self.last_publish {
            let since = stamp - last;
            if since >= 0.0 && since < self.min_publish_interval {
                return;
            }
        }
        let pose = f.pose();
        let cov: Cov3 = f.covariance();
        let msg = pose_with_covariance_stamped_to_ros(stamp, &self.map_frame, &pose, &cov);
        if let Err(e) = self.publisher.publish(msg) {
            log_error!(
                node.logger().throttle(Duration::from_secs(5)),
                "pose publish failed: {e}"
            );
            return;
        }
        self.published += 1;
        self.last_publish = Some(stamp);
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("mcl_node")?;
    let mut p = ParamStore::new();

    let scan_topic = p.string(&node, "topics.scan", "scan", "Laser scan input")?;
    let odom_topic = p.string(&node, "topics.odom", "odom", "Odometry input")?;
    let pose_topic = p.string(
        &node,
        "topics.pose",
        "mcl/pose",
        "Estimated pose output (geometry_msgs/PoseWithCovarianceStamped)",
    )?;
    let map_frame = p.string(&node, "frames.map", "map", "Frame the estimate is in")?;
    let base_frame = p.string(&node, "frames.base_link", "base_link", "Body frame")?;
    let map_file = p.string(
        &node,
        "map_file",
        "",
        "Path to the landmark map (sim/webots/config/landmarks.yaml)",
    )?;

    let use_map_spawn = p.bool(
        &node,
        "initial_pose.from_map_file",
        true,
        "Start from the spawn pose written in the map file",
    )?;
    let init_x = p.f64(&node, "initial_pose.x", 0.0, -1e3, 1e3, "Initial x (m)")?;
    let init_y = p.f64(&node, "initial_pose.y", 0.0, -1e3, 1e3, "Initial y (m)")?;
    let init_theta = p.f64(
        &node,
        "initial_pose.theta",
        0.0,
        -10.0,
        10.0,
        "Initial heading (rad)",
    )?;

    let mode_name = p.string(
        &node,
        "initial_pose.mode",
        "known",
        "How the filter starts: `known` (particles around the spawn pose or initial_pose.x/y/theta) \
         or `uniform_prior` (global localization: particles uniform over the free space, no \
         initial pose at all)",
    )?;
    let mode = match mode_name.as_str() {
        "known" => InitialMode::Known,
        "uniform_prior" => InitialMode::UniformPrior,
        other => {
            log_error!(
                node.logger(),
                "initial_pose.mode must be `known` or `uniform_prior`, got `{other}`"
            );
            return Err(RclrsError::RclError {
                code: RclReturnCode::InvalidArgument,
                msg: None,
            });
        }
    };
    let g = GlobalLocalizationConfig::default();
    let robot_radius = p.f64(
        &node,
        "global.robot_radius",
        0.11,
        0.0,
        2.0,
        "Robot footprint radius (m); with the pillar radius from the map it sets the keep-out \
         around every pillar that the uniform prior is not drawn inside",
    )?;
    let global_cfg = GlobalLocalizationConfig {
        num_particles: p.i64(
            &node,
            "global.particles",
            g.num_particles as i64,
            1,
            1_000_000,
            "Particles used while a uniform-prior search is running; thinned to particles.count \
             on convergence when larger",
        )? as usize,
        landmark_keepout_m: 0.0, // set from the map once it is loaded
        convergence_sigma_xy_m: p.f64(
            &node,
            "global.convergence_sigma_xy",
            g.convergence_sigma_xy_m,
            1e-4,
            10.0,
            "Convergence needs the cloud's RMS position spread below this (m)",
        )?,
        convergence_sigma_theta_rad: p.f64(
            &node,
            "global.convergence_sigma_theta",
            g.convergence_sigma_theta_rad,
            1e-4,
            10.0,
            "Convergence needs the cloud's circular heading spread below this (rad)",
        )?,
        convergence_sustained_scans: p.i64(
            &node,
            "global.convergence_sustained_scans",
            g.convergence_sustained_scans as i64,
            1,
            10_000,
            "Consecutive scored scans the convergence criterion has to hold for",
        )? as u32,
        convergence_min_explained_fraction: p.f64(
            &node,
            "global.convergence_min_explained_fraction",
            g.convergence_min_explained_fraction,
            0.01,
            1.0,
            "Fraction of a scan's detections the cloud's mean pose must explain for the scan to \
             count towards convergence (0.5 is the upper-median rule, 1.0 is every detection)",
        )?,
    };

    let use_tf = p.bool(
        &node,
        "sensor.from_tf",
        true,
        "Look the laser mount up in TF (base_link -> scan frame) instead of the parameters",
    )?;
    let sensor_x = p.f64(
        &node,
        "sensor.x",
        0.0,
        -5.0,
        5.0,
        "Laser x in the body frame (m)",
    )?;
    let sensor_y = p.f64(
        &node,
        "sensor.y",
        0.0,
        -5.0,
        5.0,
        "Laser y in the body frame (m)",
    )?;
    let sensor_theta = p.f64(
        &node,
        "sensor.theta",
        0.0,
        -10.0,
        10.0,
        "Laser yaw in the body frame (rad)",
    )?;

    let d = PillarDetector::default();
    let detector = PillarDetector {
        max_range: p.f64(
            &node,
            "detector.max_range",
            d.max_range,
            0.1,
            100.0,
            "Ignore returns beyond this range (m)",
        )?,
        cluster_gap: p.f64(
            &node,
            "detector.cluster_gap",
            d.cluster_gap,
            1e-3,
            5.0,
            "Range jump that splits two clusters (m)",
        )?,
        min_points: p.i64(
            &node,
            "detector.min_points",
            d.min_points as i64,
            1,
            100,
            "Smallest number of beams in a pillar",
        )? as usize,
        max_chord: p.f64(
            &node,
            "detector.max_chord",
            d.max_chord,
            0.01,
            10.0,
            "Longest chord a pillar cluster may have (m)",
        )?,
        radius: p.f64(
            &node,
            "detector.radius",
            d.radius,
            0.0,
            5.0,
            "Pillar radius added to the measured surface range (m); 0 reads it from the map file",
        )?,
    };

    let c = MclConfig::default();
    let cfg = MclConfig {
        num_particles: p.i64(
            &node,
            "particles.count",
            c.num_particles as i64,
            1,
            100_000,
            "Particle count, fixed for the filter's lifetime (see McLocalizer docs for why 300)",
        )? as usize,
        sigma_range: p.f64(
            &node,
            "measurement.sigma_range",
            c.sigma_range,
            1e-4,
            10.0,
            "Range noise of a landmark observation (m); shared with localization_node's \
             measurement.sigma_range so the two filters are graded on the same sensor",
        )?,
        sigma_bearing: p.f64(
            &node,
            "measurement.sigma_bearing",
            c.sigma_bearing,
            1e-4,
            1.0,
            "Bearing noise of a landmark observation (rad)",
        )?,
        initial_sigma_xy: p.f64(
            &node,
            "initial_pose.sigma_xy",
            c.initial_sigma_xy,
            1e-4,
            100.0,
            "Initial position sigma particles are spread with around initial_pose",
        )?,
        initial_sigma_theta: p.f64(
            &node,
            "initial_pose.sigma_theta",
            c.initial_sigma_theta,
            1e-4,
            10.0,
            "Initial heading sigma particles are spread with around initial_pose",
        )?,
        motion_alphas: [
            p.f64(
                &node,
                "motion.alpha1",
                c.motion_alphas[0],
                0.0,
                10.0,
                "Rotation noise from rotation",
            )?,
            p.f64(
                &node,
                "motion.alpha2",
                c.motion_alphas[1],
                0.0,
                10.0,
                "Rotation noise from translation",
            )?,
            p.f64(
                &node,
                "motion.alpha3",
                c.motion_alphas[2],
                0.0,
                10.0,
                "Translation noise from translation",
            )?,
            p.f64(
                &node,
                "motion.alpha4",
                c.motion_alphas[3],
                0.0,
                10.0,
                "Translation noise from rotation",
            )?,
        ],
        resample_threshold_ratio: p.f64(
            &node,
            "resample.threshold_ratio",
            c.resample_threshold_ratio,
            0.0,
            1.0,
            "Resample (low-variance) when the effective sample size drops below this \
             fraction of particles.count",
        )?,
        sensor_pose: Se2::IDENTITY,
        room_half_extent: p.f64(
            &node,
            "recovery.room_half_extent",
            c.room_half_extent,
            0.1,
            100.0,
            "Half the side of the square free-space region particles are injected into (m)",
        )?,
        recovery_gate_probability: p.f64(
            &node,
            "recovery.gate_probability",
            c.recovery_gate_probability,
            0.5,
            0.9999,
            "Same meaning as localization_node's measurement.gate_probability, reused as a \
             likelihood-ratio threshold; see McLocalizer::fit_log_threshold",
        )?,
        recovery_sustained_updates: p.i64(
            &node,
            "recovery.sustained_updates",
            c.recovery_sustained_updates as i64,
            1,
            1000,
            "Consecutive low-fit scans required before particles are injected",
        )? as u32,
        recovery_injection_fraction: p.f64(
            &node,
            "recovery.injection_fraction",
            c.recovery_injection_fraction,
            0.0,
            1.0,
            "Probability, independently per particle, of being replaced by a uniform random \
             pose when an injection triggers",
        )?,
    };
    let seed = p.i64(
        &node,
        "seed",
        robotics_core::BOOK_SEED as i64,
        0,
        i64::MAX,
        "RNG seed for particle motion sampling and recovery injection, so a run can be \
         replayed",
    )?;
    let max_rate = p.f64(
        &node,
        "max_publish_rate_hz",
        20.0,
        0.1,
        1000.0,
        "Upper bound on the output rate (Hz)",
    )?;
    let diag_period = p.i64(
        &node,
        "diagnostics_period_s",
        1,
        1,
        60,
        "Diagnostics period (s)",
    )?;

    if map_file.is_empty() {
        log_error!(node.logger(), "map_file is required");
        return Err(RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        });
    }
    let map_data = MapFile::load(&map_file).map_err(|e| {
        log_error!(node.logger(), "cannot read map {map_file}: {e}");
        RclrsError::RclError {
            code: RclReturnCode::InvalidArgument,
            msg: None,
        }
    })?;
    let detector = PillarDetector {
        radius: if detector.radius > 0.0 {
            detector.radius
        } else {
            map_data.nominal_radius()
        },
        ..detector
    };
    let global_cfg = GlobalLocalizationConfig {
        landmark_keepout_m: map_data.nominal_radius() + robot_radius,
        ..global_cfg
    };
    // Measured, not guessed: in a debug build one scan of a 3000 particle search took over
    // 100 ms (observe_ms_max of the integration test's second MCL), against a 200 ms scan
    // period, and the cost is linear in the particle count. A search that cannot keep up
    // does not fail loudly, it lags, and its convergence time then measures the build.
    if mode == InitialMode::UniformPrior
        && cfg!(debug_assertions)
        && global_cfg.num_particles > 3000
    {
        log_warn!(
            node.logger(),
            "this is a DEBUG build and global.particles is {}: expect a scan to take longer \
             than the 5 Hz scan period; build with scripts/build_release.sh for a search this \
             size (observe_ms_max in /diagnostics reports what a scan actually costs)",
            global_cfg.num_particles
        );
    }
    let initial_pose = match (use_map_spawn, map_data.spawn_pose) {
        (true, Some(p)) => p,
        _ => Se2::new(init_x, init_y, init_theta),
    };
    log_info!(
        node.logger(),
        "map {map_file}: {} landmarks, pillar radius {:.3} m",
        map_data.landmarks.len(),
        detector.radius
    );

    let publisher = node.create_publisher::<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic.as_str().qos(qos::odometry()),
    )?;
    let diagnostics = DiagnosticsPublisher::new(&node)?;
    let tf_buffer = Shared::new(
        TransformBuffer::new(TransformBufferConfig::default()).expect("default TF buffer config"),
    );
    let tf_listener = TfListener::new(&node, tf_buffer.clone())?;

    let shared = Shared::new(State {
        cfg,
        detector,
        map: map_data.landmark_map(),
        initial_pose,
        filter: None,
        rng: seeded_rng(seed as u64),
        last_odom: None,
        map_frame: FrameId::new(&map_frame),
        base_frame: FrameId::new(&base_frame),
        publisher,
        min_publish_interval: 1.0 / max_rate,
        last_publish: None,
        published: 0,
        scans: 0,
        detections: 0,
        rejects: 0,
        last_error: String::new(),
        sensor_pose_source: "pending",
        last_injected_at: None,
        mode,
        global_cfg,
        started_at: None,
        converged_at: None,
        observe_ms_last: 0.0,
        observe_ms_max: 0.0,
    });

    // ---- Odometry: prediction ----
    let n = Arc::clone(&node);
    let _odom_sub = node.create_subscription::<nav_msgs::msg::Odometry, _>(
        odom_topic.as_str().qos(qos::odometry()),
        {
            let shared = shared.clone();
            move |msg: nav_msgs::msg::Odometry| {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                let odom = match odometry_from_ros(&msg) {
                    Ok(o) => o,
                    Err(e) => {
                        st.rejects += 1;
                        st.last_error = format!("odom: {e}");
                        return;
                    }
                };
                let previous = st.last_odom;
                st.last_odom = Some(odom.pose);

                let Some(previous) = previous else { return };
                if st.filter.is_none() {
                    return;
                }
                if let Some(f) = st.filter.as_mut() {
                    f.predict(&previous, &odom.pose, &mut st.rng);
                }
                st.publish(&n, odom.stamp);
            }
        },
    )?;

    // ---- Scan: detection, reweighting, recovery, resampling ----
    let n = Arc::clone(&node);
    let _scan_sub = node.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        scan_topic.as_str().qos(qos::sensor_data()),
        {
            let shared = shared.clone();
            move |msg: sensor_msgs::msg::LaserScan| {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                let scan = match laser_scan_from_ros(&msg) {
                    Ok(s) => s,
                    Err(e) => {
                        st.rejects += 1;
                        st.last_error = format!("scan: {e}");
                        log_warn!(
                            n.logger().throttle(Duration::from_secs(5)),
                            "scan rejected: {e}"
                        );
                        return;
                    }
                };
                if st.filter.is_none() {
                    if use_tf {
                        match tf_listener.lookup_transform(
                            st.base_frame.as_str(),
                            scan.frame.as_str(),
                            LookupTime::Latest,
                        ) {
                            Ok(t) => st.ensure_filter(&n, t.transform, "tf"),
                            Err(e) => {
                                log_warn!(
                                    n.logger().throttle(Duration::from_secs(5)),
                                    "waiting for TF {} -> {}: {e}",
                                    st.base_frame,
                                    scan.frame
                                );
                                return;
                            }
                        }
                    } else {
                        st.ensure_filter(
                            &n,
                            Se2::new(sensor_x, sensor_y, sensor_theta),
                            "parameters",
                        );
                    }
                }
                if st.started_at.is_none() && st.filter.is_some() {
                    st.started_at = Some(scan.stamp);
                    if st.mode == InitialMode::UniformPrior {
                        log_info!(
                            n.logger(),
                            "GLOBAL_LOCALIZATION_STARTED stamp_s={:.3} particles={}",
                            scan.stamp.as_secs_f64(),
                            st.filter.as_ref().map_or(0, |f| f.len())
                        );
                    }
                }
                st.scans += 1;
                let detections = detect(&scan, &st.detector);
                st.detections += detections.len() as u64;
                let observations: Vec<_> = detections.iter().map(|d| d.z).collect();
                let Some(f) = st.filter.as_mut() else { return };
                let t0 = Instant::now();
                let report = f.observe(&observations, &mut st.rng);
                let observe_ms = t0.elapsed().as_secs_f64() * 1e3;
                st.observe_ms_last = observe_ms;
                st.observe_ms_max = st.observe_ms_max.max(observe_ms);
                if report.converged_now {
                    st.converged_at = Some(scan.stamp);
                    if let Some(c) = f.global_status().and_then(|g| g.converged) {
                        log_info!(
                            n.logger(),
                            "GLOBAL_LOCALIZATION_CONVERGED stamp_s={:.3} started_stamp_s={:.3} \
                             scored_updates={} travel_odom_m={:.3} sigma_xy={:.4} sigma_theta={:.4} \
                             particles_before={} particles_after={}",
                            scan.stamp.as_secs_f64(),
                            st.started_at.map_or(f64::NAN, |t| t.as_secs_f64()),
                            c.scored_updates,
                            c.travel_m,
                            c.spread.sigma_xy,
                            c.spread.sigma_theta,
                            c.particles_before,
                            c.particles_after
                        );
                    }
                }
                if report.injected {
                    st.last_injected_at = Some(scan.stamp);
                    // `stamp_s` is the scan's own stamp, the clock the evaluator's CSV uses, so
                    // scripts/phase3_global_verdict.py can grade each injection against ground
                    // truth: was the estimate already right when the trigger fired?
                    log_warn!(
                        n.logger(),
                        "KIDNAP_INJECTION stamp_s={:.3}: {} consecutive scan(s) had a \
                         best-particle median fit of {:.2} nats/obs, below the {:.2} threshold \
                         with {} observations: injecting random particles",
                        scan.stamp.as_secs_f64(),
                        st.cfg.recovery_sustained_updates,
                        report
                            .best_fit_median_log_likelihood
                            .unwrap_or(f64::NEG_INFINITY),
                        f.fit_log_threshold(),
                        report.observed
                    );
                }
                st.publish(&n, scan.stamp);
            }
        },
    )?;

    let n = Arc::clone(&node);
    let _diag = node.create_timer_repeating(Duration::from_secs(diag_period as u64), {
        let shared = shared.clone();
        move || {
            let mut guard = shared.lock();
            let st: &mut State = &mut guard;
            let stamp = match now(&n) {
                Ok(t) => t,
                Err(e) => {
                    log_error!(n.logger(), "clock error: {e}");
                    return;
                }
            };
            let (level, message) = match &st.filter {
                None => (DiagnosticLevel::Warn, "WAITING".to_string()),
                Some(f) if f.phase() == LocalizationPhase::GlobalSearch => (
                    DiagnosticLevel::Warn,
                    "SEARCHING: global localization from a uniform prior has not converged"
                        .to_string(),
                ),
                Some(f) => {
                    let s = f.stats();
                    if s.injections > 0 {
                        (
                            DiagnosticLevel::Warn,
                            "RECOVERING: at least one kidnap-style injection has fired".to_string(),
                        )
                    } else {
                        (DiagnosticLevel::Ok, "RUNNING".to_string())
                    }
                }
            };
            let mut b = StatusBuilder::new("mcl_node: particle filter", level, &message)
                .value("scans", st.scans)
                .value("detections", st.detections)
                .value("published", st.published)
                .value("rejects", st.rejects)
                .value("sensor_pose_source", st.sensor_pose_source)
                .value("last_error", &st.last_error);
            if let Some(f) = &st.filter {
                let s = f.stats();
                let p = f.covariance();
                b = b
                    .value("particles", f.len())
                    .value("x", format!("{:.3}", f.pose().x()))
                    .value("y", format!("{:.3}", f.pose().y()))
                    .value("theta", format!("{:.4}", f.pose().theta()))
                    .value(
                        "sigma_xy",
                        format!("{:.4}", p[(0, 0)].max(p[(1, 1)]).sqrt()),
                    )
                    .value("sigma_theta", format!("{:.4}", p[(2, 2)].sqrt()))
                    .value("ess", format!("{:.1}", f.effective_sample_size()))
                    .value("predictions", s.predictions)
                    .value("scored_updates", s.scored_updates)
                    .value("resamples", s.resamples)
                    .value("injections", s.injections)
                    .value("fit_log_threshold", format!("{:.3}", f.fit_log_threshold()))
                    .value(
                        "initial_pose_mode",
                        match st.mode {
                            InitialMode::Known => "known",
                            InitialMode::UniformPrior => "uniform_prior",
                        },
                    )
                    .value(
                        "phase",
                        match f.phase() {
                            LocalizationPhase::Tracking => "tracking",
                            LocalizationPhase::GlobalSearch => "global_search",
                        },
                    )
                    .value("observe_ms_last", format!("{:.2}", st.observe_ms_last))
                    .value("observe_ms_max", format!("{:.2}", st.observe_ms_max));
                if let Some(g) = f.global_status() {
                    b = b
                        .value("global_sigma_xy", format!("{:.4}", g.spread.sigma_xy))
                        .value("global_sigma_theta", format!("{:.4}", g.spread.sigma_theta))
                        .value("global_streak", g.streak)
                        .value("global_travel_odom_m", format!("{:.3}", g.travel_m));
                }
                if let Some(t) = st.converged_at {
                    b = b.value("global_converged_at_s", format!("{:.3}", t.as_secs_f64()));
                }
                if let Some(t) = st.last_injected_at {
                    b = b.value("last_injected_at_s", format!("{:.3}", t.as_secs_f64()));
                }
            }
            if let Err(e) = diagnostics.publish(stamp, vec![b.build()]) {
                log_error!(n.logger(), "diagnostics publish failed: {e}");
            }
        }
    })?;

    log_info!(
        node.logger(),
        "mcl_node up: {} + {} -> {} (frame {}), {} parameters",
        scan_topic,
        odom_topic,
        pose_topic,
        map_frame,
        p.len()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "mcl_node shutting down");
    drop(p);
    result
}
