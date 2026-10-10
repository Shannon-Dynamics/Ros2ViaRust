//! `localization_node` — EKF localization against the known pillar map (Phase 3).
//!
//! ```text
//!   sensor_msgs/LaserScan (topics.scan) ─► pillar detector ─► range/bearing observations ─┐
//!   nav_msgs/Odometry     (topics.odom) ─► relative motion u ──────────────────────────┐  │
//!   sensor_msgs/Imu       (topics.imu)  ─► integrated yaw rate (heading of u) ─────────┤  │
//!                                                                                      ▼  ▼
//!                                                         robotics_localization::EkfLocalizer
//!                                                                      │
//!              geometry_msgs/PoseWithCovarianceStamped (topics.pose, frame `frames.map`)
//!                                                                    + /diagnostics
//! ```
//!
//! The map comes from `sim/webots/config/landmarks.yaml`, the same file the Webots world
//! is built around, read by `robotics_localization::MapFile`.
//!
//! This node never subscribes to ground truth. The only absolute information it uses is
//! the laser against the map; `/ground_truth/pose` exists for the evaluator and for nobody
//! else, which is what makes the kidnapped robot experiment meaningful.
//!
//! TF: it publishes `map -> odom` and nothing else. That edge is the correction a
//! localization node owns by ROS 2 convention (REP 105): the simulated base owns
//! `odom -> base_link` and keeps it continuous, and the localizer expresses the whole
//! accumulated odometry error in the one jump above it, so that `map -> base_link`
//! resolves to this node's estimate while `odom -> base_link` stays smooth. There is
//! therefore no TF authority conflict with the driver, and `scripts/verify_phase3_localization.sh`
//! checks the complete chain `map -> odom -> base_link -> {LDS-01, imu_link}`.
//!
//! The edge is computed from the estimate and the odometry pose that produced it,
//! `map -> odom  =  (map -> base_link) * (odom -> base_link)^-1`, and is published with
//! the stamp of the estimate. It can be switched off with `publish_tf:=false`.

use rclrs::*;
use robotics_core::tf::{LookupTime, TransformBuffer, TransformBufferConfig};
use robotics_core::{Cov3, FrameId, Se2, Timestamp};
use robotics_localization::{
    detect, EkfLocalizer, LocalizerConfig, MapFile, OdometryMotion, PillarDetector,
};
use robotics_ros_bridge::convert::{
    laser_scan_from_ros, now, odometry_from_ros, pose_with_covariance_stamped_to_ros,
};
use robotics_ros_bridge::diagnostics::{DiagnosticLevel, DiagnosticsPublisher, StatusBuilder};
use robotics_ros_bridge::shared::Shared;
use robotics_ros_bridge::tf::{TfBroadcaster, TfListener};
use robotics_ros_bridge::{qos, ParamStore};
use ros_env::{geometry_msgs, nav_msgs, sensor_msgs};
use std::sync::Arc;
use std::time::Duration;

/// Everything the callbacks share.
struct State {
    cfg: LocalizerConfig,
    motion: OdometryMotion,
    detector: PillarDetector,
    map: robotics_models::sensor::landmark::LandmarkMap,
    initial_pose: Se2,
    /// Built lazily, once the laser mount is known (from TF or from parameters).
    filter: Option<EkfLocalizer>,
    /// Last odometry pose seen, the anchor of the next relative motion.
    last_odom: Option<Se2>,
    last_odom_stamp: Option<Timestamp>,
    /// Most recent gyro yaw rate and the stamp it was measured at. The rate is held
    /// constant over the next odometry interval rather than integrated over the IMU's own
    /// interval: the IMU runs at 20 Hz and the odometry at 50 Hz, so integrating per IMU
    /// sample would attribute 50 ms of rotation to a 20 ms step and over-rotate the
    /// prediction by more than a factor of two.
    last_yaw_rate: Option<(Timestamp, f64)>,
    imu_timeout: f64,
    use_gyro_heading: bool,
    map_frame: FrameId,
    odom_frame: FrameId,
    base_frame: FrameId,
    publisher: Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>,
    /// `map -> odom` broadcaster, absent when `publish_tf` is false.
    tf: Option<TfBroadcaster>,
    tf_sent: u64,
    min_publish_interval: f64,
    last_publish: Option<Timestamp>,
    published: u64,
    scans: u64,
    detections: u64,
    rejects: u64,
    last_error: String,
    sensor_pose_source: &'static str,
}

impl State {
    fn ensure_filter(&mut self, node: &Node, sensor_pose: Se2, source: &'static str) {
        if self.filter.is_some() {
            return;
        }
        let cfg = LocalizerConfig {
            sensor_pose,
            ..self.cfg
        };
        match EkfLocalizer::new(&cfg, self.map.clone(), self.initial_pose) {
            Ok(f) => {
                self.cfg = cfg;
                self.sensor_pose_source = source;
                log_info!(
                    node.logger(),
                    "localizer started at ({:.3}, {:.3}, {:.3}) with {} landmarks, \
                     laser at ({:.3}, {:.3}, {:.3}) from {}, gate chi2(2) = {:.2}",
                    self.initial_pose.x(),
                    self.initial_pose.y(),
                    self.initial_pose.theta(),
                    self.map.len(),
                    sensor_pose.x(),
                    sensor_pose.y(),
                    sensor_pose.theta(),
                    source,
                    f.gate()
                );
                self.filter = Some(f);
            }
            Err(e) => {
                self.last_error = format!("localizer: {e}");
                log_error!(node.logger(), "cannot build the localizer: {e}");
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
        // The filter keeps its covariance in the body frame (right perturbation); the
        // message convention is the parent frame, so rotate it with the adjoint.
        let ad = pose.adjoint();
        let cov: Cov3 = ad * f.covariance() * ad.transpose();
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

        // map -> odom: the correction between where the base thinks it is and where the
        // filter says it is. Without an odometry pose there is nothing to correct yet, and
        // publishing identity would claim the two frames coincide.
        if let (Some(tf), Some(odom)) = (&self.tf, self.last_odom) {
            let correction = pose.compose(&odom.inverse());
            match tf.send(stamp, &self.map_frame, &self.odom_frame, &correction) {
                Ok(()) => self.tf_sent += 1,
                Err(e) => log_error!(
                    node.logger().throttle(Duration::from_secs(5)),
                    "map -> odom broadcast failed: {e}"
                ),
            }
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let shutdown = robotics_ros_bridge::shutdown::install(&executor)?;
    let node = executor.create_node("localization_node")?;
    let mut p = ParamStore::new();

    let scan_topic = p.string(&node, "topics.scan", "scan", "Laser scan input")?;
    let odom_topic = p.string(&node, "topics.odom", "odom", "Odometry input")?;
    let imu_topic = p.string(&node, "topics.imu", "imu", "IMU input")?;
    let pose_topic = p.string(
        &node,
        "topics.pose",
        "localization/pose",
        "Estimated pose output (geometry_msgs/PoseWithCovarianceStamped)",
    )?;
    let map_frame = p.string(&node, "frames.map", "map", "Frame the estimate is in")?;
    let odom_frame = p.string(
        &node,
        "frames.odom",
        "odom",
        "Odometry frame; the child of the `map -> odom` correction this node publishes",
    )?;
    let base_frame = p.string(&node, "frames.base_link", "base_link", "Body frame")?;
    let publish_tf = p.bool(
        &node,
        "publish_tf",
        true,
        "Broadcast `map -> odom`. This node is the sole owner of that edge;          `odom -> base_link` belongs to the base and is never published here",
    )?;
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

    let c = LocalizerConfig::default();
    let m = OdometryMotion::default();
    let motion = OdometryMotion {
        alphas: [
            p.f64(
                &node,
                "motion.alpha1",
                m.alphas[0],
                0.0,
                10.0,
                "Rotation noise from rotation",
            )?,
            p.f64(
                &node,
                "motion.alpha2",
                m.alphas[1],
                0.0,
                10.0,
                "Rotation noise from translation",
            )?,
            p.f64(
                &node,
                "motion.alpha3",
                m.alphas[2],
                0.0,
                10.0,
                "Translation noise from translation",
            )?,
            p.f64(
                &node,
                "motion.alpha4",
                m.alphas[3],
                0.0,
                10.0,
                "Translation noise from rotation",
            )?,
        ],
        lateral_fraction: p.f64(
            &node,
            "motion.lateral_fraction",
            m.lateral_fraction,
            0.0,
            1.0,
            "Sideways variance as a fraction of the along-track variance",
        )?,
        min_translation_sigma: p.f64(
            &node,
            "motion.min_translation_sigma",
            m.min_translation_sigma,
            1e-9,
            1.0,
            "Floor on the translation noise per step (m)",
        )?,
        min_rotation_sigma: p.f64(
            &node,
            "motion.min_rotation_sigma",
            m.min_rotation_sigma,
            1e-9,
            1.0,
            "Floor on the rotation noise per step (rad)",
        )?,
    };
    let cfg = LocalizerConfig {
        sigma_range: p.f64(
            &node,
            "measurement.sigma_range",
            c.sigma_range,
            1e-4,
            10.0,
            "Range noise of a landmark observation (m)",
        )?,
        sigma_bearing: p.f64(
            &node,
            "measurement.sigma_bearing",
            c.sigma_bearing,
            1e-4,
            1.0,
            "Bearing noise of a landmark observation (rad)",
        )?,
        motion,
        gate_probability: p.f64(
            &node,
            "measurement.gate_probability",
            c.gate_probability,
            0.5,
            0.9999,
            "Probability mass inside the association gate",
        )?,
        initial_sigma_xy: p.f64(
            &node,
            "initial_pose.sigma_xy",
            c.initial_sigma_xy,
            1e-4,
            100.0,
            "Initial position sigma (m)",
        )?,
        initial_sigma_theta: p.f64(
            &node,
            "initial_pose.sigma_theta",
            c.initial_sigma_theta,
            1e-4,
            10.0,
            "Initial heading sigma (rad)",
        )?,
        sensor_pose: Se2::IDENTITY,
    };
    let use_gyro_heading = p.bool(
        &node,
        "use_gyro_heading",
        false,
        "Take the heading change of the motion from the gyro instead of the wheels. Off by \
         default: the simulated base reports a heading that already tracks ground truth to \
         about a milliradian, so the gyro adds noise rather than information here.",
    )?;
    let imu_timeout = p.f64(
        &node,
        "imu_timeout",
        0.2,
        1e-3,
        10.0,
        "Largest age of a gyro sample still used for the heading of the motion (s)",
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
        motion,
        detector,
        map: map_data.landmark_map(),
        initial_pose,
        filter: None,
        last_odom: None,
        last_odom_stamp: None,
        last_yaw_rate: None,
        imu_timeout,
        use_gyro_heading,
        map_frame: FrameId::new(&map_frame),
        odom_frame: FrameId::new(&odom_frame),
        base_frame: FrameId::new(&base_frame),
        publisher,
        tf: if publish_tf {
            Some(TfBroadcaster::new(&node)?)
        } else {
            None
        },
        tf_sent: 0,
        min_publish_interval: 1.0 / max_rate,
        last_publish: None,
        published: 0,
        scans: 0,
        detections: 0,
        rejects: 0,
        last_error: String::new(),
        sensor_pose_source: "pending",
    });

    // ---- IMU: integrate the yaw rate between odometry messages ----
    let n = Arc::clone(&node);
    let _imu_sub = node.create_subscription::<sensor_msgs::msg::Imu, _>(
        imu_topic.as_str().qos(qos::sensor_data()),
        {
            let shared = shared.clone();
            move |msg: sensor_msgs::msg::Imu| {
                let mut guard = shared.lock();
                let st: &mut State = &mut guard;
                let stamp =
                    Timestamp::from_sec_nanosec(msg.header.stamp.sec, msg.header.stamp.nanosec);
                let rate = msg.angular_velocity.z;
                if !rate.is_finite() {
                    st.rejects += 1;
                    st.last_error = "imu: non-finite yaw rate".to_string();
                    log_warn!(
                        n.logger().throttle(Duration::from_secs(5)),
                        "imu: non-finite yaw rate"
                    );
                    return;
                }
                st.last_yaw_rate = Some((stamp, rate));
            }
        },
    )?;

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
                let (previous, previous_stamp) = (st.last_odom, st.last_odom_stamp);
                st.last_odom = Some(odom.pose);
                st.last_odom_stamp = Some(odom.stamp);

                let Some(previous) = previous else { return };
                if st.filter.is_none() {
                    return;
                }
                let mut u = EkfLocalizer::relative_motion(&previous, &odom.pose);
                if st.use_gyro_heading {
                    // Translation from the wheels, heading change from the gyro, with the
                    // yaw rate held constant over this odometry interval.
                    if let (Some(prev_stamp), Some((imu_stamp, rate))) =
                        (previous_stamp, st.last_yaw_rate)
                    {
                        let dt = odom.stamp - prev_stamp;
                        let age = (odom.stamp - imu_stamp).abs();
                        if dt > 0.0 && dt < 1.0 && age <= st.imu_timeout {
                            u = Se2::new(u.x(), u.y(), rate * dt);
                        }
                    }
                }
                if u.translation().norm() < 1e-9 && u.theta().abs() < 1e-9 {
                    return; // standing still: nothing to predict, and no noise to add
                }
                let motion = st.motion;
                if let Some(f) = st.filter.as_mut() {
                    if let Err(e) = f.predict(&motion, &u) {
                        st.rejects += 1;
                        st.last_error = format!("predict: {e}");
                        log_warn!(
                            n.logger().throttle(Duration::from_secs(5)),
                            "prediction rejected: {e}"
                        );
                        return;
                    }
                }
                st.publish(&n, odom.stamp);
            }
        },
    )?;

    // ---- Scan: detection, association, correction ----
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
                    // The laser mount decides where the observations point, so the filter
                    // is only built once it is known.
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
                st.scans += 1;
                let detections = detect(&scan, &st.detector);
                st.detections += detections.len() as u64;
                let observations: Vec<_> = detections.iter().map(|d| d.z).collect();
                let report = match st.filter.as_mut() {
                    Some(f) => f.observe(&observations),
                    None => return,
                };
                if report.accepted == 0 && report.observed > 0 {
                    log_warn!(
                        n.logger().throttle(Duration::from_secs(5)),
                        "{} observation(s) and none inside the gate: the belief does not \
                         explain what the laser sees",
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
                Some(f) => {
                    let s = f.stats();
                    if s.accepted == 0 && s.updates > 3 {
                        (
                            DiagnosticLevel::Warn,
                            "NO ASSOCIATIONS: nothing the laser sees fits the belief".to_string(),
                        )
                    } else {
                        (DiagnosticLevel::Ok, "RUNNING".to_string())
                    }
                }
            };
            let mut b = StatusBuilder::new("localization_node: ekf", level, &message)
                .value("scans", st.scans)
                .value("detections", st.detections)
                .value("published", st.published)
                .value("rejects", st.rejects)
                .value("sensor_pose_source", st.sensor_pose_source)
                .value(
                    "tf_map_to_odom",
                    if st.tf.is_some() { "published" } else { "off" },
                )
                .value("tf_sent", st.tf_sent)
                .value("last_error", &st.last_error);
            if let Some(f) = &st.filter {
                let s = f.stats();
                let p = f.covariance();
                b = b
                    .value("x", format!("{:.3}", f.pose().x()))
                    .value("y", format!("{:.3}", f.pose().y()))
                    .value("theta", format!("{:.4}", f.pose().theta()))
                    .value(
                        "sigma_xy",
                        format!("{:.4}", p[(0, 0)].max(p[(1, 1)]).sqrt()),
                    )
                    .value("sigma_theta", format!("{:.4}", p[(2, 2)].sqrt()))
                    .value("predictions", s.predictions)
                    .value("observations_accepted", s.accepted)
                    .value("observations_rejected", s.rejected);
                if let Some(nis) = f.mean_nis() {
                    b = b.value("mean_nis_2dof", format!("{nis:.3}"));
                }
            }
            if let Err(e) = diagnostics.publish(stamp, vec![b.build()]) {
                log_error!(n.logger(), "diagnostics publish failed: {e}");
            }
        }
    })?;

    log_info!(
        node.logger(),
        "localization_node up: {} + {} + {} -> {} (frame {}), map -> odom {}, {} parameters",
        scan_topic,
        odom_topic,
        imu_topic,
        pose_topic,
        map_frame,
        if publish_tf { "published" } else { "off" },
        p.len()
    );
    let result = robotics_ros_bridge::shutdown::spin(&mut executor, &shutdown);
    log_info!(node.logger(), "localization_node shutting down");
    drop(p);
    result
}
