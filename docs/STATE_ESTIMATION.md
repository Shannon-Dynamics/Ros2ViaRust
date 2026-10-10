# State Estimation (Phase 2)

What the Phase 2 estimation stack computes, how it is wired into ROS 2, how it was
validated, and what it does not do. Library code: `crates/robotics_filters`; node:
`ros2_ws/src/robotics_estimation/src/state_estimator_node.rs`; simulator and evaluator:
`crates/robotics_sim`, `ros2_ws/src/robotics_sim_bridge`.

## 1. Pipeline

```text
sim_node ── joint_states (50 Hz) ──► wheel_odometry_node ── wheel/odometry (50 Hz) ──┐
         ── imu/data (100 Hz) ───────────────────────────────────────────────────────┼─► state_estimator_node
         ── scan (10 Hz, consumed from Phase 3 on)                                   │     ├─ odom            (nav_msgs/Odometry)
         ── ground_truth/odom (100 Hz, evaluation only)                              │     ├─ state_estimate  (PoseWithCovarianceStamped)
                                                                                     │     ├─ TF odom → base_link
                                                                                     │     └─ /diagnostics (health, counters, mean NIS)
estimator_evaluator ◄── ground_truth/odom, wheel/odometry, odom  (RMSE vs truth)
```

`state_estimator_node` is a thin adapter: it converts `sensor_msgs/Imu` (strict planar
conversion, REP 145 covariance conventions) and `nav_msgs/Odometry` into core types and
calls `robotics_filters::OdomImuFusion`. All timing policy lives in the library and is
unit tested without ROS.

## 2. Filter

`EskfSe2` — error-state Kalman filter on SE(2) (book Chapter 7):

| | |
|---|---|
| nominal state | pose `T ∈ SE(2)`, forward speed `v`, gyro bias `b` |
| error state | `δx = (δρ_x, δρ_y, δθ, δv, δb)` ∈ ℝ⁵, right (body-frame) perturbation `T = T̂·Exp(δξ)` |
| propagation | gyro `ω = ω_m − b̂` (zero-order hold between IMU samples), `T̂ ← T̂·Exp(v̂Δt, 0, ωΔt)` |
| wheel update | `z = (v_w, ω_w − ω̄_m)` observes `v` and `−b`; at `|v| > bias_update_max_speed` speed only |
| correction | Joseph form, Cholesky solves, relative pivot test on `S` (`robotics_filters::update`) |
| reset | `P ← G P Gᵀ`, `G = diag(I − ½ ad(δξ̂), 1, 1)` |

Timing (`robotics_filters::fusion` module docs): measurements are integrated by their own
stamps; zero stamps and non-finite values are rejected; measurements older than
`max_measurement_delay` are rejected as out of order, newer late ones are applied without
propagation; gaps above `max_dt` inflate the covariance instead of integrating across
them. Without a fresh gyro the filter falls back to wheel-only propagation and reports
`DEGRADED`.

Other Phase 2 estimators in the same crate (library only, no node yet): generic `Ekf` /
`Ukf` over any `Manifold`, the linear `KalmanFilter`, the `InformationFilter` (canonical
form), the `RtsSmoother`, `ParticleSet` with low-variance / multinomial resampling and
ESS, and KLD-adaptive sizing (`kld`). Consistency tools: `consistency::{nees, nis,
ConsistencyStats}`.

## 3. Parameters

Declared read-only with ranges and descriptions (`ros2 param describe`); defaults in
`robotics_bringup/config/state_estimation.yaml`. Groups: `frames.*`, `topics.*`
(`wheel_odom`, `imu`, `output_odom`, `state_estimate`), `publish_tf`,
`process_noise.*` (gyro / acceleration densities, bias random walk, slip diffusion),
`wheel_noise.*` (speed σ, yaw-rate σ₀ + per-speed + per-turn terms,
`bias_update_max_speed`, wheel-only yaw growth), `imu_noise.*`, `initial_covariance.*`,
`max_dt`, `max_measurement_delay`, `imu_timeout`, `wheel_timeout`,
`max_publish_rate_hz`, `diagnostics_period_s`.

TF: this node is the **only** owner of `odom → base_link`; the raw
`wheel_odometry_node` runs with `publish_tf:=false` in the Phase 2 launch. The
integration test checks that every TF child frame has exactly one publisher.

## 4. Validation

### 4.1 Offline regression (no ROS)

`cargo run --release -p robotics_sim --example estimator_accuracy` replays the ROS
pipeline in-process (same conversions, no DDS) on the deterministic simulator: mixed
trajectory with straight runs, turns in place, left/right arcs, a spin across ±π and
stops; slip σ = 2 % per wheel per step, right wheel radius +0.2 %, gyro 0.005 rad/s/√Hz,
bias 0.005 rad/s plus random walk. 180 s per seed, recorded 2026-09-27:

| seed | raw pos RMSE (m) | raw yaw RMSE (rad) | fused pos RMSE (m) | fused yaw RMSE (rad) | bias true / est (rad/s) | yaw in ±3σ | NEES raw | NEES fused | NIS (2 DoF) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.710 | 0.2564 | 0.152 | 0.0476 | 0.00584 / 0.00561 | 100 % | 185.24 | 0.45 | 0.90 |
| 2 | 0.700 | 0.2433 | 0.317 | 0.1475 | 0.00565 / 0.00628 | 100 % | 176.28 | 2.29 | 0.95 |
| 3 | 0.923 | 0.3123 | 0.175 | 0.0516 | 0.00392 / 0.00457 | 100 % | 274.14 | 0.50 | 0.97 |
| 4 | 0.687 | 0.2264 | 0.255 | 0.0659 | 0.00472 / 0.00520 | 100 % | 145.37 | 0.95 | 0.96 |
| 5 | 0.887 | 0.3085 | 0.108 | 0.0347 | 0.00479 / 0.00563 | 100 % | 291.30 | 0.38 | 0.98 |

With the bias updated at every speed (`bias_update_max_speed = ∞`) the fused errors
grow to 0.62–0.79 m / 0.19–0.27 rad and the yaw ±3σ coverage drops to 42–89 %: the
correlated radius error leaks into the bias estimate, which is why the default restricts
the bias update to near-standstill.

Reading the numbers:

* **Fused beats raw on every seed**, for heading by 2–9× (the gyro observes yaw rate
  directly; the wheels carry a systematic radius error and slip). This is asserted by
  `tests/estimation_pipeline.rs` for the default seed, where it is physically justified;
  position is not asserted to improve because the filter has no absolute position
  measurement.
* **NEES (3-DoF pose, expected 3):** the fused covariance is conservative (0.4–2.3). The
  raw wheel-odometry covariance (Phase 1 `OdometryNoise`, which models neither slip nor
  the radius error) is over-confident by two orders of magnitude — do not feed it to a
  consumer that trusts covariances.
* **NIS:** mean 0.9–0.98 for the 2-DoF wheel+gyro correction (expected 2) and 0.024 for
  the 1-DoF speed-only correction (default seed, expected 1): the configured wheel speed
  noise (`wheel_noise.speed_sigma = 0.02 m/s`) is conservative for this simulator. It is
  left conservative on purpose (real wheels slip more than the simulator's white noise);
  the live node reports both means on `/diagnostics` (`nis_mean_speed_1dof`,
  `nis_mean_wheel_gyro_2dof`).
* Noise-free sensors: raw position RMSE 1.8·10⁻⁹ m; fused heading RMSE 2.7·10⁻⁸ rad; fused
  position RMSE 5.7 mm, the known one-wheel-sample speed latency (§6).

### 4.2 Live ROS 2

`scripts/verify_phase2_runtime.sh` (CLI evidence in `README.md`) and
`scripts/integration_test_phase2.sh` (56 automated checks). Final run (2026-09-27, seed
1, ≈190 s of simulated driving):

| | raw wheel | fused (ESKF) |
|---|---|---|
| live `estimator_evaluator`, position / yaw RMSE | 0.775 m / 0.2706 rad | 0.159 m / 0.0514 rad |
| offline replay, same seed, 180 s | 0.710 m / 0.2564 rad | 0.152 m / 0.0476 rad |
| live mean NIS on `/diagnostics` | – | 0.918 (2 DoF, n = 1429), 0.022 (1 DoF, n = 4101) |

The raw stream is bit-identical to the offline replay (0.710 m at the 180 s report); the
fused stream differs slightly from run to run (0.151–0.159 m in the two final runs)
because message arrival order over DDS is not deterministic (§6). The integration test's
10-s window matched 502 of 502 `/odom` messages to ground truth by stamp, RMSE 6.4 mm
(start of the trajectory).

## 5. Consistency tooling

`robotics_filters::consistency`:

* `nees(e, P)`, `nis(r, S)` — quadratic forms by Cholesky solve.
* `ConsistencyStats` — running mean with a two-sided `χ²_{n·dof}/n` acceptance interval
  (`robotics_core::prob::chi2_mean_interval`; exact χ² quantiles for 1–2 DoF,
  Wilson–Hilferty otherwise). Successive filter outputs are correlated, so the interval
  is indicative, not a formal test.
* `OdomImuFusion::last_nis()` exposes the NIS and DoF of each wheel correction.

## 6. Known limitations

* **No absolute measurements.** Position drifts with slip; heading drifts with the gyro
  bias random walk. Absolute corrections (`EskfSe2::update_position`,
  `update_heading`) exist in the library and are consumed by Phase 3 localization.
* **Speed latency.** Wheel odometry reports the average speed over the last 20 ms
  interval; at a commanded speed step the fused position lags by about `Δv · 20 ms`
  (≈ 6 mm for 0.3 m/s), visible in the noise-free test.
* **Planar only.** Roll/pitch rates and accelerations are ignored; the IMU is assumed
  aligned with `base_link` (the simulator publishes an identity `base_link → imu_link`).
* **rclrs 0.8 + CycloneDDS threading.** With rclrs `Worker`s every worker gets its own
  wait-set thread, and such processes were observed (2026-09-26, logs in
  `~/phase2_runtime/stall*.log`, `matrix_*.log`) to stall for seconds whenever another DDS
  participant — e.g. a `ros2 topic hz` CLI — joined or left; a minimal worker-based talker
  reproduced it and the same talker built from node-level primitives did not. All
  framework nodes therefore use node-level callbacks with state in
  `robotics_ros_bridge::shared::Shared`, and `robotics_ros_bridge::shutdown::spin`
  resumes spinning after spurious `RCL_RET_TIMEOUT` returns of `rcl_wait` (logged,
  rate-limited). Neither symptom occurred in the final Phase 2 runs.
* **Late samples are rejected, not re-ordered.** The filter keeps no measurement buffer:
  a sample older than `max_measurement_delay` (50 ms) behind the filter time is dropped
  and counted (`rejected_out_of_order`), and the node reports `DEGRADED: invalid_timestamp`
  for `fault_hold` (2 s). In the final runtime run one wheel-odometry sample reached the
  estimator 0.42 s late while `ros2` CLI processes were joining and leaving the DDS graph
  (`rejected wheel measurement: out-of-order measurement: 0.42 s older than the filter
  state`); the node recovered to `RUNNING` two seconds later. The same event appears in
  the pre-Phase-2-closure log of 2026-09-26; the integration test (no CLI churn) saw none.
  A small re-ordering buffer is a possible later improvement; the current policy keeps the
  filter causal and bounded in latency.
* **Evaluation matches by stamp.** `estimator_evaluator` pairs estimates with ground truth
  of the same stamp (±6 ms); a handful of samples at startup are unmatched.
