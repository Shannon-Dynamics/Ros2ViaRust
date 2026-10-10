# Simulation (Rusty, Phase 2)

`crates/robotics_sim` is the deterministic simulator of the book's Chapter 4 ("Rusty,
Sensors, and the Simulator"); `ros2_ws/src/robotics_sim_bridge/src/sim_node.rs` publishes
it on standard ROS topics. Nothing in the crate depends on ROS.

## 1. What is simulated

| Component | Model | Rate (default) | Source |
|---|---|---|---|
| Kinematics | differential drive, exact constant-curvature arc per physics step | 200 Hz physics | `rusty.rs` |
| Wheel slip | per-wheel multiplicative `Δs_i = r_i ω_i Δt (1 + ε_i)`, `ε_i ~ N(0, 0.02²)` | per step | `rusty.rs` |
| Calibration error | true right wheel radius `r(1 + 0.002)` (the software does not know) | – | `rusty.rs` |
| Encoders | cumulative, quantized to 4096 ticks/rev, published as joint positions | 50 Hz | `rusty.rs` |
| Gyro | average true yaw rate over the sample interval + bias (random walk) + white noise | 100 Hz | `rusty.rs`, `robotics_models::sensor::gyro` |
| **LiDAR** | 360 beams over 2π (1° steps), exact ray casting, `N(0, 0.02²)` range noise, 1 % dropouts, 0.12–8 m | 10 Hz | `lidar.rs` |
| **World** | static line segments: walls, boxes, polylines | – | `world.rs` |

Commands come from a scripted, optionally repeating `Trajectory` (`Trajectory::mixed`:
straight runs, turns in place, left/right arcs, a spin across ±π, stops; ≈ 60 s per lap)
or, with `use_cmd_vel:=true`, from `/cmd_vel`.

## 2. Determinism

* Fixed physics step; sensor rates must divide the physics rate (checked at
  construction, `SimError::RateMismatch`).
* One seeded `SmallRng` **per noise source**, derived from the master seed with a
  SplitMix64 finalizer: 1 = wheel slip, 2 = gyro white noise, 3 = gyro bias walk,
  4 = LiDAR. Enabling, disabling or reconfiguring the LiDAR does not change a single
  wheel or IMU sample of a given seed (`lidar_does_not_perturb_the_other_noise_streams`).
* Every LiDAR beam draws its uniform (dropout) and normal (range noise) numbers whether
  or not it uses them, so changing the dropout rate does not reshuffle the noise of the
  surviving beams (`dropout_does_not_reshuffle_surviving_noise`) — the book's RNG
  discipline.
* Same seed ⇒ bit-identical `SimStep` sequence, scans included
  (`deterministic_replay`, `simulator_scans_replay_bit_exactly`).
* In `sim_node`, simulation time is anchored to the node clock at start-up and advanced
  in whole physics steps to catch up with the clock, so stamps are `start + k·Δt`
  exactly and the data sequence is independent of scheduling jitter.

## 3. LiDAR forward model

```text
T_sensor = x_t ∘ T_base_laser                  (T_base_laser = (0.05, 0, 0) by default)
for k in 0..n:   φ_k = angle_min + k·angle_increment
    z*_k = World::raycast(T_sensor.origin, T_sensor.θ + φ_k, range_max)   (exact, analytic)
    u ~ U[0,1), n ~ N(0,1)                                                (always both)
    no return  if u < p_drop or z*_k = none
    else z_k = z*_k + σ_r·n
```

Encoding of special ranges (`NoReturnPolicy`):

| Case | `Infinity` (default, REP 117) | `MaxRange` (book) |
|---|---|---|
| no hit within `range_max`, dropout, noisy `z > range_max` | `+inf` | `range_max` |
| return closer than `range_min` | `−inf` | clamped to `[0, range_max]` |

The sweep is instantaneous (`time_increment = 0`, `scan_time = 1/rate`): every beam sees
the pose at the scan stamp, which is also the stamp of the ground truth, joint state and
IMU sample of the same physics step.

The internal type is `robotics_core::LaserScan` (`stamp`, `frame`, `angle_min`,
`angle_increment`, `time_increment`, `scan_time`, `range_min`, `range_max`, `ranges`,
optional `intensities`; `angle_max()` derived). `LaserScan::validate` enforces finite
metadata, non-zero increment, at most one turn, `0 ≤ range_min < range_max`,
non-negative timing and consistent intensity length; `robotics_ros_bridge::convert::
laser_scan_{to,from}_ros` converts losslessly and additionally rejects messages whose
`angle_max` disagrees with `angle_min + (n−1)·angle_increment` by more than one
increment.

## 4. Worlds

`World` is a set of segments with `add_segment`, `add_polyline`, `add_rectangle`
(axis-aligned outline) and `add_box` (rotated). `raycast` intersects the ray with every
segment analytically (`t = (a−o)×e / d×e`, `s = (a−o)×d / d×e`, hit if `t ≥ 0` and
`s ∈ [−ε, 1+ε]`), so rays through a shared corner cannot leak. `clearance(p)` gives the
distance to the nearest segment.

`World::rusty_arena()` (the `sim_node` default, parameter `world`):

```text
 y=9 ┌──────────────────────────────────────────────┐
     │▪ pillar (−4.8, 8.3)                  ■ box    │
     │                                  (6.3–7.0,    │
 5.5 ├── stub                            7.8–8.5)    │
     │          repeated mixed trajectory            │
     │          x ∈ [−3.8, 5.6], y ∈ [−2.2, 7.2]      │
     │                 start (0,0) → +x              │
     │■ box (−5.0…−4.2, −3.5…−2.7)          ◆ box    │
y=−4 └──────────────────────────────────────────────┘
    x=−5.5                                        x=7.5
```

It is **not** a collision model: the robot is never stopped by geometry. The arena is
designed around the trajectory (the repeated lap is a net rotation of 0.854 rad, so the
path orbits inside a bounded region) and `rusty_arena_is_clear_of_the_trajectory`
checks ≥ 0.5 m clearance over 20 noise-free minutes and 10 noisy minutes. From the
origin the left wall (9 m) is beyond the 8 m range, so scans always contain `+inf`
returns — intentionally exercising the no-return path.

`world:=empty` gives a world where every beam is a no-return.

## 5. ROS interface (`sim_node`)

| Output | Type | QoS | Frame |
|---|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | sensor data | `base_link` |
| `imu/data` | `sensor_msgs/Imu` | sensor data | `imu_link` |
| `scan` | `sensor_msgs/LaserScan` | sensor data (best effort, keep last 5) | `laser` |
| `ground_truth/odom` | `nav_msgs/Odometry` (evaluation only) | odometry | `odom` → `base_link_ground_truth` |
| `/tf_static` | `base_link → imu_link` (identity), `base_link → laser` (mount) in **one** message | tf_static (transient local) | – |

`sim_node` never publishes `odom → base_link`. LiDAR parameters: `lidar.enabled`,
`lidar.rate_hz`, `lidar.beams`, `lidar.angle_min`, `lidar.angle_max`, `lidar.range_min`,
`lidar.range_max`, `lidar.range_sigma`, `lidar.dropout_probability`, `lidar.mount_x`,
`lidar.mount_y`, `lidar.mount_yaw`, `frames.laser`, `world`.

## 6. Tests

`crates/robotics_sim`: room geometry (front/rear/side walls, rotated robot, mounted
sensor), no-hit → `+inf` / `range_max`, too close → `−inf`, noise-free default scan equal
to the ideal ray cast and every endpoint on a wall, seeded noise reproducible with the
configured σ and dropout rate, dropout-independent noise, invalid configurations, scans
stamped with the ground truth, arena clearance, and `tests/lidar_regression.rs`
(hand-computed arena distances at two poses, bit-exact scan replay).

## 7. Dependencies

No new dependency. The book's design uses `parry2d`'s BVH; the Phase 2 arena has 21
segments, for which the analytic intersection over all segments (7 560 segment tests
per 360-beam scan, 10 Hz) is cheaper than maintaining a BVH, and `parry2d` would pin its
own `nalgebra` version against the workspace's 0.35. Revisit when maps become large
(occupancy-grid ray casting arrives with Phase 4 mapping).
