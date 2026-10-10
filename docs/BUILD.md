# Build

## Environment

| Item | Verified value |
|---|---|
| OS | Ubuntu 24.04 in WSL2 (distro `Ubuntu-24.04`) |
| ROS 2 | Jazzy (`ros-jazzy-desktop-full`), `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Rust | stable 1.97.1 (`rustup`), edition 2021, MSRV 1.85 |
| colcon plugins | `colcon-cargo 0.2.0`, `colcon-ros-cargo 0.2.0`, `cargo-ament-build 0.1.11` |
| rclrs | 0.8.0 (crates.io), `ros-env` 0.3.0, `rosidl_runtime_rs` 0.7.0 |
| libclang | 18 (needed by rclrs's bindgen) |

Windows/WSL layout: the git repository is the Windows path
`D:\Shannon\ROS2 via RUST\ros2-rust-robotics`. Builds run on the Linux filesystem in
`~/ros2-rust-robotics`, a mirror refreshed by `scripts/sync_to_wsl.sh` (rsync, excludes
`target/`, `ros2_ws/{build,install,log}`). Nothing is compiled under `/mnt/d`.

## One-time: interface underlay with Rust bindings

rclrs 0.8 obtains message types through the `ros-env` crate, which textually includes the
Rust crates that `rosidl_generator_rs` generated into any prefix on `AMENT_PREFIX_PATH`.
The apt distribution has no such crates, so the standard interface packages are rebuilt
once into `~/ros2_rust_interfaces_ws`:

```bash
sudo apt install -y git libclang-dev python3-pip python3-vcstool
pip install --break-system-packages colcon-cargo colcon-ros-cargo
cargo install cargo-ament-build
scripts/setup_interfaces_underlay.sh          # ~5 min; builds rcl_interfaces, rosgraph_msgs,
                                              # std/geometry/sensor/nav/diagnostic/visualization
                                              # msgs, tf2_msgs, example_interfaces
```

`rcl_interfaces` and `rosgraph_msgs` are mandatory: rclrs itself uses them for parameter
services and `/rosout`. Without them the crate fails to compile (48 unresolved types in
`rclrs/src/parameter/service.rs`).

## Every session

```bash
source scripts/env.sh        # ROS 2 Jazzy + underlay + cargo + (if built) ros2_ws/install
```

## Pure Rust libraries (no ROS needed)

```bash
cargo test                   # crates/robotics_core, robotics_models, robotics_filters, robotics_sim
cargo clippy --all-targets
cargo doc --no-deps --open
```

## ROS 2 packages

```bash
source scripts/env.sh
cargo build --workspace      # quick check of everything incl. rclrs nodes
cargo test  --workspace      # + converter/QoS unit tests in robotics_ros_bridge

cd ros2_ws
colcon build                 # ament_cargo + ament_cmake packages → ros2_ws/install
source install/setup.bash
ros2 launch robotics_bringup pubsub_demo.launch.xml
ros2 launch robotics_bringup wheel_odometry_demo.launch.xml
```

`colcon build` compiles each `ament_cargo` package in its own `build/<pkg>` target
directory (a known colcon-ros-cargo limitation: rclrs is compiled once per package). Use
`cargo build --workspace` for fast iteration and `colcon build` when you need `ros2 run`
/ `ros2 launch` to find the binaries.

Packages: `robotics_ros_bridge` (lib), `robotics_examples`, `robotics_estimation`,
`robotics_sim_bridge`, `robotics_integration` (ament_cargo), `robotics_bringup`
(ament_cmake).

## Phase 2 stack, tests and verification

```bash
ros2 launch robotics_bringup estimator_sim.launch.xml            # sim + wheel odometry + ESKF (+ evaluator)
ros2 launch robotics_bringup estimator_sim.launch.xml seed:=3 evaluate:=false

bash scripts/gates.sh                      # fmt --check, clippy -D warnings, cargo test --workspace, colcon build
bash scripts/integration_test_phase2.sh    # Rust launch_testing equivalent: spawn, 56 checks, SIGINT, clean exit
bash scripts/verify_phase2_runtime.sh 60   # ROS CLI evidence (node/topic lists, hz, info -v, params, tf2_echo, Ctrl-C)
bash scripts/verify_phase2_all.sh          # all of the above in order, summary in ~/phase2_final/summary.log
cargo run --release -p robotics_sim --example estimator_accuracy   # offline RMSE / NEES / NIS table
```

The integration test and the runtime script kill stale `robotics_*` node processes first
and write their logs to `~/phase2_integration/` and `~/phase2_runtime/` (not `/tmp`,
which WSL clears when the distro stops).

## Working from Windows

The repository is edited on Windows; every build and run happens in the WSL mirror
through scripts (no long inline `wsl bash -lc` commands):

```powershell
$S = "/mnt/d/shannon/ROS2 via RUST/ros2-rust-robotics/scripts"
wsl.exe -d Ubuntu-24.04 -- bash "$S/wsl_run.sh" cargo test               # sync + env + command
wsl.exe -d Ubuntu-24.04 -- bash "$S/wsl_run.sh" bash scripts/gates.sh
wsl.exe -d Ubuntu-24.04 -- bash "$S/fmt_sync_back.sh"                    # cargo fmt in the mirror, copy .rs + Cargo.lock back
```

`sync_to_wsl.sh` mirrors Windows → WSL with `rsync --delete` (excluding `target/` and the
colcon spaces); only formatted sources and `Cargo.lock` ever flow back.

## Troubleshooting

* `cannot find type SetParametersResult` while compiling rclrs → the underlay lacks
  `rcl_interfaces`; rerun `scripts/setup_interfaces_underlay.sh`.
* `Set ROS_DISTRO or use ROS shim` → the environment is not sourced.
* `error while loading shared libraries: librcl.so` at run time → source
  `/opt/ros/jazzy/setup.bash` in the terminal that runs the binary.
* `ros2 topic echo` shows nothing on a BEST_EFFORT topic (`/scan`, `/imu/data`,
  `/joint_states`) → pass `--qos-reliability best_effort` to the CLI.
* `tf2_echo base_link laser` prints `At time 0.0`: expected for a static transform.
* A test reports two publishers on `/tf` → another process (e.g. a Phase 1 demo with
  `publish_tf:=true`) still broadcasts `odom → base_link`; run
  `pkill -f "lib/robotics_"` or re-run the script (it cleans up first).
* `rcl_wait returned a spurious timeout ... resumed spinning` warnings are informational
  (CycloneDDS graph changes; see `docs/STATE_ESTIMATION.md` § 6).
