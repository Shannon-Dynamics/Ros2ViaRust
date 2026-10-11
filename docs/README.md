# ROS via Rust

**ROS via Rust** is the problem-based companion material of this repository. Every part
opens with a concrete problem a real robot has, states it precisely enough to be
measurable, then derives only as much theory as that problem needs, and ends in Rust code
that runs as a node in a live ROS 2 graph. The order follows *Probabilistic Robotics*
(Thrun, Burgard, Fox): the question comes first, the estimator second, the implementation
third. This is not a separate book or a website, it is plain markdown inside the repo,
read on GitHub, with relative links into the actual crates and nodes that implement each
idea.

Problem first, because the opposite order teaches the wrong lesson. A particle filter
introduced as a list of equations looks arbitrary; the same filter introduced after you
have watched wheel odometry drift two metres across a room is the obvious answer to a
question you already have. Each part therefore closes the loop it opened: the problem is
restated as a number, measured in simulation, and compared against the reference
Python/C++ implementation of the same component (see
[../benchmarks/README.md](../benchmarks/README.md)).

One robot carries the whole narrative: the **TurtleBot3 Burger**, simulated in
**Webots** through the official `webots_ros2_turtlebot` package from Cyberbotics and
ROBOTIS. It is a differential-drive base with wheel encoders, an IMU and a 360 degree
LDS-01 lidar, which is the smallest sensor set that makes localization, mapping, SLAM,
planning and control all non-trivial. That driver is Python and C++ and is used exactly
as published: it is simulation infrastructure, not a porting target. Rust enters one
level above it, in the consumer nodes that subscribe to the standard topics the driver
publishes. The in-repo simulator `crates/robotics_sim` is not replaced by Webots and is
not competing with it: it stays the fast, seeded, headless world used by unit tests of
filters and sensor models, while Webots is the shared arena where Rust and the reference
stacks are compared on equal terms. See [../sim/webots/README.md](../sim/webots/README.md)
for the split and for how to run the simulation.

## Parts

| Part | Status | Content |
|---|---|---|
| [Phase 3: localization](phase-3-localization/) | documented, implementation open | [problem](phase-3-localization/problem.md), [theory](phase-3-localization/theory.md), [implementation](phase-3-localization/implementation.md), [ROS graph](phase-3-localization/ros-graph.md), [benchmark](phase-3-localization/benchmark.md) |
| [Phase 4: mapping](phase-4-mapping.md) | planned | occupancy grids, distance fields |
| [Phase 5: SLAM](phase-5-slam.md) | planned | scan matching, pose graphs |
| [Phase 6: planning](phase-6-planning.md) | planned | search, sampling, decision making |
| [Phase 7: control](phase-7-control.md) | planned | pure pursuit, DWA, MPPI |
| [Phase 8: exploration](phase-8-exploration.md) | planned | frontiers, information gain |
| [Phase 9: full stack](phase-9-full-stack.md) | planned | supervisor, recovery, missions |

Phases 1 and 2 (foundation, state estimation and sensor models) predate this material and
are documented as engineering references rather than as problem-based parts:
[STATE_ESTIMATION.md](STATE_ESTIMATION.md), [SIMULATION.md](SIMULATION.md),
[TF_BUFFER.md](TF_BUFFER.md).

## Supporting documents

| Document | Content |
|---|---|
| [../sim/webots/README.md](../sim/webots/README.md) | Webots setup, what is Rust and what is not, how to run it |
| [../benchmarks/README.md](../benchmarks/README.md) | benchmark methodology and result files |
| [ROADMAP.md](ROADMAP.md) | the phased plan these parts follow |
| [ARCHITECTURE.md](ARCHITECTURE.md) | layers, crates, packages, TF ownership |
| [ROS2_INTERFACES.md](ROS2_INTERFACES.md) | topics, services, parameters, QoS |
| [BUILD.md](BUILD.md) | environment, underlay, build, troubleshooting |
| [../web/README.md](../web/README.md) | the interactive webbook of the closed phases: widgets run these crates compiled to WebAssembly |
| [SOURCE_MAPPING.md](SOURCE_MAPPING.md) | chapter to crate and node mapping |
