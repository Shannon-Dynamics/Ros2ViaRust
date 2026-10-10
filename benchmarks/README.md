# Benchmarks

Every phase of ["ROS via Rust"](../docs/README.md) ends with the same question: the Rust
implementation of a component against the Python and C++ implementations everyone already
uses, doing the same job, under conditions identical enough that the comparison means
something. This directory holds the method and the raw results. The prose in the phase
documents interprets those results and never contains the numbers itself, so a rerun
updates one data file instead of a paragraph nobody remembers to edit.

> **TODO: no benchmark has been run yet.** The result files in this directory contain
> their column header and nothing else. No number anywhere in this repository is a
> placeholder, an estimate or an illustration; a number appears only after a run produced
> it.

## 1. Rules

1. **Same world, same robot, same driver.** Every entry runs against
   [`sim/webots`](../sim/webots/README.md): the same world file, the same TurtleBot3
   Burger, the same official `webots_ros2_turtlebot` driver, the same `/scan`, `/odom` and
   `/imu` streams. Only the node under test changes.
2. **Same input sequence.** Where an implementation can be fed a recording instead of a
   live simulation, it is, so that every entry sees the identical measurements in the
   identical order.
3. **Same parameters where they exist.** Algorithm parameters are matched one by one.
   Anything that cannot be matched is recorded as a difference in the result row, not
   ignored.
4. **Repeat over seeds.** A single run is an anecdote. Every configuration is run over a
   fixed set of seeds and every seed produces a row, so that the spread is visible.
5. **Report the tail.** Latency is reported as median, 95th and 99th percentile. A mean
   latency hides exactly the behaviour that matters in a control loop.
6. **Accuracy next to speed, always.** A latency number is only published together with
   the error metric from the same run. Fast and wrong is not a result.
7. **Record the environment.** Host, CPU, kernel, ROS distribution, Webots version, rustc
   version, RMW implementation and whether the run was headless go in the row, because a
   number without them cannot be compared with anything later.
8. **Written by a script.** Result files are produced by the benchmark script of that
   phase. Nobody edits them by hand, and a row nobody can reproduce gets deleted.

## 2. Files

| File | Phase | State |
|---|---|---|
| [`phase-3-localization.csv`](phase-3-localization.csv) | [Phase 3, localization](../docs/phase-3-localization/benchmark.md) | header only, **TODO** |

Later phases add one file each, in the same shape: `phase-4-mapping.csv`,
`phase-5-slam.csv` and so on. CSV is the default; a phase whose results are not flat
(per-step traces, histograms) may use JSON instead, with the same field names.

## 3. Row format

One row per implementation per seed per configuration:

```text
phase, entry, language, seed, config, metric, value, unit,
host, cpu, kernel, ros_distro, webots_version, rustc_version, rmw, headless, timestamp, commit
```

* `entry` is the implementation under test, for example `rust_amcl`, `nav2_amcl`,
  `rclpy_amcl`.
* `config` names the parameter set, so that two rows of the same entry with different
  particle counts stay distinguishable.
* `metric` and `value` are long format on purpose: adding a metric later does not break the
  file or require backfilling columns.
* `commit` is the revision the measured code was built from.

## 4. Running them

**TODO.** There is no benchmark script yet. It arrives with the first phase that has
something to measure, as `scripts/benchmark_phase3.sh` alongside the existing
`scripts/verify_phase2_all.sh`, and this section then documents how to run it and roughly
how long it takes.
