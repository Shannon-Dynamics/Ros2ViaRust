# TF in Rust: buffer and listener

`rclrs` 0.8 has no `tf2` / `tf2_ros` binding. The framework therefore implements the part
of TF it needs:

```text
 ROS: /tf (tf2_msgs/TFMessage, reliable, volatile, depth 100)
      /tf_static (reliable, transient local, depth 100)
        │ robotics_ros_bridge::tf::TfListener   — two node-level subscriptions
        ▼ convert::transform_stamped_from_ros   — validation + planar projection
 robotics_core::tf::TransformStamped2 { stamp, parent, child, transform: Se2 }
        ▼
 robotics_core::tf::TransformBuffer      — ROS-independent, unit tested with `cargo test`
        ▼
 lookup_transform(target, source, LookupTime::{Latest, At(t)}) → T_target_source
```

Broadcasting uses `robotics_ros_bridge::tf::TfBroadcaster` (`new` for `/tf`,
`new_static` for `/tf_static`).

## 1. Semantics

| Feature | Behaviour |
|---|---|
| Tree | each child has exactly one parent (a forest of trees) |
| `lookup_transform(target, source, t)` | `T_target_source`: pose of `source` in `target`, maps points from `source` into `target` (same as `tf2::BufferCore::lookupTransform`) |
| Direct, inverse, chained lookups | path through the lowest common ancestor; any depth (guard: 256 hops) |
| Static edges | one value, valid at every time |
| Dynamic edges | time-ordered history; out-of-order samples are sorted in, equal stamps replace |
| `LookupTime::Latest` | latest time at which **all** dynamic edges of the path have data (tf2 "time 0") |
| `LookupTime::At(t)` | each dynamic edge interpolated on the SE(2) geodesic `a ⊞ s·(b ⊟ a)` (heading interpolates across ±π correctly) |
| Extrapolation | refused (`TfError::Extrapolation`) unless within `extrapolation_tolerance` (default 0, like tf2) |
| Expiry | per edge, samples older than `newest − cache_duration` (default 10 s) are evicted on insert; `prune_before(t)` expires against an external clock and removes edges that become empty |
| Cycles | inserting `p → c` where `c` is an ancestor of `p` is rejected (`TfError::Cycle`) |
| Static/dynamic conflict | a child published on both topics is rejected (`StaticDynamicConflict`) |
| Re-parenting | a dynamic child that changes parent moves; its history is reset (`InsertOutcome::Reparented`) |
| Errors | `EmptyFrameId`, `SelfTransform`, `NonFinite`, `UnknownFrame`, `NotConnected`, `Extrapolation`, `Cycle`, `StaticDynamicConflict`, `InvalidParameter`, `ChainTooLong` |

## 2. Message conversion

`transform_stamped_from_ros` rejects: empty frame ids (a leading `/` is stripped as tf2
does), non-finite translation or quaternion, a zero-norm quaternion, and any rotation with
`|roll|` or `|pitch|` above `PLANAR_TOLERANCE` = 0.01 rad (`ConvertError::NonPlanar`).
The z translation is dropped (sensor heights do not change planar geometry). Rejections
are counted in `TfListenerStats` with the last reason; they never panic.

## 3. TF authority

`TfListener` records the DDS publisher GID (`MessageInfo::publisher_gid`) of every
accepted transform per child frame. `authority_conflicts()` lists child frames served by
more than one publisher — the situation tf2 only warns about ("TF_REPEATED_DATA",
jumping frames) when two nodes broadcast `odom → base_link`. The Phase 2 integration test
requires it to be empty.

## 4. Phase 2 TF tree

```text
odom ──(state_estimator_node, /tf, 50 Hz)──► base_link ──(sim_node, /tf_static)──► laser     (0.05, 0, 0)
                                                        └─(sim_node, /tf_static)──► imu_link  (identity)
```

`sim_node` sends both static edges in one message: with transient-local durability a
late subscriber receives only the last message of each publisher. `map → odom` is
reserved for Phase 3 localization.

## 5. Verification

* `robotics_core::tf` unit tests: direct and inverse lookup, identity, two-hop and
  three-hop chains, siblings through a common parent, downward chains, static edges at
  any time, static/dynamic conflict, static update, SE(2) interpolation across ±π,
  out-of-order insertion, extrapolation refusal and tolerance, latest-common-time,
  unknown frames, disconnected trees, cycles (including via re-parenting), invalid
  input, re-parenting, expiry on insert and by external clock.
* `robotics_ros_bridge::convert` tests: planar round trip, leading slash, dropped z,
  non-planar rejection, empty frame, NaN.
* Live (`robotics_integration/phase2_stack_test`): `odom → base_link` resolves, the
  static `base_link → laser` equals the configured mount, `odom → laser` equals the
  composition of the two edges, interpolation 0.5 s in the past, the frame set is
  exactly `{odom, base_link, laser, imu_link}`, and every child has one publisher.

## 6. Limitations

Planar only (a 3-D `Transform` buffer is out of scope until a 3-D consumer exists); no
blocking lookups with timeout (poll `can_transform` from a timer); no "time travel"
lookups with a fixed frame; no `tf2_ros::MessageFilter` equivalent (Phase 3 consumers
buffer scans until `can_transform` succeeds).
