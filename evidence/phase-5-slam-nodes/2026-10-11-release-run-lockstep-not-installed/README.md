# Second verification run (release binaries), superseded

Run of `scripts/verify_phase5_slam_nodes.sh` with the release build of `robotics_slam_ros`: 45 passed, 12 failed.
Kept because it is a real run, but its failures are of the harness, not of the nodes:

- **C recorded 1, 3, 4.** The installed `phase5_slam.launch.py` predated the `lockstep` argument, so the
  replayer ran without lockstep (`lockstep false` in each launch.log) and published at 200 Hz faster than a
  lost front end could register; the node saw 325/1312, 352/454 and 497/527 scans. On the scans it did
  register, recorded 1 and 4 equal the reference row for row; wall-lobes differs from row 197, where a scan
  is missing from the node's input.
- **A launch file installed.** The check looked in `install_release`; the launch file is in the debug underlay.
- **D (7 checks).** Sampled during the first lockstep stream, which with release binaries ended before the 8 s sample.
- **F shutdown lines.** An unquoted path with spaces in the check; every launch.log has both shutdown lines.

Synthetic streams 6/6 each (including E1), recorded 2 5/5, real time 568/568 and 527/527, E determinism: pass.
