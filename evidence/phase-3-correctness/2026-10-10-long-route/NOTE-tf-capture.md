# `normal-tf.txt` in this directory is empty, and the TF tree is not

The measurement in this directory is complete and its verdict stands. One side capture in it
does not: `normal-tf.txt` holds four times `Terminated` and no transform.

That is a lost capture, not a missing transform, and two attempts to fix it failed. Raising
the `timeout` on `tf2_echo` from 6 s to 20 s changed nothing. Exporting
`RCUTILS_LOGGING_BUFFERED_STREAM=0` and `PYTHONUNBUFFERED=1`, which is what makes the same
call work in `scripts/verify_phase3_localization.sh`, also changed nothing here. The
remaining difference between the two scripts is the `set -m` this one needs so that the
SIGINT it sends to the launch arrives at all, and under job control the output of a
`tf2_echo` ended by `timeout` does not survive.

Rather than guess a third time at a capture that was redundant anyway, it was removed from
`scripts/run_phase3_correctness.sh`. Later runs will not have a `<scenario>-tf.txt` at all.
The reasoning is in a comment where the block used to be: this script measures accuracy, and
whether the TF tree is complete is a property of the launch.

Where the TF evidence actually is:

* `normal-localization_node.log` in this directory records `map -> odom published` at
  startup, and `normal-diagnostics.yaml` carries the node's own `tf_sent` count.
* `scripts/verify_phase3_localization.sh` checks the tree edge by edge, and composes it:
  `map -> odom`, `odom -> base_link`, `base_link -> LDS-01`, `base_link -> imu_link`,
  `base_link -> base_footprint`, and then `map -> base_link`, `map -> LDS-01` and
  `map -> imu_link` through the chain. Those are checks F1 to F10 and all of them pass. Its
  run of the same day, with the resolved transforms printed in `tf.txt`, is under
  `../../phase-3-localization-runtime/2026-10-10/`.

## One more thing a reader will notice

The first line of `normal-run.log` is
`FAIL: a webots-bin process is already running, stop it first`, from an attempt that aborted
immediately before the run recorded here. `log()` appends, so the line stayed. The guard was
right: an earlier integration test had been killed part way through and its launch, started
with `setsid`, outlived it and kept holding the controller port. That is now handled by a
cleanup trap in the test, and the measurement below it is the next attempt, which started
from a clean machine.
