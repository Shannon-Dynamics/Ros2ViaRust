# The EKF has no kidnap recovery, by design: a known limitation, not a gap

`localization_node` (the EKF, `crates/robotics_localization/src/ekf.rs`) contains no kidnap
recovery code and this phase does not add any. In every `trial-NN/kidnap-verdict-ekf.yaml`
of this directory it is `DID_NOT_RECOVER`; `kidnap-trials-summary.yaml` counts it.

Why it is expected, not an omission:

* A single Gaussian is unimodal. After a teleport the observations no longer fit the belief,
  the chi-square gate rejects them, and the estimate stays where it was. This is a property
  of the representation, not a tuning failure (`docs/phase-3-localization/theory.md`,
  the "Linearization is a local approximation" bullet).
* A unit test pins the behaviour so that it cannot change unnoticed:
  `ekf::tests::a_teleported_robot_is_not_recovered_by_a_single_gaussian`.
* `docs/phase-3-localization/problem.md` accepts "recovers, or reports that it cannot" for
  the P2 recovery row; the EKF is the "cannot" half of that comparison, and MCL is the half
  that answers it.

What the EKF numbers in this directory are therefore for: they show the teleport really
happened and really invalidated a filter that cannot recover, in the same simulation
session as the MCL it is compared against. They are not a failed attempt at recovery.
