# Phase 3 global localization runs

Raw output of the global localization experiments: a filter that starts from a uniform prior
over the free space, with no initial pose, and has to find itself. The problem is
[`docs/phase-3-localization/problem.md`](../../docs/phase-3-localization/problem.md) section 6;
the results, as tables generated from these files, are
[`docs/phase-3-localization/benchmark.md`](../../docs/phase-3-localization/benchmark.md)
section 7.

No number is retyped from a file here, and no run directory was edited after the run wrote
it. A run directory is never overwritten: the scripts refuse to start if it exists, so a
configuration that failed is still here next to the one that replaced it. Runs that were
stopped or invalidated carry a `NOTE-*.md` that says why.

## Model-level runs (`global_trials`)

One directory per run, one sub-directory `p<N>` per global particle count. Each holds
`trials.csv` (one row per trial), `summary.yaml` (computed from it), `command.txt` (the exact
command) and `run.log`; the run directory holds `environment.txt` (host, compiler, the SHA-256
of the binary and the map). Made by
[`scripts/run_phase3_global_synthetic.sh`](../../scripts/run_phase3_global_synthetic.sh).

| Run | What it is for |
|---|---|
| `2026-10-10-smoke-6000-first-look` | ten trials at the steady-state count, by hand, before the script existed; the reason the rest exists |
| `2026-10-10-sweep-particles-default-criterion` | 6000 to 96000 particles, the upper-median convergence rule, 40 development start poses |
| `2026-10-10-sweep-particles-large-default-criterion` | 192000 and 384000, same |
| `2026-10-10-sweep-particles-explained-0.51`, `...-explained-1.0` | a stricter convergence rule, 6000 to 96000, the same poses |
| `2026-10-10-sweep-particles-large-explained-1.0` | 192000 and 384000, strict |
| `2026-10-10-confirm-fresh-starts-explained-0.5`, `...-1.0` | 100 start poses drawn with a different seed, that played no part in choosing any setting |
| `2026-10-10-kidnap-model-existing-tuning` | the tracking filter's kidnap recovery as it ships, many kidnaps (`kidnap_trials.csv`) |
| `2026-10-10-kidnap-model-whatif-48000` | the same at a larger steady-state count, a labelled what-if |

## End-to-end runs in Webots

One directory per run, one `trial-NN` per start pose. Each trial holds the node log with the
filter's own claims, the evaluator's per-sample CSV against ground truth, the route that was
driven, the diagnostics at the end, and `verdict.yaml`; the run holds `summary.yaml`,
`environment.txt` and `starts.csv`. Made by
[`scripts/run_phase3_global_trials.sh`](../../scripts/run_phase3_global_trials.sh), graded by
[`scripts/phase3_global_verdict.py`](../../scripts/phase3_global_verdict.py).

| Run | What it is for |
|---|---|
| `2026-10-10-webots-smoke-1-trial` | the one-trial smoke test of the runner, kept |
| `2026-10-10-webots-6000-debug-explained-0.5` | the steady-state count, the default build, the lenient rule: the configuration as it stood |
| `2026-10-10-webots-48000-release-explained-1.0` | a larger search in the release build, the strict rule |
| `2026-10-10-webots-96000-release-explained-1.0` | larger still |

## Inputs and analysis

| Directory | What it is |
|---|---|
| `2026-10-10-starts` | every start pose set used, as files: the 40 development poses, the 100 fresh poses, the 20 used in Webots, and the ambiguity-targeted ones with the reasons they were picked |
| `2026-10-10-ambiguity` | `ambiguity.csv`, the static ambiguity ranking of start poses, and `map-symmetry.yaml`, how close the map is to being rotationally symmetric |

The integration test of this work is under
[`../phase-3-localization-runtime/`](../phase-3-localization-runtime/), in the directories whose
names end in `global-localization`.

## Where the evidence is written, and why it is safe

Every script that produced these writes straight to the Windows checkout through
`scripts/evidence_root.sh`, and `scripts/sync_to_wsl.sh` excludes `evidence/` from its
`--delete`, so the sync that has destroyed runs before cannot reach them. See
[`../phase-3-correctness/README.md`](../phase-3-correctness/README.md).
