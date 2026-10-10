# First look, kept because it is a configuration that was tried

Run by hand before `scripts/run_phase3_global_synthetic.sh` existed, so there is no
`environment.txt`. The command was:

    target/release/examples/global_trials trials --map sim/webots/config/landmarks.yaml \
        --particles 6000 --trials 10 --seed 1 --label smoke-6000 --out <dir>

Everything else is the default of `global_trials.rs` at that time (starts seed 1000,
convergence sigma 0.05 m and 0.05 rad sustained over 10 scans, detection noise 0.05 m and
0.05 rad). Read `summary.yaml` for the outcome; it is the reason the particle count sweep
that follows exists.
