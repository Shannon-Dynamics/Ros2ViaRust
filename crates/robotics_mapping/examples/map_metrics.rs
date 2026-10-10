//! Grade a saved grid against the analytic geometry of the world.
//!
//!     cargo run --release -p robotics_mapping --example map_metrics -- \
//!         <grid.txt> <landmarks.yaml> <positions.csv> <tau_cells> [out.yaml]
//!
//! * `grid.txt`: the text form written by `OccupancyGrid::to_text` (the `mapping_evaluator`
//!   node writes one at the end of a run).
//! * `landmarks.yaml`: `sim/webots/config/landmarks.yaml`, the world's ground truth.
//! * `positions.csv`: the lidar positions of the run, one `x,y` pair per line (a header line
//!   that does not parse as numbers is skipped).
//! * `tau_cells`: the tolerance in cells, as fixed in `scripts/phase4_verdict.py`.
//!
//! Writes the metrics of `docs/phase-4-mapping/benchmark.md` section 2 as `key: value` lines
//! to `out.yaml`, or to standard output. Nothing here is a verdict; the verdict is
//! `scripts/phase4_verdict.py` reading this output.

use robotics_core::nalgebra::Vector2;
use robotics_mapping::{evaluate, EvalParams, GroundTruth, OccupancyGrid};
use std::process::ExitCode;

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 4 || args.len() > 5 {
        return Err(
            "usage: map_metrics <grid.txt> <landmarks.yaml> <positions.csv> <tau_cells> [out.yaml]"
                .into(),
        );
    }
    let read = |p: &str| std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"));
    let grid = OccupancyGrid::from_text(&read(&args[0])?).map_err(|e| e.to_string())?;
    let truth = GroundTruth::from_landmarks_yaml(&read(&args[1])?).map_err(|e| e.to_string())?;
    let mut positions = Vec::new();
    for line in read(&args[2])?.lines() {
        let mut it = line.split(',');
        if let (Some(x), Some(y)) = (it.next(), it.next()) {
            if let (Ok(x), Ok(y)) = (x.trim().parse::<f64>(), y.trim().parse::<f64>()) {
                positions.push(Vector2::new(x, y));
            }
        }
    }
    if positions.is_empty() {
        return Err(format!("{}: no `x,y` positions", args[2]));
    }
    let tau_cells: f64 = args[3]
        .parse()
        .map_err(|_| format!("tau_cells: {:?}", args[3]))?;
    let params = EvalParams {
        tau: tau_cells * grid.resolution(),
        sample_spacing: 0.01,
        max_range: 3.5,
        coverage_stride: 10,
    };
    let metrics = evaluate(&grid, &truth, &positions, &params).map_err(|e| e.to_string())?;
    let text = metrics.to_yaml();
    match args.get(4) {
        Some(out) => std::fs::write(out, &text).map_err(|e| format!("{out}: {e}"))?,
        None => print!("{text}"),
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(m) => {
            eprintln!("{m}");
            ExitCode::FAILURE
        }
    }
}
