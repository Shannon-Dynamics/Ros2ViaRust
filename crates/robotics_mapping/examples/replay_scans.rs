//! Replay the scans a `mapping_node` integrated and write the grid they produce.
//!
//!     cargo run --release -p robotics_mapping --example replay_scans -- \
//!         <integrated-scans.csv> <final-grid.txt> <out-grid.txt> [clear_to_max_range|drop] [supercover|bresenham] [clear margin, m] [published|pixel_centres]
//!
//! This is the check of claim M2, determinism, on real simulator data: the recording holds
//! every scan the mapper integrated, in the order it integrated them, each with the sensor pose
//! it was integrated at (`output.record_csv` of `mapping_node`). `final-grid.txt` is the exact
//! grid the node wrote at shutdown; it is read here only for the geometry and the model
//! parameters, and its cells are discarded. If the grid written to `out-grid.txt` is
//! byte-identical to `final-grid.txt` (the text form round-trips bit for bit), the node did
//! nothing to the library's result that the library does not do on its own, and the result
//! does not depend on the run.
//!
//! The policies default to those of `mapping_node`; pass them if the run changed them. The
//! recording holds the bearings the driver published; the beam layout (`scan_beam_layout` of
//! `mapping_node`, default `published`) is applied to each scan before it is integrated, as the
//! node does, so a run made with `pixel_centres` is replayed with `pixel_centres`.

use robotics_core::{FrameId, LaserScan, Se2, Timestamp};
use robotics_mapping::{BeamLayout, NoReturn, OccupancyGrid, RayTracer, ScanIntegrator};
use std::process::ExitCode;

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 3 || args.len() > 7 {
        return Err("usage: replay_scans <integrated-scans.csv> <final-grid.txt> <out-grid.txt> [no-return policy] [traversal] [clear margin] [beam layout]".into());
    }
    let no_return = match args.get(3).map(String::as_str) {
        None | Some("clear_to_max_range") => NoReturn::ClearToMaxRange,
        Some("drop") => NoReturn::Drop,
        Some(o) => return Err(format!("unknown no-return policy {o:?}")),
    };
    let tracer = match args.get(4).map(String::as_str) {
        None | Some("supercover") => RayTracer::Supercover,
        Some("bresenham") => RayTracer::Bresenham,
        Some(o) => return Err(format!("unknown traversal {o:?}")),
    };
    let read = |p: &str| std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"));
    let template = OccupancyGrid::from_text(&read(&args[1])?).map_err(|e| e.to_string())?;
    let mut grid =
        OccupancyGrid::new(*template.geometry(), *template.params()).map_err(|e| e.to_string())?;
    let margin: f64 = match args.get(5) {
        None => robotics_mapping::DEFAULT_CLEAR_MARGIN,
        Some(v) => v
            .parse()
            .map_err(|_| format!("clear margin {v:?} is not a number"))?,
    };
    let layout = match args.get(6) {
        None => BeamLayout::Published,
        Some(v) => BeamLayout::parse(v).map_err(|e| e.to_string())?,
    };
    let mut integrator = ScanIntegrator::new(no_return, tracer)
        .with_clear_margin(margin)
        .map_err(|e| e.to_string())?;

    let mut scans = 0usize;
    for (n, line) in read(&args[0])?.lines().enumerate() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        let num = |i: usize| -> Result<f64, String> {
            f.get(i)
                .ok_or_else(|| format!("line {}: too few fields", n + 1))?
                .parse::<f64>()
                .map_err(|_| format!("line {}: field {i} is not a number", n + 1))
        };
        let beams = num(8)? as usize;
        if f.len() != 9 + beams {
            return Err(format!(
                "line {}: {} fields for {beams} beams",
                n + 1,
                f.len()
            ));
        }
        let mut ranges = Vec::with_capacity(beams);
        for tok in &f[9..] {
            ranges.push(
                tok.parse::<f32>()
                    .map_err(|_| format!("line {}: range {tok:?}", n + 1))?,
            );
        }
        let scan = LaserScan {
            stamp: Timestamp::from_secs_f64(num(0)?),
            frame: FrameId::new("replay"),
            angle_min: num(4)?,
            angle_increment: num(5)?,
            time_increment: 0.0,
            scan_time: 0.0,
            range_min: num(6)?,
            range_max: num(7)?,
            ranges,
            intensities: Vec::new(),
        };
        let scan = layout
            .apply(&scan)
            .map_err(|e| format!("line {}: {e}", n + 1))?;
        integrator
            .integrate(&mut grid, &Se2::new(num(1)?, num(2)?, num(3)?), &scan)
            .map_err(|e| format!("line {}: {e}", n + 1))?;
        scans += 1;
    }
    std::fs::write(&args[2], grid.to_text()).map_err(|e| format!("{}: {e}", args[2]))?;
    eprintln!("replayed {scans} scans (beam layout {})", layout.name());
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
