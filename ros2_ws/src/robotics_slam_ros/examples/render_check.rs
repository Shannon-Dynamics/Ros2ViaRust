//! Render a map from files, for checking what `pose_graph_node` rendered (decision DS17).
//!
//! ```text
//! cargo run --release -p robotics_slam_ros --example render_check -- \
//!     --scans integrated-scans.csv --keyframes keyframes_final.csv --out render.txt [--layout published]
//! ```
//!
//! `--scans` is the recorded stream the run replayed (the Phase 4 `integrated-scans.csv` schema),
//! `--keyframes` a CSV whose rows are `id,stamp_ns,x,y,theta,...` (`keyframes.csv`, the keyframes as
//! they were created, or `keyframes_final.csv`, the optimized ones the node writes at shutdown).
//! Every keyframe's scan is looked up by its stamp, passed through the message conversion the node's
//! subscription applies (the ROS scan has `float32` angles and limits, and the render must see the
//! same numbers), integrated at the keyframe's pose into a fresh grid with the default
//! `RenderConfig` (`--layout` overrides the beam layout: the default is the Webots lidar's
//! `pixel_centres`, which refuses a synthetic scan), and the grid is written in `robotics_mapping`'s text format.
//!
//! The point is that nothing here shares memory with the node: the poses and the scans come from
//! files, so an equal grid means the node's map is a function of those two and nothing else, and a
//! grid rendered from `keyframes.csv` that differs from the one rendered from `keyframes_final.csv`
//! shows that a loop closure moved the keyframes and that the map follows the poses it is given.

use robotics_core::{FrameId, LaserScan, Se2, Timestamp};
use robotics_ros_bridge::convert::{laser_scan_from_ros, laser_scan_to_ros};
use robotics_slam_ros::render::{render_map, RenderConfig};
use std::collections::BTreeMap;

fn read_scans(path: &str) -> Result<BTreeMap<i64, LaserScan>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let mut out = BTreeMap::new();
    for (ln, line) in text.lines().enumerate() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        let bad = |what: &str| format!("{path}:{}: {what}", ln + 1);
        if f.len() < 9 {
            return Err(bad("too few fields"));
        }
        let num = |i: usize| f[i].trim().parse::<f64>().map_err(|e| bad(&e.to_string()));
        let n = num(8)? as usize;
        if f.len() != 9 + n {
            return Err(bad("range count"));
        }
        let ranges = f[9..]
            .iter()
            .map(|r| r.trim().parse::<f32>().map_err(|e| bad(&e.to_string())))
            .collect::<Result<Vec<_>, _>>()?;
        let scan = LaserScan {
            stamp: Timestamp::from_secs_f64(num(0)?),
            frame: FrameId::new("LDS-01"),
            angle_min: num(4)?,
            angle_increment: num(5)?,
            range_min: num(6)?,
            range_max: num(7)?,
            ranges,
            ..LaserScan::default()
        };
        // The conversion the node's subscription sees: ROS carries float32 angles and limits.
        let through_ros =
            laser_scan_from_ros(&laser_scan_to_ros(&scan)).map_err(|e| bad(&e.to_string()))?;
        out.insert(through_ros.stamp.as_nanos(), through_ros);
    }
    Ok(out)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
            .unwrap_or_default()
    };
    let (scans_path, kf_path, out_path) = (get("--scans"), get("--keyframes"), get("--out"));
    let layout_name = get("--layout");
    if scans_path.is_empty() || kf_path.is_empty() || out_path.is_empty() {
        eprintln!("usage: render_check --scans integrated-scans.csv --keyframes keyframes[_final].csv --out render.txt");
        std::process::exit(2);
    }
    let scans = read_scans(&scans_path).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2)
    });
    let text = std::fs::read_to_string(&kf_path).unwrap_or_else(|e| {
        eprintln!("{kf_path}: {e}");
        std::process::exit(2)
    });
    let (mut poses, mut stamps) = (Vec::new(), Vec::new());
    for line in text.lines().skip(1).filter(|l| !l.trim().is_empty()) {
        let f: Vec<&str> = line.split(',').collect();
        let v: Vec<f64> = f
            .iter()
            .take(5)
            .map(|x| x.trim().parse().expect("number"))
            .collect();
        stamps.push(f[1].trim().parse::<i64>().expect("stamp_ns"));
        poses.push(Se2::new(v[2], v[3], v[4]));
    }
    let keyframe_scans: Vec<Option<&LaserScan>> = stamps.iter().map(|s| scans.get(s)).collect();
    let missing = keyframe_scans.iter().filter(|s| s.is_none()).count();
    let mut cfg = RenderConfig::default();
    if !layout_name.is_empty() {
        cfg.layout = robotics_mapping::BeamLayout::parse(&layout_name).unwrap_or_else(|e| {
            eprintln!("--layout: {e}");
            std::process::exit(2)
        });
    }
    let (grid, report) = render_map(&poses, &keyframe_scans, &cfg).expect("render");
    std::fs::write(&out_path, grid.to_text()).expect("write");
    println!(
        "rendered {} keyframes ({} integrated, {} without a scan in the file, {} rejected) -> {out_path}",
        poses.len(),
        report.integrated,
        missing,
        report.skipped_rejected
    );
}
