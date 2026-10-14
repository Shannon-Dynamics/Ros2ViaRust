//! Map rendering for the pose graph node: decision DS17 of `docs/phase-5-slam/implementation.md`.
//!
//! **A map is a replay.** The graph keeps every keyframe's scan; the map is made by integrating
//! those scans, each at its keyframe's *current* optimized pose, into a **fresh** Phase 4
//! [`OccupancyGrid`] with Phase 4's [`ScanIntegrator`], in keyframe order. After a loop closure
//! every keyframe pose has moved a little, and nothing here tries to work out which cells to undo:
//! the whole grid is thrown away and rendered again. There is no function in this module that
//! changes an existing grid. (`robotics_slam` itself still has no `robotics_mapping` in its
//! dependency tree; the dependency is this crate's, on purpose, and `cargo tree -p robotics_slam`
//! is how that stays true.)
//!
//! The grid's frame is the node's `map` frame, whose origin is the first keyframe's sensor pose
//! (the pose graph's gauge). That is not the Webots world's `map` frame: a SLAM map has no world
//! to be anchored in. The grid is a square of `size_m` centred on that origin.
//!
//! Defaults are the parameters Phase 4 froze on 2026-10-11 for the Webots lidar (Bresenham
//! traversal, upper clamp probability 0.99330715, the pixel-centre beam layout, clearing no-return
//! beams to `range_max` less 0.10 m): the same integrator under the same settings, so a map
//! rendered here differs from a Phase 4 map only in the poses it was handed.

use robotics_core::{LaserScan, Se2};
use robotics_mapping::{
    BeamLayout, GridGeometry, MappingError, NoReturn, OccupancyGrid, OccupancyParams, RayTracer,
    ScanIntegrator,
};

/// How a map is rendered.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RenderConfig {
    /// Edge of the square grid, centred on the origin of the node's `map` frame (m).
    pub size_m: f64,
    /// Cell edge (m).
    pub resolution: f64,
    /// Policy for beams that returned nothing.
    pub no_return: NoReturn,
    /// Ray traversal.
    pub tracer: RayTracer,
    /// How far short of `range_max` a no-return beam stops clearing (m).
    pub clear_margin: f64,
    /// Upper clamp probability of the occupancy model.
    pub p_max: f64,
    /// Bearings of the beams (`PixelCentres` is the Webots lidar's).
    pub layout: BeamLayout,
}

impl Default for RenderConfig {
    fn default() -> Self {
        Self {
            size_m: 12.0,
            resolution: 0.05,
            no_return: NoReturn::ClearToMaxRange,
            tracer: RayTracer::Bresenham,
            clear_margin: 0.10,
            p_max: 0.993_307_15,
            layout: BeamLayout::PixelCentres,
        }
    }
}

/// What a render did.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RenderReport {
    /// Keyframe scans integrated.
    pub integrated: usize,
    /// Keyframes skipped because they carry no stored scan.
    pub skipped_no_scan: usize,
    /// Keyframes skipped because their sensor was outside the grid or the scan was malformed.
    pub skipped_rejected: usize,
}

/// A render could not even start.
#[derive(Debug, thiserror::Error)]
pub enum RenderError {
    /// The grid or the integrator refused a parameter.
    #[error("render configuration: {0}")]
    Config(#[from] MappingError),
}

/// Render the map from scratch: `poses[i]` is the optimized pose of keyframe `i`'s sensor in the
/// map frame and `scans[i]` its stored scan (`None` for a keyframe made without one).
pub fn render_map(
    poses: &[Se2],
    scans: &[Option<&LaserScan>],
    cfg: &RenderConfig,
) -> Result<(OccupancyGrid, RenderReport), RenderError> {
    let n = (cfg.size_m / cfg.resolution).round() as usize;
    let half = cfg.size_m / 2.0;
    let geometry = GridGeometry {
        width: n,
        height: n,
        resolution: cfg.resolution,
        origin: robotics_core::nalgebra::Vector2::new(-half, -half),
    };
    let params = OccupancyParams::from_probabilities(0.7, 0.4, 0.12, cfg.p_max, 0.95, 0.35);
    let mut grid = OccupancyGrid::new(geometry, params)?;
    let mut integrator =
        ScanIntegrator::new(cfg.no_return, cfg.tracer).with_clear_margin(cfg.clear_margin)?;
    let mut report = RenderReport::default();
    for (pose, scan) in poses.iter().zip(scans) {
        let Some(scan) = scan else {
            report.skipped_no_scan += 1;
            continue;
        };
        let scan = match cfg.layout.apply(scan) {
            Ok(s) => s,
            Err(_) => {
                report.skipped_rejected += 1;
                continue;
            }
        };
        match integrator.integrate(&mut grid, pose, &scan) {
            Ok(_) => report.integrated += 1,
            Err(_) => report.skipped_rejected += 1,
        }
    }
    Ok((grid, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use robotics_core::nalgebra::Vector2;
    use robotics_core::{seeded_rng, FrameId, Timestamp};
    use robotics_mapping::CellState;
    use robotics_sim::{Lidar, LidarConfig, World};

    fn room() -> World {
        let mut w = World::empty();
        w.add_rectangle(Vector2::new(-2.0, -2.0), Vector2::new(2.0, 2.0))
            .unwrap();
        w.add_box(Vector2::new(0.8, 0.6), Vector2::new(0.3, 0.2), 0.4)
            .unwrap();
        w
    }

    fn lidar() -> Lidar {
        Lidar::new(
            LidarConfig {
                mount: Se2::IDENTITY,
                range_min: 0.12,
                range_max: 3.5,
                range_sigma: 0.0,
                dropout_probability: 0.0,
                rate_hz: 5.0,
                ..LidarConfig::default()
            },
            3,
        )
        .unwrap()
    }

    /// Scans of the room from a few true poses, as the node would have stored them.
    fn keyframes() -> (Vec<Se2>, Vec<LaserScan>) {
        let (world, mut lid) = (room(), lidar());
        let poses: Vec<Se2> = (0..8)
            .map(|k| Se2::new(-1.0 + 0.25 * k as f64, -0.5, 0.0))
            .collect();
        let scans = poses
            .iter()
            .enumerate()
            .map(|(k, p)| lid.scan(&world, p, Timestamp::from_secs_f64(0.2 * k as f64)))
            .collect();
        let _ = seeded_rng(0);
        let _ = FrameId::new("laser");
        (poses, scans)
    }

    fn refs(scans: &[LaserScan]) -> Vec<Option<&LaserScan>> {
        scans.iter().map(Some).collect()
    }

    fn cfg() -> RenderConfig {
        RenderConfig {
            layout: BeamLayout::Published,
            ..RenderConfig::default()
        }
    }

    #[test]
    fn a_render_integrates_every_keyframe_scan_and_marks_the_walls_occupied() {
        let (poses, scans) = keyframes();
        let (grid, rep) = render_map(&poses, &refs(&scans), &cfg()).unwrap();
        assert_eq!(rep.integrated, poses.len());
        assert_eq!(rep.skipped_no_scan + rep.skipped_rejected, 0);
        let (free, occupied, _) = grid.count_states();
        assert!(
            free > 500 && occupied > 40,
            "free {free}, occupied {occupied}"
        );
        // The wall at x = 2 is seen from the keyframes: the cell on it is occupied.
        let wall = grid
            .geometry()
            .world_to_cell(&Vector2::new(2.0, -0.5))
            .expect("inside the grid");
        let near_wall = (-2i64..=2).any(|d| {
            let i = (wall.0 as i64 + d) as usize;
            grid.state(i, wall.1) == Some(CellState::Occupied)
        });
        assert!(near_wall, "the east wall is not marked occupied");
    }

    #[test]
    fn rendering_twice_gives_the_identical_grid() {
        let (poses, scans) = keyframes();
        let (a, _) = render_map(&poses, &refs(&scans), &cfg()).unwrap();
        let (b, _) = render_map(&poses, &refs(&scans), &cfg()).unwrap();
        assert_eq!(a.to_text(), b.to_text());
    }

    #[test]
    fn a_render_depends_on_the_poses_it_is_given_and_on_nothing_it_rendered_before() {
        // After a loop closure the poses change. Rendering with the new poses must equal
        // rendering with them from the start: no cell of the old map survives.
        let (poses, scans) = keyframes();
        let moved: Vec<Se2> = poses
            .iter()
            .map(|p| Se2::new(p.x() + 0.2, p.y() - 0.1, p.theta() + 0.05))
            .collect();
        let (before, _) = render_map(&poses, &refs(&scans), &cfg()).unwrap();
        let (after, _) = render_map(&moved, &refs(&scans), &cfg()).unwrap();
        let (fresh, _) = render_map(&moved, &refs(&scans), &cfg()).unwrap();
        assert_ne!(
            before.to_text(),
            after.to_text(),
            "moving every pose changed nothing"
        );
        assert_eq!(after.to_text(), fresh.to_text());
    }

    #[test]
    fn a_keyframe_without_a_stored_scan_is_skipped_and_counted() {
        let (poses, scans) = keyframes();
        let mut r = refs(&scans);
        r[3] = None;
        let (_, rep) = render_map(&poses, &r, &cfg()).unwrap();
        assert_eq!(rep.integrated, poses.len() - 1);
        assert_eq!(rep.skipped_no_scan, 1);
    }

    #[test]
    fn a_sensor_outside_the_grid_is_skipped_and_counted_not_fatal() {
        let (mut poses, scans) = keyframes();
        poses[2] = Se2::new(50.0, 0.0, 0.0);
        let (_, rep) = render_map(&poses, &refs(&scans), &cfg()).unwrap();
        assert_eq!(rep.skipped_rejected, 1);
        assert_eq!(rep.integrated, poses.len() - 1);
    }

    #[test]
    fn the_defaults_are_the_parameters_phase_4_froze() {
        let c = RenderConfig::default();
        assert_eq!(c.tracer, RayTracer::Bresenham);
        assert_eq!(c.layout, BeamLayout::PixelCentres);
        assert_eq!(c.no_return, NoReturn::ClearToMaxRange);
        assert!((c.clear_margin - 0.10).abs() < 1e-12);
        // l_max = ln(p/(1-p)) = 5.0 exactly (docs/phase-4-mapping/implementation.md 5.4).
        assert!(((c.p_max / (1.0 - c.p_max)).ln() - 5.0).abs() < 1e-6);
    }
}
