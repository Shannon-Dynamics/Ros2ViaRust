//! Likelihood-field range model over an exact Euclidean distance transform.
//!
//! Reference: *Probabilistic Robotics via Rust*, Chapter 10 (`DistanceField` built with the
//! Felzenszwalb–Huttenlocher exact EDT, `LikelihoodField` with a sensor offset); Thrun,
//! Burgard & Fox, Table 6.3.
//!
//! For each valid, non-max-range beam the endpoint is projected into the map and scored
//! by the distance `d` to the nearest obstacle:
//! `q += ln(z_hit · N(d; 0, σ_hit²) + z_rand / z_max)`.
//! This is not a generative model of the scan; it is the smooth, cheap score MCL uses.

use super::normal_pdf;
use crate::ModelError;
use robotics_core::nalgebra::Vector2;
use robotics_core::{LaserScan, Se2};

/// Anything that can answer "distance to the nearest obstacle".
pub trait DistanceField {
    /// Distance (m) from a world point to the nearest obstacle.
    fn distance(&self, p: &Vector2<f64>) -> f64;
}

/// A metric grid of exact distances to the nearest occupied cell centre.
///
/// Cell `(i, j)` covers `[origin + (i, j)·res, origin + (i+1, j+1)·res)`; storage is
/// row-major with `j` (y) as the slow index, the `nav_msgs/OccupancyGrid` layout.
#[derive(Debug, Clone, PartialEq)]
pub struct DistanceGrid {
    origin: Vector2<f64>,
    resolution: f64,
    width: usize,
    height: usize,
    dist: Vec<f64>,
}

const FAR: f64 = 1e20;

/// 1-D squared distance transform (Felzenszwalb & Huttenlocher 2012): lower envelope of
/// the parabolas `(q − p)² + f(p)`. `v` needs `n` slots, `z` needs `n + 1`.
fn edt_1d(f: &[f64], out: &mut [f64], v: &mut [usize], z: &mut [f64]) {
    let n = f.len();
    if n == 0 {
        return;
    }
    let mut k = 0usize;
    v[0] = 0;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    for q in 1..n {
        let intersect = |p: usize| {
            ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64)) / (2.0 * (q as f64 - p as f64))
        };
        let mut s = intersect(v[k]);
        // z[0] = −∞ and every s is finite, so this never underflows k.
        while s <= z[k] {
            k -= 1;
            s = intersect(v[k]);
        }
        k += 1;
        v[k] = q;
        z[k] = s;
        z[k + 1] = f64::INFINITY;
    }
    k = 0;
    for (q, o) in out.iter_mut().enumerate().take(n) {
        while z[k + 1] < q as f64 {
            k += 1;
        }
        let d = q as f64 - v[k] as f64;
        *o = d * d + f[v[k]];
    }
}

impl DistanceGrid {
    /// Build from an occupancy mask (`true` = obstacle) with exact Euclidean distances in
    /// metres between cell centres. A grid without obstacles yields `+∞` everywhere.
    pub fn from_occupancy(
        occupied: &[bool],
        width: usize,
        height: usize,
        resolution: f64,
        origin: Vector2<f64>,
    ) -> Result<Self, ModelError> {
        if occupied.len() != width * height || width == 0 || height == 0 {
            return Err(ModelError::NonPositive {
                what: "grid size",
                value: occupied.len() as f64,
            });
        }
        if !(resolution.is_finite() && resolution > 0.0) {
            return Err(ModelError::NonPositive {
                what: "resolution",
                value: resolution,
            });
        }
        let n = width.max(height);
        let (mut v, mut z) = (vec![0usize; n], vec![0.0f64; n + 1]);
        let mut col_in = vec![0.0; height];
        let mut col_out = vec![0.0; height];
        let mut sq = vec![0.0f64; width * height];
        // Columns first.
        for i in 0..width {
            for j in 0..height {
                col_in[j] = if occupied[j * width + i] { 0.0 } else { FAR };
            }
            edt_1d(&col_in, &mut col_out, &mut v, &mut z);
            for j in 0..height {
                sq[j * width + i] = col_out[j];
            }
        }
        // Then rows.
        let mut row_out = vec![0.0; width];
        for j in 0..height {
            let row = sq[j * width..(j + 1) * width].to_vec();
            edt_1d(&row, &mut row_out, &mut v, &mut z);
            sq[j * width..(j + 1) * width].copy_from_slice(&row_out);
        }
        let dist = sq
            .into_iter()
            .map(|d| {
                if d >= FAR * 0.5 {
                    f64::INFINITY
                } else {
                    d.sqrt() * resolution
                }
            })
            .collect();
        Ok(Self {
            origin,
            resolution,
            width,
            height,
            dist,
        })
    }

    /// Distance stored at cell `(i, j)`.
    pub fn cell(&self, i: usize, j: usize) -> Option<f64> {
        (i < self.width && j < self.height).then(|| self.dist[j * self.width + i])
    }

    /// Grid width in cells.
    pub fn width(&self) -> usize {
        self.width
    }
    /// Grid height in cells.
    pub fn height(&self) -> usize {
        self.height
    }
}

impl DistanceField for DistanceGrid {
    /// Nearest-cell lookup. Points farther than half a cell diagonal from the nearest
    /// cell centre (i.e. outside the grid) get that excess added, so the field keeps
    /// growing away from the map.
    fn distance(&self, p: &Vector2<f64>) -> f64 {
        let fx = (p.x - self.origin.x) / self.resolution - 0.5;
        let fy = (p.y - self.origin.y) / self.resolution - 0.5;
        let i = fx.round().clamp(0.0, (self.width - 1) as f64);
        let j = fy.round().clamp(0.0, (self.height - 1) as f64);
        let d = self.dist[j as usize * self.width + i as usize];
        let off = ((fx - i).powi(2) + (fy - j).powi(2)).sqrt() - std::f64::consts::FRAC_1_SQRT_2;
        d + off.max(0.0) * self.resolution
    }
}

/// Likelihood-field parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LikelihoodFieldParams {
    /// Weight of the Gaussian term.
    pub z_hit: f64,
    /// Weight of the uniform term.
    pub z_rand: f64,
    /// σ of the endpoint-distance Gaussian (m).
    pub sigma_hit: f64,
    /// Sensor maximum range (m); max-range beams are skipped.
    pub max_range: f64,
}

impl Default for LikelihoodFieldParams {
    fn default() -> Self {
        Self {
            z_hit: 0.9,
            z_rand: 0.1,
            sigma_hit: 0.2,
            max_range: 8.0,
        }
    }
}

/// The likelihood-field model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LikelihoodFieldModel {
    p: LikelihoodFieldParams,
}

impl LikelihoodFieldModel {
    /// Build with validated parameters.
    pub fn new(p: LikelihoodFieldParams) -> Result<Self, ModelError> {
        for (what, value) in [("z_hit", p.z_hit), ("z_rand", p.z_rand)] {
            if !(value.is_finite() && value >= 0.0) {
                return Err(ModelError::Negative { what, value });
            }
        }
        for (what, value) in [("sigma_hit", p.sigma_hit), ("max_range", p.max_range)] {
            if !(value.is_finite() && value > 0.0) {
                return Err(ModelError::NonPositive { what, value });
            }
        }
        Ok(Self { p })
    }

    /// Log-likelihood of `scan` from robot pose `pose`, with the sensor mounted at
    /// `sensor_offset` in the body frame, using every `stride`-th valid beam.
    pub fn log_likelihood<F: DistanceField>(
        &self,
        scan: &LaserScan,
        pose: &Se2,
        sensor_offset: &Se2,
        field: &F,
        stride: usize,
    ) -> f64 {
        let sensor = pose.compose(sensor_offset);
        let p = &self.p;
        let max = p.max_range.min(scan.range_max);
        scan.valid_beams_strided(stride)
            .filter(|(_, r)| *r < max - 1e-9)
            .map(|(b, r)| {
                let end = sensor.act(&Vector2::new(r * b.cos(), r * b.sin()));
                let d = field.distance(&end);
                (p.z_hit * normal_pdf(d, p.sigma_hit) + p.z_rand / p.max_range)
                    .max(1e-300)
                    .ln()
            })
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::RngExt;
    use robotics_core::seeded_rng;

    #[test]
    fn edt_matches_brute_force() {
        let (w, h) = (37, 23);
        let mut rng = seeded_rng(5);
        let occ: Vec<bool> = (0..w * h).map(|_| rng.random::<f64>() < 0.04).collect();
        let g = DistanceGrid::from_occupancy(&occ, w, h, 0.1, Vector2::zeros()).unwrap();
        for j in 0..h {
            for i in 0..w {
                let mut best = f64::INFINITY;
                for jj in 0..h {
                    for ii in 0..w {
                        if occ[jj * w + ii] {
                            let d = ((i as f64 - ii as f64).powi(2)
                                + (j as f64 - jj as f64).powi(2))
                            .sqrt();
                            best = best.min(d * 0.1);
                        }
                    }
                }
                let got = g.cell(i, j).unwrap();
                assert!((got - best).abs() < 1e-9, "({i},{j}) {got} vs {best}");
            }
        }
    }

    #[test]
    fn empty_grid_is_infinitely_far() {
        let g = DistanceGrid::from_occupancy(&[false; 12], 4, 3, 0.5, Vector2::zeros()).unwrap();
        assert!(g.cell(1, 1).unwrap().is_infinite());
        assert!(DistanceGrid::from_occupancy(&[false; 11], 4, 3, 0.5, Vector2::zeros()).is_err());
    }

    /// Square room 6 m × 6 m, walls one cell thick; the scan simulated from the true pose
    /// scores higher there than from displaced or rotated poses.
    #[test]
    fn likelihood_field_peaks_at_true_pose() {
        let res = 0.05;
        let n = 121; // 6.05 m
        let mut occ = vec![false; n * n];
        for k in 0..n {
            occ[k] = true;
            occ[(n - 1) * n + k] = true;
            occ[k * n] = true;
            occ[k * n + n - 1] = true;
        }
        let g = DistanceGrid::from_occupancy(&occ, n, n, res, Vector2::zeros()).unwrap();
        // Walls at cell centres x, y ∈ {0.025, 6.025}.
        let (lo, hi) = (0.025, 6.025);
        let truth = Se2::new(2.0, 3.5, 0.4);
        let beams = 72;
        let inc = std::f64::consts::TAU / beams as f64;
        let ranges: Vec<f32> = (0..beams)
            .map(|k| {
                let a = truth.theta() + k as f64 * inc;
                let (c, s) = (a.cos(), a.sin());
                let mut t = f64::INFINITY;
                if c > 1e-12 {
                    t = t.min((hi - truth.x()) / c);
                }
                if c < -1e-12 {
                    t = t.min((lo - truth.x()) / c);
                }
                if s > 1e-12 {
                    t = t.min((hi - truth.y()) / s);
                }
                if s < -1e-12 {
                    t = t.min((lo - truth.y()) / s);
                }
                t as f32
            })
            .collect();
        let scan = LaserScan {
            angle_min: 0.0,
            angle_increment: inc,
            range_min: 0.05,
            range_max: 8.0,
            ranges,
            ..Default::default()
        };
        let m = LikelihoodFieldModel::new(LikelihoodFieldParams::default()).unwrap();
        let score = |p: &Se2| m.log_likelihood(&scan, p, &Se2::IDENTITY, &g, 1);
        let best = score(&truth);
        for p in [
            Se2::new(2.3, 3.5, 0.4),
            Se2::new(2.0, 3.2, 0.4),
            Se2::new(2.0, 3.5, 0.55),
        ] {
            assert!(score(&p) < best, "{p:?} scored higher than the truth");
        }
    }
}
