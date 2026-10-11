//! A voxel-hash point map: the correspondence structure of ICP.
//!
//! Points are binned into square cells of side `cell`, keyed by `(⌊x/cell⌋, ⌊y/cell⌋)`. A
//! nearest-neighbour query visits the `⌈r/cell⌉`-ring of cells around the query in a fixed
//! order, so it needs no tree and the map can grow and shed points without a rebuild. A cell
//! holds at most `max_per_cell` points; once full it ignores further points, which bounds the
//! cost of a long run and filters transient obstacles.
//!
//! Determinism (decision DS5): `std`'s `HashMap` seeds its hasher per process, so this map
//! uses a fixed multiplicative hasher, and no result depends on the order the table is
//! iterated. Ties on distance go to the lower insertion index.

use crate::SlamError;
use nalgebra::Vector2;
use std::collections::{BTreeMap, HashMap};
use std::hash::{BuildHasherDefault, Hasher};

/// A fixed, non-random hasher for pairs of cell indices.
#[derive(Default, Clone, Copy)]
pub struct CellHasher(u64);

impl Hasher for CellHasher {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0.rotate_left(5) ^ u64::from(b)).wrapping_mul(0x517c_c1b7_2722_0a95);
        }
    }
    fn write_i32(&mut self, i: i32) {
        self.0 = (self.0.rotate_left(5) ^ (i as u32 as u64)).wrapping_mul(0x517c_c1b7_2722_0a95);
    }
}

type CellKey = (i32, i32);
type CellTable = HashMap<CellKey, Vec<(u64, Vector2<f64>)>, BuildHasherDefault<CellHasher>>;

/// The point map.
#[derive(Debug, Clone)]
pub struct VoxelMap {
    cell: f64,
    max_per_cell: usize,
    cells: CellTable,
    next_seq: u64,
    len: usize,
}

impl VoxelMap {
    /// An empty map. `cell > 0` (m) and `max_per_cell ≥ 1`.
    pub fn new(cell: f64, max_per_cell: usize) -> Result<Self, SlamError> {
        if !(cell.is_finite() && cell > 0.0) {
            return Err(SlamError::InvalidParameter {
                what: "voxel cell",
                value: cell,
            });
        }
        if max_per_cell == 0 {
            return Err(SlamError::InvalidParameter {
                what: "max_per_cell",
                value: 0.0,
            });
        }
        Ok(Self {
            cell,
            max_per_cell,
            cells: CellTable::default(),
            next_seq: 0,
            len: 0,
        })
    }

    /// Cell side (m).
    pub fn cell(&self) -> f64 {
        self.cell
    }

    /// Number of points held.
    pub fn len(&self) -> usize {
        self.len
    }

    /// `true` when no point is held.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    fn key(&self, p: &Vector2<f64>) -> CellKey {
        (
            (p.x / self.cell).floor() as i32,
            (p.y / self.cell).floor() as i32,
        )
    }

    /// Add a point unless its cell is full or it is not finite. Returns whether it was stored.
    pub fn insert(&mut self, p: Vector2<f64>) -> bool {
        if !(p.x.is_finite() && p.y.is_finite()) {
            return false;
        }
        let key = self.key(&p);
        let bucket = self.cells.entry(key).or_default();
        if bucket.len() >= self.max_per_cell {
            return false;
        }
        bucket.push((self.next_seq, p));
        self.next_seq += 1;
        self.len += 1;
        true
    }

    /// Add every point in order.
    pub fn insert_all(&mut self, points: &[Vector2<f64>]) {
        for p in points {
            self.insert(*p);
        }
    }

    /// Remove every point farther than `radius` from `center`.
    pub fn retain_within(&mut self, center: &Vector2<f64>, radius: f64) {
        let r2 = radius * radius;
        let mut len = 0;
        self.cells.retain(|_, bucket| {
            bucket.retain(|(_, p)| (p - center).norm_squared() <= r2);
            len += bucket.len();
            !bucket.is_empty()
        });
        self.len = len;
    }

    fn for_each_in_ring(
        &self,
        q: &Vector2<f64>,
        radius: f64,
        mut f: impl FnMut(u64, &Vector2<f64>),
    ) {
        let (cx, cy) = self.key(q);
        let ring = (radius / self.cell).ceil().max(1.0) as i32;
        for dy in -ring..=ring {
            for dx in -ring..=ring {
                if let Some(bucket) = self.cells.get(&(cx + dx, cy + dy)) {
                    for (seq, p) in bucket {
                        f(*seq, p);
                    }
                }
            }
        }
    }

    /// The point nearest `q` within `radius`, with its squared distance.
    pub fn nearest(&self, q: &Vector2<f64>, radius: f64) -> Option<(Vector2<f64>, f64)> {
        let r2 = radius * radius;
        let mut best: Option<(f64, u64, Vector2<f64>)> = None;
        self.for_each_in_ring(q, radius, |seq, p| {
            let d2 = (p - q).norm_squared();
            if d2 <= r2 && best.as_ref().is_none_or(|b| (d2, seq) < (b.0, b.1)) {
                best = Some((d2, seq, *p));
            }
        });
        best.map(|(d2, _, p)| (p, d2))
    }

    /// Every point within `radius` of `q`, nearest first, ties by insertion order.
    pub fn within(&self, q: &Vector2<f64>, radius: f64) -> Vec<Vector2<f64>> {
        let r2 = radius * radius;
        let mut hits: Vec<(f64, u64, Vector2<f64>)> = Vec::new();
        self.for_each_in_ring(q, radius, |seq, p| {
            let d2 = (p - q).norm_squared();
            if d2 <= r2 {
                hits.push((d2, seq, *p));
            }
        });
        hits.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        hits.into_iter().map(|(_, _, p)| p).collect()
    }

    /// Every point within `radius` of `q`, in the fixed visiting order of the ring search (not
    /// sorted by distance): deterministic, and cheaper when the order does not matter.
    pub fn within_unsorted(&self, q: &Vector2<f64>, radius: f64) -> Vec<Vector2<f64>> {
        let r2 = radius * radius;
        let mut hits = Vec::new();
        self.for_each_in_ring(q, radius, |_, p| {
            if (p - q).norm_squared() <= r2 {
                hits.push(*p);
            }
        });
        hits
    }

    /// Every point held, in insertion order.
    pub fn points(&self) -> Vec<Vector2<f64>> {
        let mut all: Vec<(u64, Vector2<f64>)> = self
            .cells
            .values()
            .flat_map(|b| b.iter().copied())
            .collect();
        all.sort_by_key(|(seq, _)| *seq);
        all.into_iter().map(|(_, p)| p).collect()
    }
}

/// Voxel-grid downsampling: one point per occupied cell, the centroid of the points in it,
/// returned ordered by cell index so the output is reproducible.
pub fn downsample(points: &[Vector2<f64>], cell: f64) -> Result<Vec<Vector2<f64>>, SlamError> {
    if !(cell.is_finite() && cell > 0.0) {
        return Err(SlamError::InvalidParameter {
            what: "downsample cell",
            value: cell,
        });
    }
    let mut bins: BTreeMap<CellKey, (Vector2<f64>, usize)> = BTreeMap::new();
    for p in points.iter().filter(|p| p.x.is_finite() && p.y.is_finite()) {
        let key = ((p.x / cell).floor() as i32, (p.y / cell).floor() as i32);
        let e = bins.entry(key).or_insert((Vector2::zeros(), 0));
        e.0 += p;
        e.1 += 1;
    }
    Ok(bins.into_values().map(|(s, n)| s / n as f64).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use robotics_core::seeded_rng;

    fn v(x: f64, y: f64) -> Vector2<f64> {
        Vector2::new(x, y)
    }

    #[test]
    fn nearest_matches_brute_force_over_random_clouds() {
        use rand::RngExt;
        let mut rng = seeded_rng(11);
        let pts: Vec<_> = (0..400)
            .map(|_| v(rng.random_range(-2.0..2.0), rng.random_range(-2.0..2.0)))
            .collect();
        let mut map = VoxelMap::new(0.1, 1000).unwrap();
        map.insert_all(&pts);
        assert_eq!(map.len(), 400);
        for _ in 0..200 {
            let q = v(rng.random_range(-2.5..2.5), rng.random_range(-2.5..2.5));
            let radius = rng.random_range(0.05..0.6);
            let brute = pts
                .iter()
                .map(|p| (p - q).norm_squared())
                .filter(|d| *d <= radius * radius)
                .fold(None, |a: Option<f64>, d| Some(a.map_or(d, |m| m.min(d))));
            let got = map.nearest(&q, radius).map(|(_, d2)| d2);
            assert_eq!(got.map(f64::to_bits), brute.map(f64::to_bits));
        }
    }

    #[test]
    fn the_per_cell_cap_drops_later_points() {
        let mut map = VoxelMap::new(0.1, 2).unwrap();
        assert!(map.insert(v(0.01, 0.01)));
        assert!(map.insert(v(0.02, 0.02)));
        assert!(!map.insert(v(0.03, 0.03)));
        assert!(map.insert(v(0.15, 0.01)));
        assert_eq!(map.len(), 3);
    }

    #[test]
    fn a_tie_goes_to_the_earlier_point() {
        let mut map = VoxelMap::new(0.5, 8).unwrap();
        map.insert(v(0.1, 0.0));
        map.insert(v(-0.1, 0.0));
        let (p, d2) = map.nearest(&v(0.0, 0.0), 0.5).unwrap();
        assert_relative_eq!(p.x, 0.1);
        assert_relative_eq!(d2, 0.01);
    }

    #[test]
    fn negative_coordinates_and_cell_boundaries_are_handled() {
        let mut map = VoxelMap::new(0.1, 8).unwrap();
        map.insert(v(-0.0001, -0.0001));
        map.insert(v(0.0, 0.0));
        let near = map.within(&v(0.0, 0.0), 0.01);
        assert_eq!(near.len(), 2);
    }

    #[test]
    fn retain_within_evicts_far_points() {
        let mut map = VoxelMap::new(0.2, 8).unwrap();
        for i in 0..20 {
            map.insert(v(i as f64 * 0.5, 0.0));
        }
        map.retain_within(&v(0.0, 0.0), 2.0);
        assert_eq!(map.len(), 5);
        assert!(map.points().iter().all(|p| p.norm() <= 2.0));
    }

    #[test]
    fn insertion_is_deterministic_and_points_come_back_in_order() {
        let pts: Vec<_> = (0..50)
            .map(|i| v(i as f64 * 0.13, (i as f64).sin()))
            .collect();
        let build = || {
            let mut m = VoxelMap::new(0.1, 3).unwrap();
            m.insert_all(&pts);
            m.points()
        };
        let (a, b) = (build(), build());
        assert_eq!(a.len(), b.len());
        for (p, q) in a.iter().zip(&b) {
            assert_eq!(
                (p.x.to_bits(), p.y.to_bits()),
                (q.x.to_bits(), q.y.to_bits())
            );
        }
    }

    #[test]
    fn downsample_returns_centroids_in_cell_order() {
        let pts = [v(0.01, 0.01), v(0.03, 0.05), v(0.5, 0.5), v(-0.2, 0.0)];
        let d = downsample(&pts, 0.1).unwrap();
        assert_eq!(d.len(), 3);
        assert_relative_eq!(d[0].x, -0.2);
        assert_relative_eq!(d[1].x, 0.02, epsilon = 1e-12);
        assert_relative_eq!(d[1].y, 0.03, epsilon = 1e-12);
    }

    #[test]
    fn non_finite_points_and_bad_parameters_are_refused() {
        let mut map = VoxelMap::new(0.1, 4).unwrap();
        assert!(!map.insert(v(f64::NAN, 0.0)));
        assert!(map.is_empty());
        assert!(VoxelMap::new(0.0, 4).is_err());
        assert!(VoxelMap::new(0.1, 0).is_err());
        assert!(downsample(&[], -1.0).is_err());
    }
}
