//! Seeded random number generation.
//!
//! The book's determinism rule: every stochastic routine takes an explicit RNG, never a
//! thread-local one, so a run can be replayed bit-for-bit from its seed.

use rand::{rngs::SmallRng, SeedableRng};

/// The seed the reference book uses for its worked examples.
pub const BOOK_SEED: u64 = 0x5EED_2026;

/// A fast, non-cryptographic generator seeded from a `u64`.
///
/// `SmallRng` is deterministic for a given seed *and* crate version; tests that pin
/// sampled values must therefore compare against statistical bounds, not exact draws.
#[inline]
pub fn seeded_rng(seed: u64) -> SmallRng {
    SmallRng::seed_from_u64(seed)
}
