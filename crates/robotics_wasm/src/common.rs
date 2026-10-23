//! Items every chapter module shares.

use wasm_bindgen::prelude::*;

/// The seed the book uses for its worked examples (`robotics_core::BOOK_SEED`).
///
/// The TypeScript seed module derives every widget seed from this constant; a check
/// asserts that the two sides agree.
#[wasm_bindgen]
pub fn book_seed() -> u64 {
    robotics_core::BOOK_SEED
}

/// Crate version, so the page can show which build of the Rust it is running.
#[wasm_bindgen]
pub fn wasm_version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}
