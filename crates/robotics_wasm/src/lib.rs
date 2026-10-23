//! `robotics_wasm` — the WebAssembly target of the algorithm crates.
//!
//! Every in-page simulation of the ROS via Rust webbook calls this module. The module
//! contains no algorithms: each exported function builds the types of `robotics_core`,
//! `robotics_models`, `robotics_filters`, `robotics_sim`, `robotics_mapping`, `robotics_localization` or `robotics_slam`, calls them, and flattens the
//! result into numbers JavaScript can draw. What the page shows is therefore what the robot
//! computes, and the Rust listings printed in a chapter are the code behind its widgets.
//!
//! # Conventions
//! * Inputs and outputs are `f64`, `u64` (a `bigint` in JavaScript) and flat `Vec<f64>`
//!   (a `Float64Array`), with the layout documented on each function.
//! * Every stochastic function takes an explicit `seed: u64`; none reads OS entropy.
//! * Errors are returned as `Err(String)`, which JavaScript sees as a thrown string.
//! * One module per chapter, so a chapter's bindings can change without touching another's.

#![forbid(unsafe_code)]

pub mod ch01;
pub mod ch02;
pub mod ch03;
pub mod ch05;
pub mod ch04;
pub mod ch06;
pub mod common;
