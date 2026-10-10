//! Node state shared between callbacks, without rclrs `Worker`s.
//!
//! rclrs 0.8 gives every `Worker` its own wait-set thread. With CycloneDDS, processes whose
//! wait sets run on several threads stall for seconds (or indefinitely) whenever another DDS
//! participant joins or leaves, e.g. a `ros2 topic hz` CLI. A minimal worker-based talker
//! reproduces it; the same talker built only from node-level primitives (one wait-set
//! thread) does not (evidence in `docs/STATE_ESTIMATION.md`, "Known limitations").
//!
//! Framework nodes therefore register node-level callbacks and keep their mutable state in
//! a [`Shared`]: an `Arc<Mutex<T>>` whose lock recovers from poisoning, so one panicking
//! callback cannot wedge the node forever.

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

/// Reference-counted, mutex-protected node state.
pub struct Shared<T>(Arc<Mutex<T>>);

impl<T> Clone for Shared<T> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<T> Shared<T> {
    /// Wrap a state value.
    pub fn new(value: T) -> Self {
        Self(Arc::new(Mutex::new(value)))
    }

    /// Lock the state. A poisoned mutex (a callback panicked while holding it) is recovered
    /// rather than propagated: the state is still structurally valid Rust data.
    pub fn lock(&self) -> MutexGuard<'_, T> {
        self.0.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shared_state_is_visible_across_clones_and_survives_poison() {
        let a = Shared::new(1u32);
        let b = a.clone();
        *a.lock() += 1;
        assert_eq!(*b.lock(), 2);
        let c = a.clone();
        let _ = std::thread::spawn(move || {
            let _g = c.lock();
            panic!("poison the mutex");
        })
        .join();
        *b.lock() += 1;
        assert_eq!(*a.lock(), 3);
    }
}
