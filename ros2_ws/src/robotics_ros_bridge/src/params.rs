//! Declaring validated read-only parameters with less boilerplate.
//!
//! In rclrs 0.8 dropping a parameter handle removes the declaration (it disappears from
//! `ros2 param list`). [`ParamStore`] declares a parameter, reads its value once, and keeps
//! the handle alive for as long as the store lives, which must be until `spin` returns.
//!
//! Conventions (Phase 1): every parameter has a default, a description, and a range where
//! a range makes sense; parameters that cannot safely change at runtime are read-only.

use rclrs::{Node, ParameterRange, RclrsError, ReadOnlyParameter};
use std::any::Any;
use std::sync::Arc;

/// Owner of declared parameter handles.
#[derive(Default)]
pub struct ParamStore {
    handles: Vec<Box<dyn Any + Send + Sync>>,
}

impl ParamStore {
    /// An empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Number of parameters declared through this store.
    pub fn len(&self) -> usize {
        self.handles.len()
    }

    /// `true` if nothing was declared.
    pub fn is_empty(&self) -> bool {
        self.handles.is_empty()
    }

    /// Read-only `f64` in `[lower, upper]`.
    pub fn f64(
        &mut self,
        node: &Node,
        name: &str,
        default: f64,
        lower: f64,
        upper: f64,
        description: &str,
    ) -> Result<f64, RclrsError> {
        let p: ReadOnlyParameter<f64> = node
            .declare_parameter(name)
            .default(default)
            .range(ParameterRange {
                lower: Some(lower),
                upper: Some(upper),
                step: None,
            })
            .description(description)
            .read_only()?;
        let v = p.get();
        self.handles.push(Box::new(p));
        Ok(v)
    }

    /// Read-only `i64` in `[lower, upper]`.
    pub fn i64(
        &mut self,
        node: &Node,
        name: &str,
        default: i64,
        lower: i64,
        upper: i64,
        description: &str,
    ) -> Result<i64, RclrsError> {
        let p: ReadOnlyParameter<i64> = node
            .declare_parameter(name)
            .default(default)
            .range(ParameterRange {
                lower: Some(lower),
                upper: Some(upper),
                step: None,
            })
            .description(description)
            .read_only()?;
        let v = p.get();
        self.handles.push(Box::new(p));
        Ok(v)
    }

    /// Read-only `bool`.
    pub fn bool(
        &mut self,
        node: &Node,
        name: &str,
        default: bool,
        description: &str,
    ) -> Result<bool, RclrsError> {
        let p: ReadOnlyParameter<bool> = node
            .declare_parameter(name)
            .default(default)
            .description(description)
            .read_only()?;
        let v = p.get();
        self.handles.push(Box::new(p));
        Ok(v)
    }

    /// Read-only non-empty string.
    pub fn string(
        &mut self,
        node: &Node,
        name: &str,
        default: &str,
        description: &str,
    ) -> Result<String, RclrsError> {
        let p: ReadOnlyParameter<Arc<str>> = node
            .declare_parameter(name)
            .default(default.into())
            .description(description)
            .read_only()?;
        let v = p.get().to_string();
        self.handles.push(Box::new(p));
        if v.is_empty() {
            rclrs::log_error!(node.logger(), "parameter {name} must not be empty");
            return Err(RclrsError::RclError {
                code: rclrs::RclReturnCode::InvalidArgument,
                msg: None,
            });
        }
        Ok(v)
    }
}
