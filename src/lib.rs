#![allow(non_snake_case)] // Preserve the published Python/Rust crate import name.

pub mod baseline;
pub mod command;
pub mod config;
pub mod error;
pub mod experiment;
pub mod geo;
pub mod graph;
pub mod map;
pub mod navigation;
pub mod prelude;
pub mod rrt;
pub mod rtsp;
pub mod system;
pub mod types;

#[cfg(feature = "python")]
pub mod python;

// Legacy PyArrow demo removed — see git history if needed.

#[cfg(feature = "python")]
use pyo3::prelude::*;

/// Python module implemented in Rust (maturin entry point).
#[cfg(feature = "python")]
#[pymodule]
fn _imomd_native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    python::register(m)
}
