pub mod baseline;
pub mod command;
pub mod config;
pub mod error;
pub mod geo;
pub mod graph;
pub mod map;
pub mod prelude;
pub mod rrt;
pub mod rtsp;
pub mod system;
pub mod types;

#[cfg(feature = "python")]
pub mod python;

// Legacy storage module (PyArrow demo) — behind feature flag
#[cfg(feature = "pyarrow-demo")]
mod storage;

#[cfg(feature = "python")]
use pyo3::prelude::*;

/// Python module implemented in Rust (maturin entry point).
#[cfg(feature = "python")]
#[pymodule]
fn IMOMD_RRTStar(_py: Python, m: &PyModule) -> PyResult<()> {
    python::register(m)
}
