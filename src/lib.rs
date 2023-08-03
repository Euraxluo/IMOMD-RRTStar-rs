mod command;
pub mod config;
pub mod prelude;
mod storage;

use log::debug;
use polars_core::prelude::DataFrame;
use prelude::*;
use pyo3::prelude::*;

/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
    Ok(sum(a, b))
}

// 定义一个Python绑定函数，该函数将接受PyArrow表作为参数，并将其传递给Rust的process_arrow_table函数
#[pyfunction]
fn process_pyarrow_table(arrow_table: &PyAny) -> PyResult<()> {
    let pl_df: DataFrame = storage::pyarrow_to_polars_df(&arrow_table)?;
    println!("Arrow2 DataFrame: {}", pl_df);
    Ok(())
}

/// A Python module implemented in Rust.
#[pymodule]
fn IMOMD_RRTStar(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    m.add_function(wrap_pyfunction!(process_pyarrow_table, m)?)?;
    Ok(())
}
