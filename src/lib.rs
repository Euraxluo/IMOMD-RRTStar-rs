use pyo3::prelude::*;

/// native rust function
pub fn sum(a: usize, b: usize) -> String {
    println!("call in rust {:?} {:?}", a, b);
    (a + b).to_string()
}

/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
    Ok(sum(a, b))
}

/// A Python module implemented in Rust.
#[pymodule]
fn IMOMD_RRTStar(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    Ok(())
}