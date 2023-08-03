use log::debug;
use polars_core::export::rayon::prelude::*;
use polars_core::prelude::*;
use polars_core::utils::accumulate_dataframes_vertical_unchecked;
use polars_core::utils::arrow::ffi;
use polars_core::POOL;
use pyo3::exceptions::PyRuntimeError;
use pyo3::ffi::Py_uintptr_t;
use pyo3::prelude::*;

pub fn array_to_rust(obj: &PyAny) -> PyResult<ArrayRef> {
    // prepare a pointer to receive the Array struct
    let array = Box::new(ffi::ArrowArray::empty());
    let schema = Box::new(ffi::ArrowSchema::empty());

    let array_ptr = &*array as *const ffi::ArrowArray;
    let schema_ptr = &*schema as *const ffi::ArrowSchema;

    // make the conversion through PyArrow's private API
    // this changes the pointer's memory and is thus unsafe. In particular, `_export_to_c` can go out of bounds
    obj.call_method1(
        "_export_to_c",
        (array_ptr as Py_uintptr_t, schema_ptr as Py_uintptr_t),
    )?;

    unsafe {
        let field = ffi::import_field_from_c(schema.as_ref())
            .map_err(|err| PyRuntimeError::new_err(format!("{:?}", &err)))?;
        let array = ffi::import_array_from_c(*array, field.data_type)
            .map_err(|err| PyRuntimeError::new_err(format!("{:?}", &err)))?;
        Ok(array)
    }
}

///
/// copy from `py-polars/src/arrow_interop/to_rust.rs`
/// UseAge:
/// ```rust
/// #[cfg(not(doctest))]
/// #[pyfunction]
/// fn process_pyarrow_table(arrow_table: &PyAny) -> PyResult<()> {
///     let pl_df: DataFrame = pyarrow_to_polars_df(&arrow_table)?;
///     println!("Arrow2 DataFrame: {}", pl_df);
///     Ok(())
/// }
/// ```
pub fn pyarrow_to_polars_df(arrow_table: &PyAny) -> PyResult<DataFrame> {
    let schema = arrow_table
        .getattr("schema")
        .expect("pyarrow.Table has no schema");
    debug!("arrow_table schema:{:?}", schema);
    let columns = arrow_table
        .getattr("columns")
        .expect("pyarrow.Table has no columns");
    debug!("arrow_table columns:{:?}", columns);

    let mut rb: Vec<&PyAny> = vec![];
    for item in arrow_table
        .call_method0("to_batches")
        .expect("pyarrow.Table has no method to_batches")
        .iter()?
    {
        rb.push(item?);
    }

    let names = schema.getattr("names")?.extract::<Vec<String>>()?;

    let dfs = rb
        .iter()
        .map(|rb| {
            let mut run_parallel = false;

            let columns = (0..names.len())
                .map(|i| {
                    let array = rb.call_method1("column", (i,))?;
                    let arr = array_to_rust(array)?;
                    run_parallel |= matches!(
                        arr.data_type(),
                        ArrowDataType::Utf8 | ArrowDataType::Dictionary(_, _, _)
                    );
                    Ok(arr)
                })
                .collect::<PyResult<Vec<_>>>()?;

            // we parallelize this part because we can have dtypes that are not zero copy
            // for instance utf8 -> large-utf8
            // dict encoded to categorical
            let columns = if run_parallel {
                POOL.install(|| {
                    columns
                        .into_par_iter()
                        .enumerate()
                        .map(|(i, arr)| {
                            let s = Series::try_from((names[i].as_str(), arr))
                                .map_err(|err| PyRuntimeError::new_err(format!("{:?}", &err)))?;
                            Ok(s)
                        })
                        .collect::<PyResult<Vec<_>>>()
                })
            } else {
                columns
                    .into_iter()
                    .enumerate()
                    .map(|(i, arr)| {
                        let s = Series::try_from((names[i].as_str(), arr))
                            .map_err(|err| PyRuntimeError::new_err(format!("{:?}", &err)))?;
                        Ok(s)
                    })
                    .collect::<PyResult<Vec<_>>>()
            }?;

            // no need to check as a record batch has the same guarantees
            Ok(DataFrame::new_no_checks(columns))
        })
        .collect::<PyResult<Vec<_>>>()?;

    Ok(accumulate_dataframes_vertical_unchecked(dfs))
}
