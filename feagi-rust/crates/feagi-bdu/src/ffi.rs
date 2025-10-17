/*!
FFI bindings for Python integration via PyO3.

Exposes Rust functions to Python for seamless interop.
*/

use pyo3::prelude::*;

/// Python-exposed version of syn_projector
///
/// # Arguments (from Python)
///
/// * `src_area_id` - str
/// * `dst_area_id` - str  
/// * `src_neuron_id` - int
/// * `src_dimensions` - tuple[int, int, int]
/// * `dst_dimensions` - tuple[int, int, int]
/// * `neuron_location` - tuple[int, int, int]
/// * `transpose` - Optional[tuple[int, int, int]]
/// * `project_last_layer_of` - Optional[int]
///
/// # Returns
///
/// List[tuple[int, int, int]] - List of destination positions
#[pyfunction]
#[pyo3(signature = (src_area_id, dst_area_id, src_neuron_id, src_dimensions, dst_dimensions, neuron_location, transpose=None, project_last_layer_of=None))]
fn py_syn_projector(
    src_area_id: &str,
    dst_area_id: &str,
    src_neuron_id: u64,
    src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
    neuron_location: (i32, i32, i32),
    transpose: Option<(usize, usize, usize)>,
    project_last_layer_of: Option<usize>,
) -> PyResult<Vec<(i32, i32, i32)>> {
    let result = crate::connectivity::rules::syn_projector(
        src_area_id,
        dst_area_id,
        src_neuron_id,
        src_dimensions,
        dst_dimensions,
        neuron_location,
        transpose,
        project_last_layer_of,
    );

    match result {
        Ok(positions) => Ok(positions),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!(
            "Rust syn_projector error: {}",
            e
        ))),
    }
}

/// Python-exposed batch projector for parallel processing
///
/// # Arguments
///
/// * `src_area_id` - str
/// * `dst_area_id` - str
/// * `neuron_ids` - List[int]
/// * `neuron_locations` - List[tuple[int, int, int]]
/// * `src_dimensions` - tuple[int, int, int]
/// * `dst_dimensions` - tuple[int, int, int]
/// * `transpose` - Optional[tuple[int, int, int]]
/// * `project_last_layer_of` - Optional[int]
///
/// # Returns
///
/// List[List[tuple[int, int, int]]] - List of position lists (one per neuron)
#[pyfunction]
#[pyo3(signature = (src_area_id, dst_area_id, neuron_ids, neuron_locations, src_dimensions, dst_dimensions, transpose=None, project_last_layer_of=None))]
fn py_syn_projector_batch(
    src_area_id: &str,
    dst_area_id: &str,
    neuron_ids: Vec<u64>,
    neuron_locations: Vec<(i32, i32, i32)>,
    src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
    transpose: Option<(usize, usize, usize)>,
    project_last_layer_of: Option<usize>,
) -> PyResult<Vec<Vec<(i32, i32, i32)>>> {
    let result = crate::connectivity::rules::syn_projector_batch(
        src_area_id,
        dst_area_id,
        &neuron_ids,
        &neuron_locations,
        src_dimensions,
        dst_dimensions,
        transpose,
        project_last_layer_of,
    );

    match result {
        Ok(position_lists) => Ok(position_lists),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!(
            "Rust syn_projector_batch error: {}",
            e
        ))),
    }
}

/// Python module initialization (PyO3 0.22 API with Bound)
#[pymodule]
fn feagi_bdu(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Core functions
    m.add_function(wrap_pyfunction!(py_syn_projector, m)?)?;
    m.add_function(wrap_pyfunction!(py_syn_projector_batch, m)?)?;

    // Version info
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add("__doc__", "FEAGI BDU - High-performance Brain Development Utilities")?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ffi_compatibility() {
        // Smoke test to ensure FFI functions compile
        let result = py_syn_projector(
            "src",
            "dst",
            42,
            (128, 128, 3),
            (128, 128, 1),
            (64, 64, 1),
            None,
            None,
        );
        assert!(result.is_ok());
    }
}

