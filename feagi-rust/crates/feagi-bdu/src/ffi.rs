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

/// Block connection - maps blocks with scaling factor
#[pyfunction]
fn py_syn_block_connection(
    src_area_id: &str,
    dst_area_id: &str,
    neuron_location: (i32, i32, i32),
    src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
    scaling_factor: i32,
) -> PyResult<(i32, i32, i32)> {
    match crate::connectivity::rules::syn_block_connection(
        src_area_id, dst_area_id, neuron_location, src_dimensions, dst_dimensions, scaling_factor
    ) {
        Ok(pos) => Ok(pos),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", e))),
    }
}

/// Expander - scales coordinates from source to destination
#[pyfunction]
fn py_syn_expander(
    src_area_id: &str,
    dst_area_id: &str,
    neuron_location: (i32, i32, i32),
    src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
) -> PyResult<(i32, i32, i32)> {
    match crate::connectivity::rules::syn_expander(
        src_area_id, dst_area_id, neuron_location, src_dimensions, dst_dimensions
    ) {
        Ok(pos) => Ok(pos),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", e))),
    }
}

/// Expander batch - parallel processing
#[pyfunction]
fn py_syn_expander_batch(
    src_area_id: &str,
    dst_area_id: &str,
    neuron_locations: Vec<(i32, i32, i32)>,
    src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
) -> PyResult<Vec<(i32, i32, i32)>> {
    match crate::connectivity::rules::syn_expander_batch(
        src_area_id, dst_area_id, &neuron_locations, src_dimensions, dst_dimensions
    ) {
        Ok(positions) => Ok(positions),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", e))),
    }
}

/// Reducer - binary encoding to multiple positions
#[pyfunction]
fn py_syn_reducer_x(
    src_area_id: &str,
    dst_area_id: &str,
    neuron_location: (i32, i32, i32),
    src_dimensions: (usize, usize, usize),
    dst_dimensions: (usize, usize, usize),
    dst_y_index: i32,
    dst_z_index: i32,
) -> PyResult<Vec<(i32, i32, i32)>> {
    match crate::connectivity::rules::syn_reducer_x(
        src_area_id, dst_area_id, neuron_location, src_dimensions, dst_dimensions, dst_y_index, dst_z_index
    ) {
        Ok(positions) => Ok(positions),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{}", e))),
    }
}

/// Python module initialization (PyO3 0.22 API with Bound)
#[pymodule]
fn feagi_bdu(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Projector functions
    m.add_function(wrap_pyfunction!(py_syn_projector, m)?)?;
    m.add_function(wrap_pyfunction!(py_syn_projector_batch, m)?)?;
    
    // Phase 2 morphologies
    m.add_function(wrap_pyfunction!(py_syn_block_connection, m)?)?;
    m.add_function(wrap_pyfunction!(py_syn_expander, m)?)?;
    m.add_function(wrap_pyfunction!(py_syn_expander_batch, m)?)?;
    m.add_function(wrap_pyfunction!(py_syn_reducer_x, m)?)?;

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

