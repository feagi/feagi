pub mod data_structures;
pub mod ffi;

#[cfg(feature = "wgpu")]
pub mod gpu;

use pyo3::prelude::*;

/// Python module initialization
#[pymodule]
fn feagi_rust(_py: Python, m: &PyModule) -> PyResult<()> {
    ffi::feagi_rust(_py, m)
} 