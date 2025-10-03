use pyo3::prelude::*;
use pyo3::exceptions::PyValueError;
use pyo3::types::PyModule;
use numpy::{IntoPyArray, PyArray3, PyReadonlyArray3};
use feagi_core_sensiomotor_functions::ImageDiff;
use ndarray::{Array3, ArrayView3};

#[pyclass]
#[pyo3(name = "ImageDiff")]
struct PyImageDiff {
    image_diff: ImageDiff,
}

#[pymethods]
impl PyImageDiff {
    #[new]
    fn new(image_width: usize, image_height: usize, color_depth: usize) -> PyResult<Self> {
        // TODO error handling
        Ok(Self {
            image_diff: ImageDiff::new(image_width, image_height, color_depth),
        })
    }

    /// Returns the shape of the image as (height, width, channels)
    fn shape(&self) -> (usize, usize, usize) {
        let rust_shape = self.image_diff.shape();
        // Convert from Rust's (channels, height, width) to Python's (height, width, channels) 
        (rust_shape.0, rust_shape.1, rust_shape.2)
    }

    
    /// Returns the new delta from the new frame
    #[pyo3(text_signature = "(self, input)")]
    fn get_new_delta_from_new_frame<'py>(&mut self, py: Python<'py>, input: PyReadonlyArray3<'py, u8>) -> PyResult<Bound<'py, PyArray3<u8>>> {
        let input_array: ArrayView3<u8> = input.as_array();  // Convert from PyReadonlyArray3 to owned ArrayView3
        
        // Convert from ArrayView3 to owned Array3, with proper error handling
        let owned_input: Array3<u8> = match Array3::from_shape_vec(
            input_array.raw_dim(),
            input_array.iter().cloned().collect(),
        ) {
            Ok(array) => array,
            Err(e) => return Err(PyValueError::new_err(format!("Failed to convert input array: {}", e))),
        };
        
        let output: Result<Array3<u8>, String> = self.image_diff.get_new_delta_from_new_frame(&owned_input);

        match output {
            Ok(output_array) => Ok(output_array.into_pyarray(py)),
            Err(e) => Err(PyValueError::new_err(format!("Error in get_new_delta_from_new_frame: {}", e)))
        }
    }
    
}


#[pymodule]
fn rust_core_sensiomotor_functions_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyImageDiff>()?;
    m.add("__doc__", "Python bindings for FEAGI image processing functions")?;
    
    Ok(())
}
