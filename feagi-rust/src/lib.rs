use ndarray::{Array1, Array2};
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;
use rayon::prelude::*;

/// Error type for FEAGI Rust operations
#[derive(Debug, thiserror::Error)]
enum FeagiError {
    #[error("Invalid input dimensions: {0}")]
    InvalidDimensions(String),
    
    #[error("Computation error: {0}")]
    ComputationError(String),
}

/// A simple function that adds a value to each element of an array
#[pyfunction]
fn add_to_array(array: Vec<f64>, value: f64) -> PyResult<Vec<f64>> {
    let result = array.into_iter().map(|x| x + value).collect();
    Ok(result)
}

/// Perform a fast matrix-vector multiplication using Rust and Rayon
#[pyfunction]
fn fast_matrix_vector_mul(matrix: Vec<Vec<f64>>, vector: Vec<f64>) -> PyResult<Vec<f64>> {
    // Validate input dimensions
    if matrix.is_empty() || vector.is_empty() {
        return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
            "Empty matrix or vector provided",
        ));
    }
    
    let n_cols = matrix[0].len();
    if vector.len() != n_cols {
        return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
            format!("Matrix columns ({}) and vector length ({}) must match", n_cols, vector.len()),
        ));
    }
    
    // Convert to ndarray for better performance
    let mut ndmatrix = Vec::with_capacity(matrix.len());
    for row in &matrix {
        if row.len() != n_cols {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "All rows in matrix must have the same length",
            ));
        }
        ndmatrix.push(row.clone());
    }
    
    // Perform the multiplication in parallel
    let result: Vec<f64> = ndmatrix
        .par_iter()
        .map(|row| {
            row.iter()
                .zip(vector.iter())
                .map(|(a, b)| a * b)
                .sum()
        })
        .collect();
    
    Ok(result)
}

/// Perform activation function (ReLU) on an array
#[pyfunction]
fn relu(array: Vec<f64>) -> Vec<f64> {
    array.into_iter().map(|x| if x > 0.0 { x } else { 0.0 }).collect()
}

/// Perform activation function (Sigmoid) on an array
#[pyfunction]
fn sigmoid(array: Vec<f64>) -> Vec<f64> {
    array.into_iter().map(|x| 1.0 / (1.0 + (-x).exp())).collect()
}

/// Module containing fast neural network operations
#[pymodule]
fn feagi_rust(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(add_to_array, m)?)?;
    m.add_function(wrap_pyfunction!(fast_matrix_vector_mul, m)?)?;
    m.add_function(wrap_pyfunction!(relu, m)?)?;
    m.add_function(wrap_pyfunction!(sigmoid, m)?)?;
    
    Ok(())
} 