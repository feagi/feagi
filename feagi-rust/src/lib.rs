use ndarray::{Array1, Array2};
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;
use rayon::prelude::*;

/// Error type for FEAGI Rust operations
#[derive(Debug, thiserror::Error)]
pub enum FeagiError {
    #[error("Invalid input dimensions: {0}")]
    InvalidDimensions(String),
    
    #[error("Computation error: {0}")]
    ComputationError(String),
    
    #[error("Neural processing error: {0}")]
    NeuralProcessingError(String),
    
    #[error("Memory allocation error: {0}")]
    MemoryAllocationError(String),
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

/// Neural processing module
mod neural {
    use super::*;
    use pyo3::PyResult;
    use ndarray::{Array1, Array2, ArrayView1, Axis};
    use numpy::{IntoPyArray, PyArray1, PyArray2, PyReadonlyArray1, PyReadonlyArray2};
    use pyo3::Python;
    
    /// Structure representing a neuron for efficient processing
    #[derive(Debug, Clone)]
    pub struct Neuron {
        pub id: usize,
        pub membrane_potential: f32,
        pub threshold: f32,
        pub decay_rate: f32,
        pub refractory_period: u16,
        pub refractory_countdown: u16,
    }
    
    impl Neuron {
        /// Create a new neuron
        pub fn new(id: usize, threshold: f32, decay_rate: f32, refractory_period: u16) -> Self {
            Self {
                id,
                membrane_potential: 0.0,
                threshold,
                decay_rate,
                refractory_period,
                refractory_countdown: 0,
            }
        }
        
        /// Update the neuron's state
        pub fn update(&mut self, input: f32) -> bool {
            // If in refractory period, decrement countdown and don't fire
            if self.refractory_countdown > 0 {
                self.refractory_countdown -= 1;
                return false;
            }
            
            // Update membrane potential
            self.membrane_potential = self.membrane_potential * self.decay_rate + input;
            
            // Check if neuron should fire
            if self.membrane_potential >= self.threshold {
                // Reset membrane potential
                self.membrane_potential = 0.0;
                
                // Enter refractory period
                self.refractory_countdown = self.refractory_period;
                
                return true;
            }
            
            false
        }
    }
    
    /// Update membrane potentials for a batch of neurons
    /// 
    /// This is a high-performance implementation that can be called from Python
    #[pyfunction]
    pub fn update_membrane_potentials(
        py: Python,
        neuron_ids: PyReadonlyArray1<usize>,
        current_potentials: PyReadonlyArray1<f32>,
        synaptic_inputs: PyReadonlyArray1<f32>,
        decay_factor: f32,
        thresholds: PyReadonlyArray1<f32>,
    ) -> PyResult<(Py<PyArray1<f32>>, Py<PyArray1<usize>>)> {
        // Convert Python arrays to Rust arrays
        let neuron_ids = neuron_ids.as_array();
        let current_potentials = current_potentials.as_array();
        let synaptic_inputs = synaptic_inputs.as_array();
        let thresholds = thresholds.as_array();
        
        // Validate input dimensions
        if current_potentials.len() != synaptic_inputs.len() || 
           current_potentials.len() != neuron_ids.len() ||
           current_potentials.len() != thresholds.len() {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Input arrays must have the same length",
            ));
        }
        
        // Allocate output arrays
        let mut new_potentials = Array1::<f32>::zeros(current_potentials.len());
        let mut fired_neurons = Vec::new();
        
        // Update potentials and check firing
        for i in 0..current_potentials.len() {
            // Update potential
            let new_potential = current_potentials[i] * decay_factor + synaptic_inputs[i];
            new_potentials[i] = new_potential;
            
            // Check if neuron fired
            if new_potential >= thresholds[i] {
                fired_neurons.push(neuron_ids[i]);
                new_potentials[i] = 0.0;  // Reset potential after firing
            }
        }
        
        // Convert fired neurons to array
        let fired_neurons_array = Array1::<usize>::from_vec(fired_neurons);
        
        // Convert Rust arrays back to Python arrays
        Ok((
            new_potentials.into_pyarray(py).to_owned(),
            fired_neurons_array.into_pyarray(py).to_owned(),
        ))
    }
    
    /// Implement synaptic plasticity
    #[pyfunction]
    pub fn apply_plasticity(
        py: Python,
        synapse_weights: PyReadonlyArray1<f32>,
        plasticity_factors: PyReadonlyArray1<f32>,
        plasticity_types: PyReadonlyArray1<u8>,
        dt: f32,
    ) -> PyResult<Py<PyArray1<f32>>> {
        // Convert Python arrays to Rust arrays
        let weights = synapse_weights.as_array();
        let factors = plasticity_factors.as_array();
        let types = plasticity_types.as_array();
        
        // Validate input dimensions
        if weights.len() != factors.len() || weights.len() != types.len() {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Input arrays must have the same length",
            ));
        }
        
        // Allocate output array
        let mut new_weights = Array1::<f32>::zeros(weights.len());
        
        // Apply plasticity based on type
        for i in 0..weights.len() {
            match types[i] {
                0 => {
                    // No plasticity
                    new_weights[i] = weights[i];
                }
                1 => {
                    // Short-term plasticity (STP)
                    let new_weight = weights[i] * factors[i].powf(dt);
                    new_weights[i] = new_weight.max(0.0).min(255.0);
                }
                2 => {
                    // Long-term potentiation/depression (LTP/LTD)
                    let change = weights[i] * factors[i] * dt;
                    let new_weight = weights[i] + change;
                    new_weights[i] = new_weight.max(0.0).min(255.0);
                }
                _ => {
                    // Unknown plasticity type
                    new_weights[i] = weights[i];
                }
            }
        }
        
        // Convert Rust array back to Python array
        Ok(new_weights.into_pyarray(py).to_owned())
    }
}

/// Module containing fast neural network operations
#[pymodule]
fn feagi_rust(_py: Python, m: &PyModule) -> PyResult<()> {
    // Core operations
    m.add_function(wrap_pyfunction!(add_to_array, m)?)?;
    m.add_function(wrap_pyfunction!(fast_matrix_vector_mul, m)?)?;
    m.add_function(wrap_pyfunction!(relu, m)?)?;
    m.add_function(wrap_pyfunction!(sigmoid, m)?)?;
    
    // Neural module
    let neural_module = PyModule::new(_py, "neural")?;
    neural_module.add_function(wrap_pyfunction!(neural::update_membrane_potentials, neural_module)?)?;
    neural_module.add_function(wrap_pyfunction!(neural::apply_plasticity, neural_module)?)?;
    m.add_submodule(neural_module)?;
    
    // Add version information
    m.add("__version__", "0.1.0")?;
    
    Ok(())
} 