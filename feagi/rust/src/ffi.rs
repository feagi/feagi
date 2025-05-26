/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

use pyo3::prelude::*;
use pyo3::types::{PyList, PyDict};
use pyo3::wrap_pyfunction;

use crate::data_structures::{
    GlobalNeuronArray, FireCandidateList, Connectome, FeagiCore
};

/// Python module for FEAGI Rust optimized data structures
#[pymodule]
pub fn feagi_rust(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<PyGlobalNeuronArray>()?;
    m.add_class::<PyFireCandidateList>()?;
    m.add_class::<PyConnectome>()?;
    m.add_class::<PyFeagiCore>()?;
    
    m.add_function(wrap_pyfunction!(create_gna, m)?)?;
    m.add_function(wrap_pyfunction!(create_fcl, m)?)?;
    m.add_function(wrap_pyfunction!(create_connectome, m)?)?;
    m.add_function(wrap_pyfunction!(create_feagi_core, m)?)?;
    
    Ok(())
}

/// Python wrapper for GlobalNeuronArray
#[pyclass]
struct PyGlobalNeuronArray {
    gna: GlobalNeuronArray,
}

#[pymethods]
impl PyGlobalNeuronArray {
    /// Creates a new GNA with the specified capacity
    #[new]
    fn new(capacity: usize) -> Self {
        Self {
            gna: GlobalNeuronArray::new(capacity),
        }
    }
    
    /// Gets the number of neurons in the GNA
    #[getter]
    fn get_neuron_count(&self) -> usize {
        self.gna.neuron_count
    }
    
    /// Gets a neuron's membrane potential
    fn get_membrane_potential(&self, neuron_id: usize) -> PyResult<f32> {
        if neuron_id >= self.gna.neuron_count {
            return Err(PyErr::new::<pyo3::exceptions::PyIndexError, _>(
                format!("Neuron ID {} out of range", neuron_id),
            ));
        }
        
        Ok(self.gna.membrane_potentials[neuron_id])
    }
    
    /// Sets a neuron's membrane potential
    fn set_membrane_potential(&mut self, neuron_id: usize, value: f32) -> PyResult<()> {
        if neuron_id >= self.gna.neuron_count {
            return Err(PyErr::new::<pyo3::exceptions::PyIndexError, _>(
                format!("Neuron ID {} out of range", neuron_id),
            ));
        }
        
        self.gna.membrane_potentials[neuron_id] = value;
        Ok(())
    }
    
    /// Gets all membrane potentials as a Python list
    fn get_all_membrane_potentials(&self, py: Python) -> PyResult<Py<PyList>> {
        let potentials = PyList::new(
            py,
            &self.gna.membrane_potentials[..self.gna.neuron_count],
        );
        Ok(potentials.into())
    }
    
    /// Update membrane potentials with a decay factor
    fn decay_membrane_potentials(&mut self, decay_factor: f32) {
        self.gna.update_membrane_potentials_simd(decay_factor);
    }
    
    /// Gets neurons that are ready to fire
    fn get_fire_candidates(&self, py: Python, timestep: u32) -> Py<PyList> {
        let candidates = self.gna.find_fire_candidates(timestep);
        let py_list = PyList::new(py, &candidates);
        py_list.into()
    }
    
    /// Process fired neurons
    fn process_fired_neurons(&mut self, fired_list: Vec<u32>, timestep: u32) {
        self.gna.process_fired_neurons(&fired_list, timestep);
    }
    
    /// Update refractory counters
    fn update_refractory_counters(&mut self) {
        self.gna.update_refractory_counters();
    }
}

/// Python wrapper for FireCandidateList
#[pyclass]
struct PyFireCandidateList {
    fcl: FireCandidateList,
}

#[pymethods]
impl PyFireCandidateList {
    /// Creates a new empty FCL
    #[new]
    fn new() -> Self {
        Self {
            fcl: FireCandidateList::new(),
        }
    }
    
    /// Creates an FCL from a list of neuron IDs
    #[staticmethod]
    fn from_list(neuron_ids: Vec<u32>) -> Self {
        Self {
            fcl: FireCandidateList::from_neuron_ids(&neuron_ids),
        }
    }
    
    /// Adds a neuron to the FCL
    fn add(&mut self, neuron_id: u32) {
        self.fcl.add(neuron_id);
    }
    
    /// Adds multiple neurons to the FCL
    fn add_multiple(&mut self, neuron_ids: Vec<u32>) {
        self.fcl.add_multiple(&neuron_ids);
    }
    
    /// Removes a neuron from the FCL
    fn remove(&mut self, neuron_id: u32) {
        self.fcl.remove(neuron_id);
    }
    
    /// Clears the FCL
    fn clear(&mut self) {
        self.fcl.clear();
    }
    
    /// Checks if a neuron is in the FCL
    fn contains(&self, neuron_id: u32) -> bool {
        self.fcl.contains(neuron_id)
    }
    
    /// Gets the number of neurons in the FCL
    fn len(&self) -> usize {
        self.fcl.len() as usize
    }
    
    /// Checks if the FCL is empty
    fn is_empty(&self) -> bool {
        self.fcl.is_empty()
    }
    
    /// Gets all neurons in the FCL as a Python list
    fn to_list(&self, py: Python) -> Py<PyList> {
        let vec = self.fcl.to_vec();
        PyList::new(py, &vec).into()
    }
}

/// Python wrapper for Connectome
#[pyclass]
struct PyConnectome {
    connectome: Connectome,
}

#[pymethods]
impl PyConnectome {
    /// Creates a new empty connectome
    #[new]
    fn new(neuron_count: usize, estimated_edges: usize) -> Self {
        Self {
            connectome: Connectome::new(neuron_count, estimated_edges),
        }
    }
    
    /// Adds a connection to the connectome
    fn add_connection(
        &mut self,
        source_id: u32,
        target_id: u32,
        weight: f32,
        delay: u8,
        connection_type: u8,
        source_area_id: u32,
        target_area_id: u32,
    ) -> PyResult<()> {
        match self.connectome.add_connection(
            source_id,
            target_id,
            weight,
            delay,
            connection_type,
            source_area_id,
            target_area_id,
        ) {
            Ok(()) => Ok(()),
            Err(e) => Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(e)),
        }
    }
    
    /// Gets all connections for a specific neuron
    fn get_connections_for_neuron(&self, py: Python, neuron_id: u32) -> PyResult<Py<PyList>> {
        let connections = self.connectome.get_connections_for_neuron(neuron_id);
        
        let py_list: &PyList = PyList::empty(py);
        
        for (target_id, weight, delay, conn_type, source_area, target_area) in connections.iter() {
            let dict = PyDict::new(py);
            dict.set_item("target_id", target_id)?;
            dict.set_item("weight", weight)?;
            dict.set_item("delay", delay)?;
            dict.set_item("type", conn_type)?;
            dict.set_item("source_area_id", source_area)?;
            dict.set_item("target_area_id", target_area)?;
            
            py_list.append(dict)?;
        }
        
        Ok(py_list.into())
    }
    
    /// Gets the total number of connections in the connectome
    fn connection_count(&self) -> usize {
        self.connectome.connection_count()
    }
    
    /// Propagates activations through the connectome
    fn propagate_activations(
        &self,
        py: Python,
        source_activations: Vec<f32>,
        mut target_buffer: Vec<f32>,
    ) -> Py<PyList> {
        // Propagate activations
        self.connectome.propagate_activations_simd(&source_activations, &mut target_buffer);
        
        // Convert result to Python list
        PyList::new(py, &target_buffer).into()
    }
}

/// Python wrapper for FeagiCore
#[pyclass]
struct PyFeagiCore {
    core: FeagiCore,
}

#[pymethods]
impl PyFeagiCore {
    /// Creates a new FEAGI core
    #[new]
    fn new(neuron_capacity: usize, estimated_connections: usize) -> Self {
        Self {
            core: FeagiCore::new(neuron_capacity, estimated_connections),
        }
    }
    
    /// Performs a single simulation timestep
    fn step(&mut self) {
        self.core.step();
    }
    
    /// Performs a single simulation timestep using the fire queue process
    fn step_with_fire_queue(
        &mut self, 
        mpf: bool,
        puf: bool,
        max_consecutive_fires: u32
    ) {
        self.core.step_with_fire_queue(mpf, puf, max_consecutive_fires);
    }
    
    /// Gets the current timestep
    #[getter]
    fn get_current_timestep(&self) -> u64 {
        self.core.current_timestep
    }
    
    /// Gets the GNA object
    #[getter]
    fn get_gna(&self, _py: Python) -> PyResult<PyGlobalNeuronArray> {
        // We can't directly expose the internal GNA, so we create a copy
        let mut gna = GlobalNeuronArray::new(self.core.gna.neuron_count);
        
        // Copy data fields
        gna.membrane_potentials.copy_from_slice(&self.core.gna.membrane_potentials[..self.core.gna.neuron_count]);
        gna.thresholds.copy_from_slice(&self.core.gna.thresholds[..self.core.gna.neuron_count]);
        gna.refractory_periods.copy_from_slice(&self.core.gna.refractory_periods[..self.core.gna.neuron_count]);
        gna.last_fired.copy_from_slice(&self.core.gna.last_fired[..self.core.gna.neuron_count]);
        gna.refractory_counters.copy_from_slice(&self.core.gna.refractory_counters[..self.core.gna.neuron_count]);
        
        Ok(PyGlobalNeuronArray { gna })
    }
    
    /// Gets the FCL object
    #[getter]
    fn get_fcl(&self, _py: Python) -> PyResult<PyFireCandidateList> {
        // We can't directly expose the internal FCL, so we create a copy
        let fcl = self.core.fcl.clone();
        
        Ok(PyFireCandidateList { fcl })
    }
    
    /// Propagates activations through the connectome
    fn propagate_activations(&self, py: Python) -> Py<PyList> {
        let mut target_buffer = vec![0.0f32; self.core.gna.neuron_count];
        
        self.core.propagate_activations(&mut target_buffer);
        
        PyList::new(py, &target_buffer).into()
    }
}

/// Creates a new GNA with the specified capacity
#[pyfunction]
fn create_gna(capacity: usize) -> PyGlobalNeuronArray {
    PyGlobalNeuronArray::new(capacity)
}

/// Creates a new FCL from a list of neuron IDs
#[pyfunction]
fn create_fcl(neuron_ids: Vec<u32>) -> PyFireCandidateList {
    PyFireCandidateList::from_list(neuron_ids)
}

/// Creates a new connectome
#[pyfunction]
fn create_connectome(neuron_count: usize, estimated_edges: usize) -> PyConnectome {
    PyConnectome::new(neuron_count, estimated_edges)
}

/// Creates a new FEAGI core
#[pyfunction]
fn create_feagi_core(neuron_capacity: usize, estimated_connections: usize) -> PyFeagiCore {
    PyFeagiCore::new(neuron_capacity, estimated_connections)
} 