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
use feagi_burst_engine::{SynapticPropagationEngine as RustEngine, RustNPU as RustNPUCore, BurstResult as RustBurstResult};
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

/// Python wrapper for BurstResult
#[pyclass]
#[derive(Clone)]
struct BurstResult {
    #[pyo3(get)]
    fired_neurons: Vec<u32>,
    
    #[pyo3(get)]
    neuron_count: usize,
    
    #[pyo3(get)]
    burst: u64,
    
    #[pyo3(get)]
    power_injections: usize,
    
    #[pyo3(get)]
    synaptic_injections: usize,
    
    #[pyo3(get)]
    neurons_processed: usize,
    
    #[pyo3(get)]
    neurons_in_refractory: usize,
}

impl From<RustBurstResult> for BurstResult {
    fn from(result: RustBurstResult) -> Self {
        Self {
            fired_neurons: result.fired_neurons.iter().map(|id| id.0).collect(),
            neuron_count: result.neuron_count,
            burst: result.burst,
            power_injections: result.power_injections,
            synaptic_injections: result.synaptic_injections,
            neurons_processed: result.neurons_processed,
            neurons_in_refractory: result.neurons_in_refractory,
        }
    }
}

/// Python wrapper for the complete Rust NPU
/// 
/// This is the MAIN HIGH-PERFORMANCE NPU that processes bursts entirely in Rust.
/// 
/// ## Usage from Python
/// ```python
/// import feagi_rust
/// import numpy as np
/// 
/// # Create NPU
/// npu = feagi_rust.RustNPU(
///     neuron_count=10000,
///     synapse_capacity=100000,
///     fire_ledger_window=20
/// )
/// 
/// # Add neurons
/// for i in range(10):
///     npu.add_neuron(
///         threshold=1.0,
///         leak_rate=0.1,
///         refractory_period=5,
///         excitability=1.0,
///         cortical_area=1,
///         x=i, y=0, z=0
///     )
/// 
/// # Add synapses
/// npu.add_synapse(
///     source=0,
///     target=1,
///     weight=128,
///     conductance=255,
///     synapse_type=0  # 0=excitatory, 1=inhibitory
/// )
/// 
/// # Rebuild indexes after bulk modifications
/// npu.rebuild_indexes()
/// 
/// # Set neuron mapping
/// npu.set_neuron_mapping({0: 1, 1: 1, 2: 1})
/// 
/// # Process burst (ALL IN RUST!)
/// result = npu.process_burst(power_neurons=[0])
/// print(f"Burst {result.burst}: {result.neuron_count} neurons fired")
/// print(f"Fired: {result.fired_neurons}")
/// ```
#[pyclass]
struct RustNPU {
    npu: RustNPUCore,
}

#[pymethods]
impl RustNPU {
    /// Create a new Rust NPU
    /// 
    /// Args:
    ///     neuron_capacity: Maximum number of neurons (e.g., 100000)
    ///     synapse_capacity: Maximum number of synapses (e.g., 1000000)
    ///     fire_ledger_window: Number of historical bursts to keep (e.g., 20)
    #[new]
    fn new(neuron_capacity: usize, synapse_capacity: usize, fire_ledger_window: usize) -> Self {
        Self {
            npu: RustNPUCore::new(neuron_capacity, synapse_capacity, fire_ledger_window),
        }
    }
    
    /// Set power injection amount (default: 1.0)
    fn set_power_amount(&mut self, amount: f32) {
        self.npu.set_power_amount(amount);
    }
    
    /// Add a neuron to the NPU
    /// 
    /// Args:
    ///     threshold: Firing threshold (e.g., 1.0)
    ///     leak_rate: Membrane potential decay rate (0.0-1.0, e.g., 0.1)
    ///     refractory_period: Bursts to wait after firing (e.g., 5)
    ///     excitability: Probabilistic firing factor (0.0-1.0, e.g., 1.0)
    ///     cortical_area: Cortical area ID (e.g., 1)
    ///     x, y, z: 3D coordinates in cortical area
    /// 
    /// Returns:
    ///     Neuron ID (uint32)
    fn add_neuron(
        &mut self,
        threshold: f32,
        leak_rate: f32,
        refractory_period: u16,
        excitability: f32,
        cortical_area: u32,
        x: u32,
        y: u32,
        z: u32,
    ) -> PyResult<u32> {
        self.npu
            .add_neuron(threshold, leak_rate, refractory_period, excitability, cortical_area, x, y, z)
            .map(|id| id.0)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }
    
    /// Add a synapse to the NPU
    /// 
    /// Args:
    ///     source: Source neuron ID
    ///     target: Target neuron ID
    ///     weight: Synaptic weight (0-255)
    ///     conductance: Synaptic conductance (0-255)
    ///     synapse_type: 0=excitatory, 1=inhibitory
    /// 
    /// Returns:
    ///     Synapse index (usize)
    fn add_synapse(
        &mut self,
        source: u32,
        target: u32,
        weight: u8,
        conductance: u8,
        synapse_type: u8,
    ) -> PyResult<usize> {
        self.npu
            .add_synapse(
                NeuronId(source),
                NeuronId(target),
                SynapticWeight(weight),
                SynapticConductance(conductance),
                SynapseType::from_int(synapse_type),
            )
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }
    
    /// Remove a synapse
    /// 
    /// Args:
    ///     source: Source neuron ID
    ///     target: Target neuron ID
    /// 
    /// Returns:
    ///     True if removed, False if not found
    fn remove_synapse(&mut self, source: u32, target: u32) -> bool {
        self.npu.remove_synapse(NeuronId(source), NeuronId(target))
    }
    
    /// Update synapse weight
    /// 
    /// Args:
    ///     source: Source neuron ID
    ///     target: Target neuron ID
    ///     new_weight: New synaptic weight (0-255)
    /// 
    /// Returns:
    ///     True if updated, False if not found
    fn update_synapse_weight(&mut self, source: u32, target: u32, new_weight: u8) -> bool {
        self.npu.update_synapse_weight(NeuronId(source), NeuronId(target), SynapticWeight(new_weight))
    }
    
    /// Rebuild indexes after bulk modifications
    /// 
    /// Call this after adding/removing many synapses for optimal performance
    fn rebuild_indexes(&mut self) {
        self.npu.rebuild_indexes();
    }
    
    /// Set neuron to cortical area mapping
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
        
        self.npu.set_neuron_mapping(rust_mapping);
        Ok(())
    }
    
    /// Process a single burst (MAIN METHOD - ALL IN RUST!)
    /// 
    /// This is the complete neural processing pipeline:
    /// Phase 1: Injection → Phase 2: Dynamics → Phase 3: Archival → Phase 5: Cleanup
    /// 
    /// Args:
    ///     power_neurons: List of neuron IDs to inject power into (e.g., [0, 1, 2])
    /// 
    /// Returns:
    ///     BurstResult with fired_neurons, burst number, and performance metrics
    fn process_burst(&mut self, power_neurons: Vec<u32>) -> PyResult<BurstResult> {
        let power_neuron_ids: Vec<NeuronId> = power_neurons.iter().map(|&id| NeuronId(id)).collect();
        
        self.npu
            .process_burst(&power_neuron_ids)
            .map(BurstResult::from)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }
    
    /// Get current burst count
    fn get_burst_count(&self) -> u64 {
        self.npu.get_burst_count()
    }
    
    /// Get neuron count
    fn get_neuron_count(&self) -> usize {
        self.npu.get_neuron_count()
    }
    
    /// Get synapse count (valid only)
    fn get_synapse_count(&self) -> usize {
        self.npu.get_synapse_count()
    }
}

/// Module containing fast neural network operations
#[pymodule]
fn feagi_rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Add the complete Rust NPU (NEW!)
    m.add_class::<RustNPU>()?;
    m.add_class::<BurstResult>()?;
    
    // Add the synaptic propagation engine (legacy, for compatibility)
    m.add_class::<SynapticPropagationEngine>()?;

    // Add version information
    m.add("__version__", "0.2.0")?;

    Ok(())
}
