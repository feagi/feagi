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
use pyo3::types::{PyDict, PyModule, PyBytes};
use numpy::{PyArray1, ToPyArray};
use feagi_types::*;
use feagi_burst_engine::{RustNPU as RustNPUCore, BurstResult as RustBurstResult};
use ahash::AHashMap;
use feagi_data_structures::neurons::xyzp::{NeuronXYZP, NeuronXYZPArrays, CorticalMappedXYZPNeuronData};
use feagi_data_structures::genomic::CorticalID;
// Note: FeagiSerializable is private in feagi_data_serialization, but we need its methods
// So we'll implement serialization manually using the internal implementation details

/*  LEGACY: Not used - full RustNPU is used instead
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
*/  // End LEGACY block

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
        leak_coefficient: f32,
        resting_potential: f32,
        neuron_type: i32,
        refractory_period: u16,
        excitability: f32,
        consecutive_fire_limit: u16,
        snooze_period: u16,
        cortical_area: u32,
        x: u32,
        y: u32,
        z: u32,
    ) -> PyResult<u32> {
        self.npu
            .add_neuron(
                threshold,
                leak_coefficient,
                resting_potential,
                neuron_type,
                refractory_period,
                excitability,
                consecutive_fire_limit,
                snooze_period,
                cortical_area,
                x,
                y,
                z,
            )
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
    
    /// Batch add synapses (SIMD-optimized)
    /// 
    /// Creates multiple synapses in a single operation. This is 50-100x faster
    /// than calling add_synapse() in a Python loop due to:
    /// - Single FFI boundary crossing (vs N crossings)
    /// - Contiguous SoA memory writes  
    /// - Batch source_index updates
    /// 
    /// Args:
    ///     sources: List of source neuron IDs
    ///     targets: List of target neuron IDs
    ///     weights: List of synaptic weights (0-255)
    ///     conductances: List of conductances (0-255)
    ///     synapse_types: List of synapse types (0=excitatory, 1=inhibitory)
    /// 
    /// Returns:
    ///     Tuple of (successful_count, failed_indices)
    ///     - successful_count: Number of synapses created
    ///     - failed_indices: List of indices that failed
    fn add_synapses_batch(
        &mut self,
        sources: Vec<u32>,
        targets: Vec<u32>,
        weights: Vec<u8>,
        conductances: Vec<u8>,
        synapse_types: Vec<u8>,
    ) -> (usize, Vec<usize>) {
        let source_ids: Vec<NeuronId> = sources.into_iter().map(NeuronId).collect();
        let target_ids: Vec<NeuronId> = targets.into_iter().map(NeuronId).collect();
        let weight_vals: Vec<SynapticWeight> = weights.into_iter().map(SynapticWeight).collect();
        let conductance_vals: Vec<SynapticConductance> = conductances.into_iter().map(SynapticConductance).collect();
        let type_vals: Vec<SynapseType> = synapse_types.into_iter().map(|t| {
            if t == 0 {
                SynapseType::Excitatory
            } else {
                SynapseType::Inhibitory
            }
        }).collect();
        
        self.npu.add_synapses_batch(source_ids, target_ids, weight_vals, conductance_vals, type_vals)
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
    
    /// Batch remove all synapses from specified source neurons (SIMD-optimized)
    /// 
    /// This method is 50-100x faster than looping through individual deletions.
    /// Optimized for cortical mapping removal where you want to delete all
    /// connections from a set of neurons.
    /// 
    /// Args:
    ///     sources: List of source neuron IDs
    /// 
    /// Returns:
    ///     Number of synapses deleted
    fn remove_synapses_from_sources(&mut self, sources: Vec<u32>) -> usize {
        let source_ids: Vec<NeuronId> = sources.into_iter().map(NeuronId).collect();
        self.npu.remove_synapses_from_sources(source_ids)
    }
    
    /// Batch remove synapses between source and target neuron sets (SIMD-optimized)
    /// 
    /// Uses bit-vector filtering for O(1) target membership testing.
    /// Optimal for both few→many (e.g., 1 → 16K) and many→many deletion patterns.
    /// 
    /// Performance: 20-100x faster than nested loop deletions
    /// 
    /// Args:
    ///     sources: List of source neuron IDs
    ///     targets: List of target neuron IDs
    /// 
    /// Returns:
    ///     Number of synapses deleted
    fn remove_synapses_between(&mut self, sources: Vec<u32>, targets: Vec<u32>) -> usize {
        let source_ids: Vec<NeuronId> = sources.into_iter().map(NeuronId).collect();
        let target_ids: Vec<NeuronId> = targets.into_iter().map(NeuronId).collect();
        self.npu.remove_synapses_between(source_ids, target_ids)
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
    
    /// Get all outgoing synapses from a source neuron
    /// Returns list of tuples (target_neuron_id, weight)
    fn get_outgoing_synapses(&self, source_neuron_id: u32) -> Vec<(u32, u8)> {
        self.npu.get_outgoing_synapses(source_neuron_id)
    }
    
    /// Get all neuron positions in a cortical area (for fast batch lookups)
    /// Returns list of tuples (neuron_id, x, y, z)
    fn get_neuron_positions_in_cortical_area(&self, cortical_area: u32) -> Vec<(u32, u32, u32, u32)> {
        self.npu.get_neuron_positions_in_cortical_area(cortical_area)
    }
    
    /// Update excitability for a single neuron (live parameter change)
    /// Returns true if successful, false if neuron doesn't exist
    fn update_neuron_excitability(&mut self, neuron_id: u32, excitability: f32) -> bool {
        self.npu.update_neuron_excitability(neuron_id, excitability)
    }
    
    /// Update excitability for all neurons in a cortical area (bulk parameter change)
    /// Returns number of neurons updated
    fn update_cortical_area_excitability(&mut self, cortical_area: u32, excitability: f32) -> usize {
        self.npu.update_cortical_area_excitability(cortical_area, excitability)
    }
    
    /// Update refractory period for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_refractory_period(&mut self, cortical_area: u32, refractory_period: u16) -> usize {
        self.npu.update_cortical_area_refractory_period(cortical_area, refractory_period)
    }
    
    /// Update threshold for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_threshold(&mut self, cortical_area: u32, threshold: f32) -> usize {
        self.npu.update_cortical_area_threshold(cortical_area, threshold)
    }
    
    /// Update leak coefficient for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_leak(&mut self, cortical_area: u32, leak: f32) -> usize {
        self.npu.update_cortical_area_leak(cortical_area, leak)
    }
    
    /// Update consecutive fire limit for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_consecutive_fire_limit(&mut self, cortical_area: u32, limit: u16) -> usize {
        self.npu.update_cortical_area_consecutive_fire_limit(cortical_area, limit)
    }
    
    /// Update snooze period (extended refractory) for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_snooze_period(&mut self, cortical_area: u32, snooze_period: u16) -> usize {
        self.npu.update_cortical_area_snooze_period(cortical_area, snooze_period)
    }
    
    /// Batch update refractory period for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_refractory_period(&mut self, neuron_ids: Vec<u32>, values: Vec<u16>) -> usize {
        self.npu.batch_update_refractory_period(&neuron_ids, &values)
    }
    
    /// Batch update threshold for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_threshold(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.batch_update_threshold(&neuron_ids, &values)
    }
    
    /// Batch update leak coefficient for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_leak_coefficient(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.batch_update_leak_coefficient(&neuron_ids, &values)
    }
    
    /// Batch update consecutive fire limit for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_consecutive_fire_limit(&mut self, neuron_ids: Vec<u32>, values: Vec<u16>) -> usize {
        self.npu.batch_update_consecutive_fire_limit(&neuron_ids, &values)
    }
    
    /// Batch update snooze period (extended refractory) for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_snooze_period(&mut self, neuron_ids: Vec<u32>, values: Vec<u16>) -> usize {
        self.npu.batch_update_snooze_period(&neuron_ids, &values)
    }
    
    /// Batch update membrane potential for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_membrane_potential(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.batch_update_membrane_potential(&neuron_ids, &values)
    }
    
    /// Batch update resting potential for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_resting_potential(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.batch_update_resting_potential(&neuron_ids, &values)
    }
    
    /// Batch update excitability for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_excitability(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.batch_update_excitability(&neuron_ids, &values)
    }
    
    /// Batch update neuron type for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_neuron_type(&mut self, neuron_ids: Vec<u32>, values: Vec<i32>) -> usize {
        self.npu.batch_update_neuron_type(&neuron_ids, &values)
    }
    
    /// Delete a neuron (mark as invalid)
    /// Returns true if successful, false if neuron out of bounds
    fn delete_neuron(&mut self, neuron_id: u32) -> bool {
        self.npu.delete_neuron(neuron_id)
    }
    
    /// Get neuron state for diagnostics
    /// Returns (cfc, cfc_limit, snooze_countdown, snooze_period, potential, threshold, refrac_countdown) or None
    fn get_neuron_state(&self, neuron_id: u32) -> Option<(u16, u16, u16, f32, f32, u16)> {
        self.npu.get_neuron_state(NeuronId(neuron_id))
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // PROPERTY GETTERS (for batch_get_neuron_properties)
    // ═══════════════════════════════════════════════════════════════════
    
    /// Get neuron refractory period
    fn get_neuron_refractory_period(&self, neuron_id: u32) -> Option<u16> {
        self.npu.neuron_array.refractory_periods.get(neuron_id as usize).copied()
    }
    
    /// Get neuron firing threshold
    fn get_neuron_threshold(&self, neuron_id: u32) -> Option<f32> {
        self.npu.neuron_array.thresholds.get(neuron_id as usize).copied()
    }
    
    /// Get neuron leak coefficient (decay rate)
    fn get_neuron_leak_coefficient(&self, neuron_id: u32) -> Option<f32> {
        self.npu.neuron_array.leak_coefficients.get(neuron_id as usize).copied()
    }
    
    /// Get neuron membrane potential
    fn get_neuron_membrane_potential(&self, neuron_id: u32) -> Option<f32> {
        self.npu.neuron_array.membrane_potentials.get(neuron_id as usize).copied()
    }
    
    /// Get neuron resting potential
    fn get_neuron_resting_potential(&self, neuron_id: u32) -> Option<f32> {
        self.npu.neuron_array.resting_potentials.get(neuron_id as usize).copied()
    }
    
    /// Get neuron excitability
    fn get_neuron_excitability(&self, neuron_id: u32) -> Option<f32> {
        self.npu.neuron_array.excitabilities.get(neuron_id as usize).copied()
    }
    
    /// Get neuron consecutive fire limit
    fn get_neuron_consecutive_fire_limit(&self, neuron_id: u32) -> Option<u16> {
        self.npu.neuron_array.consecutive_fire_limits.get(neuron_id as usize).copied()
    }
}

/// Python wrapper for visualization neuron data encoding
#[pyclass]
struct VisualizationEncoder {
    mapped_data: CorticalMappedXYZPNeuronData,
}

#[pymethods]
impl VisualizationEncoder {
    #[new]
    fn new() -> Self {
        Self {
            mapped_data: CorticalMappedXYZPNeuronData::new(),
        }
    }

    /// Add neurons for a cortical area
    /// 
    /// Args:
    ///     cortical_id: Cortical area ID (string)
    ///     x_coords: X coordinates (list of u32)
    ///     y_coords: Y coordinates (list of u32)
    ///     z_coords: Z coordinates (list of u32)
    ///     potentials: Membrane potentials (list of f32)
    fn add_neurons(
        &mut self,
        cortical_id: String,
        x_coords: Vec<u32>,
        y_coords: Vec<u32>,
        z_coords: Vec<u32>,
        potentials: Vec<f32>,
    ) -> PyResult<()> {
        // Validate array lengths
        let n = x_coords.len();
        if y_coords.len() != n || z_coords.len() != n || potentials.len() != n {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "All coordinate and potential arrays must have the same length"
            ));
        }

        // Create cortical ID
        let cid = CorticalID::from_string(cortical_id)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("{:?}", e)))?;

        // Create neuron array
        let neuron_array = NeuronXYZPArrays::new_from_vectors(x_coords, y_coords, z_coords, potentials)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("{:?}", e)))?;

        // Insert into mapped data
        self.mapped_data.insert(cid, neuron_array);

        Ok(())
    }

    /// Encode to FEAGI byte structure (Type 11)
    /// 
    /// Returns:
    ///     bytes: Encoded binary data
    fn encode(&self, py: Python) -> PyResult<Py<PyBytes>> {
        // Manual serialization following feagi_data_serialization format
        // Type ID (1 byte) + Version (1 byte) + Number of cortical areas (2 bytes)  + headers + data
        
        const STRUCT_HEADER_SIZE: usize = 2;  // type + version
        const CORTICAL_COUNT_SIZE: usize = 2;  // u16 for count
        const CORTICAL_ID_BYTES: usize = 6;
        const PER_CORTICAL_HEADER: usize = CORTICAL_ID_BYTES + 4 + 4;  // ID + start + length
        const BYTES_PER_NEURON: usize = 16;  // 4 * u32/f32
        
        let num_areas = self.mapped_data.len();
        let mut total_neuron_bytes = 0usize;
        for (_, neurons) in &self.mapped_data.mappings {
            total_neuron_bytes += neurons.len() * BYTES_PER_NEURON;
        }
        
        let total_size = STRUCT_HEADER_SIZE + CORTICAL_COUNT_SIZE 
                       + (num_areas * PER_CORTICAL_HEADER) 
                       + total_neuron_bytes;
        
        let mut bytes = vec![0u8; total_size];
        
        // Write struct header
        bytes[0] = 11;  // NeuronCategoricalXYZP type
        bytes[1] = 1;   // Version 1
        
        // Write cortical area count
        bytes[2..4].copy_from_slice(&(num_areas as u16).to_le_bytes());
        
        let mut header_offset = 4;
        let mut data_offset = 4 + (num_areas * PER_CORTICAL_HEADER);
        
        // Write each cortical area
        for (cortical_id, neurons) in &self.mapped_data.mappings {
            // Write cortical ID (6 bytes)
            let id_bytes = cortical_id.as_bytes();
            bytes[header_offset..header_offset + 6].copy_from_slice(id_bytes);
            header_offset += 6;
            
            // Write data start offset (4 bytes, u32)
            bytes[header_offset..header_offset + 4].copy_from_slice(&(data_offset as u32).to_le_bytes());
            header_offset += 4;
            
            // Write data length (4 bytes, u32)
            let neuron_count = neurons.len();
            let data_len = neuron_count * BYTES_PER_NEURON;
            bytes[header_offset..header_offset + 4].copy_from_slice(&(data_len as u32).to_le_bytes());
            header_offset += 4;
            
            // Write neuron data (x, y, z, p arrays) - organized as 4 contiguous arrays
            let quarter = neuron_count * 4;  // Each value is 4 bytes
            for i in 0..neuron_count {
                let neuron = neurons.get(i)
                    .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("{:?}", e)))?;
                let x_offset = data_offset + i * 4;
                let y_offset = data_offset + quarter + i * 4;
                let z_offset = data_offset + 2 * quarter + i * 4;
                let p_offset = data_offset + 3 * quarter + i * 4;
                
                bytes[x_offset..x_offset + 4].copy_from_slice(&neuron.cortical_coordinate.x.to_le_bytes());
                bytes[y_offset..y_offset + 4].copy_from_slice(&neuron.cortical_coordinate.y.to_le_bytes());
                bytes[z_offset..z_offset + 4].copy_from_slice(&neuron.cortical_coordinate.z.to_le_bytes());
                bytes[p_offset..p_offset + 4].copy_from_slice(&neuron.potential.to_le_bytes());
            }
            
            data_offset += data_len;
        }
        
        Ok(PyBytes::new_bound(py, &bytes).into())
    }

    /// Clear all neuron data
    fn clear(&mut self) {
        self.mapped_data = CorticalMappedXYZPNeuronData::new();
    }
}

/// Python wrapper for decoding FEAGI byte structures
/// 
/// This class provides the same API as feagi_rust_py_libs for compatibility,
/// allowing feagi_core to decode incoming neural data from agents.
#[pyclass]
struct FeagiByteStructure {
    raw_bytes: Vec<u8>,
}

#[pymethods]
impl FeagiByteStructure {
    #[new]
    fn new(raw_bytes: Vec<u8>) -> Self {
        Self { raw_bytes }
    }
    
    /// Get the structure type ID from the byte structure
    /// 
    /// Returns:
    ///     int: Structure type ID (e.g., 11 for NeuronCategoricalXYZP)
    #[getter]
    fn structure_type(&self) -> PyResult<u8> {
        if self.raw_bytes.is_empty() {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Empty byte structure"
            ));
        }
        Ok(self.raw_bytes[0])
    }
}

/// Decoder for FEAGI neural data
/// 
/// Compatible with feagi_rust_py_libs API for seamless migration.
#[pyclass]
struct CorticalMappedXYZPNeuronDataDecoder {
    mapped_data: CorticalMappedXYZPNeuronData,
}

#[pymethods]
impl CorticalMappedXYZPNeuronDataDecoder {
    /// Create decoder from FeagiByteStructure
    /// 
    /// Args:
    ///     byte_structure: FeagiByteStructure containing encoded neural data
    /// 
    /// Returns:
    ///     CorticalMappedXYZPNeuronDataDecoder: Decoder with parsed data
    #[staticmethod]
    fn new_from_feagi_byte_structure(byte_structure: &FeagiByteStructure) -> PyResult<Self> {
        // Decode the byte structure manually following the format
        let bytes = &byte_structure.raw_bytes;
        
        if bytes.len() < 4 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Invalid byte structure: too short"
            ));
        }
        
        // Parse header
        let struct_type = bytes[0];
        if struct_type != 11 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                format!("Unsupported structure type: {}", struct_type)
            ));
        }
        
        let num_areas = u16::from_le_bytes([bytes[2], bytes[3]]) as usize;
        
        let mut mapped_data = CorticalMappedXYZPNeuronData::new();
        let mut offset = 4;
        
        // Parse each cortical area
        for _ in 0..num_areas {
            if offset + 14 > bytes.len() {
                break;
            }
            
            // Parse cortical ID (6 bytes ASCII)
            let cid_bytes = &bytes[offset..offset + 6];
            let cid_str = std::str::from_utf8(cid_bytes)
                .unwrap_or("??????")
                .trim_end_matches('\0');
            // Use from_string() to accept both built-in (iic, ooc, etc.) and custom (c*) cortical IDs
            let cortical_id = CorticalID::from_string(cid_str.to_string())
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("Invalid cortical ID: {}", e)))?;
            
            offset += 6;
            
            // Parse data start and length
            let _data_start = u32::from_le_bytes([
                bytes[offset], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]
            ]) as usize;
            offset += 4;
            
            let data_len = u32::from_le_bytes([
                bytes[offset], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]
            ]) as usize;
            offset += 4;
            
            // Parse neurons (16 bytes per neuron: x, y, z, p as u32/f32)
            let num_neurons = data_len / 16;
            let mut neurons = NeuronXYZPArrays::new();
            
            for i in 0..num_neurons {
                let neuron_offset = offset + (i * 16);
                if neuron_offset + 16 > bytes.len() {
                    break;
                }
                
                let x = u32::from_le_bytes([
                    bytes[neuron_offset],
                    bytes[neuron_offset + 1],
                    bytes[neuron_offset + 2],
                    bytes[neuron_offset + 3],
                ]);
                
                let y = u32::from_le_bytes([
                    bytes[neuron_offset + 4],
                    bytes[neuron_offset + 5],
                    bytes[neuron_offset + 6],
                    bytes[neuron_offset + 7],
                ]);
                
                let z = u32::from_le_bytes([
                    bytes[neuron_offset + 8],
                    bytes[neuron_offset + 9],
                    bytes[neuron_offset + 10],
                    bytes[neuron_offset + 11],
                ]);
                
                let p_bytes = [
                    bytes[neuron_offset + 12],
                    bytes[neuron_offset + 13],
                    bytes[neuron_offset + 14],
                    bytes[neuron_offset + 15],
                ];
                let p = f32::from_le_bytes(p_bytes);
                
                let neuron = NeuronXYZP::new(x, y, z, p);
                neurons.push(&neuron);
            }
            
            mapped_data.insert(cortical_id, neurons);
            offset += data_len;
        }
        
        Ok(Self { mapped_data })
    }
    
    /// Iterate over all cortical areas and their neurons
    /// 
    /// Yields:
    ///     Tuples of (cortical_id, (x_coords, y_coords, z_coords, potentials))
    ///     where coordinates are numpy arrays for performance
    fn iter_full<'py>(&self, py: Python<'py>) -> PyResult<Vec<(Py<PyAny>, (Bound<'py, PyArray1<u32>>, Bound<'py, PyArray1<u32>>, Bound<'py, PyArray1<u32>>, Bound<'py, PyArray1<f32>>))>> {
        let mut results = Vec::new();
        
        for (cortical_id, neurons) in self.mapped_data.mappings.iter() {
            // Convert CorticalID to Python string
            let cid_str = cortical_id.as_ascii_string();
            let py_cid = cid_str.to_object(py);
            
            // Extract coordinate arrays
            let mut x_coords = Vec::with_capacity(neurons.len());
            let mut y_coords = Vec::with_capacity(neurons.len());
            let mut z_coords = Vec::with_capacity(neurons.len());
            let mut potentials = Vec::with_capacity(neurons.len());
            
            for neuron in neurons.iter() {
                x_coords.push(neuron.cortical_coordinate.x);
                y_coords.push(neuron.cortical_coordinate.y);
                z_coords.push(neuron.cortical_coordinate.z);
                potentials.push(neuron.potential);
            }
            
            // Convert to numpy arrays for performance compatibility
            let x_array = x_coords.to_pyarray_bound(py);
            let y_array = y_coords.to_pyarray_bound(py);
            let z_array = z_coords.to_pyarray_bound(py);
            let p_array = potentials.to_pyarray_bound(py);
            
            results.push((py_cid, (x_array, y_array, z_array, p_array)));
        }
        
        Ok(results)
    }
}

/// Module containing fast neural network operations
#[pymodule]
fn feagi_rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Add the complete Rust NPU (NEW!)
    m.add_class::<RustNPU>()?;
    m.add_class::<BurstResult>()?;
    
    // Add visualization encoding (uses published feagi_data_structures)
    m.add_class::<VisualizationEncoder>()?;
    
    // Add data decoding (NEW! - eliminates feagi_rust_py_libs dependency)
    m.add_class::<FeagiByteStructure>()?;
    m.add_class::<CorticalMappedXYZPNeuronDataDecoder>()?;
    
    // Add the synaptic propagation engine (legacy, for compatibility)
    // m.add_class::<SynapticPropagationEngine>()?;  // LEGACY: Not used - full RustNPU is used instead

    // Add version information
    m.add("__version__", "0.5.0")?;

    Ok(())
}
