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
use pyo3::exceptions::PyValueError;
use numpy::{PyArray1, ToPyArray};
use feagi_types::*;
use feagi_burst_engine::{RustNPU as RustNPUCore, BurstResult as RustBurstResult};
use ahash::AHashMap;
use std::sync::{Arc, Mutex};
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
    npu: Arc<Mutex<RustNPUCore>>,  // Always use Arc<Mutex> for thread-safety
    burst_runner: Option<Arc<Mutex<feagi_burst_engine::BurstLoopRunner>>>,
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
            npu: Arc::new(Mutex::new(RustNPUCore::new(neuron_capacity, synapse_capacity, fire_ledger_window))),
            burst_runner: None,
        }
    }
    
    /// Set power injection amount (default: 1.0)
    fn set_power_amount(&mut self, amount: f32) {
        self.npu.lock().unwrap().set_power_amount(amount);
    }
    
    // 🔋 Power neurons auto-discovered from neuron array - no separate list!
    
    /// Start the burst loop runner (runs in background Rust thread)
    /// 
    /// Args:
    ///     frequency_hz: Burst frequency in Hz (e.g., 30.0)
    /// 
    /// Once started, the burst loop runs autonomously in Rust with ZERO Python overhead!
    /// Python is free to handle REST API, visualization, etc. while bursts process.
    fn start_burst_loop(&mut self, frequency_hz: f64) -> PyResult<()> {
        if self.burst_runner.is_some() {
            return Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                "Burst loop already running"
            ));
        }
        
        let mut runner = feagi_burst_engine::BurstLoopRunner::new(self.npu.clone(), frequency_hz);
        
        // 🦀 Power neurons are already in RustNPU - runner reads them directly!
        // NO Python involvement in power injection hot path!
        
        runner.start().map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("Failed to start burst loop: {}", e))
        })?;
        
        self.burst_runner = Some(Arc::new(Mutex::new(runner)));
        Ok(())
    }
    
    /// Stop the burst loop runner
    fn stop_burst_loop(&mut self) {
        if let Some(runner) = self.burst_runner.take() {
            runner.lock().unwrap().stop();
        }
    }
    
    /// Check if burst loop is running
    fn is_burst_loop_running(&self) -> bool {
        self.burst_runner.as_ref()
            .map(|r| r.lock().unwrap().is_running())
            .unwrap_or(false)
    }
    
    /// Get current burst count from the burst loop
    fn get_burst_loop_count(&self) -> u64 {
        self.burst_runner.as_ref()
            .map(|r| r.lock().unwrap().get_burst_count())
            .unwrap_or(0)
    }
    
    /// Register a sensory agent for automatic SHM polling and FCL injection
    ///
    /// Args:
    ///     agent_id: Unique agent identifier (e.g., "video-agent-1")
    ///     shm_path: Path to agent's sensory SHM slot (e.g., "/dev/shm/feagi_sensory_video-agent-1")
    ///     rate_hz: Polling rate in Hz (e.g., 30.0 for 30 FPS video)
    ///     area_mapping: Dict mapping cortical area IDs to cortical_idx (e.g., {"vision_center": 0})
    ///
    /// The sensory agent will run in a dedicated Rust thread, polling SHM at rate_hz
    /// and injecting decoded neurons directly into the FCL.
    fn register_sensory_agent(
        &mut self,
        agent_id: String,
        shm_path: String,
        rate_hz: f64,
        area_mapping: std::collections::HashMap<String, u32>,
    ) -> PyResult<()> {
        use std::path::PathBuf;
        
        // Check if burst runner exists
        if let Some(burst_runner) = &self.burst_runner {
            // Burst loop is running - register immediately
            let config = feagi_burst_engine::sensory::AgentConfig {
                agent_id: agent_id.clone(),
                shm_path: PathBuf::from(shm_path),
                rate_hz,
                area_mapping,
            };
            
            let mut runner = burst_runner.lock().unwrap();
            let mut sensory_mgr = runner.sensory_manager.lock().unwrap();
            sensory_mgr.register_agent(config).map_err(|e| {
                PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                    format!("Failed to register sensory agent: {}", e)
                )
            })?;
            
            drop(sensory_mgr);
            drop(runner);
            
            println!("✅ Registered sensory agent: {} at {} Hz (burst loop running)", agent_id, rate_hz);
            Ok(())
        } else {
            // Burst loop not started yet - return error instructing to load genome first
            println!("⚠️ Burst loop not running - agent {} cannot register yet", agent_id);
            Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                format!("Burst loop not running. Load genome to start burst loop, then register agent {}.", agent_id)
            ))
        }
    }
    
    /// Deregister a sensory agent (stops its polling thread)
    ///
    /// Args:
    ///     agent_id: Unique agent identifier to deregister
    fn deregister_sensory_agent(&mut self, agent_id: String) -> PyResult<()> {
        let burst_runner = self.burst_runner.as_ref().ok_or_else(|| {
            PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                "Burst loop not running"
            )
        })?;
        
        let runner = burst_runner.lock().unwrap();
        let mut sensory_mgr = runner.sensory_manager.lock().unwrap();
        sensory_mgr.deregister_agent(&agent_id).map_err(|e| {
            PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                format!("Failed to deregister sensory agent: {}", e)
            )
        })?;
        
        drop(sensory_mgr);
        drop(runner);
        
        println!("✅ Deregistered sensory agent: {}", agent_id);
        Ok(())
    }
    
    /// List all registered sensory agents
    fn list_sensory_agents(&self) -> PyResult<Vec<String>> {
        let burst_runner = self.burst_runner.as_ref().ok_or_else(|| {
            PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                "Burst loop not running"
            )
        })?;
        
        let runner = burst_runner.lock().unwrap();
        let sensory_mgr = runner.sensory_manager.lock().unwrap();
        Ok(sensory_mgr.list_agents())
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
            .lock().unwrap()
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
            .lock().unwrap()
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
        
        self.npu.lock().unwrap().add_synapses_batch(source_ids, target_ids, weight_vals, conductance_vals, type_vals)
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
        self.npu.lock().unwrap().remove_synapse(NeuronId(source), NeuronId(target))
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
        self.npu.lock().unwrap().remove_synapses_from_sources(source_ids)
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
        self.npu.lock().unwrap().remove_synapses_between(source_ids, target_ids)
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
        self.npu.lock().unwrap().update_synapse_weight(NeuronId(source), NeuronId(target), SynapticWeight(new_weight))
    }
    
    /// Rebuild indexes after bulk modifications
    /// 
    /// Call this after adding/removing many synapses for optimal performance
    fn rebuild_indexes(&mut self) {
        self.npu.lock().unwrap().rebuild_indexes();
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
        
        self.npu.lock().unwrap().set_neuron_mapping(rust_mapping);
        Ok(())
    }
    
    /// Process a single burst (MAIN METHOD - ALL IN RUST!)
    /// 
    /// This is the complete neural processing pipeline:
    /// Phase 1: Injection → Phase 2: Dynamics → Phase 3: Archival → Phase 5: Cleanup
    /// 
    /// Process a single burst
    /// 
    /// 🔋 Power neurons are auto-discovered from neuron array (cortical_idx = 1)
    /// 
    /// Returns:
    ///     BurstResult with fired_neurons, burst number, and performance metrics
    fn process_burst(&mut self) -> PyResult<BurstResult> {
        self.npu
            .lock().unwrap()
            .process_burst()
            .map(BurstResult::from)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }
    
    /// Get current burst count
    fn get_burst_count(&self) -> u64 {
        self.npu.lock().unwrap().get_burst_count()
    }
    
    /// Get neuron count
    fn get_neuron_count(&self) -> usize {
        self.npu.lock().unwrap().get_neuron_count()
    }
    
    /// Get synapse count (valid only)
    fn get_synapse_count(&self) -> usize {
        self.npu.lock().unwrap().get_synapse_count()
    }
    
    /// Get all neuron positions in a cortical area (for fast batch lookups)
    /// Returns list of tuples (neuron_id, x, y, z)
    fn get_neuron_positions_in_cortical_area(&self, cortical_area: u32) -> Vec<(u32, u32, u32, u32)> {
        self.npu.lock().unwrap().get_neuron_positions_in_cortical_area(cortical_area)
    }
    
    /// Get neuron ID at specific coordinates (spatial hash lookup for sensory injection)
    /// Returns None if no neuron exists at the given coordinates
    fn get_neuron_at_coordinate(&self, cortical_area: u32, x: u32, y: u32, z: u32) -> Option<u32> {
        self.npu.lock().unwrap().neuron_array.get_neuron_at_coordinate(cortical_area, x, y, z).map(|id| id.0)
    }
    
    /// BATCH: Get neuron IDs for multiple coordinates (high-performance sensory injection)
    /// Returns Vec<Option<u32>> parallel to input coordinates (None = no neuron at that position)
    /// 
    /// This is 10-100x faster than calling get_neuron_at_coordinate in a Python loop
    /// because it eliminates FFI overhead and enables vectorization.
    fn get_neurons_at_coordinates_batch(
        &self,
        cortical_area: u32,
        coords_x: Vec<u32>,
        coords_y: Vec<u32>,
        coords_z: Vec<u32>,
    ) -> PyResult<Vec<Option<u32>>> {
        // Validate input lengths match
        let len = coords_x.len();
        if coords_y.len() != len || coords_z.len() != len {
            return Err(PyValueError::new_err(format!(
                "Coordinate array length mismatch: x={}, y={}, z={}",
                coords_x.len(), coords_y.len(), coords_z.len()
            )));
        }
        
        // Batch lookup - single iteration, no Python FFI overhead
        let neuron_ids: Vec<Option<u32>> = coords_x.iter()
            .zip(coords_y.iter())
            .zip(coords_z.iter())
            .map(|((&x, &y), &z)| {
                self.npu.lock().unwrap().neuron_array.get_neuron_at_coordinate(cortical_area, x, y, z).map(|id| id.0)
            })
            .collect();
        
        Ok(neuron_ids)
    }
    
    // ===== RUST-NATIVE SENSORY INJECTION API =====
    
    /// Inject sensory neurons directly into FCL (Rust-native sensory injection)
    /// 
    /// This is called by Rust sensory threads to inject neurons with ZERO Python overhead.
    /// For Python-side injection, use the REST API or existing _pending_external_activations.
    /// 
    /// Args:
    ///     neuron_ids: List of neuron IDs to inject
    ///     potential: Membrane potential to add (default: 1.0)
    fn inject_sensory_batch(&mut self, neuron_ids: Vec<u32>, potential: f32) {
        let ids: Vec<NeuronId> = neuron_ids.into_iter().map(NeuronId).collect();
        self.npu.lock().unwrap().inject_sensory_batch(&ids, potential);
    }
    
    /// Update excitability for a single neuron (live parameter change)
    /// Returns true if successful, false if neuron doesn't exist
    fn update_neuron_excitability(&mut self, neuron_id: u32, excitability: f32) -> bool {
        self.npu.lock().unwrap().update_neuron_excitability(neuron_id, excitability)
    }
    
    /// Update excitability for all neurons in a cortical area (bulk parameter change)
    /// Returns number of neurons updated
    fn update_cortical_area_excitability(&mut self, cortical_area: u32, excitability: f32) -> usize {
        self.npu.lock().unwrap().update_cortical_area_excitability(cortical_area, excitability)
    }
    
    /// Update refractory period for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_refractory_period(&mut self, cortical_area: u32, refractory_period: u16) -> usize {
        self.npu.lock().unwrap().update_cortical_area_refractory_period(cortical_area, refractory_period)
    }
    
    /// Update threshold for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_threshold(&mut self, cortical_area: u32, threshold: f32) -> usize {
        self.npu.lock().unwrap().update_cortical_area_threshold(cortical_area, threshold)
    }
    
    /// Update leak coefficient for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_leak(&mut self, cortical_area: u32, leak: f32) -> usize {
        self.npu.lock().unwrap().update_cortical_area_leak(cortical_area, leak)
    }
    
    /// Update consecutive fire limit for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_consecutive_fire_limit(&mut self, cortical_area: u32, limit: u16) -> usize {
        self.npu.lock().unwrap().update_cortical_area_consecutive_fire_limit(cortical_area, limit)
    }
    
    /// Update snooze period (extended refractory) for all neurons in a cortical area
    /// Returns number of neurons updated
    fn update_cortical_area_snooze_period(&mut self, cortical_area: u32, snooze_period: u16) -> usize {
        self.npu.lock().unwrap().update_cortical_area_snooze_period(cortical_area, snooze_period)
    }
    
    /// Batch update refractory period for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_refractory_period(&mut self, neuron_ids: Vec<u32>, values: Vec<u16>) -> usize {
        self.npu.lock().unwrap().batch_update_refractory_period(&neuron_ids, &values)
    }
    
    /// Batch update threshold for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_threshold(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.lock().unwrap().batch_update_threshold(&neuron_ids, &values)
    }
    
    /// Batch update leak coefficient for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_leak_coefficient(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.lock().unwrap().batch_update_leak_coefficient(&neuron_ids, &values)
    }
    
    /// Batch update consecutive fire limit for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_consecutive_fire_limit(&mut self, neuron_ids: Vec<u32>, values: Vec<u16>) -> usize {
        self.npu.lock().unwrap().batch_update_consecutive_fire_limit(&neuron_ids, &values)
    }
    
    /// Batch update snooze period (extended refractory) for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_snooze_period(&mut self, neuron_ids: Vec<u32>, values: Vec<u16>) -> usize {
        self.npu.lock().unwrap().batch_update_snooze_period(&neuron_ids, &values)
    }
    
    /// Batch update membrane potential for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_membrane_potential(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.lock().unwrap().batch_update_membrane_potential(&neuron_ids, &values)
    }
    
    /// Batch update resting potential for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_resting_potential(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.lock().unwrap().batch_update_resting_potential(&neuron_ids, &values)
    }
    
    /// Batch update excitability for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_excitability(&mut self, neuron_ids: Vec<u32>, values: Vec<f32>) -> usize {
        self.npu.lock().unwrap().batch_update_excitability(&neuron_ids, &values)
    }
    
    /// Batch update neuron type for multiple neurons
    /// Returns number of neurons updated
    fn batch_update_neuron_type(&mut self, neuron_ids: Vec<u32>, values: Vec<i32>) -> usize {
        self.npu.lock().unwrap().batch_update_neuron_type(&neuron_ids, &values)
    }
    
    /// Delete a neuron (mark as invalid)
    /// Returns true if successful, false if neuron out of bounds
    fn delete_neuron(&mut self, neuron_id: u32) -> bool {
        self.npu.lock().unwrap().delete_neuron(neuron_id)
    }
    
    /// Get neuron state for diagnostics
    /// Returns (cfc, cfc_limit, snooze_countdown, snooze_period, potential, threshold, refrac_countdown) or None
    fn get_neuron_state(&self, neuron_id: u32) -> Option<(u16, u16, u16, f32, f32, u16)> {
        self.npu.lock().unwrap().get_neuron_state(NeuronId(neuron_id))
    }
    
    // ═══════════════════════════════════════════════════════════════════
    // PROPERTY GETTERS (for batch_get_neuron_properties)
    // ═══════════════════════════════════════════════════════════════════
    
    /// Get neuron refractory period
    fn get_neuron_refractory_period(&self, neuron_id: u32) -> Option<u16> {
        let idx = *self.npu.lock().unwrap().neuron_array.neuron_id_to_index.get(&neuron_id)?;
        self.npu.lock().unwrap().neuron_array.refractory_periods.get(idx).copied()
    }
    
    /// Get neuron firing threshold
    fn get_neuron_threshold(&self, neuron_id: u32) -> Option<f32> {
        let idx = *self.npu.lock().unwrap().neuron_array.neuron_id_to_index.get(&neuron_id)?;
        self.npu.lock().unwrap().neuron_array.thresholds.get(idx).copied()
    }
    
    /// Get neuron leak coefficient (decay rate)
    fn get_neuron_leak_coefficient(&self, neuron_id: u32) -> Option<f32> {
        let idx = *self.npu.lock().unwrap().neuron_array.neuron_id_to_index.get(&neuron_id)?;
        self.npu.lock().unwrap().neuron_array.leak_coefficients.get(idx).copied()
    }
    
    /// Get neuron membrane potential
    fn get_neuron_membrane_potential(&self, neuron_id: u32) -> Option<f32> {
        let idx = *self.npu.lock().unwrap().neuron_array.neuron_id_to_index.get(&neuron_id)?;
        self.npu.lock().unwrap().neuron_array.membrane_potentials.get(idx).copied()
    }
    
    /// Get neuron resting potential
    fn get_neuron_resting_potential(&self, neuron_id: u32) -> Option<f32> {
        let idx = *self.npu.lock().unwrap().neuron_array.neuron_id_to_index.get(&neuron_id)?;
        self.npu.lock().unwrap().neuron_array.resting_potentials.get(idx).copied()
    }
    
    /// Get neuron excitability
    fn get_neuron_excitability(&self, neuron_id: u32) -> Option<f32> {
        let idx = *self.npu.lock().unwrap().neuron_array.neuron_id_to_index.get(&neuron_id)?;
        self.npu.lock().unwrap().neuron_array.excitabilities.get(idx).copied()
    }
    
    /// Get neuron consecutive fire limit
    fn get_neuron_consecutive_fire_limit(&self, neuron_id: u32) -> Option<u16> {
        let idx = *self.npu.lock().unwrap().neuron_array.neuron_id_to_index.get(&neuron_id)?;
        self.npu.lock().unwrap().neuron_array.consecutive_fire_limits.get(idx).copied()
    }
    
    /// Get outgoing synapses for a neuron
    /// Returns list of (target_neuron_id, weight, conductance, synapse_type)
    fn get_outgoing_synapses(&self, neuron_id: u32) -> Vec<(u32, u8, u8, u8)> {
        self.npu.lock().unwrap().get_outgoing_synapses(neuron_id)
    }
    
    /// Get incoming synapses for a neuron
    /// Returns list of (source_neuron_id, weight, conductance, synapse_type)
    fn get_incoming_synapses(&self, neuron_id: u32) -> Vec<(u32, u8, u8, u8)> {
        self.npu.lock().unwrap().get_incoming_synapses(neuron_id)
    }
    
    // ═══════════════════════════════════════════════════════════
    // FIRE LEDGER API (Entry Point #3: Debugging & STDP)
    // ═══════════════════════════════════════════════════════════
    
    /// Get firing history for a cortical area from Fire Ledger
    /// 
    /// Args:
    ///     cortical_idx: Cortical area index (u32)
    ///     lookback_steps: Number of timesteps to retrieve
    /// 
    /// Returns:
    ///     List of (timestep, [neuron_ids]) tuples, newest first
    /// 
    /// Example:
    ///     history = npu.get_fire_ledger_history(9, 50)
    ///     # Returns: [(2275, [16438, ...]), (2274, [16438, ...]), ...]
    fn get_fire_ledger_history(&self, cortical_idx: u32, lookback_steps: usize) -> Vec<(u64, Vec<u32>)> {
        self.npu.lock().unwrap().get_fire_ledger_history(cortical_idx, lookback_steps)
    }
    
    /// Get Fire Ledger window size for a cortical area
    /// 
    /// Args:
    ///     cortical_idx: Cortical area index (u32)
    /// 
    /// Returns:
    ///     Window size (number of timesteps retained)
    fn get_fire_ledger_window_size(&self, cortical_idx: u32) -> usize {
        self.npu.lock().unwrap().get_fire_ledger_window_size(cortical_idx)
    }
    
    /// Configure Fire Ledger window size for a specific cortical area
    /// 
    /// Args:
    ///     cortical_idx: Cortical area index (u32)
    ///     window_size: Number of timesteps to retain
    fn configure_fire_ledger_window(&mut self, cortical_idx: u32, window_size: usize) {
        self.npu.lock().unwrap().configure_fire_ledger_window(cortical_idx, window_size);
    }
    
    /// Get all configured Fire Ledger window sizes
    /// 
    /// Returns:
    ///     List of (cortical_idx, window_size) tuples
    fn get_all_fire_ledger_configs(&self) -> Vec<(u32, usize)> {
        self.npu.lock().unwrap().get_all_fire_ledger_configs()
    }
    
    // ═══════════════════════════════════════════════════════════
    // FQ SAMPLER API (Entry Point #2: Motor/Visualization Output)
    // ═══════════════════════════════════════════════════════════
    
    /// Get current Fire Queue directly (bypasses FQ Sampler - for FCL endpoint)
    /// Returns the current Fire Queue without rate limiting or deduplication
    fn get_current_fire_queue(&self, py: Python) -> PyResult<PyObject> {
        let areas = self.npu.lock().unwrap().get_current_fire_queue();
        
        // Convert to Python dict
        let py_dict = PyDict::new_bound(py);
        
        for (cortical_idx, (neuron_ids, coords_x, coords_y, coords_z, potentials)) in areas {
            let area_tuple = (
                neuron_ids.to_object(py),
                coords_x.to_object(py),
                coords_y.to_object(py),
                coords_z.to_object(py),
                potentials.to_object(py),
            );
            
            py_dict.set_item(cortical_idx, area_tuple)?;
        }
        
        Ok(py_dict.into())
    }
    
    /// Sample the current Fire Queue for visualization/motor output
    /// 
    /// Returns None if:
    /// - Rate limit not met
    /// - Fire Queue is empty
    /// - Burst already sampled (deduplication)
    /// 
    /// Returns:
    ///     Dict[cortical_idx: int, tuple] where tuple is:
    ///     (neuron_ids, coords_x, coords_y, coords_z, potentials)
    /// 
    /// Example:
    ///     sample = npu.sample_fire_queue()
    ///     if sample:
    ///         for cortical_idx, (ids, x, y, z, potentials) in sample.items():
    ///             # Process firing neurons for this area
    ///             pass
    /// 
    /// ⚠️ DEPRECATED: Use get_latest_fire_queue_sample() instead to avoid deduplication issues.
    fn sample_fire_queue(&mut self, py: Python) -> PyResult<Option<PyObject>> {
        match self.npu.lock().unwrap().sample_fire_queue() {
            Some(areas) => {
                // Convert HashMap to Python dict
                let py_dict = PyDict::new_bound(py);
                
                for (cortical_idx, (neuron_ids, coords_x, coords_y, coords_z, potentials)) in areas {
                    // Create a tuple of arrays for this area
                    let area_tuple = (
                        neuron_ids.to_object(py),
                        coords_x.to_object(py),
                        coords_y.to_object(py),
                        coords_z.to_object(py),
                        potentials.to_object(py),
                    );
                    
                    py_dict.set_item(cortical_idx, area_tuple)?;
                }
                
                Ok(Some(py_dict.into()))
            }
            None => Ok(None),
        }
    }
    
    /// Get the latest cached Fire Queue sample (non-consuming read)
    /// 
    /// This returns the most recent sample WITHOUT triggering rate limiting or deduplication.
    /// Unlike sample_fire_queue(), this can be called multiple times for the same burst.
    /// 
    /// Returns:
    ///     Dict[cortical_idx: int, tuple] where tuple is:
    ///     (neuron_ids, coords_x, coords_y, coords_z, potentials)
    ///     
    ///     Returns None if no bursts have been processed yet.
    fn get_latest_fire_queue_sample(&self, py: Python) -> PyResult<Option<PyObject>> {
        match self.npu.lock().unwrap().get_latest_fire_queue_sample() {
            Some(areas) => {
                // Convert HashMap to Python dict
                let py_dict = PyDict::new_bound(py);
                
                for (cortical_idx, (neuron_ids, coords_x, coords_y, coords_z, potentials)) in areas {
                    // Create a tuple of arrays for this area
                    let area_tuple = (
                        neuron_ids.to_object(py),
                        coords_x.to_object(py),
                        coords_y.to_object(py),
                        coords_z.to_object(py),
                        potentials.to_object(py),
                    );
                    
                    py_dict.set_item(cortical_idx, area_tuple)?;
                }
                
                Ok(Some(py_dict.into()))
            }
            None => Ok(None),
        }
    }
    
    /// Set FQ Sampler frequency (Hz)
    /// 
    /// Args:
    ///     frequency_hz: Sampling frequency in Hz (e.g., 10.0 for 10Hz)
    fn set_fq_sampler_frequency(&mut self, frequency_hz: f64) {
        self.npu.lock().unwrap().set_fq_sampler_frequency(frequency_hz);
    }
    
    /// Get FQ Sampler frequency (Hz)
    /// 
    /// Returns:
    ///     Current sampling frequency in Hz
    fn get_fq_sampler_frequency(&self) -> f64 {
        self.npu.lock().unwrap().get_fq_sampler_frequency()
    }
    
    /// Set visualization subscriber state
    /// 
    /// Args:
    ///     has_subscribers: True if Brain Visualizer is connected
    fn set_visualization_subscribers(&mut self, has_subscribers: bool) {
        self.npu.lock().unwrap().set_visualization_subscribers(has_subscribers);
    }
    
    /// Check if visualization subscribers are connected
    /// 
    /// Returns:
    ///     True if Brain Visualizer is connected
    fn has_visualization_subscribers(&self) -> bool {
        self.npu.lock().unwrap().has_visualization_subscribers()
    }
    
    /// Set motor subscriber state
    /// 
    /// Args:
    ///     has_subscribers: True if motor agents are connected
    fn set_motor_subscribers(&mut self, has_subscribers: bool) {
        self.npu.lock().unwrap().set_motor_subscribers(has_subscribers);
    }
    
    /// Check if motor subscribers are connected
    /// 
    /// Returns:
    ///     True if motor agents are connected
    fn has_motor_subscribers(&self) -> bool {
        self.npu.lock().unwrap().has_motor_subscribers()
    }
    
    /// Get total FQ Sampler samples taken
    /// 
    /// Returns:
    ///     Total number of samples taken since initialization
    fn get_fq_sampler_samples_taken(&self) -> u64 {
        self.npu.lock().unwrap().get_fq_sampler_samples_taken()
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
        
        // Check if this is a FeagiByteContainer (version 2) or raw structure data
        let actual_bytes = if bytes[0] == 2 {
            // This is a FeagiByteContainer, parse the wrapper
            // Format: [version:u8, counter_lo:u8, counter_hi:u8, num_structs:u8, struct_len:u32, struct_data...]
            if bytes.len() < 8 {
                return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                    format!("Invalid FeagiByteContainer: too short (len={})", bytes.len())
                ));
            }
            let num_structs = bytes[3] as usize;
            if num_structs == 0 {
                return Ok(Self { mapped_data: CorticalMappedXYZPNeuronData::new() });
            }
            
            // Skip global header (4 bytes) + per-struct header (4 bytes) to get to actual data
            &bytes[8..]
        } else {
            // Raw structure data (old format or direct structure bytes)
            bytes
        };
        
        if actual_bytes.len() < 4 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                "Invalid byte structure: too short after unwrapping"
            ));
        }
        
        // Parse header
        let struct_type = actual_bytes[0];
        
        if struct_type != 11 {
            return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                format!("Unsupported structure type: {} (expected 11 for NeuronCategoricalXYZP)", struct_type)
            ));
        }
        
        let num_areas = u16::from_le_bytes([actual_bytes[2], actual_bytes[3]]) as usize;
        
        let mut mapped_data = CorticalMappedXYZPNeuronData::new();
        let mut offset = 4;
        
        // First pass: collect all cortical area headers
        struct AreaHeader {
            cortical_id: CorticalID,
            data_start: usize,
            data_size_bytes: usize,  // Total bytes for this area (NOT neuron count!)
        }
        let mut area_headers = Vec::new();
        
        for _ in 0..num_areas {
            if offset + 14 > actual_bytes.len() {
                break;
            }
            
            // Parse cortical ID (6 bytes ASCII)
            let cid_bytes = &actual_bytes[offset..offset + 6];
            let cid_str = std::str::from_utf8(cid_bytes)
                .unwrap_or("??????")
                .trim_end_matches('\0');
            let cortical_id = CorticalID::from_string(cid_str.to_string())
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyValueError, _>(format!("Invalid cortical ID '{}': {}", cid_str, e)))?;
            offset += 6;
            
            // Parse data start index (relative to whole structure) - Note: buggy in v0.0.70, recalculated below
            let _data_start = u32::from_le_bytes([
                actual_bytes[offset], actual_bytes[offset + 1], actual_bytes[offset + 2], actual_bytes[offset + 3]
            ]) as usize;
            offset += 4;
            
            // Parse data size in bytes (NOT neuron count!)
            let data_size_bytes = u32::from_le_bytes([
                actual_bytes[offset], actual_bytes[offset + 1], actual_bytes[offset + 2], actual_bytes[offset + 3]
            ]) as usize;
            offset += 4;
            
            area_headers.push(AreaHeader { cortical_id, data_start: _data_start, data_size_bytes });
        }
        
        // WORKAROUND: The data_start values from feagi-rust-py-libs v0.0.70 are incorrect
        // Calculate correct offsets: all headers come first, then data sequentially
        let first_data_offset = 4 + (num_areas * 14); // header + (num_areas * header_size)
        let mut corrected_offset = first_data_offset;
        let mut corrected_headers = Vec::new();
        
        for header in area_headers {
            let corrected_header = AreaHeader {
                cortical_id: header.cortical_id,
                data_start: corrected_offset,
                data_size_bytes: header.data_size_bytes,
            };
            corrected_headers.push(corrected_header);
            corrected_offset += header.data_size_bytes;
        }
        
        // Second pass: read neuron data for each area using CORRECTED offsets
        // Data format: [all X coords][all Y coords][all Z coords][all potentials]
        for header in corrected_headers {
            if header.data_size_bytes == 0 {
                mapped_data.insert(header.cortical_id, NeuronXYZPArrays::new());
                continue;
            }
            
            let data_offset = header.data_start;
            let num_neurons = header.data_size_bytes / 16;  // 16 bytes per neuron (4 coords × 4 bytes each)
            let x_start = data_offset;
            let y_start = x_start + (num_neurons * 4);
            let z_start = y_start + (num_neurons * 4);
            let p_start = z_start + (num_neurons * 4);
            let p_end = p_start + (num_neurons * 4);
            
            if p_end > actual_bytes.len() {
                // Not enough bytes - skip this area
                break;
            }
            
            let mut neurons = NeuronXYZPArrays::new();
            for i in 0..num_neurons {
                let x = u32::from_le_bytes([
                    actual_bytes[x_start + i*4],
                    actual_bytes[x_start + i*4 + 1],
                    actual_bytes[x_start + i*4 + 2],
                    actual_bytes[x_start + i*4 + 3],
                ]);
                let y = u32::from_le_bytes([
                    actual_bytes[y_start + i*4],
                    actual_bytes[y_start + i*4 + 1],
                    actual_bytes[y_start + i*4 + 2],
                    actual_bytes[y_start + i*4 + 3],
                ]);
                let z = u32::from_le_bytes([
                    actual_bytes[z_start + i*4],
                    actual_bytes[z_start + i*4 + 1],
                    actual_bytes[z_start + i*4 + 2],
                    actual_bytes[z_start + i*4 + 3],
                ]);
                let p = f32::from_le_bytes([
                    actual_bytes[p_start + i*4],
                    actual_bytes[p_start + i*4 + 1],
                    actual_bytes[p_start + i*4 + 2],
                    actual_bytes[p_start + i*4 + 3],
                ]);
                
                let neuron = NeuronXYZP::new(x, y, z, p);
                neurons.push(&neuron);
            }
            
            mapped_data.insert(header.cortical_id, neurons);
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
