/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Complete Rust NPU
//!
//! Integrates all burst processing phases into a single high-performance NPU.
//!
//! ## Architecture
//! ```text
//! ┌─────────────────────────────────────┐
//! │ RustNPU                            │
//! ├─────────────────────────────────────┤
//! │ - NeuronArray                      │
//! │ - SynapseArray                     │
//! │ - FireCandidateList (FCL)          │
//! │ - FireQueue (current & previous)   │
//! │ - FireLedger                       │
//! │ - SynapticPropagationEngine        │
//! └─────────────────────────────────────┘
//!          ↓
//!     process_burst()
//!          ↓
//! Phase 1: Injection → Phase 2: Dynamics → Phase 3: Archival → Phase 5: Cleanup
//! ```

use feagi_types::*;
use crate::neural_dynamics::*;
use crate::synaptic_propagation::SynapticPropagationEngine;
use ahash::AHashMap;

/// Burst processing result
#[derive(Debug, Clone)]
pub struct BurstResult {
    /// Neurons that fired this burst
    pub fired_neurons: Vec<NeuronId>,
    
    /// Number of neurons that fired
    pub neuron_count: usize,
    
    /// Burst number
    pub burst: u64,
    
    /// Performance metrics
    pub power_injections: usize,
    pub synaptic_injections: usize,
    pub neurons_processed: usize,
    pub neurons_in_refractory: usize,
}

/// Complete Rust Neural Processing Unit
pub struct RustNPU {
    // Core data structures
    pub neuron_array: NeuronArray,
    pub synapse_array: SynapseArray,
    
    // Fire structures
    fire_candidate_list: FireCandidateList,
    current_fire_queue: FireQueue,
    previous_fire_queue: FireQueue,
    fire_ledger: FireLedger,
    
    // Engines
    propagation_engine: SynapticPropagationEngine,
    
    // State
    burst_count: u64,
    
    // Configuration
    power_amount: f32,
}

impl RustNPU {
    /// Create a new Rust NPU with specified capacities
    pub fn new(
        neuron_capacity: usize,
        synapse_capacity: usize,
        fire_ledger_window: usize,
    ) -> Self {
        Self {
            neuron_array: NeuronArray::new(neuron_capacity),
            synapse_array: SynapseArray::new(synapse_capacity),
            fire_candidate_list: FireCandidateList::new(),
            current_fire_queue: FireQueue::new(),
            previous_fire_queue: FireQueue::new(),
            fire_ledger: FireLedger::new(fire_ledger_window),
            propagation_engine: SynapticPropagationEngine::new(),
            burst_count: 0,
            power_amount: 1.0,
        }
    }
    
    /// Set power injection amount
    pub fn set_power_amount(&mut self, amount: f32) {
        self.power_amount = amount;
    }
    
    /// Add a neuron to the NPU (LIF model with genome leak only)
    pub fn add_neuron(
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
    ) -> Result<NeuronId> {
        let neuron_id = self.neuron_array.add_neuron(
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
        )?;
        
        // CRITICAL: Add to propagation engine's neuron-to-area mapping
        // This is required for synaptic propagation to work!
        self.propagation_engine.neuron_to_area.insert(neuron_id, CorticalAreaId(cortical_area));
        
        Ok(neuron_id)
    }
    
    /// Add a synapse to the NPU
    pub fn add_synapse(
        &mut self,
        source: NeuronId,
        target: NeuronId,
        weight: SynapticWeight,
        conductance: SynapticConductance,
        synapse_type: SynapseType,
    ) -> Result<usize> {
        self.synapse_array.add_synapse(source, target, weight, conductance, synapse_type)
    }
    
    /// Batch add synapses (SIMD-optimized)
    /// 
    /// Creates multiple synapses in a single operation with optimal performance.
    /// This is 50-100x faster than calling add_synapse() in a loop.
    /// 
    /// Performance:
    /// - Single function call overhead (vs N calls)
    /// - Contiguous SoA memory writes
    /// - Batch source_index updates
    /// 
    /// Returns: (successful_count, failed_indices)
    pub fn add_synapses_batch(
        &mut self,
        sources: Vec<NeuronId>,
        targets: Vec<NeuronId>,
        weights: Vec<SynapticWeight>,
        conductances: Vec<SynapticConductance>,
        synapse_types: Vec<SynapseType>,
    ) -> (usize, Vec<usize>) {
        // Convert NeuronId/Weight types to raw u32/u8 for SynapseArray
        let source_ids: Vec<u32> = sources.iter().map(|n| n.0).collect();
        let target_ids: Vec<u32> = targets.iter().map(|n| n.0).collect();
        let weight_vals: Vec<u8> = weights.iter().map(|w| w.0).collect();
        let conductance_vals: Vec<u8> = conductances.iter().map(|c| c.0).collect();
        let type_vals: Vec<u8> = synapse_types.iter().map(|t| match t {
            SynapseType::Excitatory => 0,
            SynapseType::Inhibitory => 1,
        }).collect();
        
        self.synapse_array.add_synapses_batch(
            &source_ids,
            &target_ids,
            &weight_vals,
            &conductance_vals,
            &type_vals,
        )
    }
    
    /// Remove a synapse
    pub fn remove_synapse(&mut self, source: NeuronId, target: NeuronId) -> bool {
        self.synapse_array.remove_synapse(source, target)
    }
    
    /// Batch remove all synapses from specified source neurons (SIMD-optimized)
    /// 
    /// Performance: 50-100x faster than individual deletions for cortical mapping removal
    /// Returns: number of synapses deleted
    pub fn remove_synapses_from_sources(&mut self, sources: Vec<NeuronId>) -> usize {
        let source_ids: Vec<u32> = sources.iter().map(|n| n.0).collect();
        self.synapse_array.remove_synapses_from_sources(&source_ids)
    }
    
    /// Batch remove synapses between source and target neuron sets (SIMD-optimized)
    /// 
    /// Uses bit-vector filtering for O(1) target membership testing.
    /// Optimal for both few→many and many→many deletion patterns.
    /// 
    /// Performance: 20-100x faster than nested loops
    /// Returns: number of synapses deleted
    pub fn remove_synapses_between(&mut self, sources: Vec<NeuronId>, targets: Vec<NeuronId>) -> usize {
        let source_ids: Vec<u32> = sources.iter().map(|n| n.0).collect();
        let target_ids: Vec<u32> = targets.iter().map(|n| n.0).collect();
        self.synapse_array.remove_synapses_between(&source_ids, &target_ids)
    }
    
    /// Update synapse weight
    pub fn update_synapse_weight(&mut self, source: NeuronId, target: NeuronId, new_weight: SynapticWeight) -> bool {
        self.synapse_array.update_weight(source, target, new_weight)
    }
    
    /// Rebuild indexes after modifications (call after bulk modifications)
    pub fn rebuild_indexes(&mut self) {
        // ZERO-COPY: Pass synapse_array by reference
        self.propagation_engine.build_synapse_index(&self.synapse_array);
    }
    
    /// Set neuron to cortical area mapping for propagation engine
    pub fn set_neuron_mapping(&mut self, mapping: AHashMap<NeuronId, CorticalAreaId>) {
        self.propagation_engine.set_neuron_mapping(mapping);
    }
    
    /// Process a single burst (MAIN METHOD)
    /// 
    /// This is the complete neural processing pipeline:
    /// Phase 1: Injection → Phase 2: Dynamics → Phase 3: Archival → Phase 5: Cleanup
    pub fn process_burst(&mut self, power_neurons: &[NeuronId]) -> Result<BurstResult> {
        self.burst_count += 1;
        
        // Phase 1: Injection (power + synaptic propagation)
        // ZERO-COPY: Pass synapse_array by reference (no allocation)
        let injection_result = phase1_injection_with_synapses(
            &mut self.fire_candidate_list,
            &self.neuron_array,
            &mut self.propagation_engine,
            &self.previous_fire_queue,
            power_neurons,
            self.power_amount,
            &self.synapse_array,
        )?;
        
        // Phase 2: Neural Dynamics (membrane potential updates, threshold checks, firing)
        let dynamics_result = process_neural_dynamics(
            &self.fire_candidate_list,
            &mut self.neuron_array,
            self.burst_count,
        )?;
        
        // Phase 3: Archival (record to Fire Ledger)
        let neuron_ids = dynamics_result.fire_queue.get_all_neuron_ids();
        self.fire_ledger.record_burst(self.burst_count, neuron_ids);
        
        // Phase 5: Cleanup (clear FCL for next burst)
        self.fire_candidate_list.clear();
        
        // Swap fire queues: current becomes previous for next burst
        self.previous_fire_queue = self.current_fire_queue.clone();
        self.current_fire_queue = dynamics_result.fire_queue.clone();
        
        // Build result
        let fired_neurons = self.current_fire_queue.get_all_neuron_ids();
        
        Ok(BurstResult {
            neuron_count: fired_neurons.len(),
            fired_neurons,
            burst: self.burst_count,
            power_injections: injection_result.power_injections,
            synaptic_injections: injection_result.synaptic_injections,
            neurons_processed: dynamics_result.neurons_processed,
            neurons_in_refractory: dynamics_result.neurons_in_refractory,
        })
    }
    
    /// Get current burst count
    pub fn get_burst_count(&self) -> u64 {
        self.burst_count
    }
    
    /// Get all neuron positions for a cortical area (for fast batch lookups)
    /// Returns Vec<(neuron_id, x, y, z)>
    pub fn get_neuron_positions_in_cortical_area(&self, cortical_area: u32) -> Vec<(u32, u32, u32, u32)> {
        let mut positions = Vec::new();
        
        for neuron_id in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[neuron_id] 
                && self.neuron_array.cortical_areas[neuron_id] == cortical_area {
                // Coordinates stored as flat array: [x0, y0, z0, x1, y1, z1, ...]
                let coord_idx = neuron_id * 3;
                positions.push((
                    neuron_id as u32,
                    self.neuron_array.coordinates[coord_idx],
                    self.neuron_array.coordinates[coord_idx + 1],
                    self.neuron_array.coordinates[coord_idx + 2],
                ));
            }
        }
        
        positions
    }
    
    /// Update excitability for a single neuron (for live parameter changes)
    /// Returns true if successful, false if neuron doesn't exist
    pub fn update_neuron_excitability(&mut self, neuron_id: u32, excitability: f32) -> bool {
        let idx = neuron_id as usize;
        if idx >= self.neuron_array.count || !self.neuron_array.valid_mask[idx] {
            return false;
        }
        
        self.neuron_array.excitabilities[idx] = excitability.clamp(0.0, 1.0);
        true
    }
    
    /// Update excitability for all neurons in a cortical area (for bulk parameter changes)
    /// Returns number of neurons updated
    pub fn update_cortical_area_excitability(&mut self, cortical_area: u32, excitability: f32) -> usize {
        let clamped_excitability = excitability.clamp(0.0, 1.0);
        let mut updated_count = 0;
        
        for neuron_id in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[neuron_id] 
                && self.neuron_array.cortical_areas[neuron_id] == cortical_area {
                self.neuron_array.excitabilities[neuron_id] = clamped_excitability;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update refractory period for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_refractory_period(&mut self, neuron_ids: &[u32], values: &[u16]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.refractory_periods[idx] = *value;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update threshold for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_threshold(&mut self, neuron_ids: &[u32], values: &[f32]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.thresholds[idx] = *value;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update leak coefficient for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_leak_coefficient(&mut self, neuron_ids: &[u32], values: &[f32]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.leak_coefficients[idx] = value.clamp(0.0, 1.0);
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update consecutive fire limit for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_consecutive_fire_limit(&mut self, neuron_ids: &[u32], values: &[u16]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.consecutive_fire_limits[idx] = *value;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update snooze period (extended refractory) for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_snooze_period(&mut self, neuron_ids: &[u32], values: &[u16]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.snooze_periods[idx] = *value;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Delete a neuron (mark as invalid)
    /// Returns true if successful, false if neuron out of bounds
    pub fn delete_neuron(&mut self, neuron_id: u32) -> bool {
        let idx = neuron_id as usize;
        if idx >= self.neuron_array.count {
            return false;
        }
        
        self.neuron_array.valid_mask[idx] = false;
        true
    }
    
    /// Get fire history for a specific burst
    pub fn get_fire_history(&self, burst: u64) -> Option<&FireHistory> {
        self.fire_ledger.get_burst(burst)
    }
    
    /// Get recent firing history
    pub fn get_recent_history(&self, count: usize) -> Vec<&FireHistory> {
        self.fire_ledger.get_recent_history(count)
    }
    
    /// Get neuron count
    pub fn get_neuron_count(&self) -> usize {
        self.neuron_array.count
    }
    
    /// Get synapse count (valid only)
    pub fn get_synapse_count(&self) -> usize {
        self.synapse_array.valid_count()
    }
    
    /// Get all outgoing synapses from a source neuron
    /// Returns Vec of (target_neuron_id, weight)
    pub fn get_outgoing_synapses(&self, source_neuron_id: u32) -> Vec<(u32, u8)> {
        let source = NeuronId(source_neuron_id);
        
        // Look up synapse indices for this source neuron
        let synapse_indices = match self.propagation_engine.synapse_index.get(&source) {
            Some(indices) => indices,
            None => return Vec::new(),  // No synapses from this neuron
        };
        
        // Collect all valid synapses
        let mut outgoing = Vec::new();
        for &syn_idx in synapse_indices {
            if syn_idx < self.synapse_array.count && self.synapse_array.valid_mask[syn_idx] {
                let target = self.synapse_array.target_neurons[syn_idx];
                let weight = self.synapse_array.weights[syn_idx];
                outgoing.push((target, weight));
            }
        }
        
        outgoing
    }
    
    /// Get neuron state for diagnostics (CFC, extended refractory, potential, etc.)
    /// Returns (cfc, cfc_limit, extended_refrac_period, potential, threshold, refrac_countdown)
    pub fn get_neuron_state(&self, neuron_id: NeuronId) -> Option<(u16, u16, u16, f32, f32, u16)> {
        let idx = neuron_id.0 as usize;
        if idx >= self.neuron_array.count || !self.neuron_array.valid_mask[idx] {
            return None;
        }
        
        Some((
            self.neuron_array.consecutive_fire_counts[idx],
            self.neuron_array.consecutive_fire_limits[idx],
            self.neuron_array.snooze_periods[idx],  // Extended refractory period (additive)
            self.neuron_array.membrane_potentials[idx],
            self.neuron_array.thresholds[idx],
            self.neuron_array.refractory_countdowns[idx],
        ))
    }
    
}

/// Phase 1 injection result
#[derive(Debug)]
struct InjectionResult {
    power_injections: usize,
    synaptic_injections: usize,
    sensory_injections: usize,
}

/// Modified Phase 1 injection that accepts synapse array
fn phase1_injection_with_synapses(
    fcl: &mut FireCandidateList,
    neuron_array: &NeuronArray,
    propagation_engine: &mut SynapticPropagationEngine,
    previous_fire_queue: &FireQueue,
    power_neurons: &[NeuronId],
    power_amount: f32,
    synapse_array: &SynapseArray,
) -> Result<InjectionResult> {
    // Clear FCL from previous burst
    fcl.clear();
    
    let mut power_count = 0;
    let mut synaptic_count = 0;
    
    // 1. Power Injection
    for &neuron_id in power_neurons {
        let idx = neuron_id.0 as usize;
        if idx < neuron_array.count {
            fcl.add_candidate(neuron_id, power_amount);
            power_count += 1;
        }
    }
    
    // 2. Synaptic Propagation
    if !previous_fire_queue.is_empty() {
        let fired_ids = previous_fire_queue.get_all_neuron_ids();
        
        // Call synaptic propagation engine (ZERO-COPY: pass synapse_array by reference)
        let propagation_result = propagation_engine.propagate(&fired_ids, synapse_array)?;
        
        // Inject propagated potentials into FCL
        for (_cortical_area, targets) in propagation_result {
            for &(target_neuron_id, contribution) in &targets {
                fcl.add_candidate(target_neuron_id, contribution.0);  // Extract f32 from SynapticContribution
                synaptic_count += 1;
            }
        }
    }
    
    Ok(InjectionResult {
        power_injections: power_count,
        synaptic_injections: synaptic_count,
        sensory_injections: 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_npu_creation() {
        let npu = RustNPU::new(1000, 10000, 20);
        assert_eq!(npu.get_neuron_count(), 0);
        assert_eq!(npu.get_synapse_count(), 0);
        assert_eq!(npu.get_burst_count(), 0);
    }

    #[test]
    fn test_add_neurons() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        // (threshold, decay_rate, leak_coeff, resting_pot, neuron_type, refrac_period, excitability, consec_fire_limit, cortical_area, x, y, z)
        let id1 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 1, 0, 0, 0, 0).unwrap();
        let id2 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 1, 1, 0, 0, 0).unwrap();
        
        assert_eq!(id1.0, 0);
        assert_eq!(id2.0, 1);
        assert_eq!(npu.get_neuron_count(), 2);
    }

    #[test]
    fn test_add_synapses() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        let n1 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 1, 0, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 1, 1, 0, 0, 0).unwrap();
        
        npu.add_synapse(
            n1,
            n2,
            SynapticWeight(128),
            SynapticConductance(255),
            SynapseType::Excitatory,
        ).unwrap();
        
        assert_eq!(npu.get_synapse_count(), 1);
    }

    #[test]
    fn test_burst_processing() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        // Add a power neuron
        let power_neuron = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 1, 0, 0, 0, 0).unwrap();
        
        // Process burst with power injection
        let result = npu.process_burst(&[power_neuron]).unwrap();
        
        assert_eq!(result.burst, 1);
        assert_eq!(result.power_injections, 1);
        assert_eq!(result.neuron_count, 1);  // Power neuron should fire
    }

    #[test]
    fn test_synapse_removal() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        let n1 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 1, 0, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 1, 1, 0, 0, 0).unwrap();
        
        npu.add_synapse(n1, n2, SynapticWeight(128), SynapticConductance(255), SynapseType::Excitatory).unwrap();
        assert_eq!(npu.get_synapse_count(), 1);
        
        assert!(npu.remove_synapse(n1, n2));
        assert_eq!(npu.get_synapse_count(), 0);
    }
}
