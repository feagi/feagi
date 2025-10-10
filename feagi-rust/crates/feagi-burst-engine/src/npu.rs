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
use crate::fire_structures::{FireQueue, FiringNeuron};
use crate::fire_ledger::RustFireLedger;
use crate::fq_sampler::{FQSampler, SamplingMode};
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
    fire_ledger: RustFireLedger,
    fq_sampler: FQSampler,
    
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
            fire_ledger: RustFireLedger::new(fire_ledger_window),
            fq_sampler: FQSampler::new(10.0, SamplingMode::Unified), // Default: 10Hz, unified mode
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
        
        // Phase 3: Archival (ZERO-COPY archive to Fire Ledger)
        self.fire_ledger.archive_burst(self.burst_count, &dynamics_result.fire_queue);
        
        // Phase 4: Swap fire queues (current becomes previous for next burst)
        self.previous_fire_queue = self.current_fire_queue.clone();
        self.current_fire_queue = dynamics_result.fire_queue.clone();
        
        // Phase 5: Cleanup (clear FCL for next burst)
        self.fire_candidate_list.clear();
        
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
        
        // CRITICAL: Iterate by ARRAY INDEX (not neuron_id!)
        for idx in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[idx] 
                && self.neuron_array.cortical_areas[idx] == cortical_area {
                self.neuron_array.excitabilities[idx] = clamped_excitability;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Update refractory period for all neurons in a cortical area (bulk parameter change)
    pub fn update_cortical_area_refractory_period(&mut self, cortical_area: u32, refractory_period: u16) -> usize {
        println!("[RUST-UPDATE] update_cortical_area_refractory_period: cortical_area={}, refractory_period={}", 
                 cortical_area, refractory_period);
        
        let mut updated_count = 0;

        // CRITICAL: Iterate by ARRAY INDEX (not neuron_id!)
        for idx in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[idx]
                && self.neuron_array.cortical_areas[idx] == cortical_area
            {
                // Get the actual neuron_id for this array index
                let neuron_id = self.neuron_array.index_to_neuron_id[idx];
                
                // Update base refractory period (used when neuron fires)
                self.neuron_array.refractory_periods[idx] = refractory_period;

                // CRITICAL FIX: Do NOT set countdown here!
                // The countdown should only be set AFTER a neuron fires.
                // Setting it now would block the neuron immediately, which is backward.
                // 
                // Correct behavior:
                // 1. Neuron fires → countdown = refractory_period
                // 2. Next burst: countdown > 0 → BLOCKED
                // 3. Decrement countdown each burst
                // 4. When countdown = 0 → neuron can fire again
                //
                // If we set countdown=refractory_period NOW (before firing),
                // the neuron would be blocked for N bursts FIRST, then fire.
                // That's backward!
                
                // Only clear countdown if setting refractory to 0 (allow immediate firing)
                if refractory_period == 0 {
                    self.neuron_array.refractory_countdowns[idx] = 0;
                }
                
                // Reset consecutive fire count when applying a new period to avoid
                // stale state causing unexpected immediate extended refractory.
                self.neuron_array.consecutive_fire_counts[idx] = 0;

                updated_count += 1;
                
                // Log first few neurons (show actual neuron_id, not array index!)
                if updated_count <= 3 {
                    println!("[RUST-BATCH-UPDATE]   Neuron {}: refractory_period={}, countdown={}", 
                             neuron_id, refractory_period, self.neuron_array.refractory_countdowns[idx]);
                }
            }
        }

        updated_count
    }
    
    /// Update threshold for all neurons in a cortical area (bulk parameter change)
    pub fn update_cortical_area_threshold(&mut self, cortical_area: u32, threshold: f32) -> usize {
        let mut updated_count = 0;
        
        // CRITICAL: Iterate by ARRAY INDEX (not neuron_id!)
        for idx in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[idx] 
                && self.neuron_array.cortical_areas[idx] == cortical_area {
                self.neuron_array.thresholds[idx] = threshold;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Update leak coefficient for all neurons in a cortical area (bulk parameter change)
    pub fn update_cortical_area_leak(&mut self, cortical_area: u32, leak: f32) -> usize {
        let mut updated_count = 0;
        
        // CRITICAL: Iterate by ARRAY INDEX (not neuron_id!)
        for idx in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[idx] 
                && self.neuron_array.cortical_areas[idx] == cortical_area {
                self.neuron_array.leak_coefficients[idx] = leak;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Update consecutive fire limit for all neurons in a cortical area (bulk parameter change)
    pub fn update_cortical_area_consecutive_fire_limit(&mut self, cortical_area: u32, limit: u16) -> usize {
        let mut updated_count = 0;
        
        // CRITICAL: Iterate by ARRAY INDEX (not neuron_id!)
        for idx in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[idx] 
                && self.neuron_array.cortical_areas[idx] == cortical_area {
                self.neuron_array.consecutive_fire_limits[idx] = limit;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Update snooze period (extended refractory) for all neurons in a cortical area (bulk parameter change)
    pub fn update_cortical_area_snooze_period(&mut self, cortical_area: u32, snooze_period: u16) -> usize {
        let mut updated_count = 0;
        
        // CRITICAL: Iterate by ARRAY INDEX (not neuron_id!)
        for idx in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[idx] 
                && self.neuron_array.cortical_areas[idx] == cortical_area {
                self.neuron_array.snooze_periods[idx] = snooze_period;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update refractory period for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_refractory_period(&mut self, neuron_ids: &[u32], values: &[u16]) -> usize {
        println!("[RUST-BATCH-UPDATE] batch_update_refractory_period: {} neurons", neuron_ids.len());
        
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                // Update base period
                self.neuron_array.refractory_periods[idx] = *value;
                // Enforce immediately: set countdown to new period (or 0)
                if *value > 0 {
                    self.neuron_array.refractory_countdowns[idx] = *value;
                } else {
                    self.neuron_array.refractory_countdowns[idx] = 0;
                }
                // Reset consecutive fire count to avoid stale extended refractory state
                self.neuron_array.consecutive_fire_counts[idx] = 0;
                updated_count += 1;
                
                // Log first few neurons and any that match our monitored neuron 16438
                if updated_count <= 3 || *neuron_id == 16438 {
                    println!("[RUST-BATCH-UPDATE]   Neuron {}: refractory_period={}, countdown={}", 
                             neuron_id, value, self.neuron_array.refractory_countdowns[idx]);
                }
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
    
    /// Batch update membrane potential for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_membrane_potential(&mut self, neuron_ids: &[u32], values: &[f32]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.membrane_potentials[idx] = *value;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update resting potential for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_resting_potential(&mut self, neuron_ids: &[u32], values: &[f32]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.resting_potentials[idx] = *value;
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update excitability for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_excitability(&mut self, neuron_ids: &[u32], values: &[f32]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.excitabilities[idx] = value.clamp(0.0, 1.0);
                updated_count += 1;
            }
        }
        
        updated_count
    }
    
    /// Batch update neuron type for multiple neurons
    /// Returns number of neurons updated
    pub fn batch_update_neuron_type(&mut self, neuron_ids: &[u32], values: &[i32]) -> usize {
        if neuron_ids.len() != values.len() {
            return 0;
        }
        
        let mut updated_count = 0;
        for (neuron_id, value) in neuron_ids.iter().zip(values.iter()) {
            let idx = *neuron_id as usize;
            if idx < self.neuron_array.count && self.neuron_array.valid_mask[idx] {
                self.neuron_array.neuron_types[idx] = *value;
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
    pub fn get_outgoing_synapses(&self, source_neuron_id: u32) -> Vec<(u32, u8, u8, u8)> {
        let source = NeuronId(source_neuron_id);
        
        // Look up synapse indices for this source neuron
        let synapse_indices = match self.propagation_engine.synapse_index.get(&source) {
            Some(indices) => indices,
            None => return Vec::new(),  // No synapses from this neuron
        };
        
        // Collect all valid synapses with full properties
        let mut outgoing = Vec::new();
        for &syn_idx in synapse_indices {
            if syn_idx < self.synapse_array.count && self.synapse_array.valid_mask[syn_idx] {
                let target = self.synapse_array.target_neurons[syn_idx];
                let weight = self.synapse_array.weights[syn_idx];
                let conductance = self.synapse_array.conductances[syn_idx];
                let synapse_type = self.synapse_array.types[syn_idx];
                outgoing.push((target, weight, conductance, synapse_type));
            }
        }
        
        outgoing
    }
    
    /// Get incoming synapses for a neuron (neuron is the target)
    /// Returns Vec<(source_neuron_id, weight, conductance, synapse_type)>
    pub fn get_incoming_synapses(&self, target_neuron_id: u32) -> Vec<(u32, u8, u8, u8)> {
        let mut synapses = Vec::new();
        
        // Iterate through all synapses to find ones targeting this neuron
        // Note: This is O(n) - we could optimize with a target_index HashMap if needed
        for i in 0..self.synapse_array.count {
            if self.synapse_array.valid_mask[i] 
                && self.synapse_array.target_neurons[i] == target_neuron_id {
                synapses.push((
                    self.synapse_array.source_neurons[i],
                    self.synapse_array.weights[i],
                    self.synapse_array.conductances[i],
                    self.synapse_array.types[i],
                ));
            }
        }
        
        synapses
    }
    
    /// Get neuron state for diagnostics (CFC, extended refractory, potential, etc.)
    /// Returns (cfc, cfc_limit, extended_refrac_period, potential, threshold, refrac_countdown)
    pub fn get_neuron_state(&self, neuron_id: NeuronId) -> Option<(u16, u16, u16, f32, f32, u16)> {
        // CRITICAL: Use neuron_id_to_index HashMap to convert ID to array index
        let idx = *self.neuron_array.neuron_id_to_index.get(&neuron_id.0)?;
        
        // Validate index (should always be valid if in HashMap, but check anyway)
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
        // CRITICAL: Use neuron_id_to_index HashMap to convert ID to array index
        if let Some(&idx) = neuron_array.neuron_id_to_index.get(&neuron_id.0) {
            if idx < neuron_array.count && neuron_array.valid_mask[idx] {
            fcl.add_candidate(neuron_id, power_amount);
            power_count += 1;
            }
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

// ═══════════════════════════════════════════════════════════
// Fire Ledger API (Extension of RustNPU impl)
// ═══════════════════════════════════════════════════════════
impl RustNPU {
    /// Get firing history for a cortical area from Fire Ledger
    /// Returns Vec of (timestep, Vec<neuron_id>) tuples, newest first
    pub fn get_fire_ledger_history(&self, cortical_idx: u32, lookback_steps: usize) -> Vec<(u64, Vec<u32>)> {
        self.fire_ledger.get_history(cortical_idx, lookback_steps)
    }
    
    /// Get Fire Ledger window size for a cortical area
    pub fn get_fire_ledger_window_size(&self, cortical_idx: u32) -> usize {
        self.fire_ledger.get_area_window_size(cortical_idx)
    }
    
    /// Configure Fire Ledger window size for a specific cortical area
    pub fn configure_fire_ledger_window(&mut self, cortical_idx: u32, window_size: usize) {
        self.fire_ledger.configure_area_window(cortical_idx, window_size);
    }
    
    /// Get all configured Fire Ledger window sizes
    pub fn get_all_fire_ledger_configs(&self) -> Vec<(u32, usize)> {
        self.fire_ledger.get_all_window_configs()
    }
}

// ═══════════════════════════════════════════════════════════
// FQ Sampler API (Entry Point #2: Motor/Visualization Output)
// ═══════════════════════════════════════════════════════════
impl RustNPU {
    /// Sample the current Fire Queue for visualization/motor output
    /// 
    /// Returns None if:
    /// - Rate limit not met
    /// - Fire Queue is empty
    /// - Burst already sampled (deduplication)
    /// 
    /// Returns HashMap of cortical_idx -> area data
    pub fn sample_fire_queue(&mut self) -> Option<AHashMap<u32, (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, Vec<f32>)>> {
        let sample_result = self.fq_sampler.sample(&self.current_fire_queue)?;
        
        // Convert to Python-friendly format
        let mut result = AHashMap::new();
        for (cortical_idx, area_data) in sample_result.areas {
            result.insert(
                cortical_idx,
                (
                    area_data.neuron_ids,
                    area_data.coordinates_x,
                    area_data.coordinates_y,
                    area_data.coordinates_z,
                    area_data.potentials,
                )
            );
        }
        
        Some(result)
    }
    
    /// Get current Fire Queue directly (bypasses FQ Sampler rate limiting)
    /// Used by FCL endpoint to get real-time firing data without sampling delays
    pub fn get_current_fire_queue(&self) -> AHashMap<u32, (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, Vec<f32>)> {
        let mut result = AHashMap::new();
        
        // Convert current Fire Queue to the same format as sample_fire_queue
        for (cortical_idx, neurons) in &self.current_fire_queue.neurons_by_area {
            let mut neuron_ids = Vec::with_capacity(neurons.len());
            let mut coords_x = Vec::with_capacity(neurons.len());
            let mut coords_y = Vec::with_capacity(neurons.len());
            let mut coords_z = Vec::with_capacity(neurons.len());
            let mut potentials = Vec::with_capacity(neurons.len());
            
            for neuron in neurons {
                neuron_ids.push(neuron.neuron_id.0);
                coords_x.push(neuron.x);
                coords_y.push(neuron.y);
                coords_z.push(neuron.z);
                potentials.push(neuron.membrane_potential);
            }
            
            result.insert(*cortical_idx, (neuron_ids, coords_x, coords_y, coords_z, potentials));
        }
        
        result
    }
    
    /// Set FQ Sampler frequency (Hz)
    pub fn set_fq_sampler_frequency(&mut self, frequency_hz: f64) {
        self.fq_sampler.set_sample_frequency(frequency_hz);
    }
    
    /// Get FQ Sampler frequency (Hz)
    pub fn get_fq_sampler_frequency(&self) -> f64 {
        self.fq_sampler.get_sample_frequency()
    }
    
    /// Set visualization subscriber state
    pub fn set_visualization_subscribers(&mut self, has_subscribers: bool) {
        self.fq_sampler.set_visualization_subscribers(has_subscribers);
    }
    
    /// Check if visualization subscribers are connected
    pub fn has_visualization_subscribers(&self) -> bool {
        self.fq_sampler.has_visualization_subscribers()
    }
    
    /// Set motor subscriber state
    pub fn set_motor_subscribers(&mut self, has_subscribers: bool) {
        self.fq_sampler.set_motor_subscribers(has_subscribers);
    }
    
    /// Check if motor subscribers are connected
    pub fn has_motor_subscribers(&self) -> bool {
        self.fq_sampler.has_motor_subscribers()
    }
    
    /// Get total FQ Sampler samples taken
    pub fn get_fq_sampler_samples_taken(&self) -> u64 {
        self.fq_sampler.get_samples_taken()
    }
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
        
        // (threshold, leak_coeff, resting_pot, neuron_type, refrac_period, excitability, consec_fire_limit, cortical_area, x, y, z, snooze_period)
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
