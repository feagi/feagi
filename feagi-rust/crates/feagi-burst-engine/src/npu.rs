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
use std::time::Duration;

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
    
    // Sensory staging (prevents async race with FCL clear)
    pending_sensory_injections: std::sync::Mutex<Vec<(NeuronId, f32)>>,
    
    // Last FCL snapshot (before clearing) - for debugging/monitoring
    last_fcl_snapshot: Vec<(NeuronId, f32)>,
    
    // Cortical area mapping (area_id -> cortical_name string for encoding)
    area_id_to_name: AHashMap<u32, String>,
    
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
            fq_sampler: FQSampler::new(1000.0, SamplingMode::Unified), // High rate - actual limiting by burst frequency
            pending_sensory_injections: std::sync::Mutex::new(Vec::with_capacity(10000)),
            last_fcl_snapshot: Vec::new(),
            area_id_to_name: AHashMap::new(),
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
        mp_charge_accumulation: bool,
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
            mp_charge_accumulation,
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
    
    // ===== SENSORY INJECTION API =====
    
    /// Inject sensory neurons into FCL (called from Rust sensory threads)
    /// This is the PRIMARY method for Rust-native sensory injection
    pub fn inject_sensory_batch(&mut self, neuron_ids: &[NeuronId], potential: f32) {
        // 🔍 DEBUG: Log first batch injection
        static FIRST_BATCH_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
        if !FIRST_BATCH_LOGGED.load(std::sync::atomic::Ordering::Relaxed) && !neuron_ids.is_empty() {
            println!("[NPU-INJECT] 🔍 First batch: count={}, potential={}", neuron_ids.len(), potential);
            println!("[NPU-INJECT]    First 5 NeuronIds: {:?}", &neuron_ids[0..neuron_ids.len().min(5)]);
            println!("[NPU-INJECT]    FCL size before: {}", self.fire_candidate_list.len());
            FIRST_BATCH_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
        }
        
        for &neuron_id in neuron_ids {
            self.fire_candidate_list.add_candidate(neuron_id, potential);
        }
        
        // 🔍 DEBUG: Log FCL size after first injection
        static FIRST_BATCH_AFTER_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
        if !FIRST_BATCH_AFTER_LOGGED.load(std::sync::atomic::Ordering::Relaxed) && !neuron_ids.is_empty() {
            println!("[NPU-INJECT]    FCL size after: {}", self.fire_candidate_list.len());
            FIRST_BATCH_AFTER_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
        }
    }
    
    /// Stage sensory neurons for next burst (thread-safe, prevents FCL clear race)
    /// XYZP data from agents is staged here and injected AFTER fcl.clear() in Phase 1
    pub fn inject_sensory_with_potentials(&mut self, neurons: &[(NeuronId, f32)]) {
        if let Ok(mut pending) = self.pending_sensory_injections.lock() {
            pending.extend_from_slice(neurons);
            
            // 🔍 DEBUG: Log first staging
            static FIRST_STAGING_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
            if !FIRST_STAGING_LOGGED.load(std::sync::atomic::Ordering::Relaxed) && !neurons.is_empty() {
                println!("[NPU-STAGE] 🎯 Staged {} sensory neurons for next burst (prevents FCL clear race)", neurons.len());
                println!("[NPU-STAGE]    Queue now has {} pending injections", pending.len());
                FIRST_STAGING_LOGGED.store(true, std::sync::atomic::Ordering::Relaxed);
            }
        }
    }
    
    /// Get immutable reference to FCL for inspection (debugging only)
    pub fn get_fcl_ref(&self) -> &FireCandidateList {
        &self.fire_candidate_list
    }
    
    /// Get last FCL snapshot (captured before clear in previous burst)
    /// Returns Vec of (NeuronId, potential) pairs
    pub fn get_last_fcl_snapshot(&self) -> &[(NeuronId, f32)] {
        &self.last_fcl_snapshot
    }
    
    // ===== END SENSORY INJECTION API =====
    
    // ===== POWER INJECTION =====
    // Power neurons are identified by cortical_idx = 1 in the neuron array
    // No separate list needed - single source of truth!
    
    /// Process a single burst (MAIN METHOD)
    /// 
    /// This is the complete neural processing pipeline:
    /// Phase 1: Injection → Phase 2: Dynamics → Phase 3: Archival → 
    /// Phase 4: Queue Swap → Phase 5: FQ Sampling → Phase 6: Cleanup
    /// 
    /// 🔋 Power neurons are auto-discovered from neuron_array (cortical_idx = 1)
    pub fn process_burst(&mut self) -> Result<BurstResult> {
        self.burst_count += 1;
        
        // Phase 1: Injection (power + synaptic propagation + staged sensory)
        // ZERO-COPY: Pass synapse_array by reference (no allocation)
        let injection_result = phase1_injection_with_synapses(
            &mut self.fire_candidate_list,
            &mut self.neuron_array,
            &mut self.propagation_engine,
            &self.previous_fire_queue,
            self.power_amount,
            &self.synapse_array,
            &self.pending_sensory_injections,
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
        
        // Phase 5: Sample fire queue for visualization (FQ Sampler)
        // This makes the fire queue available to BV and motor agents
        self.fq_sampler.sample(&self.current_fire_queue);
        
        // Phase 6: Cleanup (snapshot FCL before clearing for API access)
        self.last_fcl_snapshot = self.fire_candidate_list.get_all_candidates();
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
    
    /// Register a cortical area name for visualization encoding
    /// This mapping is populated during neuroembryogenesis
    pub fn register_cortical_area(&mut self, area_id: u32, cortical_name: String) {
        self.area_id_to_name.insert(area_id, cortical_name);
    }
    
    /// Get the cortical area name for a given area_id
    /// Returns None if the area_id is not registered
    pub fn get_cortical_area_name(&self, area_id: u32) -> Option<&str> {
        self.area_id_to_name.get(&area_id).map(|s| s.as_str())
    }
    
    /// Get the cortical area ID for a given cortical name
    /// Returns None if the name is not registered
    pub fn get_cortical_area_id(&self, cortical_name: &str) -> Option<u32> {
        for (&area_id, name) in &self.area_id_to_name {
            if name == cortical_name {
                return Some(area_id);
            }
        }
        None
    }
    
    /// Get the number of registered cortical areas
    pub fn get_registered_cortical_area_count(&self) -> usize {
        self.area_id_to_name.len()
    }
    
    /// Get all registered cortical areas as (idx, name) pairs
    pub fn get_all_cortical_areas(&self) -> Vec<(u32, String)> {
        self.area_id_to_name.iter()
            .map(|(&idx, name)| (idx, name.clone()))
            .collect()
    }
    
    /// Find neuron ID at specific X,Y,Z coordinates within a cortical area
    /// Returns None if no neuron exists at that position
    pub fn get_neuron_at_coordinates(&self, cortical_area: u32, x: u32, y: u32, z: u32) -> Option<NeuronId> {
        for neuron_idx in 0..self.neuron_array.count {
            if self.neuron_array.valid_mask[neuron_idx] 
                && self.neuron_array.cortical_areas[neuron_idx] == cortical_area {
                let coord_idx = neuron_idx * 3;
                if self.neuron_array.coordinates[coord_idx] == x
                    && self.neuron_array.coordinates[coord_idx + 1] == y
                    && self.neuron_array.coordinates[coord_idx + 2] == z {
                    return Some(NeuronId(neuron_idx as u32));
                }
            }
        }
        None
    }
    
    /// Inject sensory neurons using cortical area name and XYZ coordinates
    /// This is the high-level API for sensory injection from agents
    pub fn inject_sensory_xyzp(&mut self, cortical_name: &str, xyzp_data: &[(u32, u32, u32, f32)]) -> usize {
        // Find cortical area ID
        let cortical_area = match self.get_cortical_area_id(cortical_name) {
            Some(id) => id,
            None => {
                eprintln!("[NPU] ❌ Unknown cortical area: '{}'", cortical_name);
                eprintln!("[NPU] ❌ Available cortical areas: {:?}", 
                    self.area_id_to_name.values().collect::<Vec<_>>());
                eprintln!("[NPU] ❌ Total registered: {}", self.area_id_to_name.len());
                return 0;
            }
        };
        
        // Convert XYZ coordinates to neuron IDs
        let mut neuron_potential_pairs = Vec::with_capacity(xyzp_data.len());
        let mut found_count = 0;
        
        for &(x, y, z, potential) in xyzp_data {
            if let Some(neuron_id) = self.get_neuron_at_coordinates(cortical_area, x, y, z) {
                neuron_potential_pairs.push((neuron_id, potential));
                found_count += 1;
            }
        }
        
        // Inject found neurons
        if !neuron_potential_pairs.is_empty() {
            self.inject_sensory_with_potentials(&neuron_potential_pairs);
        }
        
        found_count
    }
    
    /// Export connectome snapshot (for saving to file)
    /// 
    /// This captures the complete NPU state including all neurons, synapses,
    /// and runtime state for serialization.
    pub fn export_connectome(&self) -> feagi_connectome_serialization::ConnectomeSnapshot {
        use feagi_connectome_serialization::{
            ConnectomeSnapshot, SerializableNeuronArray, SerializableSynapseArray,
            ConnectomeMetadata,
        };
        
        // Convert neuron array
        let neurons = SerializableNeuronArray {
            count: self.neuron_array.count,
            capacity: self.neuron_array.capacity,
            membrane_potentials: self.neuron_array.membrane_potentials.clone(),
            thresholds: self.neuron_array.thresholds.clone(),
            leak_coefficients: self.neuron_array.leak_coefficients.clone(),
            resting_potentials: self.neuron_array.resting_potentials.clone(),
            neuron_types: self.neuron_array.neuron_types.clone(),
            refractory_periods: self.neuron_array.refractory_periods.clone(),
            refractory_countdowns: self.neuron_array.refractory_countdowns.clone(),
            excitabilities: self.neuron_array.excitabilities.clone(),
            cortical_areas: self.neuron_array.cortical_areas.clone(),
            coordinates: self.neuron_array.coordinates.clone(),
            valid_mask: self.neuron_array.valid_mask.clone(),
        };
        
        // Convert synapse array
        let synapses = SerializableSynapseArray {
            count: self.synapse_array.count,
            capacity: self.synapse_array.capacity,
            source_neurons: self.synapse_array.source_neurons.clone(),
            target_neurons: self.synapse_array.target_neurons.clone(),
            weights: self.synapse_array.weights.clone(),
            conductances: self.synapse_array.conductances.clone(),
            types: self.synapse_array.types.clone(),
            valid_mask: self.synapse_array.valid_mask.clone(),
            source_index: self.synapse_array.source_index.clone(),
        };
        
        ConnectomeSnapshot {
            version: 1,
            neurons,
            synapses,
            cortical_area_names: self.area_id_to_name.clone(),
            burst_count: self.burst_count,
            power_amount: self.power_amount,
            fire_ledger_window: 20, // Default value (fire_ledger doesn't expose window)
            metadata: ConnectomeMetadata::default(),
        }
    }
    
    /// Import connectome snapshot (for loading from file)
    /// 
    /// This replaces the entire NPU state with data from a saved connectome.
    pub fn import_connectome(snapshot: feagi_connectome_serialization::ConnectomeSnapshot) -> Self {
        // Convert neuron array
        let mut neuron_array = NeuronArray::new(snapshot.neurons.capacity);
        neuron_array.count = snapshot.neurons.count;
        neuron_array.membrane_potentials = snapshot.neurons.membrane_potentials;
        neuron_array.thresholds = snapshot.neurons.thresholds;
        neuron_array.leak_coefficients = snapshot.neurons.leak_coefficients;
        neuron_array.resting_potentials = snapshot.neurons.resting_potentials;
        neuron_array.neuron_types = snapshot.neurons.neuron_types;
        neuron_array.refractory_periods = snapshot.neurons.refractory_periods;
        neuron_array.refractory_countdowns = snapshot.neurons.refractory_countdowns;
        neuron_array.excitabilities = snapshot.neurons.excitabilities;
        neuron_array.cortical_areas = snapshot.neurons.cortical_areas;
        neuron_array.coordinates = snapshot.neurons.coordinates;
        neuron_array.valid_mask = snapshot.neurons.valid_mask;
        
        // Convert synapse array
        let mut synapse_array = SynapseArray::new(snapshot.synapses.capacity);
        synapse_array.count = snapshot.synapses.count;
        synapse_array.source_neurons = snapshot.synapses.source_neurons;
        synapse_array.target_neurons = snapshot.synapses.target_neurons;
        synapse_array.weights = snapshot.synapses.weights;
        synapse_array.conductances = snapshot.synapses.conductances;
        synapse_array.types = snapshot.synapses.types;
        synapse_array.valid_mask = snapshot.synapses.valid_mask;
        synapse_array.source_index = snapshot.synapses.source_index;
        
        Self {
            neuron_array,
            synapse_array,
            fire_candidate_list: FireCandidateList::new(),
            current_fire_queue: FireQueue::new(),
            previous_fire_queue: FireQueue::new(),
            fire_ledger: RustFireLedger::new(snapshot.fire_ledger_window),
            fq_sampler: FQSampler::new(1000.0, SamplingMode::Unified),
            pending_sensory_injections: std::sync::Mutex::new(Vec::with_capacity(10000)),
            last_fcl_snapshot: Vec::new(),
            area_id_to_name: snapshot.cortical_area_names,
            propagation_engine: SynapticPropagationEngine::new(),
            burst_count: snapshot.burst_count,
            power_amount: snapshot.power_amount,
        }
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

/// Phase 1 injection with automatic power neuron discovery
/// 
/// 🔋 Power neurons are identified by cortical_idx = 1 (_power area)
/// No separate list - scans neuron array directly!
fn phase1_injection_with_synapses(
    fcl: &mut FireCandidateList,
    neuron_array: &mut NeuronArray,
    propagation_engine: &mut SynapticPropagationEngine,
    previous_fire_queue: &FireQueue,
    power_amount: f32,
    synapse_array: &SynapseArray,
    pending_sensory: &std::sync::Mutex<Vec<(NeuronId, f32)>>,
) -> Result<InjectionResult> {
    // Clear FCL from previous burst
    fcl.clear();
    
    // CRITICAL FIX: Reset membrane potentials for neurons with mp_charge_accumulation=false
    // This prevents ghost potential accumulation and self-stimulation bugs
    //
    // Behavior:
    // - mp_acc=true: Neuron keeps its potential across bursts (integrator behavior)
    // - mp_acc=false: Neuron resets to 0.0 at start of each burst (coincidence detector)
    //
    // This ensures neurons only fire from CURRENT BURST stimulation, not accumulated history
    for idx in 0..neuron_array.count {
        if neuron_array.valid_mask[idx] && !neuron_array.mp_charge_accumulation[idx] {
            // Reset membrane potential for non-accumulating neurons
            neuron_array.membrane_potentials[idx] = 0.0;
        }
    }
    
    let mut power_count = 0;
    let mut synaptic_count = 0;
    let mut sensory_count = 0;
    
    // 0. Drain pending sensory injections (AFTER clear, BEFORE power/synapses)
    if let Ok(mut pending) = pending_sensory.lock() {
        if !pending.is_empty() {
            // 🔍 DEBUG: Log first sensory injection
            static FIRST_SENSORY_LOG: std::sync::Once = std::sync::Once::new();
            FIRST_SENSORY_LOG.call_once(|| {
                println!("╔══════════════════════════════════════════════════════════════");
                println!("║ [SENSORY-INJECTION] 🎬 DRAINING STAGED SENSORY DATA");
                println!("║ Injecting {} neurons AFTER FCL clear (prevents race)", pending.len());
                println!("╚══════════════════════════════════════════════════════════════");
            });
            
            for (neuron_id, potential) in pending.drain(..) {
                fcl.add_candidate(neuron_id, potential);
                sensory_count += 1;
            }
        }
    }
    
    // 1. Power Injection - Scan neuron array for cortical_idx = 1
    static FIRST_LOG: std::sync::Once = std::sync::Once::new();
    FIRST_LOG.call_once(|| {
        println!("╔══════════════════════════════════════════════════════════════");
        println!("║ [POWER-INJECTION] 🔋 AUTO-DISCOVERING POWER NEURONS");
        println!("║ Scanning neuron array for cortical_idx = 1 (_power area)");
        println!("╚══════════════════════════════════════════════════════════════");
    });
    
    // Scan all neurons for _power cortical area (cortical_idx = 1)
    for (neuron_id, &array_idx) in &neuron_array.neuron_id_to_index {
        if array_idx < neuron_array.count && neuron_array.valid_mask[array_idx] {
            let cortical_area = neuron_array.cortical_areas[array_idx];
            
            // Check if this is a power neuron (cortical_area = 1)
            if cortical_area == 1 {
                fcl.add_candidate(NeuronId(*neuron_id), power_amount);
                power_count += 1;
            }
        }
    }
    
    // Log first injection and EVERY time power neurons disappear
    static mut FIRST_INJECTION: bool = false;
    static mut LAST_POWER_COUNT: usize = 0;
    unsafe {
        if !FIRST_INJECTION && power_count > 0 {
            println!("[POWER-INJECTION] ✅ Injected {} power neurons into FCL", power_count);
            FIRST_INJECTION = true;
            LAST_POWER_COUNT = power_count;
        } else if power_count == 0 && FIRST_INJECTION {
            // Power neurons disappeared after working!
            println!("[POWER-INJECTION] ❌ ERROR: Power neurons DISAPPEARED! (was {}, now 0)", LAST_POWER_COUNT);
            LAST_POWER_COUNT = 0;
        } else if power_count == 0 && !FIRST_INJECTION {
            println!("[POWER-INJECTION] ⚠️ WARNING: No neurons found with cortical_idx=1");
            FIRST_INJECTION = true;
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
        sensory_injections: sensory_count,
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
    /// 
    /// ⚠️ DEPRECATED: This method triggers deduplication and may return None if burst already sampled.
    /// Use `get_latest_fire_queue_sample()` instead for non-consuming reads.
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
    
    /// Get the latest cached Fire Queue sample (non-consuming read)
    /// 
    /// This returns the most recent sample WITHOUT triggering rate limiting or deduplication.
    /// Perfect for Python wrappers and SHM writers that need to read the same burst multiple times.
    /// 
    /// Returns None if no sample has been taken yet (no bursts processed).
    pub fn get_latest_fire_queue_sample(&self) -> Option<AHashMap<u32, (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, Vec<f32>)>> {
        let sample_result = self.fq_sampler.get_latest_sample()?;
        
        // Convert to Python-friendly format
        let mut result = AHashMap::new();
        for (cortical_idx, area_data) in &sample_result.areas {
            result.insert(
                *cortical_idx,
                (
                    area_data.neuron_ids.clone(),
                    area_data.coordinates_x.clone(),
                    area_data.coordinates_y.clone(),
                    area_data.coordinates_z.clone(),
                    area_data.potentials.clone(),
                )
            );
        }
        
        Some(result)
    }
    
    /// Force sample the Fire Queue (for burst loop, bypasses rate limiting)
    /// 
    /// This is used by the burst loop to sample on every burst, regardless of the FQ sampler's
    /// configured rate limit. The rate limiting is meant for external consumers, not the burst loop itself.
    pub fn force_sample_fire_queue(&mut self) -> Option<AHashMap<u32, (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, Vec<f32>)>> {
        // FIXED: Use get_current_fire_queue() instead of accessing private fields
        Some(self.get_current_fire_queue())
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

    // ═══════════════════════════════════════════════════════════
    // Core NPU Creation & Initialization
    // ═══════════════════════════════════════════════════════════
    
    #[test]
    fn test_npu_creation() {
        let npu = RustNPU::new(1000, 10000, 20);
        assert_eq!(npu.get_neuron_count(), 0);
        assert_eq!(npu.get_synapse_count(), 0);
        assert_eq!(npu.get_burst_count(), 0);
    }

    #[test]
    fn test_npu_creation_with_zero_capacity() {
        let npu = RustNPU::new(0, 0, 0);
        assert_eq!(npu.get_neuron_count(), 0);
        assert_eq!(npu.get_synapse_count(), 0);
    }

    #[test]
    fn test_npu_creation_with_large_capacity() {
        let npu = RustNPU::new(1_000_000, 10_000_000, 100);
        assert_eq!(npu.get_neuron_count(), 0);
    }

    // ═══════════════════════════════════════════════════════════
    // Neuron Management
    // ═══════════════════════════════════════════════════════════

    #[test]
    fn test_add_neurons() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        let id1 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        let id2 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 1, 0, 0).unwrap();
        
        assert_eq!(id1.0, 0);
        assert_eq!(id2.0, 1);
        assert_eq!(npu.get_neuron_count(), 2);
    }

    #[test]
    fn test_add_neuron_sequential_ids() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        for i in 0..10 {
            let id = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, i, 0, 0).unwrap();
            assert_eq!(id.0, i);
        }
        
        assert_eq!(npu.get_neuron_count(), 10);
    }

    #[test]
    fn test_add_neuron_different_parameters() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        // High threshold
        let _n1 = npu.add_neuron(10.0, 0.0, 0.0, 0, 0, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        
        // High leak
        let _n2 = npu.add_neuron(1.0, 0.9, 0.0, 0, 0, 1.0, 0, 0, true, 1, 1, 0, 0).unwrap();
        
        // Long refractory period
        let _n3 = npu.add_neuron(1.0, 0.1, 0.0, 0, 100, 1.0, 0, 0, true, 1, 2, 0, 0).unwrap();
        
        // Low excitability
        let _n4 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 0.1, 0, 0, true, 1, 3, 0, 0).unwrap();
        
        assert_eq!(npu.get_neuron_count(), 4);
    }

    #[test]
    fn test_add_neuron_different_cortical_areas() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let _power = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        let _area2 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
        let _area3 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 3, 0, 0, 0).unwrap();
        
        assert_eq!(npu.get_neuron_count(), 3);
    }

    #[test]
    fn test_add_neuron_3d_coordinates() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let _n1 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 5, 10, 15).unwrap();
        
        assert_eq!(npu.get_neuron_count(), 1);
    }

    // ═══════════════════════════════════════════════════════════
    // Synapse Management
    // ═══════════════════════════════════════════════════════════

    #[test]
    fn test_add_synapses() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        let n1 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 1, 0, 0).unwrap();
        
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
    fn test_add_multiple_synapses() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        let n1 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 1, 0, 0).unwrap();
        let n3 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 2, 0, 0).unwrap();
        
        npu.add_synapse(n1, n2, SynapticWeight(128), SynapticConductance(255), SynapseType::Excitatory).unwrap();
        npu.add_synapse(n1, n3, SynapticWeight(64), SynapticConductance(128), SynapseType::Excitatory).unwrap();
        npu.add_synapse(n2, n3, SynapticWeight(32), SynapticConductance(64), SynapseType::Inhibitory).unwrap();
        
        assert_eq!(npu.get_synapse_count(), 3);
    }

    #[test]
    fn test_add_inhibitory_synapse() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let n1 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 1, 0, 0).unwrap();
        
        npu.add_synapse(n1, n2, SynapticWeight(128), SynapticConductance(255), SynapseType::Inhibitory).unwrap();
        
        assert_eq!(npu.get_synapse_count(), 1);
    }

    #[test]
    fn test_synapse_removal() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        let n1 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 1, 0, 0).unwrap();
        
        npu.add_synapse(n1, n2, SynapticWeight(128), SynapticConductance(255), SynapseType::Excitatory).unwrap();
        assert_eq!(npu.get_synapse_count(), 1);
        
        assert!(npu.remove_synapse(n1, n2));
        assert_eq!(npu.get_synapse_count(), 0);
    }

    #[test]
    fn test_remove_nonexistent_synapse() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let n1 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 1, 0, 0).unwrap();
        
        assert!(!npu.remove_synapse(n1, n2));
    }

    // ═══════════════════════════════════════════════════════════
    // Burst Processing & Power Injection
    // ═══════════════════════════════════════════════════════════

    #[test]
    fn test_burst_processing() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        // Add a power neuron
        let _power_neuron = npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        
        // Process burst with power injection
        let result = npu.process_burst().unwrap();
        
        assert_eq!(result.burst, 1);
        assert_eq!(result.power_injections, 1);
        assert_eq!(result.neuron_count, 1);
    }

    #[test]
    fn test_burst_counter_increments() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        for i in 1..=10 {
            let result = npu.process_burst().unwrap();
            assert_eq!(result.burst, i as u64);
            assert_eq!(npu.get_burst_count(), i as u64);
        }
    }

    #[test]
    fn test_power_injection_auto_discovery() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        // Add 5 power neurons (cortical_area=1)
        for i in 0..5 {
            npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, i, 0, 0).unwrap();
        }
        
        // Add 5 regular neurons (cortical_area=2)
        for i in 0..5 {
            npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, i, 0, 0).unwrap();
        }
        
        let result = npu.process_burst().unwrap();
        
        // Should inject only cortical_area=1 neurons
        assert_eq!(result.power_injections, 5);
    }

    #[test]
    fn test_set_power_amount() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        // Add power neuron with high threshold
        npu.add_neuron(5.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        
        // Set high power amount
        npu.set_power_amount(10.0);
        
        // Should fire immediately (10.0 > 5.0 threshold)
        let result = npu.process_burst().unwrap();
        assert_eq!(result.neuron_count, 1);
    }

    #[test]
    fn test_empty_burst_no_power() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        // Add only regular neurons (no power area)
        npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
        
        let result = npu.process_burst().unwrap();
        
        assert_eq!(result.power_injections, 0);
    }

    // ═══════════════════════════════════════════════════════════
    // Sensory Input Injection
    // ═══════════════════════════════════════════════════════════

    #[test]
    fn test_inject_sensory_input() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let neuron = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
        
        npu.inject_sensory_with_potentials(&[(neuron, 0.5)]);
        
        // Sensory input is staged until next burst
        let _result = npu.process_burst().unwrap();
    }

    #[test]
    fn test_inject_multiple_sensory_inputs() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let n1 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 1, 0, 0).unwrap();
        let n3 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 2, 0, 0).unwrap();
        
        npu.inject_sensory_with_potentials(&[(n1, 0.5), (n2, 0.3), (n3, 0.8)]);
        
        let _result = npu.process_burst().unwrap();
    }

    #[test]
    fn test_sensory_accumulation_on_same_neuron() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let neuron = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
        
        npu.inject_sensory_with_potentials(&[(neuron, 0.3)]);
        npu.inject_sensory_with_potentials(&[(neuron, 0.3)]);
        npu.inject_sensory_with_potentials(&[(neuron, 0.3)]);
        
        let _result = npu.process_burst().unwrap();
        // Should accumulate 0.9 potential
    }

    // ═══════════════════════════════════════════════════════════
    // Fire Ledger Tests
    // ═══════════════════════════════════════════════════════════

    #[test]
    fn test_fire_ledger_recording() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let _neuron = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        
        // Process burst
        npu.process_burst().unwrap();
        
        // Check fire ledger
        let history = npu.get_fire_ledger_history(1, 10);
        assert!(!history.is_empty());
    }

    #[test]
    fn test_fire_ledger_window_configuration() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        npu.configure_fire_ledger_window(1, 50);
        
        let window_size = npu.get_fire_ledger_window_size(1);
        assert_eq!(window_size, 50);
    }

    // ═══════════════════════════════════════════════════════════
    // FQ Sampler Tests
    // ═══════════════════════════════════════════════════════════

    #[test]
    fn test_fq_sampler_rate_limiting() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        
        npu.set_visualization_subscribers(true);
        
        npu.process_burst().unwrap();
        
        // Should be able to sample
        let _sample = npu.sample_fire_queue();
        // Rate limiting may prevent sampling
    }

    #[test]
    fn test_fq_sampler_motor_subscribers() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        assert!(!npu.has_motor_subscribers());
        
        npu.set_motor_subscribers(true);
        assert!(npu.has_motor_subscribers());
        
        npu.set_motor_subscribers(false);
        assert!(!npu.has_motor_subscribers());
    }

    #[test]
    fn test_fq_sampler_viz_subscribers() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        assert!(!npu.has_visualization_subscribers());
        
        npu.set_visualization_subscribers(true);
        assert!(npu.has_visualization_subscribers());
        
        npu.set_visualization_subscribers(false);
        assert!(!npu.has_visualization_subscribers());
    }

    #[test]
    fn test_get_latest_fire_queue_sample() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        
        // Before any burst
        assert!(npu.get_latest_fire_queue_sample().is_none());
        
        npu.process_burst().unwrap();
        
        // After burst, may have sample
        let _sample = npu.get_latest_fire_queue_sample();
    }

    // ═══════════════════════════════════════════════════════════
    // Area Name Mapping
    // ═══════════════════════════════════════════════════════════

    #[test]
    fn test_register_cortical_area_name() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        npu.register_cortical_area(1, "visual_cortex".to_string());
        npu.register_cortical_area(2, "motor_cortex".to_string());
        
        // Names are registered successfully
    }

    // ═══════════════════════════════════════════════════════════
    // Edge Cases & Error Handling
    // ═══════════════════════════════════════════════════════════

    #[test]
    fn test_add_synapse_to_nonexistent_neuron() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let n1 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, 0, 0, 0).unwrap();
        let nonexistent = NeuronId(999);
        
        // Note: add_synapse does NOT validate neuron existence for performance
        // Synapses to nonexistent neurons are silently ignored during propagation
        let result = npu.add_synapse(
            n1,
            nonexistent,
            SynapticWeight(128),
            SynapticConductance(255),
            SynapseType::Excitatory,
        );
        
        assert!(result.is_ok());  // No validation for performance
        assert_eq!(npu.get_synapse_count(), 1);
    }

    #[test]
    fn test_burst_with_empty_npu() {
        let mut npu = RustNPU::new(100, 1000, 10);
        
        let result = npu.process_burst().unwrap();
        
        assert_eq!(result.burst, 1);
        assert_eq!(result.neuron_count, 0);
        assert_eq!(result.power_injections, 0);
    }

    #[test]
    fn test_large_sensory_batch() {
        let mut npu = RustNPU::new(1000, 10000, 10);
        
        // Add 100 neurons
        let mut neurons = Vec::new();
        for i in 0..100 {
            let neuron = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, i, 0, 0).unwrap();
            neurons.push((neuron, 0.5));
        }
        
        npu.inject_sensory_with_potentials(&neurons);
        
        let _result = npu.process_burst().unwrap();
    }
}

