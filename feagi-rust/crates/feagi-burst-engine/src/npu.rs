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
    
    /// Add a neuron to the NPU
    pub fn add_neuron(
        &mut self,
        threshold: f32,
        leak_rate: f32,
        refractory_period: u16,
        excitability: f32,
        cortical_area: u32,
        x: u32,
        y: u32,
        z: u32,
    ) -> Result<NeuronId> {
        self.neuron_array.add_neuron(
            threshold,
            leak_rate,
            refractory_period,
            excitability,
            cortical_area,
            x,
            y,
            z,
        )
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
    
    /// Remove a synapse
    pub fn remove_synapse(&mut self, source: NeuronId, target: NeuronId) -> bool {
        self.synapse_array.remove_synapse(source, target)
    }
    
    /// Update synapse weight
    pub fn update_synapse_weight(&mut self, source: NeuronId, target: NeuronId, new_weight: SynapticWeight) -> bool {
        self.synapse_array.update_weight(source, target, new_weight)
    }
    
    /// Rebuild indexes after modifications (call after bulk modifications)
    pub fn rebuild_indexes(&mut self) {
        // Convert synapse array to Synapse vector for indexing
        let synapses = self.get_valid_synapses();
        self.propagation_engine.build_synapse_index(&synapses);
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
        
        // Convert synapse array to slice for propagation
        let synapses = self.get_valid_synapses();
        
        // Phase 1: Injection (power + synaptic propagation)
        let injection_result = phase1_injection_with_synapses(
            &mut self.fire_candidate_list,
            &self.neuron_array,
            &mut self.propagation_engine,
            &self.previous_fire_queue,
            power_neurons,
            self.power_amount,
            &synapses,
        )?;
        
        // Phase 2: Neural Dynamics (membrane potential updates, threshold checks, firing)
        let dynamics_result = process_neural_dynamics(
            &self.fire_candidate_list,
            &mut self.neuron_array,
        )?;
        
        // Phase 3: Archival (record to Fire Ledger)
        let neuron_ids = dynamics_result.fire_queue.get_all_neuron_ids();
        self.fire_ledger.record_burst(self.burst_count, neuron_ids);
        
        // Phase 5: Cleanup (clear FCL for next burst)
        self.fire_candidate_list.clear();
        
        // Swap fire queues: current becomes previous for next burst
        eprintln!("🦀 [RUST-NPU-DEBUG] BEFORE swap: burst={}, current_fq={} neurons, previous_fq={} neurons", 
                  self.burst_count,
                  self.current_fire_queue.get_all_neuron_ids().len(),
                  self.previous_fire_queue.get_all_neuron_ids().len());
        
        self.previous_fire_queue = self.current_fire_queue.clone();
        self.current_fire_queue = dynamics_result.fire_queue.clone();
        
        eprintln!("🦀 [RUST-NPU-DEBUG] AFTER swap: burst={}, current_fq={} neurons, previous_fq={} neurons", 
                  self.burst_count,
                  self.current_fire_queue.get_all_neuron_ids().len(),
                  self.previous_fire_queue.get_all_neuron_ids().len());
        
        if !self.current_fire_queue.is_empty() {
            let fired = self.current_fire_queue.get_all_neuron_ids();
            if fired.len() <= 5 {
                eprintln!("🦀 [RUST-NPU-DEBUG] Neurons that fired this burst: {:?}", fired);
            }
        }
        
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
    
    // Helper: Convert synapse array to vector of valid synapses
    fn get_valid_synapses(&self) -> Vec<Synapse> {
        let mut synapses = Vec::with_capacity(self.synapse_array.count);
        
        for i in 0..self.synapse_array.count {
            if self.synapse_array.valid_mask[i] {
                synapses.push(Synapse {
                    source_neuron: NeuronId(self.synapse_array.source_neurons[i]),
                    target_neuron: NeuronId(self.synapse_array.target_neurons[i]),
                    weight: SynapticWeight(self.synapse_array.weights[i]),
                    conductance: SynapticConductance(self.synapse_array.conductances[i]),
                    synapse_type: match self.synapse_array.types[i] {
                        0 => SynapseType::Excitatory,
                        _ => SynapseType::Inhibitory,
                    },
                    valid: true,
                });
            }
        }
        
        synapses
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
    synapses: &[Synapse],
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
        eprintln!("🦀 [RUST-NPU-DEBUG] Synaptic propagation: {} previously fired neurons", fired_ids.len());
        if !fired_ids.is_empty() && fired_ids.len() <= 5 {
            eprintln!("🦀 [RUST-NPU-DEBUG] Previously fired: {:?}", fired_ids);
        }
        
        // Call synaptic propagation engine with synapses
        let propagation_result = propagation_engine.propagate(&fired_ids, synapses)?;
        eprintln!("🦀 [RUST-NPU-DEBUG] Propagation engine returned {} source neurons with targets", propagation_result.len());
        
        // Inject propagated potentials into FCL
        for (source_id, targets) in propagation_result {
            eprintln!("🦀 [RUST-NPU-DEBUG] Source neuron {} → {} targets", source_id.0, targets.len());
            for &(target_neuron_id, contribution) in &targets {
                fcl.add_candidate(target_neuron_id, contribution.0);  // Extract f32 from SynapticContribution
                synaptic_count += 1;
                if synaptic_count <= 5 {
                    eprintln!("🦀 [RUST-NPU-DEBUG] Injecting: neuron {} += {}", target_neuron_id.0, contribution.0);
                }
            }
        }
        eprintln!("🦀 [RUST-NPU-DEBUG] Total synaptic injections: {}", synaptic_count);
    } else {
        eprintln!("🦀 [RUST-NPU-DEBUG] Skipping synaptic propagation - previous_fire_queue is empty");
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
        
        let id1 = npu.add_neuron(1.0, 0.1, 5, 1.0, 1, 0, 0, 0).unwrap();
        let id2 = npu.add_neuron(1.0, 0.1, 5, 1.0, 1, 1, 0, 0).unwrap();
        
        assert_eq!(id1.0, 0);
        assert_eq!(id2.0, 1);
        assert_eq!(npu.get_neuron_count(), 2);
    }

    #[test]
    fn test_add_synapses() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        let n1 = npu.add_neuron(1.0, 0.1, 5, 1.0, 1, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.1, 5, 1.0, 1, 1, 0, 0).unwrap();
        
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
        let power_neuron = npu.add_neuron(1.0, 0.1, 5, 1.0, 1, 0, 0, 0).unwrap();
        
        // Process burst with power injection
        let result = npu.process_burst(&[power_neuron]).unwrap();
        
        assert_eq!(result.burst, 1);
        assert_eq!(result.power_injections, 1);
        assert_eq!(result.neuron_count, 1);  // Power neuron should fire
    }

    #[test]
    fn test_synapse_removal() {
        let mut npu = RustNPU::new(1000, 10000, 20);
        
        let n1 = npu.add_neuron(1.0, 0.1, 5, 1.0, 1, 0, 0, 0).unwrap();
        let n2 = npu.add_neuron(1.0, 0.1, 5, 1.0, 1, 1, 0, 0).unwrap();
        
        npu.add_synapse(n1, n2, SynapticWeight(128), SynapticConductance(255), SynapseType::Excitatory).unwrap();
        assert_eq!(npu.get_synapse_count(), 1);
        
        assert!(npu.remove_synapse(n1, n2));
        assert_eq!(npu.get_synapse_count(), 0);
    }
}
