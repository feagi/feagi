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

//! # Synaptic Propagation Engine
//!
//! This module implements the core bottleneck identified in Python profiling:
//! computing synaptic contributions from fired neurons to their targets.
//!
//! ## Python Bottleneck Analysis
//! ```text
//! Phase 1 (Injection):  163.84 ms ( 88.7%)
//!   └─ Synaptic Propagation: 161.07 ms (100% of Phase 1)
//!      └─ Numpy Processing:  164.67 ms ( 91.7%)
//! ```
//!
//! ## Rust Optimization Strategy
//! 1. **Gather Phase**: Build synapse list (minimal Python loop overhead)
//! 2. **SIMD Phase**: Vectorized math (weight × conductance × sign)
//! 3. **Grouping Phase**: Sort/split by cortical area (np.argsort overhead removed)
//!
//! ## Performance Target
//! - Python: ~165ms for 12K neurons
//! - Rust Target: <3ms (50-100x speedup)

use feagi_types::*;
use rayon::prelude::*;
use ahash::AHashMap;  // Faster non-cryptographic hash

/// Synapse lookup index: maps source neuron → list of synapse indices
pub type SynapseIndex = AHashMap<NeuronId, Vec<usize>>;

/// Propagation result: cortical area → list of (target_neuron, contribution)
pub type PropagationResult = AHashMap<CorticalAreaId, Vec<(NeuronId, SynapticContribution)>>;

/// High-performance synaptic propagation engine
pub struct SynapticPropagationEngine {
    /// Pre-built index: source neuron → synapse indices
    pub synapse_index: SynapseIndex,
    /// Neuron → Cortical Area mapping
    pub neuron_to_area: AHashMap<NeuronId, CorticalAreaId>,
    /// Performance stats
    total_propagations: u64,
    total_synapses_processed: u64,
}

impl SynapticPropagationEngine {
    /// Create a new propagation engine
    pub fn new() -> Self {
        Self {
            synapse_index: AHashMap::new(),
            neuron_to_area: AHashMap::new(),
            total_propagations: 0,
            total_synapses_processed: 0,
        }
    }

    /// Build the synapse index from a synapse array (Structure-of-Arrays)
    /// This should be called once during initialization or when connectome changes
    /// 
    /// ZERO-COPY: Works directly with SynapseArray without allocating intermediate structures
    pub fn build_synapse_index(&mut self, synapse_array: &SynapseArray) {
        self.synapse_index.clear();
        
        for i in 0..synapse_array.count {
            if synapse_array.valid_mask[i] {
                let source = NeuronId(synapse_array.source_neurons[i]);
                self.synapse_index
                    .entry(source)
                    .or_insert_with(Vec::new)
                    .push(i);
            }
        }
    }

    /// Set the neuron-to-cortical-area mapping
    pub fn set_neuron_mapping(&mut self, mapping: AHashMap<NeuronId, CorticalAreaId>) {
        self.neuron_to_area = mapping;
    }

    /// Compute synaptic propagation for a set of fired neurons
    /// 
    /// This is the MAIN PERFORMANCE-CRITICAL function that replaces the Python bottleneck.
    /// 
    /// # Performance Notes
    /// - Uses Rayon for parallel processing
    /// - SIMD-friendly vectorized calculations
    /// - ZERO-COPY: Works directly with SynapseArray (no allocation overhead)
    /// - Cache-friendly data access patterns
    pub fn propagate(
        &mut self,
        fired_neurons: &[NeuronId],
        synapse_array: &SynapseArray,
    ) -> Result<PropagationResult> {
        self.total_propagations += 1;

        if fired_neurons.is_empty() {
            return Ok(AHashMap::new());
        }

        // PHASE 1: GATHER - Collect all synapse indices for fired neurons (parallel)
        let synapse_indices: Vec<usize> = fired_neurons
            .par_iter()
            .filter_map(|&neuron_id| self.synapse_index.get(&neuron_id))
            .flatten()
            .copied()
            .collect();

        if synapse_indices.is_empty() {
            return Ok(AHashMap::new());
        }

        let total_synapses = synapse_indices.len();
        self.total_synapses_processed += total_synapses as u64;

        // PHASE 2: COMPUTE - Calculate contributions in parallel (TRUE SIMD!)
        // This is where Python spent 165ms doing inefficient numpy ops
        // ZERO-COPY: Access SynapseArray fields directly (Structure-of-Arrays)
        let contributions: Vec<(NeuronId, CorticalAreaId, SynapticContribution)> = synapse_indices
            .par_iter()
            .filter_map(|&syn_idx| {
                // Skip invalid synapses (already filtered by build_synapse_index, but double-check)
                if !synapse_array.valid_mask[syn_idx] {
                    return None;
                }

                // Get target neuron from SoA
                let target_neuron = NeuronId(synapse_array.target_neurons[syn_idx]);
                
                // Get target cortical area
                let cortical_area = *self.neuron_to_area.get(&target_neuron)?;

                // Calculate contribution directly from SoA fields (SIMD-friendly)
                let weight = SynapticWeight(synapse_array.weights[syn_idx]);
                let conductance = SynapticConductance(synapse_array.conductances[syn_idx]);
                let synapse_type = match synapse_array.types[syn_idx] {
                    0 => SynapseType::Excitatory,
                    _ => SynapseType::Inhibitory,
                };
                
                // Calculate: weight × conductance × sign
                let sign = if synapse_type == SynapseType::Excitatory { 1.0 } else { -1.0 };
                let contribution = SynapticContribution(weight.to_float() * conductance.to_float() * sign);

                Some((target_neuron, cortical_area, contribution))
            })
            .collect();

        // PHASE 3: GROUP - Group by cortical area (sequential, but very fast)
        // This replaces Python's slow dictionary building
        let mut result: PropagationResult = AHashMap::new();
        for (target_neuron, cortical_area, contribution) in contributions {
            result
                .entry(cortical_area)
                .or_insert_with(Vec::new)
                .push((target_neuron, contribution));
        }

        Ok(result)
    }

    /// Get performance statistics
    pub fn stats(&self) -> (u64, u64) {
        (self.total_propagations, self.total_synapses_processed)
    }

    /// Reset performance statistics
    pub fn reset_stats(&mut self) {
        self.total_propagations = 0;
        self.total_synapses_processed = 0;
    }
}

impl Default for SynapticPropagationEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn create_test_synapses() -> SynapseArray {
        let mut synapse_array = SynapseArray {
            capacity: 10,
            count: 3,
            source_neurons: vec![1, 1, 2],  // Raw u32 values
            target_neurons: vec![10, 11, 10],  // Raw u32 values
            weights: vec![255, 128, 200],  // Raw u8 values
            conductances: vec![255, 255, 200],  // Raw u8 values
            types: vec![0, 1, 0],  // 0=excitatory, 1=inhibitory
            valid_mask: vec![true, true, true],
            source_index: HashMap::new(),
        };
        
        // Build source index
        for i in 0..synapse_array.count {
            let source = synapse_array.source_neurons[i];
            synapse_array.source_index.entry(source).or_insert_with(Vec::new).push(i);
        }
        
        synapse_array
    }

    #[test]
    fn test_synaptic_propagation() {
        let synapses = create_test_synapses();
        let mut engine = SynapticPropagationEngine::new();

        // Build index
        engine.build_synapse_index(&synapses);

        // Set neuron mapping
        let mut mapping = AHashMap::new();
        mapping.insert(NeuronId(10), CorticalAreaId(1));
        mapping.insert(NeuronId(11), CorticalAreaId(1));
        engine.set_neuron_mapping(mapping);

        // Propagate from neuron 1
        let fired = vec![NeuronId(1)];
        let result = engine.propagate(&fired, &synapses).unwrap();

        // Should have 2 contributions in area 1
        assert_eq!(result.len(), 1);
        let area1_contributions = result.get(&CorticalAreaId(1)).unwrap();
        assert_eq!(area1_contributions.len(), 2);

        // Check that both targets are present
        let targets: Vec<_> = area1_contributions.iter().map(|(n, _)| *n).collect();
        assert!(targets.contains(&NeuronId(10)));
        assert!(targets.contains(&NeuronId(11)));
    }

    #[test]
    fn test_parallel_propagation() {
        let synapses = create_test_synapses();
        let mut engine = SynapticPropagationEngine::new();
        engine.build_synapse_index(&synapses);

        let mut mapping = AHashMap::new();
        mapping.insert(NeuronId(10), CorticalAreaId(1));
        mapping.insert(NeuronId(11), CorticalAreaId(1));
        engine.set_neuron_mapping(mapping);

        // Propagate from multiple neurons in parallel
        let fired = vec![NeuronId(1), NeuronId(2)];
        let result = engine.propagate(&fired, &synapses).unwrap();

        let area1_contributions = result.get(&CorticalAreaId(1)).unwrap();
        assert_eq!(area1_contributions.len(), 3);  // 2 from neuron 1, 1 from neuron 2
    }
}
