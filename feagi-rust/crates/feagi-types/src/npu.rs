/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # NPU Core Data Structures
//!
//! Complete neuron and synapse arrays with SIMD-optimized operations.
//!
//! ## Design Philosophy
//! - **Structure-of-Arrays (SoA)**: Better cache locality and SIMD
//! - **Pre-allocated**: Fixed-size arrays for RTOS compatibility
//! - **Zero-copy**: Use slices and references where possible
//! - **Type-safe**: Strong typing for all neural properties

use crate::*;
use std::collections::HashMap;

/// Complete neuron array with all properties
/// 
/// Uses Structure-of-Arrays for SIMD optimization
#[derive(Debug, Clone)]
pub struct NeuronArray {
    /// Number of neurons allocated
    pub capacity: usize,
    
    /// Number of neurons actually used
    pub count: usize,
    
    /// Membrane potentials (mV or arbitrary units)
    pub membrane_potentials: Vec<f32>,
    
    /// Firing thresholds
    pub thresholds: Vec<f32>,
    
    /// Leak/decay rates (0.0 to 1.0)
    pub leak_rates: Vec<f32>,
    
    /// Refractory periods (burst counts)
    pub refractory_periods: Vec<u16>,
    
    /// Current refractory countdown
    pub refractory_countdowns: Vec<u16>,
    
    /// Neuron excitability (0.0 to 1.0 for probabilistic firing)
    pub excitabilities: Vec<f32>,
    
    /// Cortical area ID for each neuron
    pub cortical_areas: Vec<u32>,
    
    /// 3D coordinates (x, y, z) - flat array of [x0, y0, z0, x1, y1, z1, ...]
    pub coordinates: Vec<u32>,
}

impl NeuronArray {
    /// Create a new neuron array with specified capacity
    pub fn new(capacity: usize) -> Self {
        Self {
            capacity,
            count: 0,
            membrane_potentials: vec![0.0; capacity],
            thresholds: vec![1.0; capacity],
            leak_rates: vec![0.0; capacity],
            refractory_periods: vec![0; capacity],
            refractory_countdowns: vec![0; capacity],
            excitabilities: vec![1.0; capacity],
            cortical_areas: vec![0; capacity],
            coordinates: vec![0; capacity * 3],
        }
    }
    
    /// Add a neuron (returns neuron ID = index)
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
        if self.count >= self.capacity {
            return Err(FeagiError::MemoryAllocationError(
                "Neuron array capacity exceeded".to_string()
            ));
        }
        
        let id = self.count;
        self.thresholds[id] = threshold;
        self.leak_rates[id] = leak_rate;
        self.refractory_periods[id] = refractory_period;
        self.excitabilities[id] = excitability.clamp(0.0, 1.0);
        self.cortical_areas[id] = cortical_area;
        self.coordinates[id * 3] = x;
        self.coordinates[id * 3 + 1] = y;
        self.coordinates[id * 3 + 2] = z;
        
        self.count += 1;
        Ok(NeuronId(id as u32))
    }
    
    /// Get neuron threshold
    #[inline(always)]
    pub fn get_threshold(&self, neuron_id: NeuronId) -> f32 {
        self.thresholds[neuron_id.0 as usize]
    }
    
    /// Get neuron membrane potential
    #[inline(always)]
    pub fn get_potential(&self, neuron_id: NeuronId) -> f32 {
        self.membrane_potentials[neuron_id.0 as usize]
    }
    
    /// Set neuron membrane potential
    #[inline(always)]
    pub fn set_potential(&mut self, neuron_id: NeuronId, potential: f32) {
        self.membrane_potentials[neuron_id.0 as usize] = potential;
    }
    
    /// Accumulate to neuron membrane potential
    #[inline(always)]
    pub fn accumulate_potential(&mut self, neuron_id: NeuronId, delta: f32) {
        self.membrane_potentials[neuron_id.0 as usize] += delta;
    }
    
    /// Get neuron cortical area
    #[inline(always)]
    pub fn get_cortical_area(&self, neuron_id: NeuronId) -> CorticalAreaId {
        CorticalAreaId(self.cortical_areas[neuron_id.0 as usize])
    }
    
    /// Get neuron coordinates
    #[inline(always)]
    pub fn get_coordinates(&self, neuron_id: NeuronId) -> (u32, u32, u32) {
        let idx = neuron_id.0 as usize * 3;
        (self.coordinates[idx], self.coordinates[idx + 1], self.coordinates[idx + 2])
    }
}

/// Complete synapse array with dynamic operations
///
/// Uses Structure-of-Arrays for SIMD optimization
#[derive(Debug, Clone)]
pub struct SynapseArray {
    /// Number of synapses allocated
    pub capacity: usize,
    
    /// Number of synapses actually used
    pub count: usize,
    
    /// Source neuron IDs
    pub source_neurons: Vec<u32>,
    
    /// Target neuron IDs
    pub target_neurons: Vec<u32>,
    
    /// Synaptic weights (0-255)
    pub weights: Vec<u8>,
    
    /// Synaptic conductances (0-255)
    pub conductances: Vec<u8>,
    
    /// Synapse types (0=excitatory, 1=inhibitory)
    pub types: Vec<u8>,
    
    /// Valid mask (for soft deletion)
    pub valid_mask: Vec<bool>,
    
    /// Source neuron index: neuron_id -> [synapse_indices]
    pub source_index: HashMap<u32, Vec<usize>>,
}

impl SynapseArray {
    /// Create a new synapse array with specified capacity
    pub fn new(capacity: usize) -> Self {
        Self {
            capacity,
            count: 0,
            source_neurons: vec![0; capacity],
            target_neurons: vec![0; capacity],
            weights: vec![0; capacity],
            conductances: vec![255; capacity],
            types: vec![0; capacity],
            valid_mask: vec![false; capacity],
            source_index: HashMap::new(),
        }
    }
    
    /// Add a synapse (returns synapse index)
    pub fn add_synapse(
        &mut self,
        source: NeuronId,
        target: NeuronId,
        weight: SynapticWeight,
        conductance: SynapticConductance,
        synapse_type: SynapseType,
    ) -> Result<usize> {
        if self.count >= self.capacity {
            return Err(FeagiError::MemoryAllocationError(
                "Synapse array capacity exceeded".to_string()
            ));
        }
        
        let idx = self.count;
        self.source_neurons[idx] = source.0;
        self.target_neurons[idx] = target.0;
        self.weights[idx] = weight.0;
        self.conductances[idx] = conductance.0;
        self.types[idx] = match synapse_type {
            SynapseType::Excitatory => 0,
            SynapseType::Inhibitory => 1,
        };
        self.valid_mask[idx] = true;
        
        // Update source index
        self.source_index
            .entry(source.0)
            .or_insert_with(Vec::new)
            .push(idx);
        
        self.count += 1;
        Ok(idx)
    }
    
    /// Remove a synapse (soft delete - just marks as invalid)
    pub fn remove_synapse(&mut self, source: NeuronId, target: NeuronId) -> bool {
        if let Some(indices) = self.source_index.get(&source.0) {
            for &idx in indices {
                if self.valid_mask[idx] 
                    && self.source_neurons[idx] == source.0 
                    && self.target_neurons[idx] == target.0 
                {
                    self.valid_mask[idx] = false;
                    return true;
                }
            }
        }
        false
    }
    
    /// Update synapse weight
    pub fn update_weight(&mut self, source: NeuronId, target: NeuronId, new_weight: SynapticWeight) -> bool {
        if let Some(indices) = self.source_index.get(&source.0) {
            for &idx in indices {
                if self.valid_mask[idx] 
                    && self.source_neurons[idx] == source.0 
                    && self.target_neurons[idx] == target.0 
                {
                    self.weights[idx] = new_weight.0;
                    return true;
                }
            }
        }
        false
    }
    
    /// Get number of valid synapses
    pub fn valid_count(&self) -> usize {
        self.valid_mask.iter().filter(|&&v| v).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_neuron_array_creation() {
        let neurons = NeuronArray::new(100);
        assert_eq!(neurons.capacity, 100);
        assert_eq!(neurons.count, 0);
    }

    #[test]
    fn test_add_neuron() {
        let mut neurons = NeuronArray::new(100);
        let id = neurons.add_neuron(
            1.0,   // threshold
            0.1,   // leak_rate
            5,     // refractory_period
            1.0,   // excitability
            1,     // cortical_area
            10, 5, 3  // x, y, z
        ).unwrap();
        
        assert_eq!(id.0, 0);
        assert_eq!(neurons.count, 1);
        assert_eq!(neurons.get_threshold(id), 1.0);
        assert_eq!(neurons.get_coordinates(id), (10, 5, 3));
    }

    #[test]
    fn test_synapse_array() {
        let mut synapses = SynapseArray::new(1000);
        
        let idx = synapses.add_synapse(
            NeuronId(1),
            NeuronId(2),
            SynapticWeight(128),
            SynapticConductance(255),
            SynapseType::Excitatory,
        ).unwrap();
        
        assert_eq!(idx, 0);
        assert_eq!(synapses.count, 1);
        assert_eq!(synapses.valid_count(), 1);
        
        // Test removal
        assert!(synapses.remove_synapse(NeuronId(1), NeuronId(2)));
        assert_eq!(synapses.valid_count(), 0);
    }
}
