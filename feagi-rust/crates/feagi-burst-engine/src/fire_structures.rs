/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Fire Queue data structures for NPU processing
//!
//! These structures represent neurons that fired in the current burst.
//! Used for synaptic propagation and archiving to Fire Ledger.

use ahash::AHashMap;
use feagi_types::{NeuronId, CorticalAreaId};

/// A single neuron that fired in the current burst
#[derive(Debug, Clone)]
pub struct FiringNeuron {
    pub neuron_id: NeuronId,
    pub membrane_potential: f32,
    pub cortical_area: CorticalAreaId,
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

/// Fire Queue - neurons that fired in the current burst
/// Organized by cortical area for efficient processing
#[derive(Debug, Clone)]
pub struct FireQueue {
    /// Firing neurons grouped by cortical area
    pub neurons_by_area: AHashMap<u32, Vec<FiringNeuron>>,
    
    /// Total number of neurons across all areas
    total_count: usize,
    
    /// Timestep this queue represents
    pub timestep: u64,
}

impl FireQueue {
    /// Create a new empty Fire Queue
    pub fn new() -> Self {
        Self {
            neurons_by_area: AHashMap::new(),
            total_count: 0,
            timestep: 0,
        }
    }
    
    /// Add a firing neuron to the queue
    pub fn add_neuron(&mut self, neuron: FiringNeuron) {
        let cortical_idx = neuron.cortical_area.0;
        self.neurons_by_area
            .entry(cortical_idx)
            .or_insert_with(Vec::new)
            .push(neuron);
        self.total_count += 1;
    }
    
    /// Get total number of fired neurons across all areas
    pub fn total_neurons(&self) -> usize {
        self.total_count
    }
    
    /// Get neurons for a specific cortical area
    pub fn get_area_neurons(&self, cortical_idx: u32) -> Option<&Vec<FiringNeuron>> {
        self.neurons_by_area.get(&cortical_idx)
    }
    
    /// Clear the queue
    pub fn clear(&mut self) {
        self.neurons_by_area.clear();
        self.total_count = 0;
    }
    
    /// Set timestep
    pub fn set_timestep(&mut self, timestep: u64) {
        self.timestep = timestep;
    }
    
    /// Get all neuron IDs from all areas
    pub fn get_all_neuron_ids(&self) -> Vec<NeuronId> {
        let mut ids = Vec::with_capacity(self.total_count);
        for neurons in self.neurons_by_area.values() {
            ids.extend(neurons.iter().map(|n| n.neuron_id));
        }
        ids
    }
    
    /// Check if fire queue is empty
    pub fn is_empty(&self) -> bool {
        self.total_count == 0
    }
}

impl Default for FireQueue {
    fn default() -> Self {
        Self::new()
    }
}

