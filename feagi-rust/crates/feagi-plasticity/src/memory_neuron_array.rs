/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Memory Neuron Array - Structure of Arrays for memory neurons with lifecycle management
//!
//! High-performance SoA implementation optimized for:
//! - SIMD-friendly vectorized operations
//! - Rust/RTOS compatibility
//! - Thread-safe operations
//! - Efficient memory management with index reuse

use std::collections::{HashMap, HashSet};
use crate::neuron_id_manager::NeuronIdManager;

/// Memory neuron lifecycle configuration
#[derive(Debug, Clone, Copy)]
pub struct MemoryNeuronLifecycleConfig {
    /// Initial lifespan in bursts
    pub initial_lifespan: u32,
    
    /// Lifespan growth per reactivation
    pub lifespan_growth_rate: f32,
    
    /// Lifespan threshold for long-term memory conversion
    pub longterm_threshold: u32,
    
    /// Maximum reactivations before forced LTM
    pub max_reactivations: u32,
}

impl Default for MemoryNeuronLifecycleConfig {
    fn default() -> Self {
        Self {
            initial_lifespan: 20,
            lifespan_growth_rate: 3.0,
            longterm_threshold: 100,
            max_reactivations: 1000,
        }
    }
}

/// Memory neuron array statistics
#[derive(Debug, Clone, Default)]
pub struct MemoryNeuronStats {
    pub total_capacity: usize,
    pub active_neurons: usize,
    pub longterm_neurons: usize,
    pub dead_neurons: usize,
    pub reusable_indices: usize,
    pub memory_usage_bytes: usize,
    pub avg_lifespan: f64,
    pub avg_activation_count: f64,
}

/// High-performance Structure of Arrays for memory neurons
pub struct MemoryNeuronArray {
    capacity: usize,
    
    // Core neuron properties (SoA layout)
    neuron_ids: Vec<u32>,
    cortical_area_ids: Vec<u32>,
    is_active: Vec<bool>,
    
    // Lifecycle management
    lifespan_current: Vec<u32>,
    lifespan_initial: Vec<u32>,
    lifespan_growth_rate: Vec<f32>,
    is_longterm_memory: Vec<bool>,
    
    // Temporal tracking
    creation_burst: Vec<u64>,
    last_activation_burst: Vec<u64>,
    activation_count: Vec<u32>,
    
    // Pattern association
    pattern_hash_to_index: HashMap<[u8; 32], usize>,
    index_to_pattern_hash: HashMap<usize, [u8; 32]>,
    
    // Index management
    next_available_index: usize,
    reusable_indices: HashSet<usize>,
    
    // Area-specific tracking
    area_neuron_indices: HashMap<u32, HashSet<usize>>,
    
    // Neuron ID manager
    id_manager: NeuronIdManager,
}

impl MemoryNeuronArray {
    /// Create a new memory neuron array
    pub fn new(capacity: usize) -> Self {
        Self {
            capacity,
            neuron_ids: vec![0; capacity],
            cortical_area_ids: vec![0; capacity],
            is_active: vec![false; capacity],
            lifespan_current: vec![0; capacity],
            lifespan_initial: vec![0; capacity],
            lifespan_growth_rate: vec![0.0; capacity],
            is_longterm_memory: vec![false; capacity],
            creation_burst: vec![0; capacity],
            last_activation_burst: vec![0; capacity],
            activation_count: vec![0; capacity],
            pattern_hash_to_index: HashMap::new(),
            index_to_pattern_hash: HashMap::new(),
            next_available_index: 0,
            reusable_indices: HashSet::new(),
            area_neuron_indices: HashMap::new(),
            id_manager: NeuronIdManager::new(),
        }
    }
    
    /// Create a new memory neuron
    pub fn create_memory_neuron(
        &mut self,
        pattern_hash: [u8; 32],
        cortical_area_id: u32,
        current_burst: u64,
        config: &MemoryNeuronLifecycleConfig,
    ) -> Option<usize> {
        // Check if pattern already exists
        if let Some(&existing_idx) = self.pattern_hash_to_index.get(&pattern_hash) {
            if self.is_active[existing_idx] {
                // Reactivate existing neuron instead
                return self.reactivate_memory_neuron_internal(existing_idx, current_burst);
            }
        }
        
        // Get neuron index (reuse or allocate new)
        let neuron_idx = self.get_available_index_internal()?;
        
        // Allocate global neuron ID
        let neuron_id = self.id_manager.allocate_memory_neuron_id()?;
        
        // Initialize neuron properties
        self.neuron_ids[neuron_idx] = neuron_id;
        self.cortical_area_ids[neuron_idx] = cortical_area_id;
        self.is_active[neuron_idx] = true;
        
        // Initialize lifecycle
        self.lifespan_current[neuron_idx] = config.initial_lifespan;
        self.lifespan_initial[neuron_idx] = config.initial_lifespan;
        self.lifespan_growth_rate[neuron_idx] = config.lifespan_growth_rate;
        self.is_longterm_memory[neuron_idx] = false;
        
        // Initialize temporal tracking
        self.creation_burst[neuron_idx] = current_burst;
        self.last_activation_burst[neuron_idx] = current_burst;
        self.activation_count[neuron_idx] = 1;
        
        // Register pattern association
        self.pattern_hash_to_index.insert(pattern_hash, neuron_idx);
        self.index_to_pattern_hash.insert(neuron_idx, pattern_hash);
        
        // Add to area tracking
        self.area_neuron_indices
            .entry(cortical_area_id)
            .or_insert_with(HashSet::new)
            .insert(neuron_idx);
        
        Some(neuron_idx)
    }
    
    /// Reactivate an existing memory neuron
    pub fn reactivate_memory_neuron(
        &mut self,
        neuron_idx: usize,
        current_burst: u64,
    ) -> bool {
        self.reactivate_memory_neuron_internal(neuron_idx, current_burst).is_some()
    }
    
    /// Internal reactivate that returns Option<usize> for compatibility
    fn reactivate_memory_neuron_internal(
        &mut self,
        neuron_idx: usize,
        current_burst: u64,
    ) -> Option<usize> {
        if !self.is_valid_index(neuron_idx) || !self.is_active[neuron_idx] {
            return None;
        }
        
        // Update activation tracking
        self.last_activation_burst[neuron_idx] = current_burst;
        self.activation_count[neuron_idx] += 1;
        
        // Grow lifespan if not long-term memory
        if !self.is_longterm_memory[neuron_idx] {
            let current_lifespan = self.lifespan_current[neuron_idx];
            let growth = self.lifespan_growth_rate[neuron_idx] as u32;
            self.lifespan_current[neuron_idx] = current_lifespan.saturating_add(growth);
        }
        
        Some(neuron_idx)
    }
    
    /// Age all active memory neurons (vectorized operation)
    pub fn age_memory_neurons(&mut self, _current_burst: u64) -> Vec<usize> {
        let n = self.next_available_index;
        if n == 0 {
            return Vec::new();
        }
        
        let mut died_indices = Vec::new();
        
        // Age eligible neurons
        for i in 0..n {
            if self.is_active[i] && !self.is_longterm_memory[i] && self.lifespan_current[i] > 0 {
                self.lifespan_current[i] -= 1;
                
                // Check if neuron died
                if self.lifespan_current[i] == 0 {
                    self.is_active[i] = false;
                    died_indices.push(i);
                }
            }
        }
        
        // Clean up dead neurons after iteration
        for &i in &died_indices {
            self.cleanup_dead_neuron_internal(i);
        }
        
        died_indices
    }
    
    /// Check for neurons ready for long-term memory conversion
    pub fn check_longterm_conversion(&mut self, longterm_threshold: u32) -> Vec<usize> {
        
        let n = self.next_available_index;
        if n == 0 {
            return Vec::new();
        }
        
        let mut converted_indices = Vec::new();
        
        for i in 0..n {
            if self.is_active[i] 
                && !self.is_longterm_memory[i] 
                && self.lifespan_current[i] >= longterm_threshold 
            {
                self.is_longterm_memory[i] = true;
                converted_indices.push(i);
            }
        }
        
        converted_indices
    }
    
    /// Get all active neuron IDs for a cortical area
    pub fn get_active_neurons_by_area(&self, cortical_area_id: u32) -> Vec<u32> {
        
        if let Some(indices) = self.area_neuron_indices.get(&cortical_area_id) {
            indices.iter()
                .filter(|&&idx| self.is_valid_index(idx) && self.is_active[idx])
                .map(|&idx| self.neuron_ids[idx])
                .collect()
        } else {
            Vec::new()
        }
    }
    
    /// Find neuron index by pattern hash
    pub fn find_neuron_by_pattern(&self, pattern_hash: &[u8; 32]) -> Option<usize> {
        
        self.pattern_hash_to_index.get(pattern_hash)
            .copied()
            .filter(|&idx| self.is_valid_index(idx) && self.is_active[idx])
    }
    
    /// Get neuron ID at index
    pub fn get_neuron_id(&self, neuron_idx: usize) -> Option<u32> {
        if self.is_valid_index(neuron_idx) {
            Some(self.neuron_ids[neuron_idx])
        } else {
            None
        }
    }
    
    /// Get comprehensive statistics
    pub fn get_stats(&self) -> MemoryNeuronStats {
        
        let n = self.next_available_index;
        
        if n == 0 {
            return MemoryNeuronStats {
                total_capacity: self.capacity,
                ..Default::default()
            };
        }
        
        let active_count = self.is_active[..n].iter().filter(|&&x| x).count();
        let longterm_count = (0..n)
            .filter(|&i| self.is_active[i] && self.is_longterm_memory[i])
            .count();
        let dead_count = n - active_count;
        
        // Calculate averages for active neurons
        let (avg_lifespan, avg_activation_count) = if active_count > 0 {
            let total_lifespan: u32 = (0..n)
                .filter(|&i| self.is_active[i])
                .map(|i| self.lifespan_current[i])
                .sum();
            let total_activations: u32 = (0..n)
                .filter(|&i| self.is_active[i])
                .map(|i| self.activation_count[i])
                .sum();
            
            (
                total_lifespan as f64 / active_count as f64,
                total_activations as f64 / active_count as f64,
            )
        } else {
            (0.0, 0.0)
        };
        
        // Estimate memory usage
        let memory_usage = self.capacity * (
            std::mem::size_of::<u32>() * 4 +  // uint32 arrays
            std::mem::size_of::<f32>() +       // float32 array
            std::mem::size_of::<u64>() * 2 +   // uint64 arrays
            std::mem::size_of::<bool>() * 2    // bool arrays
        ) + self.pattern_hash_to_index.len() * (32 + 8)  // Pattern hash mappings
          + self.area_neuron_indices.len() * 64;         // Area tracking overhead
        
        MemoryNeuronStats {
            total_capacity: self.capacity,
            active_neurons: active_count,
            longterm_neurons: longterm_count,
            dead_neurons: dead_count,
            reusable_indices: self.reusable_indices.len(),
            memory_usage_bytes: memory_usage,
            avg_lifespan,
            avg_activation_count,
        }
    }
    
    /// Get available index (reuse or allocate new)
    fn get_available_index_internal(&mut self) -> Option<usize> {
        // Try to reuse a dead neuron index first
        if let Some(&idx) = self.reusable_indices.iter().next() {
            self.reusable_indices.remove(&idx);
            return Some(idx);
        }
        
        // Allocate new index if capacity allows
        if self.next_available_index < self.capacity {
            let idx = self.next_available_index;
            self.next_available_index += 1;
            Some(idx)
        } else {
            None
        }
    }
    
    /// Clean up associations for a dead neuron
    fn cleanup_dead_neuron_internal(&mut self, neuron_idx: usize) {
        // Deallocate global neuron ID
        let neuron_id = self.neuron_ids[neuron_idx];
        self.id_manager.deallocate_memory_neuron_id(neuron_id);
        
        // Remove pattern association
        if let Some(pattern_hash) = self.index_to_pattern_hash.remove(&neuron_idx) {
            self.pattern_hash_to_index.remove(&pattern_hash);
        }
        
        // Remove from area tracking
        let area_id = self.cortical_area_ids[neuron_idx];
        if let Some(indices) = self.area_neuron_indices.get_mut(&area_id) {
            indices.remove(&neuron_idx);
        }
        
        // Add to reusable indices
        self.reusable_indices.insert(neuron_idx);
    }
    
    /// Check if neuron index is valid
    fn is_valid_index(&self, neuron_idx: usize) -> bool {
        neuron_idx < self.next_available_index
    }
    
    /// Reset array state (for testing)
    pub fn reset(&mut self) {
        
        self.neuron_ids.fill(0);
        self.cortical_area_ids.fill(0);
        self.is_active.fill(false);
        self.lifespan_current.fill(0);
        self.lifespan_initial.fill(0);
        self.lifespan_growth_rate.fill(0.0);
        self.is_longterm_memory.fill(false);
        self.creation_burst.fill(0);
        self.last_activation_burst.fill(0);
        self.activation_count.fill(0);
        
        self.pattern_hash_to_index.clear();
        self.index_to_pattern_hash.clear();
        self.area_neuron_indices.clear();
        
        self.next_available_index = 0;
        self.reusable_indices.clear();
        
        self.id_manager.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_create_memory_neuron() {
        let mut array = MemoryNeuronArray::new(1000);
        let config = MemoryNeuronLifecycleConfig::default();
        
        let pattern_hash = [1u8; 32];
        let neuron_idx = array.create_memory_neuron(pattern_hash, 100, 0, &config);
        
        assert!(neuron_idx.is_some());
        let idx = neuron_idx.unwrap();
        assert!(array.is_active[idx]);
        assert_eq!(array.cortical_area_ids[idx], 100);
    }
    
    #[test]
    fn test_reactivate_memory_neuron() {
        let mut array = MemoryNeuronArray::new(1000);
        let config = MemoryNeuronLifecycleConfig::default();
        
        let pattern_hash = [1u8; 32];
        let idx = array.create_memory_neuron(pattern_hash, 100, 0, &config).unwrap();
        
        let initial_count = array.activation_count[idx];
        assert!(array.reactivate_memory_neuron(idx, 1));
        assert_eq!(array.activation_count[idx], initial_count + 1);
    }
    
    #[test]
    fn test_age_memory_neurons() {
        let mut array = MemoryNeuronArray::new(1000);
        let mut config = MemoryNeuronLifecycleConfig::default();
        config.initial_lifespan = 2;
        
        let pattern_hash = [1u8; 32];
        let idx = array.create_memory_neuron(pattern_hash, 100, 0, &config).unwrap();
        
        // Age once
        let died = array.age_memory_neurons(1);
        assert!(died.is_empty());
        assert_eq!(array.lifespan_current[idx], 1);
        
        // Age again - should die
        let died = array.age_memory_neurons(2);
        assert_eq!(died.len(), 1);
        assert!(!array.is_active[idx]);
    }
    
    #[test]
    fn test_longterm_conversion() {
        let mut array = MemoryNeuronArray::new(1000);
        let mut config = MemoryNeuronLifecycleConfig::default();
        config.initial_lifespan = 100;
        
        let pattern_hash = [1u8; 32];
        let idx = array.create_memory_neuron(pattern_hash, 100, 0, &config).unwrap();
        
        let converted = array.check_longterm_conversion(100);
        assert_eq!(converted.len(), 1);
        assert!(array.is_longterm_memory[idx]);
    }
}

