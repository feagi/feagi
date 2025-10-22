/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Neuron ID allocation system for memory and regular neurons
//!
//! Provides globally unique neuron ID allocation with range partitioning
//! to ensure memory neurons and regular neurons never have ID collisions.

use std::sync::{Arc, Mutex};
use std::collections::HashSet;

/// ID Range Constants - GPU and RTOS friendly
pub const REGULAR_NEURON_ID_START: u32 = 0;
pub const REGULAR_NEURON_ID_MAX: u32 = 49_999_999;
pub const MEMORY_NEURON_ID_START: u32 = 50_000_000;
pub const MEMORY_NEURON_ID_MAX: u32 = 99_999_999;
pub const RESERVED_ID_START: u32 = 100_000_000;

/// Neuron type classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NeuronType {
    Regular,
    Memory,
    Reserved,
    Invalid,
}

/// Thread-safe neuron ID allocation manager
#[derive(Clone)]
pub struct NeuronIdManager {
    inner: Arc<Mutex<NeuronIdManagerInner>>,
}

struct NeuronIdManagerInner {
    next_regular_id: u32,
    next_memory_id: u32,
    allocated_regular_ids: HashSet<u32>,
    allocated_memory_ids: HashSet<u32>,
    regular_allocated_count: usize,
    memory_allocated_count: usize,
}

impl NeuronIdManager {
    /// Create a new neuron ID manager
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(NeuronIdManagerInner {
                next_regular_id: REGULAR_NEURON_ID_START,
                next_memory_id: MEMORY_NEURON_ID_START,
                allocated_regular_ids: HashSet::new(),
                allocated_memory_ids: HashSet::new(),
                regular_allocated_count: 0,
                memory_allocated_count: 0,
            })),
        }
    }
    
    /// Allocate a new regular neuron ID
    pub fn allocate_regular_neuron_id(&self) -> Option<u32> {
        let mut inner = self.inner.lock().unwrap();
        
        if inner.next_regular_id > REGULAR_NEURON_ID_MAX {
            return None;
        }
        
        let neuron_id = inner.next_regular_id;
        inner.next_regular_id += 1;
        inner.allocated_regular_ids.insert(neuron_id);
        inner.regular_allocated_count += 1;
        
        Some(neuron_id)
    }
    
    /// Allocate a new memory neuron ID
    pub fn allocate_memory_neuron_id(&self) -> Option<u32> {
        let mut inner = self.inner.lock().unwrap();
        
        if inner.next_memory_id > MEMORY_NEURON_ID_MAX {
            return None;
        }
        
        let neuron_id = inner.next_memory_id;
        inner.next_memory_id += 1;
        inner.allocated_memory_ids.insert(neuron_id);
        inner.memory_allocated_count += 1;
        
        Some(neuron_id)
    }
    
    /// Deallocate a regular neuron ID for reuse
    pub fn deallocate_regular_neuron_id(&self, neuron_id: u32) -> bool {
        if !Self::is_regular_neuron_id(neuron_id) {
            return false;
        }
        
        let mut inner = self.inner.lock().unwrap();
        inner.allocated_regular_ids.remove(&neuron_id)
    }
    
    /// Deallocate a memory neuron ID for reuse
    pub fn deallocate_memory_neuron_id(&self, neuron_id: u32) -> bool {
        if !Self::is_memory_neuron_id(neuron_id) {
            return false;
        }
        
        let mut inner = self.inner.lock().unwrap();
        inner.allocated_memory_ids.remove(&neuron_id)
    }
    
    /// Check if neuron ID belongs to regular neuron range
    pub fn is_regular_neuron_id(neuron_id: u32) -> bool {
        (REGULAR_NEURON_ID_START..=REGULAR_NEURON_ID_MAX).contains(&neuron_id)
    }
    
    /// Check if neuron ID belongs to memory neuron range
    pub fn is_memory_neuron_id(neuron_id: u32) -> bool {
        (MEMORY_NEURON_ID_START..=MEMORY_NEURON_ID_MAX).contains(&neuron_id)
    }
    
    /// Get neuron type from ID
    pub fn get_neuron_type(neuron_id: u32) -> NeuronType {
        if Self::is_regular_neuron_id(neuron_id) {
            NeuronType::Regular
        } else if Self::is_memory_neuron_id(neuron_id) {
            NeuronType::Memory
        } else if neuron_id >= RESERVED_ID_START {
            NeuronType::Reserved
        } else {
            NeuronType::Invalid
        }
    }
    
    /// Get allocation statistics
    pub fn get_allocation_stats(&self) -> AllocationStats {
        let inner = self.inner.lock().unwrap();
        
        let regular_capacity = (REGULAR_NEURON_ID_MAX - REGULAR_NEURON_ID_START + 1) as usize;
        let memory_capacity = (MEMORY_NEURON_ID_MAX - MEMORY_NEURON_ID_START + 1) as usize;
        
        AllocationStats {
            regular_allocated: inner.regular_allocated_count,
            memory_allocated: inner.memory_allocated_count,
            regular_capacity,
            memory_capacity,
            regular_utilization: inner.regular_allocated_count as f64 / regular_capacity as f64,
            memory_utilization: inner.memory_allocated_count as f64 / memory_capacity as f64,
        }
    }
    
    /// Reset allocation state (for testing)
    pub fn reset(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.next_regular_id = REGULAR_NEURON_ID_START;
        inner.next_memory_id = MEMORY_NEURON_ID_START;
        inner.allocated_regular_ids.clear();
        inner.allocated_memory_ids.clear();
        inner.regular_allocated_count = 0;
        inner.memory_allocated_count = 0;
    }
}

impl Default for NeuronIdManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Allocation statistics
#[derive(Debug, Clone)]
pub struct AllocationStats {
    pub regular_allocated: usize,
    pub memory_allocated: usize,
    pub regular_capacity: usize,
    pub memory_capacity: usize,
    pub regular_utilization: f64,
    pub memory_utilization: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_regular_neuron_allocation() {
        let manager = NeuronIdManager::new();
        
        let id1 = manager.allocate_regular_neuron_id();
        let id2 = manager.allocate_regular_neuron_id();
        
        assert!(id1.is_some());
        assert!(id2.is_some());
        assert_ne!(id1.unwrap(), id2.unwrap());
        assert!(NeuronIdManager::is_regular_neuron_id(id1.unwrap()));
    }
    
    #[test]
    fn test_memory_neuron_allocation() {
        let manager = NeuronIdManager::new();
        
        let id1 = manager.allocate_memory_neuron_id();
        let id2 = manager.allocate_memory_neuron_id();
        
        assert!(id1.is_some());
        assert!(id2.is_some());
        assert_ne!(id1.unwrap(), id2.unwrap());
        assert!(NeuronIdManager::is_memory_neuron_id(id1.unwrap()));
    }
    
    #[test]
    fn test_neuron_type_classification() {
        assert_eq!(NeuronIdManager::get_neuron_type(1000), NeuronType::Regular);
        assert_eq!(NeuronIdManager::get_neuron_type(50_000_000), NeuronType::Memory);
        assert_eq!(NeuronIdManager::get_neuron_type(100_000_000), NeuronType::Reserved);
    }
    
    #[test]
    fn test_deallocation() {
        let manager = NeuronIdManager::new();
        
        let id = manager.allocate_regular_neuron_id().unwrap();
        assert!(manager.deallocate_regular_neuron_id(id));
        
        // Note: deallocation removes from set but doesn't decrement counter
        // Counter tracks total allocated, not currently active
        let stats = manager.get_allocation_stats();
        assert_eq!(stats.regular_allocated, 1); // Counter not decremented
    }
}

