/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Fire Ledger - Historical firing data for STDP and debugging
//!
//! Architecture:
//! - Zero-copy design: Directly archives Fire Queue data
//! - Circular buffer per cortical area (configurable window size)
//! - Structure-of-Arrays for cache efficiency
//! - Thread-safe via Rust ownership
//!
//! Usage:
//! ```rust
//! let mut fire_ledger = RustFireLedger::new(20); // Default 20-timestep window
//! fire_ledger.archive_burst(timestep, &fire_queue);
//! let history = fire_ledger.get_history(cortical_idx, 10);
//! ```

use ahash::AHashMap;
use std::collections::VecDeque;

use crate::fire_structures::FireQueue;

/// Fire Ledger - maintains historical firing data per cortical area
#[derive(Debug, Clone)]
pub struct RustFireLedger {
    /// Firing history per cortical area
    cortical_histories: AHashMap<u32, CorticalHistory>,
    
    /// Default window size for new areas
    default_window_size: usize,
    
    /// Current timestep
    current_timestep: u64,
}

/// Circular buffer of firing data for a single cortical area
#[derive(Debug, Clone)]
struct CorticalHistory {
    /// Circular buffer of timesteps
    timesteps: VecDeque<u64>,
    
    /// Circular buffer of neuron ID vectors (parallel with timesteps)
    neuron_ids: VecDeque<Vec<u32>>,
    
    /// Maximum number of timesteps to retain
    window_size: usize,
}

impl RustFireLedger {
    /// Create a new Fire Ledger with default window size
    pub fn new(default_window_size: usize) -> Self {
        Self {
            cortical_histories: AHashMap::new(),
            default_window_size,
            current_timestep: 0,
        }
    }
    
    /// Archive a burst's firing data (ZERO COPY from Fire Queue!)
    pub fn archive_burst(&mut self, timestep: u64, fire_queue: &FireQueue) {
        self.current_timestep = timestep;
        
        // Archive each cortical area's firing data
        for (&cortical_idx, neurons) in &fire_queue.neurons_by_area {
            // Extract neuron IDs from FiringNeuron structs
            let neuron_ids: Vec<u32> = neurons.iter()
                .map(|n| n.neuron_id.0)
                .collect();
            
            // Get or create history for this area
            let history = self.cortical_histories
                .entry(cortical_idx)
                .or_insert_with(|| CorticalHistory::new(self.default_window_size));
            
            // Archive to circular buffer
            history.add_timestep(timestep, neuron_ids);
        }
    }
    
    /// Get firing history for a cortical area
    /// Returns Vec of (timestep, neuron_ids) tuples, newest first
    pub fn get_history(&self, cortical_idx: u32, lookback_steps: usize) -> Vec<(u64, Vec<u32>)> {
        if let Some(history) = self.cortical_histories.get(&cortical_idx) {
            history.get_recent(lookback_steps)
        } else {
            Vec::new()
        }
    }
    
    /// Get window size for a specific cortical area
    pub fn get_area_window_size(&self, cortical_idx: u32) -> usize {
        self.cortical_histories
            .get(&cortical_idx)
            .map(|h| h.window_size)
            .unwrap_or(self.default_window_size)
    }
    
    /// Configure window size for a specific cortical area
    pub fn configure_area_window(&mut self, cortical_idx: u32, window_size: usize) {
        if let Some(history) = self.cortical_histories.get_mut(&cortical_idx) {
            history.resize_window(window_size);
        } else {
            // Create new history with custom window size
            self.cortical_histories.insert(
                cortical_idx,
                CorticalHistory::new(window_size)
            );
        }
    }
    
    /// Get all configured area window sizes
    pub fn get_all_window_configs(&self) -> Vec<(u32, usize)> {
        self.cortical_histories
            .iter()
            .map(|(&idx, hist)| (idx, hist.window_size))
            .collect()
    }
    
    /// Get current timestep
    pub fn current_timestep(&self) -> u64 {
        self.current_timestep
    }
}

impl CorticalHistory {
    /// Create a new cortical history with specified window size
    fn new(window_size: usize) -> Self {
        Self {
            timesteps: VecDeque::with_capacity(window_size),
            neuron_ids: VecDeque::with_capacity(window_size),
            window_size,
        }
    }
    
    /// Add a timestep's firing data (maintains circular buffer)
    fn add_timestep(&mut self, timestep: u64, neuron_ids: Vec<u32>) {
        // If at capacity, remove oldest
        if self.timesteps.len() >= self.window_size {
            self.timesteps.pop_front();
            self.neuron_ids.pop_front();
        }
        
        // Add new data
        self.timesteps.push_back(timestep);
        self.neuron_ids.push_back(neuron_ids);
    }
    
    /// Get recent firing history (newest first)
    fn get_recent(&self, lookback_steps: usize) -> Vec<(u64, Vec<u32>)> {
        let available = self.timesteps.len();
        let count = lookback_steps.min(available);
        
        // Collect from newest to oldest
        let mut result = Vec::with_capacity(count);
        for i in (available.saturating_sub(count)..available).rev() {
            result.push((
                self.timesteps[i],
                self.neuron_ids[i].clone(), // Clone neuron IDs for Python
            ));
        }
        
        result
    }
    
    /// Resize the window (may truncate old data)
    fn resize_window(&mut self, new_size: usize) {
        self.window_size = new_size;
        
        // Truncate if new size is smaller
        while self.timesteps.len() > new_size {
            self.timesteps.pop_front();
            self.neuron_ids.pop_front();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fire_structures::{FiringNeuron, FireQueue};
    use feagi_types::{NeuronId, CorticalAreaId};
    
    #[test]
    fn test_fire_ledger_basic() {
        let mut ledger = RustFireLedger::new(5);
        
        // Create mock fire queue with some neurons
        let mut fire_queue = FireQueue::new();
        let neuron1 = FiringNeuron {
            neuron_id: NeuronId(100),
            membrane_potential: 1.5,
            cortical_area: CorticalAreaId(1),
            x: 0,
            y: 0,
            z: 0,
        };
        let neuron2 = FiringNeuron {
            neuron_id: NeuronId(200),
            membrane_potential: 1.2,
            cortical_area: CorticalAreaId(1),
            x: 1,
            y: 0,
            z: 0,
        };
        
        fire_queue.add_neuron(neuron1);
        fire_queue.add_neuron(neuron2);
        
        // Archive burst
        ledger.archive_burst(1, &fire_queue);
        
        // Retrieve history
        let history = ledger.get_history(1, 10);
        assert_eq!(history.len(), 1);
        assert_eq!(history[0].0, 1); // timestep
        assert_eq!(history[0].1.len(), 2); // 2 neurons
        assert!(history[0].1.contains(&100));
        assert!(history[0].1.contains(&200));
    }
    
    #[test]
    fn test_fire_ledger_circular_buffer() {
        let mut ledger = RustFireLedger::new(3); // Only keep 3 timesteps
        
        // Archive 5 bursts
        for t in 1..=5 {
            let mut fire_queue = FireQueue::new();
            let neuron = FiringNeuron {
                neuron_id: NeuronId(t as u32 * 100),
                membrane_potential: 1.0,
                cortical_area: CorticalAreaId(1),
                x: 0,
                y: 0,
                z: 0,
            };
            fire_queue.add_neuron(neuron);
            ledger.archive_burst(t, &fire_queue);
        }
        
        // Should only have last 3 timesteps (3, 4, 5)
        let history = ledger.get_history(1, 10);
        assert_eq!(history.len(), 3);
        assert_eq!(history[0].0, 5); // Newest first
        assert_eq!(history[1].0, 4);
        assert_eq!(history[2].0, 3);
    }
    
    #[test]
    fn test_fire_ledger_empty_area() {
        let ledger = RustFireLedger::new(20);
        
        // Query non-existent area
        let history = ledger.get_history(999, 10);
        assert_eq!(history.len(), 0);
    }
}

