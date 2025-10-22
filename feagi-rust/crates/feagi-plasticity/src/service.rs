/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Plasticity Service - orchestrates STDP and memory formation
//!
//! RTOS-friendly design:
//! - No sleeps/timeouts; uses condition variables
//! - Read-only access to firing history
//! - Mutations are enqueued as commands

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, Condvar};
use std::thread;

use crate::stdp::STDPConfig;
use crate::pattern_detector::{PatternConfig, BatchPatternDetector};
use crate::memory_neuron_array::{MemoryNeuronArray, MemoryNeuronLifecycleConfig};

/// Plasticity configuration
#[derive(Debug, Clone)]
pub struct PlasticityConfig {
    /// Queue capacity for commands
    pub queue_capacity: usize,
    
    /// Maximum operations per burst
    pub max_ops_per_burst: usize,
    
    /// STDP configuration
    pub stdp: Option<STDPConfig>,
    
    /// Pattern detection configuration  
    pub pattern_config: PatternConfig,
    
    /// Memory neuron lifecycle configuration
    pub memory_lifecycle_config: MemoryNeuronLifecycleConfig,
}

impl Default for PlasticityConfig {
    fn default() -> Self {
        Self {
            queue_capacity: 1000,
            max_ops_per_burst: 100,
            stdp: Some(STDPConfig::default()),
            pattern_config: PatternConfig::default(),
            memory_lifecycle_config: MemoryNeuronLifecycleConfig::default(),
        }
    }
}

/// Plasticity command types
#[derive(Debug, Clone)]
pub enum PlasticityCommand {
    /// Update synaptic weights with deltas
    UpdateWeightsDelta {
        synapse_indices: Vec<usize>,
        deltas: Vec<f32>,
    },
    
    /// Register memory neuron in regular neuron array
    RegisterMemoryNeuron {
        neuron_id: u32,
        area_idx: u32,
        threshold: f32,
        membrane_potential: f32,
    },
    
    /// Inject memory neuron to Fire Candidate List
    InjectMemoryNeuronToFCL {
        neuron_id: u32,
        area_idx: u32,
        membrane_potential: f32,
        pattern_hash: [u8; 32],
        is_reactivation: bool,
    },
    
    /// Update state counters
    UpdateStateCounters {
        memory_neurons_created: usize,
        current_memory_neuron_count: usize,
        area_idx: u32,
        neuron_id: u32,
    },
}

/// Memory area configuration
#[derive(Debug, Clone)]
pub struct MemoryAreaConfig {
    pub temporal_depth: u32,
    pub upstream_areas: Vec<u32>,
}

/// Plasticity service statistics
#[derive(Debug, Clone, Default)]
pub struct PlasticityStats {
    pub memory_patterns_detected: usize,
    pub memory_neurons_created: usize,
    pub memory_neurons_reactivated: usize,
    pub memory_neurons_aged: usize,
    pub memory_neurons_converted_ltm: usize,
    pub plasticity_commands_enqueued: usize,
    pub plasticity_commands_dropped: usize,
}

/// Plasticity service - independent thread that computes plasticity every burst
pub struct PlasticityService {
    config: PlasticityConfig,
    
    // Pattern detection
    pattern_detector: BatchPatternDetector,
    
    // Memory neuron array
    memory_neuron_array: Arc<Mutex<MemoryNeuronArray>>,
    
    // Memory area tracking
    memory_areas: Arc<Mutex<HashMap<u32, MemoryAreaConfig>>>,
    memory_lifecycle_configs: Arc<Mutex<HashMap<u32, MemoryNeuronLifecycleConfig>>>,
    
    // Thread synchronization
    cv: Arc<(Mutex<(bool, u64)>, Condvar)>,  // (running, latest_timestep)
    
    // Command queue
    command_queue: Arc<Mutex<Vec<PlasticityCommand>>>,
    
    // Statistics
    stats: Arc<Mutex<PlasticityStats>>,
}

impl PlasticityService {
    /// Create a new plasticity service
    pub fn new(config: PlasticityConfig) -> Self {
        let pattern_detector = BatchPatternDetector::new(config.pattern_config.clone());
        
        Self {
            config,
            pattern_detector,
            memory_neuron_array: Arc::new(Mutex::new(MemoryNeuronArray::new(50000))),
            memory_areas: Arc::new(Mutex::new(HashMap::new())),
            memory_lifecycle_configs: Arc::new(Mutex::new(HashMap::new())),
            cv: Arc::new((Mutex::new((false, 0)), Condvar::new())),
            command_queue: Arc::new(Mutex::new(Vec::new())),
            stats: Arc::new(Mutex::new(PlasticityStats::default())),
        }
    }
    
    /// Notify service of new burst
    pub fn notify_burst(&self, timestep: u64) {
        let (lock, cvar) = &*self.cv;
        let mut data = lock.lock().unwrap();
        data.1 = timestep;
        cvar.notify_all();
    }
    
    /// Start the plasticity service thread
    pub fn start(&self) -> thread::JoinHandle<()> {
        let cv = Arc::clone(&self.cv);
        let command_queue = Arc::clone(&self.command_queue);
        let memory_neuron_array = Arc::clone(&self.memory_neuron_array);
        let memory_areas = Arc::clone(&self.memory_areas);
        let memory_lifecycle_configs = Arc::clone(&self.memory_lifecycle_configs);
        let pattern_detector = self.pattern_detector.clone();
        let stats = Arc::clone(&self.stats);
        let config = self.config.clone();
        
        thread::spawn(move || {
            let (lock, cvar) = &*cv;
            
            loop {
                let timestep = {
                    let mut data = lock.lock().unwrap();
                    while !data.0 {
                        data = cvar.wait(data).unwrap();
                    }
                    data.1
                };
                
                // Compute plasticity
                Self::compute_plasticity(
                    timestep,
                    &memory_neuron_array,
                    &memory_areas,
                    &memory_lifecycle_configs,
                    &pattern_detector,
                    &command_queue,
                    &stats,
                    &config,
                );
            }
        })
    }
    
    /// Stop the plasticity service
    pub fn stop(&self) {
        let (lock, cvar) = &*self.cv;
        let mut data = lock.lock().unwrap();
        data.0 = false;
        cvar.notify_all();
    }
    
    /// Compute plasticity for current burst
    fn compute_plasticity(
        current_timestep: u64,
        memory_neuron_array: &Arc<Mutex<MemoryNeuronArray>>,
        memory_areas: &Arc<Mutex<HashMap<u32, MemoryAreaConfig>>>,
        memory_lifecycle_configs: &Arc<Mutex<HashMap<u32, MemoryNeuronLifecycleConfig>>>,
        pattern_detector: &BatchPatternDetector,
        command_queue: &Arc<Mutex<Vec<PlasticityCommand>>>,
        stats: &Arc<Mutex<PlasticityStats>>,
        config: &PlasticityConfig,
    ) {
        let memory_areas_snapshot = memory_areas.lock().unwrap().clone();
        
        if memory_areas_snapshot.is_empty() {
            return;
        }
        
        let mut commands = Vec::new();
        let mut array = memory_neuron_array.lock().unwrap();
        
        // Step 1: Age all memory neurons
        let died_neurons = array.age_memory_neurons(current_timestep);
        if !died_neurons.is_empty() {
            let mut s = stats.lock().unwrap();
            s.memory_neurons_aged += died_neurons.len();
        }
        
        // Step 2: Check for long-term memory conversion
        let converted_neurons = array.check_longterm_conversion(config.memory_lifecycle_config.longterm_threshold);
        if !converted_neurons.is_empty() {
            let mut s = stats.lock().unwrap();
            s.memory_neurons_converted_ltm += converted_neurons.len();
        }
        
        // Step 3: Detect patterns for all memory areas
        // Note: In a real implementation, we would get firing history from the fire ledger
        // For now, this is a placeholder showing the structure
        
        for (memory_area_idx, area_config) in memory_areas_snapshot.iter() {
            // TODO: Get actual firing history from fire ledger
            // For now, create empty pattern as placeholder
            let timestep_bitmaps: Vec<HashSet<u32>> = Vec::new();
            
            let detector = pattern_detector.get_detector(*memory_area_idx, area_config.temporal_depth);
            
            if let Some(pattern) = detector.detect_pattern(
                *memory_area_idx,
                &area_config.upstream_areas,
                current_timestep,
                timestep_bitmaps,
                Some(area_config.temporal_depth),
            ) {
                let mut s = stats.lock().unwrap();
                s.memory_patterns_detected += 1;
                drop(s);
                
                // Check if pattern already has a memory neuron
                if let Some(existing_neuron_idx) = array.find_neuron_by_pattern(&pattern.pattern_hash) {
                    // Reactivate existing neuron
                    if array.reactivate_memory_neuron(existing_neuron_idx, current_timestep) {
                        let mut s = stats.lock().unwrap();
                        s.memory_neurons_reactivated += 1;
                        drop(s);
                        
                        let neuron_id = array.get_neuron_id(existing_neuron_idx).unwrap();
                        
                        // Register and inject reactivated neuron
                        commands.push(PlasticityCommand::RegisterMemoryNeuron {
                            neuron_id,
                            area_idx: *memory_area_idx,
                            threshold: 1.5,
                            membrane_potential: 0.0,
                        });
                        
                        commands.push(PlasticityCommand::InjectMemoryNeuronToFCL {
                            neuron_id,
                            area_idx: *memory_area_idx,
                            membrane_potential: 1.5,
                            pattern_hash: pattern.pattern_hash,
                            is_reactivation: true,
                        });
                        
                        let total_memory = array.get_stats().active_neurons;
                        commands.push(PlasticityCommand::UpdateStateCounters {
                            memory_neurons_created: 0,
                            current_memory_neuron_count: total_memory,
                            area_idx: *memory_area_idx,
                            neuron_id,
                        });
                    }
                } else {
                    // Create new memory neuron
                    let lifecycle_config = memory_lifecycle_configs.lock().unwrap()
                        .get(memory_area_idx)
                        .copied()
                        .unwrap_or_default();
                    
                    if let Some(neuron_idx) = array.create_memory_neuron(
                        pattern.pattern_hash,
                        *memory_area_idx,
                        current_timestep,
                        &lifecycle_config,
                    ) {
                        let mut s = stats.lock().unwrap();
                        s.memory_neurons_created += 1;
                        drop(s);
                        
                        let neuron_id = array.get_neuron_id(neuron_idx).unwrap();
                        
                        // Register and inject new neuron
                        commands.push(PlasticityCommand::RegisterMemoryNeuron {
                            neuron_id,
                            area_idx: *memory_area_idx,
                            threshold: 1.0,
                            membrane_potential: 0.0,
                        });
                        
                        commands.push(PlasticityCommand::InjectMemoryNeuronToFCL {
                            neuron_id,
                            area_idx: *memory_area_idx,
                            membrane_potential: 1.5,
                            pattern_hash: pattern.pattern_hash,
                            is_reactivation: false,
                        });
                        
                        commands.push(PlasticityCommand::UpdateStateCounters {
                            memory_neurons_created: 1,
                            current_memory_neuron_count: array.get_stats().active_neurons,
                            area_idx: *memory_area_idx,
                            neuron_id,
                        });
                    }
                }
            }
        }
        
        // Enqueue commands
        if !commands.is_empty() {
            let cmd_count = commands.len();
            let mut queue = command_queue.lock().unwrap();
            let mut s = stats.lock().unwrap();
            
            if queue.len() + cmd_count <= config.queue_capacity {
                queue.extend(commands);
                s.plasticity_commands_enqueued += cmd_count;
            } else {
                s.plasticity_commands_dropped += cmd_count;
            }
        }
    }
    
    /// Register a memory area for pattern detection
    pub fn register_memory_area(
        &self,
        area_idx: u32,
        temporal_depth: u32,
        upstream_areas: Vec<u32>,
        lifecycle_config: Option<MemoryNeuronLifecycleConfig>,
    ) -> bool {
        let mut areas = self.memory_areas.lock().unwrap();
        areas.insert(area_idx, MemoryAreaConfig {
            temporal_depth,
            upstream_areas,
        });
        
        if let Some(config) = lifecycle_config {
            let mut configs = self.memory_lifecycle_configs.lock().unwrap();
            configs.insert(area_idx, config);
        }
        
        true
    }
    
    /// Dequeue plasticity commands
    pub fn dequeue_commands(&self, max_count: usize) -> Vec<PlasticityCommand> {
        let mut queue = self.command_queue.lock().unwrap();
        let count = queue.len().min(max_count);
        queue.drain(..count).collect()
    }
    
    /// Get statistics
    pub fn get_stats(&self) -> PlasticityStats {
        self.stats.lock().unwrap().clone()
    }
    
    /// Get memory neuron array reference
    pub fn get_memory_neuron_array(&self) -> Arc<Mutex<MemoryNeuronArray>> {
        Arc::clone(&self.memory_neuron_array)
    }
}

// BatchPatternDetector Clone is implemented in pattern_detector.rs

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_plasticity_service_creation() {
        let config = PlasticityConfig::default();
        let service = PlasticityService::new(config);
        
        let stats = service.get_stats();
        assert_eq!(stats.memory_neurons_created, 0);
    }
    
    #[test]
    fn test_register_memory_area() {
        let config = PlasticityConfig::default();
        let service = PlasticityService::new(config);
        
        let result = service.register_memory_area(100, 3, vec![1, 2], None);
        assert!(result);
        
        let areas = service.memory_areas.lock().unwrap();
        assert!(areas.contains_key(&100));
    }
}

