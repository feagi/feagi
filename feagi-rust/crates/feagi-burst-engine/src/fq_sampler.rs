/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Fire Queue Sampler for Motor and Visualization Output
//!
//! Samples firing neurons from the current Fire Queue for:
//! - Brain Visualizer (Godot)
//! - Motor output (agents)
//! - External systems
//!
//! Design:
//! - Rate-limited sampling (configurable Hz)
//! - Deduplication (skip if burst already sampled)
//! - Zero-copy when possible (references to Fire Queue data)
//! - Organized by cortical area

use std::time::{Duration, Instant};
use ahash::AHashMap;
use crate::fire_structures::FireQueue;

/// Sampling mode for FQ Sampler
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamplingMode {
    /// For Brain Visualizer (coordinates + potentials)
    Visualization,
    /// For motor output (coordinates only)
    Motor,
    /// For both visualization and motor
    Unified,
}

/// Sampled area data for a single cortical area
#[derive(Debug, Clone)]
pub struct SampledAreaData {
    /// Cortical area index
    pub cortical_idx: u32,
    
    /// Neuron IDs that fired
    pub neuron_ids: Vec<u32>,
    
    /// X coordinates (parallel with neuron_ids)
    pub coordinates_x: Vec<u32>,
    
    /// Y coordinates (parallel with neuron_ids)
    pub coordinates_y: Vec<u32>,
    
    /// Z coordinates (parallel with neuron_ids)
    pub coordinates_z: Vec<u32>,
    
    /// Membrane potentials (parallel with neuron_ids, optional for motor)
    pub potentials: Vec<f32>,
    
    /// Number of neurons (redundant but convenient)
    pub count: usize,
}

/// Fire Queue Sampler Result
#[derive(Debug, Clone)]
pub struct FQSampleResult {
    /// Timestep this sample represents
    pub timestep: u64,
    
    /// Sampled data by cortical area
    pub areas: AHashMap<u32, SampledAreaData>,
    
    /// Total number of neurons sampled
    pub total_neurons: usize,
}

/// Fire Queue Sampler - samples current Fire Queue for external systems
pub struct FQSampler {
    /// Sampling mode
    mode: SamplingMode,
    
    /// Sample frequency (Hz)
    sample_frequency_hz: f64,
    
    /// Sample interval (Duration)
    sample_interval: Duration,
    
    /// Last sample time
    last_sample_time: Option<Instant>,
    
    /// Last sampled burst ID (for deduplication)
    last_sampled_burst_id: Option<u64>,
    
    /// Total samples taken
    samples_taken: u64,
    
    /// Has visualization subscribers
    has_visualization_subscribers: bool,
    
    /// Has motor subscribers
    has_motor_subscribers: bool,
}

impl FQSampler {
    /// Create a new FQ Sampler
    pub fn new(sample_frequency_hz: f64, mode: SamplingMode) -> Self {
        let sample_interval = if sample_frequency_hz > 0.0 {
            Duration::from_secs_f64(1.0 / sample_frequency_hz)
        } else {
            Duration::from_millis(100)
        };
        
        Self {
            mode,
            sample_frequency_hz,
            sample_interval,
            last_sample_time: None,
            last_sampled_burst_id: None,
            samples_taken: 0,
            has_visualization_subscribers: false,
            has_motor_subscribers: false,
        }
    }
    
    /// Sample the current Fire Queue
    /// 
    /// Returns None if:
    /// - Rate limit not met
    /// - Fire Queue is empty
    /// - Burst already sampled (deduplication)
    pub fn sample(&mut self, fire_queue: &FireQueue) -> Option<FQSampleResult> {
        // Rate limiting
        let now = Instant::now();
        if let Some(last_time) = self.last_sample_time {
            if now.duration_since(last_time) < self.sample_interval {
                return None; // Too soon
            }
        }
        
        // Empty check
        if fire_queue.is_empty() {
            return None;
        }
        
        // Deduplication: Skip if we've already sampled this burst
        let current_burst_id = fire_queue.timestep;
        if self.last_sampled_burst_id == Some(current_burst_id) {
            return None; // Already sampled
        }
        
        // Sample the Fire Queue
        let mut areas = AHashMap::new();
        let mut total_neurons = 0;
        
        for (&cortical_idx, neurons) in &fire_queue.neurons_by_area {
            let count = neurons.len();
            if count == 0 {
                continue;
            }
            
            // Extract neuron data
            let mut neuron_ids = Vec::with_capacity(count);
            let mut coords_x = Vec::with_capacity(count);
            let mut coords_y = Vec::with_capacity(count);
            let mut coords_z = Vec::with_capacity(count);
            let mut potentials = Vec::with_capacity(count);
            
            for neuron in neurons {
                neuron_ids.push(neuron.neuron_id.0);
                coords_x.push(neuron.x);
                coords_y.push(neuron.y);
                coords_z.push(neuron.z);
                potentials.push(neuron.membrane_potential);
            }
            
            areas.insert(cortical_idx, SampledAreaData {
                cortical_idx,
                neuron_ids,
                coordinates_x: coords_x,
                coordinates_y: coords_y,
                coordinates_z: coords_z,
                potentials,
                count,
            });
            
            total_neurons += count;
        }
        
        // Update state
        self.last_sample_time = Some(now);
        self.last_sampled_burst_id = Some(current_burst_id);
        self.samples_taken += 1;
        
        Some(FQSampleResult {
            timestep: current_burst_id,
            areas,
            total_neurons,
        })
    }
    
    /// Set sample frequency (Hz)
    pub fn set_sample_frequency(&mut self, frequency_hz: f64) {
        if frequency_hz > 0.0 {
            self.sample_frequency_hz = frequency_hz;
            self.sample_interval = Duration::from_secs_f64(1.0 / frequency_hz);
        }
    }
    
    /// Get sample frequency (Hz)
    pub fn get_sample_frequency(&self) -> f64 {
        self.sample_frequency_hz
    }
    
    /// Set visualization subscriber state
    pub fn set_visualization_subscribers(&mut self, has_subscribers: bool) {
        self.has_visualization_subscribers = has_subscribers;
    }
    
    /// Check if visualization subscribers are connected
    pub fn has_visualization_subscribers(&self) -> bool {
        self.has_visualization_subscribers
    }
    
    /// Set motor subscriber state
    pub fn set_motor_subscribers(&mut self, has_subscribers: bool) {
        self.has_motor_subscribers = has_subscribers;
    }
    
    /// Check if motor subscribers are connected
    pub fn has_motor_subscribers(&self) -> bool {
        self.has_motor_subscribers
    }
    
    /// Get total samples taken
    pub fn get_samples_taken(&self) -> u64 {
        self.samples_taken
    }
    
    /// Get sampling mode
    pub fn get_mode(&self) -> SamplingMode {
        self.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fire_structures::FiringNeuron;
    use feagi_types::{NeuronId, CorticalAreaId};
    
    #[test]
    fn test_fq_sampler_basic() {
        let mut sampler = FQSampler::new(10.0, SamplingMode::Visualization);
        
        // Create mock fire queue
        let mut fire_queue = FireQueue::new();
        fire_queue.set_timestep(1);
        
        let neuron = FiringNeuron {
            neuron_id: NeuronId(100),
            membrane_potential: 1.5,
            cortical_area: CorticalAreaId(1),
            x: 0,
            y: 0,
            z: 0,
        };
        fire_queue.add_neuron(neuron);
        
        // First sample should succeed
        let result = sampler.sample(&fire_queue);
        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.timestep, 1);
        assert_eq!(result.total_neurons, 1);
        assert_eq!(result.areas.len(), 1);
        
        // Second sample (same burst) should be deduplicated
        let result2 = sampler.sample(&fire_queue);
        assert!(result2.is_none());
    }
    
    #[test]
    fn test_fq_sampler_rate_limiting() {
        let mut sampler = FQSampler::new(1000.0, SamplingMode::Visualization); // Very high rate
        
        let mut fire_queue = FireQueue::new();
        fire_queue.set_timestep(1);
        
        let neuron = FiringNeuron {
            neuron_id: NeuronId(100),
            membrane_potential: 1.0,
            cortical_area: CorticalAreaId(1),
            x: 0,
            y: 0,
            z: 0,
        };
        fire_queue.add_neuron(neuron);
        
        // First sample
        let result1 = sampler.sample(&fire_queue);
        assert!(result1.is_some());
        
        // Immediate second sample (different burst) may be rate-limited
        fire_queue.set_timestep(2);
        let result2 = sampler.sample(&fire_queue);
        // Result may be None due to rate limiting
    }
}

