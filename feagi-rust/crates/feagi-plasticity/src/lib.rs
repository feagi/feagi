/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # FEAGI Plasticity Module
//!
//! This crate implements synaptic plasticity algorithms for FEAGI:
//! - STDP (Spike-Timing-Dependent Plasticity)
//! - Memory formation with pattern detection
//! - Memory neuron lifecycle management
//! - Neuron ID allocation system
//!
//! ## Architecture
//! - High-performance Rust implementation
//! - SIMD-friendly data structures
//! - Thread-safe operations
//! - RTOS-compatible design

pub mod stdp;
pub mod pattern_detector;
pub mod memory_neuron_array;
pub mod neuron_id_manager;
pub mod service;

// Re-export key types
pub use stdp::{STDPConfig, compute_activity_factors, compute_timing_factors};
pub use pattern_detector::{PatternConfig, PatternDetector, BatchPatternDetector, TemporalPattern};
pub use memory_neuron_array::{MemoryNeuronArray, MemoryNeuronLifecycleConfig, MemoryNeuronStats};
pub use neuron_id_manager::{NeuronIdManager, NeuronType, AllocationStats};
pub use service::{PlasticityService, PlasticityConfig, PlasticityCommand};




