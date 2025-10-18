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

//! # FEAGI Burst Engine
//!
//! High-performance neural burst processing engine.
//!
//! ## Performance Targets
//! - **30Hz burst frequency** with 1.2M neuron firings
//! - **50-100x faster** than Python implementation
//! - **SIMD-optimized** for modern CPUs
//! - **Cache-friendly** data structures
//!
//! ## Architecture
//! - Pure Rust, no Python overhead
//! - Rayon for multi-threading
//! - Zero-copy data access where possible
//! - Minimal allocations in hot paths

pub mod backend;
pub mod synaptic_propagation;
pub mod neural_dynamics;
pub mod npu;
pub mod fire_structures;
pub mod fire_ledger;
pub mod fq_sampler;
pub mod sensory;  // Rust sensory injection system
pub mod burst_loop_runner;  // Pure Rust burst loop
pub mod viz_shm_writer;  // Rust visualization SHM writer

pub use backend::*;
pub use synaptic_propagation::*;
pub use neural_dynamics::*;
pub use npu::*;
pub use fire_structures::*;
pub use fire_ledger::*;
pub use fq_sampler::*;
pub use sensory::*;
pub use viz_shm_writer::*;
pub use burst_loop_runner::*;

/// Burst engine performance statistics
#[derive(Debug, Clone, Default)]
pub struct BurstEngineStats {
    pub total_bursts: u64,
    pub total_neurons_fired: u64,
    pub total_synapses_processed: u64,
    pub total_processing_time_us: u64,
}

impl BurstEngineStats {
    /// Get average neurons per burst
    pub fn avg_neurons_per_burst(&self) -> f64 {
        if self.total_bursts == 0 {
            0.0
        } else {
            self.total_neurons_fired as f64 / self.total_bursts as f64
        }
    }

    /// Get average processing time per burst (microseconds)
    pub fn avg_processing_time_us(&self) -> f64 {
        if self.total_bursts == 0 {
            0.0
        } else {
            self.total_processing_time_us as f64 / self.total_bursts as f64
        }
    }

    /// Get average synapses per neuron
    pub fn avg_synapses_per_neuron(&self) -> f64 {
        if self.total_neurons_fired == 0 {
            0.0
        } else {
            self.total_synapses_processed as f64 / self.total_neurons_fired as f64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_burst_stats() {
        let mut stats = BurstEngineStats::default();
        stats.total_bursts = 100;
        stats.total_neurons_fired = 10000;
        stats.total_synapses_processed = 50000;
        stats.total_processing_time_us = 1000000;

        assert_eq!(stats.avg_neurons_per_burst(), 100.0);
        assert_eq!(stats.avg_processing_time_us(), 10000.0);
        assert_eq!(stats.avg_synapses_per_neuron(), 5.0);
    }
}
