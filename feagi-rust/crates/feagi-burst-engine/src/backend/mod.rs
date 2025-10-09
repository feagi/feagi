/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Compute Backend Abstraction
//!
//! Provides a unified interface for different compute backends (CPU, GPU).
//! This allows the burst engine to use optimal hardware acceleration without
//! changing the high-level burst processing logic.

mod cpu;
#[cfg(feature = "gpu")]
mod wgpu_backend;

pub use cpu::CPUBackend;
#[cfg(feature = "gpu")]
pub use wgpu_backend::WGPUBackend;

use feagi_types::*;

/// Result of processing a burst on any backend
#[derive(Debug, Clone)]
pub struct BackendBurstResult {
    /// Neurons that fired this burst
    pub fired_neurons: Vec<u32>,
    
    /// Performance metrics
    pub neurons_processed: usize,
    pub neurons_fired: usize,
    pub neurons_in_refractory: usize,
    
    /// Timing information (microseconds)
    pub timing: BurstTiming,
}

/// Detailed timing breakdown for burst processing
#[derive(Debug, Clone, Default)]
pub struct BurstTiming {
    /// Time spent on synaptic propagation (μs)
    pub synaptic_propagation_us: f64,
    
    /// Time spent on neural dynamics (μs)
    pub neural_dynamics_us: f64,
    
    /// Time spent on data transfer (GPU only, μs)
    pub transfer_us: f64,
    
    /// Total burst time (μs)
    pub total_us: f64,
}

/// Compute backend trait - abstracts CPU vs GPU execution
pub trait ComputeBackend: Send + Sync {
    /// Get backend type name for logging/debugging
    fn backend_name(&self) -> &str;
    
    /// Process synaptic propagation: fired neurons → membrane potential updates
    /// 
    /// # Arguments
    /// * `fired_neurons` - Neurons that fired in previous burst
    /// * `synapse_array` - All synapses (SoA structure)
    /// * `neuron_array` - Target neuron array (membrane potentials updated)
    /// 
    /// # Returns
    /// Number of synapses processed
    fn process_synaptic_propagation(
        &mut self,
        fired_neurons: &[u32],
        synapse_array: &SynapseArray,
        neuron_array: &mut NeuronArray,
    ) -> Result<usize>;
    
    /// Process neural dynamics: membrane potentials → firing decisions
    /// 
    /// # Arguments
    /// * `neuron_array` - Neuron array with updated membrane potentials
    /// * `burst_count` - Current burst number (for excitability randomness)
    /// 
    /// # Returns
    /// List of neurons that fired + statistics
    fn process_neural_dynamics(
        &mut self,
        neuron_array: &mut NeuronArray,
        burst_count: u64,
    ) -> Result<(Vec<u32>, usize, usize)>;
    
    /// Process full burst cycle (synaptic + neural)
    /// 
    /// This is the primary entry point. Backends can override this for
    /// optimizations (e.g., keep data on GPU between stages).
    /// 
    /// Default implementation calls the two methods separately.
    fn process_burst(
        &mut self,
        fired_neurons: &[u32],
        synapse_array: &SynapseArray,
        neuron_array: &mut NeuronArray,
        burst_count: u64,
    ) -> Result<BackendBurstResult> {
        let start = std::time::Instant::now();
        
        // Phase 1: Synaptic propagation
        let synaptic_start = std::time::Instant::now();
        let _synapses_processed = self.process_synaptic_propagation(
            fired_neurons,
            synapse_array,
            neuron_array,
        )?;
        let synaptic_us = synaptic_start.elapsed().as_micros() as f64;
        
        // Phase 2: Neural dynamics
        let neural_start = std::time::Instant::now();
        let (new_fired, processed, in_refractory) = self.process_neural_dynamics(
            neuron_array,
            burst_count,
        )?;
        let neural_us = neural_start.elapsed().as_micros() as f64;
        
        let total_us = start.elapsed().as_micros() as f64;
        
        Ok(BackendBurstResult {
            fired_neurons: new_fired,
            neurons_processed: processed,
            neurons_fired: 0, // Will be set by caller
            neurons_in_refractory: in_refractory,
            timing: BurstTiming {
                synaptic_propagation_us: synaptic_us,
                neural_dynamics_us: neural_us,
                transfer_us: 0.0,
                total_us,
            },
        })
    }
    
    /// Initialize/upload persistent data to backend
    /// 
    /// For GPU backends, this uploads static data that doesn't change
    /// between bursts (thresholds, leak coefficients, etc.)
    /// 
    /// For CPU backends, this is a no-op.
    fn initialize_persistent_data(
        &mut self,
        _neuron_array: &NeuronArray,
        _synapse_array: &SynapseArray,
    ) -> Result<()> {
        Ok(())
    }
    
    /// Notify backend that genome has changed (invalidate caches)
    fn on_genome_change(&mut self) -> Result<()> {
        Ok(())
    }
}

/// Backend type enum for construction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendType {
    /// CPU with SIMD optimization (current implementation)
    CPU,
    
    /// GPU via WGPU (Metal/Vulkan/DirectX)
    #[cfg(feature = "gpu")]
    WGPU,
    
    /// Auto-select based on genome size and hardware availability
    Auto,
}

impl Default for BackendType {
    fn default() -> Self {
        Self::Auto
    }
}

impl std::fmt::Display for BackendType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BackendType::CPU => write!(f, "CPU"),
            #[cfg(feature = "gpu")]
            BackendType::WGPU => write!(f, "WGPU"),
            BackendType::Auto => write!(f, "Auto"),
        }
    }
}

impl std::str::FromStr for BackendType {
    type Err = Error;
    
    fn from_str(s: &str) -> Result<Self> {
        match s.to_lowercase().as_str() {
            "cpu" => Ok(BackendType::CPU),
            #[cfg(feature = "gpu")]
            "wgpu" | "gpu" => Ok(BackendType::WGPU),
            "auto" => Ok(BackendType::Auto),
            _ => Err(Error::InvalidBackend(s.to_string())),
        }
    }
}

/// Configuration for backend auto-selection
#[derive(Debug, Clone)]
pub struct BackendConfig {
    /// Minimum neurons to consider GPU (default: 500,000)
    pub gpu_neuron_threshold: usize,
    
    /// Minimum synapses to consider GPU (default: 50,000,000)
    pub gpu_synapse_threshold: usize,
    
    /// Minimum expected firing rate to benefit from GPU (default: 0.005 = 0.5%)
    pub gpu_min_firing_rate: f32,
    
    /// Force CPU even if GPU would be beneficial
    pub force_cpu: bool,
    
    /// Force GPU even if CPU would be better (for testing)
    pub force_gpu: bool,
}

impl Default for BackendConfig {
    fn default() -> Self {
        Self {
            // Based on our analysis: >500K neurons = 2-3x speedup
            gpu_neuron_threshold: 500_000,
            
            // 500K neurons × 100 synapses/neuron = 50M synapses
            gpu_synapse_threshold: 50_000_000,
            
            // Need at least 0.5% firing for GPU parallelism to be worth it
            gpu_min_firing_rate: 0.005,
            
            force_cpu: false,
            force_gpu: false,
        }
    }
}

/// Backend selection decision with rationale
#[derive(Debug, Clone)]
pub struct BackendDecision {
    pub backend_type: BackendType,
    pub reason: String,
    pub estimated_speedup: f32,
}

/// Auto-select optimal backend based on genome size and hardware
pub fn select_backend(
    neuron_count: usize,
    synapse_count: usize,
    config: &BackendConfig,
) -> BackendDecision {
    // Force overrides
    if config.force_cpu {
        return BackendDecision {
            backend_type: BackendType::CPU,
            reason: "Forced CPU via configuration".to_string(),
            estimated_speedup: 1.0,
        };
    }
    
    #[cfg(feature = "gpu")]
    if config.force_gpu {
        if is_gpu_available() {
            return BackendDecision {
                backend_type: BackendType::WGPU,
                reason: "Forced GPU via configuration".to_string(),
                estimated_speedup: estimate_gpu_speedup(neuron_count, synapse_count),
            };
        } else {
            return BackendDecision {
                backend_type: BackendType::CPU,
                reason: "GPU forced but not available, falling back to CPU".to_string(),
                estimated_speedup: 1.0,
            };
        }
    }
    
    // Check if genome is large enough to benefit from GPU
    let _meets_neuron_threshold = neuron_count >= config.gpu_neuron_threshold;
    let _meets_synapse_threshold = synapse_count >= config.gpu_synapse_threshold;
    
    #[cfg(feature = "gpu")]
    {
        if _meets_neuron_threshold || _meets_synapse_threshold {
            if is_gpu_available() {
                let speedup = estimate_gpu_speedup(neuron_count, synapse_count);
                
                // Only use GPU if speedup is meaningful (>1.5x)
                if speedup > 1.5 {
                    return BackendDecision {
                        backend_type: BackendType::WGPU,
                        reason: format!(
                            "Large genome ({} neurons, {} synapses) benefits from GPU",
                            neuron_count, synapse_count
                        ),
                        estimated_speedup: speedup,
                    };
                }
            }
        }
    }
    
    // Default to CPU
    BackendDecision {
        backend_type: BackendType::CPU,
        reason: format!(
            "Small genome ({} neurons, {} synapses) or GPU not available",
            neuron_count, synapse_count
        ),
        estimated_speedup: 1.0,
    }
}

/// Check if GPU is available
#[cfg(feature = "gpu")]
fn is_gpu_available() -> bool {
    use wgpu::Backends;
    
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: Backends::all(),
        ..Default::default()
    });
    
    pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        compatible_surface: None,
        force_fallback_adapter: false,
    }))
    .is_some()
}

/// Estimate GPU speedup based on genome size
#[cfg(feature = "gpu")]
fn estimate_gpu_speedup(neuron_count: usize, synapse_count: usize) -> f32 {
    // Empirical model based on realistic hardware:
    // - PCIe 4.0: ~25 GB/s bidirectional
    // - M4 Pro GPU: ~10 TFLOPS FP32
    // - CPU (16-core): ~100 GFLOPS effective
    // - Full GPU pipeline (synaptic + neural) assumed
    
    let neurons = neuron_count as f32;
    let synapses = synapse_count as f32;
    
    // Transfer time (microseconds) - realistic PCIe 4.0 speeds
    // NOTE: Synapses are PERSISTENT on GPU - no transfer cost per burst!
    // Per-burst transfers:
    // - Membrane potentials (4 bytes/neuron, both ways)
    // - Fired neuron mask (1 bit/neuron = 0.125 bytes/neuron, output only)
    // - Fired neuron IDs (assumed ~1% fire rate, 4 bytes each)
    let firing_rate = 0.01; // Assume 1% neurons fire per burst
    let transfer_bytes = (neurons * 4.0 * 2.0)  // Membrane potentials bidirectional
                       + (neurons * 0.125)     // Fired mask (bitpacked)
                       + (neurons * firing_rate * 4.0);  // Fired neuron IDs
    let transfer_bandwidth_gbs = 25.0;  // GB/s for PCIe 4.0
    // Convert: bytes / (GB/s * 1e9 bytes/GB) = seconds, then * 1e6 = microseconds
    let transfer_us = (transfer_bytes / (transfer_bandwidth_gbs * 1_000_000_000.0)) * 1_000_000.0 + 200.0; // +200μs fixed overhead
    
    // CPU compute time (microseconds)
    // Synaptic: ~10 ops per synapse (hash lookup, weight calc, accumulation)
    // Neural: ~20 ops per neuron (leak, threshold check, refractory, RNG)
    let cpu_flops = 100_000_000_000.0; // 100 GFLOPS effective (cache locality, branching)
    let cpu_synaptic_us = (synapses * 10.0) / (cpu_flops / 1_000_000.0);
    let cpu_neural_us = (neurons * 20.0) / (cpu_flops / 1_000_000.0);
    let cpu_total_us = cpu_synaptic_us + cpu_neural_us;
    
    // GPU compute time (microseconds)
    // GPU benefits from massive parallelism: 100-200x speedup for compute
    let gpu_flops = 10_000_000_000_000.0; // 10 TFLOPS (M4 Pro/RTX 4090)
    let gpu_synaptic_us = (synapses * 10.0) / (gpu_flops / 1_000_000.0);
    let gpu_neural_us = (neurons * 20.0) / (gpu_flops / 1_000_000.0);
    let gpu_compute_us = gpu_synaptic_us + gpu_neural_us;
    
    let gpu_total_us = transfer_us + gpu_compute_us;
    
    // Speedup = CPU time / GPU time
    let speedup = cpu_total_us / gpu_total_us;
    
    // Cap at reasonable maximum (100x)
    speedup.min(100.0).max(0.1)
}

/// Create backend based on type
pub fn create_backend(
    backend_type: BackendType,
    neuron_capacity: usize,
    synapse_capacity: usize,
    config: &BackendConfig,
) -> Result<Box<dyn ComputeBackend>> {
    let actual_type = if backend_type == BackendType::Auto {
        // Count will be updated later, use capacity as estimate
        let decision = select_backend(neuron_capacity, synapse_capacity, config);
        println!("🎯 Backend auto-selection: {} ({})", decision.backend_type, decision.reason);
        if decision.estimated_speedup > 1.0 {
            println!("   Estimated speedup: {:.1}x", decision.estimated_speedup);
        }
        decision.backend_type
    } else {
        backend_type
    };
    
    match actual_type {
        BackendType::CPU => {
            println!("🖥️  Using CPU backend (SIMD optimized)");
            Ok(Box::new(CPUBackend::new()))
        }
        #[cfg(feature = "gpu")]
        BackendType::WGPU => {
            println!("🎮 Using WGPU backend (GPU accelerated)");
            Ok(Box::new(WGPUBackend::new(neuron_capacity, synapse_capacity)?))
        }
        BackendType::Auto => {
            // Should not reach here, but fallback to CPU
            Ok(Box::new(CPUBackend::new()))
        }
    }
}

