/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Backend Selection Tests
//!
//! Validates that the auto-selection logic chooses the right backend
//! based on genome size and hardware availability.

use feagi_burst_engine::*;

#[test]
fn test_small_genome_selects_cpu() {
    let config = BackendConfig::default();
    
    // Small genome: 10K neurons, 1M synapses
    let decision = select_backend(10_000, 1_000_000, &config);
    
    assert_eq!(decision.backend_type, BackendType::CPU);
    assert!(decision.reason.contains("Small genome"));
    assert_eq!(decision.estimated_speedup, 1.0);
}

#[test]
fn test_medium_genome_threshold() {
    let config = BackendConfig::default();
    
    // Just below threshold: 400K neurons
    let decision1 = select_backend(400_000, 40_000_000, &config);
    assert_eq!(decision1.backend_type, BackendType::CPU);
    
    // Just above threshold: 600K neurons
    let decision2 = select_backend(600_000, 60_000_000, &config);
    
    #[cfg(feature = "gpu")]
    {
        // Should select GPU if available
        if is_gpu_available() {
            assert_eq!(decision2.backend_type, BackendType::WGPU);
            assert!(decision2.estimated_speedup > 1.5);
        } else {
            assert_eq!(decision2.backend_type, BackendType::CPU);
        }
    }
    
    #[cfg(not(feature = "gpu"))]
    {
        assert_eq!(decision2.backend_type, BackendType::CPU);
    }
}

#[test]
fn test_large_genome_prefers_gpu() {
    let config = BackendConfig::default();
    
    // Large genome: 1M neurons, 100M synapses
    let decision = select_backend(1_000_000, 100_000_000, &config);
    
    #[cfg(feature = "gpu")]
    {
        if is_gpu_available() {
            assert_eq!(decision.backend_type, BackendType::WGPU);
            assert!(decision.reason.contains("Large genome"));
            assert!(decision.estimated_speedup > 2.0, "Expected >2x speedup, got {}x", decision.estimated_speedup);
        }
    }
}

#[test]
fn test_force_cpu_override() {
    let mut config = BackendConfig::default();
    config.force_cpu = true;
    
    // Even with large genome, should force CPU
    let decision = select_backend(1_000_000, 100_000_000, &config);
    
    assert_eq!(decision.backend_type, BackendType::CPU);
    assert!(decision.reason.contains("Forced CPU"));
}

#[cfg(feature = "gpu")]
#[test]
fn test_force_gpu_override() {
    let mut config = BackendConfig::default();
    config.force_gpu = true;
    
    // Even with small genome, should try GPU
    let decision = select_backend(10_000, 1_000_000, &config);
    
    if is_gpu_available() {
        assert_eq!(decision.backend_type, BackendType::WGPU);
        assert!(decision.reason.contains("Forced GPU"));
    } else {
        // Should fallback to CPU if GPU not available
        assert_eq!(decision.backend_type, BackendType::CPU);
        assert!(decision.reason.contains("not available"));
    }
}

#[test]
fn test_custom_thresholds() {
    let mut config = BackendConfig::default();
    config.gpu_neuron_threshold = 100_000; // Lower threshold
    
    // 150K neurons should now trigger GPU
    let decision = select_backend(150_000, 15_000_000, &config);
    
    #[cfg(feature = "gpu")]
    {
        if is_gpu_available() {
            assert_eq!(decision.backend_type, BackendType::WGPU);
        }
    }
}

#[test]
fn test_speedup_estimation_scales() {
    let config = BackendConfig::default();
    
    let sizes = vec![
        (100_000, 10_000_000),
        (500_000, 50_000_000),
        (1_000_000, 100_000_000),
        (5_000_000, 500_000_000),
    ];
    
    let mut prev_speedup = 0.0;
    
    for (neurons, synapses) in sizes {
        let decision = select_backend(neurons, synapses, &config);
        
        #[cfg(feature = "gpu")]
        {
            if is_gpu_available() && decision.backend_type == BackendType::WGPU {
                // Speedup should increase with genome size
                assert!(
                    decision.estimated_speedup >= prev_speedup,
                    "Speedup should increase with genome size: {} >= {}",
                    decision.estimated_speedup,
                    prev_speedup
                );
                prev_speedup = decision.estimated_speedup;
            }
        }
        
        println!(
            "Genome: {}K neurons, {}M synapses → {} ({}x speedup)",
            neurons / 1000,
            synapses / 1_000_000,
            decision.backend_type,
            decision.estimated_speedup
        );
    }
}

#[test]
fn test_backend_creation_cpu() {
    let config = BackendConfig::default();
    
    let backend = create_backend(BackendType::CPU, 10_000, 1_000_000, &config);
    assert!(backend.is_ok());
    
    let backend = backend.unwrap();
    assert!(backend.backend_name().contains("CPU"));
}

#[cfg(feature = "gpu")]
#[test]
fn test_backend_creation_gpu() {
    let config = BackendConfig::default();
    
    if is_gpu_available() {
        let backend = create_backend(BackendType::WGPU, 10_000, 1_000_000, &config);
        assert!(backend.is_ok());
        
        let backend = backend.unwrap();
        assert!(backend.backend_name().contains("WGPU"));
    }
}

#[test]
fn test_backend_creation_auto() {
    let config = BackendConfig::default();
    
    // Small genome - should get CPU
    let backend_small = create_backend(BackendType::Auto, 10_000, 1_000_000, &config);
    assert!(backend_small.is_ok());
    
    // Large genome - should get GPU if available
    let backend_large = create_backend(BackendType::Auto, 1_000_000, 100_000_000, &config);
    assert!(backend_large.is_ok());
}

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

