/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Backend Performance Benchmarks
//!
//! Validates that GPU acceleration provides expected benefits for large genomes.
//! Tests multiple genome sizes to find the crossover point where GPU becomes beneficial.

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use feagi_burst_engine::*;
use feagi_types::*;

/// Create a test genome with specified size
fn create_test_genome(neuron_count: usize, synapses_per_neuron: usize) -> (NeuronArray, SynapseArray) {
    let mut neuron_array = NeuronArray::new(neuron_count);
    let synapse_count = neuron_count * synapses_per_neuron;
    let mut synapse_array = SynapseArray::new(synapse_count);
    
    // Initialize neurons
    for i in 0..neuron_count {
        neuron_array.membrane_potentials[i] = 0.0;
        neuron_array.thresholds[i] = 10.0;
        neuron_array.leak_coefficients[i] = 0.1;
        neuron_array.resting_potentials[i] = 0.0;
        neuron_array.excitabilities[i] = 1.0;
        neuron_array.valid_mask[i] = true;
    }
    neuron_array.count = neuron_count;
    
    // Initialize synapses (fully connected within small neighborhoods)
    let mut synapse_idx = 0;
    for source in 0..neuron_count {
        for i in 0..synapses_per_neuron {
            let target = (source + i + 1) % neuron_count;
            if synapse_idx < synapse_count {
                synapse_array.source_neurons[synapse_idx] = source as u32;
                synapse_array.target_neurons[synapse_idx] = target as u32;
                synapse_array.weights[synapse_idx] = 5;
                synapse_array.conductances[synapse_idx] = 10;
                synapse_array.types[synapse_idx] = if i % 2 == 0 { 0 } else { 1 }; // Mix of excitatory/inhibitory
                synapse_array.valid_mask[synapse_idx] = true;
                
                // Build index
                synapse_array.source_index
                    .entry(source as u32)
                    .or_insert_with(Vec::new)
                    .push(synapse_idx);
                
                synapse_idx += 1;
            }
        }
    }
    synapse_array.count = synapse_idx;
    
    (neuron_array, synapse_array)
}

/// Generate fired neurons (simulate 1% firing rate)
fn generate_fired_neurons(neuron_count: usize, firing_rate: f32) -> Vec<u32> {
    let fire_count = (neuron_count as f32 * firing_rate) as usize;
    (0..fire_count).map(|i| (i * (neuron_count / fire_count)) as u32).collect()
}

/// Benchmark CPU backend
fn bench_cpu_backend(c: &mut Criterion) {
    let mut group = c.benchmark_group("cpu_backend");
    
    let test_sizes = vec![
        (10_000, "10K"),
        (50_000, "50K"),
        (100_000, "100K"),
        (500_000, "500K"),
    ];
    
    for (neuron_count, label) in test_sizes {
        let synapses_per_neuron = 100;
        let (mut neuron_array, synapse_array) = create_test_genome(neuron_count, synapses_per_neuron);
        let fired_neurons = generate_fired_neurons(neuron_count, 0.01);
        
        group.throughput(Throughput::Elements(neuron_count as u64));
        group.bench_with_input(
            BenchmarkId::new("full_burst", label),
            &neuron_count,
            |b, _| {
                let mut backend = CPUBackend::new();
                b.iter(|| {
                    let _ = backend.process_burst(
                        black_box(&fired_neurons),
                        black_box(&synapse_array),
                        black_box(&mut neuron_array),
                        black_box(1),
                    );
                });
            },
        );
        
        group.bench_with_input(
            BenchmarkId::new("synaptic_only", label),
            &neuron_count,
            |b, _| {
                let mut backend = CPUBackend::new();
                b.iter(|| {
                    let _ = backend.process_synaptic_propagation(
                        black_box(&fired_neurons),
                        black_box(&synapse_array),
                        black_box(&mut neuron_array),
                    );
                });
            },
        );
        
        group.bench_with_input(
            BenchmarkId::new("neural_only", label),
            &neuron_count,
            |b, _| {
                let mut backend = CPUBackend::new();
                b.iter(|| {
                    let _ = backend.process_neural_dynamics(
                        black_box(&mut neuron_array),
                        black_box(1),
                    );
                });
            },
        );
    }
    
    group.finish();
}

/// Benchmark GPU backend (if available)
#[cfg(feature = "gpu")]
fn bench_gpu_backend(c: &mut Criterion) {
    // Check if GPU is available
    if !is_gpu_available() {
        println!("⚠️  GPU not available, skipping GPU benchmarks");
        return;
    }
    
    let mut group = c.benchmark_group("gpu_backend");
    
    let test_sizes = vec![
        (10_000, "10K"),
        (50_000, "50K"),
        (100_000, "100K"),
        (500_000, "500K"),
        (1_000_000, "1M"),
    ];
    
    for (neuron_count, label) in test_sizes {
        let synapses_per_neuron = 100;
        let (mut neuron_array, synapse_array) = create_test_genome(neuron_count, synapses_per_neuron);
        let fired_neurons = generate_fired_neurons(neuron_count, 0.01);
        
        // Create GPU backend
        let synapse_count = neuron_count * synapses_per_neuron;
        let mut backend = match WGPUBackend::new(neuron_count, synapse_count) {
            Ok(b) => b,
            Err(_) => {
                println!("⚠️  Failed to create GPU backend for {}", label);
                continue;
            }
        };
        
        // Initialize persistent data (one-time cost)
        let _ = backend.initialize_persistent_data(&neuron_array, &synapse_array);
        
        group.throughput(Throughput::Elements(neuron_count as u64));
        group.bench_with_input(
            BenchmarkId::new("full_burst", label),
            &neuron_count,
            |b, _| {
                b.iter(|| {
                    let _ = backend.process_burst(
                        black_box(&fired_neurons),
                        black_box(&synapse_array),
                        black_box(&mut neuron_array),
                        black_box(1),
                    );
                });
            },
        );
    }
    
    group.finish();
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

/// Compare CPU vs GPU for different genome sizes
#[cfg(feature = "gpu")]
fn bench_cpu_vs_gpu_comparison(c: &mut Criterion) {
    if !is_gpu_available() {
        println!("⚠️  GPU not available, skipping comparison benchmarks");
        return;
    }
    
    let mut group = c.benchmark_group("cpu_vs_gpu");
    
    let test_sizes = vec![
        (10_000, "10K"),
        (100_000, "100K"),
        (500_000, "500K"),
        (1_000_000, "1M"),
    ];
    
    for (neuron_count, label) in test_sizes {
        let synapses_per_neuron = 100;
        let (mut neuron_array_cpu, synapse_array) = create_test_genome(neuron_count, synapses_per_neuron);
        let mut neuron_array_gpu = neuron_array_cpu.clone();
        let fired_neurons = generate_fired_neurons(neuron_count, 0.01);
        
        group.throughput(Throughput::Elements(neuron_count as u64));
        
        // CPU benchmark
        group.bench_with_input(
            BenchmarkId::new("cpu", label),
            &neuron_count,
            |b, _| {
                let mut backend = CPUBackend::new();
                b.iter(|| {
                    let _ = backend.process_burst(
                        black_box(&fired_neurons),
                        black_box(&synapse_array),
                        black_box(&mut neuron_array_cpu),
                        black_box(1),
                    );
                });
            },
        );
        
        // GPU benchmark
        let synapse_count = neuron_count * synapses_per_neuron;
        if let Ok(mut backend) = WGPUBackend::new(neuron_count, synapse_count) {
            let _ = backend.initialize_persistent_data(&neuron_array_gpu, &synapse_array);
            
            group.bench_with_input(
                BenchmarkId::new("gpu", label),
                &neuron_count,
                |b, _| {
                    b.iter(|| {
                        let _ = backend.process_burst(
                            black_box(&fired_neurons),
                            black_box(&synapse_array),
                            black_box(&mut neuron_array_gpu),
                            black_box(1),
                        );
                    });
                },
            );
        }
    }
    
    group.finish();
}

/// Test backend auto-selection logic
fn bench_auto_selection(c: &mut Criterion) {
    let mut group = c.benchmark_group("auto_selection");
    
    let test_cases = vec![
        (10_000, 1_000_000, "small_genome"),
        (100_000, 10_000_000, "medium_genome"),
        (500_000, 50_000_000, "large_genome"),
        (1_000_000, 100_000_000, "xlarge_genome"),
    ];
    
    for (neuron_count, synapse_count, label) in test_cases {
        group.bench_with_input(
            BenchmarkId::new("select_backend", label),
            &(neuron_count, synapse_count),
            |b, &(neurons, synapses)| {
                let config = BackendConfig::default();
                b.iter(|| {
                    black_box(select_backend(neurons, synapses, &config))
                });
            },
        );
    }
    
    group.finish();
}

criterion_group!(
    benches,
    bench_cpu_backend,
    #[cfg(feature = "gpu")]
    bench_gpu_backend,
    #[cfg(feature = "gpu")]
    bench_cpu_vs_gpu_comparison,
    bench_auto_selection,
);

criterion_main!(benches);

