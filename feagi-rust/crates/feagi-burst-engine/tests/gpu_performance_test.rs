/*
 * Copyright 2025 Neuraville Inc.
 */

//! GPU Performance Test - Measures actual speedup with full GPU pipeline

use feagi_burst_engine::*;
use feagi_types::*;
use std::time::Instant;

/// Create test genome with specified size
fn create_test_genome(neuron_count: usize, synapses_per_neuron: usize) -> (NeuronArray, SynapseArray) {
    let mut neuron_array = NeuronArray::new(neuron_count);
    let synapse_count = neuron_count * synapses_per_neuron;
    let mut synapse_array = SynapseArray::new(synapse_count);
    
    // Initialize neurons with varied thresholds
    for i in 0..neuron_count {
        neuron_array.membrane_potentials[i] = 0.0;
        neuron_array.thresholds[i] = 10.0 + (i % 10) as f32;  // Varied thresholds
        neuron_array.leak_coefficients[i] = 0.1;
        neuron_array.resting_potentials[i] = 0.0;
        neuron_array.excitabilities[i] = 1.0;
        neuron_array.valid_mask[i] = true;
    }
    neuron_array.count = neuron_count;
    
    // Initialize synapses (local connectivity)
    let mut synapse_idx = 0;
    for source in 0..neuron_count {
        for i in 0..synapses_per_neuron {
            let target = (source + i + 1) % neuron_count;
            if synapse_idx < synapse_count {
                synapse_array.source_neurons[synapse_idx] = source as u32;
                synapse_array.target_neurons[synapse_idx] = target as u32;
                synapse_array.weights[synapse_idx] = 128;  // Mid-range weight
                synapse_array.conductances[synapse_idx] = 200;
                synapse_array.types[synapse_idx] = if i % 4 == 0 { 1 } else { 0 }; // 75% excitatory
                synapse_array.valid_mask[synapse_idx] = true;
                
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

/// Simulate firing neurons (1% firing rate)
fn create_fired_neurons(neuron_count: usize) -> Vec<u32> {
    let fire_count = (neuron_count as f32 * 0.01).max(10.0) as usize;
    (0..fire_count).map(|i| (i * (neuron_count / fire_count)) as u32).collect()
}

#[test]
fn test_gpu_full_pipeline_speedup() {
    // Test parameters
    let neuron_count = 10_000;
    let synapses_per_neuron = 100;
    let burst_iterations = 10;
    
    println!("\n🎯 GPU Full Pipeline Performance Test");
    println!("   Neurons: {}, Synapses/neuron: {}, Iterations: {}",
             neuron_count, synapses_per_neuron, burst_iterations);
    println!("   Total synapses: {}\n", neuron_count * synapses_per_neuron);
    
    // Create test genome
    let (neuron_array, synapse_array) = create_test_genome(neuron_count, synapses_per_neuron);
    let fired_neurons = create_fired_neurons(neuron_count);
    
    // ═══════════════════════════════════════════════════════════
    // CPU BACKEND
    // ═══════════════════════════════════════════════════════════
    
    println!("🖥️  Testing CPU backend...");
    let mut cpu_backend = backend::CPUBackend::new();
    cpu_backend.initialize_persistent_data(&neuron_array, &synapse_array)
        .expect("CPU init failed");
    
    let cpu_start = Instant::now();
    for _i in 0..burst_iterations {
        let mut fcl = FireCandidateList::new();
        let mut neuron_array_cpu = neuron_array.clone();
        
        // Synaptic propagation
        cpu_backend.process_synaptic_propagation(&fired_neurons, &synapse_array, &mut fcl)
            .expect("CPU synaptic failed");
        
        // Neural dynamics
        cpu_backend.process_neural_dynamics(&fcl, &mut neuron_array_cpu, 1)
            .expect("CPU neural failed");
    }
    let cpu_duration = cpu_start.elapsed();
    let cpu_us_per_burst = cpu_duration.as_micros() / burst_iterations as u128;
    
    println!("   Total: {:.2}ms", cpu_duration.as_secs_f64() * 1000.0);
    println!("   Per burst: {}μs\n", cpu_us_per_burst);
    
    // ═══════════════════════════════════════════════════════════
    // GPU BACKEND
    // ═══════════════════════════════════════════════════════════
    
    #[cfg(feature = "gpu")]
    {
        println!("🎮 Testing GPU backend...");
        let mut gpu_backend = backend::WGPUBackend::new(neuron_count * 2, synapse_array.capacity)
            .expect("GPU init failed");
        gpu_backend.initialize_persistent_data(&neuron_array, &synapse_array)
            .expect("GPU data upload failed");
        
        // Warm-up (first run includes initial transfers)
        {
            let mut fcl = FireCandidateList::new();
            let mut neuron_array_gpu = neuron_array.clone();
            gpu_backend.process_synaptic_propagation(&fired_neurons, &synapse_array, &mut fcl)
                .expect("GPU synaptic warm-up failed");
            gpu_backend.process_neural_dynamics(&fcl, &mut neuron_array_gpu, 1)
                .expect("GPU neural warm-up failed");
        }
        
        println!("   (Warm-up complete)");
        
        let gpu_start = Instant::now();
        for _i in 0..burst_iterations {
            let mut fcl = FireCandidateList::new();
            let mut neuron_array_gpu = neuron_array.clone();
            
            // Synaptic propagation (GPU)
            gpu_backend.process_synaptic_propagation(&fired_neurons, &synapse_array, &mut fcl)
                .expect("GPU synaptic failed");
            
            // Neural dynamics (GPU)
            gpu_backend.process_neural_dynamics(&fcl, &mut neuron_array_gpu, 1)
                .expect("GPU neural failed");
        }
        let gpu_duration = gpu_start.elapsed();
        let gpu_us_per_burst = gpu_duration.as_micros() / burst_iterations as u128;
        
        println!("   Total: {:.2}ms", gpu_duration.as_secs_f64() * 1000.0);
        println!("   Per burst: {}μs\n", gpu_us_per_burst);
        
        // ═══════════════════════════════════════════════════════════
        // RESULTS
        // ═══════════════════════════════════════════════════════════
        
        let speedup = cpu_us_per_burst as f64 / gpu_us_per_burst as f64;
        
        println!("📊 RESULTS:");
        println!("   CPU: {}μs per burst", cpu_us_per_burst);
        println!("   GPU: {}μs per burst", gpu_us_per_burst);
        println!("   Speedup: {:.1}x", speedup);
        
        if speedup >= 5.0 {
            println!("   ✅ GPU acceleration working! (target: 5-160x)\n");
        } else {
            println!("   ⚠️  Speedup below expected ({}x < 5x)\n", speedup);
            println!("   Note: Small genomes may not benefit from GPU");
            println!("   Try larger genomes (100K+ neurons) for better speedup\n");
        }
    }
    
    #[cfg(not(feature = "gpu"))]
    {
        println!("⚠️  GPU feature not enabled. Run with --features gpu");
    }
}

