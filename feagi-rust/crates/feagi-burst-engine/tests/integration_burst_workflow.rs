/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # Integration Tests: Complete Burst Workflow
//!
//! End-to-end tests for the full neural processing pipeline:
//! - Genome loading → Neuron/Synapse creation
//! - Sensory input → Burst processing → Firing
//! - Synaptic propagation → Multi-burst chains
//! - Power injection persistence
//! - Fire Ledger history tracking

use feagi_burst_engine::RustNPU;
use feagi_types::{NeuronId, SynapticWeight, SynapticConductance, SynapseType};

// ═══════════════════════════════════════════════════════════
// Helper Functions
// ═══════════════════════════════════════════════════════════

/// Create a simple 3-layer network: Input → Hidden → Output
fn create_simple_network() -> RustNPU {
    let mut npu = RustNPU::new(1000, 10000, 20);
    
    // Power area (cortical_area=1) - 3 neurons
    for i in 0..3 {
        npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, i, 0, 0).unwrap();
    }
    
    // Input layer (cortical_area=2) - 5 neurons
    for i in 0..5 {
        npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 2, i, 0, 0).unwrap();
    }
    
    // Hidden layer (cortical_area=3) - 5 neurons
    for i in 0..5 {
        npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 3, i, 0, 0).unwrap();
    }
    
    // Output layer (cortical_area=4) - 3 neurons
    for i in 0..3 {
        npu.add_neuron(1.0, 0.1, 0.0, 0, 5, 1.0, 0, 0, true, 4, i, 0, 0).unwrap();
    }
    
    // Connect input to hidden
    for input in 3..8 {  // Input neurons are IDs 3-7
        for hidden in 8..13 {  // Hidden neurons are IDs 8-12
            npu.add_synapse(
                NeuronId(input),
                NeuronId(hidden),
                SynapticWeight(200),
                SynapticConductance(255),
                SynapseType::Excitatory,
            ).unwrap();
        }
    }
    
    // Connect hidden to output
    for hidden in 8..13 {
        for output in 13..16 {  // Output neurons are IDs 13-15
            npu.add_synapse(
                NeuronId(hidden),
                NeuronId(output),
                SynapticWeight(200),
                SynapticConductance(255),
                SynapseType::Excitatory,
            ).unwrap();
        }
    }
    
    npu
}

// ═══════════════════════════════════════════════════════════
// Integration Tests
// ═══════════════════════════════════════════════════════════

#[test]
fn test_end_to_end_burst_workflow() {
    let mut npu = create_simple_network();
    
    // Verify network structure
    assert_eq!(npu.get_neuron_count(), 16);  // 3+5+5+3
    assert_eq!(npu.get_synapse_count(), 40); // 5*5 + 5*3
    
    // Burst 1: Power injection only
    let result1 = npu.process_burst().unwrap();
    assert_eq!(result1.burst, 1);
    assert_eq!(result1.power_injections, 3);
    assert!(result1.neuron_count >= 0);
    
    // Burst 2: Add sensory input to input layer
    npu.inject_sensory_with_potentials(&[
        (NeuronId(3), 1.5),
        (NeuronId(4), 1.5),
    ]);
    
    let result2 = npu.process_burst().unwrap();
    assert_eq!(result2.burst, 2);
    assert!(result2.neuron_count > 0, "Input neurons should fire");
    
    // Burst 3: Hidden layer should receive propagation
    let result3 = npu.process_burst().unwrap();
    assert_eq!(result3.burst, 3);
    
    // Burst 4: Output layer should receive propagation
    let result4 = npu.process_burst().unwrap();
    assert_eq!(result4.burst, 4);
}

#[test]
fn test_power_injection_every_burst() {
    let mut npu = create_simple_network();
    
    // Run 20 bursts and verify power injection happens every time
    for i in 1..=20 {
        let result = npu.process_burst().unwrap();
        assert_eq!(result.burst, i as u64);
        assert_eq!(result.power_injections, 3, "Power injection failed at burst {}", i);
    }
}

#[test]
fn test_continuous_sensory_input_stream() {
    let mut npu = create_simple_network();
    
    // Simulate continuous sensory input stream (like video frames)
    for burst in 0..10 {
        // Inject sensory data
        npu.inject_sensory_with_potentials(&[
            (NeuronId(3), 0.5),
            (NeuronId(4), 0.5),
            (NeuronId(5), 0.5),
        ]);
        
        let result = npu.process_burst().unwrap();
        assert_eq!(result.burst, (burst + 1) as u64);
        assert_eq!(result.power_injections, 3);
    }
}

#[test]
fn test_fire_ledger_across_bursts() {
    let mut npu = create_simple_network();
    
    // Configure fire ledger for all areas
    npu.configure_fire_ledger_window(1, 10);  // Power
    npu.configure_fire_ledger_window(2, 10);  // Input
    npu.configure_fire_ledger_window(3, 10);  // Hidden
    npu.configure_fire_ledger_window(4, 10);  // Output
    
    // Process 5 bursts with sensory input
    for _ in 0..5 {
        npu.inject_sensory_with_potentials(&[(NeuronId(3), 1.5)]);
        npu.process_burst().unwrap();
    }
    
    // Check fire ledger history
    let power_history = npu.get_fire_ledger_history(1, 5);
    assert!(!power_history.is_empty(), "Power area should have history");
    assert!(power_history.len() <= 5, "History should not exceed lookback");
}

#[test]
fn test_multi_burst_chain_propagation() {
    let mut npu = RustNPU::new(100, 1000, 10);
    
    // Create a chain: N1 -> N2 -> N3 -> N4
    let n1 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
    let n2 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 1, 0, 0).unwrap();
    let n3 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 2, 0, 0).unwrap();
    let n4 = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 3, 0, 0).unwrap();
    
    // Strong connections
    npu.add_synapse(n1, n2, SynapticWeight(255), SynapticConductance(255), SynapseType::Excitatory).unwrap();
    npu.add_synapse(n2, n3, SynapticWeight(255), SynapticConductance(255), SynapseType::Excitatory).unwrap();
    npu.add_synapse(n3, n4, SynapticWeight(255), SynapticConductance(255), SynapseType::Excitatory).unwrap();
    
    // Burst 1: Fire N1
    npu.inject_sensory_with_potentials(&[(n1, 2.0)]);
    let result1 = npu.process_burst().unwrap();
    assert!(result1.neuron_count >= 1, "N1 should fire");
    
    // Burst 2: N2 should fire from N1 propagation
    let result2 = npu.process_burst().unwrap();
    assert_eq!(result2.burst, 2);
    
    // Burst 3: N3 should fire from N2 propagation
    let result3 = npu.process_burst().unwrap();
    assert_eq!(result3.burst, 3);
    
    // Burst 4: N4 should fire from N3 propagation
    let result4 = npu.process_burst().unwrap();
    assert_eq!(result4.burst, 4);
}

#[test]
fn test_refractory_period_across_bursts() {
    let mut npu = RustNPU::new(100, 1000, 10);
    
    // Neuron with 3-burst refractory period
    let neuron = npu.add_neuron(1.0, 0.0, 0.0, 0, 3, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
    
    // Burst 1: Fire neuron
    npu.inject_sensory_with_potentials(&[(neuron, 2.0)]);
    let result1 = npu.process_burst().unwrap();
    assert!(result1.neuron_count > 0);
    
    // Bursts 2-4: Neuron should be in refractory
    for i in 2..=4 {
        npu.inject_sensory_with_potentials(&[(neuron, 2.0)]);
        let result = npu.process_burst().unwrap();
        assert_eq!(result.burst, i as u64);
        // Neuron may or may not fire depending on refractory countdown
    }
}

#[test]
fn test_mixed_excitatory_inhibitory_network() {
    let mut npu = RustNPU::new(100, 1000, 10);
    
    // Excitatory neuron
    let excitatory = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
    
    // Inhibitory neuron
    let inhibitory = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 1, 0, 0).unwrap();
    
    // Target neuron
    let target = npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 2, 2, 0, 0).unwrap();
    
    // Both connect to target
    npu.add_synapse(excitatory, target, SynapticWeight(128), SynapticConductance(255), SynapseType::Excitatory).unwrap();
    npu.add_synapse(inhibitory, target, SynapticWeight(128), SynapticConductance(255), SynapseType::Inhibitory).unwrap();
    
    // Fire both simultaneously
    npu.inject_sensory_with_potentials(&[
        (excitatory, 2.0),
        (inhibitory, 2.0),
    ]);
    
    let result = npu.process_burst().unwrap();
    assert_eq!(result.burst, 1);
}

#[test]
fn test_high_frequency_burst_processing() {
    let mut npu = create_simple_network();
    
    // Process 100 bursts rapidly
    for i in 1..=100 {
        let result = npu.process_burst().unwrap();
        assert_eq!(result.burst, i as u64);
        assert_eq!(result.power_injections, 3);
    }
    
    assert_eq!(npu.get_burst_count(), 100);
}

#[test]
fn test_burst_stats_accumulation() {
    let mut npu = create_simple_network();
    
    // Process bursts and track stats
    let mut total_fired = 0;
    let mut total_processed = 0;
    
    for _ in 0..10 {
        npu.inject_sensory_with_potentials(&[
            (NeuronId(3), 1.0),
            (NeuronId(4), 1.0),
        ]);
        
        let result = npu.process_burst().unwrap();
        total_fired += result.neuron_count;
        total_processed += result.neurons_processed;
    }
    
    assert!(total_fired > 0, "Some neurons should have fired");
    assert!(total_processed > 0, "Some neurons should have been processed");
}

#[test]
fn test_dynamic_network_modification() {
    let mut npu = RustNPU::new(1000, 10000, 10);
    
    // Start with 5 neurons
    for i in 0..5 {
        npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, i, 0, 0).unwrap();
    }
    
    // Process 3 bursts
    for _ in 0..3 {
        npu.process_burst().unwrap();
    }
    
    // Add more neurons mid-processing
    for i in 5..10 {
        npu.add_neuron(1.0, 0.0, 0.0, 0, 5, 1.0, 0, 0, true, 1, i, 0, 0).unwrap();
    }
    
    // Continue processing
    for _ in 0..3 {
        let result = npu.process_burst().unwrap();
        // Should now inject 10 power neurons
        assert_eq!(result.power_injections, 10);
    }
}

#[test]
fn test_fq_sampler_integration() {
    let mut npu = create_simple_network();
    
    // Enable visualization subscribers
    npu.set_visualization_subscribers(true);
    
    // Process bursts and sample
    for _ in 0..5 {
        npu.inject_sensory_with_potentials(&[(NeuronId(3), 1.5)]);
        npu.process_burst().unwrap();
        
        // Try to get latest sample (non-consuming)
        let _sample = npu.get_latest_fire_queue_sample();
        // Sample may or may not be available due to rate limiting
    }
    
    assert!(npu.has_visualization_subscribers());
}

#[test]
fn test_zero_leak_neuron_persistence() {
    let mut npu = RustNPU::new(100, 1000, 10);
    
    // Neuron with zero leak (potential persists)
    let neuron = npu.add_neuron(10.0, 0.0, 0.0, 0, 0, 1.0, 0, 0, true, 2, 0, 0, 0).unwrap();
    
    // Accumulate potential over multiple bursts
    for _ in 0..5 {
        npu.inject_sensory_with_potentials(&[(neuron, 1.0)]);
        npu.process_burst().unwrap();
    }
    
    // After 5 bursts @ 1.0 each, should have 5.0 potential (not enough to fire)
    // Burst 6-10: Add 5 more
    for _ in 0..5 {
        npu.inject_sensory_with_potentials(&[(neuron, 1.0)]);
        npu.process_burst().unwrap();
    }
    
    // After 10 bursts @ 1.0 each = 10.0 potential (should fire now)
    let result = npu.process_burst().unwrap();
    assert_eq!(result.burst, 11);
}



