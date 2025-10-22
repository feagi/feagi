/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! Integration tests for feagi-plasticity
//!
//! These tests verify that all components work together correctly.

use feagi_plasticity::*;
use std::collections::HashSet;

#[test]
fn test_end_to_end_pattern_detection_and_memory_creation() {
    // Setup
    let pattern_config = PatternConfig::default();
    let detector = PatternDetector::new(pattern_config);
    
    let lifecycle_config = MemoryNeuronLifecycleConfig::default();
    let mut memory_array = MemoryNeuronArray::new(1000);
    
    // Simulate pattern detection
    let mut bitmap1 = HashSet::new();
    bitmap1.insert(1);
    bitmap1.insert(2);
    bitmap1.insert(3);
    
    let mut bitmap2 = HashSet::new();
    bitmap2.insert(4);
    bitmap2.insert(5);
    
    let bitmaps = vec![bitmap1, bitmap2];
    let upstream_areas = vec![1, 2];
    
    let pattern = detector.detect_pattern(100, &upstream_areas, 10, bitmaps, None);
    
    assert!(pattern.is_some());
    let pattern = pattern.unwrap();
    
    // Create memory neuron for this pattern
    let neuron_idx = memory_array.create_memory_neuron(
        pattern.pattern_hash,
        100,
        10,
        &lifecycle_config,
    );
    
    assert!(neuron_idx.is_some());
    let idx = neuron_idx.unwrap();
    
    // Verify pattern can be found
    let found_idx = memory_array.find_neuron_by_pattern(&pattern.pattern_hash);
    assert_eq!(found_idx, Some(idx));
    
    // Verify stats show active neuron
    let stats = memory_array.get_stats();
    assert_eq!(stats.active_neurons, 1);
}

#[test]
fn test_pattern_reactivation_workflow() {
    let pattern_config = PatternConfig::default();
    let detector = PatternDetector::new(pattern_config);
    
    let lifecycle_config = MemoryNeuronLifecycleConfig::default();
    let mut memory_array = MemoryNeuronArray::new(1000);
    
    // Create initial pattern
    let mut bitmap = HashSet::new();
    bitmap.insert(1);
    bitmap.insert(2);
    
    let pattern1 = detector.detect_pattern(
        100,
        &vec![1],
        10,
        vec![bitmap.clone()],
        None,
    ).unwrap();
    
    let idx1 = memory_array.create_memory_neuron(
        pattern1.pattern_hash,
        100,
        10,
        &lifecycle_config,
    ).unwrap();
    
    // Detect same pattern again (reactivation)
    let pattern2 = detector.detect_pattern(
        100,
        &vec![1],
        11,
        vec![bitmap],
        None,
    ).unwrap();
    
    // Patterns should be identical
    assert_eq!(pattern1.pattern_hash, pattern2.pattern_hash);
    
    // Reactivate the neuron
    let success = memory_array.reactivate_memory_neuron(idx1, 11);
    assert!(success);
    
    // Verify reactivation through stats
    let stats = memory_array.get_stats();
    assert_eq!(stats.active_neurons, 1);
}

#[test]
fn test_neuron_lifecycle_full_cycle() {
    let mut memory_array = MemoryNeuronArray::new(1000);
    let mut lifecycle_config = MemoryNeuronLifecycleConfig::default();
    lifecycle_config.initial_lifespan = 5;
    lifecycle_config.lifespan_growth_rate = 2.0;
    lifecycle_config.longterm_threshold = 15;
    
    let pattern_hash = [1u8; 32];
    
    // Create neuron
    let idx = memory_array.create_memory_neuron(pattern_hash, 100, 0, &lifecycle_config).unwrap();
    
    // Reactivate to grow lifespan
    for burst in 1..=3 {
        memory_array.reactivate_memory_neuron(idx, burst);
    }
    
    // Reactivate twice more to reach longterm threshold
    memory_array.reactivate_memory_neuron(idx, 4);
    memory_array.reactivate_memory_neuron(idx, 5);
    
    // Check for longterm conversion
    let converted = memory_array.check_longterm_conversion(15);
    assert_eq!(converted.len(), 1);
    
    // Verify it's still active after many aging cycles
    for burst in 6..=20 {
        memory_array.age_memory_neurons(burst);
    }
    
    let stats = memory_array.get_stats();
    assert_eq!(stats.active_neurons, 1);
    assert_eq!(stats.longterm_neurons, 1);
}

#[test]
fn test_multiple_memory_areas() {
    let pattern_config = PatternConfig::default();
    let batch_detector = BatchPatternDetector::new(pattern_config);
    
    let mut memory_array = MemoryNeuronArray::new(1000);
    let lifecycle_config = MemoryNeuronLifecycleConfig::default();
    
    // Create patterns for multiple memory areas
    for area_idx in vec![100, 200, 300] {
        let detector = batch_detector.get_detector(area_idx, 3);
        
        let mut bitmap = HashSet::new();
        bitmap.insert(area_idx);
        
        let pattern = detector.detect_pattern(
            area_idx,
            &vec![1],
            10,
            vec![bitmap],
            None,
        ).unwrap();
        
        memory_array.create_memory_neuron(
            pattern.pattern_hash,
            area_idx,
            10,
            &lifecycle_config,
        ).unwrap();
    }
    
    // Verify neurons are in correct areas
    let area100_neurons = memory_array.get_active_neurons_by_area(100);
    let area200_neurons = memory_array.get_active_neurons_by_area(200);
    let area300_neurons = memory_array.get_active_neurons_by_area(300);
    
    assert_eq!(area100_neurons.len(), 1);
    assert_eq!(area200_neurons.len(), 1);
    assert_eq!(area300_neurons.len(), 1);
}

#[test]
fn test_neuron_id_allocation_integration() {
    let _id_manager = NeuronIdManager::new();
    let mut memory_array = MemoryNeuronArray::new(1000);
    let lifecycle_config = MemoryNeuronLifecycleConfig::default();
    
    // Create multiple memory neurons
    let mut neuron_ids = Vec::new();
    for i in 0..10 {
        let mut pattern_hash = [0u8; 32];
        pattern_hash[0] = i;
        
        let idx = memory_array.create_memory_neuron(pattern_hash, 100, 0, &lifecycle_config).unwrap();
        let neuron_id = memory_array.get_neuron_id(idx).unwrap();
        
        // Verify it's a memory neuron ID
        assert!(NeuronIdManager::is_memory_neuron_id(neuron_id));
        assert!(!NeuronIdManager::is_regular_neuron_id(neuron_id));
        
        neuron_ids.push(neuron_id);
    }
    
    // All IDs should be unique
    let unique_ids: HashSet<_> = neuron_ids.iter().collect();
    assert_eq!(unique_ids.len(), 10);
}

#[test]
fn test_stdp_with_pattern_detection() {
    use feagi_types::NeuronId;
    
    let config = STDPConfig::default();
    
    // Simulate synapses
    let sources = vec![NeuronId(1), NeuronId(2), NeuronId(3)];
    let targets = vec![NeuronId(10), NeuronId(11), NeuronId(12)];
    
    // Simulate firing history
    let source_history = vec![
        (5, NeuronId(1)),
        (6, NeuronId(2)),
        (7, NeuronId(3)),
    ];
    let target_history = vec![
        (6, NeuronId(10)),
        (7, NeuronId(11)),
        (8, NeuronId(12)),
    ];
    
    // Compute timing factors
    let factors = compute_timing_factors(&sources, &targets, &source_history, &target_history, &config);
    
    // All should show potentiation (pre before post)
    assert_eq!(factors.len(), 3);
    for &factor in &factors {
        assert!(factor > 0.0);
    }
}

#[test]
fn test_plasticity_service_basic_workflow() {
    let config = PlasticityConfig::default();
    let service = PlasticityService::new(config);
    
    // Register a memory area
    let success = service.register_memory_area(100, 3, vec![1, 2], None);
    assert!(success);
    
    // Notify of a burst
    service.notify_burst(1);
    
    // Check stats
    let stats = service.get_stats();
    assert_eq!(stats.memory_patterns_detected, 0); // No actual firing history in this test
}

#[test]
fn test_pattern_cache_performance() {
    let mut config = PatternConfig::default();
    config.max_pattern_cache_size = 100;
    let detector = PatternDetector::new(config);
    
    // Create 100 different patterns
    for i in 0..100 {
        let mut bitmap = HashSet::new();
        bitmap.insert(i);
        detector.detect_pattern(100, &vec![1], 10, vec![bitmap], None);
    }
    
    let stats = detector.get_stats();
    assert_eq!(stats.patterns_detected, 100);
    assert_eq!(stats.cache_misses, 100);
    
    // Detect the first pattern again - should be cache hit
    let mut bitmap = HashSet::new();
    bitmap.insert(0);
    detector.detect_pattern(100, &vec![1], 11, vec![bitmap], None);
    
    let stats = detector.get_stats();
    assert_eq!(stats.cache_hits, 1);
}

#[test]
fn test_memory_array_capacity_and_reuse() {
    let mut memory_array = MemoryNeuronArray::new(10);
    let mut lifecycle_config = MemoryNeuronLifecycleConfig::default();
    lifecycle_config.initial_lifespan = 1;
    
    // Fill capacity
    for i in 0..10 {
        let mut pattern_hash = [0u8; 32];
        pattern_hash[0] = i;
        let result = memory_array.create_memory_neuron(pattern_hash, 100, 0, &lifecycle_config);
        assert!(result.is_some());
    }
    
    // Try to exceed capacity
    let pattern_hash = [99u8; 32];
    let result = memory_array.create_memory_neuron(pattern_hash, 100, 0, &lifecycle_config);
    assert!(result.is_none());
    
    // Age neurons to free up space
    memory_array.age_memory_neurons(1);
    
    let stats = memory_array.get_stats();
    assert_eq!(stats.active_neurons, 0);
    assert_eq!(stats.dead_neurons, 10);
    assert_eq!(stats.reusable_indices, 10);
    
    // Now we should be able to create new neurons by reusing indices
    for i in 10..20 {
        let mut pattern_hash = [0u8; 32];
        pattern_hash[0] = i;
        let result = memory_array.create_memory_neuron(pattern_hash, 100, 2, &lifecycle_config);
        assert!(result.is_some());
    }
    
    let stats = memory_array.get_stats();
    assert_eq!(stats.active_neurons, 10);
    assert_eq!(stats.reusable_indices, 0); // All reused
}

#[test]
fn test_deterministic_pattern_hashing_across_runs() {
    let config1 = PatternConfig::default();
    let config2 = PatternConfig::default();
    let detector1 = PatternDetector::new(config1);
    let detector2 = PatternDetector::new(config2);
    
    let mut bitmap = HashSet::new();
    bitmap.insert(1);
    bitmap.insert(2);
    bitmap.insert(3);
    
    // Detect pattern with both detectors
    let pattern1 = detector1.detect_pattern(100, &vec![1], 10, vec![bitmap.clone()], None).unwrap();
    let pattern2 = detector2.detect_pattern(100, &vec![1], 10, vec![bitmap], None).unwrap();
    
    // Same input should produce same hash
    assert_eq!(pattern1.pattern_hash, pattern2.pattern_hash);
}

#[test]
fn test_concurrent_pattern_detection() {
    use std::sync::Arc;
    use std::thread;
    
    let config = PatternConfig::default();
    let detector = Arc::new(PatternDetector::new(config));
    
    let handles: Vec<_> = (0..10).map(|i| {
        let detector = Arc::clone(&detector);
        thread::spawn(move || {
            let mut bitmap = HashSet::new();
            bitmap.insert(i);
            detector.detect_pattern(100, &vec![1], 10, vec![bitmap], None)
        })
    }).collect();
    
    let results: Vec<_> = handles.into_iter()
        .map(|h| h.join().unwrap())
        .collect();
    
    // All should have detected patterns
    assert_eq!(results.iter().filter(|r| r.is_some()).count(), 10);
}

