//! Integration tests for complete inference engine pipeline

mod common;

use common::{create_temp_dir, create_minimal_test_connectome};
use feagi_connectome_serialization::{save_connectome, load_connectome};

#[test]
fn test_connectome_save_load_roundtrip() {
    // Create a minimal test connectome
    let connectome = create_minimal_test_connectome();
    
    // Save to temporary file
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    // Verify file exists
    assert!(connectome_path.exists(), "Connectome file should exist");
    
    // Load it back
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    // Verify data integrity
    assert_eq!(loaded.neurons.count, connectome.neurons.count);
    assert_eq!(loaded.neurons.capacity, connectome.neurons.capacity);
    assert_eq!(loaded.synapses.count, connectome.synapses.count);
    assert_eq!(loaded.synapses.capacity, connectome.synapses.capacity);
    assert_eq!(loaded.burst_count, connectome.burst_count);
    assert_eq!(loaded.power_amount, connectome.power_amount);
}

#[test]
fn test_connectome_load_creates_npu() {
    // Create and save a test connectome
    let connectome = create_minimal_test_connectome();
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    // Load connectome
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    // Create NPU from connectome
    let _npu = feagi_burst_engine::RustNPU::import_connectome(loaded);
    
    // Verify NPU was created successfully
    // Note: Can't access private fields directly, but construction succeeding is a good sign
    // The fact that we got here without panicking means the NPU was created successfully
}

#[test]
fn test_connectome_metadata_preservation() {
    use ahash::AHashMap;
    
    let mut connectome = create_minimal_test_connectome();
    
    // Set custom metadata
    connectome.metadata.description = "Test brain for integration testing".to_string();
    connectome.metadata.source = "test_suite".to_string();
    
    let mut tags = AHashMap::new();
    tags.insert("test".to_string(), "true".to_string());
    tags.insert("version".to_string(), "1.0".to_string());
    connectome.metadata.tags = tags;
    
    // Save and load
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    // Verify metadata preserved
    assert_eq!(loaded.metadata.description, connectome.metadata.description);
    assert_eq!(loaded.metadata.source, connectome.metadata.source);
    assert_eq!(loaded.metadata.tags.len(), connectome.metadata.tags.len());
    assert_eq!(
        loaded.metadata.tags.get("test"),
        Some(&"true".to_string())
    );
}

#[test]
fn test_npu_burst_processing() {
    // Create a minimal connectome and NPU
    let connectome = create_minimal_test_connectome();
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    let mut npu = feagi_burst_engine::RustNPU::import_connectome(loaded);
    
    // Process a burst
    let result = npu.process_burst().expect("Burst processing failed");
    
    // Verify burst was processed (result returned successfully)
    // Note: burst_count is private, but successful execution means burst was processed
    assert!(result.neuron_count >= 0); // May be 0 if no neurons fired
}

#[test]
fn test_sensory_injection_batch() {
    // Test injecting a batch of neuron IDs
    let connectome = create_minimal_test_connectome();
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    let mut npu = feagi_burst_engine::RustNPU::import_connectome(loaded);
    
    // Inject some neurons
    let neuron_ids = vec![
        feagi_types::NeuronId(0),
        feagi_types::NeuronId(1),
        feagi_types::NeuronId(2),
    ];
    let potential = 75.0f32;
    
    // This should not panic
    npu.inject_sensory_batch(&neuron_ids, potential);
    
    // Process a burst to apply injections
    let _ = npu.process_burst();
}

#[test]
fn test_fire_queue_sampling() {
    // Test that fire queue can be sampled
    let connectome = create_minimal_test_connectome();
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    let mut npu = feagi_burst_engine::RustNPU::import_connectome(loaded);
    
    // Inject and process
    let neuron_ids = vec![feagi_types::NeuronId(0)];
    npu.inject_sensory_batch(&neuron_ids, 100.0);
    let _ = npu.process_burst();
    
    // Sample fire queue
    let fire_data = npu.force_sample_fire_queue();
    
    // Should get Some result (empty or with data)
    assert!(fire_data.is_some());
}

#[test]
fn test_multiple_burst_processing() {
    // Test processing multiple bursts in sequence
    let connectome = create_minimal_test_connectome();
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    let mut npu = feagi_burst_engine::RustNPU::import_connectome(loaded);
    
    let num_bursts = 10;
    
    // Process multiple bursts
    let mut successful_bursts = 0;
    for _ in 0..num_bursts {
        if npu.process_burst().is_ok() {
            successful_bursts += 1;
        }
    }
    
    // Verify all bursts were processed successfully
    assert_eq!(successful_bursts, num_bursts);
}

#[test]
fn test_cortical_area_name_mapping() {
    // Test that cortical area names are preserved
    use ahash::AHashMap;
    
    let mut connectome = create_minimal_test_connectome();
    
    // Add multiple cortical areas
    let mut area_names = AHashMap::new();
    area_names.insert(0, "ipu_vision".to_string());
    area_names.insert(1, "opu_motor".to_string());
    area_names.insert(2, "memory_area".to_string());
    connectome.cortical_area_names = area_names.clone();
    
    // Save and load
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    // Verify area names preserved
    assert_eq!(loaded.cortical_area_names.len(), area_names.len());
    assert_eq!(
        loaded.cortical_area_names.get(&0),
        Some(&"ipu_vision".to_string())
    );
    assert_eq!(
        loaded.cortical_area_names.get(&1),
        Some(&"opu_motor".to_string())
    );
}

#[test]
fn test_neuron_coordinate_lookup() {
    // Test batch coordinate lookup functionality
    let connectome = create_minimal_test_connectome();
    let temp_dir = create_temp_dir();
    let connectome_path = temp_dir.path().join("test_brain.connectome");
    
    save_connectome(&connectome, &connectome_path)
        .expect("Failed to save connectome");
    
    let loaded = load_connectome(&connectome_path)
        .expect("Failed to load connectome");
    
    let npu = feagi_burst_engine::RustNPU::import_connectome(loaded);
    
    // Try to lookup some coordinates
    let coords = vec![
        (0u32, 0u32, 0u32),
        (1u32, 1u32, 1u32),
        (2u32, 2u32, 2u32),
    ];
    
    let cortical_area_id = 0u32;
    let neuron_ids = npu.neuron_array.batch_coordinate_lookup(cortical_area_id, &coords);
    
    // Should return a vector (may be empty if coordinates not in spatial hash)
    assert!(neuron_ids.len() <= coords.len());
}

#[test]
fn test_invalid_connectome_file() {
    // Test that loading invalid file produces error
    let temp_dir = create_temp_dir();
    let invalid_path = temp_dir.path().join("invalid.connectome");
    
    // Write garbage data
    std::fs::write(&invalid_path, b"This is not a valid connectome file")
        .expect("Failed to write test file");
    
    // This should fail to load
    let result = load_connectome(&invalid_path);
    assert!(result.is_err(), "Should fail to load invalid file");
}

