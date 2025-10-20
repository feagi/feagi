//! Common test utilities and helpers

use std::path::PathBuf;
use tempfile::TempDir;

/// Create a temporary directory for test files
pub fn create_temp_dir() -> TempDir {
    tempfile::tempdir().expect("Failed to create temp directory")
}

/// Get path to test fixtures directory
pub fn fixtures_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests").join("fixtures")
}

/// Create a minimal test connectome for integration tests
pub fn create_minimal_test_connectome() -> feagi_connectome_serialization::ConnectomeSnapshot {
    use feagi_connectome_serialization::{
        ConnectomeSnapshot, SerializableNeuronArray, SerializableSynapseArray,
        ConnectomeMetadata,
    };
    use ahash::AHashMap;

    const FORMAT_VERSION: u32 = 1; // Match the crate's version

    let neurons = SerializableNeuronArray {
        count: 100,
        capacity: 100,
        membrane_potentials: vec![0.0; 100],
        thresholds: vec![50.0; 100],
        leak_coefficients: vec![0.1; 100],
        resting_potentials: vec![0.0; 100],
        neuron_types: vec![0; 100],
        refractory_periods: vec![2; 100],
        refractory_countdowns: vec![0; 100],
        excitabilities: vec![1.0; 100],
        cortical_areas: vec![0; 100], // All in area 0 for simplicity
        coordinates: vec![0; 300], // 100 neurons * 3 coords (x,y,z)
        valid_mask: vec![true; 100],
    };

    let synapses = SerializableSynapseArray {
        count: 200,
        capacity: 200,
        source_neurons: (0..200).map(|i| (i % 100) as u32).collect(),
        target_neurons: (0..200).map(|i| ((i + 1) % 100) as u32).collect(),
        weights: vec![10; 200],
        conductances: vec![5; 200],
        types: vec![0; 200], // All excitatory
        valid_mask: vec![true; 200],
        source_index: std::collections::HashMap::new(),
    };

    let mut cortical_area_names = AHashMap::new();
    cortical_area_names.insert(0, "test_area".to_string());

    ConnectomeSnapshot {
        version: FORMAT_VERSION,
        neurons,
        synapses,
        cortical_area_names,
        burst_count: 0,
        power_amount: 1.0,
        fire_ledger_window: 20,
        metadata: ConnectomeMetadata::default(),
    }
}

/// Assert that two f32 values are approximately equal
pub fn assert_approx_eq(a: f32, b: f32, epsilon: f32) {
    assert!(
        (a - b).abs() < epsilon,
        "Expected {} ≈ {}, but difference was {}",
        a,
        b,
        (a - b).abs()
    );
}

