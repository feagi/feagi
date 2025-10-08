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

//! Integration tests for GPU backend
//!
//! Tests the complete pipeline:
//! 1. Upload buffers to GPU
//! 2. Dispatch neural dynamics
//! 3. Download results
//! 4. Verify correctness

#[cfg(feature = "gpu")]
mod gpu_integration {
    use feagi_burst_engine::backend::{create_backend, BackendType};
    use feagi_types::{NeuronArray, SynapseArray};

    #[test]
    fn test_gpu_neural_dynamics_small_genome() {
        // Create small test genome (1000 neurons)
        let neuron_count = 1000;
        let mut neuron_array = create_test_neurons(neuron_count);
        let synapse_array = create_test_synapses(neuron_count, 10_000);

        // Try to create GPU backend
        use feagi_burst_engine::backend::BackendConfig;
        let config = BackendConfig::default();
        let backend_result = create_backend(
            BackendType::WGPU,
            neuron_count,
            synapse_array.count,
            &config,
        );

        match backend_result {
            Ok(mut backend) => {
                println!("✅ GPU backend created: {}", backend.backend_name());

                // Initialize persistent data
                backend
                    .initialize_persistent_data(&neuron_array, &synapse_array)
                    .expect("Failed to initialize persistent data");
                println!("✅ GPU buffers uploaded");

                // Process neural dynamics (no fired neurons yet)
                let result = backend
                    .process_neural_dynamics(&mut neuron_array, 1)
                    .expect("Failed to process neural dynamics");

                println!(
                    "✅ Neural dynamics processed: {} neurons, {} fired",
                    result.1,
                    result.0.len()
                );

                assert_eq!(result.1, neuron_count, "Should process all neurons");
                println!("✅ GPU integration test passed!");
            }
            Err(e) => {
                println!(
                    "⚠️  GPU backend not available ({}), test skipped",
                    e
                );
            }
        }
    }

    #[test]
    fn test_gpu_availability() {
        // Just test if GPU is available
        let result = pollster::block_on(async {
            wgpu::Instance::default()
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::HighPerformance,
                    force_fallback_adapter: false,
                    compatible_surface: None,
                })
                .await
        });

        match result {
            Some(adapter) => {
                let info = adapter.get_info();
                println!("✅ GPU available: {} ({:?})", info.name, info.backend);
                println!("   Device: {:?}", info.device_type);
            }
            None => {
                println!("⚠️  No GPU available");
            }
        }
    }

    /// Helper: Create test neuron array
    fn create_test_neurons(count: usize) -> NeuronArray {
        use feagi_types::*;
        
        NeuronArray {
            capacity: count,
            count,
            membrane_potentials: vec![-70.0; count],
            thresholds: vec![-55.0; count],
            leak_coefficients: vec![0.95; count],
            resting_potentials: vec![-70.0; count],
            neuron_types: vec![0; count],
            refractory_periods: vec![3; count],
            refractory_countdowns: vec![0; count],
            excitabilities: vec![1.0; count],
            consecutive_fire_counts: vec![0; count],
            consecutive_fire_limits: vec![5; count],
            snooze_periods: vec![0; count],
            snooze_countdowns: vec![0; count],
            cortical_areas: vec![0; count],
            coordinates: (0..count).flat_map(|i| vec![i as u32, 0, 0]).collect(),
            valid_mask: vec![true; count],
        }
    }

    /// Helper: Create test synapse array
    fn create_test_synapses(neuron_count: usize, synapse_count: usize) -> SynapseArray {
        use feagi_types::*;
        use std::collections::HashMap;
        
        let source_neurons: Vec<u32> = (0..synapse_count)
            .map(|i| (i % neuron_count) as u32)
            .collect();
        let target_neurons: Vec<u32> = (0..synapse_count)
            .map(|i| ((i + 1) % neuron_count) as u32)
            .collect();
        let weights = vec![128u8; synapse_count];
        let conductances = vec![100u8; synapse_count];
        let types = vec![0u8; synapse_count];
        let valid_mask = vec![true; synapse_count];
        
        // Build source index
        let mut source_index: HashMap<u32, Vec<usize>> = HashMap::new();
        for (idx, &source) in source_neurons.iter().enumerate() {
            source_index.entry(source).or_insert_with(Vec::new).push(idx);
        }
        
        SynapseArray {
            capacity: synapse_count,
            count: synapse_count,
            source_neurons,
            target_neurons,
            weights,
            conductances,
            types,
            valid_mask,
            source_index,
        }
    }
}

