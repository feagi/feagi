//! Motor output extraction for FEAGI inference engine
//!
//! Extracts motor cortical area activations from the fire queue and
//! encodes them using feagi-data-processing structures.

use anyhow::Result;
use feagi_burst_engine::RustNPU;
use feagi_data_serialization::FeagiSerializable;
use feagi_data_structures::neuron_voxels::xyzp::{CorticalMappedXYZPNeuronVoxels, NeuronVoxelXYZPArrays};
use feagi_data_structures::genomic::CorticalID;
use log::{debug, info};
use std::collections::HashMap;

/// Motor output extractor
///
/// Extracts firing neurons from motor cortical areas and encodes them
/// into the standard FEAGI data format for motor control.
pub struct MotorExtractor {
    /// Motor cortical area IDs (from genome/connectome)
    /// Maps area name -> area ID
    motor_areas: HashMap<String, u32>,
    /// Total motor outputs extracted
    total_extractions: u64,
    /// Enable verbose output printing
    verbose: bool,
}

impl MotorExtractor {
    /// Create a new motor extractor
    ///
    /// # Arguments
    /// * `motor_area_names` - Names of motor cortical areas (e.g., ["opu_motor_left", "opu_motor_right"])
    pub fn new(motor_area_names: Vec<String>) -> Self {
        // TODO: Get actual cortical IDs from connectome's area_id_to_name mapping
        let mut motor_areas = HashMap::new();
        for name in motor_area_names.iter() {
            // Simple hash for now - in production this must come from connectome
            let area_id = Self::parse_cortical_id(name);
            motor_areas.insert(name.clone(), area_id);
        }

        Self {
            motor_areas,
            total_extractions: 0,
            verbose: true,
        }
    }

    /// Parse cortical area ID from name (placeholder)
    /// TODO: This should come from the connectome's area_id_to_name mapping
    fn parse_cortical_id(name: &str) -> u32 {
        // Simple hash for now - in production this must come from connectome
        name.chars()
            .map(|c| c as u32)
            .fold(0u32, |acc, c| acc.wrapping_add(c))
            % 1000
    }

    /// Set motor area IDs from connectome
    pub fn set_motor_areas(&mut self, motor_areas: HashMap<String, u32>) {
        self.motor_areas = motor_areas;
    }

    /// Set verbose output
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose;
    }

    /// Extract motor output from NPU's fire queue
    ///
    /// Gets the current fire queue data, filters for motor cortical areas,
    /// and encodes the XYZP coordinates using feagi-data-processing.
    pub fn extract_motor_output(&mut self, npu: &mut RustNPU) -> Result<Option<Vec<u8>>> {
        self.total_extractions += 1;

        // Get fire queue data (area_id -> (ids, xs, ys, zs, ps))
        let fire_data = match npu.force_sample_fire_queue() {
            Some(data) => data,
            None => {
                debug!("Fire queue is None");
                return Ok(None);
            }
        };

        if fire_data.is_empty() {
            debug!("Fire queue is empty");
            return Ok(None);
        }

        // Build CorticalMappedXYZPNeuronVoxels for motor output
        let mut cortical_mapped = CorticalMappedXYZPNeuronVoxels::new();
        let mut total_motor_neurons = 0usize;

        // Extract motor cortical areas from fire queue
        for (area_id, (id_vec, x_vec, y_vec, z_vec, p_vec)) in fire_data.iter() {
            // Check if this is a motor area
            let area_name = npu.get_cortical_area_name(*area_id);
            let is_motor_area = if let Some(name) = area_name {
                self.motor_areas.contains_key(name)
            } else {
                false
            };

            if !is_motor_area {
                continue;
            }

            let neuron_count = id_vec.len();
            if neuron_count == 0 {
                continue;
            }

            total_motor_neurons += neuron_count;

            // Create CorticalID from area name
            // CorticalID expects exactly 6 bytes
            let area_name_str = area_name.unwrap_or("unknown");
            let area_bytes = area_name_str.as_bytes();
            let mut cortical_id_bytes = [0u8; 6]; // CorticalID::CORTICAL_ID_LENGTH
            let copy_len = area_bytes.len().min(6);
            cortical_id_bytes[..copy_len].copy_from_slice(&area_bytes[..copy_len]);
            
            let cortical_id = match CorticalID::from_bytes(&cortical_id_bytes) {
                Ok(id) => id,
                Err(e) => {
                    debug!("Failed to create CorticalID for '{}': {:?}", area_name_str, e);
                    continue;
                }
            };

            // Create NeuronVoxelXYZPArrays from the vectors
            match NeuronVoxelXYZPArrays::new_from_vectors(
                x_vec.clone(),
                y_vec.clone(),
                z_vec.clone(),
                p_vec.clone(),
            ) {
                Ok(neuron_arrays) => {
                    cortical_mapped.mappings.insert(cortical_id, neuron_arrays);

                    // Print motor output if verbose
                    if self.verbose && self.total_extractions % 10 == 0 {
                        let area_name_str = area_name.unwrap_or("unknown");
                        info!(
                            "🎮 Motor Output #{}: area='{}' (id={}), {} neurons firing",
                            self.total_extractions, area_name_str, area_id, neuron_count
                        );

                        // Print first few neurons for debugging
                        let sample_size = neuron_count.min(5);
                        for i in 0..sample_size {
                            info!(
                                "   Neuron[{}]: ({}, {}, {}) power={:.2}",
                                i, x_vec[i], y_vec[i], z_vec[i], p_vec[i]
                            );
                        }
                        if neuron_count > sample_size {
                            info!("   ... and {} more neurons", neuron_count - sample_size);
                        }
                    }
                }
                Err(e) => {
                    debug!("Failed to create NeuronVoxelXYZPArrays for area {}: {:?}", area_id, e);
                    continue;
                }
            }
        }

        if total_motor_neurons == 0 {
            return Ok(None);
        }

        // Serialize to bytes using feagi_data_serialization
        let mut buffer = Vec::with_capacity(8192);
        cortical_mapped
            .try_serialize_struct_to_byte_slice(&mut buffer)
            .map_err(|e| anyhow::anyhow!("Failed to serialize motor output: {:?}", e))?;

        if self.verbose && self.total_extractions % 10 == 0 {
            info!(
                "📤 Motor output serialized: {} bytes, {} neurons across {} areas",
                buffer.len(),
                total_motor_neurons,
                cortical_mapped.mappings.len()
            );
        }

        Ok(Some(buffer))
    }

    /// Get total number of extractions performed
    pub fn total_extractions(&self) -> u64 {
        self.total_extractions
    }

    /// Get motor area count
    pub fn motor_area_count(&self) -> usize {
        self.motor_areas.len()
    }
}

/// Configuration for motor output
#[derive(Debug, Clone)]
pub struct MotorConfig {
    /// Motor cortical area names
    pub motor_areas: Vec<String>,
    /// Enable verbose output printing
    pub verbose: bool,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            motor_areas: vec!["opu_motor".to_string()],
            verbose: true,
        }
    }
}

impl MotorConfig {
    /// Create a new motor config with specified motor areas
    pub fn new(motor_areas: Vec<String>) -> Self {
        Self {
            motor_areas,
            verbose: true,
        }
    }

    /// Set verbose output
    pub fn with_verbose(mut self, verbose: bool) -> Self {
        self.verbose = verbose;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_config_default() {
        let config = MotorConfig::default();
        assert_eq!(config.motor_areas, vec!["opu_motor"]);
        assert_eq!(config.verbose, true);
    }

    #[test]
    fn test_motor_config_builder() {
        let config = MotorConfig::new(vec!["motor_left".to_string(), "motor_right".to_string()])
            .with_verbose(false);

        assert_eq!(config.motor_areas.len(), 2);
        assert_eq!(config.verbose, false);
    }

    #[test]
    fn test_parse_cortical_id() {
        let id1 = MotorExtractor::parse_cortical_id("opu_motor");
        let id2 = MotorExtractor::parse_cortical_id("opu_motor");
        let id3 = MotorExtractor::parse_cortical_id("other_motor");
        
        // Same name should give same ID
        assert_eq!(id1, id2);
        // Different names should give different IDs (probabilistically)
        assert_ne!(id1, id3);
    }
}

