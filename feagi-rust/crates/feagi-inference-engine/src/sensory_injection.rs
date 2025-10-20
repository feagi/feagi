//! Sensory data injection for FEAGI inference engine
//!
//! Converts image frames to XYZP coordinates and injects them into the FCL
//! using the standard NPU injection methods.

use anyhow::Result;
use feagi_burst_engine::RustNPU;
use feagi_types::NeuronId;
use image::DynamicImage;
use log::{debug, warn};

/// Sensory data injector for video frames
///
/// Converts image pixels to XYZP voxel coordinates where:
/// - X: horizontal pixel position
/// - Y: vertical pixel position  
/// - Z: color channel (0=R, 1=G, 2=B)
/// - P: pixel intensity (0.0-1.0)
pub struct SensoryInjector {
    /// Cortical area ID for vision input (from genome/connectome)
    vision_cortical_area_id: u32,
    /// Frame counter for debugging
    frame_count: u64,
    /// Image dimensions (width, height)
    dimensions: (u32, u32),
    /// Activation threshold (skip dim pixels below this value)
    activation_threshold: f32,
    /// Pixel stride (sample every Nth pixel to reduce load)
    pixel_stride: u32,
}

impl SensoryInjector {
    /// Create a new sensory injector for a specific vision cortical area
    ///
    /// # Arguments
    /// * `vision_cortical_area_name` - Cortical area name from genome (e.g., "ipu_vision")
    /// * `dimensions` - Target image dimensions (width, height)
    pub fn new(vision_cortical_area_name: String, dimensions: (u32, u32)) -> Self {
        // TODO: Properly parse cortical ID from connectome's area_id_to_name mapping
        // For now, use a simple hash or default to 0
        let vision_cortical_area_id = Self::parse_cortical_id(&vision_cortical_area_name);
        
        Self {
            vision_cortical_area_id,
            frame_count: 0,
            dimensions,
            activation_threshold: 0.1, // Skip pixels with intensity < 10%
            pixel_stride: 2, // Sample every 2nd pixel by default
        }
    }

    /// Parse cortical area ID from name
    /// TODO: This should come from the connectome's area_id_to_name mapping
    fn parse_cortical_id(name: &str) -> u32 {
        // Simple hash for now - in production this must come from connectome
        name.chars()
            .map(|c| c as u32)
            .fold(0u32, |acc, c| acc.wrapping_add(c))
            % 1000
    }

    /// Set the cortical area ID directly (from connectome)
    pub fn set_cortical_area_id(&mut self, area_id: u32) {
        self.vision_cortical_area_id = area_id;
    }

    /// Set activation threshold (0.0-1.0)
    pub fn set_activation_threshold(&mut self, threshold: f32) {
        self.activation_threshold = threshold.clamp(0.0, 1.0);
    }

    /// Set pixel sampling stride
    pub fn set_pixel_stride(&mut self, stride: u32) {
        self.pixel_stride = stride.max(1);
    }

    /// Inject a video frame into FEAGI's FCL using XYZP coordinates
    ///
    /// Converts image pixels to XYZP voxel coordinates and uses the NPU's
    /// standard batch_coordinate_lookup and inject_sensory_with_potentials methods.
    pub fn inject_frame(&mut self, npu: &mut RustNPU, frame: &DynamicImage) -> Result<()> {
        self.frame_count += 1;

        // Convert image to RGB8
        let rgb_image = frame.to_rgb8();
        let (width, height) = rgb_image.dimensions();

        // Build XYZP coordinates from image pixels
        // X = horizontal position, Y = vertical position, Z = color channel, P = intensity
        let mut xyzp_data: Vec<(u32, u32, u32, f32)> = Vec::with_capacity(
            ((width * height * 3) / (self.pixel_stride * self.pixel_stride)) as usize
        );

        // Sample pixels with stride
        for y in (0..height).step_by(self.pixel_stride as usize) {
            for x in (0..width).step_by(self.pixel_stride as usize) {
                let pixel = rgb_image.get_pixel(x, y);
                
                // Process each color channel as a separate Z coordinate
                for (channel_idx, &intensity) in pixel.0.iter().enumerate() {
                    let normalized_intensity = intensity as f32 / 255.0;
                    
                    // Skip dim pixels below threshold
                    if normalized_intensity < self.activation_threshold {
                        continue;
                    }

                    // Create XYZP coordinate
                    // X, Y: pixel position
                    // Z: color channel (0=R, 1=G, 2=B)
                    // P: normalized intensity (0.0-1.0, scaled to potential)
                    xyzp_data.push((
                        x,
                        y,
                        channel_idx as u32,
                        normalized_intensity * 100.0, // Scale to reasonable membrane potential
                    ));
                }
            }
        }

        if xyzp_data.is_empty() {
            warn!(
                "Frame {} produced no XYZP coordinates (all pixels below threshold {})",
                self.frame_count, self.activation_threshold
            );
            return Ok(());
        }

        // Extract coordinates for batch lookup
        let coords: Vec<(u32, u32, u32)> = xyzp_data
            .iter()
            .map(|(x, y, z, _)| (*x, *y, *z))
            .collect();

        // Use NPU's batch coordinate lookup to convert XYZP → neuron IDs
        // Note: batch_coordinate_lookup uses filter_map internally, so it only
        // returns valid neuron IDs (coordinates that exist in the voxel grid)
        let neuron_ids = npu.neuron_array.batch_coordinate_lookup(
            self.vision_cortical_area_id,
            &coords
        );

        if neuron_ids.is_empty() {
            warn!(
                "Frame {}: No valid neurons found for {} XYZP coordinates (cortical_area={})",
                self.frame_count,
                coords.len(),
                self.vision_cortical_area_id
            );
            return Ok(());
        }

        // Build (neuron_id, potential) pairs for injection
        // Match each found neuron_id with its corresponding potential from xyzp_data
        let injection_pairs: Vec<(NeuronId, f32)> = neuron_ids
            .into_iter()
            .zip(xyzp_data.iter())
            .map(|(neuron_id, &(_, _, _, potential))| (neuron_id, potential))
            .collect();

        // Inject into FCL using standard NPU method
        npu.inject_sensory_with_potentials(&injection_pairs);

        if self.frame_count % 100 == 0 {
            debug!(
                "Frame {}: {} XYZP coords → {} neurons injected (cortical_area={})",
                self.frame_count,
                coords.len(),
                injection_pairs.len(),
                self.vision_cortical_area_id
            );
        }

        Ok(())
    }

    /// Get the number of frames injected
    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    /// Get the vision cortical area ID
    pub fn cortical_area_id(&self) -> u32 {
        self.vision_cortical_area_id
    }
}

/// Configuration for sensory input
#[derive(Debug, Clone)]
pub struct SensoryConfig {
    /// Cortical area name for vision input
    pub vision_cortical_area: String,
    /// Resize frames to this dimension (optional)
    pub resize_to: Option<(u32, u32)>,
    /// Frame skip (1 = no skip, 2 = every other frame, etc.)
    pub frame_skip: u32,
    /// Pixel sampling stride (1 = every pixel, 2 = every 2nd pixel, etc.)
    pub pixel_stride: u32,
    /// Activation threshold (0.0-1.0)
    pub activation_threshold: f32,
}

impl Default for SensoryConfig {
    fn default() -> Self {
        Self {
            vision_cortical_area: "ipu_vision".to_string(),
            resize_to: Some((64, 64)), // Default to 64x64 for faster processing
            frame_skip: 1,
            pixel_stride: 2, // Sample every 2nd pixel
            activation_threshold: 0.1, // 10% intensity threshold
        }
    }
}

impl SensoryConfig {
    /// Create a new sensory config with specified vision cortical area
    pub fn new(vision_cortical_area: String) -> Self {
        Self {
            vision_cortical_area,
            ..Default::default()
        }
    }

    /// Set frame resize dimensions
    pub fn with_resize(mut self, width: u32, height: u32) -> Self {
        self.resize_to = Some((width, height));
        self
    }

    /// Set frame skip rate
    pub fn with_frame_skip(mut self, skip: u32) -> Self {
        self.frame_skip = skip.max(1);
        self
    }

    /// Set pixel sampling stride
    pub fn with_pixel_stride(mut self, stride: u32) -> Self {
        self.pixel_stride = stride.max(1);
        self
    }

    /// Set activation threshold
    pub fn with_activation_threshold(mut self, threshold: f32) -> Self {
        self.activation_threshold = threshold.clamp(0.0, 1.0);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensory_config_default() {
        let config = SensoryConfig::default();
        assert_eq!(config.vision_cortical_area, "ipu_vision");
        assert_eq!(config.resize_to, Some((64, 64)));
        assert_eq!(config.frame_skip, 1);
        assert_eq!(config.pixel_stride, 2);
        assert_eq!(config.activation_threshold, 0.1);
    }

    #[test]
    fn test_sensory_config_builder() {
        let config = SensoryConfig::new("custom_vision".to_string())
            .with_resize(128, 128)
            .with_frame_skip(2)
            .with_pixel_stride(4)
            .with_activation_threshold(0.2);

        assert_eq!(config.vision_cortical_area, "custom_vision");
        assert_eq!(config.resize_to, Some((128, 128)));
        assert_eq!(config.frame_skip, 2);
        assert_eq!(config.pixel_stride, 4);
        assert_eq!(config.activation_threshold, 0.2);
    }

    #[test]
    fn test_parse_cortical_id() {
        let id1 = SensoryInjector::parse_cortical_id("ipu_vision");
        let id2 = SensoryInjector::parse_cortical_id("ipu_vision");
        let id3 = SensoryInjector::parse_cortical_id("motor_cortex");
        
        // Same name should give same ID
        assert_eq!(id1, id2);
        // Different names should give different IDs (probabilistically)
        assert_ne!(id1, id3);
    }
}
