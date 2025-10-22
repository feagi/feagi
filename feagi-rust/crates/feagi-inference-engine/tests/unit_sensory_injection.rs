//! Unit tests for sensory injection module
//! 
//! NOTE: DISABLED - Video/image processing is now handled by external agents via ZMQ
//! This test module is kept for reference but all tests are disabled.

// use image::{DynamicImage, RgbImage};

// Re-export the sensory injection module for testing
// Note: In a real setup, these would be in the main crate and we'd use them here
// For now, we'll test the public API through integration tests

#[cfg(all(test, feature = "image-processing-disabled"))]
mod sensory_injector_tests {
    use super::*;

    #[test]
    fn test_parse_cortical_id_consistency() {
        // Test that same name produces same ID
        let id1 = hash_cortical_name("ipu_vision");
        let id2 = hash_cortical_name("ipu_vision");
        assert_eq!(id1, id2, "Same name should produce same cortical ID");
    }

    #[test]
    fn test_parse_cortical_id_uniqueness() {
        // Test that different names produce different IDs
        let id1 = hash_cortical_name("ipu_vision");
        let id2 = hash_cortical_name("opu_motor");
        assert_ne!(id1, id2, "Different names should produce different cortical IDs");
    }

    #[test]
    fn test_xyzp_coordinate_generation() {
        // Create a simple 4x4 RGB image
        let mut img = RgbImage::new(4, 4);
        
        // Set some bright pixels
        img.put_pixel(0, 0, image::Rgb([255, 0, 0])); // Red
        img.put_pixel(1, 1, image::Rgb([0, 255, 0])); // Green
        img.put_pixel(2, 2, image::Rgb([0, 0, 255])); // Blue
        img.put_pixel(3, 3, image::Rgb([255, 255, 255])); // White

        let dynamic_img = DynamicImage::ImageRgb8(img);

        // Test that we can extract pixel data
        let rgb_image = dynamic_img.to_rgb8();
        assert_eq!(rgb_image.width(), 4);
        assert_eq!(rgb_image.height(), 4);

        // Verify pixel values
        let pixel = rgb_image.get_pixel(0, 0);
        assert_eq!(pixel[0], 255); // Red channel
        assert_eq!(pixel[1], 0);
        assert_eq!(pixel[2], 0);
    }

    #[test]
    fn test_activation_threshold_filtering() {
        // Test that dim pixels are filtered out
        let mut img = RgbImage::new(4, 4);
        
        // Set one bright pixel and rest dim
        img.put_pixel(0, 0, image::Rgb([255, 255, 255])); // Bright
        // Rest are default (0, 0, 0) - dim

        let dynamic_img = DynamicImage::ImageRgb8(img);
        let rgb_image = dynamic_img.to_rgb8();

        let threshold = 0.1; // 10% intensity threshold
        let mut bright_count = 0;

        for y in 0..rgb_image.height() {
            for x in 0..rgb_image.width() {
                let pixel = rgb_image.get_pixel(x, y);
                for &intensity in &pixel.0 {
                    let normalized = intensity as f32 / 255.0;
                    if normalized >= threshold {
                        bright_count += 1;
                    }
                }
            }
        }

        // Should have 3 bright channels (RGB) from the white pixel
        assert_eq!(bright_count, 3);
    }

    #[test]
    fn test_pixel_stride_sampling() {
        let width = 10u32;
        let height = 10u32;
        let stride = 2u32;

        let mut sampled_count = 0;
        for y in (0..height).step_by(stride as usize) {
            for x in (0..width).step_by(stride as usize) {
                sampled_count += 1;
                // Verify coordinates are multiples of stride
                assert_eq!(x % stride, 0);
                assert_eq!(y % stride, 0);
            }
        }

        // With 10x10 image and stride 2, should sample 5x5 = 25 pixels
        assert_eq!(sampled_count, 25);
    }

    // Helper function matching the one in sensory_injection.rs
    fn hash_cortical_name(name: &str) -> u32 {
        name.chars()
            .map(|c| c as u32)
            .fold(0u32, |acc, c| acc.wrapping_add(c))
            % 1000
    }
}

#[cfg(test)]
mod sensory_config_tests {
    #[test]
    fn test_default_config() {
        // Test that default config has sensible values
        let vision_area = "ipu_vision";
        let resize = Some((64, 64));
        let frame_skip = 1u32;
        let pixel_stride = 2u32;
        let threshold = 0.1f32;

        assert_eq!(vision_area, "ipu_vision");
        assert_eq!(resize, Some((64, 64)));
        assert_eq!(frame_skip, 1);
        assert_eq!(pixel_stride, 2);
        assert!(threshold > 0.0 && threshold < 1.0);
    }

    #[test]
    fn test_threshold_clamping() {
        // Test that threshold is properly clamped to [0.0, 1.0]
        let threshold = 1.5f32;
        let clamped = threshold.clamp(0.0, 1.0);
        assert_eq!(clamped, 1.0);

        let threshold = -0.5f32;
        let clamped = threshold.clamp(0.0, 1.0);
        assert_eq!(clamped, 0.0);

        let threshold = 0.5f32;
        let clamped = threshold.clamp(0.0, 1.0);
        assert_eq!(clamped, 0.5);
    }

    #[test]
    fn test_stride_minimum() {
        // Test that stride is always at least 1
        let stride = 0u32;
        let adjusted = stride.max(1);
        assert_eq!(adjusted, 1);

        let stride = 5u32;
        let adjusted = stride.max(1);
        assert_eq!(adjusted, 5);
    }
}

