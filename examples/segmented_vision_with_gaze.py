#!/usr/bin/env python3
"""
Example: Segmented Vision with Gaze Control

This example demonstrates the updated segmented vision processing with advanced
gaze control capabilities using the new feagi_rust_py_libs patterns.

Key features demonstrated:
- SegmentedVisionProcessor with eccentricity and modularity control
- GazeMotorProcessor for dynamic gaze control
- Integration between vision and motor systems
- Using the FeagiAgentConnector with motor functionality
"""

import asyncio
import time
import numpy as np
from feagi_connector import (
    FeagiClient,
    SegmentedVisionProcessor,
    GazeMotorProcessor,
    create_gaze_control_neurons,
    bgr_to_rgb_uint8,
    numpy_to_image_frame
)


async def main():
    print("🎯 Segmented Vision with Gaze Control Example")
    
    # Configuration
    cortical_group = 0
    center_dims = (128, 128)
    peripheral_dims = (64, 64)
    
    # Advanced gaze parameters (from the sample)
    eccentricity = (0.2, 0.2)  # Size of focus area
    modularity = (0.2, 0.2)    # Modulation parameters
    initial_gaze = (0.5, 0.5)  # Center gaze initially
    
    try:
        # 1. Setup segmented vision processor with advanced gaze control
        print("📹 Setting up segmented vision processor...")
        vision_processor = SegmentedVisionProcessor(
            cortical_group_index=cortical_group,
            center_dims=center_dims,
            peripheral_dims=peripheral_dims,
            eccentricity=eccentricity,
            modularity=modularity,
            gaze_position=initial_gaze,
            number_of_channels=1
        )
        
        # 2. Setup gaze motor processor
        print("🎮 Setting up gaze motor processor...")
        gaze_motor = GazeMotorProcessor(
            cortical_group_index=cortical_group,
            num_channels=10,
            gaze_resolution=8
        )
        gaze_motor.register_gaze_motor()
        
        # 3. Create synthetic test images
        print("🖼️ Creating test images...")
        test_images = create_test_images(center_dims)
        
        # 4. Demonstrate vision processing with different gaze positions
        print("🔄 Processing frames with different gaze positions...")
        
        gaze_positions = [
            (0.5, 0.5),  # Center
            (0.3, 0.3),  # Upper left
            (0.7, 0.7),  # Lower right
            (0.2, 0.8),  # Lower left
            (0.8, 0.2),  # Upper right
        ]
        
        for i, gaze_pos in enumerate(gaze_positions):
            print(f"\n📍 Frame {i+1}: Gaze position {gaze_pos}")
            
            # Update gaze position
            vision_processor.update_gaze(gaze_pos[0], gaze_pos[1])
            print(f"   Updated vision processor gaze to: {vision_processor.gaze}")
            
            # Process a test image
            test_image = test_images[i % len(test_images)]
            sensor_bytes = vision_processor.process_frame(test_image)
            
            print(f"   Encoded {len(sensor_bytes)} bytes for FEAGI transmission")
            
            # Create gaze control neurons for motor feedback
            gaze_neurons = create_gaze_control_neurons(
                gaze_pos[0], gaze_pos[1], 
                intensity=1.0, 
                resolution=8
            )
            print(f"   Created gaze control neurons")
            
            # Create mapped neurons for motor system (as shown in sample)
            mapped_neurons = gaze_motor.create_gaze_neurons(
                gaze_pos[0], gaze_pos[1], 
                intensity=1.0
            )
            
            # Convert to bytes for motor processing
            motor_byte_structure = mapped_neurons.as_new_feagi_byte_structure()
            motor_bytes = motor_byte_structure.copy_out_as_byte_vector()
            
            print(f"   Generated {len(motor_bytes)} motor bytes")
            
            # Process motor bytes to extract gaze command
            extracted_gaze = gaze_motor.process_motor_bytes(bytes(motor_bytes))
            if extracted_gaze:
                print(f"   Extracted gaze position: {extracted_gaze}")
            
            # Small delay between frames
            await asyncio.sleep(0.5)
        
        print("\n✅ Segmented vision with gaze control demonstration completed!")
        print("\n📋 Summary:")
        print(f"   - Processed {len(gaze_positions)} frames with different gaze positions")
        print(f"   - Center resolution: {center_dims}")
        print(f"   - Peripheral resolution: {peripheral_dims}")
        print(f"   - Eccentricity: {eccentricity}")
        print(f"   - Modularity: {modularity}")
        print(f"   - Motor channels: {gaze_motor.num_channels}")
        print(f"   - Gaze resolution: {gaze_motor.gaze_resolution}")
        
    except ImportError as e:
        print("❌ feagi_rust_py_libs not available. Install with:")
        print("   pip install feagi-rust-py-libs")
        print(f"   Error: {e}")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()


def create_test_images(size):
    """Create synthetic test images for demonstration."""
    images = []
    width, height = size
    
    # Image 1: Gradient
    img1 = np.zeros((height, width, 3), dtype=np.uint8)
    for i in range(height):
        for j in range(width):
            img1[i, j] = [i * 255 // height, j * 255 // width, 128]
    images.append(img1)
    
    # Image 2: Checkerboard pattern
    img2 = np.zeros((height, width, 3), dtype=np.uint8)
    for i in range(height):
        for j in range(width):
            if (i // 20 + j // 20) % 2 == 0:
                img2[i, j] = [255, 255, 255]
            else:
                img2[i, j] = [0, 0, 0]
    images.append(img2)
    
    # Image 3: Circles
    img3 = np.zeros((height, width, 3), dtype=np.uint8)
    center_x, center_y = width // 2, height // 2
    for i in range(height):
        for j in range(width):
            dist = np.sqrt((i - center_y)**2 + (j - center_x)**2)
            if int(dist) % 20 < 10:
                img3[i, j] = [255, 100, 100]
            else:
                img3[i, j] = [100, 100, 255]
    images.append(img3)
    
    return images


if __name__ == "__main__":
    asyncio.run(main())
