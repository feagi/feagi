#!/usr/bin/env python3
"""
Test script for the rust_core_sensiomotor_functions_py module.
This script tests the ImageDiff class and its methods.
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from rust_core_sensiomotor_functions_py import ImageDiff

def test_basic_functionality():
    """Test basic functionality of the ImageDiff class."""
    # Define image dimensions
    width = 640
    height = 480
    channels = 3  # RGB

    # Create an ImageDiff instance
    image_diff = ImageDiff(width, height, channels)
    
    # Check shape
    shape = image_diff.shape()
    print(f"Image shape: {shape}")
    assert shape == (height, width, channels), f"Expected shape {(height, width, channels)}, got {shape}"
    
    # Create a test image (random data)
    random_image = np.random.randint(0, 255, (height, width, channels), dtype=np.uint8)
    
    # Get delta from new frame
    delta = image_diff.get_new_delta_from_new_frame(random_image)
    
    # Print delta shape
    print(f"Delta shape: {delta.shape}")
    
    # Create a second random image
    random_image2 = np.random.randint(0, 255, (height, width, channels), dtype=np.uint8)
    
    # Get delta for second image
    delta2 = image_diff.get_new_delta_from_new_frame(random_image2)
    
    # Print delta2 shape
    print(f"Delta2 shape: {delta2.shape}")
    
    return random_image, delta, random_image2, delta2

def test_performance():
    """Test performance of the ImageDiff class."""
    # Define image dimensions
    width = 640
    height = 480
    channels = 3  # RGB

    # Create an ImageDiff instance
    image_diff = ImageDiff(width, height, channels)
    
    # Create a test image (random data)
    random_image = np.random.randint(0, 255, (height, width, channels), dtype=np.uint8)
    
    # Measure performance
    num_iterations = 100
    start_time = time.time()
    
    for _ in range(num_iterations):
        _ = image_diff.get_new_delta_from_new_frame(random_image)
    
    end_time = time.time()
    
    execution_time = end_time - start_time
    print(f"Total execution time for {num_iterations} iterations: {execution_time:.4f} seconds")
    print(f"Average execution time per iteration: {execution_time / num_iterations:.4f} seconds")

def visualize_results(original, delta):
    """Visualize the original image and the delta."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    
    ax1.imshow(original)
    ax1.set_title('Original Image')
    ax1.axis('off')
    
    # For visualization purposes, we may need to scale the delta
    # since it might contain very small values
    if delta.max() > 0:
        delta_normalized = (delta / delta.max() * 255).astype(np.uint8)
    else:
        delta_normalized = delta
    
    ax2.imshow(delta_normalized)
    ax2.set_title('Delta Image')
    ax2.axis('off')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("Testing ImageDiff functionality...")
    
    try:
        # Test basic functionality
        original1, delta1, original2, delta2 = test_basic_functionality()
        
        # Visualize results
        print("\nVisualizing results...")
        visualize_results(original1, delta1)
        visualize_results(original2, delta2)
        
        # Test performance
        print("\nTesting performance...")
        test_performance()
        
        print("\nAll tests completed successfully!")
    except Exception as e:
        print(f"Error during testing: {e}") 