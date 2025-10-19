#!/usr/bin/env python3
"""Test what methods are available on FeagiByteContainer from IOCache"""

import numpy as np
import feagi_rust_py_libs as frpl

# Create IOCache and do a write
cache = frpl.connector_core.caching.IOCache()

# Setup test data
input_res = frpl.connector_core.data.descriptors.ImageXYResolution(128, 128)
center_res = frpl.connector_core.data.descriptors.ImageXYResolution(128, 128)
per_res = frpl.connector_core.data.descriptors.ImageXYResolution(64, 64)

eccentricity = frpl.connector_core.data.Percentage2D(
    frpl.connector_core.data.Percentage.new_from_0_1(0.5),
    frpl.connector_core.data.Percentage.new_from_0_1(0.5)
)
modulation = frpl.connector_core.data.Percentage2D(
    frpl.connector_core.data.Percentage.new_from_0_1(0.7),
    frpl.connector_core.data.Percentage.new_from_0_1(0.7)
)

color_space = frpl.connector_core.data.descriptors.ColorSpace.Linear
color_channels = frpl.connector_core.data.descriptors.ColorChannelLayout.RGB
memory_order = frpl.connector_core.data.descriptors.MemoryOrderLayout.HeightsWidthsChannels

# Create image
test_image = np.ones((128, 128, 3), dtype=np.uint8) * 128
image_frame = frpl.connector_core.data.ImageFrame.new_from_array(test_image, color_space, memory_order)
input_props = image_frame.get_image_frame_properties()

# Create segment props
output_resolutions = frpl.connector_core.data.descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(center_res, per_res)
gaze = frpl.connector_core.data.descriptors.GazeProperties(eccentricity, modulation)
segment_props = frpl.connector_core.data.descriptors.SegmentedImageFrameProperties(
    output_resolutions, color_channels, color_channels, color_space
)

# Register and write
print("Registering...")
cache.sensor_segmented_vision_absolute_try_write(0, 1, input_props, segment_props, gaze)

print("Writing...")
cache.sensor_segmented_vision_absolute_try_write(0, 0, image_frame)

print("Encoding...")
cache.sensors_encode_cached_data_to_bytes()

print("\n=== Getting byte container ===")
container = cache.sensor_get_byte_container()
print(f"Container type: {type(container)}")
print(f"Container: {container}")

print("\n=== Available methods on container ===")
methods = [m for m in dir(container) if not m.startswith('_')]
for method in sorted(methods):
    print(f"  {method}")

print("\n=== Trying to get bytes ===")
# Try different methods
if hasattr(container, 'copy_out_as_byte_vector'):
    print("Trying copy_out_as_byte_vector()...")
    result = container.copy_out_as_byte_vector()
    print(f"  Result: {len(result) if result else 0} bytes")
elif hasattr(container, 'as_bytes'):
    print("Trying as_bytes()...")
    result = container.as_bytes()
    print(f"  Result: {len(result) if result else 0} bytes")
else:
    print("No known byte extraction method found!")
    
# Try to get raw bytes representation
print("\n=== Trying direct conversion ===")
try:
    raw_bytes = bytes(container)
    print(f"bytes(container): {len(raw_bytes)} bytes")
except Exception as e:
    print(f"bytes(container) failed: {e}")

