#!/usr/bin/env python3
"""Test the actual SensorCache flow"""

import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG)

from feagi_connector.cache.sensor_cache import SensorCache
import feagi_rust_py_libs as frpl

# Create cache
cache = SensorCache()
print(f"Cache type: {type(cache._rust_cache)}")

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

# Register
print("\n=== Registering ===")
cache.image_camera_with_peripheral.register(0, 1, input_props, segment_props, gaze)

# Write
print("\n=== Writing ===")
cache.image_camera_with_peripheral.write(0, 0, image_frame)

# Encode
print("\n=== Encoding ===")
cache.encode_cached_data_into_bytes()

# Get bytes
print("\n=== Getting bytes ===")
sensor_bytes = cache.get_most_recent_sensor_bytes()
print(f"Result: {len(sensor_bytes)} bytes")

if len(sensor_bytes) == 0:
    print("\n=== DEBUG: Trying manual access ===")
    container = cache._rust_cache.sensor_get_byte_container()
    print(f"Container: {container}")
    if container:
        raw = container.copy_out_as_byte_vector()
        print(f"Raw bytes from container: {len(raw)} items")
        print(f"Type of raw: {type(raw)}")
        converted = bytes(raw)
        print(f"Converted to bytes: {len(converted)} bytes")

