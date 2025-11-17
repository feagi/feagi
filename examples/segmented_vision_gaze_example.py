"""
Example: Segmented Vision with Gaze Control using feagi-rust-py-libs

This example demonstrates the new API for segmented vision processing with gaze control.
It shows how to:
1. Create image frames using Rust data structures
2. Register segmented vision sensors
3. Register gaze motors
4. Write image data to sensors
5. Access encoded byte containers

Requirements:
    pip install feagi-connector[rust,video]

Based on: archive_del/feagi-connector/Python_Connector/sample/segmented_autogaze.py
"""

import asyncio
import numpy as np
from feagi_connector import FeagiAgentConnector
import feagi_rust_py_libs as frpl


async def main():
    # Configuration variables
    resolution_size_input = (1280, 720)
    resolution_output_peripheral = (64, 64)
    resolution_output_center = (128, 128)
    
    # Gaze parameters (size of central region and peripheral tiling)
    eccentricity = frpl.connector_core.data.Percentage2D(
        frpl.connector_core.data.Percentage.new_from_0_1(0.2),
        frpl.connector_core.data.Percentage.new_from_0_1(0.2),
    )
    
    modulation = frpl.connector_core.data.Percentage2D(
        frpl.connector_core.data.Percentage.new_from_0_1(0.2),
        frpl.connector_core.data.Percentage.new_from_0_1(0.2),
    )
    
    cortical_group = 0
    number_of_channels = 1
    number_z_neurons = 10

    # Create resolution descriptors using Rust types
    input_image_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(
        resolution_size_input[0],
        resolution_size_input[1]
    )

    output_center_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(
        resolution_output_center[0],
        resolution_output_center[1]
    )

    output_peripheral_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(
        resolution_output_peripheral[0],
        resolution_output_peripheral[1]
    )

    # Create image frames using Rust ImageFrame
    # NOTE: NumPy arrays are (height, width, channels), so use HeightsWidthsChannels
    color_space = frpl.connector_core.data.descriptors.ColorSpace.Linear
    memory_order = frpl.connector_core.data.descriptors.MemoryOrderLayout.HeightsWidthsChannels
    
    # Create a test image (grayscale value 15 for first frame)
    test_image_data = np.ones(
        (resolution_size_input[0], resolution_size_input[1], 3), 
        dtype=np.uint8
    ) * 15
    
    image_frame_1 = frpl.connector_core.data.ImageFrame.new_from_array(
        test_image_data,
        color_space,
        memory_order
    )
    
    input_image_properties = image_frame_1.get_image_frame_properties()
    color_channel_layout = image_frame_1.channel_layout

    # Create segmented image properties
    output_resolutions = frpl.connector_core.data.descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
        output_center_resolution,
        output_peripheral_resolution
    )
    
    gaze = frpl.connector_core.data.descriptors.GazeProperties(
        eccentricity,
        modulation
    )
    
    segment_properties = frpl.connector_core.data.descriptors.SegmentedImageFrameProperties(
        output_resolutions,
        color_channel_layout,
        color_channel_layout,
        color_space
    )

    # Create connector instance
    agent = FeagiAgentConnector.create_dummy_connector()

    # Connect to FEAGI (dummy connection for this example)
    await agent.server.connect()

    # Register segmented vision sensor with new API
    print(f"Registering segmented vision sensor...")
    agent.sensors.image_camera_with_peripheral.register(
        cortical_group,
        number_of_channels,
        input_image_properties,
        segment_properties,
        gaze
    )

    # Register gaze motor
    print(f"Registering gaze motor...")
    agent.motors.gaze.register(
        cortical_group,
        number_of_channels,
        number_z_neurons
    )

    # Write image data to sensor
    print(f"Writing image frame to sensor...")
    agent.sensors.image_camera_with_peripheral.write(
        cortical_group,
        0,  # device_channel
        image_frame_1
    )

    # Encode cached data into bytes (automatic with new API but explicit call for clarity)
    agent.sensors.encode_cached_data_into_bytes()

    # Get the byte container with structured access
    print(f"Retrieving byte container...")
    byte_container = agent.sensor_get_byte_container()
    
    if byte_container:
        number_contained_structs = byte_container.number_contained_structures
        print(f"Byte container contains {number_contained_structs} structure(s)")
        
        # Extract the first neuron data structure
        if number_contained_structs > 0:
            neuron_data = byte_container.try_create_new_struct_from_index(0)
            print(f"Successfully extracted neuron data structure at index 0")
            print(f"Neuron data type: {type(neuron_data)}")
    else:
        print("Warning: byte_container is None (feagi-rust-py-libs may not be installed)")

    # You can also get raw bytes
    raw_bytes = agent.get_most_recent_sensor_bytes()
    print(f"Raw sensor data size: {len(raw_bytes)} bytes")

    print("\nSegmented vision with gaze control example completed successfully!")
    print("\nKey API methods demonstrated:")
    print("  - agent.sensors.image_camera_with_peripheral.register()")
    print("  - agent.sensors.image_camera_with_peripheral.write()")
    print("  - agent.motors.gaze.register()")
    print("  - agent.sensor_get_byte_container()")


if __name__ == "__main__":
    asyncio.run(main())

