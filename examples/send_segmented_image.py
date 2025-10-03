## Demonstrates registering and sending a segmented (3x3) image using SensorCache

import asyncio
import numpy as np

from feagi_connector.cache.sensor_cache import SensorCache
import feagi_rust_py_libs as frpl


async def main():
    cache = SensorCache()

    # Input frame properties
    in_w, in_h = 640, 360
    input_res = frpl.data_structures.data.image_descriptors.ImageXYResolution(in_w, in_h)
    color_space = frpl.data_structures.data.image_descriptors.ColorSpace.Linear
    color_channels = frpl.data_structures.data.image_descriptors.ColorChannelLayout.RGB
    input_props = frpl.data_structures.data.image_descriptors.ImageFrameProperties(
        input_res, color_space, color_channels
    )

    # Target segmented properties (center and peripheral resolutions) and gaze
    center_res = frpl.data_structures.data.image_descriptors.ImageXYResolution(128, 128)
    per_res = frpl.data_structures.data.image_descriptors.ImageXYResolution(64, 64)
    seg_resolutions = frpl.data_structures.data.image_descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
        center_res, per_res
    )
    seg_props = frpl.data_structures.data.image_descriptors.SegmentedImageFrameProperties(
        seg_resolutions, color_channels, color_channels, color_space
    )
    gaze = frpl.data_structures.data.image_descriptors.GazeProperties.create_default_centered()

    # Build a demo frame (uint8), using WidthsHeightsChannels memory order as expected below
    raw = np.full((in_w, in_h, 3), 200, dtype=np.uint8)
    memory_order = frpl.data_structures.data.image_descriptors.MemoryOrderLayout.WidthsHeightsChannels
    frame = frpl.data_structures.data.ImageFrame.new_from_array(raw, color_space, memory_order)

    # Register segmented camera (group 0, 1 channel)
    cache.segmented_image_camera.register(0, 1, True, input_props, seg_props, gaze)
    # Store one frame for channel 0
    cache.segmented_image_camera.store(0, 0, frame)

    # Encode cached data and print size
    cache.encode_cached_data_into_bytes()
    b = cache.get_most_recent_sensor_bytes()
    print(f"Segmented encoded bytes: {len(b)}")


if __name__ == "__main__":
    asyncio.run(main())


