## Example: Build an ImageFrame and encode to bytes via SensorCache

import asyncio
import time
import numpy as np

from feagi_connector.cache.sensor_cache import SensorCache
import feagi_rust_py_libs as frpl

#agent.sensors.register_camera(1, 1, true, )

async def main():

    resolution_size_input = (1280, 720)
    #center_resolution = (128, 128)
    #peripheral_resolution = (64, 64)

    # X Y resolutions
    resolution = frpl.data_structures.data.image_descriptors.ImageXYResolution(resolution_size_input[0], resolution_size_input[1])

    # Colorspace enum (I suggest just selecting linear for now)
    color_space = frpl.data_structures.data.image_descriptors.ColorSpace.Linear

    # Color channel enum (I suggest RGB)
    color_channels = frpl.data_structures.data.image_descriptors.ColorChannelLayout.RGB

    # How you input array is ordered, different enums are orders of Widths Heights Channels
    memory_order = frpl.data_structures.data.image_descriptors.MemoryOrderLayout.WidthsHeightsChannels

    # Image input, ensure the array is 3 Dimensional, and that dtype=np.float32
    raw_image = np.ones((resolution_size_input[0], resolution_size_input[1], 3), dtype=np.float32) * 1.0

    #converts raw numpy data into an image frame, this func takes np array, color space, memory order
    image_frame = frpl.data_structures.data.ImageFrame.new_from_array(raw_image, color_space, memory_order)

    # Define an image properties, takes in resolution, colorslpace, colorchannels
    image_properties = frpl.data_structures.data.image_descriptors.ImageFrameProperties(resolution, color_space, color_channels)

    #center_res = frpl.data_structures.data.image_descriptors.ImageXYResolution(center_resolution[0], center_resolution[1])
    #per_res = frpl.data_structures.data.image_descriptors.ImageXYResolution(peripheral_resolution[0],peripheral_resolution[1])


    #segmented_resolutions = frpl.data_structures.data.image_descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(center_res, per_res)
    #segmented_properties = frpl.data_structures.data.image_descriptors.SegmentedImageFrameProperties(segmented_resolutions, color_channels, color_channels, color_space)
    #segmented_gaze = frpl.data_structures.data.image_descriptors.GazeProperties.create_default_centered()

    cache = SensorCache()



    # segmented_image_camera

    # cortical group index, number of channels, send stale data, properties of image going in, properties of image being sent to feagi
    cache.image_camera_center.register(1, 1, True, image_properties, image_properties)

    # cortical group index, channel index, sending data (this func stores cache data, doesnt encode anything to bytes or neurons)
    cache.image_camera_center.store(1, 0, image_frame)

    time_start = time.perf_counter()

    # cortical group index, number of channels, send stale data, properties of image going in, properties of segmented image being sent to feagi, gaze
    #agent.sensors.segmented_image_camera.register(0, 1, True, image_properties, segmented_properties, segmented_gaze)


    # converts cached data to neurons -> bytes
    cache.encode_cached_data_into_bytes()


    # retrieve byte data from neurons
    byte_neuron = cache.get_most_recent_sensor_bytes()

    time_end = time.perf_counter()

    #print(data)
    print(time_end - time_start)






if __name__ == "__main__":
    asyncio.run(main())