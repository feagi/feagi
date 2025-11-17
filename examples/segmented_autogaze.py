import asyncio
import time
import numpy as np
from src.agent_connector import FeagiAgentConnector
import feagi_rust_py_libs as frpl

def update_stage_from_motor(agent: FeagiAgentConnector, cortical_group: int, device_channel: int, input_image_properties, segment_image_properties):
    percent_4d = agent.motors.gaze.read_post_processed(cortical_group, device_channel)

    if percent_4d.c.get_as_0_1() == 0.0 or percent_4d.d.get_as_0_1() == 0.0: # modulation cannot be zero
        return

    eccentricity = frpl.connector_core.data.Percentage2D(
        frpl.connector_core.data.Percentage.new_from_0_1(percent_4d.a.get_as_0_1()),
        frpl.connector_core.data.Percentage.new_from_0_1(percent_4d.b.get_as_0_1()),
    ) # size
    modularity = frpl.connector_core.data.Percentage2D(
        frpl.connector_core.data.Percentage.new_from_0_1(percent_4d.c.get_as_0_1()),
        frpl.connector_core.data.Percentage.new_from_0_1(percent_4d.d.get_as_0_1()),

        #frpl.connector_core.data.Percentage.new_from_0_1(0.5),
        #frpl.connector_core.data.Percentage.new_from_0_1(0.5),

    ) # size
    gaze = frpl.connector_core.data.descriptors.GazeProperties(eccentricity, modularity)


    segmentor_stage = frpl.connector_core.data_pipeline.stage_properties.ImageSegmentorStageProperties = (
        frpl.connector_core.data_pipeline.stage_properties.ImageSegmentorStageProperties(input_image_properties, segment_image_properties, gaze))

    agent.sensors.image_camera_with_peripheral.update_stage(cortical_group, device_channel, 0, segmentor_stage)
    print("a")


async def main():

    # config vars
    resolution_size_input = (1280, 720)
    resolution_output_peripheral = (64, 64)
    resolution_output_center = (128, 128)
    eccentricity = frpl.connector_core.data.Percentage2D(
        frpl.connector_core.data.Percentage.new_from_0_1(0.2),
        frpl.connector_core.data.Percentage.new_from_0_1(0.2),
    ) # size
    modularity = frpl.connector_core.data.Percentage2D(
        frpl.connector_core.data.Percentage.new_from_0_1(0.2),
        frpl.connector_core.data.Percentage.new_from_0_1(0.2),
    ) # size
    cortical_group = 0
    number_of_channels = 1
    number_z_neurons = 10


    input_image_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(resolution_size_input[0],
                                                                               resolution_size_input[1])

    output_center_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(resolution_output_center[0],
                                                                               resolution_output_center[1])

    output_peripheral_resolution = frpl.connector_core.data.descriptors.ImageXYResolution(resolution_output_peripheral[0],
                                                                               resolution_output_peripheral[1])

    color_space = frpl.connector_core.data.descriptors.ColorSpace.Linear
    memory_order = frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels
    image_frame_1 = frpl.connector_core.data.ImageFrame.new_from_array(np.ones((resolution_size_input[0], resolution_size_input[1], 3), dtype=np.uint8),
                                                                        color_space, memory_order)
    image_frame_10 = frpl.connector_core.data.ImageFrame.new_from_array(np.ones((resolution_size_input[0], resolution_size_input[1], 3), dtype=np.uint8) * 10,
                                                                         color_space, memory_order)
    input_image_properties = image_frame_1.get_image_frame_properties()


    color_channel_layout = image_frame_1.channel_layout
    output_resolutions =  frpl.connector_core.data.descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(output_center_resolution, output_peripheral_resolution)
    gaze = frpl.connector_core.data.descriptors.GazeProperties(eccentricity, modularity)
    segment_properties = frpl.connector_core.data.descriptors.SegmentedImageFrameProperties(output_resolutions, color_channel_layout, color_channel_layout, color_space)

    # Creates an instance of connector under the dummy method (for now)
    agent = FeagiAgentConnector.create_dummy_connector()

    await agent.server.connect()

    agent.sensors.image_camera_with_peripheral.register(cortical_group, number_of_channels, input_image_properties, segment_properties, gaze)

    agent.sensors.image_camera_with_peripheral.write(cortical_group, 0, image_frame_1)

    agent.motors.gaze.register(cortical_group, number_of_channels, number_z_neurons)

    # first bytes before chanigng gaze
    bytes_before_gaze_change = agent.encode_cache_to_bytes()

    update_stage_from_motor(agent, cortical_group, 0, input_image_properties, segment_properties)

    print("pause")

if __name__ == "__main__":
    asyncio.run(main())


