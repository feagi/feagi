
from feagi_connector_2 import FeagiAgent
import feagi_rust_py_libs as frpl
import numpy as np
import asyncio


async def main():
    input_image_resolution = (128, 128, 3)

    input_image_properties = frpl.connector_core.data.descriptors.ImageFrameProperties(
        frpl.connector_core.data.descriptors.ImageXYResolution(input_image_resolution[0], input_image_resolution[1]),
        frpl.connector_core.data.descriptors.ColorSpace.Linear,
        frpl.connector_core.data.descriptors.ColorChannelLayout.RGB
    )

    feagi_agent = FeagiAgent() # create agent instance
    feagi_agent.brain_input.image_camera_center.register(0, 1, input_image_properties) # register camera

    # connect to feagi
    registration_response = await feagi_agent.feagi.connect_via_zmq("tcp://localhost", input_image_resolution, registration_port=30000, brain_input_port=30001, brain_output_port=30002)

    # TODO loop data
    image_arr: np.ndarray = np.ones(input_image_resolution).astype(np.uint8) * 100
    image_frame = frpl.connector_core.data.ImageFrame.new_from_array(image_arr, input_image_properties.color_space, frpl.connector_core.data.descriptors.MemoryOrderLayout.WidthsHeightsChannels) # TODO why is there no in place memory loading available?
    feagi_agent.brain_input.image_camera_center.write(0, 0, image_frame)

    await feagi_agent.brain_input_cache.send_brain_input_to_feagi()


if __name__ == "__main__":
    asyncio.run(main())