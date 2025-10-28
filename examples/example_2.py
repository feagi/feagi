from feagi_connector_2 import FeagiAgent
import feagi_rust_py_libs as frpl

def to_percentage(v: float) -> frpl.connector_core.data.Percentage:
    return frpl.connector_core.data.Percentage.new_from_0_1(v)

feagi_agent = FeagiAgent()
percentage = to_percentage(0.5)

input_image_properties = frpl.connector_core.data.descriptors.ImageFrameProperties(
    frpl.connector_core.data.descriptors.ImageXYResolution(128, 128),
    frpl.connector_core.data.descriptors.ColorSpace.Linear,
    frpl.connector_core.data.descriptors.ColorChannelLayout.RGB
)

segmented_image_properties = frpl.connector_core.data.descriptors.SegmentedImageFrameProperties(
    frpl.connector_core.data.descriptors.SegmentedXYImageResolutions.create_with_same_sized_peripheral(
        frpl.connector_core.data.descriptors.ImageXYResolution(64, 64),
        frpl.connector_core.data.descriptors.ImageXYResolution(32, 32),
    ),
    frpl.connector_core.data.descriptors.ColorChannelLayout.RGB,
    frpl.connector_core.data.descriptors.ColorChannelLayout.RGB,
    frpl.connector_core.data.descriptors.ColorSpace.Linear
)

gaze = frpl.connector_core.data.descriptors.GazeProperties.create_default_centered()


feagi_agent.brain_input.infrared_absolute_linear.register(0, 1, 10)
feagi_agent.brain_input.segmented_vision_absolute.register(0, 1, input_image_properties, segmented_image_properties, gaze)


feagi_agent.brain_output.gaze_absolute_linear.register(0, 1, 10)


feagi_agent.brain_output.positional_servo_absolute_linear.register(0, 1, 10)

feagi_agent.brain_input.infrared_absolute_linear.write(0, 0, percentage)

feagi_agent.encode_sensor_data_and_send()









# Testing stuff below:

percentage_4d = feagi_agent.brain_output.gaze_absolute_linear.read_postprocessed_cache_value(0, 0)
print(percentage_4d)

feagi_agent.brain_input_cache.encode_cached_values_to_bytes() # tell rust to encode values to bytes
byte_data = feagi_agent.brain_input_cache.copy_out_encoded_bytes() # copy out the bytes
print(byte_data)

feagi_agent.brain_output_cache.load_bytes(byte_data)
feagi_agent.brain_output_cache.decode_bytes_into_cache() # Note, this wont do anything since the byte_data contains cortical ID references for input areas. This is just a test
