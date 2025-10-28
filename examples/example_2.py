from feagi_connector_2 import FeagiAgent
import feagi_rust_py_libs as frpl
import numpy as np


def to_percentage(v: float) -> frpl.connector_core.data.Percentage:
    return frpl.connector_core.data.Percentage.new_from_0_1(v)

def move_robot_arm_position(rotation_percentage: frpl.connector_core.data.Percentage):
    # spins the motor on the wheel however the robot works in its API
    print(rotation_percentage, "amount rotated arm")


feagi_agent = FeagiAgent()
percentage = to_percentage(0.5)

#input_image_properties = ImageFrameProperties.new_with_resolution(128, 128)

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





misc_size = frpl.connector_core.data.descriptors.MiscDataDimensions(5, 5, 5)

gaze = frpl.connector_core.data.descriptors.GazeProperties.create_default_centered()

# registering sensors and motors
feagi_agent.brain_input.infrared_absolute_linear.register(0, 1, 10)
feagi_agent.brain_input.segmented_vision_absolute.register(0, 1, input_image_properties, segmented_image_properties, gaze)

feagi_agent.brain_output.gaze_absolute_linear.register(0, 1, 10)
feagi_agent.brain_output.positional_servo_absolute_linear.register(0, 1, 10)
feagi_agent.brain_output.miscellaneous_absolute.register(0, 1, misc_size)
feagi_agent.brain_output.positional_servo_absolute_fractional.register(0, 2, 10) # pretend this is servos in a robot arm

# FAKE CODE: how we would register a callback for motor - TODO only option?
#feagi_agent.brain_output.positional_servo_absolute_fractional.registar_callback(move_robot_arm_position)

# setting a feedback loop
feagi_agent.premade_feedbacks.feedback_absolute_gaze_to_absolute_segmented_vision(0, 0, 0, 0)

feagi_agent.brain_input.infrared_absolute_linear.write(0, 0, percentage)

feagi_agent.send_brain_input_to_feagi()

misc_data = feagi_agent.brain_output.miscellaneous_absolute.read_postprocessed_cache_value(0, 0)
misc_data_arr = misc_data.copy_to_numpy_array()

print(misc_data_arr)

percentage_4d = feagi_agent.brain_output.gaze_absolute_linear.read_postprocessed_cache_value(0, 0)
print(percentage_4d)


exit()

# Testing stuff below:

percentage_4d = feagi_agent.brain_output.gaze_absolute_linear.read_postprocessed_cache_value(0, 0)
print(percentage_4d)

feagi_agent.brain_input_cache.encode_cached_values_to_bytes() # tell rust to encode values to bytes
byte_data = feagi_agent.brain_input_cache.copy_out_encoded_bytes() # copy out the bytes
print(byte_data)

feagi_agent.brain_output_cache.load_bytes(byte_data)
feagi_agent.brain_output_cache.decode_bytes_into_cache() # Note, this wont do anything since the byte_data contains cortical ID references for input areas. This is just a test
