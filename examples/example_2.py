from feagi_connector_2 import FeagiAgent
import feagi_rust_py_libs as frpl

def to_percentage(v: float) -> frpl.connector_core.data.Percentage:
    return frpl.connector_core.data.Percentage.new_from_0_1(v)

feagi_agent = FeagiAgent()
percentage = to_percentage(0.5)


feagi_agent.brain_input.infrared_absolute_linear.register(0, 1, 10)
feagi_agent.brain_output.gaze_absolute_linear.register(0, 1, 10)

feagi_agent.brain_input.infrared_absolute_linear.write(0, 0, percentage)
percentage_4d = feagi_agent.brain_output.gaze_absolute_linear.read_postprocessed_cache_value(0, 0)
print(percentage_4d)