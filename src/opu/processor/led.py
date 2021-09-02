import sys
sys.path.insert(1, '../third_party/freenove/smart_car/')
import controller
# from evo.neuron import block_reference_builder
# from ipu.processor.proximity import map_value
from inf import runtime_data


def convert_neuron_activity_to_led_intensity(led_vals):
    led = controller.LED()
    for led in led_vals:
        R = led_vals[led][0]
        G = led_vals[led][1]
        B = led_vals[led][2]
        led.LED_on(led, R, G, B)
