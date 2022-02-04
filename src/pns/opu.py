
# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================


import sys
import traceback
from inf import runtime_data
from evo.blocks import *
from importlib.machinery import SourceFileLoader
from pns.ipu import IPU
from evo.stats import opu_activity_report


class OPU:
    class Controller:
        # from inf import runtime_data
        # from opu.processor import led, movement
        # from evo.blocks import active_neurons_in_blocks, percent_active_neurons_in_block, block_ref_2_id
        #
        def __init__(self):
            print("OPU Controller has been initialized...")
            self.led = OPU.Destination.LED()
            self.movement = OPU.Processor.Movement()
        #
        # def initialize():
        #     return
        #

        def opu_handler(self):
            """
            This function is intended to handle all the OPU processing that needs to be addressed in burst level as opposed
            to individual neuron fire
            """
            # todo: Introduce a generalized approach to cover all OPUs

            print("Processing OPU...... ... .. .. .. . .. . .. . .. . . . . . .. . .")

            for item in runtime_data.fire_candidate_list:
                print("_______", item)

            # LED handler
            if 'o__led' in runtime_data.fire_candidate_list and runtime_data.hardware == 'raspberry_pi':
                active_led_neurons = active_neurons_in_blocks(cortical_area='led_opu')
                led_data = self.led.convert_neuron_activity_to_rgb_intensities(active_led_neurons)
                self.led.activate_leds(led_data)

            # todo: need a better differentiation between movement and motor modules
            # Movement handler
            if 'o__mot' in runtime_data.fire_candidate_list:
                print("processing            <<  o__mot >>")
                # active_neurons = active_neurons_in_blocks(cortical_area='motor_opu')
                # data = motor.convert_neuron_activity_to_motor_speed(active_neurons)
                # movement.activate_motor(data)
                activity_report = opu_activity_report(cortical_area='o__mot')
                print("motor activity report", activity_report)
                motor_data = dict()
                for device in activity_report:
                    # if there are "ties" w/r/t block activity, this will select the first index in the list w/ the tie value
                    # todo: need a better method
                    # block_with_max_activity = activity_report[device][0].index(max(activity_report[device][0]))
                    try:
                        block_with_max_z = activity_report[device][0].index(max(activity_report[device][0]))
                        tmp_list = set(activity_report[device][0])
                        tmp_list.remove(max(activity_report[device][0]))
                        block_with_2nd_max = activity_report[device][0].index(max(tmp_list))
                        chosen_block = max(block_with_max_z, block_with_2nd_max)
                    except ValueError:
                        chosen_block = 0
                    if device not in motor_data:
                        motor_data[device] = dict()
                    motor_data[device]['speed'] = chosen_block
                self.movement.activate_device(device_type='motor', device_data=motor_data)

            if 'o__ser' in runtime_data.fire_candidate_list:
                # active_neurons = active_neurons_in_blocks(cortical_area='motor_opu')
                # data = motor.convert_neuron_activity_to_motor_speed(active_neurons)
                # movement.activate_motor(data)
                activity_report = opu_activity_report(cortical_area='servo_opu')
                device_data = dict()
                for device in activity_report:
                    # if there are "ties" w/r/t block activity, this will select the first index in the list w/ the tie value
                    # todo: need a better method
                    # block_with_max_activity = activity_report[device][0].index(max(activity_report[device][0]))
                    try:
                        block_with_max_z = activity_report[device][0].index(max(activity_report[device][0]))
                        tmp_list = set(activity_report[device][0])
                        tmp_list.remove(max(activity_report[device][0]))
                        block_with_2nd_max = activity_report[device][0].index(max(tmp_list))
                        chosen_block = max(block_with_max_z, block_with_2nd_max)
                    except ValueError:
                        chosen_block = 0
                    if device not in device_data:
                        device_data[device] = dict()
                    device_data[device]['angle'] = chosen_block
                self.movement.activate_device(device_type='servo', device_data=device_data)

    class Destination:
        def __init__(self):
            print("OPU destination has been initialized...")

        class Motor:
            @staticmethod
            def motor_operator(motor_data):
                """
                motor_data = {
                    motor_id: {
                        "direction": 'F',
                        "...": ...
                    },
                    2: {},
                    3: {}
                }
                """
                try:
                    # todo: Generalize the following section. using specifics for test only
                    if motor_data:
                        runtime_data.opu_data["motor"] = motor_data
                        if runtime_data.parameters["Logs"]["print_burst_info"]:
                            for motor_id in motor_data:
                                print(f">>>>>>>>>>>>>>>>>>>>>>>> motor {motor_id} "
                                      f"activation command sent to router with direction {motor_data[motor_id]}")

                except Exception as e:
                    print("ERROR at Motor module:", e)
                    exc_info = sys.exc_info()
                    traceback.print_exception(*exc_info)

        class Servo:
            import sys
            import traceback
            from inf import runtime_data
            @staticmethod
            def servo_operator(servo_data):
                """
                servo_data = {
                    servo_id: {
                        "angle": 30
                    },
                    2: {},
                    3: {}
                }
                """
                try:
                    # todo: Generalize the following section. using specifics for test only
                    runtime_data.opu_data["servo"] = servo_data
                    if runtime_data.parameters["Logs"]["print_burst_info"]:
                        for servo_id in servo_data:
                            print(f">>>>>>>>>>>>>>>>>>>>>>>> {servo_id} "
                                  f"activation command sent to router with angle {servo_data[servo_id]['angle']}")

                except Exception as e:
                    print("ERROR at Servo module:", e)
                    exc_info = sys.exc_info()
                    traceback.print_exception(*exc_info)

        class LED:
            from evo.blocks import block_ref_2_id, percent_active_neurons_in_block
            from inf import runtime_data

            def __init__(self):
                self.range = IPU.Processor.Range()

            def convert_neuron_activity_to_rgb_intensities(self, blocks_with_active_neurons, cortical_area='led_opu'):
                led_x_dim = runtime_data.genome['blueprint'] \
                    [cortical_area] \
                    ['neuron_params'] \
                    ['block_boundaries'][0]

                led_z_dim = runtime_data.genome['blueprint'] \
                    [cortical_area] \
                    ['neuron_params'] \
                    ['block_boundaries'][-1]

                # create template for accommodating LED data insertion
                led_data = {led_id: [0] * led_z_dim for led_id in range(led_x_dim)}

                for block_ref in blocks_with_active_neurons:
                    block_id = block_ref_2_id(block_ref)
                    block_z_idx = block_id[-1]
                    led_id = block_id[0]

                    percent_active = percent_active_neurons_in_block(block_ref, cortical_area='led_opu')
                    mapped_intensity = round(self.range.map_value(percent_active, 0, 100, 1, 255))
                    if led_id in led_data:
                        led_data[led_id][block_z_idx] = mapped_intensity

                return led_data

            @staticmethod
            def activate_leds(led_data):
                controller = SourceFileLoader("controller.py", runtime_data.hw_controller_path).load_module()
                led = controller.LED()
                for led_id in led_data:
                    R = led_data[led_id][0]
                    G = led_data[led_id][1]
                    B = led_data[led_id][2]
                    led.LED_on(led_id, R, G, B)

        class UTF8:
            from inf import runtime_data

            """
            Output Processing Unit (OPU) functions responsible for translating neuronal activities into a read world
            event.
            """

            @staticmethod
            def convert_neuron_activity_to_utf8_char(cortical_area, neuron_id):
                char = int(runtime_data.brain[cortical_area][neuron_id]['soma_location'][0][2])
                activity_history = list(runtime_data.brain[cortical_area][neuron_id]['activity_history'])
                # todo: move collection span to parameters
                collection_span_counter = len(activity_history) - 1
                membrane_potential_total = 0
                while collection_span_counter > 0:
                    membrane_potential_total += activity_history[collection_span_counter][1]
                    collection_span_counter -= 1

                activity_rank = membrane_potential_total / len(activity_history)
                return chr(char), int(activity_rank)

            if __name__ == '__main__':
                import tkinter
                master = tkinter.Tk()

                text = "Comprehended Character is: " + runtime_data.parameters["Input"]["opu_char"]
                tkinter.Label(master, text=text, font=("Helvetica", 24)).grid(row=0)

                # master.update_idletasks()
                # master.update()

                master.mainloop()

        # class Matrix:

    class Processor:
        def __init__(self):
            print("OPU processor has been initialized...")

        class Movement:
            """
            Functions in this module will help translate neuronal activities associated with movement output processing unit (OPU)
            to its corresponding message that can be passed to an output device so actual movement can be facilitated.
            """

            def __init__(self):
                self.motor = OPU.Destination.Motor()
                self.servo = OPU.Destination.Servo()

            @staticmethod
            def convert_neuronal_activity_to_directions(cortical_area, neuron_id):
                move_index = int(runtime_data.brain[cortical_area][neuron_id]['soma_location'][0][2])
                if move_index == 0:
                    movement_direction = 'w'
                elif move_index == 1:
                    movement_direction = 'x'
                elif move_index == 2:
                    movement_direction = 'a'
                elif move_index == 3:
                    movement_direction = 'd'
                else:
                    movement_direction = ''

                print("\nMovement direction was detected as: ", movement_direction)
                # socket.send_string(movement_direction)

            def activate_device(self, device_type, device_data):
                """
                This function creates a mapping between neurons from a motor cortex region to values suitable for motor
                operation such as direction, speed, duration, and power.

                Speed is captured by a value between 0 to 100 or 0 to -100 where negative values reflect opposite
                rotation Power is captured by a value between 0 to 100 Duration will be derived based on the number of
                times this function is called. Spike train from the brain will trigger multiple calls to this function
                which in return will generate incremental movements on the motor.

                Motor Cortex Assumptions:
                - Each motor, actuator, servo, or an artificial muscle would have a designated cortical column assigned
                to it
                - Each motor cortex cortical column will be recognized with its geometrical dimensions as well as block
                counts in -
                x, y, and z direction
                - X direction captures the motor_id.
                - Y direction does not have any operational significance and cortical columns of the motor cortex are
                to be created-
                with 1 block in x direction
                - Z direction reflects SPEED. Neurons above the mid-point of Y axis will reflect positive speed and
                below will -
                reflect negative speeds. The further the neuron is from the center point of y access the faster the
                speed
                """

                # Speed is defined as a value between -100 and 100 with 100 being the fastest in one direction and -100
                # the other
                # zero_speed_block_offset = floor(cortical_y_block/2)
                # speed_offset = neuron_y_block - zero_speed_block_offset
                # speed = int(speed_offset / (zero_speed_block_offset+00000.1))

                # todo: placeholder for handling motor power
                # Power is defined as a value between 0 and 100 driven from Z direction
                # power = int(neuron_z_block / cortical_z_block)

                if device_type == 'motor':
                    self.motor.motor_operator(device_data)
                elif device_type == 'servo':
                    self.servo.servo_operator(device_data)
