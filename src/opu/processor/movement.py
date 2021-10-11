
"""
Functions in this module will help translate neuronal activities associated with movement output processing unit (OPU)
to its corresponding message that can be passed to an output device so actual movement can be facilitated.
"""

import os

from statistics import mode
import inf.runtime_data as runtime_data
from math import floor
from ipu.processor.proximity import map_value
from opu.destination import motor


# # todo: export socket address to config file
# socket_address = 'tcp://0.0.0.0:21000'
# print("Binding to socket ", socket_address)
#
# context = zmq.Context()
# socket = context.socket(zmq.PUB)
#
# # todo: Figure a way to externalize the binding port. feagi_configuration.ini captures it on FEAGI side.
# socket.bind(socket_address)


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


def activate_motor(movement_data):
    """
    This function creates a mapping between neurons from a motor cortex region to values suitable for motor operation -
    such as direction, speed, duration, and power.

    Speed is captured by a value between 0 to 100 or 0 to -100 where negative values reflect opposite rotation
    Power is captured by a value between 0 to 100
    Duration will be derived based on the number of times this function is called. Spike train from the brain will -
    trigger multiple calls to this function which in return will generate incremental movements on the motor.

    Motor Cortex Assumptions:
    - Each motor, actuator, servo, or an artificial muscle would have a designated cortical column assigned to it
    - Each motor cortex cortical column will be recognized with its geometrical dimensions as well as block counts in -
    x, y, and z direction
    - X direction captures the motor_id.
    - Y direction does not have any operational significance and cortical columns of the motor cortex are to be created-
    with 1 block in x direction
    - Z direction reflects SPEED. Neurons above the mid-point of Y axis will reflect positive speed and below will -
    reflect negative speeds. The further the neuron is from the center point of y access the faster the speed
    """

    # Speed is defined as a value between -100 and 100 with 100 being the fastest in one direction and -100 the other
    # zero_speed_block_offset = floor(cortical_y_block/2)
    # speed_offset = neuron_y_block - zero_speed_block_offset
    # speed = int(speed_offset / (zero_speed_block_offset+00000.1))

    # todo: placeholder for handling motor power
    # Power is defined as a value between 0 and 100 driven from Z direction
    # power = int(neuron_z_block / cortical_z_block)

    motor.motor_operator(motor_data=movement_data)
