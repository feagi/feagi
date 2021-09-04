
"""
Functions in this module will help translate neuronal activities associated with movement output processing unit (OPU)
to its corresponding message that can be passed to an output device so actual movement can be facilitated.
"""

from statistics import mode
import zmq
import inf.runtime_data as runtime_data
from math import floor
from ipu.processor.proximity import map_value
from opu.destination import motor

# todo: export socket address to config file
socket_address = 'tcp://0.0.0.0:21000'
print("Binding to socket ", socket_address)

context = zmq.Context()
socket = context.socket(zmq.PUB)

# todo: Figure a way to externalize the binding port. feagi_configuration.ini captures it on FEAGI side.
socket.bind(socket_address)


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
    socket.send_string(movement_direction)


def dominant_block_selector(block_data):
    """
    Receives a dictionary of blocks with various levels of activity and selects the dominant one

    Input Sample: block_data is fed in the form of:

    {
        1: [percentage of activity, number of active neurons, total number of neurons],
        0: [94, 49, 52],
        6: [98, 48, 49],
        2: [100, 28, 28]
    }

    """
    # todo: need to find a reasonable algorithm to detect the dominant block.

    # Selects the block with highest level of activity percentage. Due to the randomness nature of the dict data, if
    # multiple blocks has the max percentage of activity one is randomly selected.
    dominant_block = False
    pointer = 0
    for block in block_data:
        if block_data[block][0] > pointer:
            pointer = block_data[block][0]
            dominant_block = block
    print(block_data)
    print("Dominant block is:", dominant_block)
    return dominant_block


def convert_neuronal_activity_to_motor_actions(motor_stats):
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

    """
    Some sort of mapping needs to be defined such as the one below and most likely to be part of genome. This mapping
    will connect a particular motor cortical column to a corresponding motor identifier on the hardware side.

    motor_mapping = {
        "motor_2" : "M2", 
        "motor_3" : "M3", 
        "motor_4" : "M4", 
    }
    """
    motor_mapping = {
        "motor_1": "M1",
        "motor_2": "M2",
        "motor_3": "M3",
        "motor_4": "M4",
    }

    # The dominant block is the reference to the block that represents the motor speed
    for motor_id in motor_stats['current']:
        dominant_speed = dominant_block_selector(motor_stats['current'][motor_id])
        mapped_value = map_value(dominant_speed, 0, 19, 0, 4095)
        motor_speed = int(mapped_value)

        print(">>>>> Motor %s activated with speed %i" % (motor_id, motor_speed))

        # todo: remove hardcoded parameters
        motor.motor_operator(motor_brand="Freenove", motor_model="", motor_id=motor_id, speed=-motor_speed, power="")


    # scaled_motor_spd = int(-motor_speed * 0.75)

    # if runtime_data.hardware == 'raspberry_pi':

    # Power is defined as a value between 0 and 100 driven from Z direction
    # power = int(neuron_z_block / cortical_z_block)
