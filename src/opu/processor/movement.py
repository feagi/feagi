
"""
Functions in this module will help translate neuronal activities associated with movement output processing unit (OPU)
to its corresponding message that can be passed to an output device so actual movement can be facilitated.
"""


import zmq
import inf.runtime_data as runtime_data
from math import floor
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


def convert_neuronal_activity_to_motor_actions(cortical_area, neuron_id):
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
    - X direction does not have any operational significance and cortical columns of the motor cortex are to be created-
    with 1 block in x direction
    - Y direction reflects SPEED. Neurons above the mid-point of Y axis will reflect positive speed and below will -
    reflect negative speeds. The further the neuron is from the center point of y access the faster the speed
    - Z direction captures the POWER. The higher the neuron is located in Z direction, the higher the power it trigger
    """

    cortical_x_block, cortical_y_block, cortical_z_block = runtime_data.genome['blueprint'][cortical_area]['neuron_params']['block_boundaries']
    neuron_x_block, neuron_y_block, neuron_z_block = runtime_data.brain[cortical_area][neuron_id]['soma_location'][1]

    # Speed is defined as a value between -100 and 100 with 100 being the fastest in one direction and -100 the other
    zero_speed_block_offset = floor(cortical_y_block/2)
    speed_offset = neuron_y_block - zero_speed_block_offset
    speed = int(speed_offset / (zero_speed_block_offset+00000.1))

    # todo: need to define the mapping between motor cortex and a set of motor ids
    """
    Some sort of mapping needs to be defined such as the one below and most likely to be part of genome. This mapping
    will connect a particular motor cortical column to a corresponding motor identifier on the hardware side.
    
    motor_mapping = {
        "motor_1" : "M1", 
        "motor_2" : "M2", 
        "motor_3" : "M3", 
        "motor_4" : "M4", 
    }
    """
    if runtime_data.hardware == 'raspberry_pi':
        # todo: remove hardcoded parameters
        motor.motor_operator(motor_brand="Freenove", motor_model="", motor_id="", speed="", power="")

        # Power is defined as a value between 0 and 100 driven from Z direction
        power = int(neuron_z_block / cortical_z_block)
