
"""
Functions in this module will help translate neuronal activities associated with movement output processing unit (OPU)
to its corresponding message that can be passed to an output device so actual movement can be facilitated.
"""


import zmq
import inf.runtime_data as runtime_data

# todo: export socket address to config file
socket_address = 'tcp://0.0.0.0:21000'
print("Binding to socket ", socket_address)

context = zmq.Context()
socket = context.socket(zmq.PUB)

# todo: Figure a way to externalize the binding port. feagi_configuration.ini captures it on FEAGI side.
socket.bind(socket_address)


def convert_neuronal_activity_to_movement(cortical_area, neuron_id):
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


