"""
This module reads LIDAR data from a message queue and makes them available to the proximity processor.
"""

import zmq

from ipu.processor import proximity
from inf import runtime_data


def get_and_translate():
    socket_address = runtime_data.parameters["Sockets"]["lidar_socket"]

    print("Attempting to subscribe to socket ", socket_address)

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(socket_address)
    socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    listener = 0

    message = socket.recv_pyobj()
    method_list = [method for method in dir(message) if method.startswith('_') is False]

    while True:
        message = socket.recv_pyobj()

        if message is not None:
            # print("SLOT_TYPES", message.SLOT_TYPES)
            # print("angle_increment:", message.angle_increment)
            # print("angle_max:", message.angle_max)
            # print("angle_min:", message.angle_min)
            # print("get_fields_and_field_types:", message.get_fields_and_field_types)
            # print("header:", message.header)
            # print("intensities:", message.intensities)
            # print("range_max:", message.range_max)
            # print("range_min:", message.range_min)
            # print("ranges:", message.ranges)
            # print("scan_time:", message.scan_time)
            # print("time_increment:", message.time_increment)
            # print("-----")

            # differentiate between LIDAR/SONAR data
            try:
                detections = proximity.lidar_to_coords(message.ranges)
            except AttributeError:
                detections = proximity.sonar_to_coords(message)

            neurons = proximity.coords_to_neuron_ids(
                    detections, cortical_area='proximity'
            )
            # TODO: Add proximity feeder function in fcl_injector
            runtime_data.fcl_queue.put({'proximity': set(neurons)})
