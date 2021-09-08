"""
This module reads LIDAR data from a message queue and makes them available to the proximity processor.
"""
import sys

import zmq

from ipu.processor import proximity
from inf import runtime_data


# if runtime_data.hardware == 'raspberry_pi':
sys.path.insert(1, '../third_party/freenove/smart_car/')
import controller


def get_and_translate():
    # TODO: resolve interface to differentiate between running in container vs locally
    # try:
    #     if os.environ['CONTAINERIZED']:
    #         socket_address = f"tcp://{interface}:{port}"
    # except KeyError:
    #     socket_address = runtime_data.parameters["Sockets"]["lidar_socket"]

    # socket_address = runtime_data.parameters["Sockets"]["lidar_socket"]

    # print("Attempting to subscribe to socket ", socket_address)

    # context = zmq.Context()
    # socket = context.socket(zmq.SUB)
    # socket.connect(socket_address)
    # socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    sonar = controller.Ultrasonic()

    while True:
        # message = socket.recv_pyobj()
        distance = sonar.getDistance()

        if distance is not None:
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
            if hasattr(distance, '__iter__'):
                detections = proximity.lidar_to_coords(distance)
            else:
                detections = proximity.sonar_to_coords(distance)

            # print(">>>>>>>>>>>>> DISTANCE: ", distance)

            neurons = proximity.coords_to_neuron_ids(
                    detections, cortical_area='proximity_ipu'
            )

            # TODO: Add proximity feeder function in fcl_injector
            runtime_data.fcl_queue.put({'proximity_ipu': set(neurons)})
