"""
This module reads LIDAR data from a message queue and makes them available to the proximity processor.
"""
import time
import zmq

from ipu.processor import proximity
from inf import runtime_data

from importlib.machinery import SourceFileLoader


def get_and_translate():
    controller = SourceFileLoader("controller.py", runtime_data.hw_controller_path).load_module()

    # TODO: resolve interface to differentiate between running in container vs locally
    # try:
    #     if os.environ['CONTAINERIZED']:
    #         socket_address = f"tcp://{interface}:{port}"
    # except KeyError:
    #     socket_address = runtime_data.parameters["Sockets"]["lidar_socket"]

    socket_address = runtime_data.parameters["Sockets"]["lidar_socket"]

    print("Attempting to subscribe to socket ", socket_address)

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(socket_address)
    socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    sonar = controller.Ultrasonic()

    while True:
        if socket_address:
            message = socket.recv_pyobj()
            translate(message=message)

        elif sonar:
            message = sonar.getDistance()
            translate(message=message)

        # todo: need to have a formula to come up with the sleep time here
        time.sleep(1)


def translate(message, type=None):
    """
    Translate the lidar messages based on its type.

    Type is not needed at this point given the lidar vs sonar data is automatically differentiated within the func.
    """
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

        print(">>>>>>>>>>>>> MESSAGE: ", message)

        # differentiate between LIDAR/SONAR data
        if hasattr(message, '__iter__'):
            detections = proximity.lidar_to_coords(message)
        else:
            detections = proximity.sonar_to_coords(message)

        neurons = proximity.coords_to_neuron_ids(
            detections, cortical_area='proximity_ipu'
        )

        # TODO: Add proximity feeder function in fcl_injector
        runtime_data.fcl_queue.put({'proximity_ipu': set(neurons)})
