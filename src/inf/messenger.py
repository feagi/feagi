"""
This module provides the messaging and communication backbone for FEAGI. All of the OPU and IPU interactions with FEAGI
needs to leverage this module to maintain a consistent method of communication across the board.
"""

import zmq
from inf.runtime_data import parameters

class Pub:
    def __init__(self, catagory, topic):


    def send(self, id, message):




class Sub:
    def __init__(self, catagory, topic):
        # TODO: resolve interface to differentiate between running in container vs locally
        # try:
        #     if os.environ['CONTAINERIZED']:
        #         socket_address = f"tcp://{interface}:{port}"
        # except KeyError:
        #     socket_address = runtime_data.parameters["Sockets"]["lidar_socket"]

        socket_address = parameters["Sockets"]["lidar_socket"]

        print("Attempting to subscribe to socket ", socket_address)

        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(socket_address)
        socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    def receive(self, id):
        message = socket.recv_pyobj()





        return message
