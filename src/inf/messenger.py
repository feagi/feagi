"""
This module provides a common method for IPU and OPU to interact with FEAGI.

Payload template:

All communications to and from FEAGI to follow the standard below:

{'device_id', 'message'}



"""

import zmq
from inf.runtime_data import parameters


class Pub:
    def __init__(self, address):


    def send(self, id, message):




class Sub:
    def __init__(self, address):
        # TODO: resolve interface to differentiate between running in container vs locally
        # try:
        #     if os.environ['CONTAINERIZED']:
        #         socket_address = f"tcp://{interface}:{port}"
        # except KeyError:
        #     socket_address = runtime_data.parameters["Sockets"]["lidar_socket"]

        socket_address = parameters["Sockets"]["lidar_socket"]

        print("Attempting to subscribe to socket ", socket_address)

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(socket_address)
        self.socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    @staticmethod
    def validate(payload):
        """
        This function endures the received payload meets a certain expectations
        """
        try:
            # todo
            return True
        except:
            # todo
            return False



    def receive(self):
        payload = self.socket.recv_pyobj()

        if self.validate(payload):
            return payload
