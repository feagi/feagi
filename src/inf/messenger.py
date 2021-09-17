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
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(address)

    def send(self, message):
        self.socket.send_pyobj(message)
        print("<< Incomplete Code >>")


class Sub:
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    @staticmethod
    def validate(payload):
        """
        This function endures the received payload meets a certain expectations
        """
        try:
            print("<< Incomplete TRY Code >>")
            return True
        except:
            print("<< Incomplete EXCEPTION Code >>")
            return False

    def receive(self):
        payload = self.socket.recv_pyobj()

        if self.validate(payload):
            return payload
