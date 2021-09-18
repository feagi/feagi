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


class Sub:
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))

    @staticmethod
    def validate(payload):
        """
        This function ensures the received payload meets a certain expectations
        """
        try:
            print("<< Incomplete TRY Code >>")
            return True
        except:
            print("<< Incomplete EXCEPTION Code >>")
            return False

    def receive(self):
        try:
            print("listening for ipu data...")
            payload = self.socket.recv_pyobj(flags=zmq.NOBLOCK)
            print("ipu data received:", payload)
            return payload

        # if self.validate(payload):
        #     return payload

        except zmq.ZMQError as e:
            print("Error: ", e)
            if e.errno == zmq.EAGAIN:
                pass
            else:
                print(e)
