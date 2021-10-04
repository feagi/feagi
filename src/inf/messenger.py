"""
This module provides a common method for IPU and OPU to interact with FEAGI.

Payload template:

All communications to and from FEAGI to follow the standard below:

{'device_id', 'message'}



"""
from datetime import datetime

import zmq
from inf.runtime_data import parameters


class Pub:
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.bind(address)

    def send(self, message):
        self.socket.send_pyobj(message)


class Sub:
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, 1)
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
            timestamp_recv = datetime.now()
            diff = str(timestamp_recv - payload['timestamp'])
            print(">>>>>> >>>>>> PAYLOAD RECEIVED: ", payload)
            print(">>>>>>> >>>>>>>> >>>>>>> >>>>>>>>> >>>>>>>>>> TIME DIFF: ", diff)
            return payload

        # if self.validate(payload):
        #     return payload

        except zmq.ZMQError as e:
            print("Error in messenger module receive: ", e)
            if e.errno == zmq.EAGAIN:
                pass
            else:
                print(e)
