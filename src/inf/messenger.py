
# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
This module provides a common method for IPU and OPU to interact with FEAGI.

Payload template:

All communications to and from FEAGI to follow the standard below:

{'device_id', 'message'}



"""
from datetime import datetime

import zmq
from inf.runtime_data import parameters

# todo: consolidate the two publishers in a modular fashion


class Pub:
    def __init__(self, address):
        self.address = address
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        # self.socket.setsockopt(zmq.SNDHWM, 0)
        self.socket.bind(address)

    def send(self, message):
        self.socket.send_pyobj(message)
        print("FEAGI published a message:", message, "on ", self.address)


# class PubBrainActivities:
#     def __init__(self, address):
#         context = zmq.Context()
#         self.socket = context.socket(zmq.PUB)
#         # self.socket.setsockopt(zmq.SNDHWM, 0)
#         self.socket.bind(address)
#
#     def send(self, message):
#         self.socket.send_pyobj(message)
#         print("Message sent to device is:", message)


class Sub:
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        # self.socket.setsockopt(zmq.RCVHWM, 0)
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
            # print("listening for ipu data...")
            payload = self.socket.recv_pyobj(flags=zmq.NOBLOCK)
            # timestamp_recv = datetime.now()
            # diff = str(timestamp_recv - payload['timestamp'])
            print(">>>>>> >>>>>> PAYLOAD RECEIVED: ", payload)
            # print(">>>>>>> >>>>>>>> >>>>>>> >>>>>>>>> >>>>>>>>>> TIME DIFF: ", diff)
            # print(">>>>>>>>> >>>>>>>>>>> >>>>>>>>>>> >>>>>>> >>>>>> SEQUENCE VAL: ", payload['counter'])
            return payload

        # if self.validate(payload):
        #     return payload

        except zmq.ZMQError as e:
            print("Error in messenger module receive: ", e)
            if e.errno == zmq.EAGAIN:
                pass
            else:
                print(e)


if __name__ == '__main__':
    import time

    pub = Pub("tcp://0.0.0.0:30000")

    while True:

        pub.send(message='ABC')
        time.sleep(1)
