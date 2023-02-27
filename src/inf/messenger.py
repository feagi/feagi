
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
import logging
import zmq
import time
from inf import runtime_data

logger = logging.getLogger(__name__)

# todo: consolidate the two publishers in a modular fashion

class PubSub:
    def __init__(self):
        self.context = zmq.Context()

    def send(self, message):
        self.socket.send_pyobj(message)
            
    def receive(self):
        try:
            payload = self.socket.recv_pyobj(flags=zmq.NOBLOCK)
            return payload
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass
            else:
                print(e)

    def terminate(self):
        self.socket.close()
        
    def destroy(self):
        self.context.destroy()
        
        
class Pub(PubSub):
    
    def __init__(self, address, bind=True):
        PubSub.__init__(self)
        print(f"Pub -- Add - {address}, Bind - {bind}")
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 0)
        if bind: 
            self.socket.bind(address)
        else:
            self.socket.connect(address)
            
            
class Sub(PubSub):
    
    def __init__(self, address, bind=False, topic=None):
        PubSub.__init__(self)
        print(f"Sub -- Add - {address}, Bind - {bind}")
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, ''.encode('utf-8'))
        self.socket.setsockopt(zmq.CONFLATE, 1)
        if bind: 
            self.socket.bind(address)
        else:
            self.socket.connect(address)


# if __name__ == '__main__':
#     address, bind = "tcp://localhost:30010", False
#     # address, bind = "tcp://0.0.0.0:30010", True

#     pub = Pub(address, bind)

#     i = 0
#     while True:
#         pub.send({"Hello": f"World {i}"})
#         i+=-1
#         time.sleep(2)
#         print("Send")
