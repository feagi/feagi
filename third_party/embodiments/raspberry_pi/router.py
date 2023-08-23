"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""

#!/usr/bin/env python3
import zmq
from datetime import datetime
from configuration import router_settings


class Pub:
    def __init__(self, address):
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        # self.socket.setsockopt(zmq.SNDHWM, 0)
        self.socket.bind(address)

    def send(self, message):
        # if datetime.now().second - router_settings['last_message'] > router_settings['TTL']:
        self.socket.send_pyobj(message)
        print("Sent:", message)
        router_settings['last_message'] = datetime.now().second

class Sub:
    def __init__(self, address, flags=None):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        # self.socket.setsockopt(zmq.RCVHWM, 0)
        self.socket.connect(address)
        self.socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))
        self.flag = flags

    @staticmethod
    def validate(payload):
        """
        This function endures the received payload meets a certain expectations
        """
        try:
            # todo: define validation criterias
            # print("<< Incomplete TRY Code >>")
            return True
        except:
            print("<< Incomplete EXCEPTION Code >>")
            return False

    def receive(self):
        try:
            payload = self.socket.recv_pyobj(self.flag)

            if self.validate(payload):
                return payload

        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass
            else:
                print(e)


def find_feagi(address):
    print('Awaiting connection with FEAGI at...', address)
    subscriber = Sub(address=address, flags=zmq.SUB)
    message = subscriber.receive()
    print("Connection to FEAGI has been established")

    # todo: What information is useful to receive from FEAGI in this message? IPU/OPU list?
    print("Current FEAGI state is at burst number ", message['burst_counter'])

    return message


def register_with_feagi():
    """
    Provides FEAGI the IP address for the ZMQ IPU channel that the sensory data
    """
    print("Registering router with FEAGI")
    publisher_ = Pub('tcp://0.0.0.0:11000')

    # todo: need to send a set of capabilities to FEAGI
    publisher_.send(message={"A", "Hello!"})

    print("Router registration has successfully completed!")