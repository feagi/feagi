"""
"""

import zmq
import socket
import requests
from time import sleep


def app_host_info():
    host_name = socket.gethostname()
    ip_address = socket.gethostbyname(socket.gethostname())
    return {"ip_address": ip_address, "host_name": host_name}


class Pub:
    def __init__(self, address):
        context = zmq.Context()
        self.address = address
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(address)

    def send(self, message):
        self.socket.send_pyobj(message)
        print("Sent:\n", message, "... from ", self.address, "\n\n")


class Sub:
    def __init__(self, address, flags=None):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
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


def register_with_feagi(app_name, feagi_host, api_port, app_capabilities, app_host_info):
    """
    To trade information between FEAGI and Controller

    Controller                      <--     FEAGI(IPU/OPU socket info)
    Controller (Capabilities)       -->     FEAGI
    """

    api_address = 'http://' + feagi_host + ':' + api_port

    registration_endpoint = '/v1/feagi/register'
    network_endpoint = '/v1/feagi/feagi/network'
    stimulation_period_endpoint = '/v1/feagi/feagi/burst_engine/stimulation_period'
    burst_counter_endpoint = '/v1/feagi/feagi/burst_engine/burst_counter'

    registration_data = {"source": app_name,
                         "host": app_host_info["ip_address"],
                         "capabilities": app_capabilities}

    registration_complete = False

    while not registration_complete:

        print("Registration data:", registration_data)

        feagi_registration_result = requests.post(api_address + registration_endpoint, data=registration_data)

        print("FEAGI registration results: ", feagi_registration_result)

        feagi_settings = requests.get(api_address + network_endpoint).json()

        app_port_id = 'feagi_inbound_port_' + app_name
        zmq_address = 'tcp://' + feagi_host + ':' + feagi_settings[app_port_id]

        print('Awaiting connection with FEAGI at...', zmq_address)
        subscriber = Sub(address=zmq_address, flags=zmq.SUB)

        # Receive FEAGI settings
        feagi_settings['burst_duration'] = requests.get(api_address + stimulation_period_endpoint).json()
        feagi_settings['burst_counter'] = requests.get(api_address + burst_counter_endpoint).json()

        print("\nFEAGI settings received as:\n", feagi_settings, "\n\n")
        if feagi_settings and feagi_settings['burst_duration'] and feagi_settings['burst_counter']:
            print("\n\n\n\nRegistration is complete....")
            registration_complete = True
        sleep(1)

    # Transmit Controller Capabilities
    pub_address = "tcp://0.0.0.0:" + feagi_settings[app_port_id]
    publisher = Pub(address=pub_address)
    publisher.send(app_capabilities)

    return feagi_settings
