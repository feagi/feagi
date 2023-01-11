"""
"""
import json

import zmq
import socket
import requests
from time import sleep


def app_host_info():
    host_name = socket.gethostname()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        s.connect(('10.254.254.254', 1))
        ip_address = s.getsockname()[0]
    except Exception:
        ip_address = '127.0.0.1'
    finally:
        s.close()
    return {"ip_address": ip_address, "host_name": host_name}


class Pub:
    def __init__(self, address):
        context = zmq.Context()
        self.address = address
        self.socket = context.socket(zmq.PUB)
        self.socket.bind(address)

    def send(self, message):
        self.socket.send_pyobj(message)
        # print("Sent:\n", message, "... from ", self.address, "\n\n")


class Sub:
    def __init__(self, address, flags=None):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.setsockopt(zmq.CONFLATE, 1)
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


def register_with_feagi(feagi_ip, feagi_api_port, agent_type: str, agent_id: str, agent_ip: str, agent_data_port: int,
                        agent_capabilities):
    """
    To trade information between FEAGI and Controller

    Controller                      <--     FEAGI(IPU/OPU socket info)
    Controller (Capabilities)       -->     FEAGI
    """

    api_address = 'http://' + feagi_ip + ':' + feagi_api_port
    network_endpoint = '/v1/feagi/feagi/network'
    stimulation_period_endpoint = '/v1/feagi/feagi/burst_engine/stimulation_period'
    burst_counter_endpoint = '/v1/feagi/feagi/burst_engine/burst_counter'
    registration_endpoint = '/v1/agent/register'

    registration_complete = False

    feagi_settings = dict()

    while not registration_complete:
        feagi_settings = requests.get(api_address + network_endpoint).json()
        if feagi_settings:
            print("Data from FEAGI::", feagi_settings)
        else:
            print("No feagi settings!")

        agent_registration_data = dict()
        agent_registration_data["agent_type"] = str(agent_type)
        agent_registration_data["agent_id"] = str(agent_id)
        agent_registration_data["agent_ip"] = str(agent_ip)
        agent_registration_data["agent_data_port"] = int(agent_data_port)

        registration_status = requests.post(api_address + registration_endpoint, params=agent_registration_data)

        if registration_status:
            print("Agent successfully registered with FEAGI!")
            # Receive FEAGI settings
            feagi_settings['burst_duration'] = requests.get(api_address + stimulation_period_endpoint).json()
            feagi_settings['burst_counter'] = requests.get(api_address + burst_counter_endpoint).json()

            if feagi_settings and feagi_settings['burst_duration'] and feagi_settings['burst_counter']:
                print("\n\n\n\nRegistration is complete....")
                registration_complete = True
        else:
            print("Registration attempt with FEAGI failed! Most likely an agent with the same id is already registered")
        sleep(1)

    # Transmit Controller Capabilities
    pub_address = "tcp://0.0.0.0:" + str(agent_data_port)
    publisher = Pub(address=pub_address)
    publisher.send(agent_capabilities)

    return feagi_settings
