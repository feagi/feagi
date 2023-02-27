"""
"""
import json

import zmq
import socket
import requests
from time import sleep


def app_host_info():
    host_name = socket.gethostname()
    ip_address = socket.gethostbyname(socket.gethostname())
    return {"ip_address": ip_address, "host_name": host_name}


import zmq
import time 


class PubSub:
    def __init__(self, flags=None):
        self.context = zmq.Context()
        self.flags = flags

    def send(self, message):
        self.socket.send_pyobj(message)
            
    def receive(self):
        try:
            payload = self.socket.recv_pyobj(flags=self.flags)
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
    
    def __init__(self, address, bind=True, flags=None):
        PubSub.__init__(self, flags)
        print(f"Pub -- Add - {address}, Bind - {bind}")
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 0)
        if bind: 
            self.socket.bind(address)
        else:
            self.socket.connect(address)
            
            
class Sub(PubSub):
    
    def __init__(self, address, bind=False, flags=None):
        PubSub.__init__(self)
        print(f"Sub -- Add - {address}, Bind - {bind}")
        self.flags = flags
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, ''.encode('utf-8'))
        self.socket.setsockopt(zmq.CONFLATE, 1)
        if bind: 
            self.socket.bind(address)
        else:
            self.socket.connect(address)


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
            print("Registration attempt with FEAGI failed!")
        sleep(1)

    # Transmit Controller Capabilities
    address, bind = f"tcp://*:{agent_data_port}", True
    # address, bind = f"tcp://{feagi_ip}:{agent_data_port}", False
    
    publisher = Pub(address, bind)
    publisher.send(agent_capabilities)

    return feagi_settings
