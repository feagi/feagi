#!/usr/bin/env python3
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
import json

import zmq
import socket
import requests
import lz4.frame
import pickle
from time import sleep
import traceback


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
        print(f"Pub -|- Add - {address}, Bind - {bind}")
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


# def register_with_feagi(feagi_ip, feagi_api_port, agent_type: str, agent_id: str, agent_ip: str, agent_data_port: int,
#                         agent_capabilities):
#     """
#     To trade information between FEAGI and Controller

#     Controller                      <--     FEAGI(IPU/OPU socket info)
#     Controller (Capabilities)       -->     FEAGI
#     """
#     api_address = 'http://' + feagi_ip + ':' + feagi_api_port
#     network_endpoint = '/v1/feagi/feagi/network'
#     stimulation_period_endpoint = '/v1/feagi/feagi/burst_engine/stimulation_period'
#     burst_counter_endpoint = '/v1/feagi/feagi/burst_engine/burst_counter'
#     registration_endpoint = '/v1/agent/register'

#     registration_complete = False
#     feagi_settings = dict()
#     while not registration_complete:
#         try:
#             feagi_settings = requests.get(api_address + network_endpoint).json()
#             if feagi_settings:
#                 print("Data from FEAGI::", feagi_settings)
#             else:
#                 print("No feagi settings!")

#             agent_registration_data = dict()
#             agent_registration_data["agent_type"] = str(agent_type)
#             agent_registration_data["agent_id"] = str(agent_id)
#             agent_registration_data["agent_ip"] = str(agent_ip)
#             agent_registration_data["agent_data_port"] = int(agent_data_port)

#             response = requests.post(api_address + registration_endpoint, params=agent_registration_data)
#             if response.status_code == 200:
#                 feagi_settings['agent_info'] =  response.json()
#                 print("Agent successfully registered with FEAGI!")
#                 # Receive FEAGI settings
#                 feagi_settings['burst_duration'] = requests.get(api_address + stimulation_period_endpoint).json()
#                 feagi_settings['burst_counter'] = requests.get(api_address + burst_counter_endpoint).json()
                

#                 if feagi_settings and feagi_settings['burst_duration'] and feagi_settings['burst_counter']:
#                     print("\n\n\n\nRegistration is complete....")
#                     registration_complete = True
#         except Exception as e:
#             print("Trying to register with FEAGI at ", api_address)
#         sleep(1)

#     print("feagi_ip:agent_data_port", feagi_ip, agent_data_port)
#     # Transmit Controller Capabilities
#     # address, bind = f"tcp://*:{agent_data_port}", True
#     address, bind = f"tcp://{feagi_ip}:{agent_data_port}", False

#     publisher = Pub(address, bind)
#     publisher.send(agent_capabilities)

#     return feagi_settings


def feagi_settings_from_composer(feagi_auth_url, feagi_settings):
    """
    Generate all needed information and return the full data to make it easier to connect with
    FEAGI
    """
    if feagi_auth_url is not None:
        print(f"Updating feagi settings using feagi_auth_url: {feagi_auth_url}")
        new_settings = requests.get(feagi_auth_url).json()
        # update feagi settings here
        feagi_settings['feagi_dns'] = new_settings['feagi_dns']
        feagi_settings['feagi_host'] = new_settings['feagi_host']
        feagi_settings['feagi_api_port'] = new_settings['feagi_api_port']
        print(f"New Settings ---- {new_settings}")
    else:
        print(f"Missing feagi_auth_url, using default feagi settings")


    if feagi_settings.get('feagi_dns') is not None:
        feagi_settings['feagi_url'] = feagi_settings['feagi_dns'] 
    else: 
        feagi_settings['feagi_url'] = f"http://{feagi_settings['feagi_host']}:{feagi_settings['feagi_api_port']}"
    return feagi_settings


def register_with_feagi(feagi_auth_url, feagi_settings, agent_settings, agent_capabilities,
                        controller_version, agent_version):
    """
    To trade information between FEAGI and Controller

    Controller                      <--     FEAGI(IPU/OPU socket info)
    Controller (Capabilities)       -->     FEAGI
    """
    network_endpoint = '/v1/feagi/feagi/network'
    stimulation_period_endpoint = '/v1/feagi/feagi/burst_engine/stimulation_period'
    burst_counter_endpoint = '/v1/feagi/feagi/burst_engine/burst_counter'
    registration_endpoint = '/v1/agent/register'

    registration_complete = False
    while not registration_complete:
        try:
            print(f"Original Feagi Settings ---- {feagi_settings}")  
            feagi_settings = feagi_settings_from_composer(feagi_auth_url, feagi_settings)
            feagi_url = feagi_settings['feagi_url']           
               

            network_output = requests.get(feagi_url + network_endpoint).json()
            # print(f"network_output ---- {network_output}")     
            feagi_settings['feagi_opu_port'] = network_output['feagi_opu_port']
            if feagi_settings:
                print("Data from FEAGI::", feagi_settings)
            else:
                print("No feagi settings!")
                
            agent_registration_data = dict()
            agent_registration_data["agent_type"] = str(agent_settings['agent_type'])
            agent_registration_data["agent_id"] = str(agent_settings['agent_id'])
            agent_registration_data["agent_ip"] = str(agent_settings['agent_ip'])
            agent_registration_data["agent_data_port"] = int(agent_settings['agent_data_port'])
            agent_registration_data["controller_version"] = str(controller_version)
            agent_registration_data["agent_version"] = str(agent_version)

            response = requests.post(feagi_url + registration_endpoint, params=agent_registration_data)
            if response.status_code == 200:
                feagi_settings['agent_state'] =  response.json()
                print("Agent successfully registered with FEAGI!")
                # Receive FEAGI settings
                feagi_settings['burst_duration'] = requests.get(feagi_url + stimulation_period_endpoint).json()
                feagi_settings['burst_counter'] = requests.get(feagi_url + burst_counter_endpoint).json()
                

                if feagi_settings and feagi_settings['burst_duration'] and feagi_settings['burst_counter']:
                    print("\n\n\n\nRegistration is complete....")
                    registration_complete = True
        except Exception as e:
            print("Registeration failed with FEAGI: ", e)
            # traceback.print_exc()
        sleep(2)

    print(f"Final Feagi Settings ---- {feagi_settings}")     
    feagi_ip = feagi_settings['feagi_host']
    agent_data_port = feagi_settings['agent_state']['agent_data_port']
    print("feagi_ip:agent_data_port", feagi_ip, agent_data_port)
    # Transmit Controller Capabilities
    # address, bind = f"tcp://*:{agent_data_port}", True
    address, bind = f"tcp://{feagi_ip}:{agent_data_port}", False

    publisher = Pub(address, bind)
    publisher.send(agent_capabilities)

    return feagi_settings