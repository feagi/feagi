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
import os
import zmq
import json
import time
import pickle
import socket
import asyncio
import requests
import platform
import traceback
import lz4.frame
import websockets
import zmq.asyncio
from time import sleep
from websockets.sync.client import connect
from feagi_connector import pns_gateway as pns

if platform.system() == "Windows":
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy()) # For windows
global_feagi_opu_channel = ''  # Updated by feagi.connect_to_feagi()
global_api_address = ''  # Updated by feagi.connect_to_feagi
global_websocket_address = ''  # Just a full address stored
websocket = ''  # It will be an object to store
msg_counter = 0 # for SeqID in feagi data aka message_to_feagi


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
        self.context = zmq.asyncio.Context()
        self.flags = flags

    def send(self, message):
        self.socket.send_pyobj(message)

    def receive(self):
        try:
            payload = self.socket.recv_pyobj()
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


async def fetch_feagi(feagi_opu_channel):
    """
    Obtain the data from feagi's OPU
    """
    while True:
        received_data = await feagi_opu_channel.receive()  # Obtain data from FEAGI
        # Verify if the data is compressed
        if isinstance(received_data, bytes):
            # Decompress
            decompressed_data = lz4.frame.decompress(received_data)
            # Another decompress of json
            pns.message_from_feagi = pickle.loads(decompressed_data)
        else:
            # Directly obtain without any compressions
            pns.message_from_feagi = received_data


def feagi_listener(feagi_opu_channel):
    while True:
        data = fetch_feagi(feagi_opu_channel)
        if data is not None:
            pns.message_from_feagi = data
        sleep(0.001)  # hardcoded and max second that it can run up to
        # print("inside router: ", pns.message_from_feagi['opu_data']['ov_ecc'])


def send_feagi(message_to_feagi, feagi_ipu_channel, agent_settings):
    """
    send data to FEAGI
    """
    if agent_settings['compression']:
        serialized_data = pickle.dumps(message_to_feagi)
        feagi_ipu_channel.send(message=lz4.frame.compress(serialized_data))
    else:
        feagi_ipu_channel.send(message_to_feagi)


def fetch_aptr(get_size_for_aptr_cortical):
    try:
        raw_aptr = requests.get(get_size_for_aptr_cortical).json()
        return raw_aptr['cortical_dimensions'][2]
    except Exception as error:
        print("error: ", error)
        return 10


def fetch_geometry():
    try:
        list_dimesions = requests. \
            get(global_api_address + '/v1/cortical_area/cortical_area/geometry').json()
        return list_dimesions
    except Exception as e:
        print("e: ", e)
        return []


def fetch_template():
    try:
        list_template = requests.get(global_api_address + '/v1/system/cortical_area_types').json()
        return list_template
    except Exception as e:
        print("e: ", e)
        return []

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
        feagi_settings[
            'feagi_url'] = f"http://{feagi_settings['feagi_host']}:{feagi_settings['feagi_api_port']}"
    return feagi_settings


def register_with_feagi(feagi_auth_url, feagi_settings, agent_settings, agent_capabilities,
                        controller_version, agent_version):
    """
    To trade information between FEAGI and Controller

    Controller                      <--     FEAGI(IPU/OPU socket info)
    Controller (Capabilities)       -->     FEAGI
    """
    network_endpoint = '/v1/network/network'
    stimulation_period_endpoint = '/v1/burst_engine/stimulation_period'
    burst_counter_endpoint = '/v1/burst_engine/burst_counter'
    registration_endpoint = '/v1/agent/register'

    registration_complete = False


    while not registration_complete:
        try:
            # print(f"Original Feagi Settings ---- {feagi_settings}")
            feagi_settings = feagi_settings_from_composer(feagi_auth_url, feagi_settings)
            feagi_url = feagi_settings['feagi_url']

            # print("feagiurl: ", feagi_url, " network endpoint: ", network_endpoint)

            network_output = requests.get(feagi_url + network_endpoint).json()
            # print(f"network_output ---- {network_output}")
            if not os.path.exists(pns.env_current_path) and 'feagi_opu_port' not in feagi_settings:
                feagi_settings['feagi_opu_port'] = network_output['feagi_opu_port']

            agent_registration_data = dict()
            agent_registration_data['capabilities'] = agent_capabilities
            agent_registration_data["agent_type"] = str(agent_settings['agent_type'])
            agent_registration_data["agent_id"] = str(agent_settings['agent_id'])
            agent_registration_data["agent_ip"] = str(agent_settings['agent_ip'])  # str("127.0.0.1")
            agent_registration_data["agent_data_port"] = int(agent_settings['agent_data_port'])
            agent_registration_data["controller_version"] = str(controller_version)
            agent_registration_data["agent_version"] = str(agent_version)
            response = requests.post(feagi_url + registration_endpoint,
                                     data=json.dumps(agent_registration_data))
            if response.status_code == 200:
                feagi_settings['agent_state'] = response.json()
                # print("Agent successfully registered with FEAGI!")
                # Receive FEAGI settings
                feagi_settings['burst_duration'] = requests.get(feagi_url + stimulation_period_endpoint).json()
                feagi_settings['burst_counter'] = requests.get(feagi_url + burst_counter_endpoint).json()

                if feagi_settings and feagi_settings['burst_duration'] and feagi_settings['burst_counter']:
                    print("Registration is complete....")
                    registration_complete = True
        except Exception as e:
            print("Registeration failed with FEAGI: ", e)
            # traceback.print_exc()
        sleep(2)


    if not pns.env_exists:
        # feagi_settings['agent_state']['agent_ip'] = "127.0.0.1"
        feagi_ip = feagi_settings['feagi_host']
        agent_data_port = feagi_settings['agent_state']['agent_data_port']
        # print("feagi_ip:agent_data_port", feagi_ip, agent_data_port)
        # Transmit Controller Capabilities
        # address, bind = f"tcp://*:{agent_data_port}", True
        address, bind = f"tcp://{feagi_ip}:{agent_data_port}", False

        publisher = Pub(address, bind)
        publisher.send(agent_capabilities)

    return feagi_settings


# # Websocket section # #
async def main(function, ip, port):
    """
    The main function handles the websocket and spins the asyncio to run the echo function
    infinitely until it exits. Once it exits, the function will resume to the next new websocket.
    """
    async with websockets.serve(function, ip, port, max_size=None, max_queue=None, write_limit=None,
                                compression=None):
        await asyncio.Future()  # run forever


def websocket_operation(function, ip, port):
    """
    WebSocket initialized to call the echo function using asyncio.
    """
    asyncio.run(main(function, ip, port))


async def bridge_to_godot(ws_operation, ws, feagi_settings):
    while True:
        if ws:
            try:
                if ws_operation:
                    if len(ws) > 0:
                        if len(ws) > 2:
                            stored_value = ws.pop()
                            ws.clear()
                            ws.append(stored_value)
                    await ws_operation[0].send(ws[0])
                    ws.pop()
                if "stimulation_period" in feagi_settings:
                    sleep(feagi_settings["stimulation_period"])
                else:
                    sleep(0.001)
            except Exception as error:
                # print("error in websocket sender: ", error)
                # traceback.print_exc()
                ws_operation.pop()
                sleep(0.001)
        else:
            sleep(0.001)


def bridge_operation(ws_operation, ws, feagi_settings):
    asyncio.run(bridge_to_godot(ws_operation, ws, feagi_settings))


def websocket_client_initalize(ip, port, dns=''):
    global websocket, global_websocket_address
    if dns != '':
        websocket = connect(dns)
        global_websocket_address = dns
    else:
        global_websocket_address = str('ws://' + ip + ':' + port)
        websocket = connect(global_websocket_address)


def websocket_send(data):
    global global_websocket_address, websocket
    try:
        websocket.send(pickle.dumps(data))
    except:
        websocket = connect(global_websocket_address)


def websocket_recieve():
    global websocket, global_websocket_address
    while True:
        try:
            pns.message_from_feagi = pickle.loads(websocket.recv())
        except Exception as e:
            print("error in websocket recieve: ", e)
            websocket = connect(global_websocket_address)
