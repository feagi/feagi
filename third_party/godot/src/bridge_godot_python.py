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
import time
import os
import json
import ast
import asyncio
import random
import socket
import threading
from time import sleep
from collections import deque
import websockets
import requests
from configuration import *
from feagi_agent import feagi_interface as feagi

ws_queue = deque()
zmq_queue = deque()
BURST_SECOND = 0
current_cortical_area = {}
FEAGI_HOST = ""
API_PORT = ""

runtime_data = {
    "cortical_data": {},
    "current_burst_id": None,
    "stimulation_period": None,
    "feagi_state": None,
    "feagi_network": None,
    "cortical_list": set(),
    "host_network": {},
    "genome_number": 0,
    "old_cortical_data": {}
}

DIMENSIONS_ENDPOINT = '/v1/feagi/connectome/properties/dimensions'


def simulation_testing():
    """
    This is to stress the CPU using Godot. You should be able to see the red cube filled with
    small red cubes inside. You can simply uncomment this function in main() to test the
    stress and increase the iteration numbers to stress more.
    """
    array = [[random.randint(0, 64), random.randint(0, 64), random.randint(0, 64)] for _ in
             range(1000)]
    return array


def godot_data(data_input):
    """
    Simply clean the list and remove all unnecessary special characters and deliver with name, xyz
    only
    """
    data = ast.literal_eval(data_input)
    dict_with_updated_name = {"data": {}}
    dict_with_updated_name["data"]["direct_stimulation"] = dict({})
    for key in data["data"]["direct_stimulation"]:
        updated_name = key
        if dict_with_updated_name["data"]["direct_stimulation"].get(updated_name) is not None:
            pass
        else:
            dict_with_updated_name["data"]["direct_stimulation"][updated_name] = []
        for key_01 in data["data"]["direct_stimulation"][key]:
            dict_with_updated_name["data"]["direct_stimulation"][updated_name].append(key_01)

    print("godot_data: ", dict_with_updated_name)
    return dict_with_updated_name


def name_to_id(name):
    """
    Convert from name to id of the cortical area name
    """
    for cortical_area in runtime_data["cortical_data"]:
        if cortical_area == name:
            return runtime_data["cortical_data"][cortical_area][7]

    print("*** Failed to find cortical name ***")
    return None


def feagi_breakdown(data):
    """
    Designed for genome 2.0 only. Data is the input from feagi's raw data.
    This function will detect if cortical area list is different than the first, it will generate
    genome list for godot automatically.
    """
    try:
        new_list = []
        new_genome_num = data['genome_num']
        if new_genome_num > runtime_data["genome_number"]:
            runtime_data["old_cortical_data"] = runtime_data["cortical_data"]
            runtime_data["cortical_data"] = \
                requests.get('http://' + FEAGI_HOST + ':' + API_PORT + DIMENSIONS_ENDPOINT,
                             timeout=10).json()
            if 'genome_reset' not in data and data == "{}":
                runtime_data["cortical_data"] = \
                    requests.get(
                        'http://' + FEAGI_HOST + ':' + API_PORT + DIMENSIONS_ENDPOINT,
                        timeout=10).json()
            if data != "{}":
                if runtime_data["old_cortical_data"] != runtime_data["cortical_data"]:
                    pass
            runtime_data["genome_number"] = new_genome_num
        for i in data['godot']:
            new_list.append([i[1], i[2], i[3]])
        return new_list
    except Exception as error:
        print("Exception during feagi_breakdown", error)
        return None


def convert_absolute_to_relative_coordinate(stimulation_from_godot, cortical_data):
    """
    Convert absolute coordinate from godot to relative coordinate for FEAGI. Dna_information is
    from the genome["blueprint"].
    """
    relative_coordinate = {"data": {"direct_stimulation": {}}}

    if stimulation_from_godot:
        for godot_key, xyz_list in stimulation_from_godot["data"]["direct_stimulation"].items():
            for raw_id, cortical_info in cortical_data.items():
                cortical_name = cortical_info[7]
                if cortical_name == godot_key:
                    if cortical_name not in relative_coordinate["data"]["direct_stimulation"]:
                        relative_coordinate["data"]["direct_stimulation"][cortical_name] = []

                    for xyz in xyz_list:
                        new_xyz = [xyz[0] - cortical_info[0],
                                   xyz[1] - cortical_info[1],
                                   xyz[2] - cortical_info[2]]
                        relative_coordinate["data"]["direct_stimulation"][cortical_name]. \
                            append(new_xyz)
                    break

    return relative_coordinate


def is_tcp_server_reachable(server_host, server_port):
    """
    TO check if feagi is reachable.
    """
    try:
        # Create a socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((server_host, server_port))
        return True
    except Exception as error:
        return False


def download_genome():
    try:
        data_from_genome = requests.get('http://' + FEAGI_HOST + ':' + API_PORT +
                                        '/v1/feagi/genome/download',
                                        timeout=10).json()
        cortical_area_name = requests.get(
            'http://' + FEAGI_HOST + ':' + API_PORT + DIMENSIONS_ENDPOINT,
            timeout=10).json()
        return data_from_genome, cortical_area_name
    except Exception as error:
        print("Error while fetching genome from FEAGI: ", error)
        return None, None


def process_genome_data(runtime_data, cortical_data):
    cortical_name = [cortical_data[x][7] for x in cortical_data]
    cortical_genome_dictionary = {"genome": {}}

    for i in runtime_data["cortical_data"]["blueprint"]:
        for x in cortical_name:
            if x in i:
                if x not in cortical_genome_dictionary['genome']:
                    cortical_genome_dictionary['genome'][x] = []
                cortical_genome_dictionary['genome'][x].append(
                    runtime_data["cortical_data"]["blueprint"][i])

    return cortical_genome_dictionary


def reload_genome():
    """
    Every genome reloads or updated, this will be called.
    """
    while True:
        cortical_genome_dictionary = {"genome": {}}
        print("+++ 0 +++")

        data_from_genome, cortical_area_name = download_genome()
        if data_from_genome and cortical_area_name:
            runtime_data["cortical_data"] = data_from_genome
            print("cortical_area_name: ", cortical_area_name)

            cortical_genome_dictionary = process_genome_data(runtime_data, cortical_area_name)

            json_object = json.dumps(cortical_genome_dictionary)
            zmq_queue.append(json_object)

        time.sleep(2)

        print("Genome reloaded.")
        if len(ws_queue[0]) > 2:
            ws_queue.clear()
        runtime_data["cortical_data"] = cortical_area_name
        return cortical_genome_dictionary.copy()


async def echo(websocket):
    """
    Main thread for websocket only.
    """
    while True:
        try:
            if "genome" in zmq_queue[0]:
                cortical_genome_list = str(zmq_queue[0])
                zmq_queue.pop()
                await websocket.send(cortical_genome_list)
            if len(zmq_queue) > 2:  # This will eliminate any stack up queue
                stored_value = zmq_queue[len(zmq_queue) - 1]
                zmq_queue.clear()
                zmq_queue[0] = stored_value
            await websocket.send(str(zmq_queue[0]))
            zmq_queue.pop()
        except Exception as error:
            pass
        new_data = await websocket.recv()
        ws_queue.append(new_data)


async def websocket_main():
    """
    This function sets up a WebSocket server using the 'websockets' library to communicate with a
    Godot game engine.

    The function establishes a WebSocket connection with a Godot game engine running on the
    specified IP address and port provided in 'agent_settings'. It uses the 'echo' coroutine to
    handle incoming WebSocket messages, which will echo back the received messages to the sender.

    Parameters: None

    Returns:
        None

    Raises:
        None

    Note: - The 'agent_settings' dictionary should contain the following keys: -
    'godot_websocket_ip': The IP address where websocket will broadcast for. By default,
    it should be "0.0.0.0".
    'godot_websocket_port': The port is by default to 9050. You can update in configuration.

        - The WebSocket server is configured with the following options: - 'max_size': The
        maximum size (in bytes) of incoming WebSocket messages. Set to 'None' for no limit. -
        'max_queue': The maximum number of incoming WebSocket messages that can be queued for
        processing. Set to 'None' for no limit. - 'write_limit': The maximum rate (in bytes per
        second) at which outgoing WebSocket messages can be sent. Set to 'None' for no limit. -
        'compression': The compression method to use for outgoing WebSocket messages. Set to
        'None' for no compression.

        - The function uses 'asyncio.Future()' to keep the WebSocket server running indefinitely.
        This is required because the 'websockets.serve()' coroutine itself does not naturally
        keep the server running; it only sets up the server to accept incoming connections and
        requires another coroutine or task to run the event loop.

    """
    async with websockets.serve(echo, agent_settings["godot_websocket_ip"],
                                agent_settings['godot_websocket_port'], max_size=None,
                                max_queue=None, write_limit=None, compression=None):
        await asyncio.Future()


def websocket_operation():
    """
    Run asyncio using websocket operations.

    This function runs the 'websocket_main()' coroutine using asyncio's 'run' function,
    which provides a simple way to execute the coroutine in the event loop.
    """
    asyncio.run(websocket_main())


def main():
    """
    Main script for bridge to communicate with FEAGI and Godot.
    """
    global PREVIOUS_GENOME_TIMESTAMP, FEAGI_HOST, API_PORT, BURST_SECOND
    print(
        "================================ @@@@@@@@@@@@@@@ "
        "==========================================")
    print(
        "================================ @@@@@@@@@@@@@@@ "
        "==========================================")
    print(
        "================================ @@@@@@@@@@@@@@@ "
        "==========================================")
    print(
        "================================  Godot  Bridge  "
        "==========================================")
    print(
        "================================ @@@@@@@@@@@@@@@ "
        "==========================================")
    print(
        "================================ @@@@@@@@@@@@@@@ "
        "==========================================")
    print(
        "================================ @@@@@@@@@@@@@@@ "
        "==========================================")
    print(
        "================================ @@@@@@@@@@@@@@@ "
        "==========================================")

    # FEAGI section start
    print("Connecting to FEAGI resources...")
    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    print("FEAGI AUTH URL ------- ", feagi_auth_url)
    FEAGI_HOST, API_PORT, app_data_port = feagi.feagi_setting_for_registration(feagi_settings,
                                                                               agent_settings)
    print("app_data_port: ", app_data_port)
    runtime_data["feagi_state"] = feagi.feagi_registration(feagi_auth_url=feagi_auth_url,
                                                           feagi_settings=feagi_settings,
                                                           agent_settings=agent_settings,
                                                           capabilities=capabilities)
    agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
    print("** **", runtime_data["feagi_state"])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])
    ipu_channel_address = "tcp://*:" + agent_data_port
    print("ipu: ", ipu_channel_address)
    opu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                               runtime_data["feagi_state"]['feagi_opu_port'])
    feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=True)
    # FEAGI section ends

    print("FEAGI initialization completed successfully")
    godot_list = {}  # initialized the list from Godot
    detect_lag = False
    new_feagi_sub = feagi.sub_initializer(opu_address=opu_channel_address)
    flag_zmq = False
    connect_status_counter = 0
    old_data = []
    while True:
        if detect_lag:
            opu_channel_address = 'tcp://' + feagi_settings['feagi_host'] + ':' + \
                                  runtime_data["feagi_state"][
                                      'feagi_opu_port']
            new_feagi_sub = feagi.sub_initializer(opu_address=opu_channel_address)
            zmq_queue.clear()
            ws_queue.clear()
            detect_lag = False
        one_frame = new_feagi_sub.receive()
        if not flag_zmq:
            if one_frame is not None:
                connect_status_counter = 0
            else:
                connect_status_counter += 1
                if connect_status_counter >= 600000:
                    if feagi.is_FEAGI_reachable(
                            os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
                            int(os.environ.get('FEAGI_OPU_PORT', "3000"))):
                        connect_status_counter = 0
                    else:
                        zmq_queue.append("clear")
                        break
        if one_frame is not None:
            if flag_zmq:
                # FEAGI section start
                print("Connecting to FEAGI resources...")
                feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
                print("FEAGI AUTH URL ------- ", feagi_auth_url)
                FEAGI_HOST, API_PORT, app_data_port = feagi.feagi_setting_for_registration(
                    feagi_settings,
                    agent_settings)
                runtime_data["feagi_state"] = feagi.feagi_registration(
                    feagi_auth_url=feagi_auth_url,
                    feagi_settings=feagi_settings,
                    agent_settings=agent_settings,
                    capabilities=capabilities)
                # agent_data_port = agent_settings["agent_data_port"]
                agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
                print("** **", runtime_data["feagi_state"])
                if isinstance(runtime_data.get("feagi_state"), dict) and 'burst_duration' in \
                        runtime_data["feagi_state"]:
                    feagi_settings['feagi_burst_speed'] = float(
                        runtime_data["feagi_state"]['burst_duration'])
                ipu_channel_address = "tcp://*:" + agent_data_port
                feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=True)
                # FEAGI section ends
                flag_zmq = False
            if one_frame["genome_changed"] != PREVIOUS_GENOME_TIMESTAMP:
                PREVIOUS_GENOME_TIMESTAMP = one_frame["genome_changed"]
                runtime_data["cortical_data"] = \
                    requests.get(
                        'http://' + FEAGI_HOST + ':' + API_PORT + DIMENSIONS_ENDPOINT,
                        timeout=10).json()
                if one_frame["genome_changed"] is not None:
                    print("updated time")
                    zmq_queue.append("updated")
            BURST_SECOND = one_frame['burst_frequency']
            if 'genome_reset' in one_frame:
                runtime_data["cortical_data"] = {}
            one_frame = feagi_breakdown(one_frame)
            # Debug section start
            if one_frame != old_data:
                old_data = one_frame
            # Debug section end
            # one_frame = simulation_testing() # This is to test the stress
            if BURST_SECOND > agent_settings['burst_duration_threshold']:
                zmq_queue.append(one_frame)
        if ws_queue:
            data_from_godot = ws_queue[0].decode(
                'UTF-8')  # ADDED this line to decode into string only
            ws_queue.pop()
        else:
            data_from_godot = "{}"
        # print("DATA FROM GODOT: ", data_from_godot)
        if data_from_godot != "{}":
            print(data_from_godot)
        if data_from_godot == "lagged":
            detect_lag = True
            data_from_godot = "{}"
        if data_from_godot == "empty":
            print("EMPTY!")
            data_from_godot = "{}"
            data_from_genome = requests.get('http://' + FEAGI_HOST + ':' + API_PORT +
                                            '/v1/feagi/connectome/properties/dimensions',
                                            timeout=10).json()
            json_object = json.dumps(data_from_genome)
            zmq_queue.append("genome: " + json_object)
        if data_from_godot == "updated":
            data_from_godot = "{}"
            reload_genome()
            runtime_data["cortical_data"] = \
                requests.get('http://' + FEAGI_HOST + ':' + API_PORT + DIMENSIONS_ENDPOINT,
                             timeout=10).json()
        if "cortical_name" in data_from_godot:
            url = "http://" + FEAGI_HOST + ":" + API_PORT + "/v1/feagi/genome/cortical_area"
            request_obj = data_from_godot
            requests.post(url, data=request_obj, timeout=10)
            data_from_godot = {}

        invalid_values = {"None", "{}", "refresh", "[]"}
        if data_from_godot not in invalid_values and data_from_godot != godot_list:
            godot_list = godot_data(data_from_godot)
            converted_data = convert_absolute_to_relative_coordinate(
                stimulation_from_godot=godot_list,
                cortical_data=runtime_data[
                    "cortical_data"])
            print("raw data from godot:", godot_list)
            print(">>> > > > >> > converted data:", converted_data)
            feagi_ipu_channel.send(converted_data)
            godot_list = {}
            converted_data = {}

        if data_from_godot == "refresh":
            godot_list = {}
            converted_data = {}
            feagi_ipu_channel.send(godot_list)
        else:
            pass


if __name__ == "__main__":
    threading.Thread(target=websocket_operation, daemon=True).start()
    while True:
        PREVIOUS_GENOME_TIMESTAMP = 0
        FEAGI_FLAG = False
        print("Waiting on FEAGI...")
        while not FEAGI_FLAG:
            FEAGI_FLAG = feagi.is_FEAGI_reachable(
                os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
                int(os.environ.get('FEAGI_OPU_PORT', "3000")))
            sleep(2)
        main()
