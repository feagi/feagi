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
import traceback
import sys
import configuration
import router
import zmq
import json
import ast
import asyncio
import websockets
import requests
import random
import shutil
import threading
from time import sleep
from router import Pub, Sub
from configuration import *
import concurrent.futures
from threading import Thread
from collections import deque

ws_queue = deque()
zmq_queue = deque()
burst_second = 0
previous_genome_timestamp = 0  # TO keep record of timestamp
current_cortical_area = dict()

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

dimensions_endpoint = '/v1/feagi/connectome/properties/dimensions'


def simulation_testing():
    x = 0
    y = 0
    z = 0
    array = []
    for i in range(1000):
        x = random.randint(0, 64)
        y = random.randint(0, 64)
        z = random.randint(0, 64)
        array.append((x, y, z))
    return array


def breakdown(feagi_input):
    """
    #TODO: Explain what this is for
    """
    data = feagi_input
    data = list(data)
    increment = 0
    list1 = []

    while increment < len(data):
        voxel = [data[increment][1], data[increment][2], data[increment][3]]
        list1.append(voxel)
        increment += 1
    print(list1)


def godot_data(input):
    """
    Simply clean the list and remove all unnecessary special characters and deliver with name, xyz only
    """
    data = ast.literal_eval(input)
    dict_with_updated_name = {"data": dict()}
    dict_with_updated_name["data"]["direct_stimulation"] = dict(dict())
    for key in data["data"]["direct_stimulation"]:
        Updated_name = name_to_id(key)
        if dict_with_updated_name["data"]["direct_stimulation"].get(Updated_name) is not None:
            pass
        else:
            dict_with_updated_name["data"]["direct_stimulation"][Updated_name] = []
        for key_01 in data["data"]["direct_stimulation"][key]:
            dict_with_updated_name["data"]["direct_stimulation"][Updated_name].append(key_01)

    print("godot_data: ", dict_with_updated_name)
    return dict_with_updated_name


def godot_selected_list(outside_list, godot_list):
    name = outside_list[0]
    x = int(outside_list[1])
    y = int(outside_list[2])
    z = int(outside_list[3])
    if godot_list:
        list_to_dict = godot_list
    else:
        list_to_dict = dict()
        list_to_dict["data"] = dict()
        list_to_dict["data"]["direct_stimulation"] = dict()

    if list_to_dict["data"]["direct_stimulation"].get(name) is not None:
        pass
    else:
        list_to_dict["data"]["direct_stimulation"][name] = list()
    for key in list_to_dict["data"]["direct_stimulation"]:
        if key not in list_to_dict["data"]["direct_stimulation"]:
            list_to_dict["data"]["direct_stimulation"][name] = list()
            list_to_dict["data"]["direct_stimulation"][name].append([x, y, z])
        else:
            list_to_dict["data"]["direct_stimulation"][name].append([x, y, z])
    return list_to_dict


def godot_deselected_list(outside_list, godot_list):
    for key in godot_list["data"]["direct_stimulation"]:
        if outside_list[0] == key:
            for xyz in godot_list["data"]["direct_stimulation"][key]:
                if xyz[0] == int(outside_list[1]) and xyz[1] == int(outside_list[2]) and xyz[2] == int(outside_list[3]):
                    godot_list["data"]["direct_stimulation"][key].remove(xyz)
    return godot_list


def name_to_id(name):
    for cortical_area in runtime_data["cortical_data"]:
        if cortical_area == name:
            return runtime_data["cortical_data"][cortical_area][7]
    else:
        # pass
        print("*** Failed to find cortical name ***")


def feagi_breakdown(data):
    """
    Designed for genome 2.0 only. Data is the input from feagi's raw data.
    This function will detect if cortical area list is different than the first, it will generate genome list for godot
     automatically.
    """
    new_list = []
    new_genome_num = data['genome_num']
    if new_genome_num > runtime_data["genome_number"]:
        runtime_data["old_cortical_data"] = runtime_data["cortical_data"]
        runtime_data["cortical_data"] = \
            requests.get('http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()
        if 'genome_reset' not in data and data == "{}":
            runtime_data["cortical_data"] = \
                requests.get('http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()
        if data != "{}":
            if runtime_data["old_cortical_data"] != runtime_data["cortical_data"]:
                pass  # TODO: add to detect if cortical is changed
            #     csv_writer(runtime_data["cortical_data"])
        runtime_data["genome_number"] = new_genome_num
    for i in data['godot']:
        xyz = i[1], i[2], i[3]
        new_list.append(xyz)
    return new_list


def convert_absolute_to_relative_coordinate(stimulation_from_godot, cortical_data):
    """
    Convert absolute coordinate from godot to relative coordinate for FEAGI. Dna_information is from the
    genome["blueprint"].
    """
    relative_coordinate = {}
    relative_coordinate["data"] = dict()
    relative_coordinate["data"]["direct_stimulation"] = dict()
    if stimulation_from_godot:
        for key in stimulation_from_godot["data"]["direct_stimulation"]:
            for name_match in cortical_data:
                raw_id = name_match
                name_match = name_to_id(name_match)  # convert the human readable name into feagi name
                if name_match == key:
                    if relative_coordinate["data"]["direct_stimulation"].get(name_match) is not None:
                        pass
                    else:
                        relative_coordinate["data"]["direct_stimulation"][name_match] = list()
                    for xyz in stimulation_from_godot["data"]["direct_stimulation"][name_match]:
                        new_xyz = [xyz[0] - cortical_data[raw_id][0], xyz[1] - cortical_data[raw_id][1],
                                   xyz[2] - cortical_data[raw_id][2]]
                        relative_coordinate["data"]["direct_stimulation"][name_match].append(new_xyz)

            else:
                pass
    else:
        pass

    return relative_coordinate


def feagi_registration(feagi_host, api_port):
    host_info = router.app_host_info()
    runtime_data = {
        "host_network": {},
        "feagi_state": None
    }
    runtime_data["host_network"]["host_name"] = host_info["host_name"]
    runtime_data["host_network"]["ip_address"] = host_info["ip_address"] if \
        configuration.host_info["ip_address"] is None else configuration.host_info["ip_address"]

    while runtime_data["feagi_state"] is None:
        print("\nAwaiting registration with FEAGI...")
        try:
            print("MNM")
            runtime_data["feagi_state"] = \
                router.register_with_feagi(feagi_ip=feagi_host,
                                           feagi_api_port=api_port,
                                           agent_type=configuration.agent_settings['agent_type'],
                                           agent_id=configuration.agent_settings['agent_id'],
                                           agent_ip=runtime_data["host_network"]["ip_address"],
                                           agent_data_port=configuration.agent_settings['agent_data_port'],
                                           agent_capabilities=configuration.capabilities)
        except Exception as e:
            print("ERROR__: ", e, traceback.print_exc())
            pass
        sleep(1)
    return runtime_data["feagi_state"]


def reload_genome():
    bool_flag = True
    cortical_name = []
    cortical_genome_dictionary = {"genome": {}}
    print("+++ 0 +++")
    while bool_flag:
        if len(cortical_genome_dictionary["genome"]) == 0:
            print("+++ a +++")
            try:
                data_from_genome = requests.get('http://' + feagi_host + ':' + api_port +
                                                '/v1/feagi/genome/download').json()
                cortical_area_name = requests.get('http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()
                runtime_data["cortical_data"] = data_from_genome
                print("cortical_area_name: ", cortical_area_name)

                for x in cortical_area_name:
                    # print("worked: ", x)
                    cortical_name.append(cortical_area_name[x][7])

                # print(50 * "*")
                # print(runtime_data["cortical_data"])
                # print(50 * "*")
                # print(50 * "#")
                # print(cortical_genome_dictionary)
                # print(50 * "#")

                if runtime_data["cortical_data"]:
                    for i in runtime_data["cortical_data"]["blueprint"]:
                        # print("i: ", runtime_data["cortical_data"]["blueprint"][i], " and solo i: ", i)
                        for x in cortical_name:
                            # print("CURRENT: ", cortical_name)
                            if x in i:
                                if x not in cortical_genome_dictionary['genome']:
                                    # print("NOT FOUND: ", x)
                                    cortical_genome_dictionary['genome'][x] = list()
                                cortical_genome_dictionary['genome'][x].append(
                                    runtime_data["cortical_data"]["blueprint"][i])
                    json_object = json.dumps(cortical_genome_dictionary)
                    zmq_queue.append(json_object)
                    # zmq_queue.append(cortical_genome_dictionary)
            except Exception as e:
                bool_flag = True
                print("Error while fetching genome from FEAGI\n", traceback.print_exc())
        else:
            bool_flag = False
        time.sleep(2)
    print("Genome reloaded.")
    if len(ws_queue[0]) > 2:
        ws_queue.clear()
    # print("ws queue: ", len(ws_queue[0]))
    return cortical_genome_dictionary.copy()


def feagi_init(feagi_host, api_port):
    # Send a request to FEAGI for cortical dimensions
    awaiting_feagi_registration = True
    while awaiting_feagi_registration:
        print("********* ************ ********** ************* ***************\n")
        print("Awaiting registration with FEAGI...2")
        FEAGI_pub.send({"godot_init": True})

        # print("Cortical_data", runtime_data["cortical_data"])

        cortical_name = []
        cortical_genome_dictionary = {"genome": {}}
        while awaiting_feagi_registration:
            if len(cortical_genome_dictionary["genome"]) == 0:
                try:
                    data_from_genome = requests.get('http://' + feagi_host + ':' + api_port +
                                                    '/v1/feagi/genome/download').json()
                    cortical_area_name = requests.get(
                        'http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()
                    runtime_data["cortical_data"] = data_from_genome

                    for x in cortical_area_name:
                        cortical_name.append(cortical_area_name[x][7])

                    if runtime_data["cortical_data"]:
                        for i in runtime_data["cortical_data"]["blueprint"]:
                            for x in cortical_name:
                                if x in i:
                                    if x not in cortical_genome_dictionary['genome']:
                                        cortical_genome_dictionary['genome'][x] = list()
                                    cortical_genome_dictionary['genome'][x].append(
                                        runtime_data["cortical_data"]["blueprint"][i])
                        json_object = json.dumps(cortical_genome_dictionary)
                        # zmq_queue.append(json_object)
                        # zmq_queue.append(cortical_genome_dictionary)
                except Exception as e:
                    awaiting_feagi_registration = True
                    print("Error while fetching genome from FEAGI\n", traceback.print_exc())
            else:
                awaiting_feagi_registration = False
            time.sleep(2)
        return cortical_genome_dictionary.copy()


async def echo(websocket):
    while True:
        try:
            if "genome" in zmq_queue[0]:
                cortical_genome_list = str(zmq_queue[0])
                zmq_queue.pop()
                cortical_area_name = requests.get('http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()
                await websocket.send(cortical_genome_list)
            if len(zmq_queue) > 2:  # This will eliminate any stack up queue
                stored_value = zmq_queue[len(zmq_queue) - 1]
                zmq_queue.clear()
                zmq_queue[0] = stored_value
            await websocket.send(str(zmq_queue[0]))
            zmq_queue.pop()
        except Exception as e:
            pass
        new_data = await websocket.recv()
        ws_queue.append(new_data)


async def websocket_main():
    async with websockets.serve(echo, configuration.agent_settings["godot_websocket_ip"],
                                configuration.agent_settings['godot_websocket_port'], max_size=None,
                                max_queue=None, write_limit=None, compression=None):
        await asyncio.Future()


def websocket_operation():
    asyncio.run(websocket_main())


if __name__ == "__main__":
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================  Godot  Bridge  ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")

    feagi_host = configuration.feagi_settings["feagi_host"]
    api_port = configuration.feagi_settings["feagi_api_port"]

    runtime_data["feagi_state"] = feagi_registration(feagi_host=feagi_host, api_port=api_port)

    api_address = 'http://' + feagi_settings['feagi_host'] + ':' + feagi_settings['feagi_api_port']

    sockets = requests.get(api_address + '/v1/feagi/feagi/network').json()
    stimulation_period = requests.get(api_address + '/v1/feagi/feagi/burst_engine/stimulation_period').json()
    runtime_data["feagi_state"]['feagi_burst_speed'] = float(stimulation_period)

    bgsk = threading.Thread(target=websocket_operation, daemon=True).start()

    ipu_channel_address = f"tcp://*:{agent_settings['agent_data_port']}"
    # ipu_channel_address = f"tcp://{feagi_host}:{agent_settings["agent_data_port"]}"
    FEAGI_pub = Pub(ipu_channel_address, bind=True)

    opu_channel_address = f"tcp://{feagi_settings['feagi_host']}:{runtime_data['feagi_state']['feagi_opu_port']}"
    FEAGI_sub = Sub(address=opu_channel_address, bind=False, flags=zmq.NOBLOCK)

    current_cortical_area = feagi_init(feagi_host=feagi_host, api_port=api_port)
    print("FEAGI initialization completed successfully")
    godot_list = {}  # initialized the list from Godot
    detect_lag = False
    new_FEAGI_sub = FEAGI_sub
    flag = 0
    data_from_genome = 0
    one_frame = new_FEAGI_sub.receive()
    # previous_genome_timestamp = one_frame["genome_changed"]
    while True:
        if detect_lag:
            opu_channel_address = 'tcp://' + feagi_settings['feagi_host'] + ':' + runtime_data["feagi_state"][
                'feagi_opu_port']
            new_FEAGI_sub = Sub(address=opu_channel_address, bind=False, flags=zmq.NOBLOCK)
            zmq_queue.clear()
            ws_queue.clear()
            detect_lag = False
        one_frame = new_FEAGI_sub.receive()

        if one_frame is not None:
            if one_frame["genome_changed"] != previous_genome_timestamp:
                previous_genome_timestamp = one_frame["genome_changed"]
                runtime_data["cortical_data"] = \
                    requests.get('http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()
                print("updated time")
                zmq_queue.append("updated")
            burst_second = one_frame['burst_frequency']
            if 'genome_reset' in one_frame:
                runtime_data["cortical_data"] = {}
                try:
                    f = open("../godot_source/reset.txt", "w")
                    f.write("reset")
                    f.close()
                except Exception as e:
                    print("Error during genome reset:\n", e)
            one_frame = feagi_breakdown(one_frame)
            # one_frame = simulation_testing() # This is to test the stress
            if burst_second > agent_settings['burst_duration_threshold']:
                zmq_queue.append(one_frame)
        if ws_queue:
            data_from_godot = ws_queue[0].decode('UTF-8')  # ADDED this line to decode into string only
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
            # json_object = {}
            data_from_godot = "{}"
            data_from_genome = requests.get('http://' + feagi_host + ':' + api_port +
                                            '/v1/feagi/connectome/properties/dimensions').json()
            json_object = json.dumps(data_from_genome)
            zmq_queue.append("genome: " + json_object)
        if data_from_godot == "updated":
            data_from_godot = "{}"
            reload_genome()
            runtime_data["cortical_data"] = \
                requests.get('http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()
        # if "new" in data_from_godot:
        #     json_object = json.dumps(data_from_godot)
        #     print(json_object)
        if "cortical_name" in data_from_godot:
            # data_from_godot = data_from_godot.replace("relocate", "\"relocate\"")
            url = "http://127.0.0.1:8000/v1/feagi/genome/cortical_area"
            request_obj = data_from_godot
            requests.post(url, data=request_obj)
            data_from_godot = {}

        if data_from_godot != "None" and data_from_godot != "{}" and data_from_godot != godot_list and data_from_godot \
                != "refresh" and data_from_godot != "[]":
            godot_list = godot_data(data_from_godot)
            converted_data = convert_absolute_to_relative_coordinate(stimulation_from_godot=godot_list,
                                                                     cortical_data=runtime_data[
                                                                         "cortical_data"])
            print("raw data from godot:", godot_list)
            print(">>> > > > >> > converted data:", converted_data)
            FEAGI_pub.send(converted_data)
            godot_list = {}
            converted_data = {}

        if data_from_godot == "refresh":
            godot_list = {}
            converted_data = {}
            FEAGI_pub.send(godot_list)
        else:
            pass
