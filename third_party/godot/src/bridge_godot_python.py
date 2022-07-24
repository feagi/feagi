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
from router import *
from configuration import *

import sys
import socket
import configuration
import router
import zmq
import csv
import ast
import asyncio
import websockets
import requests
import traceback
import shutil
from threading import Thread
from time import sleep

runtime_data = {
    "cortical_data": {},
    "current_burst_id": None,
    "stimulation_period": None,
    "feagi_state": None,
    "feagi_network": None,
    "cortical_list": set(),
    "host_network": {},
    "genome_number": 0
}

dimensions_endpoint = '/v1/feagi/connectome/properties/dimensions'
feagi_host = configuration.network_settings["feagi_host"]
api_port = configuration.network_settings["feagi_api_port"]


#  CSV Handlers
def csv_writer(cortical_dimensions):
    print("Generating CSV...")
    cwd = os.getcwd()
    print("Current path is:", cwd)
    f = open('../godot_source/csv_data.gdc', 'w', newline='')
    writer = csv.writer(f)
    godot_cortical_dimensions = list()
    for cortical_area in cortical_dimensions:
        if cortical_dimensions[cortical_area][3]:
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][0])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][1])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][2])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][4])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][5])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][6])
            godot_cortical_dimensions.append(cortical_area)
            writer.writerow(godot_cortical_dimensions)
            godot_cortical_dimensions = list()
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++")

    print("Godot CSV has been created.")


def genome_reset():
    # todo This function assumes the path structure and launch point dictated in the dockerfile
    try:
        if os.path.exists('../html/index.js'):
            shutil.rmtree("../html")
            print("HTML folder is deleted")
            os.mkdir('../html')
            print("HTML folder created")
            for file in os.listdir('../html_backup/'):
                shutil.copy('../html_backup/' + file, '../html/')
                print("All backup files copied to HTML folder")

    except Exception as e:
        print("Error during genome reset:\n", traceback.print_exc())


# Data Processors
def name_to_id(name):
    for cortical_area in runtime_data["cortical_data"]:
        if cortical_area == name:
            return runtime_data["cortical_data"][cortical_area][7]
    else:
        pass


def godot_data_processor(input):
    """
    Simply clean the list and remove all unnecessary special characters and deliver with name, xyz only
    """
    data = ast.literal_eval(input)
    dict_with_updated_name = {}
    dict_with_updated_name["data"] = dict()
    dict_with_updated_name["data"]["direct_stimulation"] = dict(dict())
    for key in data["data"]["direct_stimulation"]:
        Updated_name = name_to_id(key)
        if dict_with_updated_name["data"]["direct_stimulation"].get(Updated_name) is not None:
            pass
        else:

            dict_with_updated_name["data"]["direct_stimulation"][Updated_name] = []
        for key_01 in data["data"]["direct_stimulation"][key]:
            dict_with_updated_name["data"]["direct_stimulation"][Updated_name].append(key_01)

    print("godot_data_processor: ", dict_with_updated_name)
    return dict_with_updated_name


def feagi_data_processor(data):
    """
    Designed for genome 2.0 only. Data is the input from feagi's raw data
    """
    new_list = []
    new_genome_num = data['genome_num']
    print("Previous genome #:", runtime_data["genome_number"])
    print("Latest genome #:", new_genome_num)

    if 'genome_reset' in data:
        print("+" * 60)
        print("\nGenome reset command received!")
        genome_reset()

    if new_genome_num == 1:
        runtime_data["genome_number"] = 1

    elif new_genome_num > runtime_data["genome_number"]:
        runtime_data["genome_number"] = new_genome_num
        try:
            runtime_data["cortical_data"] = \
                requests.get('http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()
            csv_writer(runtime_data["cortical_data"])
            sleep(2)
        except Exception as e:
            print("Error during CSV creation\n", e)

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
                name_match = name_to_id(name_match)  # convert the human-readable name into feagi name
                if name_match == key:
                    if relative_coordinate["data"]["direct_stimulation"].get(name_match) is not None:
                        pass
                    else:
                        relative_coordinate["data"]["direct_stimulation"][name_match] = list()
                    for xyz in stimulation_from_godot["data"]["direct_stimulation"][name_match]:
                        new_xyz = [xyz[0] - cortical_data[raw_id][0],
                                   xyz[1] - cortical_data[raw_id][1],
                                   xyz[2] - cortical_data[raw_id][2]]
                        relative_coordinate["data"]["direct_stimulation"][name_match].append(new_xyz)

            else:
                pass
    else:
        pass

    return relative_coordinate


# FEAGI Registration
def feagi_registration():
    app_host_info_ = router.app_host_info()
    runtime_data["host_network"]["host_name"] = app_host_info_["host_name"]
    runtime_data["host_network"]["ip_address"] = app_host_info_["ip_address"]

    while runtime_data["feagi_state"] is None:
        print("Awaiting registration with FEAGI...1")
        try:
            runtime_data["feagi_state"] = router.register_with_feagi(app_name=configuration.app_name,
                                                                     feagi_host=feagi_host,
                                                                     api_port=api_port,
                                                                     app_capabilities=configuration.capabilities,
                                                                     app_host_info=runtime_data["host_network"]
                                                                     )
        except Exception as e:
            print("Error: The following occurred during registration with FEAGI\n", e)
            pass
        sleep(1)

    print("** **", runtime_data["feagi_state"])
    api_address = 'http://' + network_settings['feagi_host'] + ':' + network_settings['feagi_api_port']
    sockets = requests.get(api_address + '/v1/feagi/feagi/network').json()
    stimulation_period = requests.get(api_address + '/v1/feagi/feagi/burst_engine/stimulation_period').json()
    runtime_data["feagi_state"]['feagi_burst_speed'] = float(stimulation_period)

    # Establish ZMQ Sessions with FEAGI
    print("--->> >> >> \n", sockets, network_settings)
    feagi_pub = Pub(address='tcp://0.0.0.0:' + runtime_data["feagi_state"]['feagi_inbound_port_godot'])
    opu_channel_address = 'tcp://' + network_settings['feagi_host'] + ':' + runtime_data["feagi_state"]['feagi_outbound_port']
    feagi_sub = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)

    # Send a request to FEAGI for cortical dimensions
    awaiting_feagi_registration = True

    while awaiting_feagi_registration:
        print("********* ************ ********** ************* ***************\n")
        print("Awaiting registration with FEAGI...2")
        feagi_pub.send({"godot_init": True})

        runtime_data["cortical_data"] = \
            requests.get('http://' + feagi_host + ':' + api_port + dimensions_endpoint).json()

        print("Cortical_data", runtime_data["cortical_data"])

        if runtime_data["cortical_data"]:
            print("###### ------------------------------------------------------------#######")
            print("Cortical Dimensions:\n", runtime_data["cortical_data"])
            csv_writer(runtime_data["cortical_data"])
            awaiting_feagi_registration = False
        time.sleep(1)

    return feagi_pub, feagi_sub

# Communication Handling
async def echo(websocket):
    godot_list = {}  # initalized the list from Godot
    detect_lag = False
    new_FEAGI_sub = feagi_sub
    while True:
        one_frame = new_FEAGI_sub.receive()

        if one_frame is not None:
            one_frame = feagi_data_processor(one_frame)
            await websocket.send(str(one_frame))
            try:
                await websocket.send(str(one_frame))
            except Exception as e:
                print("Error during websocket processing:\n   ", e)

        if detect_lag:
            opu_channel_address = 'tcp://' + network_settings['feagi_host'] + ':' + runtime_data["feagi_state"][
                'feagi_outbound_port']
            new_FEAGI_sub = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)
            detect_lag = False

        data_from_godot = await websocket.recv()
        data_from_godot = data_from_godot.decode('UTF-8')  # ADDED this line to decode into string only
        if data_from_godot == "lagged":
            detect_lag = True
            data_from_godot = "{}"
        if (data_from_godot != "None" and data_from_godot != "{}" and data_from_godot != godot_list and data_from_godot != "refresh" and data_from_godot != "[]"):
            print(data_from_godot)
            godot_list = godot_data_processor(data_from_godot)
            converted_data = convert_absolute_to_relative_coordinate(stimulation_from_godot=godot_list,
                                                                     cortical_data=runtime_data[
                                                                         "cortical_data"])
            print(">>> > > > >> > converted data:", converted_data)
            feagi_pub.send(converted_data)
        if data_from_godot == "refresh":
            godot_list = {}
            converted_data = {}
            feagi_pub.send(godot_list)
        else:
            pass


async def main():
    print("- -- -- --  --- -- -- ")
    async with websockets.serve(echo, "0.0.0.0", configuration.network_settings['godot_websocket_port']):
        print("+++")
        await asyncio.Future()  # run forever


# def breakdown(feagi_input):  # add input soon
#     """
#     # TODO: Explain what this is for
#     """
#     data = feagi_input
#     data = list(data)
#     increment = 0
#     list1 = []
#
#     while increment < len(data):
#         voxel = [data[increment][1], data[increment][2], data[increment][3]]
#         list1.append(voxel)
#         increment += 1
#     print(list1)

# def godot_selected_list(outside_list, godot_list):
#     name = outside_list[0]
#     x = int(outside_list[1])
#     y = int(outside_list[2])
#     z = int(outside_list[3])
#     if godot_list:
#         list_to_dict = godot_list
#     else:
#         list_to_dict = dict()
#         list_to_dict["data"] = dict()
#         list_to_dict["data"]["direct_stimulation"] = dict()
#
#     if list_to_dict["data"]["direct_stimulation"].get(name) is not None:
#         pass
#     else:
#         list_to_dict["data"]["direct_stimulation"][name] = list()
#     for key in list_to_dict["data"]["direct_stimulation"]:
#         if key not in list_to_dict["data"]["direct_stimulation"]:
#             list_to_dict["data"]["direct_stimulation"][name] = list()
#             list_to_dict["data"]["direct_stimulation"][name].append([x,y,z])
#         else:
#             list_to_dict["data"]["direct_stimulation"][name].append([x,y,z])
#     return list_to_dict

# def godot_deselected_list(outside_list, godot_list):
#     for key in godot_list["data"]["direct_stimulation"]:
#         if outside_list[0] == key:
#             for xyz in godot_list["data"]["direct_stimulation"][key]:
#                 if xyz[0] == int(outside_list[1]) and xyz[1] == int(outside_list[2]) and xyz[2] == int(outside_list[3]):
#                     godot_list["data"]["direct_stimulation"][key].remove(xyz)
#     return godot_list


async def feagi_to_godot():
    pass


async def godot_to_feagi():
    pass


if __name__ == "__main__":
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================  Godot  Bridge  ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")
    print("================================ @@@@@@@@@@@@@@@ ==========================================")

    # Registration with FEAGI
    feagi_pub, feagi_sub = feagi_registration()
    print("FEAGI registration completed successfully")

    Thread(target=feagi_to_godot).start()
    Thread(target=godot_to_feagi).start()

    asyncio.run(main())
