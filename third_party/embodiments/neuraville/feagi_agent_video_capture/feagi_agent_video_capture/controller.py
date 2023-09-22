#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

import cv2
import feagi_agent.feagi_interface
import requests
from time import sleep
from datetime import datetime
from feagi_agent import retina as retina
from feagi_agent import pns_gateway as pns
from feagi_agent import feagi_interface as feagi
import traceback
import threading
import time
import pickle
import lz4.frame
import mss
import screeninfo
import numpy
import multiprocessing
from PIL import Image

camera_data = {"vision": {}}


def check_aptr():
    try:
        raw_aptr = requests.get(get_size_for_aptr_cortical).json()
        return raw_aptr['cortical_dimensions'][2]
    except Exception as error:
        print("error: ", error)
        return 10


def process_video(video_path, capabilities):
    cam = cv2.VideoCapture(video_path)
    screen_info = screeninfo.get_monitors()[0]  # Assuming you want the primary monitor
    screen_width = 600
    screen_height = 600
    monitor = {"top": 40, "left": 0, "width": screen_width, "height": screen_height}
    pixels = []
    while True:
        if capabilities['camera']['video_device_index'] != "monitor":
            check, pixels = cam.read()
        else:
            check = True
        if capabilities['camera']['video_device_index'] != "monitor":
            if bool(capabilities["camera"]["video_loop"]):
                if check:
                    pass
                else:
                    print("check status: ", check)
                    cam.set(cv2.CAP_PROP_POS_FRAMES, 0)
        if capabilities['camera']['video_device_index'] == "monitor":
            with mss.mss() as sct:
                img = numpy.array(sct.grab(monitor))
                pixels = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
                cv2.imshow("OpenCV/Numpy normal", pixels)
            cv2.waitKey(25)
        else:
            if check:
                # cv2.imshow("test", pixels)
                cv2.waitKey(30)
        if capabilities['camera']['current_select']:
            dim = (capabilities['camera']['current_select'][0], capabilities['camera'][
                'current_select'][1])
            pixels = cv2.resize(pixels, dim, interpolation=cv2.INTER_AREA)
            camera_data["vision"] = pixels
        else:
            camera_data["vision"] = pixels

    cam.release()
    cv2.destroyAllWindows()


def main(feagi_auth_url, feagi_settings, agent_settings, capabilities, message_to_feagi):
    # Generate runtime dictionary
    previous_data_frame = dict()
    runtime_data = {"vision": {}, "current_burst_id": None, "stimulation_period": None,
                    "feagi_state": None,
                    "feagi_network": None}
    feagi_flag = False
    print("retrying...")
    print("Waiting on FEAGI...")
    while not feagi_flag:
        feagi_flag = feagi.is_FEAGI_reachable(feagi_settings["feagi_host"], 30000)
        sleep(2)
    burst_counter_endpoint = feagi.feagi_api_burst_counter()
    # FEAGI section start
    print("Connecting to FEAGI resources...")
    runtime_data["feagi_state"] = feagi.feagi_registration(feagi_auth_url=feagi_auth_url,
                                                           feagi_settings=feagi_settings,
                                                           agent_settings=agent_settings,
                                                           capabilities=capabilities)
    print("FEAGI REGISTERED!")
    api_address = runtime_data['feagi_state']["feagi_url"]

    # agent_data_port = agent_settings["agent_data_port"]
    agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
    print("** **", runtime_data["feagi_state"])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

    # todo: to obtain this info directly from FEAGI as part of registration
    # ipu_channel_address = feagi.feagi_inbound(agent_settings["agent_data_port"])
    ipu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'], agent_data_port)
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                               runtime_data["feagi_state"]['feagi_opu_port'])

    feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=False)
    feagi_opu_channel = feagi.sub_initializer(opu_address=opu_channel_address)
    # FEAGI section ends
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    rgb = dict()
    capabilities['camera']['current_select'] = []
    rgb['camera'] = dict()
    genome_tracker = 0
    get_size_for_aptr_cortical = api_address + '/v1/feagi/genome/cortical_area?cortical_area=o_aptr'
    raw_aptr = requests.get(get_size_for_aptr_cortical).json()
    try:
        aptr_cortical_size = raw_aptr['cortical_dimensions'][2]
    except:
        aptr_cortical_size = None
    threading.Thread(target=process_video, args=(
        capabilities['camera']['video_device_index'],
        capabilities), daemon=True).start()

    while True:
        try:
            compressed_data = feagi_opu_channel.receive()  # Get data from FEAGI
            if compressed_data is not None:
                decompressed_data = lz4.frame.decompress(compressed_data)
                message_from_feagi = pickle.loads(decompressed_data)
            else:
                message_from_feagi = None

            pixels = camera_data['vision']
            previous_data_frame, rgb['camera'], capabilities['camera']['current_select'] = \
                pns.generate_rgb(pixels,
                                capabilities['camera']['central_vision_allocation_percentage'][0],
                                capabilities['camera']['central_vision_allocation_percentage'][1],
                                capabilities['camera']["central_vision_resolution"],
                                capabilities['camera']['peripheral_vision_resolution'],
                                previous_data_frame,
                                capabilities['camera']['current_select'],
                                capabilities['camera']['iso_default'],
                                capabilities['camera']["aperture_default"])

            if message_from_feagi is not None:
                # OPU section STARTS
                if 'genome_num' in message_from_feagi:
                    if message_from_feagi['genome_num'] != genome_tracker:
                        genome_tracker = message_from_feagi['genome_num']

                if "o_aptr" in message_from_feagi["opu_data"]:
                    if message_from_feagi["opu_data"]["o_aptr"]:
                        for i in message_from_feagi["opu_data"]["o_aptr"]:
                            feagi_aptr = (int(i.split('-')[-1]))
                            if aptr_cortical_size is None:
                                aptr_cortical_size = check_aptr()
                            elif aptr_cortical_size <= feagi_aptr:
                                aptr_cortical_size = check_aptr()
                            max_range = capabilities['camera']['aperture_range'][1]
                            min_range = capabilities['camera']['aperture_range'][0]
                            capabilities['camera']["aperture_default"] = \
                                ((feagi_aptr / aptr_cortical_size) *
                                 (max_range - min_range)) + min_range
                if "o__dev" in message_from_feagi["opu_data"]:
                    if message_from_feagi["opu_data"]["o__dev"]:
                        for i in message_from_feagi["opu_data"]["o__dev"]:
                            feagi_aptr = (int(i.split('-')[-1]))
                            if aptr_cortical_size is None:
                                aptr_cortical_size = check_aptr()
                            elif aptr_cortical_size <= feagi_aptr:
                                aptr_cortical_size = check_aptr()
                            max_range = capabilities['camera']['iso_range'][1]
                            min_range = capabilities['camera']['iso_range'][0]
                            capabilities['camera']["iso_default"] = \
                                ((feagi_aptr / aptr_cortical_size) *
                                 (max_range - min_range)) + min_range
                if "o_vres" in message_from_feagi["opu_data"]:
                    if message_from_feagi["opu_data"]["o_vres"]:
                        for i in message_from_feagi["opu_data"]["o_vres"]:
                            dev_data = feagi_agent.feagi_interface.block_to_array(i)
                            if dev_data[0] == 0:
                                capabilities['camera']['current_select'] = \
                                capabilities['camera']['resolution_presets'][dev_data[2]]
                if "o_vact" in message_from_feagi["opu_data"]:
                    if message_from_feagi["opu_data"]["o_vact"]:
                        for i in message_from_feagi["opu_data"]["o_vact"]:
                            dev_data = feagi_agent.feagi_interface.block_to_array(i)
                            if dev_data[0] == 0:
                                capabilities['camera']['central_vision_allocation_percentage'][0] \
                                    = \
                                    message_from_feagi["opu_data"]["o_vact"][i]
                            if dev_data[0] == 1:
                                capabilities['camera']['central_vision_allocation_percentage'][1] \
                                    = \
                                    message_from_feagi["opu_data"]["o_vact"][i]
                # OPU section ENDS

            try:
                if "data" not in message_to_feagi:
                    message_to_feagi["data"] = dict()
                if "sensory_data" not in message_to_feagi["data"]:
                    message_to_feagi["data"]["sensory_data"] = dict()
                message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
            except Exception as e:
                pass
            # Psychopy game ends
            message_to_feagi['timestamp'] = datetime.now()
            message_to_feagi['counter'] = msg_counter
            if message_from_feagi is not None:
                feagi_settings['feagi_burst_speed'] = message_from_feagi['burst_frequency']
            sleep(feagi_settings['feagi_burst_speed'])
            if agent_settings['compression']:
                serialized_data = pickle.dumps(message_to_feagi)
                feagi_ipu_channel.send(message=lz4.frame.compress(serialized_data))
            else:
                feagi_ipu_channel.send(message_to_feagi)
            message_to_feagi.clear()
            for i in rgb['camera']:
                rgb['camera'][i].clear()
        except Exception as e:
            print("ERROR! : ", e)
            traceback.print_exc()
            break
