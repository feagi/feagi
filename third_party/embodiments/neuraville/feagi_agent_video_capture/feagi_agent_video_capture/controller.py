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
from feagi_agent.version import __version__
from feagi_agent import retina as retina
from feagi_agent import pns_gateway as pns
from feagi_agent import feagi_interface as feagi
import traceback
import threading
import os
import time
import pickle
import lz4.frame
import screeninfo
import mss
import numpy
import multiprocessing
from PIL import Image

camera_data = {"vision": {}}


def process_video(video_path, capabilities):
    cam = cv2.VideoCapture(video_path)
    cam.set(3, 320)
    cam.set(4, 240)
    if capabilities['camera']['video_device_index'] == "monitor":
        all_monitors = screeninfo.get_monitors() # Needs to create an IPU for this
    pixels = []
    while True:
        if capabilities['camera']['video_device_index'] != "monitor":
            check, pixels = cam.read()
        else:
            check = True
        if capabilities['camera']['video_device_index'] != "monitor":
            if bool(capabilities["camera"]["video_loop"]):
                if check:
                    sleep(0.01)
                    cv2.imshow("OpenCV/Numpy normal", pixels)
                else:
                    cam.set(cv2.CAP_PROP_POS_FRAMES, 0)
        if capabilities['camera']['video_device_index'] == "monitor":
            with mss.mss() as sct:
              monitors = all_monitors[capabilities['camera']['monitor']]
              monitor = {
                "top": monitors.y,
                "left": monitors.x,
                "width": monitors.width,
                "height": monitors.height
              }

              img = numpy.array(sct.grab(monitor))
              pixels = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
            if capabilities["camera"]["mirror"]:
                pixels = cv2.flip(pixels, 1)
            camera_data["vision"] = pixels
        else:
            if capabilities["camera"]["mirror"]:
                pixels = cv2.flip(pixels, 1)
            camera_data["vision"] = pixels
        cv2.waitKey(30)
        # print("len: ", len(pixels), " and shape: ", pixels.shape)
        # try:
        #   if len(pixels) == 600:
        #     cv2.imshow("OpenCV/Numpy normal", pixels)
        #     if cv2.waitKey(25) & 0xFF == ord("q"):
        #       cv2.destroyAllWindows()
        #       break
        # except:
        #   traceback.print_exc()

    cam.release()
    cv2.destroyAllWindows()


def main(feagi_auth_url, feagi_settings, agent_settings, capabilities, message_to_feagi):
    # Generate runtime dictionary
    previous_data_frame = dict()
    runtime_data = {"vision": {}, "current_burst_id": None, "stimulation_period": None,
                    "feagi_state": None,
                    "feagi_network": None}
    print("retrying...")
    FEAGI_FLAG = False
    print("Waiting on FEAGI...")
    while not FEAGI_FLAG:
        FEAGI_FLAG = feagi.is_FEAGI_reachable(
            os.environ.get('FEAGI_HOST_INTERNAL', feagi_settings["feagi_host"]),
            int(os.environ.get('FEAGI_OPU_PORT', "3000")))
        sleep(2)
    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    rgb = dict()
    capabilities['camera']['current_select'] = [[], []]
    rgb['camera'] = dict()
    genome_tracker = 0
    get_size_for_aptr_cortical = api_address + '/v1/feagi/genome/cortical_area?cortical_area=o_aptr'
    raw_aptr = requests.get(get_size_for_aptr_cortical).json()
    aptr_cortical_size = pns.fetch_aptr_size(10, raw_aptr, None)
    threading.Thread(target=process_video, args=(capabilities['camera']['video_device_index'],
                                                 capabilities), daemon=True).start()

    while True:
        try:
            message_from_feagi = pns.efferent_signaling(feagi_opu_channel)
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
                                 capabilities['camera']["aperture_default"],
                                 camera_index=capabilities['camera']["index"])
            if message_from_feagi is not None:
                # Obtain the size of aptr
                if aptr_cortical_size is None:
                    aptr_cortical_size = pns.check_aptr(raw_aptr)
                # Update the aptr
                capabilities = pns.fetch_aperture_data(message_from_feagi, capabilities,
                                                       aptr_cortical_size)
                # Update the ISO
                capabilities = pns.fetch_iso_data(message_from_feagi, capabilities,
                                                  aptr_cortical_size)
                # Update the vres
                capabilities = pns.fetch_resolution_selected(message_from_feagi, capabilities)
                # Update the aceture
                capabilities = pns.fetch_vision_acuity(message_from_feagi, capabilities)
                # OPU section STARTS
                if 'genome_num' in message_from_feagi:
                    if message_from_feagi['genome_num'] != genome_tracker:
                        genome_tracker = message_from_feagi['genome_num']
                if "o__mon" in message_from_feagi["opu_data"]:
                  if message_from_feagi["opu_data"]["o__mon"]:
                    for i in message_from_feagi["opu_data"]["o__mon"]:
                      monitor_update = feagi.block_to_array(i)
                      capabilities['camera']['monitor'] = monitor_update[0]
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
