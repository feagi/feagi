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
import requests
from time import sleep
from datetime import datetime
from feagi_agent import pns_gateway as pns
from feagi_agent.version import __version__
from feagi_agent import retina as retina
from feagi_agent import feagi_interface as feagi
import traceback
import threading
import os
import screeninfo
import mss
import numpy

camera_data = {"vision": {}}


def process_video(video_path, capabilities):
    if capabilities["camera"]["image"] == "":
      cam = cv2.VideoCapture(video_path)
    # cam.set(3, 320)
    # cam.set(4, 240)
    if capabilities['camera']['video_device_index'] == "monitor":
        all_monitors = screeninfo.get_monitors()  # Needs to create an IPU for this
    pixels = []
    static_image = []
    while True:
        if capabilities['camera']['video_device_index'] != "monitor":
          if capabilities["camera"]["image"] != "":
            if static_image == []:
              pixels = cv2.imread(capabilities["camera"]["image"], -1)
              static_image = pixels
            else:
              pixels = static_image
              pixels = adjust_gamma(pixels)
          else:
            check, pixels = cam.read()
        else:
            check = True
        if capabilities['camera']['video_device_index'] != "monitor":
            if bool(capabilities["camera"]["video_loop"]):
                if check:
                    sleep(0.05)
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

    cam.release()
    cv2.destroyAllWindows()


def adjust_gamma(image, gamma=5.0):
  invGamma = 1.0 / gamma
  table = numpy.array([((i / 255.0) ** invGamma) * 255
		for i in numpy.arange(0, 256)]).astype("uint8")
  # apply gamma correction using the lookup table
  return cv2.LUT(image, table)

def main(feagi_auth_url, feagi_settings, agent_settings, capabilities, message_to_feagi):
    threading.Thread(target=process_video, args=(capabilities['camera']['video_device_index'],
                                                 capabilities), daemon=True).start()
    # Generate runtime dictionary
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
    rgb['camera'] = dict()
    response = requests.get(api_address + '/v1/feagi/genome/cortical_area/geometry')
    capabilities['camera']['size_list'] = retina.obtain_cortical_vision_size(capabilities['camera']["index"], response)
    previous_genome_timestamp = 0
    previous_frame_data = {}
    raw_frame = []
    while True:
        try:
            if camera_data['vision'] is not None:
                raw_frame = camera_data['vision']
                cv2.imshow("OpenCV/Numpy normal", raw_frame) # Move to main due to Mac's restriction
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    pass
            if capabilities['camera']['blink'] != []:
                raw_frame = capabilities['camera']['blink']
            previous_frame_data, rgb = retina.detect_change_edge(raw_frame, capabilities,
                                                                 capabilities['camera']["index"],
                                                                 capabilities['camera']['size_list'],
                                                                 previous_frame_data, rgb)
            capabilities['camera']['blink'] = []
            capabilities, previous_genome_timestamp, feagi_settings['feagi_burst_speed'] = \
                retina.vision_progress(capabilities, previous_genome_timestamp, feagi_opu_channel,
                                       api_address, feagi_settings, raw_frame)

            message_to_feagi = pns.generate_feagi_data(rgb, msg_counter, datetime.now(),
                                                       message_to_feagi)
            sleep(feagi_settings['feagi_burst_speed'])
            pns.afferent_signaling(message_to_feagi, feagi_ipu_channel, agent_settings)

            message_to_feagi.clear()
            for i in rgb['camera']:
                rgb['camera'][i].clear()
        except Exception as e:
            # pass
            print("ERROR! : ", e)
            traceback.print_exc()
            break
