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

from time import sleep
from datetime import datetime
from feagi_agent import pns_gateway as pns
from feagi_agent import retina as retina
from feagi_agent.version import __version__
from feagi_agent import feagi_interface as feagi
import trainer as feagi_trainer
import traceback
from configuration import *
import requests
import os

if __name__ == "__main__":
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
    if not pns.full_list_dimension:
        pns.full_list_dimension = pns.fetch_full_dimensions()
    rgb = dict()
    rgb['camera'] = dict()
    previous_frame_data = {}
    response = requests.get(api_address + '/v1/feagi/genome/cortical_area/geometry')
    size_list = retina.obtain_cortical_vision_size("00", response)  # Temporarily
    start_timer = 0
    raw_frame = []
    continue_loop = True
    while continue_loop:
        list_images = feagi_trainer.scan_the_folder(capabilities['image_reader']['path'])
        for image in list_images:
            message_to_feagi, image, raw_frame = \
                feagi_trainer.id_training_with_image(message_to_feagi, image,capabilities,raw_frame)
            # Post image into vision
            previous_frame_data, rgb = retina.detect_change_edge(raw_frame, capabilities, "00",
                                                                 size_list, previous_frame_data,
                                                                 rgb)
            capabilities, feagi_settings['feagi_burst_speed'] = \
                retina.vision_progress(capabilities,feagi_opu_channel, api_address,
                                       feagi_settings, raw_frame)

            message_to_feagi = pns.generate_feagi_data(rgb, msg_counter, datetime.now(),
                                                       message_to_feagi)
            # Vision process ends
            if start_timer == 0:
                start_timer = datetime.now()
            while capabilities['image_reader']['pause'] >= int((datetime.now()-start_timer).total_seconds()):
                pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings)
            start_timer = 0
            message_to_feagi.clear()
            continue_loop = capabilities['image_reader']['loop']
