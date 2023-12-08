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

import lz4.frame
from time import sleep
from datetime import datetime
from feagi_agent import pns_gateway as pns
from feagi_agent.version import __version__
import pickle
from feagi_agent import feagi_interface as feagi
import traceback
from configuration import *
import os


def PRINT_FROM_IPU(message_from_feagi):
    print(message_from_feagi)


def SEND_AS_IPU_DATA():
    return {"1-1-1": 10, "0-0-0": 50, "0-1-2": 30}


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
    previous_genome_timestamp = 0
    while True:
        try:
            message_from_feagi = pns.efferent_signaling(feagi_opu_channel)
            if message_from_feagi is not None:
                print("TEST: ", message_from_feagi)
                if "data" not in message_to_feagi:
                    message_to_feagi["data"] = {}
                if "sensory_data" not in message_to_feagi["data"]:
                    message_to_feagi["data"]["sensory_data"] = {}
                message_to_feagi["data"]["sensory_data"]['training'] = SEND_AS_IPU_DATA()  #
                # SENDTOIPU!!
            message_to_feagi['timestamp'] = datetime.now()
            message_to_feagi['counter'] = msg_counter
            if message_from_feagi is not None:
                feagi_settings['feagi_burst_speed'] = message_from_feagi['burst_frequency']
                runtime_data["stimulation_period"] = message_from_feagi['burst_frequency']
            sleep(feagi_settings['feagi_burst_speed'])
            if agent_settings['compression']:
                serialized_data = pickle.dumps(message_to_feagi)
                feagi_ipu_channel.send(message=lz4.frame.compress(serialized_data))
            else:
                feagi_ipu_channel.send(message_to_feagi)
            message_to_feagi.clear()
        except Exception as e:
            # pass
            print("ERROR! : ", e)
            traceback.print_exc()
            break
