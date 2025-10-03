#!/usr/bin/env python
"""
Copyright 2016-2024 The FEAGI Authors. All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
===============================================================================
"""
import os
import json
import time
import pickle
import asyncio
import requests
import traceback
import lz4.frame
import threading
import websockets
import numpy as np
from time import sleep
from collections import deque
from datetime import datetime
from version import __version__
from feagi_connector import retina
from feagi_connector import sensors
from feagi_connector import pns_gateway as pns
from feagi_connector import feagi_interface as feagi

ws = deque()
ws_operation = deque()


async def echo(websocket):
    """
    The function echoes the data it receives from other connected websockets
    and sends the data from FEAGI to the connected websockets.
    """
    global message_to_feagi, connected_agents
    try:
        async for message in websocket:
            connected_agents['0'] = True # Since this section gets data from client, its marked as true
            if not ws_operation:
                ws_operation.append(websocket)
            else:
                ws_operation[0] = websocket
            message_to_feagi = pickle.loads(message)
    except Exception:
        pass
    connected_agents['0'] = False  # Once client disconnects, mark it as false
    message_to_feagi.clear()


if __name__ == "__main__":
    connected_agents = dict()  # Initalize
    connected_agents['0'] = False  # By default, it is not connected by client's websocket
    message_to_feagi = dict()

    # NEW JSON UPDATE
    fnet = open('networking.json')
    fcap = open('capabilities.json')
    configuration = json.load(fnet)
    skills = json.load(fcap)
    feagi_settings =  configuration["feagi_settings"]
    agent_settings = configuration['agent_settings']
    capabilities = skills['capabilities']
    feagi_settings['feagi_host'] = os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1")
    feagi_settings['feagi_api_port'] = os.environ.get('FEAGI_API_PORT', "8000")
    agent_settings['godot_websocket_port'] = os.environ.get('WS_CONTROLLER_PORT', "9053")
    fnet.close()
    fcap.close()
    # END JSON UPDATE

    while True:
        feagi_flag = False
        print("Waiting on FEAGI...")
        while not feagi_flag:
            feagi_flag = feagi.is_FEAGI_reachable(os.environ.get('FEAGI_HOST_INTERNAL',
                                                                 "127.0.0.1"),
                                                  int(os.environ.get('FEAGI_OPU_PORT', "3000")))
            sleep(2)
        print("DONE")
        previous_data_frame = {}
        runtime_data = {"cortical_data": {}, "current_burst_id": None,
                        "stimulation_period": 0.01, "feagi_state": None,
                        "feagi_network": None}

        # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
            feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                                   __version__)
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        pns.start_websocket_in_threads(echo,
                                       agent_settings["godot_websocket_ip"],
                                       agent_settings['godot_websocket_port'],
                                       ws_operation, ws,
                                       feagi_settings)
        threading.Thread(target=pns.feagi_listener, args=(feagi_opu_channel,), daemon=True).start()
        while True:
            try:
                message_from_feagi = pns.message_from_feagi
                if message_from_feagi:
                    ws.append(pickle.dumps(message_from_feagi))
                    feagi_settings['feagi_burst_speed'] = message_from_feagi['burst_frequency']
                message_to_feagi = sensors.add_agent_status(connected_agents['0'], message_to_feagi,
                                                            agent_settings)
                sleep(feagi_settings['feagi_burst_speed'])  # bottleneck
                pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings, feagi_settings)
                # message_to_feagi.clear()
            except Exception as e:
                # pass
                print("ERROR! : ", e)
                traceback.print_exc()
                break
