#!/usr/bin/env python
"""
Copyright 2016-2023 The FEAGI Authors. All Rights Reserved.
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

import asyncio
import threading
from time import sleep
from datetime import datetime

import numpy as np
import websockets
import requests

from configuration import *
from feagi_agent import retina
from feagi_agent import feagi_interface as feagi


async def echo(websocket):
    """
    The function echoes the data it receives from other connected websockets
    and sends the data from FEAGI to the connected websockets.
    """
    async for message in websocket:
        print(message)


async def main():
    """
    The main function handles the websocket and spins the asyncio to run the echo function
    infinitely until it exits. Once it exits, the function will resume to the next new websocket.
    """
    async with websockets.serve(echo, "0.0.0.0", 9052):
        await asyncio.Future()  # run forever


def websocket_operation():
    """
    WebSocket initialized to call the echo function using asyncio.
    """
    asyncio.run(main())


if __name__ == "__main__":
    previous_data_frame = {}
    runtime_data = {"cortical_data": {}, "current_burst_id": None,
                    "stimulation_period": None, "feagi_state": None,
                    "feagi_network": None}

    # FEAGI section start
    print("Connecting to FEAGI resources...")

    feagi_host, api_port, app_data_port = \
        feagi.feagi_setting_for_registration(feagi_settings, agent_settings)

    print(feagi_host, api_port, app_data_port)

    # address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_opu_port']

    api_address = 'http://' + feagi_host + ':' + api_port

    stimulation_period_endpoint = feagi.feagi_api_burst_engine()
    burst_counter_endpoint = feagi.feagi_api_burst_counter()
    print("^ ^ ^")
    runtime_data["feagi_state"] = feagi.feagi_registration(feagi_host=feagi_host,
                                                           api_port=api_port,
                                                           agent_settings=agent_settings,
                                                           capabilities=capabilities)

    print("** **", runtime_data["feagi_state"])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

    # ipu_channel_address = feagi.feagi_inbound(agent_settings["agent_data_port"])
    ipu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                               agent_settings["agent_data_port"])
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                               runtime_data["feagi_state"]['feagi_opu_port'])
    feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=False)
    feagi_opu_channel = feagi.sub_initializer(opu_address=opu_channel_address)

    msg_counter = runtime_data["feagi_state"]['burst_counter']
    CHECKPOINT_TOTAL = 5
    FLAG_COUNTER = 0
    BGSK = threading.Thread(target=websocket_operation, daemon=True).start()
    while True:
        message_from_feagi = feagi_opu_channel.receive()  # Get data from FEAGI
        # OPU section STARTS
        # OPU section ENDS

        ir_data = "NEED UPDATED"
        if ir_data:
            formatted_ir_data = {'ir': {sensor: True for sensor in ir_data}}
        else:
            formatted_ir_data = {}

        if ir_data:
            for ir_sensor in range(int(capabilities['infrared']['count'])):
                if ir_sensor not in formatted_ir_data['ir']:
                    formatted_ir_data['ir'][ir_sensor] = False
        else:
            formatted_ir_data['ir'] = {}
            for ir_sensor in range(int(capabilities['infrared']['count'])):
                formatted_ir_data['ir'][ir_sensor] = False

        for ir_sensor in range(int(capabilities['infrared']['count'])):
            if ir_sensor not in formatted_ir_data['ir']:
                formatted_ir_data['ir'][ir_sensor] = False
        ultrasonic_data = "NEED UPDATED"
        if ultrasonic_data:
            formatted_ultrasonic_data = {
                'ultrasonic': {
                    sensor: data for sensor, data in enumerate([ultrasonic_data])
                }
            }
        else:
            formatted_ultrasonic_data = {}
        message_to_feagi, battery = feagi.compose_message_to_feagi(
            original_message={**formatted_ir_data, **formatted_ultrasonic_data})
        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        msg_counter += 1
        FLAG_COUNTER += 1
        if FLAG_COUNTER == int(CHECKPOINT_TOTAL):
            feagi_burst_speed = requests.get(api_address + stimulation_period_endpoint,
                                             timeout=5).json()
            feagi_burst_counter = requests.get(api_address + burst_counter_endpoint,
                                               timeout=5).json()
            FLAG_COUNTER = 0
            if feagi_burst_speed > 1:
                CHECKPOINT_TOTAL = 5
            if feagi_burst_speed < 1:
                CHECKPOINT_TOTAL = 5 / feagi_burst_speed
            if msg_counter < feagi_burst_counter:
                feagi_opu_channel = feagi.sub_initializer(opu_address=opu_channel_address)
                if feagi_burst_speed != feagi_settings['feagi_burst_speed']:
                    feagi_settings['feagi_burst_speed'] = feagi_burst_speed
            if feagi_burst_speed != feagi_settings['feagi_burst_speed']:
                feagi_settings['feagi_burst_speed'] = feagi_burst_speed
                msg_counter = feagi_burst_counter
        sleep(feagi_settings['feagi_burst_speed'])
        try:
            pass
            # print(len(message_to_feagi['data']['sensory_data']['camera']['C']))
        except Exception as error:
            pass
        feagi_ipu_channel.send(message_to_feagi)
        message_to_feagi.clear()

