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
import websockets
import requests
from time import sleep
from configuration import *
from datetime import datetime
from collections import deque
from feagi_agent import feagi_interface as feagi

ws = deque()


async def echo(websocket):
    """
    The function echoes the data it receives from other connected websockets
    and sends the data from FEAGI to the connected websockets.
    """
    async for message in websocket:
        ir0, ir1 = False, False
        # TODO: Fix this issue
        # if '-' in message:
        #     message = message.replace('-', '')
        #     print("message: ", message)
        if message[0] == 'f':
            ir0 = 0
        else:
            ir0 = 1
        if message[1] == 'f':
            ir1 = 0
        else:
            ir1 = 1
        try:
            x = int(message[2:6]) - 1000
            y = int(message[6:10]) - 1000
            z = int(message[10:14]) - 1000
            ultrasonic = float(message[14:16])
            sound_level = int(message[16:18])
        except Exception as e:
            print("error: ", e)
            print("raw: ", message)

        # Store values in dictionary
        microbit_data['ir'] = [ir0, ir1]
        microbit_data['ultrasonic'] = ultrasonic / 25
        microbit_data['accelerator'] = [x, y, z]
        microbit_data['sound_level'] = {sound_level}
        try:
            if len(ws) > 2:  # This will eliminate any stack up queue
                stored_value = ws[len(ws) - 1]
                ws.clear()
                ws[0] = stored_value
            await websocket.send(str(ws[0]))
            ws.pop()
        except Exception as e:
            pass
            # print("error: ", e)


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
    microbit_data = {'ir': [], 'ultrasonic': {}, 'acc': {}, 'sound_level': {}}
    runtime_data['accelerator'] = dict()
    BGSK = threading.Thread(target=websocket_operation, daemon=True).start()
    flag = True
    while True:
        ws_string = ""
        message_from_feagi = feagi_opu_channel.receive()  # Get data from FEAGI
        # OPU section STARTS
        if message_from_feagi is not None:
            opu_data = feagi.opu_processor(message_from_feagi)
            if 'motor' in opu_data:
                for data_point in opu_data['motor']:
                    if data_point == 0:
                        ws_string = "f"
                    elif data_point == 1:
                        ws_string = "b"
                    elif data_point == 2:
                        ws_string = "r"
                    elif data_point == 3:
                        ws_string = "l"
                    else:
                        ws_string = "s"  # Skip
                    ws_string = ws_string + str(opu_data['motor'][data_point] * 10)
            if ws_string != "":
                ws.append(ws_string + '#')
            if flag:
                flag = False
                ws.append("f#")
        # OPU section ENDS

        if microbit_data['ir']:
            ir_data = {0: bool(microbit_data['ir'][0]), 1: bool(microbit_data['ir'][1])}
            formatted_ir_data = {'ir': dict.fromkeys(ir_data.keys(), 1)}
            formatted_ir_data['ir'].update(ir_data)  # Should work
        else:
            formatted_ir_data = {}
        ultrasonic_data = microbit_data['ultrasonic']
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

        # Add accelerator section
        try:
            runtime_data['accelerator']['0'] = microbit_data['accelerator'][0]
            runtime_data['accelerator']['1'] = microbit_data['accelerator'][1]
            runtime_data['accelerator']['2'] = microbit_data['accelerator'][2]
            if "data" not in message_to_feagi:
                message_to_feagi["data"] = dict()
            if "sensory_data" not in message_to_feagi["data"]:
                message_to_feagi["data"]["sensory_data"] = dict()
            message_to_feagi["data"]["sensory_data"]['accelerator'] = runtime_data['accelerator']
        except Exception as e:
            message_to_feagi["data"]["sensory_data"]['accelerator'] = {}
        # End accelerator section

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
