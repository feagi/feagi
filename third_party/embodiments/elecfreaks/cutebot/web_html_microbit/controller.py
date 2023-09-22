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
from collections import deque
from datetime import datetime
from time import sleep
import requests
import lz4.frame
import pickle
import websockets
from configuration import *
from feagi_agent import feagi_interface as feagi

ws = deque()
ws_operation = deque()
previous_data = ""


async def bridge_to_godot():
    while True:
        if ws:
            try:
                if ws_operation:
                    if len(ws) > 0:
                        if len(ws) > 2:
                            stored_value = ws.pop()
                            ws.clear()
                            ws.append(stored_value)
                    await ws_operation[0].send(str(ws[0]))
                    ws.pop()
                if "stimulation_period" in runtime_data:
                    sleep(runtime_data["stimulation_period"])
            except Exception as error:
                print("error: ", error)
                sleep(0.001)
        else:
            sleep(0.001)


def bridge_operation():
    asyncio.run(bridge_to_godot())


async def echo(websocket):
    """
    The function echoes the data it receives from other connected websockets
    and sends the data from FEAGI to the connected websockets.
    """
    async for message in websocket:
        if not ws_operation:
            ws_operation.append(websocket)
        else:
            ws_operation[0] = websocket
        ir0, ir1 = False, False
        if message[0] == 'f':
            ir0 = 0
        else:
            ir0 = 1
        if message[1] == 'f':
            ir1 = 0
        else:
            ir1 = 1
        try:
            x_acc = int(message[2:6]) - 1000
            y_acc = int(message[6:10]) - 1000
            z_acc = int(message[10:14]) - 1000
            ultrasonic = float(message[14:16])
            sound_level = int(message[16:18])
            # Store values in dictionary
            microbit_data['ir'] = [ir0, ir1]
            microbit_data['ultrasonic'] = ultrasonic / 25
            microbit_data['accelerator'] = [x_acc, y_acc, z_acc]
            microbit_data['sound_level'] = {sound_level}
        except Exception as Error_case:
            print("error: ", Error_case)
            print("raw: ", message)


async def main():
    """
    The main function handles the websocket and spins the asyncio to run the echo function
    infinitely until it exits. Once it exits, the function will resume to the next new websocket.
    """
    async with websockets.serve(echo, agent_settings["godot_websocket_ip"],
                                agent_settings['godot_websocket_port']):
        await asyncio.Future()  # run forever


def websocket_operation():
    """
    WebSocket initialized to call the echo function using asyncio.
    """
    asyncio.run(main())


if __name__ == "__main__":
    CHECKPOINT_TOTAL = 5
    FLAG_COUNTER = 0
    microbit_data = {'ir': [], 'ultrasonic': {}, 'acc': {}, 'sound_level': {}}
    threading.Thread(target=websocket_operation, daemon=True).start()
    # threading.Thread(target=bridge_to_godot, daemon=True).start()
    threading.Thread(target=bridge_operation, daemon=True).start()
    FLAG = True
    while True:
        feagi_flag = False
        print("Waiting on FEAGI...")
        while not feagi_flag:
            feagi_flag = feagi.is_FEAGI_reachable(
                os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
                int(os.environ.get('FEAGI_OPU_PORT', "3000"))
            )
            sleep(2)
        previous_data_frame = {}
        runtime_data = {"cortical_data": {}, "current_burst_id": None,
                        "stimulation_period": 0.01, "feagi_state": None,
                        "feagi_network": None}

        feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
        print("FEAGI AUTH URL ------- ", feagi_auth_url)

        # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # - - - - - - - - - - - - - - - - - - #
        print("Connecting to FEAGI resources...")
        runtime_data["feagi_state"] = feagi.feagi_registration(feagi_auth_url=feagi_auth_url,
                                                               feagi_settings=feagi_settings,
                                                               agent_settings=agent_settings,
                                                               capabilities=capabilities)
        api_address = runtime_data['feagi_state']["feagi_url"]
        stimulation_period_endpoint = feagi.feagi_api_burst_engine()
        burst_counter_endpoint = feagi.feagi_api_burst_counter()

        agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
        print("** **", runtime_data["feagi_state"])
        feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

        # todo: to obtain this info directly from FEAGI as part of registration
        ipu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'], agent_data_port)
        print("IPU_channel_address=", ipu_channel_address)
        opu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                                   runtime_data["feagi_state"]['feagi_opu_port'])

        feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=False)
        feagi_opu_channel = feagi.sub_initializer(opu_address=opu_channel_address)
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # - - - - - #
        msg_counter = runtime_data["feagi_state"]['burst_counter']
        runtime_data['accelerator'] = {}
        while True:
            try:
                WS_STRING = ""

                # Decompression section starts
                received_data = feagi_opu_channel.receive() # Obtain data from FEAGI
                if received_data is not None:
                    if isinstance(received_data, bytes):
                        decompressed_data = lz4.frame.decompress(received_data)
                        message_from_feagi = pickle.loads(decompressed_data)
                    else:
                        message_from_feagi = received_data
                else:
                    message_from_feagi = None
                # Decompression section ends

                # OPU section STARTS
                if message_from_feagi is not None:
                    opu_data = feagi.opu_processor(message_from_feagi)
                    WS_STRING = ""
                    if 'motor' in opu_data:
                        if opu_data['motor']:
                            # Ensure that it's in order for microbit
                            for i in sorted(opu_data['motor']):
                                if i in [0, 1]:
                                    # Append the motor data as a two-digit
                                    WS_STRING += str(i) + str(opu_data['motor'][i]-10).zfill(2)
                                    # string
                                elif i in [2, 3]:
                                    # Append the motor data as a two-digit
                                    WS_STRING += str(i) + str(opu_data['motor'][i]-10).zfill(2)
                                    # string
                                else:
                                    # If the motor value is not present, append "00"
                                    WS_STRING += str(i) + "00"
                            if len(WS_STRING) != 6:
                                if int(WS_STRING[0]) < 2:
                                    WS_STRING = WS_STRING + "500"
                                else:
                                    WS_STRING = "500" + WS_STRING
                            WS_STRING = WS_STRING + "#"
                            ws.append(WS_STRING)

                    if FLAG:
                        FLAG = False
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
                        message_to_feagi["data"] = {}
                    if "sensory_data" not in message_to_feagi["data"]:
                        message_to_feagi["data"]["sensory_data"] = {}
                    message_to_feagi["data"]["sensory_data"]['accelerator'] = runtime_data[
                        'accelerator']
                except Exception as ERROR:
                    message_to_feagi["data"]["sensory_data"]['accelerator'] = {}
                # End accelerator section

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
                print("ERROR: ", e)
                break
