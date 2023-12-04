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
===============================================================================
"""

import asyncio
import threading
from time import sleep
import time
from datetime import datetime
import os
import traceback

import numpy as np
import websockets
import requests
import pickle
import lz4.frame

from configuration import *
from collections import deque
from feagi_agent import retina
from version import __version__
from feagi_agent import feagi_interface as feagi
from feagi_agent import pns_gateway as pns

rgb_array = {}
ws = deque()
ws_operation = deque()
webcam_size = {'size': []}


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
                print("error in websocket sender: ", error)
                traceback.print_exc()
                sleep(0.001)
        else:
            sleep(0.001)


def bridge_operation():
    asyncio.run(bridge_to_godot())


def rgba2rgb(rgba, background=(255, 255, 255)):
    """
    The rgba2rgb function takes an input image in the form of a numpy array with shape (row, col,
    ch), where ch is equal to 4, indicating that the input image has an RGBA color space.

    The function converts the RGBA image to an RGB image by blending the RGB channels of the
    input image with a specified background color using the alpha channel as a weighting factor.
    The resulting image is then returned as a numpy array with shape (row, col, 3), where ch is
    now equal to 3, indicating that the output image is in the RGB color space.
    """
    row, col, channels = rgba.shape

    if channels == 3:
        return rgba

    assert channels == 4, 'RGBA image has 4 channels.'

    R_CHANNEL, G_CHANNEL, B_CHANNEL = background

    alpha = rgba[:, :, 3] / 255.0

    rgb_input = np.empty((row, col, 3), dtype='uint8')
    rgb_input[:, :, 0] = (rgba[:, :, 0] * alpha + (1.0 - alpha) * R_CHANNEL).astype('uint8')
    rgb_input[:, :, 1] = (rgba[:, :, 1] * alpha + (1.0 - alpha) * G_CHANNEL).astype('uint8')
    rgb_input[:, :, 2] = (rgba[:, :, 2] * alpha + (1.0 - alpha) * B_CHANNEL).astype('uint8')

    return rgb_input


def utc_time():
    current_time = datetime.utcnow()
    return current_time


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
        test = message
        rgb_array['current'] = list(lz4.frame.decompress(test))
        webcam_size['size'] = []


async def main():
    """
    The main function handles the websocket and spins the asyncio to run the echo function
    infinitely until it exits. Once it exits, the function will resume to the next new websocket.
    """
    async with websockets.serve(echo, agent_settings["godot_websocket_ip"],
                                agent_settings['godot_websocket_port'], max_size=None,
                                max_queue=None, write_limit=None, compression=None):
        await asyncio.Future()  # run forever


def websocket_operation():
    """
    WebSocket initialized to call the echo function using asyncio.
    """
    asyncio.run(main())


if __name__ == "__main__":
    rgb = {}
    CHECKPOINT_TOTAL = 5
    rgb['camera'] = {}
    rgb_array['current'] = {}
    capabilities['camera']['current_select'] = [[], []]
    threading.Thread(target=websocket_operation, daemon=True).start()
    threading.Thread(target=bridge_operation, daemon=True).start()
    while True:
        feagi_flag = False
        print("Waiting on FEAGI...")
        while not feagi_flag:
            feagi_flag = feagi.is_FEAGI_reachable(os.environ.get('FEAGI_HOST_INTERNAL',
                                                                 "127.0.0.1"), int(os.environ.get(
                'FEAGI_OPU_PORT', "3000")))
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
        msg_counter = runtime_data["feagi_state"]['burst_counter']
        previous_frame_data = dict()
        previous_genome_timestamp = 0
        response = requests.get(api_address + '/v1/feagi/genome/cortical_area/geometry')
        capabilities['camera']['size_list'] = retina.obtain_cortical_vision_size(
            capabilities['camera']["index"], response)
        while True:
            try:
                if np.any(rgb_array['current']):
                    if not webcam_size['size']:
                        webcam_size['size'].append(rgb_array['current'].pop(0))
                        webcam_size['size'].append(rgb_array['current'].pop(0))
                    raw_frame = retina.RGB_list_to_ndarray(rgb_array['current'],
                                                           webcam_size['size'])
                    raw_frame = retina.update_astype(raw_frame)
                    if capabilities["camera"]["mirror"]:
                        raw_frame = retina.flip_video(raw_frame)
                    if capabilities['camera']['snap'] != []:
                        raw_frame = capabilities['camera']['snap']
                    previous_frame_data, rgb = retina.detect_change_edge(raw_frame, capabilities,
                                                                         capabilities['camera'][
                                                                             "index"],
                                                                         capabilities['camera'][
                                                                             'size_list'],
                                                                         previous_frame_data, rgb)
                    capabilities['camera']['snap'] = []
                    capabilities, previous_genome_timestamp, feagi_settings['feagi_burst_speed'] = \
                        retina.vision_progress(capabilities, previous_genome_timestamp,
                                               feagi_opu_channel,
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
