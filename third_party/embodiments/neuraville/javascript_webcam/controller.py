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
import time
from datetime import datetime
import os

import numpy as np
import websockets
import requests
import pickle
import lz4.frame

from configuration import *
from feagi_agent import retina
from version import __version__
from feagi_agent import feagi_interface as feagi
from feagi_agent import pns_gateway as pns

rgb_array = {}
webcam_size = {'size': []}


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
        test = message
        rgb_array['current'] = list(test)
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
    BGSK = threading.Thread(target=websocket_operation, daemon=True).start()
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
                        "stimulation_period": None, "feagi_state": None,
                        "feagi_network": None}

        # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
            feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                                   __version__)
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        genome_tracker = 0
        get_size_for_aptr_cortical = api_address + '/v1/feagi/genome/cortical_area?cortical_area' \
                                                   '=o_aptr'
        raw_aptr = requests.get(get_size_for_aptr_cortical).json()
        try:
            aptr_cortical_size = raw_aptr['cortical_dimensions'][2]
        except:
            aptr_cortical_size = None
        msg_counter = runtime_data["feagi_state"]['burst_counter']
        while True:
            try:
                start_time = 0
                message_from_feagi = pns.efferent_signaling(feagi_opu_channel)
                if message_from_feagi is not None:
                    start_time = utc_time()
                    # OPU section STARTS
                    if 'genome_num' in message_from_feagi:
                        if message_from_feagi['genome_num'] != genome_tracker:
                            genome_tracker = message_from_feagi['genome_num']
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
                    # OPU section ENDS
                if np.any(rgb_array['current']):
                    if not webcam_size['size']:
                        webcam_size['size'].append(rgb_array['current'].pop(0))
                        webcam_size['size'].append(rgb_array['current'].pop(0))
                    new_rgb = retina.RGBA_list_to_ndarray(rgb_array['current'], webcam_size['size'])
                    new_rgb = retina.update_astype(new_rgb)
                    new_rgb = rgba2rgb(new_rgb)
                    if capabilities["camera"]["mirror"]:
                        new_rgb = retina.flip_video(new_rgb)
                    previous_data_frame, rgb['camera'], capabilities['camera']['current_select'] \
                        = pns.generate_rgb(new_rgb,
                                           capabilities['camera'][
                                               'central_vision_allocation_percentage'][0],
                                           capabilities['camera'][
                                               'central_vision_allocation_percentage'][1],
                                           capabilities['camera']["central_vision_resolution"],
                                           capabilities['camera']['peripheral_vision_resolution'],
                                           previous_data_frame,
                                           capabilities['camera']['current_select'],
                                           capabilities['camera']['iso_default'],
                                           capabilities['camera']["aperture_default"])
                # Prepare thee dict to send camera data to FEAGI
                try:
                    if "data" not in message_to_feagi:
                        message_to_feagi["data"] = {}
                    if "sensory_data" not in message_to_feagi["data"]:
                        message_to_feagi["data"]["sensory_data"] = {}
                    message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
                except Exception as e:
                    pass
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
                print("ERROR: ", e)
                break
