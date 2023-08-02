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

from configuration import *
from feagi_agent import retina
from feagi_agent import feagi_interface as feagi

rgb_array = {}


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


def check_aptr():
    try:
        raw_aptr = requests.get(get_size_for_aptr_cortical).json()
        return raw_aptr['cortical_dimensions'][2]
    except Exception as error:
        print("error: ", error)
        return 10


async def echo(websocket):
    """
    The function echoes the data it receives from other connected websockets
    and sends the data from FEAGI to the connected websockets.
    """
    async for message in websocket:
        test = message
        rgb_array['current'] = list(test)


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

        # FEAGI section start
        print("Connecting to FEAGI resources...")
        feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
        print("FEAGI AUTH URL ------- ", feagi_auth_url)
        runtime_data["feagi_state"] = feagi.feagi_registration(feagi_auth_url=feagi_auth_url,
                                                               feagi_settings=feagi_settings,
                                                               agent_settings=agent_settings,
                                                               capabilities=capabilities)
        api_address = runtime_data['feagi_state']["feagi_url"]

        stimulation_period_endpoint = feagi.feagi_api_burst_engine()
        burst_counter_endpoint = feagi.feagi_api_burst_counter()

        # agent_data_port = agent_settings["agent_data_port"]
        agent_data_port = str(runtime_data["feagi_state"]['agent_state']['agent_data_port'])
        print("** **", runtime_data["feagi_state"])
        feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

        # todo: to obtain this info directly from FEAGI as part of registration
        # ipu_channel_address = feagi.feagi_inbound(agent_settings["agent_data_port"])
        ipu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'], agent_data_port)
        print("IPU_channel_address=", ipu_channel_address)
        opu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                                   runtime_data["feagi_state"]['feagi_opu_port'])

        feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=False)
        feagi_opu_channel = feagi.sub_initializer(opu_address=opu_channel_address)
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
                message_from_feagi = feagi_opu_channel.receive()  # Get data from FEAGI
                if message_from_feagi is not None:
                    start_time = utc_time()
                    # OPU section STARTS
                    if 'genome_num' in message_from_feagi:
                        if message_from_feagi['genome_num'] != genome_tracker:
                            genome_tracker = message_from_feagi['genome_num']

                    if "o_aptr" in message_from_feagi["opu_data"]:
                        if message_from_feagi["opu_data"]["o_aptr"]:
                            for i in message_from_feagi["opu_data"]["o_aptr"]:
                                feagi_aptr = (int(i.split('-')[-1]))
                                if aptr_cortical_size is None:
                                    aptr_cortical_size = check_aptr(aptr_cortical_size)
                                elif aptr_cortical_size <= feagi_aptr:
                                    aptr_cortical_size = check_aptr(aptr_cortical_size)
                                max_range = capabilities['camera']['aperture_range'][1]
                                min_range = capabilities['camera']['aperture_range'][0]
                                capabilities['camera']["aperture_default"] = \
                                    ((feagi_aptr / aptr_cortical_size) *
                                     (max_range - min_range)) + min_range
                    # OPU section ENDS
                if np.any(rgb_array['current']):
                    if len(rgb_array['current']) == 1228800:
                        new_rgb = np.array(rgb_array['current'])
                        new_rgb = new_rgb.reshape(480, 640, 4)
                    elif len(rgb_array['current']) == 266256:
                        new_rgb = np.array(rgb_array['current'])
                        new_rgb = new_rgb.reshape(258, 258, 4)
                    elif len(rgb_array['current']) == 360000:
                        new_rgb = np.array(rgb_array['current'])
                        new_rgb = new_rgb.reshape(300, 300, 4)
                    elif len(rgb_array['current']) == 65536:
                        new_rgb = np.array(rgb_array['current'])
                        new_rgb = new_rgb.reshape(128, 128, 4)
                    elif len(rgb_array['current']) == 16384:
                        new_rgb = np.array(rgb_array['current'])
                        new_rgb = new_rgb.reshape(64, 64, 4)
                    elif len(rgb_array['current']) == 120000:
                        new_rgb = np.array(rgb_array['current'])
                        new_rgb = new_rgb.reshape(150, 200, 4)
                    # new_rgb = new_rgb.astype(np.uint8)
                    new_rgb = rgba2rgb(new_rgb)
                    retina_data = retina.frame_split(new_rgb,
                                                     capabilities['camera']['retina_width_percent'],
                                                     capabilities['camera'][
                                                         'retina_height_percent'])
                    for i in retina_data:
                        if 'C' in i:
                            retina_data[i] = \
                                retina.center_data_compression(retina_data[i],
                                                               capabilities['camera']
                                                               ["central_vision_compression"])
                        else:
                            retina_data[i] = \
                                retina.center_data_compression(retina_data[i],
                                                               capabilities['camera']
                                                               ['peripheral_vision_compression'])
                    if not previous_data_frame:
                        for i in retina_data:
                            PREVIOUS_NAME = str(i) + "_prev"
                            previous_data_frame[PREVIOUS_NAME] = {}
                    for i in retina_data:
                        name = i
                        if 'prev' not in i:
                            data = retina.ndarray_to_list(retina_data[i])
                            if 'C' in i:
                                PREVIOUS_NAME = str(i) + "_prev"
                                rgb_data, previous_data_frame[PREVIOUS_NAME] = \
                                    retina.get_rgb(data,
                                                   capabilities['camera'][
                                                       'central_vision_compression'],
                                                   previous_data_frame[
                                                       PREVIOUS_NAME],
                                                   name,
                                                   capabilities[
                                                       'camera'][
                                                       'deviation_threshold'],
                                                   capabilities['camera']["aperture_default"])
                            else:
                                PREVIOUS_NAME = str(i) + "_prev"
                                rgb_data, previous_data_frame[PREVIOUS_NAME] = \
                                    retina.get_rgb(data,
                                                   capabilities[
                                                       'camera'][
                                                       'peripheral_vision_compression'],
                                                   previous_data_frame[
                                                       PREVIOUS_NAME],
                                                   name,
                                                   capabilities[
                                                       'camera'][
                                                       'deviation_threshold'],
                                                   capabilities['camera']["aperture_default"])
                            for a in rgb_data['camera']:
                                rgb['camera'][a] = rgb_data['camera'][a]
                    try:
                        if "data" not in message_to_feagi:
                            message_to_feagi["data"] = {}
                        if "sensory_data" not in message_to_feagi["data"]:
                            message_to_feagi["data"]["sensory_data"] = {}
                        message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
                    except Exception as e:
                        pass
                    # Psychopy game ends
                # message_to_feagi, battery = feagi.compose_message_to_feagi({**rgb},
                # battery=aliens.healthpoint*10)
                message_to_feagi['timestamp'] = datetime.now()
                message_to_feagi['counter'] = msg_counter
                if message_from_feagi is not None:
                        feagi_settings['feagi_burst_speed'] = message_from_feagi['burst_frequency']
                sleep(feagi_settings['feagi_burst_speed'])
                try:
                    pass
                    # print(len(message_to_feagi['data']['sensory_data']['camera']['C']))
                except Exception as error:
                    pass

                feagi_ipu_channel.send(message_to_feagi)
                message_to_feagi.clear()
                for i in rgb['camera']:
                    rgb['camera'][i].clear()
            except Exception as e:
                print("ERROR: ", e)
                break
