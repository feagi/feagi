#!/usr/bin/env python

import asyncio
import websockets
import gzip
import struct

import random

# import gymnasium as gym
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, StreamingResponse
import cv2
import threading
import requests
from configuration import *
from time import sleep
from datetime import datetime
from feagi_agent import retina as retina
from feagi_agent import feagi_interface as feagi
import numpy as np

rgb_array = dict()


def rgba2rgb(rgba, background=(255, 255, 255)):
    row, col, ch = rgba.shape

    if ch == 3:
        return rgba

    assert ch == 4, 'RGBA image has 4 channels.'

    rgb_input = np.zeros((row, col, 3), dtype='float32')
    r, g, b, a = rgba[:, :, 0], rgba[:, :, 1], rgba[:, :, 2], rgba[:, :, 3]

    a = np.asarray(a, dtype='float32') / 255.0

    R, G, B = background

    rgb_input[:, :, 0] = r * a + (1.0 - a) * R
    rgb_input[:, :, 1] = g * a + (1.0 - a) * G
    rgb_input[:, :, 2] = b * a + (1.0 - a) * B

    return np.asarray(rgb_input, dtype='uint8')


async def echo(websocket):
    async for message in websocket:
        test = message
        # test = gzip.decompress(message)
        rgb_array['current'] = list(test)
        await websocket.send("thanks")


async def main():
    async with websockets.serve(echo, "0.0.0.0", 9051, max_size=None,
                                max_queue=None, write_limit=None, compression=None):
        await asyncio.Future()  # run forever


def websocket_operation():
    asyncio.run(main())


if __name__ == "__main__":
    previous_data_frame = dict()
    runtime_data = {"cortical_data": {}, "current_burst_id": None, "stimulation_period": None, "feagi_state": None,
                    "feagi_network": None}

    # FEAGI section start
    print("Connecting to FEAGI resources...")

    feagi_host, api_port, app_data_port = feagi.feagi_setting_for_registration(feagi_settings, agent_settings)

    print(feagi_host, api_port, app_data_port)

    # address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_opu_port']

    api_address = 'http://' + feagi_host + ':' + api_port

    stimulation_period_endpoint = feagi.feagi_api_burst_engine()
    burst_counter_endpoint = feagi.feagi_api_burst_counter()
    print("^ ^ ^")
    runtime_data["feagi_state"] = feagi.feagi_registration(feagi_host=feagi_host,
                                                           api_port=api_port, agent_settings=agent_settings,
                                                           capabilities=capabilities)

    print("** **", runtime_data["feagi_state"])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

    # todo: to obtain this info directly from FEAGI as part of registration
    # ipu_channel_address = feagi.feagi_inbound(agent_settings["agent_data_port"])
    ipu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                               agent_settings["agent_data_port"])
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                               runtime_data["feagi_state"]['feagi_opu_port'])
    feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=False)
    feagi_opu_channel = feagi.sub_initializer(opu_address=opu_channel_address)

    previous_frame_data = dict()
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    rgb = dict()
    checkpoint_total = 5
    flag_counter = 0
    rgb['camera'] = dict()
    rgb_array['current'] = {}
    bgsk = threading.Thread(target=websocket_operation, daemon=True).start()
    while True:
        message_from_feagi = feagi_opu_channel.receive()  # Get data from FEAGI
        # OPU section STARTS
        # OPU section ENDS
        if np.any(rgb_array['current']):
            new_rgb = np.array(rgb_array['current'])
            new_rgb = new_rgb.reshape(480, 640, 4)
            # new_rgb = new_rgb.astype(np.uint8)
            new_rgb = rgba2rgb(new_rgb)
            # cv2.imshow('Frame', new_rgb)
            # if cv2.waitKey(10) & 0xFF == ord('q'):
            #     break
            # new_rgb = cv2.rotate(new_rgb, cv2.ROTATE_90_CLOCKWISE) # rotate 90 clockwise
            # new_rgb = cv2.flip(new_rgb, 1) # flip horizontally
            retina_data = retina.frame_split(new_rgb, capabilities['camera']['retina_width_percent'],
                                             capabilities['camera']['retina_height_percent'])
            for i in retina_data:
                if 'C' in i:
                    retina_data[i] = retina.center_data_compression(retina_data[i],
                                                                    capabilities['camera']["central_vision_compression"]
                                                                    )
                else:
                    retina_data[i] = retina.center_data_compression(retina_data[i],
                                                                    capabilities['camera']
                                                                    ['peripheral_vision_compression'])
            if previous_data_frame == {}:
                for i in retina_data:
                    previous_name = str(i) + "_prev"
                    previous_data_frame[previous_name] = {}
            for i in retina_data:
                name = i
                if 'prev' not in i:
                    data = retina.ndarray_to_list(retina_data[i])
                    if 'C' in i:
                        previous_name = str(i) + "_prev"
                        rgb_data, previous_data_frame[previous_name] = retina.get_rgb(data,
                                                                                      capabilities['camera'][
                                                                                          'central_vision_compression'],
                                                                                      previous_data_frame[
                                                                                          previous_name],
                                                                                      name,
                                                                                      capabilities[
                                                                                          'camera'][
                                                                                          'deviation_threshold'])
                    else:
                        previous_name = str(i) + "_prev"
                        rgb_data, previous_data_frame[previous_name] = retina.get_rgb(data,
                                                                                      capabilities['camera'][
                                                                                          'peripheral_vision_compression'],
                                                                                      previous_data_frame[
                                                                                          previous_name],
                                                                                      name,
                                                                                      capabilities[
                                                                                          'camera'][
                                                                                          'deviation_threshold'])
                    for a in rgb_data['camera']:
                        rgb['camera'][a] = rgb_data['camera'][a]
            try:
                if "data" not in message_to_feagi:
                    message_to_feagi["data"] = dict()
                if "sensory_data" not in message_to_feagi["data"]:
                    message_to_feagi["data"]["sensory_data"] = dict()
                message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
            except Exception as e:
                pass
            # Psychopy game ends
        # message_to_feagi, battery = feagi.compose_message_to_feagi({**rgb}, battery=aliens.healthpoint*10)
        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        msg_counter += 1
        flag_counter += 1
        if flag_counter == int(checkpoint_total):
            feagi_burst_speed = requests.get(api_address + stimulation_period_endpoint).json()
            feagi_burst_counter = requests.get(api_address + burst_counter_endpoint).json()
            flag_counter = 0
            if feagi_burst_speed > 1:
                checkpoint_total = 5
            if feagi_burst_speed < 1:
                checkpoint_total = 5 / feagi_burst_speed
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
        except:
            pass
        feagi_ipu_channel.send(message_to_feagi)
        message_to_feagi.clear()
        for i in rgb['camera']:
            rgb['camera'][i].clear()
