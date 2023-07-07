#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Demo of dot kinematogram
"""

import sys
sys.path.append('../../feagi_agent_core/')
import cv2
import requests
from time import sleep
from datetime import datetime
from feagi_agent import retina as retina
from feagi_agent import feagi_interface as feagi
import traceback


def chroma_keyer(frame, size, name_id):
    """
    This function allows you to remove the specific color. In psychopy window, it shows a gray which is 128,128,128.
    So this function will remove the 128 and focus on something else than the gray. Consider this as making the data
    into a transparent.

    Currently, this is in BETA and not used in any of this code.
    """
    vision_dict = dict()
    frame_row_count = size[0]  # width
    frame_col_count = size[1]  # height

    x_vision = 0  # row counter
    y_vision = 0  # col counter
    z_vision = 0  # RGB counter

    previous_frame = {}
    frame_len = frame_row_count * frame_col_count * 3  # hardcoded. Needs to update this section.
    try:
        if frame_len == frame_row_count * frame_col_count * 3:  # check to ensure frame length matches the
            # resolution setting
            for index in range(frame_len):
                if frame[index] != 128:
                    dict_key = str(y_vision) + '-' + str(abs((frame_row_count - 1) - x_vision)) + '-' + str(
                        0)
                    vision_dict[dict_key] = frame[index]  # save the value for the changed index to the dict
                z_vision += 1
                if z_vision == 3:
                    z_vision = 0
                    y_vision += 1
                    if y_vision == frame_col_count:
                        y_vision = 0
                        x_vision += 1
    except Exception as e:
        print("Error: Raw data frame does not match frame resolution")
        print("Error due to this: ", e)

    if len(vision_dict) > 3500:
        return {'camera': {name_id: {}}}
    else:
        return {'camera': {name_id: vision_dict}}


def main(feagi_auth_url, feagi_settings, agent_settings, capabilities, message_to_feagi):
    print("feagi setting: ", feagi_settings)
    print("agent setting: ", agent_settings)
    print("capa: ", capabilities)
    print("message: ", message_to_feagi)
    # Generate runtime dictionary
    previous_data_frame = dict()
    runtime_data = {"cortical_data": {}, "current_burst_id": None, "stimulation_period": None,
                    "feagi_state": None,
                    "feagi_network": None}

    # FEAGI section start
    print("Connecting to FEAGI resources...")
    runtime_data["feagi_state"] = feagi.feagi_registration(feagi_auth_url=feagi_auth_url, 
            feagi_settings=feagi_settings, agent_settings=agent_settings, capabilities=capabilities)
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
    # FEAGI section ends
    previous_frame_data = dict()
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    rgb = dict()
    checkpoint_total = 5
    flag_counter = 0
    rgb['camera'] = dict()

    cam = cv2.VideoCapture(capabilities['camera']['video_device_index'])

    while True:
        message_from_feagi = feagi_opu_channel.receive()
        check, pixels = cam.read()
        if bool(capabilities["camera"]["video_loop"]):
            if check:
                pass
            else:
                cam.set(cv2.CAP_PROP_POS_FRAMES, 0)
                # check, pixels = cam.read()
        retina_data = retina.frame_split(pixels, capabilities['camera']['retina_width_percent'],
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
        opu_data = feagi.opu_processor(message_from_feagi)
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
                                                                                  previous_data_frame[previous_name],
                                                                                  name,
                                                                                  capabilities[
                                                                                      'camera']['deviation_threshold'])
                else:
                    previous_name = str(i) + "_prev"
                    rgb_data, previous_data_frame[previous_name] = retina.get_rgb(data,
                                                                                  capabilities['camera'][
                                                                                      'peripheral_vision_compression'],
                                                                                  previous_data_frame[previous_name],
                                                                                  name,
                                                                                  capabilities[
                                                                                      'camera']['deviation_threshold'])
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
            print("Len --", len(message_to_feagi['data']['sensory_data']['camera']['C']))
        except:
            pass
        feagi_ipu_channel.send(message_to_feagi)
        message_to_feagi.clear()
        for i in rgb['camera']:
            rgb['camera'][i].clear()
    win.close()
    core.quit()