#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Demo of dot kinematogram
"""

import cv2
import requests
from time import sleep
from datetime import datetime
from feagi_agent import retina as retina
from feagi_agent import feagi_interface as feagi
import traceback
import time
import mss
import screeninfo
import numpy
from PIL import Image


def chroma_keyer(frame, size, name_id):
    """
    This function allows you to remove the specific color. In psychopy window, it shows a gray
    which is 128,128,128. So this function will remove the 128 and focus on something else than
    the gray. Consider this as making the data into a transparent.

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
        if frame_len == frame_row_count * frame_col_count * 3:  # check to ensure frame length
            # matches the resolution setting
            for index in range(frame_len):
                if frame[index] != 128:
                    dict_key = str(y_vision) + '-' + str(
                        abs((frame_row_count - 1) - x_vision)) + '-' + str(
                        0)
                    vision_dict[dict_key] = frame[
                        index]  # save the value for the changed index to the dict
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


def pil_frombytes(im):
    """ Efficient Pillow version. """
    return Image.frombytes('RGB', im.size, im.bgra, 'raw', 'BGRX').tobytes()


def check_aptr(size):
    try:
        raw_aptr = requests.get(get_size_for_aptr_cortical).json()
        return raw_aptr['cortical_dimensions'][2]
    except Exception as error:
        print("error: ", error)
        return 10


def main(feagi_auth_url, feagi_settings, agent_settings, capabilities, message_to_feagi):
    # Generate runtime dictionary
    previous_data_frame = dict()
    runtime_data = {"cortical_data": {}, "current_burst_id": None, "stimulation_period": None,
                    "feagi_state": None,
                    "feagi_network": None}
    feagi_flag = False
    print("retrying...")
    print("Waiting on FEAGI...")
    while not feagi_flag:
        feagi_flag = feagi.is_FEAGI_reachable(feagi_settings["feagi_host"], 3000)
        sleep(2)
    burst_counter_endpoint = feagi.feagi_api_burst_counter()
    # FEAGI section start
    print("Connecting to FEAGI resources...")
    runtime_data["feagi_state"] = feagi.feagi_registration(feagi_auth_url=feagi_auth_url,
                                                           feagi_settings=feagi_settings,
                                                           agent_settings=agent_settings,
                                                           capabilities=capabilities)
    print("FEAGI REGISTERED!")
    api_address = runtime_data['feagi_state']["feagi_url"]

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
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    rgb = dict()
    checkpoint_total = 5
    flag_counter = 0
    rgb['camera'] = dict()
    genome_tracker = 0
    get_size_for_aptr_cortical = api_address + '/v1/feagi/genome/cortical_area?cortical_area=o_aptr'
    raw_aptr = requests.get(get_size_for_aptr_cortical).json()
    try:
        aptr_cortical_size = raw_aptr['cortical_dimensions'][2]
    except:
        aptr_cortical_size = None
    screen_info = screeninfo.get_monitors()[0]  # Assuming you want the primary monitor
    screen_width = 600
    screen_height = 600
    monitor = {"top": 40, "left": 0, "width": screen_width, "height": screen_height}
    if capabilities['camera']['video_device_index'] != "monitor":
        cam = cv2.VideoCapture(capabilities['camera']['video_device_index'])

    while True:
        try:
            message_from_feagi = feagi_opu_channel.receive()
            if capabilities['camera']['video_device_index'] != "monitor":
                check, pixels = cam.read()
            else:
                check = True
            if capabilities['camera']['video_device_index'] != "monitor":
                if bool(capabilities["camera"]["video_loop"]):
                    if check:
                        pass
                    else:
                        cam.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        # check, pixels = cam.read()
            if capabilities['camera']['video_device_index'] == "monitor":
                with mss.mss() as sct:
                    img = numpy.array(sct.grab(monitor))
                    pixels = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
                    cv2.imshow("OpenCV/Numpy normal", pixels)
                    # print(rgb)
                cv2.waitKey(25)
            retina_data = retina.frame_split(pixels, capabilities['camera']['retina_width_percent'],
                                             capabilities['camera']['retina_height_percent'])
            for i in retina_data:
                if 'C' in i:
                    retina_data[i] = retina.center_data_compression(
                        retina_data[i],
                        capabilities['camera'][
                            "central_vision_compression"]
                    )
                else:
                    retina_data[i] = retina.center_data_compression(
                        retina_data[i],
                        capabilities['camera']
                        ['peripheral_vision_compression'])
            if message_from_feagi is not None:
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
                        rgb_data, previous_data_frame[previous_name] = \
                            retina.get_rgb(data,
                                           capabilities[
                                               'camera'][
                                               'central_vision_compression'],
                                           previous_data_frame[
                                               previous_name],
                                           name,
                                           capabilities[
                                               'camera'][
                                               'deviation_threshold'],
                                           capabilities['camera']["aperture_default"])
                    else:
                        previous_name = str(i) + "_prev"
                        rgb_data, previous_data_frame[previous_name] = \
                            retina.get_rgb(data,
                                           capabilities[
                                               'camera'][
                                               'peripheral_vision_compression'],
                                           previous_data_frame[
                                               previous_name],
                                           name,
                                           capabilities[
                                               'camera'][
                                               'deviation_threshold'],
                                           capabilities['camera']["aperture_default"])
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
            if message_from_feagi is not None:
                feagi_settings['feagi_burst_speed'] = message_from_feagi['burst_frequency']
            sleep(feagi_settings['feagi_burst_speed'])
            try:
                print("Len --", len(message_to_feagi['data']['sensory_data']['camera']['C']))
            except:
                pass
            feagi_ipu_channel.send(message_to_feagi)
            message_to_feagi.clear()
            for i in rgb['camera']:
                rgb['camera'][i].clear()
        except Exception as e:
            print("ERROR! : ", e)
            cam.release()
            break
