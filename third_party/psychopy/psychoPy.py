#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Demo of dot kinematogram
"""

import cv2
import numpy as np
import retina as retina
import feagi_interface as FEAGI
from time import sleep
from configuration import *
from datetime import datetime
from psychopy import visual, event, core


def ndarray_to_list(array):
    array = array.flatten()
    new_list = (array.tolist())
    return new_list


def get_rgb(frame, size, previous_frame_data, name_id):
    vision_dict = dict()
    frame_row_count = size[0]  # width
    frame_col_count = size[1]  # height

    x_vision = 0  # row counter
    y_vision = 0  # col counter
    z_vision = 0  # RGB counter

    try:
        previous_frame = previous_frame_data
    except Exception:
        previous_frame = [0, 0]
    frame_len = len(previous_frame)
    try:
        if frame_len == frame_row_count * frame_col_count * 3:  # check to ensure frame length matches the
            # resolution setting
            for index in range(frame_len):
                if previous_frame[index] != frame[index]:
                    if (abs((previous_frame[index] - frame[index])) / 100) > \
                            capabilities['vision']['deviation_threshold']:
                        dict_key = str(y_vision) + '-' + str(abs((frame_row_count - 1) - x_vision)) + '-' + str(
                            z_vision)
                        vision_dict[dict_key] = frame[index]  # save the value for the changed index to the dict
                z_vision += 1
                if z_vision == 3:
                    z_vision = 0
                    y_vision += 1
                    if y_vision == frame_col_count:
                        y_vision = 0
                        x_vision += 1
        if frame != {}:
            previous_frame_data = frame
    except Exception as e:
        print("Error: Raw data frame does not match frame resolution")
        print("Error due to this: ", e)

    if len(vision_dict) > 3500:
        return {'camera': {name_id: {}}}, previous_frame_data
    else:
        return {'camera': {name_id: vision_dict}}, previous_frame_data


if __name__ == "__main__":
    # Generate runtime dictionary
    previous_data_frame = dict()
    runtime_data = {"cortical_data": {}, "current_burst_id": None, "stimulation_period": None, "feagi_state": None,
                    "feagi_network": None}

    # FEAGI section start
    print("Connecting to FEAGI resources...")

    # address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_outbound_port']

    feagi_host, api_port = FEAGI.feagi_setting_for_registration()
    api_address = FEAGI.feagi_gui_address(feagi_host, api_port)

    stimulation_period_endpoint = FEAGI.feagi_api_burst_engine()
    burst_counter_endpoint = FEAGI.feagi_api_burst_counter()

    runtime_data["feagi_state"] = FEAGI.feagi_registration(feagi_host=feagi_host, api_port=api_port)

    print("** **", runtime_data["feagi_state"])
    network_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

    # todo: to obtain this info directly from FEAGI as part of registration
    ipu_channel_address = FEAGI.feagi_inbound(runtime_data["feagi_state"]['feagi_inbound_port_gazebo'])
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = FEAGI.feagi_outbound(network_settings['feagi_host'],
                                               runtime_data["feagi_state"]['feagi_outbound_port'])

    feagi_ipu_channel = FEAGI.pub_initializer(ipu_channel_address)
    feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
    # FEAGI section ends

    win = visual.Window((600, 600), allowGUI=False, winType='pyglet')

    # Initialize some stimuli
    fixSpot = visual.GratingStim(win, tex="none", mask="gauss",
                                 pos=(0, 0), size=(0.05, 0.05), color='black', autoLog=False)
    grating = visual.GratingStim(win, pos=(0.5, 0),
                                 tex="sin", mask="gauss",
                                 color=[1.0, 0.5, -1.0],
                                 size=(1.0, 1.0), sf=(3, 0),
                                 autoLog=False)  # autologging not useful for dynamic stimuli
    myMouse = event.Mouse()  # will use win by default

    # print(dotPatch)

    message = visual.TextStim(win, text='Any key to quit', pos=(0, -0.5))
    trialClock = core.Clock()
    previous_frame_data = dict()
    msg_counter = runtime_data["feagi_state"]['burst_counter']

    while not event.getKeys():
        message_from_feagi = feagi_opu_channel.receive()
        mouse_dX, mouse_dY = myMouse.getRel()
        mouse1, mouse2, mouse3 = myMouse.getPressed()
        if mouse1:
            grating.setSF(mouse_dX, '+')
        elif mouse3:
            grating.setPos([mouse_dX, mouse_dY], '+')
        else:
            fixSpot.setPos(myMouse.getPos())

        # Handle the wheel(s):
        # dY is the normal mouse wheel, but some have a dX as well
        wheel_dX, wheel_dY = myMouse.getWheelRel()
        grating.setOri(wheel_dY * 5, '+')

        # get rid of other, unprocessed events
        event.clearEvents()

        # Do the drawing
        fixSpot.draw()
        # grating.setPhase(0.05, '+')  # advance 0.05 cycles per frame
        # grating.draw()
        pixels = win._getFrame()
        pixels = np.array(pixels)
        win.flip()
        retina_data = retina.frame_split(pixels)
        rgb = dict()
        rgb['camera'] = dict()
        # dim = (16, 16)
        # resized = cv2.resize(pixels, dim, interpolation=cv2.INTER_AREA)
        # n, m = 16, 16  # We need to implement that to improve way to code instead of a fixed hardcoded numbers.
        # pixels = ndarray_to_list(resized)
        if previous_data_frame == {}:
            for i in retina_data:
                previous_name = str(i) + "_prev"
                previous_data_frame[previous_name] = {}
        for i in retina_data:
            name = i
            if 'prev' not in i:
                data = ndarray_to_list(retina_data[i])
                if 'C' in i:
                    previous_name = str(i) + "_prev"
                    rgb_data, previous_data_frame[previous_name] = get_rgb(data,
                                                                           capabilities['vision'][
                                                                               'central_vision_compression'],
                                                                           previous_data_frame[previous_name], name)
                else:
                    previous_name = str(i) + "_prev"
                    rgb_data, previous_data_frame[previous_name] = get_rgb(data,
                                                                           capabilities['vision'][
                                                                               'peripheral_vision_compression'],
                                                                           previous_data_frame[previous_name], name)
                for a in rgb_data['camera']:
                    rgb['camera'][a] = rgb_data['camera'][a]
        # pixels_changed = get_rgb(pixels, n, m, name)
        try:
            if "data" not in message_to_feagi:
                message_to_feagi["data"] = dict()
            if "sensory_data" not in message_to_feagi["data"]:
                message_to_feagi["data"]["sensory_data"] = dict()
            message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
        except Exception as e:
            pass
        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        sleep(network_settings['feagi_burst_speed'])
        feagi_ipu_channel.send(message_to_feagi)
        message_to_feagi.clear()
    win.close()
    core.quit()
