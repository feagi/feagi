#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Demo of dot kinematogram
"""

import random
import requests
import numpy as np
from feagi_agent import retina as retina
from feagi_agent import feagi_interface as FEAGI

from time import sleep
from configuration import *
from datetime import datetime
from psychopy import visual, event, core, misc


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
    redPill = visual.Circle(win, size=[0.1, 0.4], fillColor='red')
    bluePill = visual.Circle(win, size=[0.1, 0.4], fillColor='blue')
    myMouse = event.Mouse()  # will use win by default

    # print(dotPatch)

    message = visual.TextStim(win, text='Any key to quit', pos=(0, -0.5))
    trialClock = core.Clock()
    previous_frame_data = dict()
    msg_counter = runtime_data["feagi_state"]['burst_counter']

    redTheta = 0
    blueTheta = 0
    X = 0
    Y = 0
    flag = False
    mouse_flag = False
    redPill.pos = misc.pol2cart(redTheta, radius=0.6)
    rgb = dict()
    rgb['camera'] = dict()

    while not event.getKeys():
        message_from_feagi = feagi_opu_channel.receive()
        mouse_dX, mouse_dY = myMouse.getRel()
        mouse1, mouse2, mouse3 = myMouse.getPressed()

        # Handle the wheel(s):
        # dY is the normal mouse wheel, but some have a dX as well
        # wheel_dX, wheel_dY = myMouse.getWheelRel()
        # grating.setOri(wheel_dY * 5, '+')
        redPill.draw()

        # Do the drawing
        fixSpot.draw()
        pixels = np.array(win._getFrame()) #FULL frame of world
        win.flip()
        pixels = retina.pan(pixels, capabilities['camera']["field_of_vision_origin"], capabilities['camera'][
                                      "field_of_vision_x"], capabilities['camera'][
                                      "field_of_vision_y"]) # Snippet of field of vision
        retina_data = retina.frame_split(pixels, capabilities['camera']['retina_width_percent'],
                                         capabilities['camera']['retina_height_percent']) # Create cortical area with 64 64 and 8 8
        for i in retina_data:
            if 'C' in i:
                retina_data[i] = retina.center_data_compression(retina_data[i],
                                                                capabilities['camera']["central_vision_compression"]
                                                                )
            else:
                retina_data[i] = retina.center_data_compression(retina_data[i],
                                                                capabilities['camera']
                                                                ['peripheral_vision_compression'])
        opu_data = FEAGI.opu_processor(message_from_feagi)
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
        if opu_data is not None:
            if 'motor' in opu_data:
                if opu_data['motor']:
                    for i in opu_data['motor']:
                        if i // 2 == 0:
                            if i % 2 == 0:
                                X += opu_data['motor'][i] / 100
                            else:
                                X -= opu_data['motor'][i] / 100
                        if i // 2 == 1:
                            if i % 2 == 0:
                                Y += opu_data['motor'][i] / 100
                            else:
                                Y -= opu_data['motor'][i] / 100
                if opu_data['misc']:
                    for i in opu_data['misc']:
                        if i == 0:
                            mouse1 = 1
                        if i == 1:
                            mouse2 = 1
                        if i == 2:
                            mouse3 = 1
                fixSpot.setPos([X, Y])
            if 'oculomotor' in opu_data:
                for i in opu_data['oculomotor']:
                    print("I: ", i)
                    if i == 0:
                        capabilities['camera']['field_of_vision_origin'][0] = \
                            capabilities['camera']['field_of_vision_origin'][0] + opu_data['oculomotor'][i]
                    if i == 1:
                        capabilities['camera']['field_of_vision_origin'][0] = \
                            capabilities['camera']['field_of_vision_origin'][0] - opu_data['oculomotor'][i]
                    if i == 2:
                        capabilities['camera']['field_of_vision_origin'][1] = \
                            capabilities['camera']['field_of_vision_origin'][1] + opu_data['oculomotor'][i]
                    if i == 3:
                        capabilities['camera']['field_of_vision_origin'][1] = \
                            capabilities['camera']['field_of_vision_origin'][1] - opu_data['oculomotor'][i]

        # Psychopy game start here
        # if mouse1:
        #     print(fixSpot.pos)
        #     for i in range(10):
        #         for y in range(10):
        #             print("*" * 5)
        # if mouse2:
        #     for i in range(10):
        #         for y in range(10):
        #             print("@" * 5)
        # if mouse3:
        #     for i in range(10):
        #         for y in range(10):
        #             print("z" * 5)
        if mouse1 == 1:
            if redPill.contains(fixSpot.pos):
                rnd_number = random.randint(0, 2)
                if rnd_number == 0:
                    redPill = visual.Rect(win, size=[0.5, 0.5], fillColor='blue')
                elif rnd_number == 1:
                    redPill = visual.Circle(win, size=[0.5, 0.5], fillColor='red')
                elif rnd_number == 2:
                    redPill = visual.Polygon(win, size=[0.5, 0.5], fillColor='yellow')
                redPill.pos = (random.uniform(-1, 1), random.uniform(-1, 1))

        # Psychopy game ends
        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        # msg_counter += 1
        # flag += 1
        # if flag == 10:
        #     feagi_burst_speed = requests.get(api_address + stimulation_period_endpoint).json()
        #     feagi_burst_counter = requests.get(api_address + burst_counter_endpoint).json()
        #     flag = 0
        #     if msg_counter < feagi_burst_counter:
        #         feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
        #         if feagi_burst_speed != network_settings['feagi_burst_speed']:
        #             network_settings['feagi_burst_speed'] = feagi_burst_speed
        feagi_ipu_channel.send(message_to_feagi)
        # sleep(network_settings['feagi_burst_speed'])
        message_to_feagi.clear()
        for i in rgb['camera']:
            rgb['camera'][i].clear()
    win.close()
    core.quit()
