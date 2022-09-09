#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Demo of dot kinematogram
"""

import numpy as np
import feagi_interface as FEAGI
from time import sleep
from configuration import *
from datetime import datetime
from psychopy import visual, event, core


def ndarray_to_list(array):
    array = array.flatten()
    new_list = (array.tolist())
    return new_list


def get_rgb(frame, w, h):
    vision_dict = dict()
    frame_row_count = w
    frame_col_count = h

    x_vision = 0  # row counter
    y_vision = 0  # col counter
    z_vision = 0  # RGB counter

    try:
        previous_frame = previous_frame_data[0]
    except Exception:
        previous_frame = [0, 0]
    frame_len = len(previous_frame)
    try:
        if frame_len == frame_row_count * frame_col_count * 3:  # check to ensure frame length matches the
            # resolution setting
            for index in range(frame_len):
                if previous_frame[index] != frame[index]:
                    if (abs((previous_frame[index] - frame[index])) / 100) > \
                            0:
                        # print("CHANGED FRAME: ", frame[index])
                        dict_key = str(x_vision) + '-' + str(y_vision) + '-' + str(z_vision)
                        # print("dict: ", dict_key)
                        vision_dict[dict_key] = frame[index]  # save the value for the changed index to the dict
                z_vision += 1
                if z_vision == 3:
                    z_vision = 0
                    y_vision += 1
                    if y_vision == frame_col_count:
                        y_vision = 0
                        x_vision += 1
        if frame != {}:
            previous_frame_data[0] = frame
    except Exception as e:
        print("Error: Raw data frame does not match frame resolution")
        print("Error due to this: ", e)

    return {'vision': vision_dict}


if __name__ == "__main__":
    # Generate runtime dictionary
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
    dotPatch = visual.DotStim(win, color=(1.0, 1.0, 1.0), dir=270,
                              nDots=500, fieldShape='circle', fieldPos=(0.0, 0.0), fieldSize=3,
                              dotLife=5,  # number of frames for each dot to be drawn
                              signalDots='same',  # are signal dots 'same' on each frame? (see Scase et al)
                              noiseDots='position',
                              # do the noise dots follow random- 'walk', 'direction', or 'position'
                              speed=0.0001, coherence=0.9)

    print(dotPatch)

    message = visual.TextStim(win, text='Any key to quit', pos=(0, -0.5))
    trialClock = core.Clock()
    previous_frame_data = dict()
    msg_counter = runtime_data["feagi_state"]['burst_counter']

    while not event.getKeys():
        message_from_feagi = feagi_opu_channel.receive()
        pixels = win._getFrame()
        pixels = np.array(pixels)
        # print("BEFORE RESIZE: ", np.shape(pixels))
        n, m = 600, 600  # We need to implement that to improve way to code instead of a fixed hardcoded numbers.
        # pixels.resize(16, 16, 3)
        # print("AFTER RESIZE: ", np.shape(pixels))
        pixels = ndarray_to_list(pixels)
        pixels_changed = get_rgb(pixels, n, m)
        dotPatch.draw()
        message.draw()
        win.flip()  # make the drawn things visible
        try:
            if "data" not in message_to_feagi:
                message_to_feagi["data"] = dict()
            if "sensory_data" not in message_to_feagi["data"]:
                message_to_feagi["data"]["sensory_data"] = dict()
            message_to_feagi["data"]["sensory_data"]['camera'] = pixels_changed['vision']
        except Exception as e:
            pass
        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        sleep(network_settings['feagi_burst_speed'])
        feagi_ipu_channel.send(message_to_feagi)
        message_to_feagi.clear()
    win.close()
    core.quit()
