#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Demo of dot kinematogram
"""

from psychopy import visual, event, core
import numpy as np


def ndarray_to_list(array):
    array = array.flatten()
    new_list = (array.tolist())
    return new_list


def get_rgb(frame):
    vision_dict = dict()
    frame_row_count = 600
    frame_col_count = 600

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
                        print("CHANGED FRAME: ", frame[index])
                        dict_key = str(x_vision) + '-' + str(y_vision) + '-' + str(z_vision)
                        print("dict: ", dict_key)
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


win = visual.Window((600, 600), allowGUI=False, winType='pyglet')

# Initialize some stimuli
dotPatch = visual.DotStim(win, color=(1.0, 1.0, 1.0), dir=270,
                          nDots=1, fieldShape='circle', fieldPos=(0.0, 0.0), fieldSize=1,
                          dotLife=5,  # number of frames for each dot to be drawn
                          signalDots='same',  # are signal dots 'same' on each frame? (see Scase et al)
                          noiseDots='position',  # do the noise dots follow random- 'walk', 'direction', or 'position'
                          speed=0.0001, coherence=0.9)

print(dotPatch)

message = visual.TextStim(win, text='Any key to quit', pos=(0, -0.5))
trialClock = core.Clock()
previous_frame_data = dict()

while not event.getKeys():
    pixels = win._getFrame()
    pixels = np.array(pixels)
    print("BEFORE RESIZE: ", np.shape(pixels))
    pixels.resize(50, 50)
    print("AFTER RESIZE: ", np.shape(pixels))
    # pixels = ndarray_to_list(pixels)
    # pixels_changed = get_rgb(pixels)
    # print(pixels_changed)
    dotPatch.draw()
    message.draw()
    win.flip()  # make the drawn things visible

    event.clearEvents('mouse')  # only really needed for pygame windows

print(win.fps())
win.close()
core.quit()

# The contents of this file are in the public domain.
