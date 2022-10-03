#!/usr/bin/env python
# The contents of this file are in the public domain.

import cv2
from configuration import *


def resize_calculate(a, b, p):
    after_percent = p / 100
    remain_percent = ((100 - p) / 2) / 100
    c = b * remain_percent
    d = b * (after_percent + remain_percent)
    return int(c), int(d)


def snippet_rgb(inner_width, percent_a, inner_height, percent_b):
    data0, data1 = resize_calculate(0, inner_width, percent_a)
    data2, data3 = resize_calculate(0, inner_height, percent_b)
    return data0, data1, data2, data3


def center_data_compression(frame):
    compressed = cv2.resize(frame, capabilities['camera']["central_vision_compression"], interpolation=cv2.INTER_AREA)
    return compressed


def peripheral_data_compression(frame):
    compressed = cv2.resize(frame, capabilities['camera']["peripheral_vision_compression"],
                            interpolation=cv2.INTER_AREA)
    return compressed


def feagi_language(dictionary, previous_frame_data):
    vision_dict = dict()
    frame_row_count = capabilities['camera']['width']
    frame_col_count = capabilities['camera']['height']

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
                if previous_frame[index] != dictionary[index]:
                    if (abs((previous_frame[index] - dictionary[index])) / 100) > \
                            capabilities['camera']['deviation_threshold']:
                        dict_key = str(x_vision) + '-' + str(y_vision) + '-' + str(z_vision)
                        vision_dict[dict_key] = dictionary[index]  # save the value for the changed index to the dict
                z_vision += 1
                if z_vision == 3:
                    z_vision = 0
                    y_vision += 1
                    if y_vision == frame_col_count:
                        y_vision = 0
                        x_vision += 1
        if dictionary != {}:
            previous_frame_data[0] = dictionary
    except Exception as e:
        print("Error: Raw data frame does not match frame resolution")
        print("Error due to this: ", e)

    return {'camera': vision_dict}


def frame_split(frame):
    vision = dict()
    full_data = frame.shape
    width_data1, width_data2, height_data1, height_data2 = snippet_rgb(full_data[0],
                                                                       capabilities['camera']['retina_width_percent'],
                                                                       full_data[1],
                                                                       capabilities['camera']['retina_height_percent'])
    print("C and D for width: ", width_data1, " ", width_data2)
    print("C and D for height: ", height_data1, " ", height_data2)
    TL = frame[0:width_data1, 0:height_data1]
    TM = frame[0:width_data1, height_data1:height_data2]
    TR = frame[0:width_data1, height_data2:]
    ML = frame[width_data1:width_data2, 0:height_data1]
    C = frame[width_data1:width_data2, height_data1:height_data2]
    MR = frame[width_data1:width_data2, height_data2:]
    LL = frame[width_data2:, 0:height_data1]
    LM = frame[width_data2:, height_data1: height_data2]
    LR = frame[width_data2:, height_data2:]
    vision['C'] = center_data_compression(C)
    vision['TL'] = peripheral_data_compression(TL)
    vision['TM'] = peripheral_data_compression(TM)
    vision['TR'] = peripheral_data_compression(TR)
    vision['ML'] = peripheral_data_compression(ML)
    vision['MR'] = peripheral_data_compression(MR)
    vision['LL'] = peripheral_data_compression(LL)
    vision['LM'] = peripheral_data_compression(LM)
    vision['LR'] = peripheral_data_compression(LR)
    return vision


def pan(frame):
    """
    No filter involves. No resize or compression. Just return all boxes.
    This is heavily leveraged on the frame_split() function.
    """
    vision = dict()
    vision['vision_box'] = frame[capabilities['vision']["field_of_vision_origin"][1]:
                                 capabilities['vision']["field_of_vision_origin"][1] + capabilities['vision'][
                                     "field_of_vision_y"],
                           capabilities['vision']["field_of_vision_origin"][0]:
                           capabilities['vision']["field_of_vision_origin"][
                               0] + capabilities['vision'][
                               "field_of_vision_x"]]
    return vision
