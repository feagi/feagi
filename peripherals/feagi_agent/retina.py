#!/usr/bin/env python

import cv2


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


def center_data_compression(frame, central_vision_compression):
    """capabilities['camera']["central_vision_compression"]"""
    compressed = cv2.resize(frame, central_vision_compression, interpolation=cv2.INTER_AREA)
    return compressed


def peripheral_data_compression(frame, peripheral_vision_compression):
    """capabilities['camera']["peripheral_vision_compression"]"""
    compressed = cv2.resize(frame, peripheral_vision_compression,
                            interpolation=cv2.INTER_AREA)
    return compressed


def ndarray_to_list(array):
    array = array.flatten()
    new_list = (array.tolist())
    return new_list


def get_rgb(frame, size, previous_frame_data, name_id, deviation_threshold):
    """
    frame should be a full raw rgb after used ndarray_to_list().

    size should be coming from the configuration.capabilities['camera']['peripheral_vision_compression'] or
    configuration.capabilities['camera']['central_vision_compression']

    previous should be held and used by a global dictionary to hold the old data so that way, it can compare and see the
    difference.

    name_id is cortical area's name.

    deviation_threshold is the threshold to reduce the massive red voxels on godot.
    """

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
                    if (abs((previous_frame[index] - frame[index])) / 100) > deviation_threshold:
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


def frame_split(frame, width_percent, height_percent):
    vision = dict()
    full_data = frame.shape
    width_data1, width_data2, height_data1, height_data2 = snippet_rgb(full_data[0],
                                                                       width_percent,
                                                                       full_data[1],
                                                                       height_percent)
    vision['TL'] = frame[0:width_data1, 0:height_data1]
    vision['TM'] = frame[0:width_data1, height_data1:height_data2]
    vision['TR'] = frame[0:width_data1, height_data2:]
    vision['ML'] = frame[width_data1:width_data2, 0:height_data1]
    vision['C'] = frame[width_data1:width_data2, height_data1:height_data2]
    vision['MR'] = frame[width_data1:width_data2, height_data2:]
    vision['LL'] = frame[width_data2:, 0:height_data1]
    vision['LM'] = frame[width_data2:, height_data1: height_data2]
    vision['LR'] = frame[width_data2:, height_data2:]
    return vision