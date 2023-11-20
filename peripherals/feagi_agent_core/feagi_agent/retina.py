#!/usr/bin/env python3
"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

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

import cv2
import numpy as np


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
    try:
        compressed = cv2.resize(frame, central_vision_compression, interpolation=cv2.INTER_AREA)
    except Exception as error:
        compressed = []
    return compressed


def check_previous_data(previous_data_frame, retina_data):
    """
    Verify if `previous_data_frame` is not empty.
    If it is empty, create an empty dictionary with multiple pieces inside using the `retina_data`.
    """

    if previous_data_frame == {}:
        for i in retina_data:
            previous_name = str(i) + "_prev"
            previous_data_frame[previous_name] = {}
        return previous_data_frame
    return previous_data_frame


def detect_change_edge(frame, previous_data_frame, retina_data, current_selected_size,
                       central_resolution, peripheral_resolution, current_iso_selected,
                       aperture_default):
    """
    This function is designed to compare the previous data with the current data and then detect
    which is different, while being handled by an ISO threshold which can be found in configuration.py.

    Parameters:
        frame (ndarray): RGB data.
        previous_data_frame (dict): Previous data containing old RGB values stored in the controller.
        retina_data (dict): Latest RGB data.
        current_selected_size (array): It is capabilities['camera']['current_select'] in the config.
        central_resolution (array): Capabilities['camera']["central_vision_resolution"].
        peripheral_resolution (array): Capabilities['camera']['peripheral_vision_resolution'].
        current_iso_selected (float): Capabilities['camera']['iso_threshold'].
        aperture_default (float): Capabilities['camera']["aperture_default"].
    """

    rgb = {'camera': {}}
    for i in retina_data:
        name = i
        if 'prev' not in i:
            data = ndarray_to_list(retina_data[i])
            if data is False:
                if len(frame) > 0:
                    print("shape: ", frame.shape)
                print("The size: ", current_selected_size, " is not available for this")
                print("Reverting to the original size")
                current_selected_size = []
                frame = {}
                data = []
            if '_C' in i:
                previous_name = str(i) + "_prev"
                rgb_data, previous_data_frame[previous_name] = \
                    get_rgb(data, central_resolution, previous_data_frame[previous_name], name,
                            current_iso_selected, aperture_default)
            else:
                previous_name = str(i) + "_prev"
                rgb_data, previous_data_frame[previous_name] = \
                    get_rgb(data, peripheral_resolution, previous_data_frame[previous_name],
                            name, current_iso_selected, aperture_default)
            for a in rgb_data['camera']:
                rgb['camera'][a] = rgb_data['camera'][a]

    return previous_data_frame, rgb['camera'], current_selected_size


def frame_compression(frame, central_resolution, peripheral_resolution):
    """
    It will compress all pieces using the `central_resolution` and `peripheral_resolution`.
    The key with 'C' will be compressed with central resolution, while the non-'C' keys will use
    peripheral resolution.

    Parameters:
        frame (array): Latest RGB data.
        central_resolution (array): Capabilities['camera']["central_vision_resolution"].
        peripheral_resolution (array): Capabilities['camera']['peripheral_vision_resolution'].
    """

    for i in frame:
        if 'C' in i:
            frame[i] = center_data_compression(frame[i], central_resolution)
        else:
            frame[i] = center_data_compression(frame[i], peripheral_resolution)
    return frame


def peripheral_data_compression(frame, peripheral_vision_compression):
    """capabilities['camera']["peripheral_vision_compression"]"""
    compressed = cv2.resize(frame, peripheral_vision_compression, interpolation=cv2.INTER_AREA)
    return compressed


def ndarray_to_list(array):
    if isinstance(array, np.ndarray) and array.size > 0:
        return array.flatten().tolist()
    return False


def get_rgb(frame, size, previous_frame_data, name_id, deviation_threshold, atpr_level,
            single_RGB=None):
    """
    frame should be a full raw rgb after used ndarray_to_list().

    size should be coming from the configuration.capabilities['camera'][
    'peripheral_vision_compression'] or configuration.capabilities['camera'][
    'central_vision_compression']

    previous should be held and used by a global dictionary to hold the old data so that way,
    it can compare and see the difference.

    name_id is cortical area's name.

    deviation_threshold is the threshold to reduce the massive red voxels on godot.

    single_RGB should be 0 to 2. R = 0, G = 1, B = 2
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
        # if frame_len == frame_row_count * frame_col_count * 3:  # check to ensure frame length
        # matches the
        # resolution setting
        for index in range(frame_len):
            if previous_frame[index] != frame[index]:
                if (abs((previous_frame[index] - frame[index])) / 100) > deviation_threshold:
                    dict_key = str(y_vision) + '-' + \
                               str(abs((frame_col_count - 1) - x_vision)) + '-' + str(z_vision)
                    if single_RGB != None:
                        dict_key = str(y_vision) + '-' + \
                                   str(abs((frame_col_count - 1) - x_vision)) + '-' + str(
                            single_RGB)
                    vision_dict[dict_key] = frame[index]  # save the value for the changed
                    # index to the dict
            z_vision += 1
            if z_vision == 3:
                z_vision = 0
                y_vision += 1
                if y_vision == frame_row_count:
                    y_vision = 0
                    x_vision += 1
        if frame != {}:
            previous_frame_data = frame
    except Exception as e:
        print("Error: Raw data frame does not match frame resolution")
        print("Error due to this: ", e)
        previous_frame_data = {}
    if len(vision_dict) > (frame_row_count * frame_col_count) / atpr_level:
        return {'camera': {name_id: {}}}, previous_frame_data
    else:
        return {'camera': {name_id: vision_dict}}, previous_frame_data


def frame_split(frame, width_percent, height_percent, camera_index: str):
    vision = dict()
    try:
        full_data = frame.shape
        if width_percent == height_percent:
            width_data1, width_data2, height_data1, height_data2 = snippet_rgb(full_data[0],
                                                                               width_percent,
                                                                               full_data[1],
                                                                               height_percent)
            vision[camera_index + '_C'] = frame[width_data1:width_data2, height_data1:height_data2]
            vision[camera_index + 'TL'] = np.zeros((8, 8, 3))
            vision[camera_index + 'TM'] = np.zeros((8, 8, 3))
            vision[camera_index + 'TR'] = np.zeros((8, 8, 3))
            vision[camera_index + 'ML'] = np.zeros((8, 8, 3))
            vision[camera_index + 'MR'] = np.zeros((8, 8, 3))
            vision[camera_index + 'LL'] = np.zeros((8, 8, 3))
            vision[camera_index + 'LM'] = np.zeros((8, 8, 3))
            vision[camera_index + 'LR'] = np.zeros((8, 8, 3))

        else:
            if width_percent == 100:
                width_percent = 100 - 1
            if height_percent == 100:
                height_percent = 100 - 1
            width_data1, width_data2, height_data1, height_data2 = snippet_rgb(full_data[0],
                                                                               width_percent,
                                                                               full_data[1],
                                                                               height_percent)
            vision[camera_index + 'TL'] = frame[0:width_data1, 0:height_data1]
            vision[camera_index + 'TM'] = frame[0:width_data1, height_data1:height_data2]
            vision[camera_index + 'TR'] = frame[0:width_data1, height_data2:]
            vision[camera_index + 'ML'] = frame[width_data1:width_data2, 0:height_data1]
            vision[camera_index + '_C'] = frame[width_data1:width_data2, height_data1:height_data2]
            vision[camera_index + 'MR'] = frame[width_data1:width_data2, height_data2:]
            vision[camera_index + 'LL'] = frame[width_data2:, 0:height_data1]
            vision[camera_index + 'LM'] = frame[width_data2:, height_data1: height_data2]
            vision[camera_index + 'LR'] = frame[width_data2:, height_data2:]
    except AttributeError:
        # print("No visual data to process!")
        pass
    return vision


def pan(frame, origin, x, y):
    """
    No filter involves. No resize or compression. Just return all boxes.
    This is heavily leveraged on the frame_split() function.
    """
    vision = frame[origin[1]:origin[1] + y, origin[0]:origin[0] + x]
    return vision


def obtain_dimension(data, data_type):
    if data_type == "list":
        dimension = np.array(data)
        return dimension.shape
    elif data_type == "ndarray":
        return data.shape


def pitina_to_retina(data, size):
    rgb_value = list(data)
    new_rgb = np.array(rgb_value)
    return new_rgb.reshape(size[1], size[0], 3)


def update_astype(data):
    return data.astype(np.uint8)


def RGBA_list_to_ndarray(data, size):
    new_rgb = np.array(data)
    new_rgb = new_rgb.reshape(size[0], size[1], 4)
    return new_rgb


def RGB_list_to_ndarray(data, size):
    new_rgb = np.array(data)
    new_rgb = new_rgb.reshape(size[0], size[1], 3)
    return new_rgb


def flip_video(data):
    return cv2.flip(data, 1)
