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
import traceback
from datetime import datetime
from feagi_agent import pns_gateway as pns

genome_tracker = 0


def get_device_of_vision(device):
    """
    Obtain the camera source and bind it using the provided address.

    Args:
    - device: The path to the file, video, or webcam. Webcam should be an integer number.

    Returns:
    - An address corresponding to the webcam source, enabling its use across different files.
    """
    return cv2.VideoCapture(device)


def vision_frame_capture(device, RGB_flag=True):
    """
    Capture frames from the specified `device`, which represents the camera source.

    Args:
    - device: The camera device obtained using the `get_device_of_vision()` function.
    - RGB_flag: A boolean indicating whether to retrieve data in RGB format (default: True).
      If set to False, the function returns grayscale data.

    Returns:
    - An nd.array representing the captured frame data. For RGB, it contains three dimensions;
      for grayscale, it displays a single dimension.
      Example format: [[x, y, z], [x, y, z]].
    """
    start_time = datetime.now()
    check, frame = device.read()  # 0 is the default
    # print("vision_frame_capture time total: ", (datetime.now() - start_time).total_seconds())
    if RGB_flag:
        return frame, datetime.now(), check
    else:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), datetime.now(), check


def vision_region_coordinates(frame_width, frame_height, x1, x2, y1, y2, camera_index):
    """
    Calculate coordinates for nine different regions within a frame based on given percentages.

    This function computes the coordinates for nine regions within a frame, defined by x1, x2,
    y1, and y2 percentages. These percentages indicate the position of a point within the frame's
    width and height.

    Inputs:
    - frame_width: Integer, width of the frame.
    - frame_height: Integer, height of the frame.
    - x1, x2, y1, y2: integers representing percentages (0 to 100) along x-axis
    and y-axis.
                      For example, x1=50, y1=40 corresponds to 50% and 40%.

    Output:
    - region_coordinates: Dictionary containing coordinates for nine different regions:
                          'TL', 'TM', 'TR', 'ML', '_C', 'MR', 'LL', 'LM', 'LR'.
                          Each region has its respective coordinates within the frame.

    Note: Make sure that x1, x2, y1, and y2 are valid percentage values within the range of 0 to
    100.
    """
    start_time = datetime.now()
    x1_prime = int(frame_width * (x1 / 100))
    x2_prime = x1_prime + int((frame_width - x1_prime) * (x2 / 100))
    y1_prime = int(frame_height * (y1 / 100))
    y2_prime = y1_prime + int((frame_height - y1_prime) * (y2 / 100))

    region_coordinates = dict()
    region_coordinates[camera_index + 'TL'] = [0, 0, x1_prime, y1_prime]
    region_coordinates[camera_index + 'TM'] = [x1_prime, 0, x2_prime, y1_prime]
    region_coordinates[camera_index + 'TR'] = [x2_prime, 0, frame_width, y1_prime]
    region_coordinates[camera_index + 'ML'] = [0, y1_prime, x1_prime, y2_prime]
    region_coordinates[camera_index + '_C'] = [x1_prime, y1_prime, x2_prime, y2_prime]
    region_coordinates[camera_index + 'MR'] = [x2_prime, y1_prime, frame_width, y2_prime]
    region_coordinates[camera_index + 'LL'] = [0, y2_prime, x1_prime, frame_height]
    region_coordinates[camera_index + 'LM'] = [x1_prime, y2_prime, x2_prime, frame_height]
    region_coordinates[camera_index + 'LR'] = [x2_prime, y2_prime, frame_width, frame_height]
    # print("vision_region_coordinates time total: ", (datetime.now() - start_time).total_seconds())
    return region_coordinates


def split_vision_regions(coordinates, raw_frame_data):
    """
    Split a frame into separate regions based on provided coordinates.

    This function takes the output coordinates from the 'vision_region_coordinates()' function
    and the raw frame data, then splits the frame into nine distinct regions according to those
    coordinates.

    Inputs:
    - coordinates: Dictionary containing the coordinates for nine regions, usually obtained
                   from the 'vision_region_coordinates()' function.
    - raw_frame_data: The original frame data or image used for splitting into regions.

    Output:
    - Display: Visual representation or display of all nine regions independently within the frame.
    """

    start_time = datetime.now()
    frame_segments = dict()
    for region in coordinates:
        frame_segments[region] = raw_frame_data[coordinates[region][1]:coordinates[region][3],
                                 coordinates[region][0]:coordinates[region][2]]
    # print("split_vision_regions time total: ", (datetime.now() - start_time).total_seconds())
    return frame_segments


def downsize_regions(frame, resize):
    """
    Downsize regions within a frame using specified width and height for compression.

    This function utilizes the resize parameter to compress regions within a frame obtained from
     FEAGI's API.
    The frame should be represented as a NumPy ndarray.

    Inputs:
    - frame: NumPy ndarray representing the image/frame data.
    - resize: Tuple containing width and height values for compression.
              Example: (8, 8), (64, 64), (64, 32)

    Output:
    - compressed_dict: Dictionary containing compressed data for nine regions.
                       Each region will be represented within the compressed_dict.

    Make sure that the 'frame' input is a valid NumPy ndarray and the 'resize' parameter contains
    appropriate width and height values for compression.
    """
    start_time = datetime.now()
    if resize[2] == 3:
        compressed_dict = cv2.resize(frame, [resize[0], resize[1]], interpolation=cv2.INTER_AREA)
    if resize[2] == 1:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        compressed_dict = cv2.resize(frame, [resize[0], resize[1]], interpolation=cv2.INTER_AREA)
    # print("downsize_regions time total: ", (datetime.now() - start_time).total_seconds())
    return compressed_dict


def create_feagi_data(significant_changes, current, shape):
    start_time = datetime.now()
    feagi_data = {}
    size_of_frame = shape
    for x in range(size_of_frame[0]):
        for y in range(size_of_frame[1]):
            for z in range(size_of_frame[2]):
                if significant_changes[x, y, z]:
                    key = f'{y}-{(size_of_frame[1] - 1) - x}-{z}'
                    feagi_data[key] = int(current[x, y, z])
    # print("C change_detector_optimized time total: ",
    #       (datetime.now() - start_time).total_seconds())
    return feagi_data


def create_feagi_data_grayscale(significant_changes, current, shape):
    start_time = datetime.now()
    feagi_data = {}
    size_of_frame = shape
    for x in range(size_of_frame[0]):
        for y in range(size_of_frame[1]):
            if significant_changes[x, y]:
                key = f'{y}-{(size_of_frame[1] - 1) - x}-{0}'
                feagi_data[key] = int(current[x, y])
    return feagi_data


def change_detector_grayscale(previous, current, capabilities):
    """
    Detects changes between previous and current frames and checks against a threshold.

    Compares the previous and current frames to identify differences. If the difference
    exceeds a predefined threshold (iso), it records the change in a dictionary for Feagi.

    Inputs:
    - previous: Dictionary with 'cortical' keys containing NumPy ndarray frames.
    - current: Dictionary with 'cortical' keys containing NumPy ndarray frames.

    Output:
    - Dictionary containing changes in the ndarray frames.
    """

    # Using cv2.absdiff for optimized difference calculation
    if current.shape == previous.shape:

        if capabilities['camera']['snap'] == []:
            difference = cv2.absdiff(previous, current)

        else:
            print("Snap!")
            difference = cv2.absdiff(0, current)
            print(current)

        thresholded = cv2.threshold(difference, capabilities['camera']['iso_default'][0],
                                    capabilities['camera']['iso_default'][1],
                                    cv2.THRESH_TOZERO)[1]

        thresholded = cv2.threshold(thresholded, capabilities['camera']['iso_default'][0],
                                    capabilities['camera']['iso_default'][1],
                                    cv2.THRESH_TOZERO_INV)[1]
        # Convert to boolean array for significant changes
        significant_changes = thresholded > 0

        feagi_data = create_feagi_data_grayscale(significant_changes, current, previous.shape)
        #
        # print("change_detector_optimized time total: ",
        #       (datetime.now() - start_time).total_seconds())
    else:
        return {}
    return feagi_data


def change_detector(previous, current, capabilities):
    """
    Detects changes between previous and current frames and checks against a threshold.

    Compares the previous and current frames to identify differences. If the difference
    exceeds a predefined threshold (iso), it records the change in a dictionary for Feagi.

    Inputs:
    - previous: Dictionary with 'cortical' keys containing NumPy ndarray frames.
    - current: Dictionary with 'cortical' keys containing NumPy ndarray frames.

    Output:
    - Dictionary containing changes in the ndarray frames.
    """

    # Using cv2.absdiff for optimized difference calculation
    if current.shape == previous.shape:
        difference = cv2.absdiff(previous, current)
        if capabilities['camera']['snap'] == []:
            thresholded = cv2.threshold(difference, capabilities['camera']['iso_default'][0],
                                        capabilities['camera']['iso_default'][1],
                                        cv2.THRESH_BINARY)[1]
            # Convert to boolean array for significant changes
            significant_changes = thresholded > 0
        else:
            thresholded = cv2.threshold(difference, 0, 255,
                                        cv2.THRESH_BINARY)[1]
            # Convert to boolean array for significant changes
            significant_changes = thresholded > 0

        # print("RGB signifcant change: ", significant_changes)

        feagi_data = create_feagi_data(significant_changes, current, previous.shape)
        #
        # print("change_detector_optimized time total: ",
        #       (datetime.now() - start_time).total_seconds())
    else:
        return {}
    return dict(feagi_data)


def detect_change_edge(raw_frame, capabilities, camera_index, resize_list, previous_frame_data,
                       rgb):
    region_coordinates = vision_region_coordinates(raw_frame.shape[1],
                                                   raw_frame.shape[0], capabilities['camera'][
                                                       'gaze_control'][0], capabilities['camera'][
                                                       'gaze_control'][1],
                                                   capabilities['camera']['pupil_control'][0],
                                                   capabilities['camera']['pupil_control'][1],
                                                   camera_index)
    segmented_frame_data = split_vision_regions(coordinates=region_coordinates,
                                                raw_frame_data=raw_frame)
    compressed_data = dict()
    for cortical in segmented_frame_data:
        compressed_data[cortical] = downsize_regions(
            segmented_frame_data[cortical],
            resize_list[cortical])
    vision_dict = dict()

    # for segment in compressed_data:
    #     cv2.imshow(segment, compressed_data[segment])
    # if cv2.waitKey(30) & 0xFF == ord('q'):
    #     pass
    for get_region in compressed_data:
        if resize_list[get_region][2] == 3:
            if previous_frame_data != {}:
                vision_dict[get_region] = change_detector(
                    previous_frame_data[get_region],
                    compressed_data[get_region],
                    capabilities)
        else:
            if previous_frame_data != {}:
                vision_dict[get_region] = change_detector_grayscale(
                    previous_frame_data[get_region],
                    compressed_data[get_region],
                    capabilities)
    previous_frame_data = compressed_data
    rgb['camera'] = vision_dict
    return previous_frame_data, rgb


def obtain_cortical_vision_size(camera_index, response):
    size_list = {}
    data = response.json()
    items = [camera_index + "_C", camera_index + "LL", camera_index + "LM", camera_index + "LR",
             camera_index + "MR", camera_index + "ML", camera_index + "TR", camera_index + "TL",
             camera_index + "TM"]
    for name_from_data in data:
        for fetch_name in items:
            if fetch_name in name_from_data:
                name = name_from_data.replace("iv", "")
                dimension_array = data[name_from_data]["dimensions"][0], \
                                  data[name_from_data]["dimensions"][1], \
                                  data[name_from_data]["dimensions"][2]
                size_list[name] = dimension_array
    return size_list


def vision_progress(capabilities, previous_genome_timestamp, feagi_opu_channel, api_address,
                    feagi_settings, raw_frame):
    message_from_feagi = pns.efferent_signaling(feagi_opu_channel)
    if message_from_feagi is not None:
        # OPU section STARTS
        # Obtain the size of aptr
        if pns.global_aptr_cortical_size is None:
            pns.global_aptr_cortical_size = pns.check_aptr(
                api_address + '/v1/feagi/genome/cortical_area?cortical_area=o_aptr')
        # Update the aptr
        capabilities = pns.fetch_aperture_data(message_from_feagi, capabilities,
                                               pns.global_aptr_cortical_size)
        # Update the ISO
        capabilities = pns.fetch_iso_data(message_from_feagi, capabilities,
                                          pns.global_aptr_cortical_size)
        # Update the vres
        capabilities = pns.fetch_resolution_selected(message_from_feagi, capabilities)
        # Update the aceture
        capabilities = pns.fetch_vision_acuity(message_from_feagi, capabilities)
        genome_changed = pns.detect_genome_change(message_from_feagi)
        # This applies to cortical change.
        if genome_changed != previous_genome_timestamp:
            response = pns.grab_geometry()
            capabilities['camera']['size_list'] = \
                obtain_cortical_vision_size(capabilities['camera']["index"], response)
            previous_genome_timestamp = message_from_feagi["genome_changed"]
        capabilities = pns.obtain_snap_data(raw_frame, message_from_feagi, capabilities)
        capabilities = pns.monitor_switch(message_from_feagi, capabilities)
        capabilities = pns.gaze_control_update(message_from_feagi, capabilities)
        capabilities = pns.pupil_control_update(message_from_feagi, capabilities)
        feagi_settings['feagi_burst_speed'] = pns.check_refresh_rate(message_from_feagi,
                                                                     feagi_settings[
                                                                         'feagi_burst_speed'])
    return capabilities, previous_genome_timestamp, feagi_settings['feagi_burst_speed']


def update_astype(data):
    return data.astype(np.uint8)


def RGB_list_to_ndarray(data, size):
    new_rgb = np.array(data)
    new_rgb = new_rgb.reshape(size[0], size[1], 3)
    return new_rgb


def flip_video(data):
    return cv2.flip(data, 1)
