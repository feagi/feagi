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
import requests
from datetime import datetime


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
    check, frame = device.read()  # 0 is the default
    if RGB_flag:
        return frame, datetime.now()
    else:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), datetime.now()


def vision_region_coordinates(frame_width, frame_height, x1, x2, y1, y2):
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
    x1_prime = int(frame_width * (x1 / 100))
    x2_prime = x1_prime + int((frame_width - x1_prime) * (x2 / 100))
    y1_prime = int(frame_height * (y1 / 100))
    y2_prime = y1_prime + int((frame_height - y1_prime) * (y2 / 100))

    region_coordinates = dict()
    region_coordinates['00TL'] = [0, 0, x1_prime, y1_prime]
    region_coordinates['00TM'] = [x1_prime, 0, x2_prime, y1_prime]
    region_coordinates['00TR'] = [x2_prime, 0, frame_width, y1_prime]
    region_coordinates['00ML'] = [0, y1_prime, x1_prime, y2_prime]
    region_coordinates['00_C'] = [x1_prime, y1_prime, x2_prime, y2_prime]
    region_coordinates['00MR'] = [x2_prime, y1_prime, frame_width, y2_prime]
    region_coordinates['00LL'] = [0, y2_prime, x1_prime, frame_height]
    region_coordinates['00LM'] = [x1_prime, y2_prime, x2_prime, frame_height]
    region_coordinates['00LR'] = [x2_prime, y2_prime, frame_width, frame_height]
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

    frame_segments = dict()
    for region in coordinates:
        frame_segments[region] = raw_frame_data[coordinates[region][1]:coordinates[region][3],
                                 coordinates[region][0]:coordinates[region][2]]
    return frame_segments


def downsize_regions(frame, resize, RGB_flag=True):
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
    if RGB_flag:
        compressed_dict = cv2.resize(frame, resize, interpolation=cv2.INTER_AREA)
    else:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        compressed_dict = cv2.resize(frame, resize, interpolation=cv2.INTER_AREA)
    return compressed_dict


def change_detector(previous, current):
    feagi_data = dict()
    threshold = 10
    # print("$$$$$------------------------------shape", previous.shape)
    if len(previous.shape) < 3:
        pass
        # for x in range(previous.shape[0]):
        #     for y in range(previous.shape[1]):
        #         if previous[x, y] != current[x, y]:
        #             if (abs((previous[x, y] - current[x, y])) * 100 / 255) > threshold:
        #                 # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        #                 # print((abs((previous[x, y] - current[x, y])) / 100), previous[x, y],
        #                 #       " ", current[x, y])
        #                 # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n")
        #                 key = f'{x}-{y}-{0}'
        #                 feagi_data[key] = (current[x, y])
    else:
        for x in range(previous.shape[0]):
            for y in range(previous.shape[1]):
                # print("UPDATED Y: ", )
                for z in range(previous.shape[2]):
                    # print("z: ", z, " shape: ", previous.shape[2])
                    # print("^^^___" * 20)
                    if previous[x, y, z] != current[x, y, z]:
                        difference = abs(int(previous[x, y, z]) - int(current[x, y, z]))
                        # print("difference-1", difference)
                        difference = difference * 100 / 255
                        # print("difference-2", difference)
                        # print("$%_" * 20)
                        if difference > threshold:
                            # print("####################################")
                            # print("previous:", previous[x, y, z])
                            # print("current:", current[x, y, z])
                            # print("difference_:", difference)
                            # print("####################################\n")
                            key = f'{y}-{64-x}-{z}'
                            feagi_data[key] = (current[y, x, z])
    return feagi_data

cam = get_device_of_vision(2)
url = 'http://127.0.0.1:8000/v1/feagi/genome/cortical_area/geometry'
response = requests.get(url)
data = response.json()
items = ["_C", "LL", "LM", "LR", "MR", "ML", "TR", "TL", "TM"]
resize_list = {}
for i in data:
    for x in items:
        if x in i:
            dimension_array = data[i]["dimensions"][0], data[i]["dimensions"][1]
            resize_list[x] = dimension_array
while True:
    raw_frame, time = vision_frame_capture(cam)
    region_coordinates = vision_region_coordinates(frame_width=raw_frame.shape[1],
                                                   frame_height=raw_frame.shape[0],
                                                   x1=25, x2=50,
                                                   y1=25, y2=50)
    segmented_frame_data = split_vision_regions(coordinates=region_coordinates,
                                                raw_frame_data=raw_frame)
    compressed_data = dict()
    for i in segmented_frame_data:
        if "_C" in i:
            compressed_data[i] = downsize_regions(segmented_frame_data[i], resize_list[i])
        else:
            compressed_data[i] = downsize_regions(segmented_frame_data[i], resize_list[i], False)
    for segment in compressed_data:
        cv2.imshow(segment, compressed_data[segment])
    # cv2.imshow("full", raw_frame)
    # # for i in vision_dict:
    # #     print(vision_dict)
    #     # cv2.imshow(i, vision_dict[i])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
