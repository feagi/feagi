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
import traceback
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
    x1_prime = int(frame_width * (x1 / 100))
    x2_prime = int((frame_width * (x2 / 100) + x1_prime))
    y1_prime = int(frame_height * (y1 / 100))
    y2_prime = int((frame_height * y2 / 100) + y1_prime)

    region_coordinates = dict()
    region_coordinates['TL'] = [0, 0, x1_prime, y1_prime]
    region_coordinates['TM'] = [x1_prime, 0, x2_prime, y1_prime]
    region_coordinates['TR'] = [x2_prime, 0, frame_width, y1_prime]
    region_coordinates['ML'] = [0, y1_prime, x1_prime, y2_prime]
    region_coordinates['_C'] = [x1_prime, y1_prime, x2_prime, y2_prime]
    region_coordinates['MR'] = [x2_prime, y1_prime, frame_width, y2_prime]
    region_coordinates['LL'] = [0, y2_prime, x1_prime, frame_height]
    region_coordinates['LM'] = [x1_prime, y2_prime, x2_prime, frame_height]
    region_coordinates['LR'] = [x2_prime, y2_prime, frame_width, frame_height]
    return region_coordinates


def split_vision_regions(coordinates, raw_frame_data):
    frame_segments = dict()
    for region in coordinates:
        frame_segments[region] = raw_frame_data[coordinates[region][1]:coordinates[region][3],
                                                coordinates[region][0]:coordinates[region][2]]
    return frame_segments


cam = get_device_of_vision(0)
while True:
    raw_frame, time = vision_frame_capture(cam)
    region_coordinates = vision_region_coordinates(frame_width=raw_frame.shape[1],
                                                   frame_height=raw_frame.shape[0],
                                                   x1=25, x2=50,
                                                   y1=25, y2=50)
    segmented_frame_data = split_vision_regions(coordinates=region_coordinates, raw_frame_data=raw_frame)
    for segment in segmented_frame_data:
        cv2.imshow(segment, segmented_frame_data[segment])
    cv2.imshow("full", raw_frame)
    # # for i in vision_dict:
    # #     print(vision_dict)
    #     # cv2.imshow(i, vision_dict[i])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
