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
    prime_x1 = int(frame_width * (x1 / 100))
    prime_x2 = int((frame_width - prime_x1) * (x2 / 100))
    prime_x3 = int(frame_width - (prime_x1 + prime_x2))
    prime_y1 = int(frame_height * (y1 / 100))
    prime_y2 = int((frame_height - prime_y1) * (y2 / 100))
    prime_y3 = int(frame_height - (prime_x1 + prime_x2))

    vision = dict()
    vision['TL'] = [prime_x1, 0, prime_y1, 0]
    vision['TM'] = [0, prime_x2, prime_y1, 0]
    vision['TR'] = [prime_x3, prime_x2, prime_y1, 0]
    vision['ML'] = [prime_x1, 0, prime_y2, 0]
    vision['C'] = [0, prime_x2, prime_y2, 0]
    vision['MR'] = [prime_x3, prime_x2, prime_y2, 0]
    vision['LL'] = [prime_x1, prime_y3, 0]
    vision['LM'] = [0, prime_x2, prime_y3, 0]
    vision['LR'] = [prime_x2, prime_x3, prime_y3, 0]
    return vision


def split_vision_regions(vision, frame):
    test = dict()
    test['TL'] = frame[0:vision['TL'][0], 0:(vision['TL'][2] + vision['TL'][3])]
    test['TM'] = frame[vision['TL'][0]:vision['TM'][1], 0:vision['TM'][2]+vision['TM'][3]]
    return test

cam = get_device_of_vision(2)
while True:
    pixels, time = vision_frame_capture(cam)
    vision_dict = vision_region_coordinates(1000, 600, 10, 20, 40, 60)
    data = split_vision_regions(vision_dict, pixels)
    print(data)
    for i in data:
        cv2.imshow(i, data[i])
    cv2.imshow("full", pixels)
    # # for i in vision_dict:
    # #     print(vision_dict)
    #     # cv2.imshow(i, vision_dict[i])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
