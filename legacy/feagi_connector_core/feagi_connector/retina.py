#!/usr/bin/env python3
"""
Copyright 2016-present Neuraville Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""

import cv2
import numpy
import numpy as np
from time import sleep
from datetime import datetime
from feagi_connector import pns_gateway as pns

genome_tracker = 0
previous_genome_timestamp = 0
current_dimension_list = {}
current_mirror_status = False
preview_flag = False


def get_device_of_vision(device):
    """
    Obtain the camera source and bind it using the provided address.

    Args:
    - device: The path to the file, video, or webcam. Webcam should be an integer number.

    Returns:
    - An address corresponding to the webcam source, enabling its use across different files.
    """
    return cv2.VideoCapture(device)


def vision_frame_capture(device=0, rgb_flag=True):
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

    if rgb_flag:
        return frame, datetime.now(), check
    else:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), datetime.now(), check


def vision_region_coordinates(frame_width=None, frame_height=None, x1=None, x2=None, y1=None,
                              y2=None, camera_index="0", size_list=None):
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
                          'TL', 'TM', 'TR', 'ML', '_C', 'MR', 'BL', 'BM', 'BR'.
                          Each region has its respective coordinates within the frame.

    Note: Make sure that x1, x2, y1, and y2 are valid percentage values within the range of 0 to
    100.
    """
    # start_time = datetime.now()
    # Gaze controls
    x1_prime = int(frame_width * (x1 / 100))
    y1_prime = int(frame_height * (y1 / 100))
    # Pupil controls
    x2_prime = min(x1_prime + int(frame_width * x2 / 100), frame_width)
    y2_prime = min(y1_prime + int(frame_height * y2 / 100), frame_height)

    region_coordinates = dict()
    if (camera_index + 'TL') in size_list:
        region_coordinates[camera_index + 'TL'] = [0, 0, x1_prime, y1_prime]
    if (camera_index + 'TM') in size_list:
        region_coordinates[camera_index + 'TM'] = [x1_prime, 0, x2_prime, y1_prime]
    if (camera_index + 'TR') in size_list:
        region_coordinates[camera_index + 'TR'] = [x2_prime, 0, frame_width, y1_prime]
    if (camera_index + 'ML') in size_list:
        region_coordinates[camera_index + 'ML'] = [0, y1_prime, x1_prime, y2_prime]
    if (camera_index + '_C') in size_list:
        region_coordinates[camera_index + '_C'] = [x1_prime, y1_prime, x2_prime, y2_prime]
    if (camera_index + 'MR') in size_list:
        region_coordinates[camera_index + 'MR'] = [x2_prime, y1_prime, frame_width, y2_prime]
    if (camera_index + 'BL') in size_list:
        region_coordinates[camera_index + 'BL'] = [0, y2_prime, x1_prime, frame_height]
    if (camera_index + 'BM') in size_list:
        region_coordinates[camera_index + 'BM'] = [x1_prime, y2_prime, x2_prime, frame_height]
    if (camera_index + 'BR') in size_list:
        region_coordinates[camera_index + 'BR'] = [x2_prime, y2_prime, frame_width, frame_height]
    if (camera_index + 'CC') in size_list:
        region_coordinates[camera_index + 'CC'] = [x1_prime, y1_prime, x2_prime, y2_prime]
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

    # start_time = datetime.now()
    frame_segments = dict()
    for region in coordinates:
        frame_segments[region] = \
            raw_frame_data[coordinates[region][1]:coordinates[region][3], coordinates[region][0]:coordinates[region][2]]
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

    if resize[2] == 3:
        try:
            compressed_dict = cv2.resize(frame, [int(resize[0]), int(resize[1])],
                                         interpolation=cv2.INTER_NEAREST)
            return compressed_dict
        except Exception as e:
            # print("error inside downsize_regions on retina.py: ", e)
            compressed_dict = np.zeros(resize, dtype=np.uint8)
            compressed_dict = update_astype(compressed_dict)
            return compressed_dict

    # In case of grayscale image
    if resize[2] == 1:
        try:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            compressed_dict = cv2.resize(frame, [resize[0], resize[1]],
                                         interpolation=cv2.INTER_AREA)
            return compressed_dict
        except Exception as e:
            # print(e)
            compressed_dict = np.zeros(resize, dtype=np.uint8)
            compressed_dict = update_astype(compressed_dict)
            return compressed_dict


def create_feagi_data(significant_changes, current, shape, index, cortical_name, grayscale=False):
    """
        :param significant_changes: An array of modified data derived from the raw data.
        :param current: The current raw data.
        :param shape: The shape of the array (e.g., '(64, 64, 3)').
        :param index: An index within [input][camera].
        :param cortical_name: An ID for the cortical area.
        :param grayscale: A boolean indicating if the data is grayscale. True for 3 dimensions, false for 1 dimension.
        :return: A dictionary of difference locations in the FEAGI format.
    """
    # start_time = datetime.now()
    feagi_data = {}
    size_of_frame = shape
    name = 'iv' + cortical_name
    offset_x = (pns.full_list_dimension[name]['cortical_dimensions_per_device'][0] * index)
    if len(significant_changes) > 0:
        if grayscale:
            for x in range(size_of_frame[0]):
                for y in range(size_of_frame[1]):
                    if significant_changes[x, y]:
                        # key = f'{offset_x + y}-{((int(size_of_frame[0]) - 1) - x)}-{0}'
                        key = (offset_x + y, ((int(size_of_frame[0]) - 1) - x), 0)
                        # key = f'{y}-{(int(size_of_frame[0]) - 1) - x}-{0}'
                        feagi_data[key] = int(current[x, y])
        else:
            for x in range(size_of_frame[0]):
                for y in range(size_of_frame[1]):
                    for z in range(size_of_frame[2]):
                        if significant_changes[x, y, z]:
                            # key = f'{offset_x + y}-{((int(size_of_frame[0]) - 1) - x)}-{z}'
                            key = (offset_x + y, ((int(size_of_frame[0]) - 1) - x), z)
                            # key = f'{y}-{((int(size_of_frame[0]) - 1) - x)}-{z}'
                            feagi_data[key] = int(current[x, y, z])
    return feagi_data


def get_difference_from_two_images(previous, current):
    """
    Compare two images and detect which pixel changed using cv2 functions.
    """
    return cv2.absdiff(previous, current)


def vision_blink(image, blink):
    """
    Blink to turn the whole screen bright until next burst
    """
    if len(blink) != 0:
        difference = blink
        image = cv2.threshold(difference, 0, 255, cv2.THRESH_TOZERO_INV)
    return image[1]


def apply_threshold(difference, src=50):
    return cv2.threshold(difference, src, 255, cv2.THRESH_TOZERO)


def change_detector(previous, current, src=50, compare_image=True, cortical_name=""):
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
    if current.shape == previous.shape:
        if compare_image:
            difference = get_difference_from_two_images(previous, current)
        else:
            difference = current
        _, pixel_change_threshold = apply_threshold(difference, src=src)
        return pixel_change_threshold
    else:
        return {}


def generate_vision_ipu_data(cortical_name, pixel_change_threshold, current, previous, feagi_index, percentage=1.0,
                             grayscale=False):
    if drop_high_frequency_events(pixel_change_threshold) <= \
            (get_full_dimension_of_cortical_area(cortical_name) * percentage):
        feagi_data = create_feagi_data(pixel_change_threshold, current, previous.shape, feagi_index,
                                       cortical_name, grayscale=grayscale)
        return dict(feagi_data)
    else:
        return {}


def flip_the_camera(data):
    return cv2.flip(data, 1)


def grab_cortical_resolution(name="", cortical=""):
    return [pns.full_list_dimension[name]['cortical_dimensions_per_device'][0],
            pns.full_list_dimension[name]['cortical_dimensions_per_device'][1],
            current_dimension_list[cortical][2]]


def grab_xy_cortical_resolution(name=""):
    return pns.full_list_dimension[name]['cortical_dimensions_per_device'][0], \
        pns.full_list_dimension[name]['cortical_dimensions_per_device'][1]


def get_full_dimension_of_cortical_area(cortical_name=""):
    global current_dimension_list
    return current_dimension_list[cortical_name][0] * current_dimension_list[cortical_name][1] * \
        current_dimension_list[cortical_name][2]


def process_visual_stimuli(raw_data_from_controller, capabilities, previous_frame_data, rgb, actual_capabilities,
                           compare_image=True):
    """
    This is the most critical function for retina and deserves a doc string

    # todo: add docstring!
    Order of operations:
    1.
    2.
    3.
    4.

    """

    global current_dimension_list, current_mirror_status, preview_flag

    if isinstance(raw_data_from_controller, numpy.ndarray):
        temp_dict = {0: raw_data_from_controller}
        raw_data_from_controller = temp_dict.copy()
    capabilities = pns.create_runtime_default_list(capabilities, actual_capabilities)

    raw_frame = {}
    if pns.resize_list:
        current_dimension_list = pns.resize_list

        all_vision_data_list = {}
        for obtain_raw_data in raw_data_from_controller:
            raw_frame[obtain_raw_data] = []
            if not capabilities['input']['camera'][str(obtain_raw_data)]['disabled']:
                if capabilities['input']['camera'][str(obtain_raw_data)]["mirror"]:
                    raw_frame[obtain_raw_data] = cv2.flip(raw_data_from_controller[obtain_raw_data], 1)
                else:
                    raw_frame[obtain_raw_data] = raw_data_from_controller[obtain_raw_data]
                # Blink accommodation
                if len(capabilities['input']['camera'][str(obtain_raw_data)]['blink']) > 0:
                    raw_frame[obtain_raw_data] = vision_blink(raw_data_from_controller[obtain_raw_data],
                                                              capabilities['input']['camera'][str(obtain_raw_data)][
                                                                  'blink'])
                    capabilities['input']['camera'][str(obtain_raw_data)]['blink'] = []

                # Eccentricity and modulation and size adjustment
                region_coordinates = vision_region_coordinates(
                    frame_width=raw_frame[obtain_raw_data].shape[1],
                    frame_height=raw_frame[obtain_raw_data].shape[0],
                    x1=abs(capabilities['input']['camera'][str(obtain_raw_data)]['eccentricity_control'][
                               'X offset percentage']),
                    x2=abs(capabilities['input']['camera'][str(obtain_raw_data)]['modulation_control'][
                               'X offset percentage']),
                    y1=abs(capabilities['input']['camera'][str(obtain_raw_data)]['eccentricity_control'][
                               'Y offset percentage']),
                    y2=abs(capabilities['input']['camera'][str(obtain_raw_data)]['modulation_control'][
                               'Y offset percentage']),
                    camera_index=capabilities['input']['camera'][str(obtain_raw_data)]['index'],
                    size_list=current_dimension_list)
                if not region_coordinates:
                    if not (capabilities['input']['camera'][str(obtain_raw_data)][
                                'index'] + '_C') in current_dimension_list:
                        pns.resize_list.update(
                            obtain_cortical_vision_size(camera_index=capabilities['input']['camera'][
                                str(obtain_raw_data)]['index'], response=pns.full_list_dimension))
                # Split visual data to segments accounting for central vision and peripheral vision
                segmented_frame_data = split_vision_regions(coordinates=region_coordinates,
                                                            raw_frame_data=raw_frame[obtain_raw_data])

                if len(all_vision_data_list) == 0:
                    for region in segmented_frame_data:
                        all_vision_data_list[region] = []

                compressed_data = dict()
                # Applying lighting enhancements including brightness, contrast, and shadows
                for cortical in segmented_frame_data:
                    name = 'iv' + cortical
                    updated_size = grab_cortical_resolution(name, cortical)
                    compressed_data[cortical] = downsize_regions(frame=segmented_frame_data[cortical],
                                                                 resize=updated_size)
                    if 0 in capabilities['input']['camera'][str(obtain_raw_data)]['enhancement']:
                        compressed_data[cortical] = adjust_brightness(image=compressed_data[cortical],
                                                                      bright=capabilities['input']['camera'][
                                                                          str(obtain_raw_data)]['enhancement'][0])
                    if 1 in capabilities['input']['camera'][str(obtain_raw_data)]['enhancement']:
                        compressed_data[cortical] = adjust_contrast(image=compressed_data[cortical],
                                                                    contrast=capabilities['input']['camera'][
                                                                        str(obtain_raw_data)]['enhancement'][1])
                    if 2 in capabilities['input']['camera'][str(obtain_raw_data)]['enhancement']:
                        compressed_data[cortical] = adjust_shadow(image=compressed_data[cortical],
                                                                  shadow=capabilities['input']['camera'][
                                                                      str(obtain_raw_data)]['enhancement'][2])
                    if len(all_vision_data_list[
                               cortical]) == 0:  # update the newest data into empty all_vision_data_list
                        all_vision_data_list[cortical] = compressed_data[cortical]
                    else:
                        all_vision_data_list[cortical] = numpy.concatenate(
                            (all_vision_data_list[cortical], compressed_data[cortical]), axis=1)
                        if (len(raw_frame) - 1) == obtain_raw_data:  # Reach to end of the list for camera
                            all_vision_data_list[cortical] = cv2.resize(all_vision_data_list[cortical],
                                                                        grab_xy_cortical_resolution(name),
                                                                        interpolation=cv2.INTER_AREA)

        # todo: add a shell frag such as --preview so when that is set the following code runs automatically
        if preview_flag:
            for segment in compressed_data:
                if "_C" in segment or "CC" in segment:
                    cv2.imshow(segment, compressed_data[segment])
            if cv2.waitKey(30) & 0xFF == ord('q'):
                pass

        vision_dict = dict()

        # Generate FEAGI vision IPU data by detecting changes between current and prior frame
        for get_region in all_vision_data_list:
            # In case of color vision
            if current_dimension_list[get_region][2] == 3:
                if previous_frame_data != {}:
                    if get_region in previous_frame_data:
                        modified_data = change_detector(
                            previous=previous_frame_data[get_region],
                            current=all_vision_data_list[get_region],
                            src=capabilities['input']['camera'][str(obtain_raw_data)]['threshold_default'],
                            compare_image=compare_image,
                            cortical_name=get_region)
                        vision_dict[get_region] = generate_vision_ipu_data(
                            percentage=capabilities['input']['camera'][
                                str(obtain_raw_data)]['percentage_to_allow_data'],
                            cortical_name=get_region,
                            pixel_change_threshold=modified_data,
                            current=all_vision_data_list[get_region],
                            previous=previous_frame_data[get_region],
                            feagi_index=capabilities['input']['camera'][str(obtain_raw_data)]['feagi_index'])
                    else:
                        vision_dict[get_region] = change_detector(
                            previous=np.zeros((3, 3, 3)),
                            current=all_vision_data_list[get_region],
                            src=capabilities['input']['camera'][str(obtain_raw_data)]['threshold_default'],
                            compare_image=compare_image,
                            cortical_name=get_region)
            else:
                # In case of grayscale (which is depth 1)
                if previous_frame_data != {}:
                    if get_region in previous_frame_data:
                        modified_data = change_detector(
                            previous=previous_frame_data[get_region],
                            current=all_vision_data_list[get_region],
                            src=capabilities['input']['camera'][str(obtain_raw_data)]['threshold_default'],
                            compare_image=compare_image,
                            cortical_name=get_region)

                        vision_dict[get_region] = generate_vision_ipu_data(
                            percentage=capabilities['input']['camera'][str(obtain_raw_data)][
                                'percentage_to_allow_data'],
                            cortical_name=get_region,
                            pixel_change_threshold=modified_data,
                            current=all_vision_data_list[get_region],
                            previous=previous_frame_data[get_region],
                            feagi_index=capabilities['input']['camera'][str(obtain_raw_data)]['feagi_index'],
                            grayscale=True)
                    else:
                        vision_dict[get_region] = change_detector(
                            previous=np.zeros((3, 3, 3)),
                            current=all_vision_data_list[get_region],
                            src=capabilities,
                            compare_image=compare_image,
                            cortical_name=get_region)

        if previous_frame_data:
            previous_frame_data.update(all_vision_data_list)
        else:
            previous_frame_data = all_vision_data_list

        if 'camera' in rgb:
            rgb['camera'].update(vision_dict)
        else:
            rgb['camera'] = vision_dict

        return previous_frame_data, rgb, capabilities
    return pns.resize_list, pns.resize_list, capabilities  # sending empty dict


def activation_region_break_down(message_from_feagi, obtained_signals):
    obtained_signals['activation_regions'] = []
    if 'ov_reg' in message_from_feagi['opu_data']:
        for data_point in message_from_feagi['opu_data']['ov_reg']:
            obtained_signals['activation_regions'].append(data_point)
    return obtained_signals


def obtain_cortical_vision_size(response, camera_index="00"):
    size_list = {}
    data = response
    items = [camera_index + "_C", camera_index + "BL", camera_index + "BM", camera_index + "BR",
             camera_index + "MR", camera_index + "ML", camera_index + "TR", camera_index + "TL",
             camera_index + "TM", camera_index + "CC"]
    if data is not None:
        for name_from_data in data:
            for fetch_name in items:
                if fetch_name in name_from_data:
                    name = name_from_data.replace("iv", "")
                    dimension_array = data[name_from_data]["cortical_dimensions"][0], \
                        data[name_from_data]["cortical_dimensions"][1], \
                        data[name_from_data]["cortical_dimensions"][2]
                    size_list[name] = dimension_array
    return size_list


def drop_high_frequency_events(data):
    return np.count_nonzero(data)


def process_visual_stimuli_trainer(raw_data_from_controller, capabilities, previous_frame_data, rgb,
                                   actual_capabilities, compare_image=False):
    global current_dimension_list, current_mirror_status, preview_flag
    raw_frame = {}
    if isinstance(raw_data_from_controller, numpy.ndarray):
        temp_dict = {0: raw_data_from_controller}
        raw_data_from_controller = temp_dict.copy()

    capabilities = pns.create_runtime_default_list(capabilities, actual_capabilities)
    if pns.resize_list:
        current_dimension_list = pns.resize_list
        all_vision_data_list = {}
        for obtain_raw_data in raw_data_from_controller:
            raw_frame[obtain_raw_data] = []
            if not capabilities['input']['camera'][str(obtain_raw_data)]['disabled']:
                if capabilities['input']['camera'][str(obtain_raw_data)]["mirror"]:
                    raw_frame[obtain_raw_data] = cv2.flip(raw_data_from_controller[obtain_raw_data], 1)
                else:
                    raw_frame[obtain_raw_data] = raw_data_from_controller[obtain_raw_data]
                region_coordinates = vision_region_coordinates(
                    frame_width=raw_frame[obtain_raw_data].shape[1],
                    frame_height=raw_frame[obtain_raw_data].shape[0],
                    x1=abs(capabilities['input']['camera'][str(obtain_raw_data)]['eccentricity_control'][
                               'X offset percentage']),
                    x2=abs(capabilities['input']['camera'][str(obtain_raw_data)]['modulation_control'][
                               'X offset percentage']),
                    y1=abs(capabilities['input']['camera'][str(obtain_raw_data)]['eccentricity_control'][
                               'Y offset percentage']),
                    y2=abs(capabilities['input']['camera'][str(obtain_raw_data)]['modulation_control'][
                               'Y offset percentage']),
                    camera_index=capabilities['input']['camera'][str(obtain_raw_data)]['index'],
                    size_list=current_dimension_list)

                if not region_coordinates:
                    if not (capabilities['input']['camera'][str(obtain_raw_data)][
                                'index'] + '_C') in current_dimension_list:
                        pns.resize_list.update(
                            obtain_cortical_vision_size(
                                camera_index=capabilities['input']['camera'][str(obtain_raw_data)]['index'],
                                response=pns.full_list_dimension))
                segmented_frame_data = split_vision_regions(coordinates=region_coordinates,
                                                            raw_frame_data=raw_frame[
                                                                obtain_raw_data])

                if len(all_vision_data_list) == 0:
                    for region in segmented_frame_data:
                        all_vision_data_list[region] = []
                compressed_data = dict()

                for cortical in segmented_frame_data:
                    name = 'iv' + cortical
                    updated_size = grab_cortical_resolution(name=name, cortical=cortical)
                    compressed_data[cortical] = downsize_regions(frame=segmented_frame_data[cortical],
                                                                 resize=updated_size)
                    if 0 in capabilities['input']['camera'][str(obtain_raw_data)]['enhancement']:
                        compressed_data[cortical] = adjust_brightness(image=compressed_data[cortical],
                                                                      bright=capabilities['input']['camera'][
                                                                          str(obtain_raw_data)]['enhancement'][0])
                    if 1 in capabilities['input']['camera'][str(obtain_raw_data)]['enhancement']:
                        compressed_data[cortical] = adjust_contrast(image=compressed_data[cortical],
                                                                    contrast=capabilities['input']['camera'][
                                                                        str(obtain_raw_data)]['enhancement'][1])
                    if 2 in capabilities['input']['camera'][str(obtain_raw_data)]['enhancement']:
                        compressed_data[cortical] = adjust_shadow(image=compressed_data[cortical],
                                                                  shadow=capabilities['input']['camera'][
                                                                      str(obtain_raw_data)]['enhancement'][2])
                    if len(all_vision_data_list[cortical]) == 0:
                        # update the newest data into empty all_vision_data_list
                        all_vision_data_list[cortical] = compressed_data[cortical]
                    else:
                        all_vision_data_list[cortical] = numpy.concatenate(
                            (all_vision_data_list[cortical], compressed_data[cortical]), axis=1)
                        if (len(raw_frame) - 1) == obtain_raw_data:  # Reach to end of the list for camera
                            all_vision_data_list[cortical] = cv2.resize(all_vision_data_list[cortical],
                                                                        grab_xy_cortical_resolution(name),
                                                                        interpolation=cv2.INTER_AREA)

        vision_dict = dict()
        if preview_flag:
            for segment in compressed_data:
                if "_C" in segment:
                    cv2.imshow(segment, compressed_data[segment])
            if cv2.waitKey(30) & 0xFF == ord('q'):
                pass
        modified_data_dict = {}
        for get_region in all_vision_data_list:
            if current_dimension_list[get_region][2] == 3:
                if previous_frame_data != {}:
                    if get_region in previous_frame_data:
                        modified_data = change_detector(
                            previous=previous_frame_data[get_region],
                            current=all_vision_data_list[get_region],
                            src=capabilities['input']['camera'][str(obtain_raw_data)]['threshold_default'],
                            compare_image=compare_image, cortical_name=get_region)

                        vision_dict[get_region] = generate_vision_ipu_data(
                            percentage=capabilities['input']['camera'][str(obtain_raw_data)][
                                'percentage_to_allow_data'],
                            cortical_name=get_region,
                            pixel_change_threshold=modified_data,
                            current=all_vision_data_list[get_region],
                            previous=previous_frame_data[get_region],
                            feagi_index=capabilities['input']['camera'][str(obtain_raw_data)]['feagi_index'])

                        modified_data_dict[get_region] = modified_data
                    else:
                        vision_dict[get_region] = change_detector(
                            previous=np.zeros((3, 3, 3)),
                            current=all_vision_data_list[get_region],
                            src=capabilities['input']['camera'][str(obtain_raw_data)]['threshold_default'],
                            compare_image=compare_image,
                            cortical_name=get_region)
            else:
                if previous_frame_data != {}:
                    if get_region in previous_frame_data:
                        modified_data = change_detector(
                            previous=previous_frame_data[get_region],
                            current=all_vision_data_list[get_region],
                            src=capabilities['input']['camera'][str(obtain_raw_data)]['threshold_default'],
                            compare_image=compare_image,
                            cortical_name=get_region)

                        vision_dict[get_region] = generate_vision_ipu_data(
                            percentage=capabilities['input']['camera'][str(obtain_raw_data)][
                                'percentage_to_allow_data'],
                            cortical_name=get_region,
                            pixel_change_threshold=modified_data,
                            current=all_vision_data_list[get_region],
                            previous=previous_frame_data[get_region],
                            feagi_index=capabilities['input']['camera'][str(obtain_raw_data)]['feagi_index'],
                            grayscale=True)
                        modified_data_dict[get_region] = modified_data
                    else:
                        vision_dict[get_region] = change_detector(
                            previous=np.zeros((3, 3, 3)),
                            current=all_vision_data_list[get_region],
                            src=capabilities['input']['camera'][str(obtain_raw_data)]['threshold_default'],
                            compare_image=compare_image,
                            cortical_name=get_region)
        if previous_frame_data:
            previous_frame_data.update(all_vision_data_list)
        else:
            previous_frame_data = all_vision_data_list
        if 'camera' in rgb:
            rgb['camera'].update(vision_dict)
        else:
            rgb['camera'] = vision_dict
        return previous_frame_data, rgb, capabilities, modified_data_dict
    return pns.resize_list, pns.resize_list, capabilities, {}  # sending empty dict


def vision_progress(capabilities, feagi_settings, raw_frame):
    global genome_tracker, previous_genome_timestamp
    burst_counter = {}
    while True:
        message_from_feagi = pns.message_from_feagi
        opu_data_message_only = pns.obtain_opu_data(message_from_feagi)
        if message_from_feagi is not None and message_from_feagi and \
                message_from_feagi['burst_counter'] != burst_counter:
            burst_counter = message_from_feagi['burst_counter']
            capabilities = fetch_vision_turner(opu_data_message_only, capabilities)
            capabilities = fetch_enhancement_data(opu_data_message_only, capabilities)
            # capabilities = pns.fetch_threshold_type(opu_data_message_only, capabilities) # TODO: revisit this
            capabilities = fetch_mirror_opu(opu_data_message_only, capabilities)
            # Update resize if genome has been changed:
            pns.check_genome_status(message_from_feagi, capabilities)
            if isinstance(raw_frame, dict):
                if 'vision' in raw_frame:
                    capabilities = obtain_blink_data(raw_frame['vision'], message_from_feagi,
                                                     capabilities)  # for javascript webcam
            capabilities = eccentricity_control_update(opu_data_message_only, capabilities)
            capabilities = modulation_control_update(opu_data_message_only, capabilities)
            feagi_settings['feagi_burst_speed'] = pns.check_refresh_rate(message_from_feagi,
                                                                         feagi_settings['feagi_burst_speed'])
        sleep(feagi_settings['feagi_burst_speed'])


def eccentricity_control_update(message_from_feagi, capabilities):
    """
  Update camera eccentricity control settings based on FEAGI message.

  Args:
      message_from_feagi (dict): Message containing eccentricity control settings
      capabilities (dict): System capabilities configuration

  Returns:
      dict: Updated capabilities with new eccentricity values
  """
    if pns.full_list_dimension:
        default_ranges = {
            0: {'min': 1, 'max': 99},  # X offset range
            1: {'min': 1, 'max': 99}  # Y offset range
        }

        if 'eccentricity_control' in message_from_feagi:
            if 'camera' in capabilities.get('input', {}):
                for camera_index in capabilities['input']['camera']:
                    for eccentricity_index, eccentricity_value in message_from_feagi['eccentricity_control'].items():
                        eccentricity_index = int(eccentricity_index)
                        if eccentricity_index not in default_ranges:
                            continue

                        ranges = default_ranges[eccentricity_index]
                        scaled_value = int((eccentricity_value * (ranges['max'] - ranges['min'])) + ranges['min'])

                        if eccentricity_index == 0:
                            capabilities['input']['camera'][camera_index]["eccentricity_control"][
                                "X offset percentage"] = scaled_value
                        elif eccentricity_index == 1:
                            capabilities['input']['camera'][camera_index]["eccentricity_control"][
                                "Y offset percentage"] = scaled_value

    return capabilities


def modulation_control_update(message_from_feagi, capabilities):
    """
  Update camera modulation control settings based on FEAGI message.

  Args:
      message_from_feagi (dict): Message containing modulation control settings
      capabilities (dict): System capabilities configuration

  Returns:
      dict: Updated capabilities with new modulation values
  """
    if pns.full_list_dimension:
        default_ranges = {
            0: {'min': 1, 'max': 99},  # X offset range
            1: {'min': 1, 'max': 99}  # Y offset range
        }
        if 'modulation_control' in message_from_feagi:
            if 'camera' in capabilities.get('input', {}):
                for camera_index in capabilities['input']['camera']:
                    for modulation_index, modulation_value in message_from_feagi['modulation_control'].items():
                        modulation_index = int(modulation_index)
                        if modulation_index not in default_ranges:
                            continue
                        ranges = default_ranges[modulation_index]
                        scaled_value = int((modulation_value * (ranges['max'] - ranges['min'])) + ranges['min'])

                        if modulation_index == 0:
                            capabilities['input']['camera'][camera_index]["modulation_control"][
                                "X offset percentage"] = scaled_value
                        elif modulation_index == 1:
                            capabilities['input']['camera'][camera_index]["modulation_control"][
                                "Y offset percentage"] = scaled_value

    return capabilities


def obtain_blink_data(raw_frame, message_from_feagi, capabilities):
    """
  It will update based on the blink opu.
  """
    if isinstance(raw_frame, dict):
        if "o_blnk" in message_from_feagi["opu_data"]:
            if message_from_feagi["opu_data"]["o_blnk"]:
                if 'camera' in capabilities['input']:
                    for index in capabilities['input']['camera']:
                        if raw_frame[index].any():
                            capabilities['input']['camera'][index]['blink'] = raw_frame[index]
    return capabilities


def fetch_mirror_opu(message_from_feagi, capabilities):
    if 'horizontal_flip' in message_from_feagi:
        if 'camera' in capabilities['input']:
            for index in capabilities['input']['camera']:
                if capabilities['input']['camera'][index]["mirror"]:
                    capabilities['input']['camera'][index]["mirror"] = False
                else:
                    capabilities['input']['camera'][index]["mirror"] = True
    return capabilities


def fetch_vision_turner(message_from_feagi, capabilities):
    """
  Updates vision threshold settings for camera inputs based on FEAGI messages.

  This function processes two threshold parameters that control edge detection sensitivity
  and data transmission rates:

  Parameters:
      message_from_feagi (dict): Message containing threshold settings where:
          - threshold[0] (float): Edge detection sensitivity multiplier (0-1)
              - Will be multiplied by 255 to get final threshold
              - Higher values = less sensitive
              - Lower values = more sensitive
          - threshold[1] (float): Data transmission filter (0-1)
              - Controls how much visual data passes to FEAGI
              - 1.0 = allow all data
              - 0.0 = minimal data transmission

      capabilities (dict): System capabilities configuration containing camera settings

  Returns:
      dict: Updated capabilities dictionary with new threshold values

  Example:
      message = {
          'threshold': {
              0: 0.2,  # Will set threshold_default to 51 (0.2 * 255)
              1: 0.5   # Will allow 50% of data through
          }
      }
      capabilities = fetch_vision_turner(message, capabilities)
  """
    if pns.full_list_dimension:
        if 'threshold' in message_from_feagi:
            if message_from_feagi["threshold"]:
                if 'camera' in capabilities['input']:
                    for index in capabilities['input']['camera']:
                        for data_point in message_from_feagi["threshold"]:
                            if data_point == 0:
                                capabilities['input']['camera'][index]["threshold_default"] = 255 * message_from_feagi[
                                    "threshold"][
                                    data_point]
                            if data_point == 1:
                                capabilities['input']['camera'][index]["percentage_to_allow_data"] = \
                                    message_from_feagi["threshold"][data_point]
    return capabilities


def fetch_enhancement_data(message_from_feagi, capabilities):
    if pns.full_list_dimension:

        default_ranges = {
            0: {'min': -100, 'max': 100},
            1: {'min': 0.0, 'max': 2.0},
            2: {'min': 0.8, 'max': 2.0}
        }
        if "enhancement" in message_from_feagi:
            if message_from_feagi["enhancement"]:
                if capabilities:
                    if 'camera' in capabilities['input']:
                        for index in message_from_feagi["enhancement"]:
                            # Index 0: brightness
                            # Index 1: contrast
                            # Index 2: shadow
                            enhancement_value = message_from_feagi["enhancement"][index]
                            enhancement_id = int(index)
                            if enhancement_id not in default_ranges:
                                continue
                            ranges = default_ranges[enhancement_id]
                            calculated_enhancement_value = float((enhancement_value *
                                                                  (ranges['max'] - ranges['min'])) + ranges['min'])

                            for camera_index in capabilities['input']['camera']:
                                capabilities['input']['camera'][camera_index]["enhancement"][enhancement_id] = \
                                    calculated_enhancement_value

    return capabilities


def update_astype(data):
    return data.astype(np.uint8)


def RGB_list_to_ndarray(data, size):
    new_rgb = np.array(data)
    new_rgb = new_rgb.reshape(size[1], size[0], 3)
    return new_rgb


def flip_video(data):
    return cv2.flip(data, 1)


def check_brightness(frame):
    # Calculate the average pixel intensity (brightness)
    average_intensity = cv2.mean(frame)[0]

    # Define thresholds for brightness
    brightness_threshold = 127  # Adjust this threshold as needed

    # Check if the average intensity is above or below the threshold
    if average_intensity > brightness_threshold:
        return "Image is too bright"
    elif average_intensity < brightness_threshold:
        return "Image is too dark"
    else:
        return "Image is neither too bright nor too dark"


# def threshold_detect(capabilities):
#     threshold_type = [cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV, cv2.THRESH_TRUNC, cv2.THRESH_TOZERO,
#                       cv2.THRESH_TOZERO_INV, cv2.THRESH_OTSU]
#     threshold_total = cv2.THRESH_BINARY
#     if capabilities['input']['camera'][str(obtain_raw_data)]['threshold_type']:
#         for threshold_selected in range(len(capabilities['input']['camera'][str(obtain_raw_data)]['threshold_type'])):
#             threshold_total = threshold_type[threshold_selected]
#     capabilities['input']['camera'][str(obtain_raw_data)]['threshold_type'].clear()
#     return threshold_total


def adjust_brightness(image, bright=None):
    if bright:
        image = cv2.convertScaleAbs(image, beta=bright)
    return image


def adjust_contrast(image, contrast=None):
    if contrast:
        image = cv2.convertScaleAbs(image, alpha=contrast)
    return image


def adjust_shadow(image, shadow=None):
    if shadow:
        max_intensity = 255.0
        phi = 1
        theta = 1
        adjusted = (max_intensity / phi) * (image / (max_intensity / theta)) ** shadow
        image = np.array(adjusted, dtype=np.uint8)
    return image


def grab_visual_cortex_dimension(capabilities):
    cortical_area_exist_list = []
    index = []
    a_cortical_dict = {}
    if pns.full_list_dimension:
        if 'camera' in capabilities['input']:
            for camera_index in capabilities['input']['camera']:
                index.append(capabilities['input']['camera'][camera_index]['index'])
            for name in ['_C', 'BL', 'BR', 'TL']:
                for nested_index in index:
                    cortical_area_exist_list.append('iv' + nested_index + name)

            for name in cortical_area_exist_list:
                if name in pns.full_list_dimension:
                    a_cortical_dict[name] = pns.full_list_dimension[name]['cortical_dimensions_per_device']
                else:
                    a_cortical_dict[name] = (0, 0, 0)
        # Todo: Needs to address this in near future
        x = a_cortical_dict['iv00_C'][0] + a_cortical_dict['iv00BL'][0] + a_cortical_dict['iv00BR'][0]
        y = a_cortical_dict['iv00_C'][1] + a_cortical_dict['iv00BL'][1] + a_cortical_dict['iv00TL'][1]
        z = a_cortical_dict['iv00_C'][2]
        return [x, y, z]
    return [0, 0, 0]  # Send empty
