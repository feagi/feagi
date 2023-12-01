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

import pickle
import lz4.frame
import requests
import traceback
from feagi_agent import feagi_interface as feagi
from feagi_agent import retina as retina


def generate_rgb(frame, width_percentage, height_percentage, central_resolution,
                 peripheral_resolution, previous_data_frame, current_selected_size,
                 current_iso_selected, aperture_default, camera_index):
    """"
        frame (ndarray): RGB data.
        previous_data_frame (dict): Previous data containing old RGB values stored in the
        controller.
        retina_data (dict): Latest RGB data.
        current_selected_size (array): It is capabilities['camera']['current_select'] in the config.
        central_resolution (array): Capabilities['camera']["central_vision_resolution"].
        peripheral_resolution (array): Capabilities['camera']['peripheral_vision_resolution'].
        current_iso_selected (float): Capabilities['camera']['iso_threshold'].
        aperture_default (float): Capabilities['camera']["aperture_default"].
    """
    retina.detect_change_edge(raw_frame, capabilities, camera_index, resize_list,
                              previous_frame_data)
    retina_data = retina.frame_compression(retina_data,
                                           central_resolution, peripheral_resolution)
    previous_data_frame = retina.check_previous_data(previous_data_frame, retina_data)
    previous_data_frame, camera = \
        retina.detect_change_edge(frame, previous_data_frame,
                                  retina_data, current_selected_size, central_resolution,
                                  peripheral_resolution, current_iso_selected,
                                  aperture_default, single_RGB, snap)
    return previous_data_frame, camera


def generate_feagi_data(rgb, msg_counter, date, message_to_feagi):
    try:
        if "data" not in message_to_feagi:
            message_to_feagi["data"] = dict()
        if "sensory_data" not in message_to_feagi["data"]:
            message_to_feagi["data"]["sensory_data"] = dict()
        message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
    except Exception as e:
        print("ERROR: ", e)
        traceback.print_exc()
    message_to_feagi['timestamp'] = date
    message_to_feagi['counter'] = msg_counter
    return message_to_feagi


def efferent_signaling(feagi_opu_channel):
    """
    Obtain the data from feagi's OPU
    """
    received_data = feagi_opu_channel.receive()  # Obtain data from FEAGI
    # Verify if the data is not None
    if received_data is not None:
        # Verify if the data is compressed
        if isinstance(received_data, bytes):
            # Decompress
            decompressed_data = lz4.frame.decompress(received_data)
            # Another decompress of json
            message_from_feagi = pickle.loads(decompressed_data)
            return message_from_feagi
        else:
            # Directly obtain without any compressions
            message_from_feagi = received_data
            return message_from_feagi
    else:
        # It's None so no action will taken once it returns the None
        message_from_feagi = None
        return message_from_feagi


def afferent_signaling(message_to_feagi, feagi_ipu_channel, agent_settings):
    if agent_settings['compression']:
        serialized_data = pickle.dumps(message_to_feagi)
        feagi_ipu_channel.send(message=lz4.frame.compress(serialized_data))
    else:
        feagi_ipu_channel.send(message_to_feagi)


def fetch_aperture_data(message_from_feagi, capabilities, aptr_cortical_size):
    if "o_aptr" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o_aptr"]:
            for i in message_from_feagi["opu_data"]["o_aptr"]:
                feagi_aptr = (int(i.split('-')[-1]))
                aptr_cortical_size = fetch_aptr_size(aptr_cortical_size, aptr_cortical_size,
                                                     feagi_aptr)
                max_range = capabilities['camera']['aperture_range'][1]
                min_range = capabilities['camera']['aperture_range'][0]
                capabilities['camera']["aperture_default"] = \
                    ((feagi_aptr / aptr_cortical_size) *
                     (max_range - min_range)) + min_range
    return capabilities


def fetch_iso_data(message_from_feagi, capabilities, aptr_cortical_size):
    if "o__dev" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o__dev"]:
            for i in message_from_feagi["opu_data"]["o__dev"]:
                device_id = i.split('-')
                feagi_aptr = (int(i.split('-')[-1]))
                aptr_cortical_size = fetch_aptr_size(aptr_cortical_size, aptr_cortical_size,
                                                     feagi_aptr)
                max_range = capabilities['camera']['iso_range'][1]
                min_range = capabilities['camera']['iso_range'][0]
                capabilities['camera']["iso_default"][int(device_id[0])] = \
                    int(((feagi_aptr / aptr_cortical_size) * (max_range - min_range)) + min_range)
            print(capabilities['camera']["iso_default"])
    return capabilities


def fetch_resolution_selected(message_from_feagi, capabilities):
    if "o_vres" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o_vres"]:
            for i in message_from_feagi["opu_data"]["o_vres"]:
                dev_data = feagi.block_to_array(i)  # TODO: remove use feagi interface
                if dev_data[0] == 0:
                    capabilities['camera']['current_select'][0] = \
                        capabilities['camera']['resolution_presets'][dev_data[2]]
                if dev_data[0] == 1:
                    capabilities['camera']['current_select'][1] = \
                        capabilities['camera']['resolution_presets'][dev_data[2]]
    return capabilities


def fetch_resolution_peripherals_selected(message_from_feagi, capabilities):
    if "o_pres" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o_pres"]:
            for i in message_from_feagi["opu_data"]["o_pres"]:
                dev_data = feagi.block_to_array(i)  # TODO: remove use feagi interface
                if dev_data[0] == 0:
                    capabilities['camera']['current_select_peripheral'] = \
                        capabilities['camera']['resolution_presets'][dev_data[2]]
    return capabilities


def fetch_vision_acuity(message_from_feagi, capabilities):
    if "o_vact" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o_vact"]:
            for i in message_from_feagi["opu_data"]["o_vact"]:
                dev_data = feagi.block_to_array(i)
                if dev_data[0] == 0:
                    capabilities['camera']['central_vision_allocation_percentage'][0] \
                        = \
                        message_from_feagi["opu_data"]["o_vact"][i]
                if dev_data[0] == 1:
                    capabilities['camera']['central_vision_allocation_percentage'][1] \
                        = \
                        message_from_feagi["opu_data"]["o_vact"][i]
    return capabilities


def fetch_aptr_size(aptr_cortical_size, get_size_for_aptr_cortical, feagi_aptr=None):
    if aptr_cortical_size is None:
        if feagi_aptr is not None:
            if feagi_aptr >= aptr_cortical_size:
                return aptr_cortical_size
        aptr_cortical_size = check_aptr(get_size_for_aptr_cortical)
        return aptr_cortical_size
    else:
        return aptr_cortical_size


def check_aptr(get_size_for_aptr_cortical):
    try:
        raw_aptr = requests.get(get_size_for_aptr_cortical).json()
        return raw_aptr['cortical_dimensions'][2]
    except Exception as error:
        print("error: ", error)
        return 10


def generate_OPU_list(capabilities):
    sensor_list = []
    for i in capabilities:
        if "type" in capabilities[i]:
            if "opu" in capabilities[i]["type"]:
                sensor_list.append(i)
    return sensor_list


def obtain_opu_data(device_list, message_from_feagi):
    opu_signal_dict = {}
    opu_data = feagi.opu_processor(message_from_feagi)
    for i in device_list:
        if i in opu_data and opu_data[i]:
            for x in opu_data[i]:
                if i not in opu_signal_dict:
                    opu_signal_dict[i] = {}
                opu_signal_dict[i][x] = opu_data[i][x]
    return opu_signal_dict


def obtain_data_type(data):
    if type(data).__name__ == "ImagingCore":
        print("ImagingCore")
        return "ImagingCore"
    elif type(data).__name__ == "ndarray":
        print("numpy.ndarray")
        return "ndarray"
    elif type(data).__name__ == "list":
        print("list")
        return "list"
    else:
        print("Couldn't find: ", type(data).__name__, " and full name of the class: ", type(data))
        return "Unknown"


def obtain_snap_data(raw_frame, message_from_feagi, capabilities):
    if "o_snap" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o_snap"]:
            capabilities['camera']['snap'] = raw_frame
    return capabilities


def obtain_genome_number(genome_tracker, message_from_feagi):
    if 'genome_num' in message_from_feagi:
        if message_from_feagi['genome_num'] != genome_tracker:
            return message_from_feagi['genome_num']
    return genome_tracker


def monitor_switch(message_from_feagi, capabilities):
    if "o__mon" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o__mon"]:
            for i in message_from_feagi["opu_data"]["o__mon"]:
                monitor_update = feagi.block_to_array(i)
                capabilities['camera']['monitor'] = monitor_update[0]
    return capabilities


def gaze_control_update(message_from_feagi, capabilities):
    if 'o__gaz' in message_from_feagi["opu_data"]:
        for data_point in message_from_feagi["opu_data"]['o__gaz']:
            processed_data_point = feagi.block_to_array(data_point)
            device_id = processed_data_point[0]
            device_power = message_from_feagi["opu_data"]['o__gaz'][data_point]
            if device_power == 100:
                device_power -= 1
            capabilities['camera']['gaze_control'][device_id] = device_power
    return capabilities


def pupil_control_update(message_from_feagi, capabilities):
    if 'o__pup' in message_from_feagi["opu_data"]:
        for data_point in message_from_feagi["opu_data"]['o__pup']:
            processed_data_point = feagi.block_to_array(data_point)
            device_id = processed_data_point[0]
            device_power = message_from_feagi["opu_data"]['o__pup'][data_point]
            if device_power == 100:
                device_power -= 1
            capabilities['camera']['pupil_control'][device_id] = device_power
    return capabilities


def detect_genome_change(message_from_feagi):
    if "genome_changed" in message_from_feagi:
        if message_from_feagi["genome_changed"]:
            return message_from_feagi["genome_changed"]


def check_refresh_rate(message_from_feagi, current_second):
    if message_from_feagi is not None:
        return message_from_feagi['burst_frequency']
    return current_second
