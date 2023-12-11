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

from feagi_agent import feagi_interface as feagi
from feagi_agent import router
import traceback

# Variable storage #
raw_aptr = -1
global_aptr_cortical_size = None
global_ID_cortical_size = None
full_list_dimension = []


def generate_feagi_data(rgb, msg_counter, date, message_to_feagi):
    """
    This function generates data for Feagi by combining RGB values, message counter, and date into
    the provided message.
    """
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


def append_sensory_data_for_feagi(sensory_category, sensory_data, message_to_feagi):
    """
    :param sensory_category: A name such as training, camera, IR, ultrasonic and so on.
    :param sensory_data: The data of dict only
    :param message_to_feagi: Use the existing dict to append
    :return: the updated dict called `message_to_feagi`
    """
    if "data" not in message_to_feagi:
        message_to_feagi["data"] = {}
    if "sensory_data" not in message_to_feagi["data"]:
        message_to_feagi["data"]["sensory_data"] = {}
    message_to_feagi["data"]["sensory_data"][sensory_category] = sensory_data
    return message_to_feagi


def signals_from_feagi(feagi_opu_channel):
    """ get OPU from FEAGI """
    return router.fetch_feagi(feagi_opu_channel)


def signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings):
    router.send_feagi(message_to_feagi, feagi_ipu_channel, agent_settings)


def fetch_aperture_data(message_from_feagi, capabilities, aptr_cortical_size):
    """
    This determines the WebSocket transmission capacity. A lower value allows more WebSocket data
    to pass through, whereas a higher value restricts the amount of WebSocket data that can
    be transmitted.
    """
    if "o_aptr" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o_aptr"]:
            for i in message_from_feagi["opu_data"]["o_aptr"]:
                feagi_aptr = (int(i.split('-')[-1]))
                aptr_cortical_size = fetch_aptr_size(global_aptr_cortical_size, aptr_cortical_size,
                                                     feagi_aptr)
                max_range = capabilities['camera']['aperture_range'][1]
                min_range = capabilities['camera']['aperture_range'][0]
                capabilities['camera']["aperture_default"] = \
                    ((feagi_aptr / global_aptr_cortical_size) *
                     (max_range - min_range)) + min_range
    return capabilities


def fetch_iso_data(message_from_feagi, capabilities, aptr_cortical_size):
    """
       The higher the threshold, the lower its sensitivity. Conversely, the lower the threshold,
       the higher the sensitivity. In essence, a lower threshold makes the camera more sensitive to
       pick things up.
    """
    if "o__dev" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o__dev"]:
            for i in message_from_feagi["opu_data"]["o__dev"]:
                device_id = i.split('-')
                feagi_aptr = (int(i.split('-')[-1]))
                aptr_cortical_size = fetch_aptr_size(global_aptr_cortical_size,
                                                     global_aptr_cortical_size,
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
            if feagi_aptr >= global_aptr_cortical_size:
                return global_aptr_cortical_size
        aptr_cortical_size = check_aptr(get_size_for_aptr_cortical)
        return aptr_cortical_size
    else:
        return aptr_cortical_size


def check_aptr(get_size_for_aptr_cortical):
    return router.fetch_aptr(get_size_for_aptr_cortical)


def check_ID_size(get_size_for_ID):
    return router.fetch_ID(get_size_for_ID)


def grab_geometry():
    return router.fetch_geometry()


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


def obtain_blink_data(raw_frame, message_from_feagi, capabilities):
    if "o_blnk" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o_blnk"]:
            capabilities['camera']['blink'] = raw_frame
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


def detect_ID_data(message_from_feagi):
    """
    :param message_from_feagi: Should be a dict from FEAGI data only
    :return: Return the data that given by FEAGI
    """
    if "o___ID" in message_from_feagi["opu_data"]:
        if message_from_feagi["opu_data"]["o___ID"]:
            return message_from_feagi["opu_data"]["o___ID"]
    return {}


def detect_genome_change(message_from_feagi):
    if "genome_changed" in message_from_feagi:
        if message_from_feagi["genome_changed"]:
            return message_from_feagi["genome_changed"]


def check_refresh_rate(message_from_feagi, current_second):
    if message_from_feagi is not None:
        return message_from_feagi['burst_frequency']
    return current_second


def fetch_full_dimensions(url):
    return router.fetch_all_size(url)
