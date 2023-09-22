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

from feagi_agent import retina as retina


def generate_rgb(frame, width_percentage, height_percentage, central_resolution,
                 peripheral_resolution, previous_data_frame, current_selected_size,
                 current_iso_selected, aperture_default):
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
    retina_data = retina.frame_split(frame, width_percentage, height_percentage)
    retina_data = retina.frame_compression(retina_data,
                                           central_resolution, peripheral_resolution)
    previous_data_frame = retina.check_previous_data(previous_data_frame, retina_data)
    previous_data_frame, camera, selected = \
        retina.detect_change_edge(frame, previous_data_frame,
                                  retina_data, current_selected_size, central_resolution,
                                  peripheral_resolution, current_iso_selected,
                                  aperture_default)
    return previous_data_frame, camera, selected
