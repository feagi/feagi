#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

from src.inf import runtime_data
from src.evo.synaptogenesis_rules import neighbor_finder


def collect_vision_configuration():
    central_vision_dimension = get_central_vision_dimension()
    peripheral_vision_dimension = get_peripheral_vision_dimension()

    color_vision = None
    if central_vision_dimension:
        if central_vision_dimension[2] == 1:
            color_vision = False
        elif central_vision_dimension[2] == 3:
            color_vision = True

    else:
        central_vision_dimension = (None, None)

    if not peripheral_vision_dimension:
        peripheral_vision_dimension = (None, None)

    brightness, contrast, shadows = get_lighting_enhancement_values()
    pixel_change_limit = get_lighting_threshold_values()

    vision_configuration = {
        "central_vision_resolution": (central_vision_dimension[0], central_vision_dimension[1]),
        "peripheral_vision_resolution": (peripheral_vision_dimension[0], peripheral_vision_dimension[1]),
        "flicker_period": get_vision_flicker_period(),
        "color_vision": color_vision,
        "eccentricity": get_eccentricity_values(),
        "modulation": get_modulation_values(),
        "brightness": brightness,
        "contrast": contrast,
        "shadows": shadows,
        "pixel_change_limit": pixel_change_limit,
    }

    return vision_configuration


def reconfigure_vision(vision_parameters):
    pass


def get_central_vision_dimension():
    central_vision_cortical_area = "iv00_C"
    area_properties = runtime_data.genome["blueprint"].get(central_vision_cortical_area)
    if area_properties:
        cortical_dimension = runtime_data.genome["blueprint"][central_vision_cortical_area].get("block_boundaries")
        return cortical_dimension
    else:
        return


def power_is_connected(cortical_area):
    power_area = "___pwr"
    neighbor_candidates = None
    connected_power_coordinates = None
    src_subregion = (
        (0, 0, 0),
        tuple(runtime_data.genome["blueprint"][power_area]["block_boundaries"])
    )
    if cortical_area in runtime_data.genome["blueprint"]:
        power_mappings = runtime_data.genome["blueprint"][power_area].get("cortical_mapping_dst")
        if cortical_area in power_mappings:
            for morphology in runtime_data.genome["blueprint"][power_area]["cortical_mapping_dst"][cortical_area]:
                for src_id in runtime_data.brain[power_area]:
                    neighbor_candidates = neighbor_finder(cortical_area_src=power_area,
                                                          cortical_area_dst=cortical_area,
                                                          src_neuron_id=src_id,
                                                          morphology_=morphology,
                                                          src_subregion=src_subregion)

    if neighbor_candidates:
        connected_power_coordinates = set()
        for candidate in neighbor_candidates:
            candidate_id = candidate[0]
            candidate_psp = candidate[1]
            candidate_coordinate = runtime_data.brain[cortical_area][candidate_id]["soma_location"]
            connected_power_coordinates.add((candidate_coordinate, candidate_psp))

    return connected_power_coordinates


def get_peripheral_vision_dimension():
    central_vision_cortical_areas = ["iv00TR", "iv00TL", "iv00TM", "iv00MR", "iv00ML", "iv00BR", "iv00BL", "iv00BM"]
    cortical_dimension = set()
    for area in central_vision_cortical_areas:
        area_properties = runtime_data.genome["blueprint"].get(area)
        if area_properties:
            dimension = area_properties.get("block_boundaries")
            cortical_dimension.add(tuple(dimension))
    if len(cortical_dimension) == 1:
        return next(iter(cortical_dimension))
    else:
        return


def get_vision_flicker_period():
    flicker_area = "o_blnk"
    flicker_period = None

    power_connectivity = power_is_connected(cortical_area=flicker_area)

    if power_connectivity:
        flicker_period = runtime_data.genome["blueprint"][flicker_area].get("refractory_period")

    return flicker_period


def get_eccentricity_values():
    eccentricity_area = "ov_ecc"
    eccentricity_x = None
    eccentricity_y = None

    power_connectivity = power_is_connected(cortical_area=eccentricity_area)

    if power_connectivity:
        for connection_entry in power_connectivity:
            neuron_coordinate = connection_entry[0]
            if neuron_coordinate[0] == 0 and neuron_coordinate[1] == 0:
                eccentricity_x = neuron_coordinate[2]
            elif neuron_coordinate[0] == 1 and neuron_coordinate[1] == 0:
                eccentricity_y = neuron_coordinate[2]

    return eccentricity_x, eccentricity_y


def get_modulation_values():
    modulation_area = "ov_mod"
    modulation_x = None
    modulation_y = None

    power_connectivity = power_is_connected(cortical_area=modulation_area)

    if power_connectivity:
        for connection_entry in power_connectivity:
            neuron_coordinate = connection_entry[0]
            if neuron_coordinate[0] == 0 and neuron_coordinate[1] == 0:
                modulation_x = neuron_coordinate[2]
            elif neuron_coordinate[0] == 1 and neuron_coordinate[1] == 0:
                modulation_y = neuron_coordinate[2]

    return modulation_x, modulation_y


def get_lighting_enhancement_values():
    lighting_enhancement_area = "ov_enh"
    brightness = None
    contrast = None
    shadows = None

    power_connectivity = power_is_connected(cortical_area=lighting_enhancement_area)

    if power_connectivity:
        for connection_entry in power_connectivity:
            neuron_coordinate = connection_entry[0]
            if neuron_coordinate[0] == 0 and neuron_coordinate[1] == 0:
                brightness = neuron_coordinate[2]
            elif neuron_coordinate[0] == 1 and neuron_coordinate[1] == 0:
                contrast = neuron_coordinate[2]
            elif neuron_coordinate[0] == 2 and neuron_coordinate[1] == 0:
                shadows = neuron_coordinate[2]

    return brightness, contrast, shadows


def get_lighting_threshold_values():
    lighting_threshold_area = "ovtune"
    pixel_change_limit = None

    power_connectivity = power_is_connected(cortical_area=lighting_threshold_area)

    if power_connectivity:
        for connection_entry in power_connectivity:
            neuron_coordinate = connection_entry[0]
            if neuron_coordinate[0] == 0 and neuron_coordinate[1] == 0:
                pixel_change_limit = neuron_coordinate[2]

    return pixel_change_limit
