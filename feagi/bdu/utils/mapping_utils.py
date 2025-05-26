"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Mapping utilities for connectome cortical area mappings.
"""
from typing import Dict, Any
from math import floor
from random import randrange
from feagi.core.state_manager import FeagiStateManager
from feagi.evo.templates import cortical_types

def get_detailed_cortical_map(state) -> Dict[str, Dict[str, list]]:
    """
    Builds a nested dictionary representing all cortical area mappings in the connectome.
    Args:
        state: The FeagiStateManager or similar object with .genome["blueprint"]
    Returns:
        Dict[src_area][dst_area] = list of mapping objects
    """
    cortical_map = dict()
    blueprint = state.genome["blueprint"]
    for cortical_area in blueprint:
        cortical_map[cortical_area] = dict()
        for dst in blueprint[cortical_area].get("cortical_mapping_dst", {}):
            cortical_map[cortical_area][dst] = list()
            for mapping in blueprint[cortical_area]["cortical_mapping_dst"][dst]:
                cortical_map[cortical_area][dst].append(mapping)
    return cortical_map 

def build_power_connections(connectome, target_area_id: str, cortical_type: str, mapping_dict: dict) -> None:
    """
    Create or update power connections for a target cortical area, based on mapping_dict.
    Args:
        connectome: The ConnectomeManager instance to use.
        target_area_id: The cortical area to connect to.
        cortical_type: The type of the cortical area (e.g., 'OPU').
        mapping_dict: Dict of {entry: value} for mapping patterns.
    """
    state = FeagiStateManager.get_instance()
    power_area = "___pwr"
    cortical_template = cortical_types[cortical_type]["supported_devices"][target_area_id].copy()

    if target_area_id not in state.genome["blueprint"]:
        connectome.add_core_cortical_area({
            "cortical_id": target_area_id,
            "cortical_type": cortical_type,
            "cortical_name": cortical_template["cortical_name"],
            "coordinates_3d": cortical_template["coordinate_3d"],
            "dev_count": 1,
            "coordinates_2d": [randrange(0, 10), randrange(0, 10)]
        })

    target_area_width = state.genome["blueprint"][target_area_id]["block_boundaries"][0]
    target_area_depth = state.genome["blueprint"][target_area_id]["block_boundaries"][2]

    morphology_template = {
        "parameters": {
            "patterns": []
        },
        "type": "patterns",
        "class": "custom"
    }

    for entry in mapping_dict:
        if mapping_dict[entry] or mapping_dict[entry] == 0:
            if int(entry) < target_area_width and 0 <= mapping_dict[entry] <= 1:
                target_voxel = floor((target_area_depth - 1) * mapping_dict[entry])
                morphology_template["parameters"]["patterns"].append([[0, 0, 0], [int(entry), 0, target_voxel]])

    # Remove existing mapping if present
    power_mappings = state.genome["blueprint"][power_area].get("cortical_mapping_dst")
    if power_mappings:
        existing_power_to_target_area = power_mappings.get(target_area_id)
        if existing_power_to_target_area:
            del state.genome["blueprint"][power_area]["cortical_mapping_dst"][target_area_id]

    morphology_name = f"system-{power_area}-{target_area_id}"
    state.genome["neuron_morphologies"][morphology_name] = morphology_template

    mapping_data = [{
        'morphology_id': morphology_name,
        'morphology_scalar': [1, 1, 1],
        'plasticity_flag': False,
        'postSynapticCurrent_multiplier': 1
    }]

    connectome.update_cortical_mappings({
        "src_cortical_area": power_area,
        "dst_cortical_area": target_area_id,
        "mapping_data": mapping_data
    }) 