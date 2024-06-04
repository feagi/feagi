
# Copyright 2016-2024 The FEAGI Authors. All Rights Reserved.
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

"""
This module covers needed functions for brain region management
"""
import json
import string
import random
import datetime
import traceback

from time import time

from src.inf import runtime_data
from src.evo.voxels import generate_cortical_dimensions_by_id
from src.evo.stats import cortical_area_anatomical_stats
from src.evo.genetics import genome_id_gen
from src.evo.genome_processor import genome_v1_v2_converter
from src.evo.genome_editor import generate_hash
from src.api.commons import CustomError


def region_id_gen(size=6, chars=string.ascii_uppercase + string.digits):

    now = datetime.datetime.now()
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_' + (''.join(random.choice(chars) for _ in range(size))) + '_R'


def region_id_2_title(region_id):
    if region_id in runtime_data.genome["brain_regions"]:
        return runtime_data.genome["brain_regions"][region_id]["title"]


def change_cortical_area_parent(cortical_area_id, new_parent_id):
    try:
        current_parent_id = runtime_data.cortical_area_region_association[cortical_area_id]
        print("current_parent_id:", current_parent_id, new_parent_id)
        runtime_data.cortical_area_region_association[cortical_area_id] = new_parent_id
        print(">>", runtime_data.genome["brain_regions"][current_parent_id]["areas"])
        if cortical_area_id in runtime_data.genome["brain_regions"][current_parent_id]["areas"]:
            runtime_data.genome["brain_regions"][current_parent_id]["areas"].remove(cortical_area_id)
        else:
            print(f"{cortical_area_id} not found in current region {current_parent_id}")
        print(">>", runtime_data.genome["brain_regions"][current_parent_id]["areas"])
        runtime_data.genome["brain_regions"][new_parent_id]["areas"].append(cortical_area_id)
    except Exception as e:
        print(f"Exception during change of cortical area region assignment: {e}", traceback.print_exc())


def change_brain_region_parent(region_id, new_parent_id):
    current_parent_id = runtime_data.genome["brain_regions"][region_id]["parent_region_id"]
    runtime_data.genome["brain_regions"][region_id]["parent_region_id"] = new_parent_id
    runtime_data.genome["brain_regions"][current_parent_id]["regions"].remove(region_id)
    runtime_data.genome["brain_regions"][new_parent_id]["regions"].append(region_id)


def create_region(region_data):
    """

    suggested_afferents = [
        {
            "src_cortical_area_id": "___pwr",
            "src_cortical_area_name": "brain_power",
            "src_cortical_area_custom_label": "power",
            "dst_cortical_area_id": "C2WFW3",
            "morphology_id": "projector",
            "postSynapticCurrent_multiplier": "1",
            "morphology_scalar": [1,1,1],
            "plasticity_flag": True,
            "plasticity_constant": 1,
            "ltp_multiplier": 2,
            "ltd_multiplier": 1
        },
        {}
    ]
    """

    region_id = region_id_gen()
    runtime_data.genome["brain_regions"][region_id] = {}
    runtime_data.genome["brain_regions"][region_id]["title"] = region_data.region_title
    runtime_data.genome["brain_regions"][region_id]["description"] = region_data.region_description
    runtime_data.genome["brain_regions"][region_id]["parent_region_id"] = region_data.parent_region_id
    runtime_data.genome["brain_regions"][region_id]["coordinate_2d"] = region_data.coordinates_2d
    runtime_data.genome["brain_regions"][region_id]["coordinate_3d"] = region_data.coordinates_3d
    runtime_data.genome["brain_regions"][region_id]["areas"] = list()
    runtime_data.genome["brain_regions"][region_id]["regions"] = list()
    runtime_data.genome["brain_regions"][region_id]["inputs"] = dict()
    runtime_data.genome["brain_regions"][region_id]["outputs"] = dict()
    runtime_data.genome["brain_regions"][region_id]["suggested_afferents"] = list()
    runtime_data.genome["brain_regions"][region_id]["suggested_efferents"] = list()
    runtime_data.genome["brain_regions"][region_id]["regions"] = list()
    runtime_data.genome["brain_regions"][region_id]["signature"] = ""
    runtime_data.genome["brain_regions"][region_data.parent_region_id]["regions"].append(region_id)
    if region_data.areas:
        for associated_area in region_data.areas:
            if associated_area in runtime_data.cortical_list:
                change_cortical_area_parent(cortical_area_id=associated_area,
                                            new_parent_id=region_id)

    if region_data.regions:
        for associated_region in region_data.regions:
            if associated_region in runtime_data.genome["brain_regions"]:
                runtime_data.genome["brain_regions"][region_id]["regions"].append(associated_region)
    return region_id


def update_region(region_data):
    region_id = region_data["region_id"]
    region_data.pop("region_id")
    for update in region_data:
        if update not in ["area", "region"]:
            if update == "parent_region_id":
                change_brain_region_parent(region_id=region_id, new_parent_id=update["parent_region_id"])
            else:
                runtime_data.genome["brain_regions"][region_id][update] = region_data[update]
        else:
            raise CustomError(status_code=400, message=f"{update} cannot be updated using this endpoint")


def delete_region_with_members(region_id):
    if region_id in runtime_data.genome["brain_regions"]:
        parent_region = runtime_data.genome["brain_regions"][region_id]["parent_region_id"]
        for area in runtime_data.genome["brain_regions"][region_id]:
            change_cortical_area_parent(cortical_area_id=area, new_parent_id=parent_region)
        for region in runtime_data.genome["brain_regions"][region_id]:
            change_brain_region_parent(region_id=region, new_parent_id=parent_region)
        runtime_data.genome["brain_regions"].pop(region_id)


def relocate_region_members(relocation_data):
    for object_id in relocation_data:
        if object_id in runtime_data.genome["blueprint"]:
            if "coordinate_2d" in relocation_data[object_id]:
                runtime_data.genome["blueprint"][object_id]["2d_coordinate"][0] = \
                    relocation_data[object_id]["coordinate_2d"][0]
                runtime_data.genome["blueprint"][object_id]["2d_coordinate"][1] = \
                    relocation_data[object_id]["coordinate_2d"][1]
            if "parent_region_id" in relocation_data[object_id]:
                change_cortical_area_parent(cortical_area_id=object_id,
                                            new_parent_id=relocation_data[object_id]["parent_region_id"])
        elif object_id in runtime_data.genome["brain_regions"]:
            if "coordinate_2d" in relocation_data[object_id]:
                runtime_data.genome["brain_regions"][object_id]["coordinate_2d"][0] = \
                    relocation_data[object_id]["coordinate_2d"][0]
                runtime_data.genome["brain_regions"][object_id]["coordinate_2d"][1] = \
                    relocation_data[object_id]["coordinate_2d"][1]
            if "parent_region_id" in relocation_data[object_id]:
                if relocation_data[object_id]["parent_region_id"] in runtime_data.genome["brain_regions"]:
                    change_brain_region_parent(region_id=object_id,
                                               new_parent_id=relocation_data[object_id]["parent_region_id"])
                else:
                    parent_id = relocation_data[object_id]["parent_region_id"]
                    raise CustomError(status_code=400, message=f"{parent_id} is not a valid region id")
        else:
            raise CustomError(status_code=400, message=f"{object_id} is not a valid region nor cortical id")

    runtime_data.cortical_dimensions_by_id = generate_cortical_dimensions_by_id()


def construct_genome_from_region(region_id):
    genome_from_region = {
        "title": region_id_2_title(region_id=region_id),
        "timestamp": time(),
        "genome_id": genome_id_gen(size=6, chars=string.ascii_uppercase + string.digits),
        "version": runtime_data.genome["version"],
        "signatures": {},
        "stats": {},
        "hosts": {},
        "physiology": runtime_data.genome["physiology"],
        "neuron_morphologies": {},
        "blueprint": {},
        "brain_regions": {}
    }

    region_cortical_list = runtime_data.genome["brain_regions"][region_id]["areas"]

    print("region_cortical_list:", region_cortical_list)

    # Set region stats
    region_neuron_count = 0
    region_synapse_count = 0

    for cortical_area in region_cortical_list:
        neuron_cnt, synapse_cnt = cortical_area_anatomical_stats(cortical_area=cortical_area)
        region_neuron_count += neuron_cnt
        region_synapse_count += synapse_cnt

    genome_from_region["stats"]["innate_cortical_area_count"] = len(region_cortical_list)
    genome_from_region["stats"]["innate_neuron_count"] = region_neuron_count
    genome_from_region["stats"]["innate_synapse_count"] = region_synapse_count

    # Generate blueprint
    for cortical_area in region_cortical_list:
        genome_from_region["blueprint"][cortical_area] = runtime_data.genome["blueprint"][cortical_area].copy()
    genome_from_region["blueprint"] = genome_v1_v2_converter(genome_from_region)["blueprint"]

    # Set morphologies
    morphology_list = set()
    for cortical_area in region_cortical_list:
        for mapping_destination in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
            for mapping in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"][mapping_destination]:
                morphology_list.add(mapping["morphology_id"])

    for morphology_id in morphology_list:
        genome_from_region["neuron_morphologies"][morphology_id] = \
            runtime_data.genome["neuron_morphologies"][morphology_id].copy()

    # Set signatures
    genome_from_region["signatures"]["genome"] = generate_hash(genome_from_region)
    genome_from_region["signatures"]["blueprint"] = generate_hash(genome_from_region["blueprint"])
    genome_from_region["signatures"]["physiology"] = generate_hash(genome_from_region["physiology"])

    print("#__" * 20)
    print(json.dumps(genome_from_region, indent=4))
    return genome_from_region
