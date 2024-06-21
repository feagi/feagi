
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
from src.evo.synapse import neighboring_cortical_areas
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
    region_id = region_id_gen()
    runtime_data.genome["brain_regions"][region_id] = {}
    runtime_data.genome["brain_regions"][region_id]["title"] = region_data.title
    runtime_data.genome["brain_regions"][region_id]["description"] = region_data.region_description
    runtime_data.genome["brain_regions"][region_id]["parent_region_id"] = region_data.parent_region_id
    runtime_data.genome["brain_regions"][region_id]["coordinate_2d"] = region_data.coordinates_2d
    runtime_data.genome["brain_regions"][region_id]["coordinate_3d"] = region_data.coordinates_3d
    runtime_data.genome["brain_regions"][region_id]["areas"] = list()
    runtime_data.genome["brain_regions"][region_id]["regions"] = list()
    runtime_data.genome["brain_regions"][region_id]["inputs"] = list()
    runtime_data.genome["brain_regions"][region_id]["outputs"] = list()
    # runtime_data.genome["brain_regions"][region_id]["suggested_afferents"] = list()
    # runtime_data.genome["brain_regions"][region_id]["suggested_efferents"] = list()
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


def direct_region_cortical_areas(region_id):
    area_list = runtime_data.genome["brain_regions"][region_id]["areas"]
    return area_list


def recursive_sub_regions(region_id):
    region_list = set()
    for region in runtime_data.genome["brain_regions"][region_id]["regions"]:
        region_list.add(region)

    return region_list


def recursive_region_cortical_areas(region_id):
    recursive_region_list = recursive_sub_regions(region_id)
    area_list = set()
    area_list.update(set(direct_region_cortical_areas(region_id=region_id)))
    for sub_region_id in recursive_region_list:
        area_list.update(set(direct_region_cortical_areas(region_id=sub_region_id)))
    return area_list


def construct_genome_from_region(region_id):
    genome_from_region = {
        "genome_title": runtime_data.genome["brain_regions"][region_id].get("title", "No Title"),
        "genome_description": runtime_data.genome["brain_regions"][region_id].get("description", "No Description"),
        "timestamp": time(),
        "genome_id": genome_id_gen(size=6, chars=string.ascii_uppercase + string.digits),
        "version": runtime_data.genome["version"],
        "signatures": {},
        "stats": {},
        "hosts": {},
        "physiology": runtime_data.genome["physiology"],
        "neuron_morphologies": {},
        "blueprint": {},
        "brain_regions": {
            "root": {
                "title": runtime_data.genome["brain_regions"][region_id].get("title", "No Title"),
                "parent_region_id": None,
                "coordinate_2d": [0, 0],
                "coordinate_3d": [0, 0, 0],
                "areas": runtime_data.genome["brain_regions"][region_id]["areas"],
                "regions": runtime_data.genome["brain_regions"][region_id]["regions"],
                "inputs": [],
                "outputs": []
            }
        }
    }

    region_cortical_list = direct_region_cortical_areas(region_id=region_id)
    comprehensive_subregion_list = recursive_sub_regions(region_id=region_id)
    comprehensive_area_list = recursive_region_cortical_areas(region_id=region_id)

    # Generate blueprint
    for cortical_area in comprehensive_area_list:
        genome_from_region["blueprint"][cortical_area] = runtime_data.genome["blueprint"][cortical_area].copy()

    # Create Region list
    genome_from_region["brain_regions"]["root"] = runtime_data.genome["brain_regions"][region_id].copy()
    genome_from_region["brain_regions"]["root"]["parent_region_id"] = None
    genome_from_region["brain_regions"]["root"]["coordinate_2d"] = [0, 0]
    genome_from_region["brain_regions"]["root"]["coordinate_3d"] = [0, 0, 0]

    for region in comprehensive_subregion_list:
        genome_from_region["brain_regions"][region] = runtime_data.genome["brain_regions"][region].copy()

    region_afferents = set()
    region_efferents = set()

    # Create suggested input/output mappings for all regions
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
    for subregion in runtime_data.genome["brain_regions"][region_id]["regions"] + [region_id]:
        subregion_suggested_afferents = []
        subregion_suggested_efferents = []
        afferent_areas = set()
        efferent_areas = set()
        subregion_recursive_cortical_list = recursive_region_cortical_areas(region_id=subregion)

        # Develop Afferent / Efferent lists
        for area in runtime_data.genome["brain_regions"][subregion]["areas"]:
            upstream_cortical_areas, downstream_cortical_areas = \
                neighboring_cortical_areas(cortical_area=area, blueprint=runtime_data.genome["blueprint"])

            # Afferents
            for area_ in upstream_cortical_areas:
                if area_ not in subregion_recursive_cortical_list:
                    afferent_areas.add(area_)

            # Efferents
            for area_ in downstream_cortical_areas:
                if area_ not in subregion_recursive_cortical_list:
                    efferent_areas.add(area_)

        # Build suggested afferent list for each region
        for afferent_area in afferent_areas:
            for dst in runtime_data.genome["blueprint"][afferent_area]["cortical_mapping_dst"]:
                if dst in region_cortical_list:
                    for mapping in runtime_data.genome["blueprint"][afferent_area]["cortical_mapping_dst"][dst]:
                        mapping["src_cortical_area_id"] = afferent_area
                        mapping["src_cortical_area_name"] = runtime_data.genome["blueprint"][afferent_area][
                            "cortical_name"]
                        mapping["src_cortical_area_custom_label"] = None
                        mapping["dst_cortical_area_id"] = dst
                        subregion_suggested_afferents.append(mapping)

        # Build suggested efferent list
        for cortical_area in comprehensive_area_list:
            for dst in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"]:
                if dst in efferent_areas:
                    for mapping in runtime_data.genome["blueprint"][cortical_area]["cortical_mapping_dst"][dst]:
                        mapping["src_cortical_area_id"] = cortical_area
                        mapping["src_cortical_area_name"] = runtime_data.genome["blueprint"][cortical_area][
                            "cortical_name"]
                        mapping["src_cortical_area_custom_label"] = None
                        mapping["dst_cortical_area_id"] = dst
                        subregion_suggested_efferents.append(mapping)

        if subregion == region_id:
            genome_from_region["brain_regions"]["root"]["inputs"] = subregion_suggested_afferents
            genome_from_region["brain_regions"]["root"]["outputs"] = subregion_suggested_efferents
        # else:
        #     genome_from_region["brain_regions"][subregion]["inputs"] = subregion_suggested_afferents
        #     genome_from_region["brain_regions"][subregion]["outputs"] = subregion_suggested_efferents

        region_efferents.update(efferent_areas)
        region_afferents.update(afferent_areas)

    # Scrub Efferents
    for cortical_area in genome_from_region["blueprint"].copy():
        for dst in genome_from_region["blueprint"][cortical_area]["cortical_mapping_dst"].copy():
            if dst in region_efferents:
                genome_from_region["blueprint"][cortical_area]["cortical_mapping_dst"].pop(dst)

    # Set region stats
    region_neuron_count = 0
    region_synapse_count = 0

    for cortical_area in comprehensive_area_list:
        neuron_cnt, synapse_cnt = cortical_area_anatomical_stats(cortical_area=cortical_area)
        region_neuron_count += neuron_cnt
        region_synapse_count += synapse_cnt

    genome_from_region["stats"]["innate_cortical_area_count"] = len(comprehensive_area_list)
    genome_from_region["stats"]["innate_neuron_count"] = region_neuron_count
    genome_from_region["stats"]["innate_synapse_count"] = region_synapse_count

    # Set morphologies
    morphology_list = set()
    for cortical_area in comprehensive_area_list:
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

    # Convert Genome to 2.0
    genome_from_region["blueprint"] = genome_v1_v2_converter(genome_from_region)["blueprint"]

    return genome_from_region


def cortical_area_region_sanity_check(cortical_area):
    if cortical_area not in runtime_data.cortical_area_region_association:
        runtime_data.cortical_area_region_association[cortical_area] = "root"
