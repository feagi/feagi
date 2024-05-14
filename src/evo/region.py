
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

import string
import random
import datetime

from fastapi.responses import JSONResponse

from src.inf import runtime_data


def region_id_gen(size=6, chars=string.ascii_uppercase + string.digits):

    now = datetime.datetime.now()
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_' + (''.join(random.choice(chars) for _ in range(size))) + '_R'


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
        print(f"Exception during change of cortical area region assignment: {e}")


def change_brain_region_parent(region_id, new_parent_id):
    current_parent_id = runtime_data.genome["brain_regions"][region_id]["parent_region_id"]
    runtime_data.genome["brain_regions"][region_id]["parent_region_id"] = new_parent_id
    runtime_data.genome["brain_regions"][current_parent_id]["regions"].remove(region_id)
    runtime_data.genome["brain_regions"][new_parent_id]["regions"].append(region_id)


def create_region(region_data):
    region_id = region_id_gen()
    runtime_data.genome["brain_regions"][region_id] = {}
    runtime_data.genome["brain_regions"][region_id]["title"] = region_data.region_title
    runtime_data.genome["brain_regions"][region_id]["parent_region_id"] = region_data.parent_region_id
    runtime_data.genome["brain_regions"][region_id]["coordinate_2d"] = region_data.coordinates_2d
    runtime_data.genome["brain_regions"][region_id]["coordinate_2d"] = region_data.coordinates_3d
    runtime_data.genome["brain_regions"][region_id]["areas"] = list()
    runtime_data.genome["brain_regions"][region_id]["regions"] = list()
    runtime_data.genome["brain_regions"][region_id]["inputs"] = dict()
    runtime_data.genome["brain_regions"][region_id]["outputs"] = dict()
    runtime_data.genome["brain_regions"][region_data.parent_region_id]["regions"].append(region_id)
    if region_data.areas:
        for area in region_data.areas:
            runtime_data.genome["brain_regions"][region_id]["areas"][area].append(region_data.areas[area])
    if region_data.regions:
        for region in region_data.regions:
            runtime_data.genome["brain_regions"][region_id]["regions"][region].append(region_data.regions[region])
    return region_id


def delete_region():
    pass






