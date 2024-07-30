
# Copyright 2016-2023 The FEAGI Authors. All Rights Reserved.
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


from src.evo.templates import cortical_types


def area_is_system(cortical_area):
    result = False
    for area_type in cortical_types:
        if "supported_devices" in cortical_types[area_type]:
            for cortical_id in cortical_types[area_type]["supported_devices"]:
                if cortical_area == cortical_id:
                    result = True
    return result


def cortical_area_type(cortical_area):
    cortical_type = "CUSTOM"
    for area_type in cortical_types:
        if "supported_devices" in cortical_types[area_type]:
            for cortical_id in cortical_types[area_type]["supported_devices"]:
                if cortical_area == cortical_id:
                    cortical_type = area_type
    return cortical_type
