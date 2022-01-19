
# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
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

from inf import runtime_data


def convert_ir_to_fire_list(ir_data):
    """

    The keys in ir_data correlate to the index id of each Infrared Sensor

    ir_data = {
        0: True,
        1: True,
        2: False
    }
    """
    fire_list = set()
    for sensor_idx in ir_data:
        if ir_data[sensor_idx]:
            for key in runtime_data.brain['ir_ipu']:
                if sensor_idx == runtime_data.brain['ir_ipu'][key]['soma_location'][0][0]:
                    fire_list.add(key)

    runtime_data.fcl_queue.put({'ir_ipu': fire_list})
