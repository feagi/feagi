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

feagi_settings = {
    "feagi_host": "127.0.0.1",
    "feagi_api_port": "8000",
}

agent_settings = {
    "agent_data_port": "30004",
    "agent_id": "psychopy",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
}

capabilities = {
    "camera": {
        "type": "ipu",
        "disabled": False,
        "index": "00",
        "deviation_threshold": 0.05,
        "retina_width_percent": 60,
        "retina_height_percent": 40,
        "central_vision_compression": [64, 64],
        "peripheral_vision_compression": [8, 8],
        "previous_data": {},
        "field_of_vision_x": 10,
        "field_of_vision_y": 10,
        "field_of_vision_dimension": [500, 500],
        "camera_pose": [0, 0]
    }
}
message_to_feagi = {"data": {}}
