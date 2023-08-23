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

# !/usr/bin/env python3

feagi_settings = {
    # "feagi_auth_url": "http://127.0.0.1:9000/v1/k8/feagi_settings/auth_token",
    "feagi_url": None,
    "feagi_dns": None,
    "feagi_host": "127.0.0.1",
    "feagi_api_port": "8000",
}

agent_settings = {
    "agent_data_port": "10002",
    "agent_id": "tello_drone",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
}


capabilities = {
    "motor": {
        "type": "opu",
        "disabled": False,
        "count": 4,
        'topic_identifier': '/M',
        "refresh_rate": 1,
        "cortical_mapping": "o__mot",
        "power_coefficient": 11,
        "wheel_diameter": 0.065,  # radius is in sdf under wheels inside <link></link>
    },
    "camera": {
        "type": "ipu",
        "disabled": False,
        "count": 1,
        "width": 8,
        "height": 8,
        "deviation_threshold": 0.3,
        "retina_width_percent": 60,
        "retina_height_percent": 40,
        "central_vision_compression": [64, 64],
        "peripheral_vision_compression": [8, 8],
        "previous_data": {},
        "field_of_vision_x": 800,
        "field_of_vision_y": 600,
        "field_of_vision_origin": [100, 100]
    },
    "battery": {
        "type": "ipu",
        "disabled": False,
        "count": 4,
        "refresh_rate": 1,
        "cortical_mapping": "i__bat",
        "capacity": 100,
        "depletion_per_burst": 0.01,
        "charge_increment": 0.1
    },
    "gyro": {
        "resolution": 20,
        "range": [-180, 180]  # -180 to 180 degree
    },
    "acc": {
        "resolution": 20,
        "range": [-1200, 1200]
    }
}

message_to_feagi = {"data": {}}
