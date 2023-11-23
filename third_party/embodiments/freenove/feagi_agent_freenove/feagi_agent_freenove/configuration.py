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
    "feagi_url": None,  # gets updated
    "feagi_auth_url": None,  # composer for getting the Feagi URL link - First Priority
    "feagi_dns": None,  # URL https://neurobotics.studio - Second priority
    "feagi_host": "127.0.0.1",  # feagi IP  - third priority
    "feagi_api_port": "8000",  # feagi Port - third priority
}

agent_settings = {
    "agent_data_port": "10004",
    "agent_id": "freenove",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
    'compression': True
}

capabilities = {
    "servo": {
        "type": "opu",
        "disabled": False,
        "refresh_rate": 1,
        "cortical_mapping": "o__ser",
        'count': 2,
        'topic_identifier': '/S',
        'power_amount': 0.5

    },
    "motor": {
        "type": "opu",
        "disabled": False,
        "count": 4,
        'topic_identifier': '/M',
        "refresh_rate": 1,
        "cortical_mapping": "o__mot",
        "rolling_window_len": 5,
        "diameter_of_wheel": 0.065,
        "power_amount": 65
    },
    "infrared": {
        "type": "ipu",
        "disabled": False,
        "count": 3,
        "refresh_rate": 1,
        "cortical_mapping": "i__inf",
        'topic_identifier': 'IR'
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
    "camera": {
        "type": "ipu",
        "disabled": False,
        "index": "00",
        "iso_default": 0.9,
        "iso_range": [0.1, 2],
        "central_vision_allocation_percentage": [95, 80],
        "central_vision_resolution": [64, 64],
        "peripheral_vision_resolution": [8, 8],
        "resolution_presets": [[8, 8], [16, 16], [32, 32], [64, 64], [128, 128], [256, 256],
                               [400, 400], [500, 500], [800, 800], [1024, 900]],
        "previous_data": {},
        "aperture_range": [0.2, 2],
        "aperture_default": 2,
        "mirror": True
    },
    "led": {
        "type": "opu"
    }
}

message_to_feagi = {"data": {}}
