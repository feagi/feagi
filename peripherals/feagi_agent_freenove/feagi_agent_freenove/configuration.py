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
    "agent_data_port": "10004",
    "agent_id": "freenove",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
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
        "power_amount": 650
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
        "count": 1,
        "width": 8,
        "height": 8,
        "deviation_threshold": 0.7,
        "retina_width_percent": 90,
        "retina_height_percent": 80,
        "central_vision_compression": [64, 64],
        "peripheral_vision_compression": [8, 8],
        "previous_data": {}
    },
}

message_to_feagi = {"data": {}}
