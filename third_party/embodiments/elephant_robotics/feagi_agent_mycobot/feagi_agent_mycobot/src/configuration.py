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
    "agent_data_port": "10005",
    "agent_id": "mycobot",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
    'compression': True
}

capabilities = {
    "camera": {
        "type": "ipu",
        "disabled": False,
        "count": 1,
        "width": 8,
        "height": 8,
        "deviation_threshold": 0.1,
        "retina_width_percent": 90,
        "retina_height_percent": 80,
        "central_vision_compression": [64, 64],
        "peripheral_vision_compression": [8, 8],
        "previous_data": {}
    },
    "servo": {
        "type": "opu",
        "disabled": False,
        "refresh_rate": 1,
        "cortical_mapping": "o__ser",
        'count': 7,
        'topic_identifier': '/S',
        'port': {
            '0': '/dev/ttyUSB0'  # Find a way to add without hardcode
        },
        'servo_range': {
            '1': [200, 3600],
            '2': [0, 0],
            '3': [400, 3400],
            '4': [700, 3300],
            '5': [150, 3600],
            '6': [0, 4096]
        },
        'power': 50,
        'sensitivity':
            {
                'micro': 50,
                'macro': 150
            }
    }
}

message_to_feagi = {"data": {}}
