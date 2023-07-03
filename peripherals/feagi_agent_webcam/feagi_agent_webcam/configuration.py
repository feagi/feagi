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
composer_url = f'http://127.0.0.1:9000/v1/k8/feagi_settings/auth_token'

feagi_settings = {
    "feagi_url": None,
    "feagi_host": "127.0.0.1",
    "feagi_api_port": "8000",
}

agent_settings = {
    "agent_data_port": "40010",
    "agent_id": "camera_1",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
}

capabilities = {
    "camera": {
        "type": "ipu",
        "disabled": False,
        "count": 1,
        "width": 8,
        "height": 8,
        "deviation_threshold": 0.5,
        "retina_width_percent": 90,
        "retina_height_percent": 70,
        "central_vision_compression": [64, 64],
        "peripheral_vision_compression": [8, 8],
        "previous_data": {},
        "video_device_index": 0,
        "video_loop": False
    }
}

message_to_feagi = {"data": {}}
