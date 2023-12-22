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

import os

feagi_settings = {
    # "feagi_auth_url": "http://127.0.0.1:9000/v1/k8/feagi_settings/auth_token",
    "feagi_url": None,
    "feagi_dns": None,
    "feagi_host": os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
    "feagi_api_port": os.environ.get('FEAGI_API_PORT', "8000")
}
agent_settings = {
    "agent_data_port": "10006",
    "agent_id": "javascript_webcam",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
    'godot_websocket_ip': "0.0.0.0",
    'godot_websocket_port': os.environ.get('WS_WEBCAM_PORT', "9051"),
    'compression': True
}

capabilities = {
    "camera": {
        "type": "ipu",
        "disabled": False,
        "index": "00",
        "threshold_default": [10, 255, 130, 51],  # min #1, max #1, min #2, max #2,
        "threshold_range": [1, 255],
        "threshold_type": {},
        "threshold_name": 0, # Binary_threshold
        "central_vision_allocation_percentage": [80, 60],
        "central_vision_resolution": [64, 64],
        "peripheral_vision_resolution": [8, 8],
        "resolution_presets": [[8, 8], [16, 16], [32, 32], [64, 64], [128, 128], [256, 256],
                               [400, 400], [500, 500], [800, 800], [1024, 900]],
        "previous_data": {},
        "video_device_index": 2,
        "aperture_range": [0.1, 2],
        "aperture_default": 2,
        "mirror": True,
        "blink": [],
        "gaze_control": {0: 25, 1: 50},
        "pupil_control": {0: 25, 1: 50},
        "vision_range": [1, 99],
        "size_list": [],
        "effect": {},
        "enhancement": {},
        "vision_tuner_range": [0, 255]
    }
}

message_to_feagi = {"data": {}}
