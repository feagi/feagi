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
import os

feagi_settings = {
    # "feagi_auth_url": os.environ.get('URL_MICROBIT', None),
    "feagi_url": None,
    "feagi_dns": None,
    "feagi_host": os.environ.get('FEAGI_HOST_INTERNAL', "127.0.0.1"),
    "feagi_api_port": "8000",
}

agent_settings = {
    "agent_data_port": "10003",
    "agent_id": "microbit",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
    'godot_websocket_ip': "0.0.0.0",
    'godot_websocket_port': os.environ.get('WS_MICROBIT_PORT', "9052"),
    'compression': True

}

capabilities = {
    "motor": {
        "type": "opu",
        "disabled": False,
        "count": 2,
        'topic_identifier': '/M',
        "refresh_rate": 1,
        "cortical_mapping": "o__mot",
        "rolling_window_len": 5,
        "diameter_of_wheel": 0.065,
        "power_amount": 0.1
    },
    "infrared": {
        "type": "ipu",
        "disabled": False,
        "count": 2,
        "refresh_rate": 1,
        "cortical_mapping": "i__inf",
        'topic_identifier': 'IR'
    }
}

message_to_feagi = {"data": {}}
