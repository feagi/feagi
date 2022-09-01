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

app_name = 'gazebo'

network_settings = {
    "feagi_host": "127.0.0.1",
    "feagi_api_port": "8000",
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
        "power_coefficient": 0.02,
        "wheel_diameter": 0.065,  # radius is in sdf under wheels inside <link></link>
    },
    "camera": {
        "type": "ipu",
        "disabled": False,
        "count": 1,
        "width": 16,
        "height": 16,
        "deviation_threshold": 0
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
        "range": 360  # -180 to 180 degree
    },
    "acc": {
        "resolution": 20,
        "range": 2400  # -180 to 180 degree
    }
}

message_to_feagi = {"data": {}}
