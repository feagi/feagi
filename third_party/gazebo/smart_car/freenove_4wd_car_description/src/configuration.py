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
    "feagi_host": "feagi",
    "feagi_api_port": "8000",
    'TTL': 2,
    'last_message': 0,
}

capabilities = {
    "servo": {
        "type": "opu",
        "disabled": True,
        "refresh_rate": 1,
        "cortical_mapping": "o__ser",
        'count': 2,
        'topic_identifier': '/S'
    },
    "motor": {
        "type": "opu",
        "disabled": False,
        "count": 4,
        'topic_identifier': '/M',
        "refresh_rate": 1,
        "cortical_mapping": "o__mot",
        "power_coefficient": 0.05,
        "wheel_diameter": 0.065,  # radius is in sdf under wheels inside <link></link>
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
    "position": {
        "x": 0,
        "y": 4.12,
        "z": 0.325
    }
}

Gazebo_world = {
    "counter": 0,
    "margin": 0.20,
    "size_of_plane": {
        "x": 10,  # in sdf, it is 100x100 so 100/10 = 10
        "y": 10
    }
}

message_to_feagi = {"data": {}}


Model_data = {
    "path_to_robot": "models/sdf/",
    "file_name": "freenove_smart_car.sdf",
    "mu": 1.0,
    "mu2": 50.0,
    "fdir1": [1, 1, 1],
    "slip1": 0.1,
    "slip2": 0.1
}
