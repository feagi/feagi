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
    "feagi_api_port": "8000",
}

agent_settings = {
    "agent_data_port": "10007",
    "agent_id": "gazebo",
    "agent_type": "embodiment",
    'TTL': 2,
    'last_message': 0,
    'compression': True
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
        "power_coefficient": 0.02,
        "wheel_diameter": 0.065,  # radius is in sdf under wheels inside <link></link>
    },
    "infrared": {
        "type": "ipu",
        "disabled": False,
        "count": 3,
        "refresh_rate": 1,
        "cortical_mapping": "i__inf",
        "threshold": 25,
        'topic_identifier': 'IR'
    },
    "camera": {
        "type": "ipu",
        "disabled": False,
        "index": "00",
        "iso_default": 0.1,
        "iso_range": [0.1, 2],
        "central_vision_allocation_percentage": [60, 40],
        "central_vision_resolution": [64, 64],
        "peripheral_vision_resolution": [8, 8],
        "resolution_presets": [[8, 8], [16, 16], [32, 32], [64, 64], [128, 128], [256, 256],
                               [400, 400], [500, 500], [600, 600], [800, 800], [1024, 900]],
        "previous_data": {},
        "video_device_index": 2,
        "aperture_range": [0.1, 2],
        "aperture_default": 2
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
        "0": {
            "x": 0,
            "y": 4.12,
            "z": 0.325
        },
        "1": {
            "x": 0,
            "y": 0,
            "z": 0
        },
        "2": {
            "x": 0,
            "y": 1,
            "z": 0
        },
        "3": {
            "x": 0,
            "y": 2,
            "z": 0
        },
        "4": {
            "x": 0,
            "y": 3,
            "z": 0
        }
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
    "robot_model_path": "robots/smart_car/",
    "robot_model": "smart_car.sdf",
    "floor_img": "map1.png",
    "floor_img_path": "environments/material",
    "mu": 1.0,
    "mu2": 50.0,
    "fdir1": [1, 1, 1],
    "slip1": 0.1,
    "slip2": 0.1
}
