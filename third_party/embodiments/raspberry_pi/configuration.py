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
#!/usr/bin/env python3

router_settings = {
    "feagi_ip": "127.0.0.1",
    "feagi_port": "30000",
    "ipu_port": "30001",
    'TTL': 2,
    'last_message': 0,
    'feagi_burst_speed':  1
}

model_properties = {
    'motor': {
        'count': 4,
        'topic_identifier': '/M',
        'motor_statuses': {},
        'rolling_window_len': 5
    },
    'servo': {
        'count': 2,
        'topic_identifier': '/S'
    },
    'infrared': {
        'count': 3,
        'topic_identifier': 'IR'
    }
}