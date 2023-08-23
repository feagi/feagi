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

app_name = 'gazebo'

network_settings = {
    "feagi_host": "127.0.0.1",
    "feagi_api_port": "8000",
    "feagi_opu_port": "30000",
    "feagi_inbound_port_gazebo": "30002",
    'TTL': 2,
    'last_message': 0,
    "microbit_mac_address": "xx.xx.xx.xx.xx.xx",
    "primary_mac_address": "xx.xx.xx.xx.xx.xx"
    }

capabilities = {}

message_to_feagi = {"data": {}}
