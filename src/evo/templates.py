
# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
Defines all supported sensors and actuator types and properties.
"""

cortical_types = {
    "IPU": {
        "gui_name": "Sensors",
        "supported_devices": {
            "infrared_sensor": {
                "enabled": True,
                "cortical_id": "i__inf",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "count": int
            },
            "inverse_infrared_sensor": {
                "enabled": True,
                "cortical_id": "i__inf",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "count": int
            },
            "proximity_sensor": {
                "enabled": True,
                "cortical_id": "i__pro",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "count": int
            },
            "accelerometer_sensor": {
                "enabled": True,
                "cortical_id": "i__acc",
                "structure": "symmetric",
                "resolution": [1, 1, 20],
                "count": int
            },
            "gyroscope_sensor": {
                "enabled": True,
                "cortical_id": "i__gyr",
                "structure": "symmetric",
                "resolution": [1, 1, 20],
                "count": int
            },
            "shock_sensor": {
                "enabled": True,
                "cortical_id": "ishock",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "count": int
            },
            "battery_sensor": {
                "enabled": True,
                "cortical_id": "i__bat",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "count": int
            },
            "vision_sensor": {
                "enabled": True,
                "cortical_id": "i__vis",
                "structure": "asymmetric",
                "resolution": [64, 64, 1],
                "count": int
            }
        }
    },
    "OPU": {
        "gui_name": "Actuators",
        "supported_devices": {
            "Motor": {
                "enabled": True,
                "cortical_id": "o__mot",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "count": int
            },
            "Servo": {
                "enabled": True,
                "cortical_id": "o__srv",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "count": int
            },
            "Navigation": {
                "enabled": True,
                "cortical_id": "o__nav",
                "structure": "asymmetric",
                "resolution": [3, 1, 20],
                "count": int
            },
            "Speed": {
                "enabled": True,
                "cortical_id": "o__spd",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "count": int
            },
            "Battery": {
                "enabled": True,
                "cortical_id": "o__bat",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "count": int
            },
            "Position_Initializer": {
                "enabled": True,
                "cortical_id": "o_init",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "count": int
            },
            "Misc": {
                "enabled": True,
                "cortical_id": "o__mot",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "count": int
            }
        }
    },
    "CORE": {
        "gui_name": "Core",
        "supported_devices": {
            "Death": {
                "enabled": True,
                "cortical_id": "_death",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "count": int
            },
            "Power": {
                "enabled": False,
                "cortical_id": "___pwr",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "count": int
            }
        }
    },
    "MEMORY": {
        "gui_name": "Memory"
    },
    "CUSTOM": {
        "gui_name": "Custom"
    }
}


cortical_template = {
    "per_voxel_neuron_cnt": 1,
    "synapse_attractivity": 100,
    "postsynaptic_current": 1,
    "plasticity_constant": 1,
    "degeneration": 0,
    "psp_uniform_distribution": True,
    "postsynaptic_current_max": 10,
    "cortical_mapping_dst": {},
    'firing_threshold': 1,
    "firing_threshold_increment": 0,
    "mp_charge_accumulation": True,
    "refractory_period": 0,
    "leak_coefficient": 0,
    "leak_variability": 0,
    "consecutive_fire_cnt_max": 0,
    "snooze_length": 0,
    "visualization": True
}
