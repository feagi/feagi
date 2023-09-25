
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
                "resolution": [1, 1, 1]
            },
            "inverse_infrared_sensor": {
                "enabled": True,
                "cortical_id": "i__inf",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "proximity_sensor": {
                "enabled": True,
                "cortical_id": "i__pro",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "accelerometer_sensor": {
                "enabled": True,
                "cortical_id": "i__acc",
                "structure": "symmetric",
                "resolution": [1, 1, 20]
            },
            "gyroscope_sensor": {
                "enabled": True,
                "cortical_id": "i__gyr",
                "structure": "symmetric",
                "resolution": [1, 1, 20]
            },
            "shock_sensor": {
                "enabled": True,
                "cortical_id": "ishock",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "battery_sensor": {
                "enabled": True,
                "cortical_id": "i__bat",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "vision_sensor": {
                "enabled": True,
                "cortical_id": "i__vis",
                "structure": "asymmetric",
                "resolution": [64, 64, 1]
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
                "resolution": [1, 1, 10]
            },
            "Servo": {
                "enabled": True,
                "cortical_id": "o__srv",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "Navigation": {
                "enabled": True,
                "cortical_id": "o__nav",
                "structure": "asymmetric",
                "resolution": [3, 1, 20]
            },
            "Speed": {
                "enabled": True,
                "cortical_id": "o__spd",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "Battery": {
                "enabled": True,
                "cortical_id": "o__bat",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "Position_Initializer": {
                "enabled": True,
                "cortical_id": "o_init",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "Misc": {
                "enabled": True,
                "cortical_id": "o__mot",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "Aperture": {
                "enabled": True,
                "cortical_id": "o_aptr",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "Vision_Resolution": {
                "enabled": True,
                "cortical_id": "o_vres",
                "structure": "asymmetric",
                "resolution": [2, 1, 10]
            },
            "Vision_Acuity": {
                "enabled": True,
                "cortical_id": "o_vact",
                "structure": "asymmetric",
                "resolution": [2, 1, 1]
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
                "resolution": [1, 1, 1]
            },
            "Power": {
                "enabled": False,
                "cortical_id": "___pwr",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
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
    "degeneration": 0,
    "psp_uniform_distribution": False,
    "postsynaptic_current_max": 99999,
    "plasticity_constant": 1,
    "cortical_mapping_dst": {},
    "firing_threshold_increment": 0,
    "visualization": True,
    "postsynaptic_current": 1,
    'firing_threshold': 1,
    "refractory_period": 0,
    "leak_coefficient": 0,
    "leak_variability": 0,
    "consecutive_fire_cnt_max": 0,
    "snooze_length": 0,
    "firing_threshold_increment_x": 0,
    "firing_threshold_increment_y": 0,
    "firing_threshold_increment_z": 0,
    "firing_threshold_limit": 0,
    "mp_charge_accumulation": True
}
