
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
            "i__inf": {
                "enabled": True,
                "cortical_name": "infrared_sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "i_iinf": {
                "enabled": True,
                "cortical_name": "inverse_infrared_sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "i__pro": {
                "enabled": True,
                "cortical_name": "proximity_sensor ",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "i__acc": {
                "enabled": True,
                "cortical_name": "accelerometer_sensor",
                "structure": "symmetric",
                "resolution": [1, 1, 20]
            },
            "i__gyr": {
                "enabled": True,
                "cortical_name": "gyroscope_sensor",
                "structure": "symmetric",
                "resolution": [1, 1, 20]
            },
            "ishock": {
                "enabled": True,
                "cortical_name": "shock_sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "i__bat": {
                "enabled": True,
                "cortical_name": "battery_sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "iv00_C": {
                "enabled": True,
                "cortical_name": "central_vision_sensor",
                "structure": "asymmetric",
                "resolution": [64, 64, 3]
            },
            "iv00TR": {
                "enabled": True,
                "cortical_name": "top_right_peripheral_vision_sensor",
                "structure": "asymmetric",
                "resolution": [8, 8, 3]
            },
            "iv00TL": {
                "enabled": True,
                "cortical_name": "top_left_peripheral_vision_sensor",
                "structure": "asymmetric",
                "resolution": [8, 8, 3]
            },
            "iv00TM": {
                "enabled": True,
                "cortical_name": "top_middle_peripheral_vision_sensor",
                "structure": "asymmetric",
                "resolution": [8, 8, 3]
            },
            "iv00ML": {
                "enabled": True,
                "cortical_name": "middle_left_peripheral_vision_sensor",
                "structure": "asymmetric",
                "resolution": [8, 8, 3]
            },
            "iv00MR": {
                "enabled": True,
                "cortical_name": "middle_right_peripheral_vision_sensor",
                "structure": "asymmetric",
                "resolution": [8, 8, 3]
            },
            "iv00LL": {
                "enabled": True,
                "cortical_name": "lower_left_peripheral_vision_sensor",
                "structure": "asymmetric",
                "resolution": [8, 8, 3]
            },
            "iv00LR": {
                "enabled": True,
                "cortical_name": "lower_right_peripheral_vision_sensor",
                "structure": "asymmetric",
                "resolution": [8, 8, 3]
            },
            "iv00LM": {
                "enabled": True,
                "cortical_name": "lower_middle_peripheral_vision_sensor",
                "structure": "asymmetric",
                "resolution": [8, 8, 3]
            },
            "i___id": {
                "enabled": True,
                "cortical_name": "ID_Trainer",
                "structure": "asymmetric",
                "resolution": [1, 10, 1]
            },
            "i_spos": {
                "enabled": True,
                "cortical_name": "servo_position_ipu",
                "structure": "asymmetric",
                "resolution": [6, 1, 20]
            }
        }
    },
    "OPU": {
        "gui_name": "Actuators",
        "supported_devices": {
            "o__mot": {
                "enabled": True,
                "cortical_name": "motor_opu",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "o__ser": {
                "enabled": True,
                "cortical_name": "servo_opu",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "o__nav": {
                "enabled": True,
                "cortical_name": "Navigation",
                "structure": "asymmetric",
                "resolution": [3, 1, 20]
            },
            "o__spd": {
                "enabled": True,
                "cortical_name": "Speed",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "o__bat": {
                "enabled": True,
                "cortical_name": "Battery",
                "structure": "asymmetric",
                "resolution": [1, 1, 10]
            },
            "o_init": {
                "enabled": True,
                "cortical_name": "Position_Initializer",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "o__pup": {
                "enabled": True,
                "cortical_name": "Vision_Pupil",
                "structure": "asymmetric",
                "resolution": [2, 1, 1]
            },
            "o__gaz": {
                "enabled": True,
                "cortical_name": "Vision_Gaze",
                "structure": "asymmetric",
                "resolution": [2, 1, 1]
            },
            "o_blnk": {
                "enabled": True,
                "cortical_name": "Visual_Blink",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "o___id": {
                "enabled": True,
                "cortical_name": "ID_Recognition",
                "structure": "asymmetric",
                "resolution": [1, 10, 1]
            },
            "ov_enh": {
                "enabled": True,
                "cortical_name": "enhancement",
                "structure": "asymmetric",
                "resolution": [3, 1, 10]
            },
            "ovtune": {
                "enabled": True,
                "cortical_name": "threshold",
                "structure": "asymmetric",
                "resolution": [2, 1, 10]
            },
            "ov_thr": {
                "enabled": True,
                "cortical_name": "threshold_type",
                "structure": "asymmetric",
                "resolution": [4, 1, 1]
            },
            "o_misc": {
                "enabled": True,
                "cortical_name": "Misc",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "o_mctl": {
                "enabled": True,
                "cortical_name": "motion_control_opu",
                "structure": "asymmetric",
                "resolution": [4, 1, 1]
            },
            "ovflph": {
                "enabled": True,
                "cortical_name": "vision_horizontal_flip",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "ovflpv": {
                "enabled": True,
                "cortical_name": "vision_vertical_flip",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            },
            "o_stop": {
                "enabled": True,
                "cortical_name": "emergency_stop",
                "structure": "asymmetric",
                "resolution": [1, 1, 1]
            }
        }
    },
    "CORE": {
        "gui_name": "Core",
        "supported_devices": {
            "_death": {
                "enabled": True,
                "cortical_name": "Death",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [0, 0, -10],
                "coordinate_2d": [-10, -20]
            },
            "___pwr": {
                "enabled": False,
                "cortical_name": "Power",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [0, 0, -20],
                "coordinate_2d": [-10, -10]
            }
        }
    },
    "CUSTOM": {
        "gui_name": "Custom"
    }
}


cortical_template = {
    "sub_group_id": "",
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
    "mp_charge_accumulation": True,
    "mp_driven_psp": False,
    "is_mem_type": False,
    "longterm_mem_threshold": 100,
    "lifespan_growth_rate": 1,
    "init_lifespan": 9,
    "neuron_excitability": 100
}


core_morphologies = {
    "block_to_block": {
        "parameters": {
            "vectors": [
                [
                    0,
                    0,
                    0
                ]
            ]
        },
        "type": "vectors",
        "class": "core"
    },
    "projector": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "memory": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "0-0-0_to_all": {
        "type": "patterns",
        "parameters": {
            "patterns": [
                [
                    [
                        0,
                        0,
                        0
                    ],
                    [
                        "*",
                        "*",
                        "*"
                    ]
                ]
            ]
        },
        "class": "core"
    },
    "all_to_0-0-0": {
        "type": "patterns",
        "parameters": {
            "patterns": [
                [
                    [
                        "*",
                        "*",
                        "*"
                    ],
                    [
                        0,
                        0,
                        0
                    ]
                ]
            ]
        },
        "class": "core"
    },
    "all_to_all": {
        "type": "patterns",
        "parameters": {
            "patterns": [
                [
                    [
                        "?",
                        "?",
                        "?"
                    ],
                    [
                        "*",
                        "*",
                        "*"
                    ]
                ]
            ]
        },
        "class": "core"
    },
    "lateral_+x": {
        "parameters": {
            "vectors": [
                [
                    1,
                    0,
                    0
                ]
            ]
        },
        "type": "vectors",
        "class": "core"
    },
    "lateral_-x": {
        "parameters": {
            "vectors": [
                [
                    -1,
                    0,
                    0
                ]
            ]
        },
        "type": "vectors",
        "class": "core"
    },
    "lateral_+y": {
        "parameters": {
            "vectors": [
                [
                    0,
                    1,
                    0
                ]
            ]
        },
        "type": "vectors",
        "class": "core"
    },
    "lateral_-y": {
        "parameters": {
            "vectors": [
                [
                    0,
                    -1,
                    0
                ]
            ]
        },
        "type": "vectors",
        "class": "core"
    },
    "lateral_+z": {
        "parameters": {
            "vectors": [
                [
                    0,
                    0,
                    1
                ]
            ]
        },
        "type": "vectors",
        "class": "core"
    },
    "lateral_-z": {
        "parameters": {
            "vectors": [
                [
                    0,
                    0,
                    -1
                ]
            ]
        },
        "type": "vectors",
        "class": "core"
    },
    "randomizer": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "expander_x": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "reducer_x": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "lateral_pairs_x": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "tile": {
        "parameters": {
            "src_seed": [
                16,
                16,
                1
            ],
            "src_pattern": [
                [
                    1,
                    0
                ],
                [
                    1,
                    0
                ],
                [
                    1,
                    0
                ]
            ],
            "mapper_morphology": "projector"
        },
        "type": "composite",
        "class": "core"
    }
}
