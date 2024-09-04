
# Copyright 2016-2022 Neuraville Inc. Authors. All Rights Reserved.
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
                "cortical_name": "Infrared sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "i_iinf": {
                "enabled": True,
                "cortical_name": "Infrared sensor (Inverse)",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "idgpio": {
                "enabled": True,
                "cortical_name": "Digital GPIO input",
                "structure": "asymmetric",
                "resolution": [28, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "iagpio": {
                "enabled": True,
                "cortical_name": "Analog GPIO input",
                "structure": "asymmetric",
                "resolution": [28, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "i__pro": {
                "enabled": True,
                "cortical_name": "Proximity sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [70, 0, 0]
            },
            "i__acc": {
                "enabled": True,
                "cortical_name": "Accelerometer sensor",
                "structure": "symmetric",
                "resolution": [3, 1, 21],
                "coordinate_3d": [40, 0, 0]
            },
            "i__gyr": {
                "enabled": True,
                "cortical_name": "Gyro sensor",
                "structure": "symmetric",
                "resolution": [3, 1, 21],
                "coordinate_3d": [50, 0, 0]
            },
            "ishock": {
                "enabled": True,
                "cortical_name": "Shock sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "i__bat": {
                "enabled": True,
                "cortical_name": "Battery gauge",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [60, 0, 0]
            },
            "iv00_C": {
                "enabled": True,
                "cortical_name": "Central vision sensor",
                "structure": "asymmetric",
                "resolution": [64, 64, 1],
                "coordinate_3d": [30, 40, -20]
            },
            "iv00TR": {
                "enabled": True,
                "cortical_name": "Peripheral vision sensor - top right",
                "structure": "asymmetric",
                "resolution": [8, 8, 1],
                "coordinate_3d": [95, 105, -20]
            },
            "iv00TL": {
                "enabled": True,
                "cortical_name": "Peripheral vision sensor - top left",
                "structure": "asymmetric",
                "resolution": [8, 8, 1],
                "coordinate_3d": [20, 105, -20]
            },
            "iv00TM": {
                "enabled": True,
                "cortical_name": "Peripheral vision sensor - top middle",
                "structure": "asymmetric",
                "resolution": [8, 8, 1],
                "coordinate_3d": [55, 105, -20]
            },
            "iv00ML": {
                "enabled": True,
                "cortical_name": "Peripheral vision sensor - middle left",
                "structure": "asymmetric",
                "resolution": [8, 8, 1],
                "coordinate_3d": [20, 70, -20]
            },
            "iv00MR": {
                "enabled": True,
                "cortical_name": "Peripheral vision sensor - middle right",
                "structure": "asymmetric",
                "resolution": [8, 8, 1],
                "coordinate_3d": [95, 70, -20]
            },
            "iv00BL": {
                "enabled": True,
                "cortical_name": "Peripheral vision sensor - bottom left",
                "structure": "asymmetric",
                "resolution": [8, 8, 1],
                "coordinate_3d": [20, 30, -20]
            },
            "iv00BR": {
                "enabled": True,
                "cortical_name": "Peripheral vision sensor - bottom right",
                "structure": "asymmetric",
                "resolution": [8, 8, 1],
                "coordinate_3d": [95, 30, -20]
            },
            "iv00BM": {
                "enabled": True,
                "cortical_name": "Peripheral vision sensor - bottom middle",
                "structure": "asymmetric",
                "resolution": [8, 8, 1],
                "coordinate_3d": [55, 30, -20]
            },
            "i___id": {
                "enabled": True,
                "cortical_name": "ID Trainer",
                "structure": "asymmetric",
                "resolution": [1, 10, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "i_spos": {
                "enabled": True,
                "cortical_name": "Servo position sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 21],
                "coordinate_3d": [20, 0, 0]
            },
            "i_smot": {
                "enabled": True,
                "cortical_name": "Servo motion sensor",
                "structure": "asymmetric",
                "resolution": [1, 1, 21],
                "coordinate_3d": [20, 0, 0]
            },
            "i__bci": {
                "enabled": True,
                "cortical_name": "BCI",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "i_misc": {
                "enabled": True,
                "cortical_name": "Miscellaneous Input",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
        },
        "name_to_id_mapping": {
            "infrared": [
                "i__inf"
            ],
            "infrared_inverse": [
                "ii_inf"
            ],
            "proximity": [
                "i__pro"
            ],
            "gpio_digital": [
                "idgpio"
            ],
            "gpio_analog": [
                "iagpio"
            ],
            "accelerometer": [
                "i__acc"
            ],
            "gyro": [
                "i__gyr"
            ],
            "shock": [
                "ishock"
            ],
            "battery": [
                "i__bat"
            ],
            "camera": [
                "iv00_C",
                "iv00TL",
                "iv00TM",
                "iv00TR",
                "iv00ML",
                "iv00MR",
                "iv00BL",
                "iv00BR",
                "iv00BM"
            ],
            "miscellaneous": [
                "i_misc"
            ],
            "servo": [
                "i_spos"
            ],
            "servo_motion": [
                "i_smot"
            ]
        }
    },
    "OPU": {
        "gui_name": "Actuators",
        "supported_devices": {
            "o__mot": {
                "enabled": True,
                "cortical_name": "Motor control",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [-30, 0, -20]
            },
            "odgpio": {
                "enabled": True,
                "cortical_name": "Digital GPIO output",
                "structure": "asymmetric",
                "resolution": [28, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "oagpio": {
                "enabled": True,
                "cortical_name": "Analog GPIO output",
                "structure": "asymmetric",
                "resolution": [28, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "oogpio": {
                "enabled": True,
                "cortical_name": "GPIO output pin assignment",
                "structure": "asymmetric",
                "resolution": [28, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "oigpio": {
                "enabled": True,
                "cortical_name": "GPIO input pin assignment",
                "structure": "asymmetric",
                "resolution": [28, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o__ser": {
                "enabled": True,
                "cortical_name": "Servo control",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [-50, 0, -20]
            },
            "o__nav": {
                "enabled": True,
                "cortical_name": "Navigation vector",
                "structure": "asymmetric",
                "resolution": [3, 1, 21],
                "coordinate_3d": [20, 0, 0]
            },
            "o__spd": {
                "enabled": True,
                "cortical_name": "Navigation speed",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "o__bat": {
                "enabled": True,
                "cortical_name": "Battery charger",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "o_init": {
                "enabled": True,
                "cortical_name": "Position initializer",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o_stop": {
                "enabled": True,
                "cortical_name": "Emergency stop",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o_spos": {
                "enabled": True,
                "cortical_name": "Servo Position OPU",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [-50, 0, -35]
            },
            "o_blnk": {
                "enabled": True,
                "cortical_name": "Blink",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o___id": {
                "enabled": True,
                "cortical_name": "ID Recognition",
                "structure": "asymmetric",
                "resolution": [1, 10, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o__loc": {
                "enabled": True,
                "cortical_name": "Recognition Location",
                "structure": "asymmetric",
                "resolution": [32, 32, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o_misc": {
                "enabled": True,
                "cortical_name": "Miscellaneous",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o_mctl": {
                "enabled": True,
                "cortical_name": "Motion control",
                "structure": "asymmetric",
                "resolution": [4, 3, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "ovflph": {
                "enabled": True,
                "cortical_name": "Vision horizontal flip",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "ovflpv": {
                "enabled": True,
                "cortical_name": "Vision vertical flip",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "ov_mod": {
                "enabled": True,
                "cortical_name": "Central Vision Modulation",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "ov_ecc": {
                "enabled": True,
                "cortical_name": "Central Vision Eccentricity",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "ov_enh": {
                "enabled": True,
                "cortical_name": "Lighting enhancement",
                "structure": "asymmetric",
                "resolution": [3, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "ovtune": {
                "enabled": True,
                "cortical_name": "Lighting threshold",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "o__led": {
                "enabled": True,
                "cortical_name": "LED",
                "structure": "asymmetric",
                "resolution": [2, 1, 1],
                "coordinate_3d": [-40, 0, -50]
            }
        },
        "name_to_id_mapping": {
            "gpio": [
                "odgpio",
                "oagpio",
                "oogpio",
                "oigpio",
            ],
            "motor": [
                "o__mot",
            ],
            "servo": [
                "o__ser",
                "o_spos",
            ],
            "navigation": [
                "o__nav",
                "o__spd",
                "o_init",
                "o_mctl",
                "o_stop",
            ],
            "battery": [
                "o__bat",
            ],
            "led": [
                "o__led"
            ],
            "camera": [
                "ov_mod",
                "ov_ecc",
                "o_blnk",
                "ov_enh",
                "ovtune",
                "ovflph",
                "ovflpv",
            ],
            "recognition": [
                "o___id",
                "o__loc",
            ],
            "motion_control": [
                "o_mctl"
            ],
            "miscellaneous": [
                "o_misc"
            ]

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
