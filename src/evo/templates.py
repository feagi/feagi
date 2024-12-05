
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
                "cortical_name": "Digital input",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [35, 20, -70]
            },
            "iagpio": {
                "enabled": True,
                "cortical_name": "Analog input",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [35, 0, -70]
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
                "coordinate_3d": [111, 91, 0]
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
            "i_pres": {
                "enabled": True,
                "cortical_name": "Pressure Sensor Input",
                "structure": "asymmetric",
                "resolution": [3, 1, 21],
                "coordinate_3d": [60, 0, 0]
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
            "servo_position": [
                "i_spos"
            ],
            "servo_motion": [
                "i_smot"
            ],
            "digital_input": [
                "idgpio"
            ],
            "analog_input": [
                "iagpio"
            ],
            "id_trainer": [
                "i___id"
            ],
            "pressure": [
                "i_pres"
            ]

        }
    },
    "OPU": {
        "gui_name": "Actuators",
        "supported_devices": {
            "o__mot": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Motor control",
                "controller_id": "motor",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [-30, 0, -20],
            },
            "odgpio": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Digital output",
                "controller_id": "gpio",
                "structure": "asymmetric",
                "resolution": [28, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "oagpio": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Analog output",
                "controller_id": "gpio",
                "structure": "asymmetric",
                "resolution": [28, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "oogpio": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "GPIO output pin assignment",
                "controller_id": "gpio",
                "structure": "asymmetric",
                "resolution": [28, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "oigpio": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "GPIO input pin assignment",
                "controller_id": "gpio_input",
                "structure": "asymmetric",
                "resolution": [28, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o__nav": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Navigation vector",
                "controller_id": "navigation",
                "structure": "asymmetric",
                "resolution": [3, 1, 21],
                "coordinate_3d": [20, 0, 0]
            },
            "o__spd": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Navigation speed",
                "controller_id": "speed",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "o__bat": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Battery charger",
                "controller_id": "battery",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "o_init": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Position initializer",
                "controller_id": "reset",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o_stop": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Emergency stop",
                "controller_id": "emergency",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o__ser": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Servo control",
                "controller_id": "servo",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [-50, 0, -20]
            },
            "o_spos": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Servo Position OPU",
                "controller_id": "servo_position",
                "structure": "asymmetric",
                "resolution": [1, 1, 10],
                "coordinate_3d": [-50, 0, -35]
            },
            "o_blnk": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Blink",
                "controller_id": "blink",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o___id": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "ID Recognition",
                "controller_id": "id_recognition",
                "structure": "asymmetric",
                "resolution": [1, 10, 1],
                "coordinate_3d": [120, 91, 0]
            },
            "o__sid": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Trainer Stimuli ID Selector",
                "controller_id": "stimuli_id_selector",
                "structure": "asymmetric",
                "resolution": [1, 10, 1],
                "coordinate_3d": [30, 0, 0]
            },
            "o__loc": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Recognition Location",
                "controller_id": "recognition_location",
                "structure": "asymmetric",
                "resolution": [32, 32, 1],
                "coordinate_3d": [110, 51, 0]
            },
            "o_misc": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Miscellaneous",
                "controller_id": "misc",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "o_mctl": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Motion control",
                "controller_id": "motion_control",
                "structure": "asymmetric",
                "resolution": [4, 3, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "ovflph": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Vision horizontal flip",
                "controller_id": "horizontal_flip",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "ovflpv": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Vision vertical flip",
                "controller_id": "vertical_flip",
                "structure": "asymmetric",
                "resolution": [1, 1, 1],
                "coordinate_3d": [20, 0, 0]
            },
            "ov_mod": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Central vision modulation",
                "controller_id": "modulation_control",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "ov_ecc": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Central vision eccentricity",
                "controller_id": "eccentricity_control",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "ov_enh": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Lighting enhancement",
                "controller_id": "enhancement",
                "structure": "asymmetric",
                "resolution": [3, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "ovtune": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "Lighting threshold",
                "controller_id": "threshold",
                "structure": "asymmetric",
                "resolution": [2, 1, 10],
                "coordinate_3d": [20, 0, 0]
            },
            "o__led": {
                "enabled": True,
                "measurable": True,
                "cortical_name": "LED",
                "controller_id": "led",
                "structure": "asymmetric",
                "resolution": [2, 1, 1],
                "coordinate_3d": [-40, 0, -50]
            },
            "ov_reg": {
                "enabled": True,
                "measurable": False,
                "cortical_name": "Vision activation regions",
                "controller_id": "activation_regions",
                "structure": "asymmetric",
                "resolution": [3, 3, 1],
                "coordinate_3d": [-20, 0, -50]
            }
        },
        "name_to_id_mapping": {
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
            "misc": [
                "o_misc"
            ],
            "digital_output": [
                "odgpio",
                "oogpio",
                "oigpio",
            ],
            "analog_output": [
                "oagpio"
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
    "psp_uniform_distribution": True,
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
    "mp_charge_accumulation": False,
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
    "projector_xy": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "projector_xz": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "projector_yz": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "project_from_end_x": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "project_from_end_y": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "project_from_end_z": {
        "parameters": {},
        "type": "functions",
        "class": "core"
    },
    "last_to_first": {
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
