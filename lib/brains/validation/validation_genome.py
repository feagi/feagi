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

genome = {
    "version": "2.0",
    "max_burst_count": 3,
    "burst_delay": 1,
    "evolution_burst_count": 50,
    "ipu_idle_threshold": 1000,
    "neuron_morphologies": {
        "placeholder": {},
        "block_to_block": {
            "vectors": [[0, 0, 0]]
        },
        "decrease_filter_diagonal": {
            "vectors": [[0, 1, 1]]
        },
        "decrease_filter_diagonal2": {
            "vectors": [[0, 2, 1]]
        },
        "increase_filter_diagonal": {
            "vectors": [[0, 1, -1]]
        },
        "increase_filter_diagonal2": {
            "vectors": [[0, 2, -1]]
        },
        "y_consolidator": {
            "patterns": [["*", "*", "*"], ["*", "?", "*"]]
        },
        "lateral_+x": {
            "vectors": [[1, 0, 0]]
        },
        "lateral_-x": {
            "vectors": [[-1, 0, 0]]
        },
        "lateral_+y": {
            "vectors": [[0, 1, 0]]
        },
        "lateral_-y": {
            "vectors": [[0, -1, 0]]
        },
        "lateral_+z": {
            "vectors": [[0, 0, 1]]
        },
        "lateral_-z": {
            "vectors": [[0, 0, -1]]
        },
        "one_to_all": {
            "patterns": [[1, 1, 1], ['*', '*', '*']]
        },
        "all_to_one": {
            "patterns": [['*', '*', '*'], [1, 1, 1]]
        },
        "to_block_[5, 7, 4]": {
            "patterns": [["*", "*", "*"], [5, 7, 4]]
        },
        "expander_x": {
            "functions": True
        },
        "reducer_x": {
            "functions": True
        },
        "randomizer": {
            "functions": True
        },
        "lateral_pairs_x": {
            "functions": True
        }
    },
    "species": {
        "parents": {},
        "species_id": "",
        "class": "toy",
        "brand": "gazebo",
        "model": "smart_car"
    },
    "blueprint": {
        "_____10b-_____s-__-__name-t": "smart_car",

        # +++ Plasticity Test +++
        "_____10c-p____1-cx-__name-t": "plast_a",
        "_____10c-p____1-cx-_group-t": "IPU",
        "_____10c-p____1-cx-_n_cnt-i": 1,
        "_____10c-p____1-nx-gd_vis-b": True,
        "_____10c-p____1-nx-rcordx-i": -50,
        "_____10c-p____1-nx-rcordy-i": 0,
        "_____10c-p____1-nx-rcordz-i": 0,
        "_____10c-p____1-nx-___bbx-i": 1,
        "_____10c-p____1-nx-___bby-i": 1,
        "_____10c-p____1-nx-___bbz-i": 1,
        "_____10c-p____1-cx-synatt-i": 100,
        "_____10c-p____1-cx-pstcr_-f": 5,
        "_____10c-p____1-cx-pstcrm-f": 35,
        "_____10c-p____1-cx-plst_c-f": 0.05,
        "_____10c-p____1-nx-fire_t-f": 1,
        "_____10c-p____1-nx-refrac-i": 0,
        "_____10c-p____1-nx-leak_c-f": 10,
        "_____10c-p____1-nx-c_fr_c-i": 3,
        "_____10c-p____1-nx-snooze-f": 0,
        "_____10c-p____1-cx-de_gen-f": 0,
        "_____10c-p____1-cx-dstmap-d": {
            "p____2": [["block_to_block", [1, 1, 1], 1, False]]
        },

        "_____10c-p____2-cx-__name-t": "plast_b",
        "_____10c-p____2-cx-_group-t": "IPU",
        "_____10c-p____2-cx-_n_cnt-i": 1,
        "_____10c-p____2-nx-gd_vis-b": True,
        "_____10c-p____2-nx-rcordx-i": -55,
        "_____10c-p____2-nx-rcordy-i": 10,
        "_____10c-p____2-nx-rcordz-i": 0,
        "_____10c-p____2-nx-___bbx-i": 1,
        "_____10c-p____2-nx-___bby-i": 1,
        "_____10c-p____2-nx-___bbz-i": 1,
        "_____10c-p____2-cx-synatt-i": 100,
        "_____10c-p____2-cx-pstcr_-f": 5.8,
        "_____10c-p____2-cx-pstcrm-f": 35,
        "_____10c-p____2-cx-plst_c-f": 0.05,
        "_____10c-p____2-nx-fire_t-f": 1,
        "_____10c-p____2-nx-refrac-i": 0,
        "_____10c-p____2-nx-leak_c-f": 10,
        "_____10c-p____2-nx-c_fr_c-i": 3,
        "_____10c-p____2-nx-snooze-f": 0,
        "_____10c-p____2-cx-de_gen-f": 0,
        "_____10c-p____2-cx-dstmap-d": {
        },

        "_____10c-p____3-cx-__name-t": "plast_c",
        "_____10c-p____3-cx-_group-t": "IPU",
        "_____10c-p____3-cx-_n_cnt-i": 1,
        "_____10c-p____3-nx-gd_vis-b": True,
        "_____10c-p____3-nx-rcordx-i": -45,
        "_____10c-p____3-nx-rcordy-i": 10,
        "_____10c-p____3-nx-rcordz-i": 0,
        "_____10c-p____3-nx-___bbx-i": 1,
        "_____10c-p____3-nx-___bby-i": 1,
        "_____10c-p____3-nx-___bbz-i": 1,
        "_____10c-p____3-cx-synatt-i": 100,
        "_____10c-p____3-cx-pstcr_-f": 2.4,
        "_____10c-p____3-cx-pstcrm-f": 15,
        "_____10c-p____3-cx-plst_c-f": 3,
        "_____10c-p____3-nx-fire_t-f": 1,
        "_____10c-p____3-nx-refrac-i": 0,
        "_____10c-p____3-nx-leak_c-f": 10,
        "_____10c-p____3-nx-c_fr_c-i": 3,
        "_____10c-p____3-nx-snooze-f": 0,
        "_____10c-p____3-cx-de_gen-f": 0,
        "_____10c-p____3-cx-dstmap-d": {
            "p____2": [["block_to_block", [1, 1, 1], 3, True]]
        },



        # +++ Misc +++

        # +++ Thalami +++

        # vvvvvvv   Input and Output processing related Cortical Areas are all listed below  vvvvvvvv
        #
        # # +++ Input Processing Units (IPU) +++
        # "_____10c-i__inf-cx-__name-t": "ir_ipu",
        # "_____10c-i__inf-cx-_group-t": "IPU",
        # "_____10c-i__inf-cx-_n_cnt-i": 1,
        # "_____10c-i__inf-nx-gd_vis-b": True,
        # "_____10c-i__inf-nx-rcordx-i": 80,
        # "_____10c-i__inf-nx-rcordy-i": 0,
        # "_____10c-i__inf-nx-rcordz-i": 0,
        # "_____10c-i__inf-nx-___bbx-i": 3,
        # "_____10c-i__inf-nx-___bby-i": 1,
        # "_____10c-i__inf-nx-___bbz-i": 1,
        # "_____10c-i__inf-cx-synatt-i": 100,
        # "_____10c-i__inf-cx-pstcr_-f": 5,
        # "_____10c-i__inf-cx-pstcrm-f": 35,
        # "_____10c-i__inf-cx-plst_c-f": 0.05,
        # "_____10c-i__inf-nx-fire_t-f": 1,
        # "_____10c-i__inf-nx-refrac-i": 0,
        # "_____10c-i__inf-nx-leak_c-f": 10,
        # "_____10c-i__inf-nx-c_fr_c-i": 3,
        # "_____10c-i__inf-nx-snooze-f": 0,
        # "_____10c-i__inf-cx-de_gen-f": 0,
        # "_____10c-i__inf-cx-dstmap-d": {
        #     "t__inf": [["block_to_block", [1, 1, 1], 1, False]]
        # },
        #
        # "_____10c-ishock-cx-__name-t": "shock_ipu",
        # "_____10c-ishock-cx-_group-t": "IPU",
        # "_____10c-ishock-cx-_n_cnt-i": 1,
        # "_____10c-ishock-nx-gd_vis-b": True,
        # "_____10c-ishock-nx-rcordx-i": 10,
        # "_____10c-ishock-nx-rcordy-i": 0,
        # "_____10c-ishock-nx-rcordz-i": 35,
        # "_____10c-ishock-nx-___bbx-i": 1,
        # "_____10c-ishock-nx-___bby-i": 1,
        # "_____10c-ishock-nx-___bbz-i": 10,
        # "_____10c-ishock-cx-synatt-i": 100,
        # "_____10c-ishock-cx-pstcr_-f": 5,
        # "_____10c-ishock-cx-pstcrm-f": 35,
        # "_____10c-ishock-cx-plst_c-f": 0.05,
        # "_____10c-ishock-nx-fire_t-f": 1,
        # "_____10c-ishock-nx-refrac-i": 0,
        # "_____10c-ishock-nx-leak_c-f": 10,
        # "_____10c-ishock-nx-c_fr_c-i": 3,
        # "_____10c-ishock-nx-snooze-f": 0,
        # "_____10c-ishock-cx-de_gen-f": 0,
        # "_____10c-ishock-cx-dstmap-d": {
        # },
        #
        # "_____10c-i__pro-cx-__name-t": "proximity_ipu",
        # "_____10c-i__pro-cx-_group-t": "memory",
        # "_____10c-i__pro-cx-_n_cnt-i": 1,
        # "_____10c-i__pro-nx-gd_vis-b": True,
        # "_____10c-i__pro-nx-rcordx-i": 70,
        # "_____10c-i__pro-nx-rcordy-i": 0,
        # "_____10c-i__pro-nx-rcordz-i": 0,
        # "_____10c-i__pro-nx-___bbx-i": 1,
        # "_____10c-i__pro-nx-___bby-i": 1,
        # "_____10c-i__pro-nx-___bbz-i": 10,
        # "_____10c-i__pro-cx-synatt-i": 100,
        # "_____10c-i__pro-cx-pstcr_-f": 5,
        # "_____10c-i__pro-cx-pstcrm-f": 35,
        # "_____10c-i__pro-cx-plst_c-f": 0.05,
        # "_____10c-i__pro-nx-fire_t-f": 1,
        # "_____10c-i__pro-nx-refrac-i": 0,
        # "_____10c-i__pro-nx-leak_c-f": 10,
        # "_____10c-i__pro-nx-c_fr_c-i": 1,
        # "_____10c-i__pro-nx-snooze-f": 0,
        # "_____10c-i__pro-cx-de_gen-f": 0,
        # "_____10c-i__pro-cx-dstmap-d": {
        #     "t__pro": [["randomizer", [1, 1, 1], 1, False]],
        #     "t__inf": [["randomizer", [1, 1, 1], 5, True]]
        # },
        #
        #
        # "_____10c-i__bat-cx-__name-t": "battery_ipu",
        # "_____10c-i__bat-cx-_group-t": "IPU",
        # "_____10c-i__bat-cx-_n_cnt-i": 1,
        # "_____10c-i__bat-nx-gd_vis-b": True,
        # "_____10c-i__bat-nx-rcordx-i": 50,
        # "_____10c-i__bat-nx-rcordy-i": 0,
        # "_____10c-i__bat-nx-rcordz-i": 0,
        # "_____10c-i__bat-nx-___bbx-i": 1,
        # "_____10c-i__bat-nx-___bby-i": 1,
        # "_____10c-i__bat-nx-___bbz-i": 10,
        # "_____10c-i__bat-cx-synatt-i": 100,
        # "_____10c-i__bat-cx-pstcr_-f": 5,
        # "_____10c-i__bat-cx-pstcrm-f": 35,
        # "_____10c-i__bat-cx-plst_c-f": 0.05,
        # "_____10c-i__bat-nx-fire_t-f": 1,
        # "_____10c-i__bat-nx-refrac-i": 0,
        # "_____10c-i__bat-nx-leak_c-f": 0,
        # "_____10c-i__bat-nx-c_fr_c-i": 3,
        # "_____10c-i__bat-nx-snooze-f": 0,
        # "_____10c-i__bat-cx-de_gen-f": 0,
        # "_____10c-i__bat-cx-dstmap-d": {},
        #
        # # +++ Middle Layers +++
        #
        # "_____10c-t__inf-cx-__name-t": "ir_thalamus",
        # "_____10c-t__inf-cx-_group-t": "Thalamic",
        # "_____10c-t__inf-cx-_n_cnt-i": 1,
        # "_____10c-t__inf-nx-gd_vis-b": True,
        # "_____10c-t__inf-nx-rcordx-i": 80,
        # "_____10c-t__inf-nx-rcordy-i": 10,
        # "_____10c-t__inf-nx-rcordz-i": 0,
        # "_____10c-t__inf-nx-___bbx-i": 3,
        # "_____10c-t__inf-nx-___bby-i": 1,
        # "_____10c-t__inf-nx-___bbz-i": 1,
        # "_____10c-t__inf-cx-synatt-i": 100,
        # "_____10c-t__inf-cx-pstcr_-f": 5,
        # "_____10c-t__inf-cx-pstcrm-f": 35,
        # "_____10c-t__inf-cx-plst_c-f": 0.1,
        # "_____10c-t__inf-nx-fire_t-f": 20,
        # "_____10c-t__inf-nx-refrac-i": 1,
        # "_____10c-t__inf-nx-leak_c-f": 0,
        # "_____10c-t__inf-nx-c_fr_c-i": 3,
        # "_____10c-t__inf-nx-snooze-f": 0,
        # "_____10c-t__inf-cx-de_gen-f": 0,
        # "_____10c-t__inf-cx-dstmap-d": {
        #     "m__inf": [["block_to_block", [1, 1, 1], 1, True]]
        # },
        #
        # "_____10c-m__inf-cx-__name-t": "ir_memory",
        # "_____10c-m__inf-cx-_group-t": "Memory",
        # "_____10c-m__inf-cx-_n_cnt-i": 1,
        # "_____10c-m__inf-nx-gd_vis-b": True,
        # "_____10c-m__inf-nx-rcordx-i": 80,
        # "_____10c-m__inf-nx-rcordy-i": 20,
        # "_____10c-m__inf-nx-rcordz-i": 0,
        # "_____10c-m__inf-nx-___bbx-i": 3,
        # "_____10c-m__inf-nx-___bby-i": 1,
        # "_____10c-m__inf-nx-___bbz-i": 1,
        # "_____10c-m__inf-cx-synatt-i": 100,
        # "_____10c-m__inf-cx-pstcr_-f": 5,
        # "_____10c-m__inf-cx-pstcrm-f": 35,
        # "_____10c-m__inf-cx-plst_c-f": 0.05,
        # "_____10c-m__inf-nx-fire_t-f": 100,
        # "_____10c-m__inf-nx-refrac-i": 0,
        # "_____10c-m__inf-nx-leak_c-f": 0,
        # "_____10c-m__inf-nx-c_fr_c-i": 3,
        # "_____10c-m__inf-nx-snooze-f": 0,
        # "_____10c-m__inf-cx-de_gen-f": 0,
        # "_____10c-m__inf-cx-dstmap-d": {
        #     # "m__inf": [["block_to_block", [1, 1, 1], 1, False]]
        # },
        #
        # "_____10c-t__pro-cx-__name-t": "proximity_thalamus",
        # "_____10c-t__pro-cx-_group-t": "Thalamic",
        # "_____10c-t__pro-cx-_n_cnt-i": 1,
        # "_____10c-t__pro-nx-gd_vis-b": True,
        # "_____10c-t__pro-nx-rcordx-i": 70,
        # "_____10c-t__pro-nx-rcordy-i": 10,
        # "_____10c-t__pro-nx-rcordz-i": 0,
        # "_____10c-t__pro-nx-___bbx-i": 1,
        # "_____10c-t__pro-nx-___bby-i": 1,
        # "_____10c-t__pro-nx-___bbz-i": 10,
        # "_____10c-t__pro-cx-synatt-i": 20,
        # "_____10c-t__pro-cx-pstcr_-f": 5,
        # "_____10c-t__pro-cx-pstcrm-f": 35,
        # "_____10c-t__pro-cx-plst_c-f": 0.05,
        # "_____10c-t__pro-nx-fire_t-f": 100,
        # "_____10c-t__pro-nx-refrac-i": 0,
        # "_____10c-t__pro-nx-leak_c-f": 0,
        # "_____10c-t__pro-nx-c_fr_c-i": 0,
        # "_____10c-t__pro-nx-snooze-f": 0,
        # "_____10c-t__pro-cx-de_gen-f": 0,
        # "_____10c-t__pro-cx-pspuni-b": True,
        # "_____10c-t__pro-cx-dstmap-d": {
        #     "m__pro": [["block_to_block", [1, 1, 1], 1, False]]
        # },
        #
        # "_____10c-m__pro-cx-__name-t": "proximity_memory",
        # "_____10c-m__pro-cx-_group-t": "memory",
        # "_____10c-m__pro-cx-_n_cnt-i": 1,
        # "_____10c-m__pro-nx-gd_vis-b": True,
        # "_____10c-m__pro-nx-rcordx-i": 70,
        # "_____10c-m__pro-nx-rcordy-i": 20,
        # "_____10c-m__pro-nx-rcordz-i": 0,
        # "_____10c-m__pro-nx-___bbx-i": 1,
        # "_____10c-m__pro-nx-___bby-i": 1,
        # "_____10c-m__pro-nx-___bbz-i": 10,
        # "_____10c-m__pro-cx-synatt-i": 20,
        # "_____10c-m__pro-cx-pstcr_-f": 5,
        # "_____10c-m__pro-cx-pstcrm-f": 35,
        # "_____10c-m__pro-cx-plst_c-f": 0.05,
        # "_____10c-m__pro-nx-fire_t-f": 1,
        # "_____10c-m__pro-nx-refrac-i": 0,
        # "_____10c-m__pro-nx-leak_c-f": 0,
        # "_____10c-m__pro-nx-c_fr_c-i": 0,
        # "_____10c-m__pro-nx-snooze-f": 0,
        # "_____10c-m__pro-cx-de_gen-f": 0,
        # "_____10c-m__pro-cx-pspuni-b": True,
        # "_____10c-m__pro-cx-dstmap-d": {
        #     "m__pro": [["randomizer", [1, 1, 1], 1, True]],
        #     "o__mot": [["randomizer", [1, 1, 1], 1, True]],
        # },
        #
        #
        #
        # # +++ Output Processing Units (OPU) +++
        # "_____10c-o__bat-cx-__name-t": "battery_opu",
        # "_____10c-o__bat-cx-_group-t": "OPU",
        # "_____10c-o__bat-cx-_n_cnt-i": 1,
        # "_____10c-o__bat-nx-gd_vis-b": True,
        # "_____10c-o__bat-nx-rcordx-i": 60,
        # "_____10c-o__bat-nx-rcordy-i": 0,
        # "_____10c-o__bat-nx-rcordz-i": 0,
        # "_____10c-o__bat-nx-___bbx-i": 1,
        # "_____10c-o__bat-nx-___bby-i": 1,
        # "_____10c-o__bat-nx-___bbz-i": 10,
        # "_____10c-o__bat-cx-synatt-i": 100,
        # "_____10c-o__bat-cx-pstcr_-f": 5,
        # "_____10c-o__bat-cx-pstcrm-f": 35,
        # "_____10c-o__bat-cx-plst_c-f": 0.05,
        # "_____10c-o__bat-nx-fire_t-f": 1,
        # "_____10c-o__bat-nx-refrac-i": 0,
        # "_____10c-o__bat-nx-leak_c-f": 0,
        # "_____10c-o__bat-nx-c_fr_c-i": 3,
        # "_____10c-o__bat-nx-snooze-f": 0,
        # "_____10c-o__bat-cx-de_gen-f": 0,
        # "_____10c-o__bat-cx-dstmap-d": {},
        #
        # "_____10c-o_init-cx-__name-t": "position_init_opu",
        # "_____10c-o_init-cx-_group-t": "OPU",
        # "_____10c-o_init-cx-_n_cnt-i": 1,
        # "_____10c-o_init-nx-gd_vis-b": True,
        # "_____10c-o_init-nx-rcordx-i": 50,
        # "_____10c-o_init-nx-rcordy-i": 10,
        # "_____10c-o_init-nx-rcordz-i": 0,
        # "_____10c-o_init-nx-___bbx-i": 1,
        # "_____10c-o_init-nx-___bby-i": 1,
        # "_____10c-o_init-nx-___bbz-i": 1,
        # "_____10c-o_init-cx-synatt-i": 100,
        # "_____10c-o_init-cx-pstcr_-f": 5,
        # "_____10c-o_init-cx-pstcrm-f": 35,
        # "_____10c-o_init-cx-plst_c-f": 0.05,
        # "_____10c-o_init-nx-fire_t-f": 1,
        # "_____10c-o_init-nx-refrac-i": 0,
        # "_____10c-o_init-nx-leak_c-f": 0,
        # "_____10c-o_init-nx-c_fr_c-i": 3,
        # "_____10c-o_init-nx-snooze-f": 0,
        # "_____10c-o_init-cx-de_gen-f": 0,
        # "_____10c-o_init-cx-dstmap-d": {},
        #
        # "_____10c-o__mot-cx-__name-t": "motor_opu",
        # "_____10c-o__mot-cx-_group-t": "OPU",
        # "_____10c-o__mot-cx-_n_cnt-i": 1,
        # "_____10c-o__mot-nx-gd_vis-b": True,
        # "_____10c-o__mot-nx-rcordx-i": 20,
        # "_____10c-o__mot-nx-rcordy-i": 0,
        # "_____10c-o__mot-nx-rcordz-i": 0,
        # "_____10c-o__mot-nx-___bbx-i": 8,
        # "_____10c-o__mot-nx-___bby-i": 1,
        # "_____10c-o__mot-nx-___bbz-i": 10,
        # "_____10c-o__mot-cx-synatt-i": 100,
        # "_____10c-o__mot-cx-pstcr_-f": 6,
        # "_____10c-o__mot-cx-pstcrm-f": 35,
        # "_____10c-o__mot-cx-plst_c-f": 0.05,
        # "_____10c-o__mot-nx-fire_t-f": 1,
        # "_____10c-o__mot-nx-refrac-i": 0,
        # "_____10c-o__mot-nx-leak_c-f": 0,
        # "_____10c-o__mot-nx-c_fr_c-i": 3,
        # "_____10c-o__mot-nx-snooze-f": 0,
        # "_____10c-o__mot-cx-de_gen-f": 0.01,
        # "_____10c-o__mot-cx-dstmap-d": {},
        #
        # "_____10c-o__ser-cx-__name-t": "servo_opu",
        # "_____10c-o__ser-cx-_group-t": "OPU",
        # "_____10c-o__ser-cx-_n_cnt-i": 1,
        # "_____10c-o__ser-nx-gd_vis-b": True,
        # "_____10c-o__ser-nx-rcordx-i": 35,
        # "_____10c-o__ser-nx-rcordy-i": 0,
        # "_____10c-o__ser-nx-rcordz-i": 0,
        # "_____10c-o__ser-nx-___bbx-i": 4,
        # "_____10c-o__ser-nx-___bby-i": 1,
        # "_____10c-o__ser-nx-___bbz-i": 10,
        # "_____10c-o__ser-cx-synatt-i": 100,
        # "_____10c-o__ser-cx-pstcr_-f": 3,
        # "_____10c-o__ser-cx-pstcrm-f": 35,
        # "_____10c-o__ser-cx-plst_c-f": 0.05,
        # "_____10c-o__ser-nx-fire_t-f": 1,
        # "_____10c-o__ser-nx-refrac-i": 0,
        # "_____10c-o__ser-nx-leak_c-f": 0,
        # "_____10c-o__ser-nx-c_fr_c-i": 3,
        # "_____10c-o__ser-nx-snooze-f": 0,
        # "_____10c-o__ser-cx-de_gen-f": 0,
        # "_____10c-o__ser-cx-dstmap-d": {}
    }
}
