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

        # +++ Memory +++


        # +++ Misc +++



        # +++ Thalami +++


        # vvvvvvv   Input and Output processing related Cortical Areas are all listed below  vvvvvvvv

        # +++ Input Processing Units (IPU) +++
        "_____10c-i__inf-cx-__name-t": "ir_ipu",
        "_____10c-i__inf-cx-_group-t": "IPU",
        "_____10c-i__inf-cx-_n_cnt-i": 1,
        "_____10c-i__inf-nx-gd_vis-b": True,
        "_____10c-i__inf-nx-rcordx-i": 80,
        "_____10c-i__inf-nx-rcordy-i": 0,
        "_____10c-i__inf-nx-rcordz-i": 0,
        "_____10c-i__inf-nx-___bbx-i": 3,
        "_____10c-i__inf-nx-___bby-i": 1,
        "_____10c-i__inf-nx-___bbz-i": 1,
        "_____10c-i__inf-cx-synatt-i": 100,
        "_____10c-i__inf-cx-pstcr_-f": 5,
        "_____10c-i__inf-cx-pstcrm-f": 35,
        "_____10c-i__inf-cx-plst_c-f": 0.05,
        "_____10c-i__inf-nx-fire_t-f": 1,
        "_____10c-i__inf-nx-refrac-i": 0,
        "_____10c-i__inf-nx-leak_c-f": 10,
        "_____10c-i__inf-nx-c_fr_c-i": 3,
        "_____10c-i__inf-nx-snooze-f": 0,
        "_____10c-i__inf-cx-de_gen-f": 0.01,
        "_____10c-i__inf-cx-dstmap-d": {"i__inf": ["block_to_block", [1, 1, 1], 1, False]},

        "_____10c-i__pro-cx-__name-t": "proximity_ipu",
        "_____10c-i__pro-cx-_group-t": "memory",
        "_____10c-i__pro-cx-_n_cnt-i": 1,
        "_____10c-i__pro-nx-gd_vis-b": True,
        "_____10c-i__pro-nx-rcordx-i": 100,
        "_____10c-i__pro-nx-rcordy-i": 0,
        "_____10c-i__pro-nx-rcordz-i": -35,
        "_____10c-i__pro-nx-___bbx-i": 1,
        "_____10c-i__pro-nx-___bby-i": 1,
        "_____10c-i__pro-nx-___bbz-i": 10,
        "_____10c-i__pro-cx-synatt-i": 100,
        "_____10c-i__pro-cx-pstcr_-f": 5,
        "_____10c-i__pro-cx-pstcrm-f": 35,
        "_____10c-i__pro-cx-plst_c-f": 0.05,
        "_____10c-i__pro-nx-fire_t-f": 1,
        "_____10c-i__pro-nx-refrac-i": 0,
        "_____10c-i__pro-nx-leak_c-f": 10,
        "_____10c-i__pro-nx-c_fr_c-i": 1,
        "_____10c-i__pro-nx-snooze-f": 0,
        "_____10c-i__pro-cx-de_gen-f": 0,
        "_____10c-i__pro-cx-dstmap-d": {"t__pro": ["block_to_block", [1, 1, 1], 1, False]},

        "_____10c-t__pro-cx-__name-t": "proximity_thalamus",
        "_____10c-t__pro-cx-_group-t": "memory",
        "_____10c-t__pro-cx-_n_cnt-i": 1,
        "_____10c-t__pro-nx-gd_vis-b": True,
        "_____10c-t__pro-nx-rcordx-i": 70,
        "_____10c-t__pro-nx-rcordy-i": 0,
        "_____10c-t__pro-nx-rcordz-i": 0,
        "_____10c-t__pro-nx-___bbx-i": 1,
        "_____10c-t__pro-nx-___bby-i": 1,
        "_____10c-t__pro-nx-___bbz-i": 10,
        "_____10c-t__pro-cx-synatt-i": 20,
        "_____10c-t__pro-cx-pstcr_-f": 5,
        "_____10c-t__pro-cx-pstcrm-f": 35,
        "_____10c-t__pro-cx-plst_c-f": 0.05,
        "_____10c-t__pro-nx-fire_t-f": 1,
        "_____10c-t__pro-nx-refrac-i": 0,
        "_____10c-t__pro-nx-leak_c-f": 10,
        "_____10c-t__pro-nx-c_fr_c-i": 1,
        "_____10c-t__pro-nx-snooze-f": 0,
        "_____10c-t__pro-cx-de_gen-f": 0,
        "_____10c-t__pro-cx-dstmap-d": {},

        "_____10c-i__bat-cx-__name-t": "battery_ipu",
        "_____10c-i__bat-cx-_group-t": "IPU",
        "_____10c-i__bat-cx-_n_cnt-i": 1,
        "_____10c-i__bat-nx-gd_vis-b": True,
        "_____10c-i__bat-nx-rcordx-i": 50,
        "_____10c-i__bat-nx-rcordy-i": 0,
        "_____10c-i__bat-nx-rcordz-i": 0,
        "_____10c-i__bat-nx-___bbx-i": 1,
        "_____10c-i__bat-nx-___bby-i": 1,
        "_____10c-i__bat-nx-___bbz-i": 10,
        "_____10c-i__bat-cx-synatt-i": 100,
        "_____10c-i__bat-cx-pstcr_-f": 5,
        "_____10c-i__bat-cx-pstcrm-f": 35,
        "_____10c-i__bat-cx-plst_c-f": 0.05,
        "_____10c-i__bat-nx-fire_t-f": 1,
        "_____10c-i__bat-nx-refrac-i": 0,
        "_____10c-i__bat-nx-leak_c-f": 0,
        "_____10c-i__bat-nx-c_fr_c-i": 3,
        "_____10c-i__bat-nx-snooze-f": 0,
        "_____10c-i__bat-cx-de_gen-f": 0,
        "_____10c-i__bat-cx-dstmap-d": {},

        # +++ Output Processing Units (OPU) +++
        "_____10c-o__bat-cx-__name-t": "battery_opu",
        "_____10c-o__bat-cx-_group-t": "OPU",
        "_____10c-o__bat-cx-_n_cnt-i": 1,
        "_____10c-o__bat-nx-gd_vis-b": True,
        "_____10c-o__bat-nx-rcordx-i": 60,
        "_____10c-o__bat-nx-rcordy-i": 0,
        "_____10c-o__bat-nx-rcordz-i": 0,
        "_____10c-o__bat-nx-___bbx-i": 1,
        "_____10c-o__bat-nx-___bby-i": 1,
        "_____10c-o__bat-nx-___bbz-i": 10,
        "_____10c-o__bat-cx-synatt-i": 100,
        "_____10c-o__bat-cx-pstcr_-f": 5,
        "_____10c-o__bat-cx-pstcrm-f": 35,
        "_____10c-o__bat-cx-plst_c-f": 0.05,
        "_____10c-o__bat-nx-fire_t-f": 1,
        "_____10c-o__bat-nx-refrac-i": 0,
        "_____10c-o__bat-nx-leak_c-f": 0,
        "_____10c-o__bat-nx-c_fr_c-i": 3,
        "_____10c-o__bat-nx-snooze-f": 0,
        "_____10c-o__bat-cx-de_gen-f": 0,
        "_____10c-o__bat-cx-dstmap-d": {},

        "_____10c-o__mot-cx-__name-t": "motor_opu",
        "_____10c-o__mot-cx-_group-t": "OPU",
        "_____10c-o__mot-cx-_n_cnt-i": 1,
        "_____10c-o__mot-nx-gd_vis-b": True,
        "_____10c-o__mot-nx-rcordx-i": 20,
        "_____10c-o__mot-nx-rcordy-i": 0,
        "_____10c-o__mot-nx-rcordz-i": 0,
        "_____10c-o__mot-nx-___bbx-i": 8,
        "_____10c-o__mot-nx-___bby-i": 1,
        "_____10c-o__mot-nx-___bbz-i": 10,
        "_____10c-o__mot-cx-synatt-i": 100,
        "_____10c-o__mot-cx-pstcr_-f": 6,
        "_____10c-o__mot-cx-pstcrm-f": 35,
        "_____10c-o__mot-cx-plst_c-f": 0.05,
        "_____10c-o__mot-nx-fire_t-f": 1,
        "_____10c-o__mot-nx-refrac-i": 0,
        "_____10c-o__mot-nx-leak_c-f": 0,
        "_____10c-o__mot-nx-c_fr_c-i": 3,
        "_____10c-o__mot-nx-snooze-f": 0,
        "_____10c-o__mot-cx-de_gen-f": 0.01,
        "_____10c-o__mot-cx-dstmap-d": {},

        "_____10c-o__ser-cx-__name-t": "servo_opu",
        "_____10c-o__ser-cx-_group-t": "OPU",
        "_____10c-o__ser-cx-_n_cnt-i": 1,
        "_____10c-o__ser-nx-gd_vis-b": True,
        "_____10c-o__ser-nx-rcordx-i": 35,
        "_____10c-o__ser-nx-rcordy-i": 0,
        "_____10c-o__ser-nx-rcordz-i": 0,
        "_____10c-o__ser-nx-___bbx-i": 4,
        "_____10c-o__ser-nx-___bby-i": 1,
        "_____10c-o__ser-nx-___bbz-i": 10,
        "_____10c-o__ser-cx-synatt-i": 100,
        "_____10c-o__ser-cx-pstcr_-f": 3,
        "_____10c-o__ser-cx-pstcrm-f": 35,
        "_____10c-o__ser-cx-plst_c-f": 0.05,
        "_____10c-o__ser-nx-fire_t-f": 1,
        "_____10c-o__ser-nx-refrac-i": 0,
        "_____10c-o__ser-nx-leak_c-f": 0,
        "_____10c-o__ser-nx-c_fr_c-i": 3,
        "_____10c-o__ser-nx-snooze-f": 0,
        "_____10c-o__ser-cx-de_gen-f": 0,
        "_____10c-o__ser-cx-dstmap-d": {}
    }
}

