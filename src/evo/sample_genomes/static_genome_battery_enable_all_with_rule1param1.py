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
    "neighbor_locator_rule": {'rule_0': {'param_1': 0, 'param_2': 0},
                              'rule_1': {'param_1': 1, 'param_2': 1},
                              'rule_10': {'param_1': 1, 'param_2': 1},
                              'rule_11': {'param_1': 1, 'param_2': 1},
                              'rule_12': {'param_1': 1, 'param_2': 1},
                              'rule_13': {'param_1': 1, 'param_2': 1},
                              'rule_14': {'param_1': 1, 'param_2': 1},
                              'rule_15': {'param_1': 1, 'param_2': 1},
                              'rule_16': {'param_1': 1, 'param_2': 1},
                              'rule_17': {'param_1': 1, 'param_2': 1},
                              'rule_18': {'param_1': 1, 'param_2': 1},
                              'rule_19': {'param_1': 1, 'param_2': 1},
                              'rule_2': {'param_1': 5, 'param_2': 5, 'param_3': 3},
                              'rule_3': {'param_1': 3, 'param_2': 3},
                              'rule_4': {'param_1': 3, 'param_2': 3},
                              'rule_5': {'param_1': 3,
                                         'param_2': 3,
                                         'param_3': 3,
                                         'param_4': 3,
                                         'param_5': 3,
                                         'param_6': 3,
                                         'param_7': 3},
                              'rule_6': {'param_1': 1, 'param_2': 1},
                              'rule_7': {'param_1': 1, 'param_2': 1},
                              'rule_9': {'param_1': 1, 'param_2': 1}},
    "species": {
        "parents": {},
        "species_id": "",
        "class": "toy",
        "brand": "gazebo",
        "model": "smart_car"
    },
    "blueprint": {
        "_____10b-_____s-__-__name-t": "smart_car",

        "_____10c-i__bat-cx-__name-t": "battery_ipu",
        "_____10c-i__bat-cx-_group-t": "IPU",
        "_____10c-i__bat-cx-_n_cnt-i": 500,
        "_____10c-i__bat-nx-gd_vis-b": True,
        "_____10c-i__bat-nx-rcordx-i": 50,
        "_____10c-i__bat-nx-rcordy-i": 0,
        "_____10c-i__bat-nx-rcordz-i": 0,
        "_____10c-i__bat-nx-___bbx-i": 1,
        "_____10c-i__bat-nx-___bby-i": 1,
        "_____10c-i__bat-nx-___bbz-i": 10,
        "_____10c-i__bat-cx-__rand-b": True,
        "_____10c-i__bat-cx-synatt-i": 100,
        "_____10c-i__bat-cx-init_s-b": False,
        "_____10c-i__bat-cx-pstcr_-f": 50000,
        "_____10c-i__bat-cx-pstcrm-f": 35,
        "_____10c-i__bat-cx-plst_c-f": 0.05,
        "_____10c-i__bat-nx-fire_t-f": 1,
        "_____10c-i__bat-nx-refrac-i": 0,
        "_____10c-i__bat-nx-leak_c-f": 0,
        "_____10c-i__bat-nx-c_fr_c-i": 3,
        "_____10c-i__bat-nx-snooze-f": 0,
        "_____10c-i__bat-nx-geo_x0-i": 0,
        "_____10c-i__bat-nx-geo_x1-i": 10,
        "_____10c-i__bat-nx-geo_y0-i": 0,
        "_____10c-i__bat-nx-geo_y1-i": 10,
        "_____10c-i__bat-nx-geo_z0-i": 0,
        "_____10c-i__bat-nx-geo_z1-i": 100,
        "_____10c-i__bat-cx-dstmap-d": {"i__bat": ["rule_1", "param_1"]},

        "_____10c-o__bat-cx-__name-t": "battery_opu",
        "_____10c-o__bat-cx-_group-t": "OPU",
        "_____10c-o__bat-cx-_n_cnt-i": 500,
        "_____10c-o__bat-nx-gd_vis-b": True,
        "_____10c-o__bat-nx-rcordx-i": 60,
        "_____10c-o__bat-nx-rcordy-i": 0,
        "_____10c-o__bat-nx-rcordz-i": 0,
        "_____10c-o__bat-nx-___bbx-i": 1,
        "_____10c-o__bat-nx-___bby-i": 1,
        "_____10c-o__bat-nx-___bbz-i": 10,
        "_____10c-o__bat-cx-__rand-b": True,
        "_____10c-o__bat-cx-synatt-i": 100,
        "_____10c-o__bat-cx-init_s-b": False,
        "_____10c-o__bat-cx-pstcr_-f": 50000,
        "_____10c-o__bat-cx-pstcrm-f": 35,
        "_____10c-o__bat-cx-plst_c-f": 0.05,
        "_____10c-o__bat-nx-fire_t-f": 1,
        "_____10c-o__bat-nx-refrac-i": 0,
        "_____10c-o__bat-nx-leak_c-f": 0,
        "_____10c-o__bat-nx-c_fr_c-i": 3,
        "_____10c-o__bat-nx-snooze-f": 0,
        "_____10c-o__bat-nx-geo_x0-i": 0,
        "_____10c-o__bat-nx-geo_x1-i": 10,
        "_____10c-o__bat-nx-geo_y0-i": 0,
        "_____10c-o__bat-nx-geo_y1-i": 10,
        "_____10c-o__bat-nx-geo_z0-i": 0,
        "_____10c-o__bat-nx-geo_z1-i": 100,
        "_____10c-o__bat-cx-dstmap-d": {"o__bat": ["rule_1", "param_1"]},

        "_____10c-o__mot-cx-__name-t": "motor_opu",
        "_____10c-o__mot-cx-_group-t": "OPU",
        "_____10c-o__mot-cx-_n_cnt-i": 1000,
        "_____10c-o__mot-nx-gd_vis-b": True,
        "_____10c-o__mot-nx-rcordx-i": 20,
        "_____10c-o__mot-nx-rcordy-i": 0,
        "_____10c-o__mot-nx-rcordz-i": 0,
        "_____10c-o__mot-nx-___bbx-i": 4,
        "_____10c-o__mot-nx-___bby-i": 1,
        "_____10c-o__mot-nx-___bbz-i": 20,
        "_____10c-o__mot-cx-__rand-b": True,
        "_____10c-o__mot-cx-synatt-i": 100,
        "_____10c-o__mot-cx-init_s-b": False,
        "_____10c-o__mot-cx-pstcr_-f": 500,
        "_____10c-o__mot-cx-pstcrm-f": 35,
        "_____10c-o__mot-cx-plst_c-f": 0.05,
        "_____10c-o__mot-nx-fire_t-f": 1,
        "_____10c-o__mot-nx-refrac-i": 0,
        "_____10c-o__mot-nx-leak_c-f": 0,
        "_____10c-o__mot-nx-c_fr_c-i": 3,
        "_____10c-o__mot-nx-snooze-f": 0,
        "_____10c-o__mot-nx-geo_x0-i": 0,
        "_____10c-o__mot-nx-geo_x1-i": 40,
        "_____10c-o__mot-nx-geo_y0-i": 0,
        "_____10c-o__mot-nx-geo_y1-i": 10,
        "_____10c-o__mot-nx-geo_z0-i": 0,
        "_____10c-o__mot-nx-geo_z1-i": 200,
        "_____10c-o__mot-cx-dstmap-d": {"o__mot": ["rule_1", "param_1"]},

        "_____10c-i__inf-cx-__name-t": "ir_ipu",
        "_____10c-i__inf-cx-_group-t": "IPU",
        "_____10c-i__inf-cx-_n_cnt-i": 3,
        "_____10c-i__inf-nx-gd_vis-b": True,
        "_____10c-i__inf-nx-rcordx-i": 100,
        "_____10c-i__inf-nx-rcordy-i": 0,
        "_____10c-i__inf-nx-rcordz-i": 15,
        "_____10c-i__inf-nx-___bbx-i": 3,
        "_____10c-i__inf-nx-___bby-i": 1,
        "_____10c-i__inf-nx-___bbz-i": 1,
        "_____10c-i__inf-cx-__rand-b": False,
        "_____10c-i__inf-cx-synatt-i": 100,
        "_____10c-i__inf-cx-init_s-b": False,
        "_____10c-i__inf-cx-pstcr_-f": 500,
        "_____10c-i__inf-cx-pstcrm-f": 35,
        "_____10c-i__inf-cx-plst_c-f": 0.05,
        "_____10c-i__inf-nx-fire_t-f": 1,
        "_____10c-i__inf-nx-refrac-i": 0,
        "_____10c-i__inf-nx-leak_c-f": 10,
        "_____10c-i__inf-nx-c_fr_c-i": 3,
        "_____10c-i__inf-nx-snooze-f": 0,
        "_____10c-i__inf-nx-geo_x0-i": 0,
        "_____10c-i__inf-nx-geo_x1-i": 30,
        "_____10c-i__inf-nx-geo_y0-i": 0,
        "_____10c-i__inf-nx-geo_y1-i": 10,
        "_____10c-i__inf-nx-geo_z0-i": 0,
        "_____10c-i__inf-nx-geo_z1-i": 10,
        "_____10c-i__inf-cx-dstmap-d": {"i__inf": ["rule_1", "param_1"]},

        "_____10c-i__mot-cx-__name-t": "motor_ipu",
        "_____10c-i__mot-cx-_group-t": "IPU",
        "_____10c-i__mot-cx-_n_cnt-i": 1000,
        "_____10c-i__mot-nx-gd_vis-b": True,
        "_____10c-i__mot-nx-rcordx-i": 100,
        "_____10c-i__mot-nx-rcordy-i": 0,
        "_____10c-i__mot-nx-rcordz-i": 25,
        "_____10c-i__mot-nx-___bbx-i": 4,
        "_____10c-i__mot-nx-___bby-i": 1,
        "_____10c-i__mot-nx-___bbz-i": 20,
        "_____10c-i__mot-cx-__rand-b": True,
        "_____10c-i__mot-cx-synatt-i": 100,
        "_____10c-i__mot-cx-init_s-b": False,
        "_____10c-i__mot-cx-pstcr_-f": 500,
        "_____10c-i__mot-cx-pstcrm-f": 35,
        "_____10c-i__mot-cx-plst_c-f": 0.05,
        "_____10c-i__mot-nx-fire_t-f": 1,
        "_____10c-i__mot-nx-refrac-i": 0,
        "_____10c-i__mot-nx-leak_c-f": 1,
        "_____10c-i__mot-nx-c_fr_c-i": 1,
        "_____10c-i__mot-nx-snooze-f": 0,
        "_____10c-i__mot-nx-geo_x0-i": 0,
        "_____10c-i__mot-nx-geo_x1-i": 50,
        "_____10c-i__mot-nx-geo_y0-i": 0,
        "_____10c-i__mot-nx-geo_y1-i": 50,
        "_____10c-i__mot-nx-geo_z0-i": 0,
        "_____10c-i__mot-nx-geo_z1-i": 200,
        "_____10c-i__mot-cx-dstmap-d": {"i__mot": ["rule_1", "param_1"]},

        "_____10c-i__pro-cx-__name-t": "proximity_ipu",
        "_____10c-i__pro-cx-_group-t": "memory",
        "_____10c-i__pro-cx-_n_cnt-i": 1000,
        "_____10c-i__pro-nx-gd_vis-b": True,
        "_____10c-i__pro-nx-rcordx-i": 100,
        "_____10c-i__pro-nx-rcordy-i": 0,
        "_____10c-i__pro-nx-rcordz-i": -35,
        "_____10c-i__pro-nx-___bbx-i": 1,
        "_____10c-i__pro-nx-___bby-i": 1,
        "_____10c-i__pro-nx-___bbz-i": 20,
        "_____10c-i__pro-cx-__rand-b": True,
        "_____10c-i__pro-cx-synatt-i": 100,
        "_____10c-i__pro-cx-init_s-b": False,
        "_____10c-i__pro-cx-pstcr_-f": 500,
        "_____10c-i__pro-cx-pstcrm-f": 35,
        "_____10c-i__pro-cx-plst_c-f": 0.05,
        "_____10c-i__pro-nx-fire_t-f": 1,
        "_____10c-i__pro-nx-refrac-i": 0,
        "_____10c-i__pro-nx-leak_c-f": 10,
        "_____10c-i__pro-nx-c_fr_c-i": 1,
        "_____10c-i__pro-nx-snooze-f": 0,
        "_____10c-i__pro-nx-geo_x0-i": 0,
        "_____10c-i__pro-nx-geo_x1-i": 100,
        "_____10c-i__pro-nx-geo_y0-i": 0,
        "_____10c-i__pro-nx-geo_y1-i": 100,
        "_____10c-i__pro-nx-geo_z0-i": 0,
        "_____10c-i__pro-nx-geo_z1-i": 2000,
        "_____10c-i__pro-cx-dstmap-d": {"i__pro": ["rule_1", "param_1"]},

        "_____10c-t__mot-cx-__name-t": "motor_thalamus",
        "_____10c-t__mot-cx-_group-t": "thalamus",
        "_____10c-t__mot-cx-_n_cnt-i": 1000,
        "_____10c-t__mot-nx-gd_vis-b": True,
        "_____10c-t__mot-nx-rcordx-i": 80,
        "_____10c-t__mot-nx-rcordy-i": 0,
        "_____10c-t__mot-nx-rcordz-i": 0,
        "_____10c-t__mot-nx-___bbx-i": 4,
        "_____10c-t__mot-nx-___bby-i": 1,
        "_____10c-t__mot-nx-___bbz-i": 20,
        "_____10c-t__mot-cx-__rand-b": True,
        "_____10c-t__mot-cx-synatt-i": 100,
        "_____10c-t__mot-cx-init_s-b": False,
        "_____10c-t__mot-cx-pstcr_-f": -500,
        "_____10c-t__mot-cx-pstcrm-f": 35,
        "_____10c-t__mot-cx-plst_c-f": 0.05,
        "_____10c-t__mot-nx-fire_t-f": 1,
        "_____10c-t__mot-nx-refrac-i": 0,
        "_____10c-t__mot-nx-leak_c-f": 10,
        "_____10c-t__mot-nx-c_fr_c-i": 1,
        "_____10c-t__mot-nx-snooze-f": 0,
        "_____10c-t__mot-nx-geo_x0-i": 0,
        "_____10c-t__mot-nx-geo_x1-i": 3600,
        "_____10c-t__mot-nx-geo_y0-i": 0,
        "_____10c-t__mot-nx-geo_y1-i": 1800,
        "_____10c-t__mot-nx-geo_z0-i": 0,
        "_____10c-t__mot-nx-geo_z1-i": 200,
        "_____10c-t__mot-cx-dstmap-d": {"t__mot": ["rule_1", "param_1"]}

    }
}
