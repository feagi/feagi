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

        "one_to_row_zero": {
            "patterns": [[[0, 0, 0], ["*", 0, "*"]]]
        },
        "one_to_row_one": {
            "patterns": [[[0, 0, 0], ["*", 1, "*"]]]
        },
        "block_to_block": {
            "vectors": [[0, 0, 0]]
        },

        "block_to_block_one_row_up": {
            "vectors": [[0, 1, 0]]
        },


        # Rules to describe connectivity between integration and detection areas
        "integrate_to_detect_incr_one": {
            "patterns": [[[1, "*", 0], [0, 0, 0]],
                         [[2, "*", 0], [1, 0, 0]],
                         [[3, "*", 0], [2, 0, 0]]]
        },
        "integrate_to_detect_incr_two": {
            "patterns": [[[2, "*", 0], [1, 0, 0]],
                         [[3, "*", 0], [2, 0, 0]]]
        },
        "integrate_to_detect_incr_three": {
            "patterns": [[[3, "*", 0], [0, 0, 0]]]
        },
        "integrate_to_detect_decr_one": {
            "patterns": [[[2, "*", 0], [0, 0, 0]],
                         [[1, "*", 0], [1, 0, 0]],
                         [[0, "*", 0], [2, 0, 0]]]
        },
        "integrate_to_detect_decr_two": {
            "patterns": [[[1, "*", 0], [1, 0, 0]],
                         [[0, "*", 0], [2, 0, 0]]]
        },
        "integrate_to_detect_decr_three": {
            "patterns": [[[0, "*", 0], [0, 0, 0]]]
        },
        

        # Rules to define diagonal connectivity in various integration areas
        "decrease_filter_diagonal_one": {
            "vectors": [[-1, 1, 0]]
        },
        "decrease_filter_diagonal_two": {
            "vectors": [[-2, 1, 0]]
        },
        "decrease_filter_diagonal_three": {
            "vectors": [[-3, 1, 0]]
        },
        "increase_filter_diagonal_one": {
            "vectors": [[1, 1, 0]]
        },
        "increase_filter_diagonal_two": {
            "vectors": [[2, 1, 0]]
        },
        "increase_filter_diagonal_three": {
            "vectors": [[3, 1, 0]]
        },
        # Rules used to connect motion inducing regions to motor OPU
        "turn_right_e": {
            "patterns": [
                [[0, 0, 0], [0, 0, 4]],
                [[0, 0, 0], [3, 0, 3]],
                [[0, 0, 0], [4, 0, 4]],
                [[0, 0, 0], [7, 0, 3]]
            ]
        },
        "turn_left_e": {
            "patterns": [
                [[0, 0, 0], [1, 0, 4]],
                [[0, 0, 0], [2, 0, 3]],
                [[0, 0, 0], [5, 0, 4]],
                [[0, 0, 0], [6, 0, 3]]
            ]
        },
        "forward_e": {
            "patterns": [
                [[0, 0, 0], [0, 0, 3]],
                [[0, 0, 0], [2, 0, 3]],
                [[0, 0, 0], [4, 0, 3]],
                [[0, 0, 0], [6, 0, 3]]
            ]
        },
        "backward_e": {
            "patterns": [
                [[0, 0, 0], [1, 0, 7]],
                [[0, 0, 0], [3, 0, 7]],
                [[0, 0, 0], [5, 0, 7]],
                [[0, 0, 0], [7, 0, 7]],
                [[1, 0, 0], [1, 0, 7]],
                [[1, 0, 0], [3, 0, 7]],
                [[1, 0, 0], [5, 0, 7]],
                [[1, 0, 0], [7, 0, 7]],
                [[2, 0, 0], [1, 0, 7]],
                [[2, 0, 0], [3, 0, 7]],
                [[2, 0, 0], [5, 0, 7]],
                [[2, 0, 0], [7, 0, 7]]
            ]
        },
        "lat_+x": {
            "vectors": [[1, 0, 0]]
        },
        "ir_patterns_excitation" : {
            "patterns": [
                [[2, 0, 0], [1, 0, 0]],
                [[1, 0, 0], [2, 0, 0]],
                [[1, 0, 0], [3, 0, 0]],
                [[2, 0, 0], [3, 0, 0]],
                [[0, 0, 0], [4, 0, 0]],
                [[0, 0, 0], [5, 0, 0]],
                [[2, 0, 0], [5, 0, 0]],
                [[0, 0, 0], [6, 0, 0]],
                [[1, 0, 0], [6, 0, 0]],
                [[0, 0, 0], [7, 0, 0]],
                [[1, 0, 0], [7, 0, 0]],
                [[2, 0, 0], [7, 0, 0]]]

        },
        "ir_patterns_inhibition" : {
            "patterns": [
                [[0, 0, 0], [0, 0, 0]],
                [[1, 0, 0], [0, 0, 0]],
                [[2, 0, 0], [0, 0, 0]],
                [[0, 0, 0], [1, 0, 0]],
                [[1, 0, 0], [1, 0, 0]],
                [[0, 0, 0], [2, 0, 0]],
                [[2, 0, 0], [2, 0, 0]],
                [[0, 0, 0], [3, 0, 0]],
                [[1, 0, 0], [4, 0, 0]],
                [[2, 0, 0], [4, 0, 0]],
                [[1, 0, 0], [5, 0, 0]],
                [[2, 0, 0], [6, 0, 0]]
            ]
        },

        "brightness_integration": {
            "patterns": [
                [[0,0,0], [0,0,0]],
                [[1,0,0], [1,0,0]],
                [[2,0,0], [1,0,0]],
                [[3,0,0], [2,0,0]],
                [[4,0,0], [1,0,0]],
                [[5,0,0], [2,0,0]],
                [[6,0,0], [2,0,0]],
                [[7,0,0], [3,0,0]]
            ]
        },

        # Patterns to connect steering cortical area to motor control areas
        "steering_to_forward":{
            "patterns": [
                [[0,0,0], [0,0,0]],
                [[0,1,0], [0,0,0]],
                [[2,0,0], [0,0,0]],
                [[2,1,0], [0,0,0]],
                [[5,0,0], [0,0,0]],
                [[5,1,0], [0,0,0]],
                [[7,0,0], [0,0,0]],
                [[7,1,0]], [0,0,0]
        ]},

        "steering_to_right": {
            "patterns": [
                [[1, 0, 0], [0, 0, 0]],
                [[3, 0, 0], [0, 0, 0]],
                [[4, 1, 0], [0, 0, 0]],
                [[6, 1, 0], [0, 0, 0]]
            ]},
        "steering_to_left": {
            "patterns": [
                [[1, 1, 0], [0, 0, 0]],
                [[3, 1, 0], [0, 0, 0]],
                [[4, 0, 0], [0, 0, 0]],
                [[6, 0, 0], [0, 0, 0]]
            ]},


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

        # +++ Input Processing Units (IPU) +++
        "_____10c-i__inf-cx-__name-t": "ir_ipu",
        "_____10c-i__inf-cx-_group-t": "IPU",
        "_____10c-i__inf-cx-_n_cnt-i": 1,
        "_____10c-i__inf-nx-gd_vis-b": True,
        "_____10c-i__inf-nx-rcordx-i": 15,
        "_____10c-i__inf-nx-rcordy-i": 0,
        "_____10c-i__inf-nx-rcordz-i": 0,
        "_____10c-i__inf-nx-___bbx-i": 3,
        "_____10c-i__inf-nx-___bby-i": 1,
        "_____10c-i__inf-nx-___bbz-i": 1,
        "_____10c-i__inf-cx-synatt-i": 100,
        "_____10c-i__inf-cx-pstcr_-f": 80,
        "_____10c-i__inf-cx-pstcrm-f": 100,
        "_____10c-i__inf-cx-plst_c-f": 0.05,
        "_____10c-i__inf-nx-fire_t-f": 1,
        "_____10c-i__inf-nx-refrac-i": 0,
        "_____10c-i__inf-nx-leak_c-f": 0,
        "_____10c-i__inf-nx-c_fr_c-i": 0,
        "_____10c-i__inf-nx-snooze-f": 0,
        "_____10c-i__inf-cx-de_gen-f": 0,
        "_____10c-i__inf-cx-dstmap-d": {"t__vis": [["ir_patterns_excitation", [1,1,1], 1, False], ["ir_patterns_inhibition", [1,1,1], -1, False]]},

        # +++ Inverse Input Processing Unit (IPU) +++
        "_____10c-ii_inf-cx-__name-t": "inv_ir_ipu",
        "_____10c-ii_inf-cx-_group-t": "IPU",
        "_____10c-ii_inf-cx-_n_cnt-i": 1,
        "_____10c-ii_inf-nx-gd_vis-b": True,
        "_____10c-ii_inf-nx-rcordx-i": 25,
        "_____10c-ii_inf-nx-rcordy-i": 0,
        "_____10c-ii_inf-nx-rcordz-i": 0,
        "_____10c-ii_inf-nx-___bbx-i": 3,
        "_____10c-ii_inf-nx-___bby-i": 1,
        "_____10c-ii_inf-nx-___bbz-i": 1,
        "_____10c-ii_inf-cx-synatt-i": 100,
        "_____10c-ii_inf-cx-pstcr_-f": 80,
        "_____10c-ii_inf-cx-pstcrm-f": 100,
        "_____10c-ii_inf-cx-plst_c-f": 0.05,
        "_____10c-ii_inf-nx-fire_t-f": 1,
        "_____10c-ii_inf-nx-refrac-i": 0,
        "_____10c-ii_inf-nx-leak_c-f": 0,
        "_____10c-ii_inf-nx-c_fr_c-i": 0,
        "_____10c-ii_inf-nx-snooze-f": 0,
        "_____10c-ii_inf-cx-de_gen-f": 0,
        "_____10c-ii_inf-cx-dstmap-d": {"t__vis": [["ir_patterns_excitation", [1,1,1], -1, False], ["ir_patterns_inhibition", [1,1,1], 1, False]]
        },


        # +++ Detect unique IR patterns +++
        "_____10c-t__vis-cx-__name-t": "IR_patterns",
        "_____10c-t__vis-cx-_group-t": "vision",
        "_____10c-t__vis-cx-_n_cnt-i": 1,
        "_____10c-t__vis-nx-gd_vis-b": True,
        "_____10c-t__vis-nx-rcordx-i": 20,
        "_____10c-t__vis-nx-rcordy-i": 10,
        "_____10c-t__vis-nx-rcordz-i": 0,
        "_____10c-t__vis-nx-___bbx-i": 8,
        "_____10c-t__vis-nx-___bby-i": 1,
        "_____10c-t__vis-nx-___bbz-i": 1,
        "_____10c-t__vis-cx-synatt-i": 100,
        "_____10c-t__vis-cx-pstcr_-f": 50,
        "_____10c-t__vis-cx-pstcrm-f": 100,
        "_____10c-t__vis-cx-plst_c-f": 0.05,
        "_____10c-t__vis-nx-fire_t-f": 29,
        "_____10c-t__vis-nx-refrac-i": 0,
        "_____10c-t__vis-nx-leak_c-f": 100,
        "_____10c-t__vis-nx-c_fr_c-i": 0,
        "_____10c-t__vis-nx-snooze-f": 0,
        "_____10c-t__vis-cx-de_gen-f": 0,
        "_____10c-t__vis-cx-pspuni-b": True,
        "_____10c-t__vis-cx-dstmap-d": {
            "is_dcr": [["brightness_integration", [1,1,1], 1, False]],
            "im_dcr": [["brightness_integration", [1,1,1], 1, False]],
            "il_dcr": [["brightness_integration", [1,1,1], 1, False]],
            "is_inc": [["brightness_integration", [1,1,1], 1, False]],
            "im_inc": [["brightness_integration", [1,1,1], 1, False]],
            "il_inc": [["brightness_integration", [1,1,1], 1, False]],
            "_steer": [["block_to_block", [1,1,1], 1, False],
                       ["block_to_block_one_row_up", [1,1,1], 1, False]]
        },

        # Brightness Increase Integration 1 (small increase)
        "_____10c-is_inc-cx-__name-t": "small_incr_intgr",
        "_____10c-is_inc-cx-_group-t": "vision",
        "_____10c-is_inc-cx-_n_cnt-i": 1,
        "_____10c-is_inc-nx-gd_vis-b": True,
        "_____10c-is_inc-nx-rcordx-i": 0,
        "_____10c-is_inc-nx-rcordy-i": 20,
        "_____10c-is_inc-nx-rcordz-i": 0,
        "_____10c-is_inc-nx-___bbx-i": 4,
        "_____10c-is_inc-nx-___bby-i": 2,
        "_____10c-is_inc-nx-___bbz-i": 1,
        "_____10c-is_inc-cx-synatt-i": 100,
        "_____10c-is_inc-cx-pstcr_-f": 50,
        "_____10c-is_inc-cx-pstcrm-f": 200,
        "_____10c-is_inc-cx-plst_c-f": 0.05,
        "_____10c-is_inc-nx-fire_t-f": 49,
        "_____10c-is_inc-nx-refrac-i": 0,
        "_____10c-is_inc-nx-leak_c-f": 50,
        "_____10c-is_inc-nx-c_fr_c-i": 0,
        "_____10c-is_inc-nx-snooze-f": 0,
        "_____10c-is_inc-cx-de_gen-f": 0,
        "_____10c-is_inc-cx-pspuni-b": True,
        "_____10c-is_inc-cx-dstmap-d": {"is_inc": [["increase_filter_diagonal_one", [1,1,1], 1, False]],
                                        "ds_inc": [["integrate_to_detect_incr_one", [1,1,1], 1, False]]
        },

        # Brightness Increase Integration 2 (medium increase)
        "_____10c-im_inc-cx-__name-t": "med_incr_intgr",
        "_____10c-im_inc-cx-_group-t": "vision",
        "_____10c-im_inc-cx-_n_cnt-i": 1,
        "_____10c-im_inc-nx-gd_vis-b": True,
        "_____10c-im_inc-nx-rcordx-i": 10,
        "_____10c-im_inc-nx-rcordy-i": 20,
        "_____10c-im_inc-nx-rcordz-i": 0,
        "_____10c-im_inc-nx-___bbx-i": 4,
        "_____10c-im_inc-nx-___bby-i": 2,
        "_____10c-im_inc-nx-___bbz-i": 1,
        "_____10c-im_inc-cx-synatt-i": 100,
        "_____10c-im_inc-cx-pstcr_-f": 50,
        "_____10c-im_inc-cx-pstcrm-f": 200,
        "_____10c-im_inc-cx-plst_c-f": 0.05,
        "_____10c-im_inc-nx-fire_t-f": 49,
        "_____10c-im_inc-nx-refrac-i": 0,
        "_____10c-im_inc-nx-leak_c-f": 50,
        "_____10c-im_inc-nx-c_fr_c-i": 0,
        "_____10c-im_inc-nx-snooze-f": 0,
        "_____10c-im_inc-cx-de_gen-f": 0,
        "_____10c-im_inc-cx-pspuni-b": True,
        "_____10c-im_inc-cx-dstmap-d": {"im_inc": [["increase_filter_diagonal_two", [1, 1, 1], 1, False]],
                                        "dm_inc": [["integrate_to_detect_incr_two", [1, 1, 1], 1, False]]
                                        },

        # Brightness Increase Integration 3 (threshold detection)
        "_____10c-il_inc-cx-__name-t": "lar_incr_intgr",
        "_____10c-il_inc-cx-_group-t": "vision",
        "_____10c-il_inc-cx-_n_cnt-i": 1,
        "_____10c-il_inc-nx-gd_vis-b": True,
        "_____10c-il_inc-nx-rcordx-i": 20,
        "_____10c-il_inc-nx-rcordy-i": 20,
        "_____10c-il_inc-nx-rcordz-i": 0,
        "_____10c-il_inc-nx-___bbx-i": 4,
        "_____10c-il_inc-nx-___bby-i": 2,
        "_____10c-il_inc-nx-___bbz-i": 1,
        "_____10c-il_inc-cx-synatt-i": 100,
        "_____10c-il_inc-cx-pstcr_-f": 50,
        "_____10c-il_inc-cx-pstcrm-f": 200,
        "_____10c-il_inc-cx-plst_c-f": 0.05,
        "_____10c-il_inc-nx-fire_t-f": 49,
        "_____10c-il_inc-nx-refrac-i": 0,
        "_____10c-il_inc-nx-leak_c-f": 50,
        "_____10c-il_inc-nx-c_fr_c-i": 0,
        "_____10c-il_inc-nx-snooze-f": 0,
        "_____10c-il_inc-cx-de_gen-f": 0,
        "_____10c-il_inc-cx-pspuni-b": True,
        "_____10c-il_inc-cx-dstmap-d": {"il_inc": [["increase_filter_diagonal_three", [1, 1, 1], 1, False]],
                                        "dl_inc": [["integrate_to_detect_incr_three", [1, 1, 1], 1, False]]
                                        },



        # Brightness Increase Detection 1 (Detect small increase in Brightness)
        "_____10c-ds_inc-cx-__name-t": "small_incr_detect",
        "_____10c-ds_inc-cx-_group-t": "vision",
        "_____10c-ds_inc-cx-_n_cnt-i": 1,
        "_____10c-ds_inc-nx-gd_vis-b": True,
        "_____10c-ds_inc-nx-rcordx-i": 0,
        "_____10c-ds_inc-nx-rcordy-i": 30,
        "_____10c-ds_inc-nx-rcordz-i": 0,
        "_____10c-ds_inc-nx-___bbx-i": 3,
        "_____10c-ds_inc-nx-___bby-i": 1,
        "_____10c-ds_inc-nx-___bbz-i": 1,
        "_____10c-ds_inc-cx-synatt-i": 100,
        "_____10c-ds_inc-cx-pstcr_-f": 100,
        "_____10c-ds_inc-cx-pstcrm-f": 200,
        "_____10c-ds_inc-cx-plst_c-f": 0.05,
        "_____10c-ds_inc-nx-fire_t-f": 90,
        "_____10c-ds_inc-nx-refrac-i": 0,
        "_____10c-ds_inc-nx-leak_c-f": 100,
        "_____10c-ds_inc-nx-c_fr_c-i": 0,
        "_____10c-ds_inc-nx-snooze-f": 0,
        "_____10c-ds_inc-cx-de_gen-f": 0,
        "_____10c-ds_inc-cx-dstmap-d": {
        },

        # Brightness Increase Detection 2 (Detect medium increase in Brightness)
        "_____10c-dm_inc-cx-__name-t": "med_incr_detect",
        "_____10c-dm_inc-cx-_group-t": "vision",
        "_____10c-dm_inc-cx-_n_cnt-i": 1,
        "_____10c-dm_inc-nx-gd_vis-b": True,
        "_____10c-dm_inc-nx-rcordx-i": 10,
        "_____10c-dm_inc-nx-rcordy-i": 30,
        "_____10c-dm_inc-nx-rcordz-i": 0,
        "_____10c-dm_inc-nx-___bbx-i": 3,
        "_____10c-dm_inc-nx-___bby-i": 1,
        "_____10c-dm_inc-nx-___bbz-i": 1,
        "_____10c-dm_inc-cx-synatt-i": 100,
        "_____10c-dm_inc-cx-pstcr_-f": 100,
        "_____10c-dm_inc-cx-pstcrm-f": 200,
        "_____10c-dm_inc-cx-plst_c-f": 0.05,
        "_____10c-dm_inc-nx-fire_t-f": 90,
        "_____10c-dm_inc-nx-refrac-i": 0,
        "_____10c-dm_inc-nx-leak_c-f": 100,
        "_____10c-dm_inc-nx-c_fr_c-i": 0,
        "_____10c-dm_inc-nx-snooze-f": 0,
        "_____10c-dm_inc-cx-de_gen-f": 0,
        "_____10c-dm_inc-cx-dstmap-d": {
        },

        # Brightness Increase Detection 3 (dark to bright)
        "_____10c-dl_inc-cx-__name-t": "lrg_incr_detect",
        "_____10c-dl_inc-cx-_group-t": "vision",
        "_____10c-dl_inc-cx-_n_cnt-i": 1,
        "_____10c-dl_inc-nx-gd_vis-b": True,
        "_____10c-dl_inc-nx-rcordx-i": 20,
        "_____10c-dl_inc-nx-rcordy-i": 30,
        "_____10c-dl_inc-nx-rcordz-i": 0,
        "_____10c-dl_inc-nx-___bbx-i": 1,
        "_____10c-dl_inc-nx-___bby-i": 1,
        "_____10c-dl_inc-nx-___bbz-i": 1,
        "_____10c-dl_inc-cx-synatt-i": 100,
        "_____10c-dl_inc-cx-pstcr_-f": 50,
        "_____10c-dl_inc-cx-pstcrm-f": 200,
        "_____10c-dl_inc-cx-plst_c-f": 0.05,
        "_____10c-dl_inc-nx-fire_t-f": 90,
        "_____10c-dl_inc-nx-refrac-i": 0,
        "_____10c-dl_inc-nx-leak_c-f": 100,
        "_____10c-dl_inc-nx-c_fr_c-i": 0,
        "_____10c-dl_inc-nx-snooze-f": 0,
        "_____10c-dl_inc-cx-de_gen-f": 0,
        "_____10c-dl_inc-cx-pspuni-b": True,
        "_____10c-dl_inc-cx-dstmap-d": {
            "__back": [["block_to_block", [1, 1, 1], 1, False]]
        },

        ############ BRIGHTNESS DECREASING DETECTION and INTEGRATION AREAS ##############

        # Brightness Decrease Integration 1 (small decrease)
        "_____10c-is_dcr-cx-__name-t": "sml_decr_intgr",
        "_____10c-is_dcr-cx-_group-t": "vision",
        "_____10c-is_dcr-cx-_n_cnt-i": 1,
        "_____10c-is_dcr-nx-gd_vis-b": True,
        "_____10c-is_dcr-nx-rcordx-i": 35,
        "_____10c-is_dcr-nx-rcordy-i": 20,
        "_____10c-is_dcr-nx-rcordz-i": 0,
        "_____10c-is_dcr-nx-___bbx-i": 4,
        "_____10c-is_dcr-nx-___bby-i": 2,
        "_____10c-is_dcr-nx-___bbz-i": 1,
        "_____10c-is_dcr-cx-synatt-i": 100,
        "_____10c-is_dcr-cx-pstcr_-f": 50,
        "_____10c-is_dcr-cx-pstcrm-f": 200,
        "_____10c-is_dcr-cx-plst_c-f": 0.05,
        "_____10c-is_dcr-nx-fire_t-f": 49,
        "_____10c-is_dcr-nx-refrac-i": 0,
        "_____10c-is_dcr-nx-leak_c-f": 50,
        "_____10c-is_dcr-nx-c_fr_c-i": 0,
        "_____10c-is_dcr-nx-snooze-f": 0,
        "_____10c-is_dcr-cx-de_gen-f": 0,
        "_____10c-is_dcr-cx-pspuni-b": True,
        "_____10c-is_dcr-cx-dstmap-d": {"is_dcr": [["decrease_filter_diagonal_one", [1,1,1], 1, False]],
                                        "ds_dcr": [["integrate_to_detect_decr_one", [1,1,1], 1, False]]
        },

        # Brightness Decrease Integration 2 (medium decrease)
        "_____10c-im_dcr-cx-__name-t": "med_decr_intgr",
        "_____10c-im_dcr-cx-_group-t": "vision",
        "_____10c-im_dcr-cx-_n_cnt-i": 1,
        "_____10c-im_dcr-nx-gd_vis-b": True,
        "_____10c-im_dcr-nx-rcordx-i": 45,
        "_____10c-im_dcr-nx-rcordy-i": 20,
        "_____10c-im_dcr-nx-rcordz-i": 0,
        "_____10c-im_dcr-nx-___bbx-i": 4,
        "_____10c-im_dcr-nx-___bby-i": 2,
        "_____10c-im_dcr-nx-___bbz-i": 1,
        "_____10c-im_dcr-cx-synatt-i": 100,
        "_____10c-im_dcr-cx-pstcr_-f": 50,
        "_____10c-im_dcr-cx-pstcrm-f": 200,
        "_____10c-im_dcr-cx-plst_c-f": 0.05,
        "_____10c-im_dcr-nx-fire_t-f": 49,
        "_____10c-im_dcr-nx-refrac-i": 0,
        "_____10c-im_dcr-nx-leak_c-f": 50,
        "_____10c-im_dcr-nx-c_fr_c-i": 0,
        "_____10c-im_dcr-nx-snooze-f": 0,
        "_____10c-im_dcr-cx-de_gen-f": 0,
        "_____10c-im_dcr-cx-pspuni-b": True,
        "_____10c-im_dcr-cx-dstmap-d": {"im_dcr": [["decrease_filter_diagonal_two", [1, 1, 1], 1, False]],
                                        "dm_dcr": [["integrate_to_detect_decr_two", [1, 1, 1], 1, False]]
                                        },

        # Brightness Decrease Integration 3 (threshold from bright to dark)
        "_____10c-il_dcr-cx-__name-t": "lrg_decr_intgr",
        "_____10c-il_dcr-cx-_group-t": "vision",
        "_____10c-il_dcr-cx-_n_cnt-i": 1,
        "_____10c-il_dcr-nx-gd_vis-b": True,
        "_____10c-il_dcr-nx-rcordx-i": 55,
        "_____10c-il_dcr-nx-rcordy-i": 20,
        "_____10c-il_dcr-nx-rcordz-i": 0,
        "_____10c-il_dcr-nx-___bbx-i": 4,
        "_____10c-il_dcr-nx-___bby-i": 2,
        "_____10c-il_dcr-nx-___bbz-i": 1,
        "_____10c-il_dcr-cx-synatt-i": 100,
        "_____10c-il_dcr-cx-pstcr_-f": 50,
        "_____10c-il_dcr-cx-pstcrm-f": 200,
        "_____10c-il_dcr-cx-plst_c-f": 0.05,
        "_____10c-il_dcr-nx-fire_t-f": 49,
        "_____10c-il_dcr-nx-refrac-i": 0,
        "_____10c-il_dcr-nx-leak_c-f": 50,
        "_____10c-il_dcr-nx-c_fr_c-i": 0,
        "_____10c-il_dcr-nx-snooze-f": 0,
        "_____10c-il_dcr-cx-de_gen-f": 0,
        "_____10c-il_dcr-cx-pspuni-b": True,
        "_____10c-il_dcr-cx-dstmap-d": {"il_dcr": [["decrease_filter_diagonal_three", [1, 1, 1], 1, False]],
                                        "dl_dcr": [["integrate_to_detect_decr_three", [1, 1, 1], 1, False]]
                                        },

        # Brightness Decrease Detection 1 (small decrease)
        "_____10c-ds_dcr-cx-__name-t": "small_decr_detect",
        "_____10c-ds_dcr-cx-_group-t": "vision",
        "_____10c-ds_dcr-cx-_n_cnt-i": 1,
        "_____10c-ds_dcr-nx-gd_vis-b": True,
        "_____10c-ds_dcr-nx-rcordx-i": 35,
        "_____10c-ds_dcr-nx-rcordy-i": 30,
        "_____10c-ds_dcr-nx-rcordz-i": 0,
        "_____10c-ds_dcr-nx-___bbx-i": 3,
        "_____10c-ds_dcr-nx-___bby-i": 1,
        "_____10c-ds_dcr-nx-___bbz-i": 1,
        "_____10c-ds_dcr-cx-synatt-i": 100,
        "_____10c-ds_dcr-cx-pstcr_-f": 100,
        "_____10c-ds_dcr-cx-pstcrm-f": 200,
        "_____10c-ds_dcr-cx-plst_c-f": 0.05,
        "_____10c-ds_dcr-nx-fire_t-f": 90,
        "_____10c-ds_dcr-nx-refrac-i": 0,
        "_____10c-ds_dcr-nx-leak_c-f": 100,
        "_____10c-ds_dcr-nx-c_fr_c-i": 0,
        "_____10c-ds_dcr-nx-snooze-f": 0,
        "_____10c-ds_dcr-cx-de_gen-f": 0,
        "_____10c-ds_dcr-cx-dstmap-d": {
        },

        # Brightness Decrease Detection 2 (medium decrease)
        "_____10c-dm_dcr-cx-__name-t": "med_decr_detect",
        "_____10c-dm_dcr-cx-_group-t": "vision",
        "_____10c-dm_dcr-cx-_n_cnt-i": 1,
        "_____10c-dm_dcr-nx-gd_vis-b": True,
        "_____10c-dm_dcr-nx-rcordx-i": 45,
        "_____10c-dm_dcr-nx-rcordy-i": 30,
        "_____10c-dm_dcr-nx-rcordz-i": 0,
        "_____10c-dm_dcr-nx-___bbx-i": 3,
        "_____10c-dm_dcr-nx-___bby-i": 1,
        "_____10c-dm_dcr-nx-___bbz-i": 1,
        "_____10c-dm_dcr-cx-synatt-i": 100,
        "_____10c-dm_dcr-cx-pstcr_-f": 100,
        "_____10c-dm_dcr-cx-pstcrm-f": 200,
        "_____10c-dm_dcr-cx-plst_c-f": 0.05,
        "_____10c-dm_dcr-nx-fire_t-f": 90,
        "_____10c-dm_dcr-nx-refrac-i": 0,
        "_____10c-dm_dcr-nx-leak_c-f": 100,
        "_____10c-dm_dcr-nx-c_fr_c-i": 0,
        "_____10c-dm_dcr-nx-snooze-f": 0,
        "_____10c-dm_dcr-cx-de_gen-f": 0,
        "_____10c-dm_dcr-cx-dstmap-d": {
        },

        # Brightness Detection Decrease 3 (threshold from bright to dark)
        "_____10c-dl_dcr-cx-__name-t": "lrg_decr_detect",
        "_____10c-dl_dcr-cx-_group-t": "vision",
        "_____10c-dl_dcr-cx-_n_cnt-i": 1,
        "_____10c-dl_dcr-nx-gd_vis-b": True,
        "_____10c-dl_dcr-nx-rcordx-i": 55,
        "_____10c-dl_dcr-nx-rcordy-i": 30,
        "_____10c-dl_dcr-nx-rcordz-i": 0,
        "_____10c-dl_dcr-nx-___bbx-i": 1,
        "_____10c-dl_dcr-nx-___bby-i": 1,
        "_____10c-dl_dcr-nx-___bbz-i": 1,
        "_____10c-dl_dcr-cx-synatt-i": 100,
        "_____10c-dl_dcr-cx-pstcr_-f": 100,
        "_____10c-dl_dcr-cx-pstcrm-f": 200,
        "_____10c-dl_dcr-cx-plst_c-f": 0.05,
        "_____10c-dl_dcr-nx-fire_t-f": 90,
        "_____10c-dl_dcr-nx-refrac-i": 0,
        "_____10c-dl_dcr-nx-leak_c-f": 100,
        "_____10c-dl_dcr-nx-c_fr_c-i": 0,
        "_____10c-dl_dcr-nx-snooze-f": 0,
        "_____10c-dl_dcr-cx-de_gen-f": 0,
        "_____10c-dl_dcr-cx-pspuni-b": True,
        "_____10c-dl_dcr-cx-dstmap-d": {
                                        },






        # +++ Steering by integrating sensory info and action pattern commands  +++
        "_____10c-_steer-cx-__name-t": "Steering",
        "_____10c-_steer-cx-_group-t": "vision",
        "_____10c-_steer-cx-_n_cnt-i": 1,
        "_____10c-_steer-nx-gd_vis-b": True,
        "_____10c-_steer-nx-rcordx-i": 80,
        "_____10c-_steer-nx-rcordy-i": 20,
        "_____10c-_steer-nx-rcordz-i": 0,
        "_____10c-_steer-nx-___bbx-i": 8,
        "_____10c-_steer-nx-___bby-i": 2,
        "_____10c-_steer-nx-___bbz-i": 1,
        "_____10c-_steer-cx-synatt-i": 100,
        "_____10c-_steer-cx-pstcr_-f": 90,
        "_____10c-_steer-cx-pstcrm-f": 100,
        "_____10c-_steer-cx-plst_c-f": 0.05,
        "_____10c-_steer-nx-fire_t-f": 90,
        "_____10c-_steer-nx-refrac-i": 0,
        "_____10c-_steer-nx-leak_c-f": 100,
        "_____10c-_steer-nx-c_fr_c-i": 0,
        "_____10c-_steer-nx-snooze-f": 0,
        "_____10c-_steer-cx-de_gen-f": 0,
        "_____10c-_steer-cx-dstmap-d": {"_right": [["steering_to_right", [1, 1, 1], 1, False]],
                                        "__left": [["steering_to_left", [1, 1, 1], 1, False]],
                                        "__forw": [["steering_to_forward", [1, 1, 1], 1, False]]},

        # +++ Towards Bright Action Pattern +++
        "_____10c-c___tb-cx-__name-t": "Toward_Bright",
        "_____10c-c___tb-cx-_group-t": "vision",
        "_____10c-c___tb-cx-_n_cnt-i": 1,
        "_____10c-c___tb-nx-gd_vis-b": True,
        "_____10c-c___tb-nx-rcordx-i": 80,
        "_____10c-c___tb-nx-rcordy-i": 30,
        "_____10c-c___tb-nx-rcordz-i": 0,
        "_____10c-c___tb-nx-___bbx-i": 1,
        "_____10c-c___tb-nx-___bby-i": 1,
        "_____10c-c___tb-nx-___bbz-i": 1,
        "_____10c-c___tb-cx-synatt-i": 100,
        "_____10c-c___tb-cx-pstcr_-f": 50,
        "_____10c-c___tb-cx-pstcrm-f": 100,
        "_____10c-c___tb-cx-plst_c-f": 0.05,
        "_____10c-c___tb-nx-fire_t-f": 5,
        "_____10c-c___tb-nx-refrac-i": 0,
        "_____10c-c___tb-nx-leak_c-f": 10,
        "_____10c-c___tb-nx-c_fr_c-i": 0,
        "_____10c-c___tb-nx-snooze-f": 0,
        "_____10c-c___tb-cx-de_gen-f": 0,
        "_____10c-c___tb-cx-pspuni-b": True,
        "_____10c-c___tb-cx-dstmap-d": {"_steer": [["one_to_row_one", [1, 1, 1], 1, False]]},

        # +++ Towards Darkness Action Pattern +++
        "_____10c-c___ab-cx-__name-t": "Avoid_Bright",
        "_____10c-c___ab-cx-_group-t": "vision",
        "_____10c-c___ab-cx-_n_cnt-i": 1,
        "_____10c-c___ab-nx-gd_vis-b": True,
        "_____10c-c___ab-nx-rcordx-i": 90,
        "_____10c-c___ab-nx-rcordy-i": 30,
        "_____10c-c___ab-nx-rcordz-i": 0,
        "_____10c-c___ab-nx-___bbx-i": 1,
        "_____10c-c___ab-nx-___bby-i": 1,
        "_____10c-c___ab-nx-___bbz-i": 1,
        "_____10c-c___ab-cx-synatt-i": 100,
        "_____10c-c___ab-cx-pstcr_-f": 50,
        "_____10c-c___ab-cx-pstcrm-f": 100,
        "_____10c-c___ab-cx-plst_c-f": 0.05,
        "_____10c-c___ab-nx-fire_t-f": 5,
        "_____10c-c___ab-nx-refrac-i": 0,
        "_____10c-c___ab-nx-leak_c-f": 10,
        "_____10c-c___ab-nx-c_fr_c-i": 0,
        "_____10c-c___ab-nx-snooze-f": 0,
        "_____10c-c___ab-cx-de_gen-f": 0,
        "_____10c-c___ab-cx-pspuni-b": True,
        "_____10c-c___ab-cx-dstmap-d": {"_steer": [["one_to_row_zero", [1, 1, 1], 1, False]]},

        # +++ Right turn inducer +++
        "_____10c-_right-cx-__name-t": "right_activate",
        "_____10c-_right-cx-_group-t": "motion",
        "_____10c-_right-cx-_n_cnt-i": 1,
        "_____10c-_right-nx-gd_vis-b": True,
        "_____10c-_right-nx-rcordx-i": 95,
        "_____10c-_right-nx-rcordy-i": 10,
        "_____10c-_right-nx-rcordz-i": 0,
        "_____10c-_right-nx-___bbx-i": 1,
        "_____10c-_right-nx-___bby-i": 1,
        "_____10c-_right-nx-___bbz-i": 1,
        "_____10c-_right-cx-synatt-i": 100,
        "_____10c-_right-cx-pstcr_-f": 100,
        "_____10c-_right-cx-pstcrm-f": 200,
        "_____10c-_right-cx-plst_c-f": 0.05,
        "_____10c-_right-nx-fire_t-f": 5,
        "_____10c-_right-nx-refrac-i": 0,
        "_____10c-_right-nx-leak_c-f": 0,
        "_____10c-_right-nx-c_fr_c-i": 0,
        "_____10c-_right-nx-snooze-f": 0,
        "_____10c-_right-cx-de_gen-f": 0,
        "_____10c-_right-cx-dstmap-d": {
            "o__mot": [["turn_right_e", [1, 1, 1], 1, False]]
        },

        # +++ Left turn inducer +++
        "_____10c-__left-cx-__name-t": "left_inducer",
        "_____10c-__left-cx-_group-t": "motion",
        "_____10c-__left-cx-_n_cnt-i": 1,
        "_____10c-__left-nx-gd_vis-b": True,
        "_____10c-__left-nx-rcordx-i": 75,
        "_____10c-__left-nx-rcordy-i": 10,
        "_____10c-__left-nx-rcordz-i": 0,
        "_____10c-__left-nx-___bbx-i": 1,
        "_____10c-__left-nx-___bby-i": 1,
        "_____10c-__left-nx-___bbz-i": 1,
        "_____10c-__left-cx-synatt-i": 100,
        "_____10c-__left-cx-pstcr_-f": 100,
        "_____10c-__left-cx-pstcrm-f": 200,
        "_____10c-__left-cx-plst_c-f": 0.05,
        "_____10c-__left-nx-fire_t-f": 5,
        "_____10c-__left-nx-refrac-i": 0,
        "_____10c-__left-nx-leak_c-f": 0,
        "_____10c-__left-nx-c_fr_c-i": 0,
        "_____10c-__left-nx-snooze-f": 0,
        "_____10c-__left-cx-de_gen-f": 0,
        "_____10c-__left-cx-dstmap-d": {
            "o__mot": [["turn_left_e", [1, 1, 1], 1, False]],
        },

        # +++ Forward motion inducer +++
        "_____10c-__forw-cx-__name-t": "forward_activate",
        "_____10c-__forw-cx-_group-t": "motion",
        "_____10c-__forw-cx-_n_cnt-i": 1,
        "_____10c-__forw-nx-gd_vis-b": True,
        "_____10c-__forw-nx-rcordx-i": 85,
        "_____10c-__forw-nx-rcordy-i": 10,
        "_____10c-__forw-nx-rcordz-i": 0,
        "_____10c-__forw-nx-___bbx-i": 1,
        "_____10c-__forw-nx-___bby-i": 1,
        "_____10c-__forw-nx-___bbz-i": 1,
        "_____10c-__forw-cx-synatt-i": 100,
        "_____10c-__forw-cx-pstcr_-f": 100,
        "_____10c-__forw-cx-pstcrm-f": 200,
        "_____10c-__forw-cx-plst_c-f": 0.05,
        "_____10c-__forw-nx-fire_t-f": 5,
        "_____10c-__forw-nx-refrac-i": 0,
        "_____10c-__forw-nx-leak_c-f": 0,
        "_____10c-__forw-nx-c_fr_c-i": 0,
        "_____10c-__forw-nx-snooze-f": 0,
        "_____10c-__forw-cx-de_gen-f": 0,
        "_____10c-__forw-cx-dstmap-d": {
            "o__mot": [["forward_e", [1, 1, 1], 1, False]]
        },
        # +++ Backward motion inducer +++
        "_____10c-__back-cx-__name-t": "escape",
        "_____10c-__back-cx-_group-t": "motion",
        "_____10c-__back-cx-_n_cnt-i": 1,
        "_____10c-__back-nx-gd_vis-b": True,
        "_____10c-__back-nx-rcordx-i": 105,
        "_____10c-__back-nx-rcordy-i": 10,
        "_____10c-__back-nx-rcordz-i": 0,
        "_____10c-__back-nx-___bbx-i": 3,
        "_____10c-__back-nx-___bby-i": 1,
        "_____10c-__back-nx-___bbz-i": 1,
        "_____10c-__back-cx-synatt-i": 100,
        "_____10c-__back-cx-pstcr_-f": 100,
        "_____10c-__back-cx-pstcrm-f": 200,
        "_____10c-__back-cx-plst_c-f": 0.05,
        "_____10c-__back-nx-fire_t-f": 5,
        "_____10c-__back-nx-refrac-i": 0,
        "_____10c-__back-nx-leak_c-f": 0,
        "_____10c-__back-nx-c_fr_c-i": 0,
        "_____10c-__back-nx-snooze-f": 0,
        "_____10c-__back-cx-de_gen-f": 0,
        "_____10c-__back-cx-pspuni-b": True,

        "_____10c-__back-cx-dstmap-d": {
            "o__mot": [["backward_e", [1, 1, 1], 1, False]],
            "__back": [["lat_+x", [1,1,1],1,False]]
        },

        # Motor output region
        "_____10c-o__mot-cx-__name-t": "motor_opu",
        "_____10c-o__mot-cx-_group-t": "OPU",
        "_____10c-o__mot-cx-_n_cnt-i": 1,
        "_____10c-o__mot-nx-gd_vis-b": True,
        "_____10c-o__mot-nx-rcordx-i": 80,
        "_____10c-o__mot-nx-rcordy-i": 0,
        "_____10c-o__mot-nx-rcordz-i": 0,
        "_____10c-o__mot-nx-___bbx-i": 8,
        "_____10c-o__mot-nx-___bby-i": 1,
        "_____10c-o__mot-nx-___bbz-i": 10,
        "_____10c-o__mot-cx-synatt-i": 100,
        "_____10c-o__mot-cx-pstcr_-f": 6,
        "_____10c-o__mot-cx-pstcrm-f": 35,
        "_____10c-o__mot-cx-plst_c-f": 0.05,
        "_____10c-o__mot-nx-fire_t-f": 20,
        "_____10c-o__mot-nx-refrac-i": 0,
        "_____10c-o__mot-nx-leak_c-f": 0,
        "_____10c-o__mot-nx-c_fr_c-i": 3,
        "_____10c-o__mot-nx-snooze-f": 0,
        "_____10c-o__mot-cx-de_gen-f": 0.01,
        "_____10c-o__mot-cx-dstmap-d": {
        },

    }
}