
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

# todo: Need cohesiveness between content here and what defined under genome_processor.py

gui_baseline = {
    'ipu': {
        'i__inf',
        'i__pro'
    },
    'opu': {
        'o__mot',
        'o__ser'
    },
    "morphology": {
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
    "morphology_scalar": [1, 1, 1],
    "postSynapticCurrent_multiplier": 1,
    "plasticity_flag": False,
    "cortical_genes": {
        "per_voxel_neuron_cnt (int)": ["cx-_n_cnt-i", 1],
        "synapse_attractivity (int)": ["cx-synatt-i", 100],
        "postsynaptic_current (float)": ["nx-pstcr_-f", 5.0],
        "postsynaptic_current_max (float)": ["nx-pstcrm-f", 35.0], 
        "plasticity_constant (float)": ["nx-plst_c-f", 0.05],
        "firing_threshold (float)": ["nx-fire_t-f", 1.0],
        "refractory_period (int)": ["nx-refrac-i", 0],
        "leak_coefficient (float)": ["nx-leak_c-f", 0],
        "consecutive_fire_cnt_max (int)": ["nx-c_fr_c-i", 3],
        "snooze_length (float)": ["nx-snooze-f", 0],
        "group_id (str)": ["cx-_group-t", "null"],
        "degeneration (float)": ["cx-de_gen-f", 0]
    }
}
