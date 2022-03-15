
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
The fake_stimulation will provide one or multiple stimulations that can be played in sequence against the brain.
each instance of the list will be injected to a single burst. if a sequence is left empty then the corresponding burst
sequence will go without injection.

Note: This method can help us record brain activities and have them replayed back.

Every key in the stimulation_data dictionary is corresponding to the corresponding burst_id that the stimuli exposure is
to happen.

# todo: need to introduce a mechanism so a sequence can be replaced in a loop


Fake Stimulation 2.0 additions:

- Ability to define fake stimulation using multiple methods and have them combined as one data structure


"""

stimulation_pattern = {
    "o__mot": [
        [['0-0-3', '2-0-3', '4-0-3',  '6-0-3'], [3, 15]],
        [['0-0-6', '2-0-6', '4-0-6', '6-0-6'], [16, 25]],
        [['0-0-9', '2-0-9', '4-0-9', '6-0-9'], [26, 35]],
    ],
    "o__ser": [
        [['0-0-9'], [5, 10]],
        [['0-0-7', '1-0-7'], [11, 20]],
    ],
    "i__pro": [
        [['0-0-0'], [2, 20]]
    ]
}


raw_stimulation = {
    3: {"i__pro": ["0-0-3"], "o__mot": []},
    5: {"i__pro": ["0-0-8"]},
    13: {"i__bat": ["0-0-7"]},
    14: {"i__bat": ["0-0-6"]},
    15: {"i__bat": ["0-0-5"]},
    16: {"i__bat": ["0-0-4"]},
    17: {"i__bat": ["0-0-3"]},
    18: {"i__bat": ["0-0-2"]},
    19: {"i__bat": ["0-0-1"]},
    }
