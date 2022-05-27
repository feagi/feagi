
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

stimulations = {
    "IR_pain": {
        "start_burst": None,
        "end_burst": None,
        "definition": [
            {"i__pro": ["0-0-3"], "o__mot": ["2-0-7"]},
            {"i__pro": ["0-0-8"]},
            {"i__bat": ["0-0-7"]},
            {"i__bat": ["0-0-6"]},
            {"i__bat": ["0-0-5"]},
            {"i__bat": ["0-0-4"]},
            {"i__bat": ["0-0-3"]},
            {},
            {"i__bat": ["0-0-2"]},
            {"i__bat": ["0-0-1"]},
            {},
            {}
            ]
    },
    "exploration": {
        "start_burst": None,
        "end_burst": None,
        "definition": []
    },
    "move_forward": {
        "start_burst": None,
        "end_burst": None,
        "definition": []
    },
    "charge_batteries": {
        "start_burst": None,
        "end_burst": None,
        "definition": [
            {"i__inf": ["2-0-0"]}
        ]
    }
}
