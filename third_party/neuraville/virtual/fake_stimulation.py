
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

"""

stimulation_data = {
    # 11: {"servo_opu": ["0-0-90", "1-0-0"]},
    # 15: {"servo_opu": ["0-0-90", "1-0-90"]},
    # 20: {"servo_opu": ["0-0-90", "1-0-180"]},
    # 25: {"servo_opu": ["0-0-90", "1-0-90"]},
    # 30: {"servo_opu": ["0-0-0", "1-0-90"]},
    # 35: {"servo_opu": ["0-0-90", "1-0-90"]},
    # 40: {"servo_opu": ["0-0-180", "1-0-90"]},
    11: {"i__bat": ["0-0-9"]},
    12: {"i__bat": ["0-0-8"]},
    13: {"i__bat": ["0-0-7"]},
    14: {"i__bat": ["0-0-6"]},
    15: {"i__bat": ["0-0-5"]},
    16: {"i__bat": ["0-0-4"]},
    17: {"i__bat": ["0-0-3"]},
    18: {"i__bat": ["0-0-2"]},
    19: {"i__bat": ["0-0-1"]},
    # 11: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # 12: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # 13: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 14: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 15: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 16: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 17: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 5: {"i__bat": ["0-0-0"]},
    # 10: {"i__bat": ["0-0-1", "0-0-2", "0-0-3", "0-0-4", "0-0-0"]},
    # 15: {"i__bat": ["0-0-1", "0-0-2", "0-0-3", "0-0-4", "0-0-5"]},
    # 20: {"o__mot": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # #27: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # #28: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # #29: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 30: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 31: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 32: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 33: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 34: {"motor_opu": ["2-0-15", "0-0-5", "0-0-11", "3-0-0", "1-0-9"]},
    # 35: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # 36: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # 37: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 38: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 39: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 40: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 41: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 42: {"motor_opu": ["2-0-15", "0-0-5", "0-0-11", "3-0-0", "1-0-9"]},
    # 43: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # 44: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # 45: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 46: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 47: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 48: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 49: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 50: {"motor_opu": ["2-0-15", "0-0-5", "0-0-11", "3-0-0", "1-0-9"]},
    # 11: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 12: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 13: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 14: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 15: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 16: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 17: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 18: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 19: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 20: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 11: {"motor_thalamus": ["0-0-0"]},
    # 12: {"motor_thalamus": ["0-0-1"]},
    # 13: {"motor_thalamus": ["0-0-2"]},
    # 14: {"motor_thalamus": ["0-0-3"]},
    # 15: {"motor_thalamus": ["0-0-0"]},
    # 16: {"motor_thalamus": ["0-0-1"]},
    # 17: {"motor_thalamus": ["0-0-2"]},
    # 18: {"motor_thalamus": ["0-0-3"]},
    # 19: {"motor_thalamus": ["0-0-0"]},
    # 20: {"motor_thalamus": ["0-0-1"]},
    # 21: {"motor_thalamus": ["0-0-2"]},
    # 22: {"motor_thalamus": ["0-0-3"]},
    # 23: {"motor_thalamus": ["0-0-0"]},
    # 24: {"motor_thalamus": ["0-0-1"]},
    # 25: {"motor_thalamus": ["0-0-2"]},
    # 26: {"motor_thalamus": ["0-0-3"]},
    # 27: {"motor_thalamus": ["0-0-0"]},
    # 28: {"motor_thalamus": ["0-0-1"]},
    # 29: {"motor_thalamus": ["0-0-2"]},
    # 30: {"motor_thalamus": ["0-0-3"]},
    }
