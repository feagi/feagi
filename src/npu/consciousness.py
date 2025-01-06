
#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
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

from src.inf import runtime_data


def set_brain_readiness_to_false(context=None):
    runtime_data.brain_readiness = False
    print(f"⚠️ !!! Brain readiness is set to False !!! {context}")


def set_brain_readiness_to_ture():
    runtime_data.brain_readiness = True
    print("🟩 Brain is now in ready state.")


def start():
    return


def candidate_list_counter(candidate_list):
    count = 0
    for cortical_area in candidate_list:
        count += len(candidate_list[cortical_area])
        # print("&&$$%%>>", cortical_area, len(candidate_list[cortical_area]))
    return count


def stop():
    return

