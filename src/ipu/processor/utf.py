
# Copyright 2019 The FEAGI Authors. All Rights Reserved.
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


from inf import runtime_data


def convert_char_to_fire_list(char):
    utf_value = ord(char)
    fire_set = set()
    for key in runtime_data.brain["utf8_ipu"]:
        if utf_value == runtime_data.brain["utf8_ipu"][key]['soma_location'][0][2]:
            fire_set.add(key)
    return fire_set
