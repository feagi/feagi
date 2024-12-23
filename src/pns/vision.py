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


def collect_vision_configuration():
    vision_configuration = {
        "central_vision_resolution": (64, 64),
        "peripheral_vision_resolution": (8, 8),
        "flicker_period": 16,
        "color_vision": True,
        "eccentricity": (0.4, 0.6),
        "modulation": (0.4, 0.6),
        "brightness": 0.2,
        "contrast": 0.6,
        "shadows": 0.1,
        "pixel_change_limit": 0.2,
    }

    return vision_configuration


def reconfigure_vision(vision_parameters):
    pass
