
# Copyright 2016-2024 The FEAGI Authors. All Rights Reserved.
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
This module covers needed functions for brain region management
"""

import string
import random
import datetime


def region_id_gen(size=6, chars=string.ascii_uppercase + string.digits):

    now = datetime.datetime.now()
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return str(now.strftime("%Y%m%d%H%M%S%f")[2:]) + '_' + (''.join(random.choice(chars) for _ in range(size))) + '_R'


def create_region():
    pass


def delete_region():
    pass






