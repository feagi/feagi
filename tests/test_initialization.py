"""
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
"""


import requests


def test():
    try:
        data = requests.get('http://127.0.0.1:8000' + '/v1/feagi/feagi/burst_engine/stimulation_period')
        if data.status_code == 200:
            print("FEAGI is reachable and is not having any issue")
            return "OK"
        else:
            return "FAILED"
    except Exception as e:
        err = "ERROR AT: " + str(e)
        return err


result = test()
if result == "OK":
    pass
else:
    print(result)
    print(5 / 0)
