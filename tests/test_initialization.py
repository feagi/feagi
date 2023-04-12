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
    """
    A simple function to ping the FEAGI's endpoints.
    """

    try:
        files = {'file': open('genome_test/static_genome.json', 'rb')}
        response = requests.post('http://127.0.0.1:8000/v1/feagi/genome/upload/file', files=files)
        data = requests.get('http://127.0.0.1:8000' + '/v1/feagi/feagi/burst_engine/stimulation_period')
        if response.status_code == 200:
            print("Loaded OK!")
        if data.status_code == 200:
            print("FEAGI is reachable and is not having any issue")
            return "OK"
    except Exception as error:
        err = "ERROR AT: " + str(error)
        return err


RESULT = test()
if RESULT == "OK":
    pass
else:
    print(RESULT)
    print(5 / 0)
