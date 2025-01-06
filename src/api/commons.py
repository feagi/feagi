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

from time import time
from queue import Queue
from datetime import datetime

from src.inf import runtime_data


api_queue = Queue()


class CustomError(Exception):
    def __init__(self, message, status_code):
        self.message = message
        self.status_code = status_code
        print(message, status_code)


def pending_amalgamation():
    if not runtime_data.pending_amalgamation:
        return False
    else:
        # todo: externalize the amalgamation timeout currently hardcoded below
        amalgamation_reqeust_timeout = 500
        elapsed_time = time() - runtime_data.pending_amalgamation["initiation_time"]
        if elapsed_time > amalgamation_reqeust_timeout:
            print(f"Pending amalgamation got voided due to exceeding {amalgamation_reqeust_timeout} threshold! ")
            runtime_data.pending_amalgamation = {}
            return False
        else:
            return True


def cancel_pending_amalgamation(amalgamation_id):
    runtime_data.pending_amalgamation = {}
    if amalgamation_id in runtime_data.amalgamation_history:
        runtime_data.amalgamation_history[amalgamation_id] = "cancelled"
