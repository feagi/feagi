
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

import sys
import traceback
from inf import runtime_data


def motor_operator(motor_data):
    """
    motor_data = {
        motor_id: {
            "direction": 'F',
            "...": ...
        },
        2: {},
        3: {}
    }
    """
    try:
        # todo: Generalize the following section. using specifics for test only
        if motor_data:
            message_to_router = {"motor": motor_data}
            runtime_data.opu_pub.send(message=message_to_router)
            if runtime_data.parameters["Logs"]["print_burst_info"]:
                for motor_id in motor_data:
                    print(f">>>>>>>>>>>>>>>>>>>>>>>> motor {motor_id} "
                        f"activation command sent to router with direction {motor_data[motor_id]}")

    except Exception as e:
        print("ERROR at Motor module:", e)
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
