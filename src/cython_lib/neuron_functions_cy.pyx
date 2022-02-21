
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

def neuron_update(float presynaptic_current,
                  int burst_count,
                  int last_membrane_potential_update,
                  float leak_coefficient,
                  float membrane_potential):
    """
    This function's only task is to update the membrane potential of the destination neuron.
    """

    # Leaky behavior
    if leak_coefficient > 0:
        if last_membrane_potential_update < burst_count:
            leak_window = burst_count - last_membrane_potential_update - 1
            leak_value = leak_window * leak_coefficient
            membrane_potential -= leak_value
            if membrane_potential < 0:
                membrane_potential = 0

    # Increasing the cumulative counter on destination based on the received signal from upstream Axon
    # The following is considered as LTP or Long Term Potentiation of Neurons

    membrane_potential += presynaptic_current

    return membrane_potential
