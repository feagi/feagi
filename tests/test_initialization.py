
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

import pytest
import feagi_configuration
from evo import neuroembryogenesis
from inf import initialize, runtime_data


def test_feagi_initialization():
    # ensure runtime_data parameters are updated upon initialization
    assert init_parameters()
    assert initialize.init_infrastructure()
    assert runtime_data.brain_run_id
    assert runtime_data.parameters
    assert runtime_data.working_directory
    assert runtime_data.connectome_path

    # confirm genome creation using static genome
    assert runtime_data.parameters['Switches']['use_static_genome']
    assert runtime_data.genome
    assert runtime_data.genome_id

    # suppress connectome figure generation during testing
    runtime_data.parameters['Visualization']['connectome_visualizer'] = False

    # ensure completion of brain development following neuroembryogenesis
    neuroembryogenesis.develop_brain(reincarnation_mode=False)
    assert runtime_data.brain
