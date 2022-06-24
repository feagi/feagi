
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

"""
This module is responsible for the processes involved with the death of a brain. It will involve the needed cleanups and
preparations so a new brain can come into life.
"""

from inf import runtime_data
from inf.initialize import reset_runtime_data


def cleanup():
    """
    Clears all the runtime variables and registers associated with a running brain
    """
    # Stop all IPU activities

    # Clear all existing brain activities
    runtime_data.previous_fcl = {}
    runtime_data.fire_candidate_list = {}
    runtime_data.future_fcl = {}

    # Clear connectome data
    runtime_data.brain = {}

    # Clear genome data
    runtime_data.genome = {}

    # Reset all other runtime-data
    runtime_data.plasticity_dict = {}


def announce():
    """
    Reports the death of a brain and FEAGI readiness to host a new one
    """
    runtime_data.beacon_flag = True


def death_eligibility():
    """
    Evaluates the conditions leading to brain death
    """
    death_condition = False
    age_condition = False

    # Conditions triggering the brain death

    # --- Age related ---
    if 'max_age' in runtime_data.genome:
        if runtime_data.current_age > runtime_data.genome['max_age']:
            death_condition = True
            print('\n\n\nAge related death has been triggered!\n\n\n')

    # --- Stimulation related ---
    if '_death' in runtime_data.previous_fcl:
        if len(runtime_data.previous_fcl['_death']) > 0:
            death_condition = True
            print('\n\n\nCortical stimulation death has been triggered!\n\n\n')

    # --- Performance related ---

    # --- Inactivity related ---

    return death_condition


def death_manager():
    """
    Manages activities associated with brain death
    """
    if death_eligibility():
        for x in range(5):
            print('*' * 50)
        print('\n\t\tBrain death condition has been met!\n')
        for x in range(5):
            print('*' * 50)

        # ------- Preserve Connectome -------

        # ------- Reset Runtime Data -------
        reset_runtime_data()
        runtime_data.death_flag = True
        runtime_data.feagi_state["state"] = "idle"
