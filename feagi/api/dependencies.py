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

from fastapi import HTTPException, Depends
from feagi.core.state_manager import FeagiStateManager

# FEAGI state check
# Genome running conditions
# Agent connectivity

# Helper to get state manager instance
state = FeagiStateManager.instance()

def check_burst_engine():
    if state.is_burst_engine_ready():
        return True
    else:
        raise HTTPException(status_code=400, detail="Burst engine is not running!")


def check_active_genome():
    """Check if there is an active genome loaded."""
    state_manager = FeagiStateManager.instance()
    
    # Check if genome is loaded based on state manager's genome state
    if not state_manager.is_genome_loaded():
        raise HTTPException(status_code=400, detail="No active genome found! Load a genome first.")
    
    return True


def check_brain_running(_: bool = Depends(check_active_genome)):
    if state.get_brain_readiness():
        return True
    else:
        raise HTTPException(status_code=400, detail="Brain not yet ready! Please try again later.")
