# Copyright 2016-2024 Neuraville Inc. Authors. All Rights Reserved.
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
from src.inf import runtime_data

# FEAGI state check
# Genome running conditions
# Agent connectivity


def check_burst_engine():
    burst_engine_running = not runtime_data.exit_condition
    if burst_engine_running:
        return True
    else:
        raise HTTPException(status_code=400, detail="Burst engine is not running!")


def check_active_genome(_: bool = Depends(check_burst_engine)):
    if runtime_data.genome:
        return True
    else:
        raise HTTPException(status_code=400, detail="No active genome found! Load a genome first.")


def check_brain_running(_: bool = Depends(check_active_genome)):
    if runtime_data.brain_readiness:
        return True
    else:
        raise HTTPException(status_code=400, detail="Brain not yet ready! Please try again later.")
