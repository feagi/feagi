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


from fastapi import APIRouter, HTTPException

from ...schemas import *
from ...commons import *

from feagi.core.state_manager import FeagiStateManager


router = APIRouter()

# Helper to get state manager instance
state = FeagiStateManager.instance()


# ######  Training Endpoints #######
# ##################################

@router.delete("/reset_fitness_stats")
async def delete_fitness_stats_from_db():
    """
    Erases the fitness statistics from the database.
    """

    state.influxdb.drop_fitness_activity()


@router.post("/gameover")
async def gameover_signal():
    """
    Captures feedback from the environment during training
    """
    message = {'gameover': True}
    api_queue.put(item=message)


@router.get("/training_report")
async def training_report():
    """
    Returns stats associated with training
    """
    return state.training_stats


@router.get("/brain_fitness")
async def brain_average_fitness_value():
    """
    Calculates fitness score based on the defined fitness criteria
    """
    cumulative_score = 0
    try:
        counter = 0
        for stat in state.fitness_stats:
            fitness_score = 0
            counter += 1
            criteria = stat.get("FITNESS_KEYS")
            if criteria:
                for criterion in stat["FITNESS_KEYS"]:
                    if criterion not in state.fitness_criteria:
                        raise HTTPException(status_code=400, detail=f"{criterion} is not defied as a fitness criteria!"
                                                                    f"Use post:/fitness_criteria to define it first.")
                    fitness_score += stat["FITNESS_KEYS"][criterion] * state.fitness_criteria[criterion]
            cumulative_score += fitness_score

        if counter == 0:
            return 1
        else:
            state.fitness_score = cumulative_score / counter
            return state.fitness_score
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Error during fitness calculation as {e}")


@router.get("/fitness_criteria")
async def fetch_fitness_criteria():
    """
    Returns the effective fitness criteria
    """
    return state.fitness_criteria


@router.post("/fitness_criteria")
async def configure_fitness_criteria(fitness_criteria: dict):
    """
    Configure the weights associated with each fitness criteria.

    Important: Total weights has to equal to 1.

    Actual game stats will be weighted based on the defined criteria and produce a single fitness value between 0 and 1.
    ```json
    {
        "time_alive": 0.4,
        "max_level_reached": 0.2,
        "score_trying_to_max": 1.0,
        "score_trying_to_min": -1.0,
        "something_custom": 0.4
    }
    ```

    """

    key_sum = 0
    for criterion in fitness_criteria:
        key_sum += fitness_criteria[criterion]

    if key_sum != 1:
        raise HTTPException(status_code=400, detail=f"The sum of all FITNESS_KEYS should be equal to 1")

    state.fitness_criteria = fitness_criteria


@router.get("/fitness_stats")
async def reset_fitness_stats():
    """
    Resets fitness stats
    """
    return state.fitness_stats


@router.put("/fitness_stats")
async def capture_fitness_stats_instance(fitness_stats: dict):
    """
    updates fitness stats. Data should be in a dictionary form following the structure defined under /fitness_criteria

    Sample:
    ```json
    {
    "FITNESS_KEYS":
        {
            "time_alive": 672,
            "max_level_reached": 2,
            "score_trying_to_max": 78,
            "score_trying_to_min": 42,
            "something_custom": 23
        },
    "METADATA":
        {
            "event": "changed such and such environment variable"
        }
    }
    ```
    Note: Metadata is optional and to provide additional context.

    """

    if "FITNESS_KEYS" not in fitness_stats:
        raise HTTPException(status_code=400, detail=f"FITNESS_KEYS is not defined as a dictionary key")

    if "METADATA" not in fitness_stats:
        fitness_stats["METADATA"] = {}

    state.fitness_stats.append(fitness_stats)


@router.delete("/fitness_stats")
async def reset_fitness_stats():
    """
    Resets fitness stats
    """
    state.fitness_stats = []
