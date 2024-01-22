from fastapi import APIRouter, HTTPException

from ...schemas import *
from ...commons import *

from src.inf import runtime_data


router = APIRouter()


# ######  Training Endpoints #######
# ##################################

@router.delete("/reset_game_stats")
async def delete_game_stats_from_db():
    """
    Erases the game statistics from the database.
    """

    runtime_data.influxdb.drop_game_activity()


@router.get("/shock/options")
async def list_available_shock_scenarios():
    """
    Get a list of available shock scenarios.
    """
    if runtime_data.shock_scenarios_options:
        return runtime_data.shock_scenarios_options


@router.get("/shock/status")
async def list_activated_shock_scenarios():
    if runtime_data.shock_scenarios:
        return runtime_data.shock_scenarios
    else:
        raise HTTPException(status_code=400, detail="No shock scenario is defined")


@router.post("/shock/activate")
async def activate_shock_scenarios(shock: Shock):
    """
    Enables shock for given scenarios. One or many shock scenario could coexist. e.g.

    {
      "shock": [
        "shock_scenario_1",
        "shock_scenario_2"
      ]
    }

    """
    print("----Shock API----")
    message = shock.dict()
    print(message)
    api_queue.put(item=message)


@router.post("/reward")
async def reward_intensity(intensity: Intensity):
    """
    Captures feedback from the environment during training
    """

    message = {'reward': intensity.intensity}
    api_queue.put(item=message)


@router.post("/punishment")
async def punishment_intensity(intensity: Intensity):
    """
    Captures feedback from the environment during training
    """

    message = {'punishment': intensity.intensity}
    api_queue.put(item=message)


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
    return runtime_data.training_stats
