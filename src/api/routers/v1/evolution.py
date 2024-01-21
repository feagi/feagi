from fastapi import APIRouter

from ....inf import runtime_data
from ....evo import autopilot

router = APIRouter()


# ######  Evolution #########
# #############################

@router.get("/v1/feagi/evolution/autopilot/status")
async def return_autopilot_status():
    """
    Returns the status of genome autopilot system.
    """

    if runtime_data.autopilot:
        return runtime_data.autopilot
    else:
        return False


@router.post("/v1/feagi/evolution/autopilot/on")
async def turn_autopilot_on():
    if not runtime_data.autopilot:
        autopilot.init_generation_dict()
        if runtime_data.brain_run_id:
            autopilot.update_generation_dict()
        runtime_data.autopilot = True
        print("<" * 30, "  Autopilot has been turned on  ", ">" * 30)


@router.post("/v1/feagi/evolution/autopilot/off", tags=["Evolution"])
async def turn_autopilot_off():
    runtime_data.autopilot = False
    return


@router.get("/v1/feagi/evolution/generations")
async def list_generations():
    """
    Return details about all generations.
    """

    if runtime_data.generation_dict:
        return runtime_data.generation_dict
    else:
        return {}


@router.get("/v1/feagi/evolution/change_register")
async def list_generations():
    """
    Return details about all generations.
    """

    if runtime_data.evo_change_register:
        return runtime_data.evo_change_register
    else:
        return {}
