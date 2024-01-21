from fastapi import APIRouter
from ....inf import runtime_data
from ...schemas import BurstEngine
from ...commons import *


router = APIRouter()


# ######  Burst-Engine Endpoints #########
# ########################################
@router.api_route("/v1/feagi/feagi/burst_engine/burst_counter", methods=['GET'])
async def burst_engine_params():
    """
    Return the number associated with current FEAGI burst instance.
    """
    if runtime_data.burst_count:
        return runtime_data.burst_count


@router.api_route("/v1/feagi/feagi/burst_engine/stimulation_period", methods=['GET'])
async def burst_engine_params():
    """
    Returns the time it takes for each burst to execute in seconds.
    """
    if runtime_data.burst_timer:
        return runtime_data.burst_timer


@router.api_route("/v1/feagi/feagi/burst_engine", methods=['POST'])
async def burst_management(message: BurstEngine):
    """
    Enables changes against various Burst Engine parameters.
    """

    message = message.dict()
    message = {'burst_management': message}
    api_queue.put(item=message)
