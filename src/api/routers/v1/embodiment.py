import os
from fastapi import APIRouter

from ...schemas import *
from ...commons import *


router = APIRouter()


# #########  Robot   ###########
# ##############################
@router.post("/v1/robot/parameters")
async def robot_controller_tunner(message: RobotController):
    """
    Enables changes against various Burst Engine parameters.
    """

    message = message.dict()
    message = {'robot_controller': message}
    api_queue.put(item=message)


@router.post("/v1/robot/model")
async def robot_model_modification(message: RobotModel):
    """
    Enables changes against various Burst Engine parameters.
    """

    message = message.dict()
    message = {'robot_model': message}
    api_queue.put(item=message)


@router.get("/v1/feagi/robot/gazebo/files")
async def gazebo_robot_default_files():

    default_robots_path = "./evo/defaults/robot/"
    default_robots = os.listdir(default_robots_path)
    return {"robots": default_robots}
