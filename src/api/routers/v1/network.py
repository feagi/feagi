from fastapi import APIRouter, HTTPException

from ...commons import *

router = APIRouter()


# ######  Networking Endpoints #########
# ##################################

@router.get("/v1/feagi/feagi/network")
async def network_management():
    if runtime_data.parameters['Sockets']:
        return runtime_data.parameters['Sockets']
    else:
        raise HTTPException(status_code=400, detail=f"Networking data not available!")


# @router.api_route("/v1/feagi/feagi/network", methods=['POST'], tags=["Networking"])
# async def network_management(message: Network):
#     try:
#         message = message.dict()
#         message = {'network_management': message}
#         api_queue.put(item=message)
#         return runtime_data.parameters['Sockets']
#     except Exception as e:
#         print("API Error:", e)
#
