import os
from fastapi import APIRouter, HTTPException

from ...commons import *
from ...schemas import *

from src.version import __version__


router = APIRouter()


# ######   System Endpoints #########
# ###################################

def human_readable_version(version):
    print(version)
    time_portion = str(version)[-10:]
    reminder = str(version)[:-10]
    human_readable_time = datetime.utcfromtimestamp(int(time_portion))
    if reminder:
        if int(reminder) == 0:
            reminder = "C"
        else:
            reminder = "N"
    else:
        reminder = "N"
    return reminder + '-' + human_readable_time.strftime("%Y-%m-%d %H:%M:%S UTC")


@router.get("/versions")
def get_versions():
    try:
        all_versions = dict()
        all_versions["feagi"] = str(__version__)
        for agent_id in runtime_data.agent_registry:
            if agent_id not in all_versions:
                all_versions[agent_id] = {}
            all_versions[agent_id]["agent_version"] = \
                str(runtime_data.agent_registry[agent_id]["agent_version"])
            all_versions[agent_id]["controller_version"] = \
                str(runtime_data.agent_registry[agent_id]["controller_version"])
        return all_versions
    except Exception as e:
        print(f"Error during version collection {e}")


@router.get("/health_check")
async def feagi_health_check():
    health = dict()
    health["burst_engine"] = not runtime_data.exit_condition

    if runtime_data.genome:
        health["genome_availability"] = True
    else:
        health["genome_availability"] = False

    health["genome_validity"] = runtime_data.genome_validity
    health["brain_readiness"] = runtime_data.brain_readiness

    if pending_amalgamation():
        health["amalgamation_pending"] = {
            "initiation_time": runtime_data.pending_amalgamation["initiation_time"],
            "genome_id": runtime_data.pending_amalgamation["genome_id"],
            "amalgamation_id": runtime_data.pending_amalgamation["amalgamation_id"],
            "genome_title": runtime_data.pending_amalgamation["genome_title"],
            "circuit_size": runtime_data.pending_amalgamation["circuit_size"]
        }

    return health


@router.get("/unique_logs")
async def unique_log_entries():
    return runtime_data.logs


@router.post("/register")
async def feagi_registration(message: Registration):
    message = message.dict()
    source = message['source']

    host = message['host']
    capabilities = message['capabilities']
    print("########## ###### >>>>>> >>>> ", source, host, capabilities)


@router.post("/logs")
async def log_management(message: Logs):
    message = message.dict()
    message = {"log_management": message}
    api_queue.put(item=message)


@router.get("/configuration")
async def configuration_parameters():
    return runtime_data.parameters


@router.get("/beacon/subscribers")
async def beacon_query():
    if runtime_data.beacon_sub:
        return tuple(runtime_data.beacon_sub)
    else:
        raise HTTPException(status_code=400, detail=f"No subscriber found")


@router.post("/beacon/subscribe")
async def beacon_subscribe(message: Subscriber):
    message = {'beacon_sub': message.subscriber_address}
    api_queue.put(item=message)


@router.delete("/beacon/unsubscribe")
async def beacon_unsubscribe(message: Subscriber):
    message = {"beacon_unsub": message.subscriber_address}
    api_queue.put(item=message)


@router.get("/db/influxdb/test")
async def test_influxdb():
    """
    Enables changes against various Burst Engine parameters.
    """

    influx_status = runtime_data.influxdb.test_influxdb()
    if influx_status:
        return influx_status


@router.post("/circuit_library_path")
async def change_circuit_library_path(circuit_library_path: str):
    if os.path.exists(circuit_library_path):
        runtime_data.circuit_lib_path = circuit_library_path
        print(f"{circuit_library_path} is the new circuit library path.")
    else:
        raise HTTPException(status_code=400, detail=f"{circuit_library_path} is not a valid path.")
