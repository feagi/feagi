from fastapi import APIRouter, HTTPException

from ...commons import *
from ....inf import runtime_data


router = APIRouter()


# ######  Statistics and Reporting Endpoints #########
# ####################################################

@router.get("/v1/feagi/monitoring/neuron/membrane_potential")
async def cortical_neuron_membrane_potential_monitoring(cortical_area):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)

    if cortical_area in runtime_data.neuron_mp_collection_scope:
        return True
    else:
        return False


@router.post("/v1/feagi/monitoring/neuron/membrane_potential")
async def cortical_neuron_membrane_potential_monitoring(cortical_area, state: bool):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)

    if runtime_data.influxdb:
        influx_readiness = runtime_data.influxdb.test_influxdb()
        if influx_readiness:
            if cortical_area in runtime_data.genome['blueprint']:
                if state and cortical_area not in runtime_data.neuron_mp_collection_scope:
                    runtime_data.neuron_mp_collection_scope[cortical_area] = {}
                elif not state and cortical_area in runtime_data.neuron_mp_collection_scope:
                    runtime_data.neuron_mp_collection_scope.pop(cortical_area)
                else:
                    pass
            return True
    else:
        raise HTTPException(status_code=400, detail="InfluxDb service is not running!")


@router.get("/v1/feagi/monitoring/neuron/synaptic_potential", tags=["Insights"])
async def cortical_synaptic_potential_monitoring(cortical_area):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_psp_collection_scope)

    if cortical_area in runtime_data.neuron_psp_collection_scope:
        return True
    else:
        return False


@router.post("/v1/feagi/monitoring/neuron/synaptic_potential")
async def cortical_synaptic_potential_monitoring(cortical_area, state: bool):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_psp_collection_scope)
    if runtime_data.influxdb:
        if runtime_data.influxdb.test_influxdb():
            if cortical_area in runtime_data.genome['blueprint']:
                if state and cortical_area not in runtime_data.neuron_psp_collection_scope:
                    runtime_data.neuron_psp_collection_scope[cortical_area] = {}
                elif not state and cortical_area in runtime_data.neuron_psp_collection_scope:
                    runtime_data.neuron_psp_collection_scope.pop(cortical_area)
                else:
                    pass
            return True
    else:
        raise HTTPException(status_code=400, detail="InfluxDb service is not running!")


@router.get("/v1/feagi/neuron/physiology/membrane_potential_monitoring/filter_setting")
async def neuron_membrane_potential_collection_filters():
    print("Membrane potential monitoring filter setting:", runtime_data.neuron_mp_collection_scope)

    if runtime_data.neuron_mp_collection_scope:
        return runtime_data.neuron_mp_collection_scope
    else:
        return {}


@router.get("/v1/feagi/neuron/physiology/postsynaptic_potential_monitoring/filter_setting")
async def neuron_postsynaptic_potential_collection_filters():
    print("Membrane potential monitoring filter setting:", runtime_data.neuron_psp_collection_scope)
    if runtime_data.neuron_psp_collection_scope:
        return runtime_data.neuron_psp_collection_scope
    else:
        return {}


@router.post("/v1/feagi/neuron/physiology/membrane_potential_monitoring/filter_setting")
async def neuron_membrane_potential_monitoring_scope(message: dict):
    """
    Monitor the membrane potential of select cortical areas and voxels in Grafana.
    Message Template:
            {
                "o__mot": {
                    "voxels": [[0, 0, 0], [2, 0, 0]],
                    "neurons": []
                },
                "i__inf": {
                    "voxels": [[1, 1, 1]],
                    "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
                },
                ...
            }
    """

    message = {'neuron_mp_collection_scope': message}
    api_queue.put(item=message)


@router.post("/v1/feagi/neuron/physiology/postsynaptic_potential_monitoring")
async def neuron_postsynaptic_potential_monitoring_scope(message: dict):
    """
    Monitor the post synaptic potentials of select cortical areas and voxels in Grafana.

    Message Template:
            {
                "o__mot": {
                    "dst_filter": {
                        "voxels": [[0, 0, 0], [2, 0, 0]],
                        "neurons": []
                        },
                    "sources": {
                        "i__inf": {
                            "voxels": [[1, 1, 1]],
                            "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
                            },
                        "o__inf": {
                            "voxels": [[1, 1, 1]],
                            "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
                            }
                    },
                },
                "i__bat": {
                    "dst_filter": {
                        "voxels": [[0, 0, 0], [2, 0, 0]],
                        "neurons": []
                    },
                    "sources": {
                        "i__inf": {
                            "voxels": [[1, 1, 1]],
                            "neurons": ['neuron_id_1', 'neuron_id_2', 'neuron_id_3']
                        }
                    }
                }
            }
    """

    message = {'neuron_psp_collection_scope': message}
    api_queue.put(item=message)
