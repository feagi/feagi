# ######  Statistics and Reporting Endpoints #########
# ####################################################

@app.get("/v1/feagi/monitoring/neuron/membrane_potential", tags=["Insights"])
async def cortical_neuron_membrane_potential_monitoring(cortical_area, response: Response):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)
    try:
        if cortical_area in runtime_data.neuron_mp_collection_scope:
            response.status_code = status.HTTP_200_OK
            return True
        else:
            response.status_code = status.HTTP_200_OK
            return False

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.post("/v1/feagi/monitoring/neuron/membrane_potential", tags=["Insights"])
async def cortical_neuron_membrane_potential_monitoring(cortical_area, state: bool, response: Response):
    print("Cortical membrane potential monitoring", runtime_data.neuron_mp_collection_scope)
    try:
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
                response.status_code = status.HTTP_200_OK
                return True
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            print("Error: InfluxDb is not setup to collect timeseries data!")
            return "Error: Timeseries database is not setup!"
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/monitoring/neuron/synaptic_potential", tags=["Insights"])
async def cortical_synaptic_potential_monitoring(cortical_area, response: Response):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_psp_collection_scope)
    try:
        if cortical_area in runtime_data.neuron_psp_collection_scope:
            response.status_code = status.HTTP_200_OK
            return True
        else:
            response.status_code = status.HTTP_200_OK
            return False
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.post("/v1/feagi/monitoring/neuron/synaptic_potential", tags=["Insights"])
async def cortical_synaptic_potential_monitoring(cortical_area, state: bool, response: Response):
    print("Cortical synaptic potential monitoring flag", runtime_data.neuron_psp_collection_scope)
    try:
        if runtime_data.influxdb:
            if runtime_data.influxdb.test_influxdb():
                if cortical_area in runtime_data.genome['blueprint']:
                    if state and cortical_area not in runtime_data.neuron_psp_collection_scope:
                        runtime_data.neuron_psp_collection_scope[cortical_area] = {}
                    elif not state and cortical_area in runtime_data.neuron_psp_collection_scope:
                        runtime_data.neuron_psp_collection_scope.pop(cortical_area)
                    else:
                        pass
                response.status_code = status.HTTP_200_OK
                return True
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
            return "Error: Timeseries database is not setup!"
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/neuron/physiology/membrane_potential_monitoring/filter_setting", tags=["Insights"])
async def neuron_membrane_potential_collection_filters(response: Response):
    print("Membrane potential monitoring filter setting:", runtime_data.neuron_mp_collection_scope)
    try:
        if runtime_data.neuron_mp_collection_scope:
            response.status_code = status.HTTP_200_OK
            return runtime_data.neuron_mp_collection_scope
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.get("/v1/feagi/neuron/physiology/postsynaptic_potential_monitoring/filter_setting", tags=["Insights"])
async def neuron_postsynaptic_potential_collection_filters(response: Response):
    print("Membrane potential monitoring filter setting:", runtime_data.neuron_psp_collection_scope)
    try:
        if runtime_data.neuron_psp_collection_scope:
            response.status_code = status.HTTP_200_OK
            return runtime_data.neuron_psp_collection_scope
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/neuron/physiology/membrane_potential_monitoring/filter_setting",
               methods=['POST'], tags=["Insights"])
async def neuron_membrane_potential_monitoring_scope(message: dict, response: Response):
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

    try:
        message = {'neuron_mp_collection_scope': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/neuron/physiology/postsynaptic_potential_monitoring", methods=['POST'], tags=["Insights"])
async def neuron_postsynaptic_potential_monitoring_scope(message: dict, response: Response):
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

    try:
        message = {'neuron_psp_collection_scope': message}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
