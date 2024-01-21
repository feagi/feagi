


# ######  Peripheral Nervous System Endpoints #########
# #####################################################

@app.api_route("/v1/feagi/feagi/pns/current/ipu", methods=['GET'], tags=["Peripheral Nervous System"])
async def current_ipu_list():
    try:
        if runtime_data.ipu_list:
            response.status_code = status.HTTP_200_OK
            return runtime_data.ipu_list
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/feagi/pns/current/opu", methods=['GET'], tags=["Peripheral Nervous System"])
async def current_opu_list(response: Response):
    try:
        if runtime_data.opu_list:
            response.status_code = status.HTTP_200_OK
            return runtime_data.opu_list
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc)


@app.api_route("/v1/agent/list", methods=['GET'], tags=["Peripheral Nervous System"])
async def agent_list(response: Response):
    try:
        agents = set(runtime_data.agent_registry.keys())
        if agents:
            response.status_code = status.HTTP_200_OK
            return agents
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/agent/properties", methods=['GET'], tags=["Peripheral Nervous System"])
async def agent_properties(agent_id: str, response: Response):
    try:
        print("agent_id", agent_id)
        print("agent_registry", runtime_data.agent_registry)
        agent_info = {}
        if agent_id in runtime_data.agent_registry:
            agent_info["agent_type"] = runtime_data.agent_registry[agent_id]["agent_type"]
            agent_info["agent_ip"] = runtime_data.agent_registry[agent_id]["agent_ip"]
            agent_info["agent_data_port"] = runtime_data.agent_registry[agent_id]["agent_data_port"]
            agent_info["agent_router_address"] = runtime_data.agent_registry[agent_id]["agent_router_address"]
            agent_info["agent_version"] = runtime_data.agent_registry[agent_id]["agent_version"]
            agent_info["controller_version"] = runtime_data.agent_registry[agent_id]["controller_version"]
            response.status_code = status.HTTP_200_OK
            return agent_info
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e, traceback.print_exc)


def assign_available_port():
    ports_used = []
    PORT_RANGES = (40001, 40050)
    for agent_id, agent_info in runtime_data.agent_registry.items():
        print(agent_id, agent_info, agent_info['agent_type'], type(agent_info['agent_type']))
        if agent_info['agent_type'] != 'monitor':
            ports_used.append(agent_info['agent_data_port'])
    print("ports_used", ports_used)
    for port in range(PORT_RANGES[0], PORT_RANGES[1]):
        if port not in ports_used:
            return port
    return None


@app.api_route("/v1/agent/register", methods=['POST'], tags=["Peripheral Nervous System"])
async def agent_registration(request: Request, agent_type: str, agent_id: str, agent_ip: str, agent_data_port: int,
                             agent_version: str, controller_version: str, response: Response):

    try:
        if agent_id in runtime_data.agent_registry:
            agent_info = runtime_data.agent_registry[agent_id]
        else:
            agent_info = {}
            agent_info["agent_id"] = agent_id
            agent_info["agent_type"] = agent_type
            # runtime_data.agent_registry[agent_id]["agent_ip"] = agent_ip
            agent_info["agent_ip"] = request.client.host
            if agent_type == 'monitor':
                agent_router_address = f"tcp://{request.client.host}:{agent_data_port}"
                agent_info["listener"] = Sub(address=agent_router_address, bind=False)
                print("Publication of brain activity turned on!")
                runtime_data.brain_activity_pub = True
            else:
                agent_data_port = assign_available_port()
                agent_router_address = f"tcp://*:{agent_data_port}"
                agent_info["listener"] = Sub(address=agent_router_address, bind=True)

            agent_info["agent_data_port"] = agent_data_port
            agent_info["agent_router_address"] = agent_router_address
            agent_info["agent_version"] = agent_version
            agent_info["controller_version"] = controller_version

        print(f"AGENT Details -- {agent_info}")
        runtime_data.agent_registry[agent_id] = agent_info

        print("New agent has been successfully registered:", runtime_data.agent_registry[agent_id])
        agent_info = runtime_data.agent_registry[agent_id].copy()
        agent_info.pop('listener')
        response.status_code = status.HTTP_200_OK
        return agent_info
    except Exception as e:
        print("API Error:", e, traceback.print_exc())
        print("Error during agent registration.:", agent_id)
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        return False


@app.api_route("/v1/agent/deregister", methods=['DELETE'], tags=["Peripheral Nervous System"])
async def agent_deregisteration(agent_id: str, response: Response):
    try:
        if agent_id in runtime_data.agent_registry:
            agent_info = runtime_data.agent_registry.pop(agent_id)
            agent_info['listener'].terminate()
            response.status_code = status.HTTP_200_OK
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)
