# ######  Training Endpoints #######
# ##################################


@app.api_route("/v1/feagi/training/reset_game_stats", methods=['DELETE'], tags=["Training"])
async def delete_game_stats_from_db(response: Response):
    """
    Erases the game statistics from the database.
    """
    try:
        runtime_data.influxdb.drop_game_activity()
        response.status_code = status.HTTP_200_OK
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/shock/options", methods=['Get'], tags=["Training"])
async def list_available_shock_scenarios(response: Response):
    """
    Get a list of available shock scenarios.
    """
    try:
        if runtime_data.shock_scenarios_options:
            response.status_code = status.HTTP_200_OK
            return runtime_data.shock_scenarios_options
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/shock/status", methods=['Get'], tags=["Training"])
async def list_activated_shock_scenarios(response: Response):
    try:
        if runtime_data.shock_scenarios:
            response.status_code = status.HTTP_200_OK
            return runtime_data.shock_scenarios
        else:
            response.status_code = status.HTTP_404_NOT_FOUND
    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/shock/activate", methods=['POST'], tags=["Training"])
async def activate_shock_scenarios(shock: Shock, response: Response):
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
    try:
        message = shock.dict()
        print(message)
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/reward", methods=['POST'], tags=["Training"])
async def reward_intensity(intensity: Intensity, response: Response):
    """
    Captures feedback from the environment during training
    """
    try:
        message = {'reward': intensity.intensity}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/punishment", methods=['POST'], tags=["Training"])
async def punishment_intensity(intensity: Intensity, response: Response):
    """
    Captures feedback from the environment during training
    """
    try:
        message = {'punishment': intensity.intensity}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/gameover", methods=['POST'], tags=["Training"])
async def gameover_signal(response: Response):
    """
    Captures feedback from the environment during training
    """
    try:
        message = {'gameover': True}
        api_queue.put(item=message)
        response.status_code = status.HTTP_200_OK

    except Exception as e:
        response.status_code = status.HTTP_422_UNPROCESSABLE_ENTITY
        print("API Error:", e)


@app.api_route("/v1/feagi/training/training_report", methods=['GET'], tags=["Training"])
async def training_report():
    """
    Returns stats associated with training
    """
    return runtime_data.training_stats
