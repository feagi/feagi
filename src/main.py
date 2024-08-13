
# FEAGI is a brain inspired evolutionary framework capable of growing an artificial brain from a
# genome and helping it evolve over generations.
#
# This main module is responsible for driving the lifecycle of a single generation of an
# artificial brain at a time. To scale up the system to many parallel generations, FEAGI
# is intended to run within a container and scale up to many container instances.


if __name__ == "__main__":
    import sys
    import os
    from configuration import init_parameters

    sys.path.append('../')
    import uvicorn
    import platform
    import json
    import logging.config

    from inf import runtime_data

    if platform.system() == 'Windows':
        with open("logging_config.json", "r") as config_file:
            logging_config_data = json.load(config_file)
        # grab temp from local only regardleess account name.
        temp_misc = os.getenv('TEMP')
        logging_config_data['handlers']['misc_logs']['filename'] = temp_misc + r'\feagi-misc.log'
        logging_config_data['handlers']['api_logs']['filename'] = temp_misc + r'\feagi-api.log'
        # setup loggers
        logging.config.dictConfig(logging_config_data)

        # get root logger
        logger = logging.getLogger(__name__)

    runtime_data.parameters = init_parameters()

    if runtime_data.parameters['Sockets']['feagi_api_port'] is None:
        runtime_data.parameters['Sockets']['feagi_api_port'] = 8000  # Default port

    print("Starting FEAGI API on port ", runtime_data.parameters['Sockets']['feagi_api_port'])

    uvicorn.run("api.api:app", host="0.0.0.0", port=int(runtime_data.parameters['Sockets']['feagi_api_port']),
                reload=False, log_level="debug", debug=True, workers=4)

