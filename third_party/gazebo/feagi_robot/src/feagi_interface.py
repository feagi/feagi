import router
import configuration
from time import sleep


def feagi_registration(feagi_host, api_port, host_info, ):
    runtime_data = {
        "host_network": {},
        "feagi_state": None
    }
    app_host_info = host_info
    runtime_data["host_network"]["host_name"] = app_host_info["host_name"]
    runtime_data["host_network"]["ip_address"] = app_host_info["ip_address"]

    while runtime_data["feagi_state"] is None:
        print("Awaiting registration with FEAGI...")
        try:
            runtime_data["feagi_state"] = router.register_with_feagi(app_name=configuration.app_name,
                                                                     feagi_host=feagi_host,
                                                                     api_port=api_port,
                                                                     app_capabilities=configuration.capabilities,
                                                                     app_host_info=runtime_data["host_network"]
                                                                     )
        except:
            pass
        sleep(1)


def block_to_array(block_ref):
    block_id_str = block_ref.split('-')
    array = [int(x) for x in block_id_str]
    return array


def feagi_setting_for_registration():
    """
    Generate all needed information and return the full data to make it easier to connect with
    FEAGI
    """
    feagi_ip_host = configuration.network_settings["feagi_host"]
    api_data = configuration.network_settings["feagi_api_port"]
    return feagi_ip_host, api_data


def feagi_gui_address(feagi_ip_host, api_data):
    """
    return a full path to api
    """
    return 'http://' + feagi_ip_host + ':' + api_data


def feagi_api_burst_engine():
    return '/v1/feagi/feagi/burst_engine/stimulation_period'


def feagi_api_burst_counter():
    return '/v1/feagi/feagi/burst_engine/burst_counter'


