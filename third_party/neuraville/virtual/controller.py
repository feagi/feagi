
"""
This module contains a virtual set of hardware controllers used for simulation and testing only

todo: Need to have all of the controller.py modules follow the same convention and be consistent especially input data
"""
import router
import configuration
import requests
from time import sleep
from fake_stimulation import raw_stimulation, stimulation_pattern, stimulations

runtime_data = {
    "current_burst_id": 0,
    "global_timer": 0.5,
    "feagi_state": None,
    "feagi_network": None,
    "cortical_list": set(),
    "host_network": {},
    "stimulation_index": {}
}


class FakeStimulator:
    def __init__(self):
        print("Fake stimulation initialized")

    @staticmethod
    def stimulate(burst_id):
        print("=====================================================================================================")
        if burst_id:
            print("+++ burst id:", burst_id, runtime_data["stimulation_index"])
            stimuli = dict()
            for play in stimulations:
                play_length = len(stimulations[play]["definition"])
                print("play --$------------$-------------$-- :", play, play_length)
                try:
                    if stimulations[play]["start_burst"] is not None and stimulations[play]["start_burst"] >= burst_id:
                        continue
                    if stimulations[play]["end_burst"] is not None and stimulations[play]["end_burst"] <= burst_id:
                        continue
                    if play_length == 0:
                        continue
                except KeyError:
                    pass
                if play not in runtime_data["stimulation_index"]:
                    runtime_data["stimulation_index"][play] = 0

                print("    >> >> >> stimulation_index:", runtime_data["stimulation_index"][play])

                for cortical_area in stimulations[play]["definition"][runtime_data["stimulation_index"][play]]:
                    print("           cortical area:", cortical_area)
                    if cortical_area in runtime_data["cortical_list"]:
                        stimuli[cortical_area] = \
                            stimulations[play]["definition"][runtime_data["stimulation_index"][play]][cortical_area]

                runtime_data["stimulation_index"][play] += 1
                if runtime_data["stimulation_index"][play] == play_length:
                    runtime_data["stimulation_index"][play] = 0

            return stimuli


def build_message_to_feagi():
    """
    This function encodes the sensory information in a dictionary that can be decoded on the FEAGI end.

    expected ipu_data structure:

        ipu_data = {
            "capabilities": {},
            "network": {},
            "data": {
                "direct_stimulation": {
                    "cortical_area_id": {voxel},
                    "cortical_area_id": sensor_data,
                    "cortical_area_id": sensor_data
                    ...
                    },
                "sensory_data": {
                    "sensor type": sensor data,
                    "sensor type": sensor data,
                    "sensor type": sensor data,
                    ...
                }
            }
    """

    stimulator = FakeStimulator()
    # Process IPU data received from controller.py and pass it along to FEAGI
    # todo: move class instantiations to outside function
    # ir = IR()

    # todo: figure a better way of obtaining the device count
    # ir_count = 1

    message = dict()
    message["controller_burst_id"] = runtime_data["current_burst_id"]
    message['data'] = dict()
    message['data']["direct_stimulation"] = dict()
    message['data']["sensory_data"] = dict()
    message['data']["direct_stimulation"] = stimulator.stimulate(runtime_data["current_burst_id"])
    # if runtime_params["current_burst_id"] % 10 == 0:
    #     message["capabilities"] = configuration.capabilities
    #     message["network"] = configuration.network_settings

    # ipu_data['ultrasonic'] = {
    #     1: [randrange(0, 30) / 10, randrange(0, 30) / 10, randrange(0, 30) / 10, randrange(0, 30) / 10,
    #         randrange(0, 30) / 10, randrange(0, 30) / 10]
    # }
    # ipu_data['ir'] = {}

    # for _ in range(ir_count):
    #     ipu_data['ir'][_] = ir.read()

    return message


def cortical_mapping_list_gen(capabilities, feagi_host, api_port):
    api_address = 'http://' + feagi_host + ':' + api_port
    end_point = '/v1/feagi/connectome/properties/dimensions'

    cortical_data = requests.get(api_address + end_point).json()

    cortical_list = set()
    for cortical_area in cortical_data:
        cortical_list.add(cortical_data[cortical_area][7])

    return cortical_list


def feagi_registration(feagi_host, api_port):
    app_host_info = router.app_host_info()
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


def main():

    feagi_host = configuration.network_settings["feagi_host"]
    api_port = configuration.network_settings["feagi_api_port"]

    feagi_registration(feagi_host=feagi_host, api_port=api_port)

    print("** **", runtime_data["feagi_state"])

    runtime_data["cortical_list"] = cortical_mapping_list_gen(capabilities=configuration.capabilities,
                                                              feagi_host=feagi_host,
                                                              api_port=api_port)
    runtime_data["stimulation_index"] = dict()
    print("## ### ####: cortical list:", runtime_data["cortical_list"])

    print("configuration.network_settings:", configuration.network_settings)

    # todo: to obtain this info directly from FEAGI as part of registration
    ipu_channel_address = 'tcp://0.0.0.0:' + runtime_data["feagi_state"]['feagi_inbound_port_virtual']
    print("IPU_channel_address=", ipu_channel_address, "\nfeagi_network:", runtime_data["feagi_network"])
    opu_channel_address = 'tcp://' + feagi_host + ':' + runtime_data["feagi_state"]['feagi_outbound_port']

    feagi_ipu_channel = router.Pub(address=ipu_channel_address)
    feagi_opu_channel = router.Sub(address=opu_channel_address, flags=router.zmq.NOBLOCK)

    print("Connecting to FEAGI resources...")

    # todo: identify a method to instantiate all classes without doing it one by one
    # Instantiate Controller Classes
    # motor = Motor()

    # Listen and route
    print("Starting the routing engine")
    print("Communication frequency is set once every %f seconds" % runtime_data['global_timer'])

    # todo: need to have a method to sync burst id with the FEAGI
    runtime_data["current_burst_id"] = runtime_data["feagi_state"]["burst_counter"]

    while True:
        # Process OPU data received from FEAGI and pass it along to the controller.py
        opu_data = feagi_opu_channel.receive()
        print("Received:", opu_data)
        # if opu_data is not None:
        #     if 'motor' in opu_data:
        #         for motor_id in opu_data['motor']:
        #             motor.move(motor_id, opu_data['motor'][motor_id])
        #

        message_to_feagi = build_message_to_feagi()
        feagi_ipu_channel.send(message_to_feagi)

        # todo: IMPORTANT!!! need to figure how to correlate the flow on incoming data with the rate data is passed to FEAGI
        sleep(runtime_data['global_timer'])
        if opu_data:
            runtime_data["current_burst_id"] = opu_data['burst_counter']


if __name__ == '__main__':
    main()
