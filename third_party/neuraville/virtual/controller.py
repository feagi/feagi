
"""
This module contains a virtual set of hardware controllers used for simulation and testing only

todo: Need to have all of the controller.py modules follow the same convention and be consistent especially input data
"""
import router
import configuration
import requests
from time import time, sleep
from fake_stimulation import raw_stimulation, stimulation_pattern
from random import randrange, getrandbits


runtime_params = {
    "current_burst_id": 0,
    "global_timer": 0.5,
    "feagi_state": None,
    "feagi_network": None,
    "cortical_list": set(),
    "host_network": None
}


class FakeStimulator:
    def __init__(self):
        print("Fake stimulation initialized")

    @staticmethod
    def stimulate(burst_id):
        stimuli = dict()
        if burst_id in raw_stimulation:
            print("Cortical_list: ", runtime_params["cortical_list"])
            for cortical_area in raw_stimulation[burst_id]:
                if cortical_area in runtime_params["cortical_list"]:
                    stimuli[cortical_area] = raw_stimulation[burst_id][cortical_area]
        for cortical_area in stimulation_pattern:
            for pattern in stimulation_pattern[cortical_area]:
                pattern_burst_range_low = pattern[1][0]
                pattern_burst_range_high = pattern[1][1]
                if burst_id in range(pattern_burst_range_low, pattern_burst_range_high):
                    if cortical_area not in stimuli:
                        stimuli[cortical_area] = pattern[0]
                    else:
                        for item in pattern[0]:
                            stimuli[cortical_area].append(item)
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
    # ir_count = 3

    message = dict()
    message["controller_burst_id"] = runtime_params["current_burst_id"]
    message['data'] = dict()
    message['data']["direct_stimulation"] = dict()
    message['data']["sensory_data"] = dict()
    message['data']["direct_stimulation"] = stimulator.stimulate(runtime_params["current_burst_id"])
    if runtime_params["current_burst_id"] % 10 == 0:
        message["capabilities"] = configuration.capabilities
        message["network"] = configuration.network_settings

    # ipu_data['ultrasonic'] = {
    #     1: [randrange(0, 30) / 10, randrange(0, 30) / 10, randrange(0, 30) / 10, randrange(0, 30) / 10,
    #         randrange(0, 30) / 10, randrange(0, 30) / 10]
    # }
    # ipu_data['ir'] = {}

    # for _ in range(ir_count):
    #     ipu_data['ir'][_] = ir.read()

    return message


def cortical_mapping_list_gen(capabilities):
    feagi_address = 'http://127.0.0.1:8000'
    end_point = '/v1/feagi/connectome/properties/dimensions'

    cortical_data = requests.get(feagi_address + end_point).json()

    cortical_list = set()
    for cortical_area in cortical_data:
        cortical_list.add(cortical_area)
    return cortical_list


def main():
    runtime_params["cortical_list"] = cortical_mapping_list_gen(configuration.capabilities)
    print("## ### ####: cortical list:", runtime_params["cortical_list"])
    host_info = router.host_info()
    runtime_params["host_network"]["host_name"] = host_info["host_name"]
    runtime_params["host_network"]["ip_address"] = host_info["ip_address"]

    feagi_ip = configuration.network_settings["feagi_ip"]

    feagi_api_address = 'tcp://' + feagi_ip + ':' + runtime_params["feagi_network"]['feagi_outbound_port']
    
    runtime_params["feagi_state"] = router.handshake_with_feagi(api_address=feagi_api_address, capabilities=configuration.capabilities)

    print("** **", runtime_params["feagi_state"])

    # todo: to obtain this info directly from FEAGI as part of registration
    ipu_channel_address = 'tcp://0.0.0.0:' + configuration.network_settings['feagi_inbound_port_virtual']
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = 'tcp://' + configuration.network_settings['feagi_ip'] + ':' + \
                          runtime_params["feagi_state"]['feagi_outbound_port']

    feagi_ipu_channel = router.Pub(address=ipu_channel_address)
    feagi_opu_channel = router.Sub(address=opu_channel_address, flags=router.zmq.NOBLOCK)

    print("Connecting to FEAGI resources...")

    # todo: identify a method to instantiate all classes without doing it one by one
    # Instantiate Controller Classes
    # motor = Motor()

    # Listen and route
    print("Starting the routing engine")
    print("Communication frequency is set once every %f seconds" % runtime_params['global_timer'])

    # todo: need to have a method to sync burst id with the FEAGI
    runtime_params["current_burst_id"] = runtime_params["feagi_state"]["burst_counter"]

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
        sleep(runtime_params['global_timer'])
        if opu_data:
            runtime_params["current_burst_id"] = opu_data['burst_counter']


if __name__ == '__main__':
    main()
