"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""
from bluezero import microbit
from router import *
from configuration import *
from time import sleep

import os
import configuration
import zmq
import router
import requests


runtime_data = {
    "cortical_data": {},
    "current_burst_id": None,
    "stimulation_period": None,
    "feagi_state": None,
    "feagi_network": None,
    "cortical_list": set(),
    "host_network": {}
}


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


computer_mac_address = os.environ.get("DOCKER_COMPUTER_MAC_ADDRESS")
microbit_mac_address = os.environ['DOCKER_MICROBIT_MAC_ADDRESS']
computer_mac_address = computer_mac_address.replace('"', "")
microbit_mac_address = microbit_mac_address.replace('"', "")
print(computer_mac_address)
ubit = microbit.Microbit(adapter_addr=computer_mac_address,
                         device_addr=microbit_mac_address,
                         accelerometer_service=False,
                         button_service=False,
                         led_service=False,
                         magnetometer_service=False,
                         pin_service=False,
                         temperature_service=False,
                         uart_service=True)

print("Searching for microbit...")
ubit.connect()
print("Microbit has been connected.")
print("Connecting to FEAGI resources...")

# address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_opu_port']

feagi_host = configuration.network_settings["feagi_host"]
api_port = configuration.network_settings["feagi_api_port"]

feagi_registration(feagi_host=feagi_host, api_port=api_port)

print("** **", runtime_data["feagi_state"])
network_settings['feagi_burst_speed'] = runtime_data["feagi_state"]['burst_duration']

# todo: to obtain this info directly from FEAGI as part of registration
ipu_channel_address = 'tcp://0.0.0.0:' + runtime_data["feagi_state"]['feagi_inbound_port_gazebo']
print("IPU_channel_address=", ipu_channel_address)
opu_channel_address = 'tcp://' + network_settings['feagi_host'] + ':' + \
                      runtime_data["feagi_state"]['feagi_opu_port']

feagi_ipu_channel = router.Pub(address=ipu_channel_address)
feagi_opu_channel = router.Sub(address=opu_channel_address, flags=router.zmq.NOBLOCK)
message_from_feagi = feagi_opu_channel.receive()


flag=True

while flag:
    try:
        message_from_feagi = feagi_opu_channel.receive()
        if message_from_feagi is not None:
            opu_data = message_from_feagi["opu_data"]
            print(message_from_feagi)
            if "o__mot" in opu_data:
                print(opu_data["o__mot"])
                for i in opu_data['o__mot']:
                    print("Sending now:" , i)
                    if i == "0-0-0":
                        ubit.uart = 'f#'
                    if i == "1-0-0":
                        ubit.uart = 'b#'
                    if i == "2-0-0":
                        ubit.uart = 'r#'
                    if i == "3-0-0":
                        ubit.uart = 'l#'
                    if i == "4-0-0":
                        ubit.uart = 'e#'
            if "o__led" in opu_data:
                for i in opu_data['o__led']:
                    print("Sending now:" , i)
                    if i == "0-0-0":
                        ubit.uart = '0#'
                    if i == "1-0-0":
                        ubit.uart = '1#'
                    if i == "2-0-0":
                        ubit.uart = '2#'
                    if i == "3-0-0":
                        ubit.uart = '3#'
                    if i == "4-0-0":
                        ubit.uart = '7#'
                    if i == "5-0-0":
                        ubit.uart = '8#'
                    if i == "6-0-0":
                        ubit.uart = '9#'
                    if i == "7-0-0":
                        ubit.uart = '12#'
            if len(str(opu_data)) < 13: #uart limits to 13 characters, so this will not send data to uart if its exceed
                #to avoid the distrupt in the program
                ubit.uart = opu_data
    except KeyboardInterrupt:
        flag = False
        ubit.disconnect()
        print('Program exited')
