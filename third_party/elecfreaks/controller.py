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
import zmq


ubit = microbit.Microbit(adapter_addr=network_settings['primary_mac_address'],
                         device_addr=network_settings['microbit_mac_address'],
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
address = 'tcp://' + network_settings['feagi_ip'] + ':' + network_settings['feagi_outbound_port']
feagi_state = handshake_with_feagi(address=address, capabilities=capabilities)
print("** **", feagi_state)
sockets = feagi_state['sockets']
network_settings['feagi_burst_speed'] = float(feagi_state['burst_frequency'])
print("--->> >> >> ", sockets)
ipu_channel_address = 'tcp://0.0.0.0:' + network_settings['feagi_inbound_port_gazebo']
print("IPU_channel_address=", ipu_channel_address)
opu_channel_address = 'tcp://' + network_settings['feagi_ip'] + ':' + sockets['feagi_outbound_port']

feagi_ipu_channel = Pub(address=ipu_channel_address)
feagi_opu_channel = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)

flag=True

while flag:
    try:
        message_from_feagi = feagi_opu_channel.receive()
        if message_from_feagi is not None:
            opu_data = message_from_feagi["opu_data"]
            #print(message_from_feagi)
            if "o__mic" in opu_data:
                print(opu_data["o__mic"])
                for i in opu_data['o__mic']:
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


        # text = input("Input: ")
        # ubit.uart = '#'
        # sleep(1)
    except KeyboardInterrupt:
        flag = False
        ubit.disconnect()
        print('Program exited')
