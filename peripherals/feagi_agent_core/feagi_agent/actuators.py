#!/usr/bin/env python3
from feagi_agent import feagi_interface as feagi
import serial
import json


def obtain_opu_data(device_list, message_from_feagi):
    opu_signal_dict = {}
    opu_data = feagi.opu_processor(message_from_feagi)
    for i in device_list:
        if i in opu_data and opu_data[i]:
            for x in opu_data[i]:
                if i not in opu_signal_dict:
                    opu_signal_dict[i] = {}
                opu_signal_dict[i][x] = opu_data[i][x]
    return opu_signal_dict

def initialization_port (port):
    return serial.Serial(port,
                         baudrate=9600,
                         timeout=2.5,
                         parity=serial.PARITY_NONE,
                         bytesize=serial.EIGHTBITS,
                         stopbits=serial.STOPBITS_ONE)


def convert_dict_to_json(data):
    data = json.dumps(data)
    return data

def send_serial(ser, data):
    ser.write(data.encode('ascii'))