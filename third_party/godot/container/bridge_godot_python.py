import time

from router import *
from configuration import *

import sys
import socket
import zmq
import csv
import ast

runtime_data = {
    "cortical_data": {}
}


def csv_writer(cortical_dimensions):
    f = open('csv_data.csv', 'w', newline='')
    writer = csv.writer(f)
    godot_cortical_dimensions = list()
    for cortical_area in cortical_dimensions:
        if cortical_dimensions[cortical_area][3]:
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][0])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][1])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][2])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][4])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][5])
            godot_cortical_dimensions.append(cortical_dimensions[cortical_area][6])
            godot_cortical_dimensions.append(cortical_area)
            writer.writerow(godot_cortical_dimensions)
            godot_cortical_dimensions = list()


def breakdown(feagi_input):  ##add input soon
    """
    #TODO: Explain what this is for
    """
    data = feagi_input
    data = list(data)
    increment = 0
    list1 = []

    while increment < len(data):
        voxel = [data[increment][1], data[increment][2], data[increment][3]]
        list1.append(voxel)
        UDP(str(voxel))
        increment += 1
    print(list1)
    UDP(str(list1))


def UDP(input):
    """
    This allows you to send any data to UDP. This port is what Godot's UDP using to recieve.
    """
    ip = "127.0.0.1"
    port = 20001

    # Create socket for server
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)

    # while True: ##this must be in loop in order to work with FEAGI
    s.sendto(input.encode('utf-8'), (ip, port))
    print("FEAGI: This Sent to Godot with the input: ", input, "\n\n")
    # close the socket
    s.close()


def godot_listener():
    """
    This is to recieve data from the Godot's data through UDP. You should expect to get a name,
    """
    godot_host = "127.0.0.1"
    godot_port = 20002
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((godot_host, godot_port))
    #sock.setblocking(1)

    data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes

    return data.decode("utf-8")


def godot_data(input):
    """
    Simply clean the list and remove all unnecessary special characters and deliver with name, xyz only
    """
    data = ast.literal_eval(input)
    dict_with_updated_name = {}
    dict_with_updated_name["data"] = dict()
    dict_with_updated_name["data"]["direct_stimulation"] = dict(dict())
    for key in data["data"]["direct_stimulation"]:
        Updated_name = name_to_id(key)
        if dict_with_updated_name["data"]["direct_stimulation"].get(Updated_name) is not None:
            pass
        else:

            dict_with_updated_name["data"]["direct_stimulation"][Updated_name] = []
        for key_01 in data["data"]["direct_stimulation"][key]:
            dict_with_updated_name["data"]["direct_stimulation"][Updated_name].append(key_01)

    print("godot_data: " , dict_with_updated_name)
    return dict_with_updated_name


def godot_selected_list(outside_list, godot_list):

    name = outside_list[0]
    x = int(outside_list[1])
    y = int(outside_list[2])
    z = int(outside_list[3])
    if godot_list:
        list_to_dict = godot_list
    else:
        list_to_dict = dict()
        list_to_dict["data"] = dict()
        list_to_dict["data"]["direct_stimulation"] = dict()

    if list_to_dict["data"]["direct_stimulation"].get(name) is not None:
        pass
    else:
        list_to_dict["data"]["direct_stimulation"][name] = list()
    for key in list_to_dict["data"]["direct_stimulation"]:
        if key not in list_to_dict["data"]["direct_stimulation"]:
            list_to_dict["data"]["direct_stimulation"][name] = list()
            list_to_dict["data"]["direct_stimulation"][name].append([x,y,z])
        else:
            list_to_dict["data"]["direct_stimulation"][name].append([x,y,z])

    return list_to_dict

def godot_deselected_list(outside_list, godot_list):
    for key in godot_list["data"]["direct_stimulation"]:
        if outside_list[0] == key:
            for xyz in godot_list["data"]["direct_stimulation"][key]:
                if xyz[0] == int(outside_list[1]) and xyz[1] == int(outside_list[2]) and xyz[2] == int(outside_list[3]):
                    godot_list["data"]["direct_stimulation"][key].remove(xyz)
    return godot_list

def name_to_id(name):
    for cortical_area in runtime_data["cortical_data"]:
        if cortical_area == name:
            return runtime_data["cortical_data"][cortical_area][7]
    else:
        pass
        #print("*** Failed to find cortical name ***" )


def feagi_breakdown(data):
    """
    Designed for genome 2.0 only. Data is the input from feagi's raw data
    """
    new_list = []
    for i in data['godot']:
        xyz = i[1], i[2], i[3]
        new_list.append(xyz)
    return new_list


def convert_absolute_to_relative_coordinate(stimulation_from_godot, cortical_data):
    """
    Convert absolute coordinate from godot to relative coordinate for FEAGI. Dna_information is from the
    genome["blueprint"].
    """

    relative_coordinate = {}
    relative_coordinate["data"] = dict()
    relative_coordinate["data"]["direct_stimulation"] = dict()
    if stimulation_from_godot:
        for key in stimulation_from_godot["data"]["direct_stimulation"]:
            for name_match in cortical_data:
                raw_id = name_match
                name_match = name_to_id(name_match) ##convert the human readable name into feagi name
                if name_match == key:
                    if relative_coordinate["data"]["direct_stimulation"].get(name_match) is not None:
                        pass
                    else:
                        relative_coordinate["data"]["direct_stimulation"][name_match] = list()
                    for xyz in stimulation_from_godot["data"]["direct_stimulation"][name_match]:
                        new_xyz =[xyz[0] - cortical_data[raw_id][0], xyz[1] - cortical_data[raw_id][1], xyz[2] - cortical_data[raw_id][2]]
                        relative_coordinate["data"]["direct_stimulation"][name_match].append(new_xyz)

            else:
                pass
    else:
        pass

    return relative_coordinate


Godot_list = {} ##initalized the list from Godot
address = 'tcp://' + network_settings['feagi_ip'] + ':' + network_settings['feagi_outbound_port']
feagi_state = handshake_with_feagi(address=address, capabilities=capabilities) ##I was trying to leverage on router only
print("feagi_state: " , feagi_state)
print("** **", feagi_state)
sockets = feagi_state['sockets']
network_settings['feagi_burst_speed'] = float(feagi_state['burst_frequency'])

print("--->> >> >> ", sockets)
FEAGI_pub = Pub(address='tcp://0.0.0.0:' + network_settings['feagi_inbound_port_godot'])
opu_channel_address = 'tcp://' + network_settings['feagi_ip'] + ':' + sockets['feagi_outbound_port']
FEAGI_sub = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)
UDP(str("0,0,0"))


# Send a request to FEAGI for cortical dimensions
awaiting_feagi_registration = True
while awaiting_feagi_registration:
    print("Awaiting registration with FEAGI...")
    FEAGI_pub.send({"godot_init": True})
    message_from_feagi = FEAGI_sub.receive()
    if message_from_feagi:
        if "cortical_dimensions" in message_from_feagi:
            if len(message_from_feagi["cortical_dimensions"]) > 0:
                print(">>> >> > >", message_from_feagi["cortical_dimensions"])
                runtime_data["cortical_data"] = message_from_feagi["cortical_dimensions"]
                csv_writer(message_from_feagi["cortical_dimensions"])
                awaiting_feagi_registration = False
    time.sleep(1)
time.sleep(2)

while True:
    one_frame = FEAGI_sub.receive()
    if one_frame is not None:
        one_frame = feagi_breakdown(one_frame)
        UDP(str(one_frame))
    data_from_godot = godot_listener()
    if (data_from_godot != "None" and data_from_godot != "{}" and data_from_godot != Godot_list and data_from_godot != "refresh"):
        print(data_from_godot)
        Godot_list = godot_data(data_from_godot)
        converted_data = convert_absolute_to_relative_coordinate(stimulation_from_godot=Godot_list,
                                                                 cortical_data=runtime_data["cortical_data"])
        print(">>> > > > >> > converted data:", converted_data)
        FEAGI_pub.send(converted_data)
    if data_from_godot == "refresh":
        Godot_list = {}
        converted_data = {}
        FEAGI_pub.send(Godot_list)
    else:
        pass

