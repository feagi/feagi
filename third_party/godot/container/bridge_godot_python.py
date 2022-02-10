import time

from router import *
from configuration import *

import sys
import socket
import zmq
import csv

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
    data = input
    data = data.split(",")
    data_holder = data[0]
    data_holder = ''.join([i for i in data_holder if not i.isdigit()])
    data[0] = data_holder.replace("@", "")
    data[0] = data_holder.replace(" @", "")
    data[1] = data[1].replace("(", "")
    data[3] = data[3].replace(")", "")
    return data


def godot_selected_list(outside_list, godot_list): ##This one will get raw forward from feagi so is this right

    name = outside_list[0]
    x = int(outside_list[1])
    y = int(outside_list[2])
    z = int(outside_list[3])
    if godot_list:
        list_to_dict = godot_list
        #print("experienced")
    else:
        list_to_dict = dict()
        #print(type(list_to_dict))

        list_to_dict["data"] = dict()
        list_to_dict["data"]["direct_stimulation"] = dict()
        #print("newbie")
    #print("middle", list_to_dict)

    if list_to_dict["data"]["direct_stimulation"].get(name) is not None:
        pass
        #print("true")
    else:
        #print("added ", name)
        list_to_dict["data"]["direct_stimulation"][name] = list()
    for key in list_to_dict["data"]["direct_stimulation"]:
        #print(key)
        if key not in list_to_dict["data"]["direct_stimulation"]:
            list_to_dict["data"]["direct_stimulation"][name] = list()
            list_to_dict["data"]["direct_stimulation"][name].append([x,y,z])
            #print("first time")
        else:
            list_to_dict["data"]["direct_stimulation"][name].append([x,y,z])
           # print("second or more time")
    #print("Selected: ", list_to_dict)

    return list_to_dict


def name_to_id(name):

    for cortical_area in runtime_data["cortical_data"]:
        if runtime_data["cortical_data"][cortical_area][7] == name:
            return cortical_area
    else:
        print("*** Failed to find cortical name ***")


def feagi_breakdown(data):
    """
    Designed for genome 2.0 only. Data is the input from feagi's raw data
    """
    new_list = []
    for i in data['godot']:
        xyz = i[1], i[2], i[3]
        new_list.append(xyz)
    return new_list


def convert_absolute_to_relative_coordinate(dict_input, dna_information):
    """
    Convert absolute coordinate from godot to relative coordinate for FEAGI. Dna_information is from the
    genome["blueprint"].
    """
    absolute_dict = dict_input
    print(absolute_dict)
    relative_coordinate = {}
    relative_coordinate["data"] = dict()
    relative_coordinate["data"]["direct_stimulation"] = dict()
    if absolute_dict:
        for key in absolute_dict["data"]["direct_stimulation"]:
            for name_match in dna_information:
                if name_match == key:
                    if relative_coordinate["data"]["direct_stimulation"].get(name_match) is not None:
                        pass
                    else:
                        relative_coordinate["data"]["direct_stimulation"][name_match] = list()
                    for xyz in absolute_dict["data"]["direct_stimulation"][name_match]:
                        new_xyz =[xyz[0] - dna_information[name_match][2], xyz[1] - dna_information[name_match][3], xyz[2] - dna_information[name_match][4]]
                        relative_coordinate["data"]["direct_stimulation"][name_match].append(new_xyz)
                    #absolute_dict["data"]["direct_stimulation"][name_match][0] - dna_information[name_match][5]

            else:
                pass
    else:
        pass
    return relative_coordinate


Godot_list = {}
print("Godot_list = ", Godot_list)
#UDP("{'godot': {(59, 5, 0, 3), (59, 5, 0, 9), (59, 5, 0, 2), (59, 5, 0, 5), (59, 5, 0, 8), (59, 5, 0, 4)}}")

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
    #print("one_frame after feagi: ", one_frame)
    #one_frame = "{'godot': {(59, 5, 0, 3), (59, 5, 0, 9), (59, 5, 0, 2), (59, 5, 0, 5), (59, 5, 0, 8), (59, 5, 0, 4)}}"

    if one_frame is not None:
        print("FEAGI's raw data: " , one_frame) #Don't delete this, it worked perfectly
        one_frame = feagi_breakdown(one_frame)
        #print("one frame after breakdown: ", one_frame)
        UDP(str(one_frame))


    # one_frame = feagi_initalize() #disable to comment
    # print(one_frame)
    #UDP("[0,0,0")
    #breakdown(one_frame)


    ##This stops all processing and force to wait for the return data from FEAGI
    #data = "None"
    data = godot_listener()
    if data != "None":
        if data == "ready":
            converted_data = convert_absolute_to_relative_coordinate(Godot_list, runtime_data["cortical_data"])
            FEAGI_pub.send(converted_data)
        elif data == "refresh":
            Godot_list = {}
            converted_data = {}
            FEAGI_pub.send(Godot_list)
        else:
            data = godot_data(data)
            name = data[0]
            data[0] = name_to_id(name)
            if Godot_list:
                Godot_list = godot_selected_list(data, Godot_list)
            else:
                Godot_list = godot_selected_list(data, Godot_list)
    else:
        pass

