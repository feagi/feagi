from static_genome import genome
import socket
import zmq
import csv
from router import *
from configuration import *
from configuration import message_to_feagi

host = "127.0.0.1"
port = "30003"

address = 'tcp://' + router_settings['feagi_ip'] + ':' + router_settings['feagi_port']
feagi_state = find_feagi(address=address) ##I was trying to leverage on router only
print("feagi_state: " , feagi_state)
print("** **", feagi_state)
sockets = feagi_state['sockets']
router_settings['feagi_burst_speed'] = float(feagi_state['burst_frequency'])

print("--->> >> >> ", sockets)

# def feagi_initalize():
#     # Getting FEAGI's raw data
#
#     context = zmq.Context()
#     socket = context.socket(zmq.SUB)
#     print('Listening FEAGI...')
#     socket.connect("tcp://{}:{}".format(host, port))
#     socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))
#     set_stored = socket.recv_pyobj()
#
#     return set_stored


def genome_2_cortical_list(flat_genome):
    """
    Generates a list of cortical areas inside static_genome.py. This will reads the data and add the list if it has
    "x-gd_vis-b" defined True.
    """
    cortical_list = {}
    for key in flat_genome:
        # print(key[17:])
        cortical_id = key[9:15]
        if key[19:] == "rcordx-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
        if key[19:] == "rcordy-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
        if key[19:] == "rcordz-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
        if cortical_id not in cortical_list and key[7] == "c":
            cortical_list[cortical_id] = [genome["blueprint"][key]]
        if key[17:] == "x-gd_vis-b":
            cortical_list[cortical_id].append(genome["blueprint"][key])
            # print(genome["blueprint"][key])
        if key[22:] == "bbx-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
        if key[22:] == "bby-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
        if key[22:] == "bbz-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
    return cortical_list


def CSV_writer(cortical_list):
    f = open('csv_data.csv', 'w', newline='')
    writer = csv.writer(f)
    godot_cortical_list = list()
    for key in cortical_list:
        godot_cortical_list.append(cortical_list[key][2])
        godot_cortical_list.append(cortical_list[key][3])
        godot_cortical_list.append(cortical_list[key][4])
        godot_cortical_list.append(cortical_list[key][5])
        godot_cortical_list.append(cortical_list[key][6])
        godot_cortical_list.append(cortical_list[key][7])
        godot_cortical_list.append(cortical_list[key][0])
        writer.writerow((godot_cortical_list))
        godot_cortical_list = list()


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
    print("\n\n 1. This Sent to Godot with the input: ", input, "\n\n")
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

    data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
    return data

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

def godot_selected_list(outside_list, godot_list):
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
        list_to_dict["stimulation"] = dict()
        #print("newbie")
    #print("middle", list_to_dict)

    if list_to_dict["stimulation"].get(name) is not None:
        pass
        #print("true")
    else:
        #print("added ", name)
        list_to_dict["stimulation"][name] = list()
    for key in list_to_dict["stimulation"]:
        #print(key)
        if key not in list_to_dict["stimulation"]:
            list_to_dict["stimulation"][name] = list()
            list_to_dict["stimulation"][name].append([x,y,z])
            #print("first time")
        else:
            list_to_dict["stimulation"][name].append([x,y,z])
           # print("second or more time")
    print("end: ", list_to_dict)

    return list_to_dict

def name_to_id(name):
    list = genome['blueprint']
    feagi_name_readable = name
    for key in list:
        if genome['blueprint'][key] == name:
            feagi_name_readable = key[9:15]
    return feagi_name_readable

def feagi_breakdown(data):
    """
    Designed for genome 2.0 only
    """
    new_list = []
    print("bwuk", type(data))
    print(data['godot'],data['godot'])

    new_data = data[37:]
    new_data = new_data.replace(" ", "")
    new_data = new_data.replace("{", "")
    new_data = new_data.replace("}","")
    new_data = new_data.replace(")", "")
    new_data = new_data.replace("(","")
    new_data = new_data.split(",")
    length_array = len(new_data) / 3
    for i in range(int(length_array)):
        new_list.append([int(new_data[i + 1]), int(new_data[i + 2]), int(new_data[i + 3])])
    return new_list

Godot_list = {}
#UDP("{'godot': {(59, 5, 0, 3), (59, 5, 0, 9), (59, 5, 0, 2), (59, 5, 0, 5), (59, 5, 0, 8), (59, 5, 0, 4)}}")

one_frame = genome_2_cortical_list(genome['blueprint'])
CSV_writer(one_frame)
FEAGI_pub = Pub(address='tcp://0.0.0.0:' + router_settings['ipu_port'])
opu_channel_address = 'tcp://' + router_settings['feagi_ip'] + ':' + sockets['opu_port']
FEAGI_sub = Sub(address=opu_channel_address, flags=zmq.NOBLOCK)
while True:
    one_frame = FEAGI_sub.receive()
    #one_frame = "{'godot': {(59, 5, 0, 3), (59, 5, 0, 9), (59, 5, 0, 2), (59, 5, 0, 5), (59, 5, 0, 8), (59, 5, 0, 4)}}"

    if one_frame is not None:
        #print(one_frame) #Don't delete this, it worked perfectly
        one_frame = feagi_breakdown(one_frame)
        UDP(str(one_frame))


    # one_frame = feagi_initalize() #disable to comment
    # print(one_frame)
    #UDP("[0,0,0")
    #breakdown(one_frame)
    data = godot_listener().decode("utf-8")
    if data == "ready":
        FEAGI_pub.send(Godot_list)
    elif data == "refresh":
        Godot_list = {}
    else:
        data = godot_data(data)
        name = data[0]
        data[0] = name_to_id(name)
        if Godot_list:
            Godot_list = godot_selected_list(data, Godot_list)
        else:
            Godot_list = godot_selected_list(data, Godot_list)


    # exists = 4 in data
    # print(exists)
    # if exists:
    #     print("deleted")
    # #    FEAGI_pub.send(data)
