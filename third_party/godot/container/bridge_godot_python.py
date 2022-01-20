from static_genome import genome
import socket
import zmq
import csv
from router import *

host = "127.0.0.1"
port = "30003"

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



one_frame = genome_2_cortical_list(genome['blueprint'])
CSV_writer(one_frame)
while True:
    FEAGI_sub = Sub(address="tcp://{}:{}".format(host, port), flags=zmq.NOBLOCK)
    one_frame = FEAGI_sub.receive()

    if one_frame is not None:
        print(one_frame)
    # one_frame = feagi_initalize() #disable to comment
    # print(one_frame)
    #UDP("[[0, 5, 90], [0, 4, 91], [0, 2, 93], [0, 3, 92]]")
    #breakdown(one_frame)
    # data = godot_listener().decode("utf-8")
    # data = data.split(",")
    # data_holder = data[0]
    # data_holder = ''.join([i for i in data_holder if not i.isdigit()])
    # data[0] = data_holder.replace("@", "")
    # data[2] = data[2].replace("(", "")
    # data[4] = data[4].replace(")", "")
    # print(data)
