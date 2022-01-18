import zmq
import socket
import time
import json
from static_genome import genome
import csv



host = "feagi"
port = "30003"

def FEAGI_initalize():
    # Getting FEAGI's raw data
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    subby = socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))
    #print(subby)
    print('Listening FEAGI...')
    socket.connect("tcp://{}:{}".format(host, port))
    socket.set(zmq.SUBSCRIBE, ''.encode('utf-8'))
    set_stored = socket.recv_pyobj()
    # print(set_stored)
    # print(type(set_stored))
    return set_stored

def UDP(input):
    ip     = "127.0.0.1"
    port   = 20001

    # Create socket for server
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)

    #while True: ##this must be in loop in order to work with FEAGI
    s.sendto(input.encode('utf-8'), (ip, port))
    print("\n\n 1. This Sent to Godot with the input: ", input, "\n\n")
    # close the socket
    s.close()

def genome_2_cortical_list(flat_genome):
    """
    Generates a list of cortical areas inside genome
    """
    cortical_list = {}
    godot_cortical_list =list()
    for key in flat_genome:
        #print(key[17:])
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
            #print(genome["blueprint"][key])
        if key[22:] == "bbx-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
        if key[22:] == "bby-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
        if key[22:] == "bbz-i":
            cortical_list[cortical_id].append(genome["blueprint"][key])
    # for key in cortical_list:
    #     godot_cortical_list.append(cortical_list[key][2])
    #     godot_cortical_list.append(cortical_list[key][3])
    #     godot_cortical_list.append(cortical_list[key][4])
    #     godot_cortical_list.append(cortical_list[key][5])
    #     godot_cortical_list.append(cortical_list[key][6])
    #     godot_cortical_list.append(cortical_list[key][7])
    #     godot_cortical_list.append(cortical_list[key][0])
    #print(godot_cortical_list)
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

    # for cortical_area in genome["blueprint"]:
    #     if "visualization" in genome["blueprint"][cortical_area]["neuron_params"]:
    #         if genome['blueprint'][cortical_area]['neuron_params']['visualization']:
    #             writer.writerow((
    #                 genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][0],
    #                 genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][1],
    #                 genome['blueprint'][cortical_area]['neuron_params']['relative_coordinate'][2],
    #                 genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][0],
    #                 genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][1],
    #                 genome['blueprint'][cortical_area]['neuron_params']['block_boundaries'][2],
    #                 cortical_area
    #             ))


def breakdown(feagi_input): ##add input soon
    """
    #TODO: Explain what this is for
    """
    data = feagi_input
    data = list(data)
    increment = 0
    voxel = []
    list1 = []
    keys = range(len(data))

    while increment < len(data):
        voxel = [data[increment][1], data[increment][2], data[increment][3]]
        list1.append(voxel)

        #print(list1)
        UDP(str(voxel))
        #time.sleep(1.0)
        increment += 1
    print(list1)
    UDP(str(list1))


one_frame = genome_2_cortical_list(genome['blueprint'])
CSV_writer(one_frame)
while True:
    one_frame = FEAGI_initalize()
    #print(one_frame)
    #UDP("[[0, 5, 90], [0, 4, 91], [0, 2, 93], [0, 3, 92]]")
    breakdown(one_frame)

