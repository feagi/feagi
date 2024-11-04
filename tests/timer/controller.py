#!/usr/bin/env python
"""
Copyright 2016-present Neuraville Inc. All Rights Reserved.

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
import time
import psutil
import argparse
import requests
import traceback
from time import sleep
from version import __version__
from feagi_connector import sensors
from feagi_connector import pns_gateway as pns
from feagi_connector import feagi_interface as FEAGI

runtime_data = {}
previous_frame_data = {}
FEAGI.validate_requirements('requirements.txt')  # you should get it from the boilerplate generator


def current_milli_time():
    return time.time() * 1000

def prepare_the_genome_of_timer(args):
    # Prepare the file to be uploaded
    print("Loading genome now...")
    with open('test_timer.json', 'rb') as f:
        files = {'file': f}

        # Send the POST request with the file
        url = api_address + '/v1/genome/upload/file'
        response = requests.post(url, files=files)

    flag = True
    # print(response.status_code)
    while flag:
        # Print the response
        if response.status_code == 200:
            # print("genome loaded")
            flag = False
        else:
            # print("failed. type: ", type(response.status_code)," and status: ", response.status_code)
            sleep(1)


    json_data = {"cortical_id": "CM0Tim", "cortical_name": "Timer",
                 "cortical_dimensions": [int(args['number']), 100, 1]}
    url = api_address + '/v1/cortical_area/cortical_area'
    response = requests.put(url, json=json_data)
    flag = True
    while flag:
        # Print the response
        if response.status_code == 200:
            # print("updated timer cortical")
            flag = False
        else:
            # print("failed. type: ", type(response.status_code)," and status: ", response.status_code)
            response = requests.put(url, json=json_data)
            sleep(3)

    json_data = {"cortical_id": "o_misc", "cortical_name": "Miscellaneous",
                 "cortical_dimensions_per_device": [int(args['number']), 1, 1]}
    url = api_address + '/v1/cortical_area/cortical_area'
    response = requests.put(url, json=json_data)
    flag = True
    while flag:
        # Print the response
        if response.status_code == 200:
            # print("updated misc cortical")
            flag = False
        else:
            # print("failed. type: ", type(response.status_code)," and status: ", response.status_code)
            response = requests.put(url, json=json_data)
            sleep(3)
    print("genome is completed")



if __name__ == '__main__':
    config = FEAGI.build_up_from_configuration()
    feagi_settings = config['feagi_settings'].copy()
    agent_settings = config['agent_settings'].copy()
    default_capabilities = config['default_capabilities'].copy()
    message_to_feagi = config['message_to_feagi'].copy()
    capabilities = config['capabilities'].copy()


    message_to_feagi = {}
    parser = argparse.ArgumentParser(description='Number to update width')
    parser.add_argument('-number', '--number', help='for the number', default=10, required=False)

    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        FEAGI.connect_to_feagi(feagi_settings, runtime_data, agent_settings,
                               capabilities, __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - -
    args = vars(parser.parse_args())
    prepare_the_genome_of_timer(args)
    message_to_feagi = sensors.add_generic_input_to_feagi_data(
        {'i_misc': {"0-0-0": 100}}, message_to_feagi)
    pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel, agent_settings,
                         feagi_settings)

    start_time = current_milli_time()
    previous_total_time = start_time
    iteration_list = []
    flag = False
    while True:
        try:
            message_from_feagi = pns.message_from_feagi
            if message_from_feagi:
                obtained_signals = pns.obtain_opu_data(message_from_feagi)
                pns.check_genome_status_no_vision(message_from_feagi)
                if 'misc' in obtained_signals:
                    # print(obtained_signals['misc'])
                    for x in obtained_signals['misc']:
                        iteration = int(x) + 1
                        if iteration not in iteration_list:
                            total_numbers = iteration * 100
                            total_time = current_milli_time() - start_time  # unix time - unix time
                            time_delta = current_milli_time() - previous_total_time  # unix - (seconds)
                            time_per_neuron = time_delta / (100 * iteration)
                            cpu = psutil.cpu_percent(interval=1)
                            memory = psutil.virtual_memory().percent
                            print(f"iteration = {iteration}, total_numbers = {total_numbers}, total_time = {total_time}, time_delta = {time_delta}, time_per_neuron = {time_per_neuron}, cpu utilization = {cpu}, memory_usuage={memory}" )
                            # previous_total_time = total_time
                            iteration_list.append(iteration)
                            previous_total_time = current_milli_time()
                if len(iteration_list) == int(args['number']):
                    print("number reached!")
                    start_time = current_milli_time()
                    previous_total_time = start_time
                    iteration_list = []
                    if flag:
                        break
                    prepare_the_genome_of_timer(args)
                    json_data = {"global_visualization": False}
                    url = api_address + '/v1/system/global_activity_visualization'
                    response = requests.put(url, json=json_data)
                    print("! ! " * 10, "VISUALIZATION TOGGLED TO FALSE", " ! !" * 10)
                    flag = True
            if current_milli_time() - start_time >= 2000.0 and not iteration_list:
                # print("missed, trying again...")
                message_to_feagi = sensors.add_generic_input_to_feagi_data({'i_misc': {"0-0-0": 100}}, message_to_feagi)
                start_time = current_milli_time()
                previous_total_time = start_time


            sleep(feagi_settings['feagi_burst_speed'])  # bottleneck
            pns.signals_to_feagi(message_to_feagi, feagi_ipu_channel,
                                 agent_settings, feagi_settings)
            message_to_feagi.clear()
        except Exception as e:
            print("ERROR IN COZMO MAIN CODE: ", e)
            traceback.print_exc()
            break
