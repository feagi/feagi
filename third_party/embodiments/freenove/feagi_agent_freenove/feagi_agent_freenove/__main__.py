#!/usr/bin/env python3
import argparse
import subprocess
import sys
import os
import sysconfig
import feagi_agent_freenove
import traceback
import requests
from time import sleep
from feagi_agent_freenove.configuration import *

if __name__ == '__main__':
    # Check if feagi_agent has arg
    parser = argparse.ArgumentParser(description='configuration for any webcam')
    parser.add_argument('-ip', '--ip', help='to connect FEAGI, required=False')
    parser.add_argument('-setup', '--setup', help='first time setup only', required=False)
    parser.add_argument('-zmq_port', '--zmq_port', help='zmq port to connect with FEAGI thru zmq',
                        required=False)
    parser.add_argument('-api_port', '--api_port', help='api_port for FEAGI thru API',
                        required=False)
    parser.add_argument('-port_disabled', '--port_disabled', help='not include port',
                        required=False)
    parser.add_argument('-http_type', '--http_type', help='https:// or http://',
                        required=False)
    parser.add_argument('-url', '--url', help='full url',
                        required=False)
    args = vars(parser.parse_args())
    if args['ip']:
        feagi_settings["feagi_host"] = args['ip']
    if args['setup']:
        current_path = feagi_agent_freenove.__path__
        new_path = current_path[0] + "/setup.sh " + current_path[0]
        subprocess.run([new_path, "arguments"], shell=True)
    if args['zmq_port']:
        agent_settings["agent_data_port"] = args['zmq_port']
    if args['api_port']:
        feagi_settings["feagi_api_port"] = args['api_port']
    from feagi_agent_freenove import controller as freenove_smartcar_controller
    if args['url']:
        feagi_settings['feagi_dns'] = args['url']
    feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
    print("FEAGI AUTH URL ------- ", feagi_auth_url)
    while True:
        try:
            freenove_smartcar_controller.main(feagi_auth_url,
                                              feagi_settings,
                                              agent_settings,
                                              capabilities)
        except Exception as e:
            print(f"Controller run failed", e)
            traceback.print_exc()
            sleep(2)
