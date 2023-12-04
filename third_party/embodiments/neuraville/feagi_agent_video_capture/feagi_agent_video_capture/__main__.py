#!/usr/bin/env python3
import argparse
import subprocess
import sys
import os
import sysconfig
import feagi_agent_video_capture
import traceback
from time import sleep
from feagi_agent_video_capture.configuration import *

if __name__ == '__main__':
    # Check if feagi_agent has arg
    parser = argparse.ArgumentParser(description='configuration for any webcam')
    parser.add_argument('-loop', '--loop', help='Enable loop for the video', required=False)
    parser.add_argument('-ip', '--ip', help='Description for ip address argument', required=False)
    parser.add_argument('-device', '--device', help='To bind the location or index of webcam.',
                        required=False)
    parser.add_argument('-video', '--video', help='Use the path to video to read', required=False)
    parser.add_argument('-port', '--port', help='Change the port instead of default 8000.',
                        required=False)
    args = vars(parser.parse_args())
    if args['ip']:
        feagi_settings["feagi_host"] = args['ip']
    if args['loop'] == "true" or args['loop'] == "True":
        capabilities["camera"]["video_loop"] = bool(args['loop'])
    if args['device']:
        if args['device'] == "monitor":
            capabilities["camera"]["video_device_index"] = "monitor"
        else:
            capabilities["camera"]["video_device_index"] = int(args['device'])
    else:
        capabilities["camera"]["video_device_index"] = 0
    if args['video']:
        capabilities["camera"]["video_device_index"] = args['video']
    if args['port']:
        feagi_settings["feagi_api_port"] = args['port']
    if __name__ == '__main__':
        inital_feagi_setting = feagi_settings.copy()
        inital_agent_settings = agent_settings.copy()
        inital_capabilities = capabilities.copy()
        inital_message_to_feagi = message_to_feagi.copy()
        while True:
            try:
                from feagi_agent_video_capture import controller as video_controller

                feagi_auth_url = feagi_settings.pop('feagi_auth_url', None)
                print("FEAGI AUTH URL ------- ", feagi_auth_url)
                video_controller.main(feagi_auth_url, feagi_settings, agent_settings,
                                      capabilities, message_to_feagi)
            except Exception as e:
                feagi_settings = inital_feagi_setting.copy()
                agent_settings = inital_agent_settings.copy()
                capabilities = inital_capabilities.copy()
                message_to_feagi = inital_message_to_feagi.copy()
                print(f"Controller run failed", e)
                # traceback.print_exc()
                sleep(2)
