#!/usr/bin/env python3
import argparse
import subprocess
import sys
import os
import sysconfig
import feagi_agent_webcam
from feagi_agent_webcam.configuration import *

if __name__ == '__main__':
    # Check if feagi_agent has arg
    parser = argparse.ArgumentParser(description='configuration for any webcam')
    parser.add_argument('-loop', '--loop', help='Enable loop for the video', required=False)
    parser.add_argument('-ip', '--ip', help='Description for ip address argument', required=False)
    parser.add_argument('-device', '--device', help='To bind the location or index of webcam.', required=False)
    parser.add_argument('-video', '--video', help='Use the path to video to read', required=False)
    parser.add_argument('-port', '--port', help='Change the port instead of default 8000.', required=False)
    args = vars(parser.parse_args())
    if args['ip']:
        feagi_settings["feagi_host"] = args['ip']
    if args['loop'] == "true" or args['loop'] == "True":
        capabilities["camera"]["video_loop"] = bool(args['loop'])
    if args['device']:
        capabilities["camera"]["video_device_index"] = int(args['device'])
    else:
        capabilities["camera"]["video_device_index"] = 0
    if args['video']:
        capabilities["camera"]["video_device_index"] = args['video']
    if args['port']:
        feagi_settings["feagi_api_port"] = args['port']
    from feagi_agent_webcam import controller as webcam_controller
    webcam_controller.main(feagi_settings, agent_settings, capabilities, message_to_feagi)
