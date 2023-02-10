#!/usr/bin/env python3
import argparse
import subprocess
import sys
import os
import sysconfig
import feagi_agent_webcam


def read_contents(file_path):
    with open(file_path, 'r') as f:
        return f.read()

if __name__ == '__main__':
    # Check if feagi_agent has arg
    parser = argparse.ArgumentParser(description='configuration for any webcam')
    parser.add_argument('-ip', '--ip', help='Description for ip address argument', required=False)
    args = vars(parser.parse_args())
    current_path = feagi_agent_webcam.__path__
    path = current_path[0] + "/configuration.py"
    obtain_line = ""
    whole_file = ""
    if args['ip']:
        with open(path, "r") as f:
            for textline in f:
                if "feagi_host" in textline:
                    obtain_line = textline
        whole_file = read_contents(path)
        with open(path, "w") as f:
            new_file = whole_file.replace(obtain_line, "     \"feagi_host\": \"" + args['ip'] + "\",\n")
            f.write(new_file)
    from feagi_agent_webcam import controller as webcam_controller
    webcam_controller.main()