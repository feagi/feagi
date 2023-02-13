#!/usr/bin/env python3
import argparse
import subprocess
import sys
import os
import sysconfig
import feagi_agent_mycobot
import platform


def read_contents(file_path):
    with open(file_path, 'r') as f:
        return f.read()


if __name__ == '__main__':
    if platform.uname()[0] == "Linux":
        # Check if feagi_agent has arg
        parser = argparse.ArgumentParser(description='configuration for any webcam')
        parser.add_argument('-ip', '--ip', help='Description for ip address argument', required=False)
        args = vars(parser.parse_args())
        current_path = feagi_agent_mycobot.__path__
        path = current_path[0] + "/src/configuration.py"
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
        subprocess.run(["./start_mycobot.sh"], cwd=current_path[0])
    elif platform.uname()[0] == "Windows":
        print("This feature is currently not supported on Windows.")
        print("Please report any issues at https://github.com/feagi/feagi/issues.")
        print("Currently, only Linux is supported. Windows and Mac are still under development. "
              "If you are interested in making this work, please submit an issue on the Feagi GitHub page. "
              "We will get in touch with you.")
    elif platform.uname()[0] == "Darwin":
        print("This feature is currently not supported on Mac.")
        print("Please report any issues at https://github.com/feagi/feagi/issues.")
        print("Currently, only Linux is supported. Windows and Mac are still under development. "
              "If you are interested in making this work, please submit an issue on the Feagi GitHub page. "
              "We will get in touch with you.")
    else:
        print(platform.uname())
        print("Please report any issues at https://github.com/feagi/feagi/issues.")
