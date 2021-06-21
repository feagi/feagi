#!/bin/bash

cd arduino-cli/
export PATH=$PATH:/home/ubuntu/arduino-cli/bin
arduino-cli board attach serial:///dev/ttyACM0 micro-ros_publisher
arduino-cli compile --port /dev/ttyACM0 micro-ros_publisher
arduino-cli upload --port /dev/ttyACM0 micro-ros_publisher
