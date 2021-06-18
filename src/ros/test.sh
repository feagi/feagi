#!/bin/bash

cd arduino-cli/
export PATH=$PATH:/home/bwuk/arduino-cli/bin
arduino-cli compile --port /dev/ttyACM0 micro-ros_publisher
arduino-cli upload --port /dev/ttyACM0 micro-ros_publisher
