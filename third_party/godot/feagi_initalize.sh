#!/bin/bash

FILE=./csv_data.gdc
rm /root/html/*
rm /root/godot_source/csv_data.gdc
python3 bridge_godot_python.py &


cd ../godot_source/

while [ ! -s csv_data.gdc ]; do
  sleep 0
done
./Godot_v3.4.4-stable_linux_headless.64 --export "HTML5" 
cd ../html
http-server -c-1
