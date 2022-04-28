#!/bin/bash

FILE=./csv_data.gd
rm /root/html/*
rm /root/godot_source/csv_data.gd
python3 bridge_godot_python.py &


cd ../godot_source/

while [ ! -s csv_data.gd ]; do
  echo "Waiting on csv_data being generated..."
  sleep 2
done
./Godot_v3.4.4-stable_linux_headless.64 --export "HTML5" 
cd ../html
http-server
