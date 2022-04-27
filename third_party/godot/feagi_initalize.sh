#!/bin/bash

FILE=./csv_data.gdc

python3 bridge_godot_python.py &


cd ../godot_source/
echo "---------------- +++ -----------------"
while  [ ! -f "$FILE" ]; do
  echo "./\."
  sleep 5
done

echo "CSV exists!"

while [ ! -s csv_data.gdc ]; do
  echo "Waiting on csv_data being generated..."
  sleep 2
done

sleep 10

./Godot_v3.4.4-stable_linux_headless.64 --export "HTML5" 
cd ../html
http-server
