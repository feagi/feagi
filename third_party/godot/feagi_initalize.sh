#!/bin/bash

FILE=./csv_data.csv
python3 bridge_godot_python.py &
cd ../godot_source
while  [ ! -f "$FILE" ]; do
  sleep 5
done
cp ../godot_source/csv_data.csv ../html/
cd ..
cd html
http-server