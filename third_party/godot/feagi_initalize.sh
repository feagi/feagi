#!/bin/bash

FILE=./csv_data.csv
python3 bridge_godot_python.py &
cd ../html
ls
while  [ ! -f "$FILE" ]; do
  sleep 5
done
http-server