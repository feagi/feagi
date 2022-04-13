#!/bin/bash
FILE=./csv_data.csv
xterm -hold -e "echo -e '\033]2;'FEAGI_bridge'\007' && python3 bridge_godot_python.py" &
python3 -m http.server
#FEAGI_monitor.html
