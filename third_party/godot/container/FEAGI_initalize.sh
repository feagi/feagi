#!/bin/bash

#mv Godot_v3.2.2-stable_export_templates.tpz FEAGI_activity
#mkdir -p ./.local/share/godot/templates/3.2.stable/linux_x11_64_release/
#wget https://downloads.tuxfamily.org/godotengine/3.2.2/Godot_v3.2.stable_export_templates.tpz
#mv Godot_v3.2-stable_export_templates ./.local/share/godot/templates/3.2.stable/linux_x11_64_release/

FILE=./csv_data.csv
xterm -hold -e "echo -e '\033]2;'FEAGI_bridge'\007' && python3 bridge_godot_python.py" &

while  [ ! -f "$FILE" ]; do
  sleep 5
done

xterm -hold -e "echo -e '\033]2;'FEAGI_activity'\007' && ./Godot_v3.4.2-stable_x11.64" &
#./FEAGI_activity_container
