#!/bin/bash

#mv Godot_v3.2.2-stable_export_templates.tpz FEAGI_activity
godot3 project.godot --export "Linux/X11" FEAGI_activity
xterm -hold -e "echo -e '\033]2;'FEAGI_activity'\007' && python3 bridge_godot_python.py" &
#./FEAGI_activity_container
