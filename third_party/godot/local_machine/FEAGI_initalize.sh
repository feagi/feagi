#!/bin/bash

# FEAGI static_genome.py
#git clone -b feature-publish_brain_activities https://github.com/feagi/feagi-core
#cp ~/feagi-core/src/evo/static_genome.py .
./FEAGI_activity_container.x86_64 &
xterm -hold -e "echo -e '\033]2;'FEAGI_activity'\007' && python3 bridge_godot_python.py" &

