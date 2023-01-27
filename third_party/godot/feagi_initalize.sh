#!/bin/bash

python3 bridge_godot_python.py &
echo "% post-bridge launch"

cd ../html
live-server &

cd ../godot_source/
./Godot_v3.4.4-stable_linux_headless.64 --export "HTML5"

while true
  do
    :
  done
