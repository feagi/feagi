#!/bin/bash

FILE=./csv_data.gdc
rm /root/html/*
rm /root/godot_source/csv_data.gdc
echo "% pre-bridge launch"
python3 bridge_godot_python.py &
echo "% post-bridge launch"

cd ../html
live-server &

cd ../godot_source/

while true; do
  while [ ! -s csv_data.gdc ]; do
    echo "* * Awaiting new CSV * *"
    sleep 1
  done
  echo "\n\n\n ### ### ### ### ### ### ### ### New CSV has been detected \n\n\n"
  ./Godot_v3.4.4-stable_linux_headless.64 --export "HTML5"
  rm ./csv_data.gdc

done