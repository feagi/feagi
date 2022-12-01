#!/bin/bash

FLAG=true
FILE=./csv_data.gdc
rm ../html/*
cp ../html_backup/* ../html/
rm /root/godot_source/csv_data.gdc
echo "% pre-bridge launch"
python3 bridge_godot_python.py &
echo "% post-bridge launch"

cd ../html

live-server &
#http-server -c-1 &

cd ../godot_source/

while true; do
  while [ ! -f csv_data.gdc ]; do
    if [ -f reset.txt ]; then
      if $FLAG; then
        FLAG=false
        rm ../html/*
        cp ../html_backup/* ../html/
      fi
    fi
  done
  echo "\n\n\n ### ### ### ### ### ### ### ### New CSV has been detected \n\n\n"

#  rm ../html/*
#  cp ../html_backup/* ../html/
# commented above as it's already handled in bridge
  ./Godot_v3.4.4-stable_linux_headless.64 --export "HTML5"
  rm ./csv_data.gdc
  rm ./reset.txt
  FLAG=true

done
