#!/bin/bash

python3 bridge_godot_python.py &
echo "% post-bridge launch"

cd ../html
live-server &

cd ../godot_source/

api_ip="$GODOT_TO_API"
api_port="$API_PORT"
websocket_ip="$GODOT_WEBSOCKET"
websocket_port="$WEBSOCKET_PORT"

sed -i "s/var websocket_ip_address = \"127\.0\.0\.1\"/var websocket_ip_address = \"$websocket_ip\"/g" network_configuration.gd
sed -i "s/var api_port_address = \"8000\"/var api_port_address = \"$api_port\"/g" network_configuration.gd
sed -i "s/var api_ip_address = \"127\.0\.0\.1\"/var api_ip_address = \"$api_ip\"/g" network_configuration.gd
sed -i "s/var websocket_port_address = \"9050\"/var websocket_port_address = \"$websocket_port\"/g" network_configuration.gd


./Godot_v3.4.4-stable_linux_headless.64 --export "HTML5"

while true
  do
    :
  done