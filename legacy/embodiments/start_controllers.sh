#!/bin/bash

# Define a cleanup function
cleanup() {
    echo "Terminating scripts"
    if [[ ! -z "$PID1" ]]; then
        kill $PID1
        echo "media capture script terminated"
    fi
    if [[ ! -z "$PID2" ]]; then
        kill $PID2
        echo "Bluetooth script terminated"
    fi
        if [[ ! -z "$PID3" ]]; then
        kill $PID3
        echo "Godot games script terminated"
    fi
        if [[ ! -z "$PID4" ]]; then
        wait $PID4
    fi
    echo "All scripts terminated"
    exit 0
}

trap cleanup SIGTERM SIGINT

export FEAGI_HOST_INTERNAL=$(hostname)
unset INFLUXDB

cd /root/godot-bridge/
python3 bridge_godot_python.py &
cd /root/

# Load javascript media capture if WEBCAM_FLAG is true
if [[ "$WEBCAM_FLAG" == "true" ]]; then
    cd media_capture_controller/
    python3 controller.py &
    PID1=$!
    echo "PID of the media capture: $PID1"
    cd /root/
fi

# Load microbit if BT_CONTROLLER_FLAG is true
if [[ "$BT_CONTROLLER_FLAG" == "true" ]]; then
    cd bluetooth_controller/
    python3 controller.py &
    PID2=$!
    echo "PID of the Bluetooth CTRL: $PID2"
    cd /root/
fi

# Load microbit if BT_CONTROLLER_FLAG is true
if [[ "$GODOT_GAMES_FLAG" == "true" ]]; then
    cd godot-games-controller
    python3 controller.py &
    PID3=$!
    echo "PID of the godot games: $PID3"
    cd /root/
fi

# Load microbit if BT_CONTROLLER_FLAG is true
if [[ "$WEBSOCKET_BRIDGE" == "true" ]]; then
    cd controller-bridge
    python3 controller.py &
    PID4=$!
    echo "PID of the controller-bridge: $PID4"
    cd /root/
fi

# Load another godot game controller if GODOT_GAMES_2_FLAG is true
if [[ "$GODOT_GAMES_2_FLAG" == "true" ]]; then
    cd godot-games-controller
    export WS_GODOT_GENERIC_PORT="9056"
    python3 controller.py &
    PID5=$!
    echo "PID of the godot games 2: $PID5"
    cd /root/
fi

cd /opt/source-code/feagi/src/
python3 main.py

# Optionally, wait for the scripts to finish if they were started
if [[ ! -z "$PID1" ]]; then
    wait $PID1
fi

if [[ ! -z "$PID2" ]]; then
    wait $PID2
fi

if [[ ! -z "$PID3" ]]; then
    wait $PID3
fi

if [[ ! -z "$PID4" ]]; then
    wait $PID4
fi

if [[ ! -z "$PID5" ]]; then
    wait $PID5
fi

cleanup
