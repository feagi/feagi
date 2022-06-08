#!/bin/bash
/etc/init.d/dbus start
echo "Starting rfkill..."
rfkill unblock all
echo "Completed with rfkill."
echo "Creating bluetoothhd now..."
bluetoothd &
#/bin/bash
echo "Scanning bluetooth devices now..."
bluetoothctl power on
bluetoothctl --timeout 20 scan on
echo "Scanned them all. Starting the program."
python3 controller.py

