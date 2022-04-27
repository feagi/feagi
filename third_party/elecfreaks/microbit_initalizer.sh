#!/bin/bash
/etc/init.d/dbus start
rfkill unblock all
bluetoothd &
/bin/bash
bluetoothctl scan on $ 
FOO_PID=$! 

kill $FOO_PID

