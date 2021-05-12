# Micro-ROS on FEAGI
## Setup on linux on Teensy and Arduino
1. Plug the micro USB to Teensy 4.1 (the board should have an orange blinking)
2. Press the button on the board.(The blinking should stop)
3. Open terminal and follow this list:
```
wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
wget https://www.pjrc.com/teensy/td_153/TeensyduinoInstall.linux64
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
tar -xf arduino-1.8.13-linux64.tar.xz
chmod 755 TeensyduinoInstall.linux64
./TeensyduinoInstall.linux64 --dir=arduino-1.8.13
cd arduino-1.8.13/hardware/teensy/avr/cores/teensy4
make
cd ~/arduino-1.8.13/
sudo ./install
```
4. Open and test with Teensy.
5. If it runs without any issue, you are good with Teensy and Arduino.


## Add Micro-ROS to Ardiuno
1. Download latest zip from (Micro-ROS)[https://github.com/micro-ROS/micro_ros_arduino/releases/tag/v1.0.0]
2. Open Arduino IDE
3. Click Sketch > Include library > Add Zip > [the zip you downloaded] 
4. cd [Ardiuno PATH]/hardware/teensy/avr/
5. curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
6. Click Download in Arduino IDE 
7. 




# Problem section:
## Problem 1: 
WARNING: library micro_ros_arduino-1.0.0 claims to run on OpenCR, Teensyduino, samd, sam, mbed architecture(s) and may be incompatible with your current board which runs on avr architecture(s).

## Solution 1: 
Go to arduino-1.8.13/hardware/teensy/avr
Then paste this, 
curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt


# More information:
1. https://www.youtube.com/watch?v=ze-HiCr5s60&ab_channel=pk
2. https://www.pjrc.com/teensy/td_download.html
3. https://github.com/micro-ROS/micro_ros_arduino
4. 

