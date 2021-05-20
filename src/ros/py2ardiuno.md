# Publishing sonar sensor data through ROS2

This document is for sonar sensor (HC-0SR04)


## Set up with hardware:
```
VCC connects to 5V on the board.
Trig connects to pin #9
Echo connects to pin #11
GND connnects to GND
```

**IMPORTANT INFORMATION:** be sure that your board can take the 5v output. For example: DUE, ZERO, and other 32bit ardiuno boards _REQUIRE_ resistors in the output values. 8bits ardiuno boards such as UNO doesn't need a resistor. 


# To verify ardiuno/python3 only by doing this steps:
1. Navigate to /src/ros/ardiunotopython/
2. chmod a+x test.py
3. Click ardiunotopython.ino
4. Set board to your type of board and set the port up as well too on the official ardiuno IDE.
5. Upload it to your board.
6. On your terminal and type this; python3 test.py
7. If Python recieves value from ardiuno, it means you are ready to move to the ardiuno/python to ROS2 section.

# Ardiuno/python to ROS2

First time only:
1. navigate to ~/feagi-core/src/ros/
2. ./ws_setup.sh
3. Source ~/ros_ws/install/setup.bash
4. Be sure to have the setup.py match with this section:
    entry_points={
        'console_scripts': [
             'listener = py_topic.ros_laser_scan:main',
             'sonar_sensor = py_topic.HC_SR04_Foxy:main',
        ],
        

To update/test the modified by you on the code:
1. Source ~/ros_ws/install/setup.bash
2. ./update.sh (do this in the root of ros2_ws when you update the code in ~/ros_ws/src/py_topic/py_topic/HC_SR04_Foxy.py)
3. ros2 run py_topic sonar_sensor








# Problem section:

### Problem #1:
Traceback (most recent call last):
  File "./py2ardiuno", line 3, in <module>
    import pyfirmata
ModuleNotFoundError: No module named 'pyfirmata'

### Solution #1:
pip3 install pyfirmata

### Problem #2:
 No respond or stuck in Ardiuno() in Python3 code
### Solution #2:
 Use this line: board = Arduino("/dev/ttyACM0",baudrate=9600, timeout=3.0)

### Problem #3: 
Python doesn't recieve anything from ardiuno. No error output.
### Solution #3:
This has a lot of things that could cause this:
```
1. Is sonar sensor working properly? Verify it with only ardiuno. You can obtain it from (Sonar Sensor example (link))
2. Is sonar sensor connected properly? Are pins secured? 
3. Does sonar sensor connected to 5v?
4. Is Python listening to the same port address as ardiuno board in? (Such as /dev/ttyACM0 for Ubutnu, COM1 for windows) You can verify the port from ardiuno ide's bottom right corner on any platform.
5. is /dev/ttyACM0 in dialout group? Is it with root? Can it read/write? (Winodws 8 and 10 need to follow this link:https://www.isunshare.com/windows-8/how-to-disable-or-enable-driver-signature-enforcement-in-windows-8-and-8.1.html and https://www.howtogeek.com/167723/how-to-disable-driver-signature-verification-on-64-bit-windows-8.1-so-that-you-can-install-unsigned-drivers/ ) as for linux, sudo chmod 777 /dev/ttyACM0
6.  is ardiuno connected properly?
7. Does wiring match the number as variable set on the Trig and Echo on your code?
```

###  Problem #4:
Traceback (most recent call last):
  File "py2ardiuno", line 11, in <module>
    board = Arduino("/dev/ttyACM0", baudrate = 9600, timeout=2) # plugged in via USB, serial com at rate 115200
TypeError: __init__() got an unexpected keyword argument 'baudrate'

###  Solution #4:
import serial in the python file. Such as;
import serial

### Problem #5: 
Sonar sensor outputs "???????????" instead of values.

### Solution #5:
Is baudrate or Serial.begin(#) in 9600?

### Problem #6:
Sonar sensor kept outputs zero.

###  Solution #6:
It's likely defective sonar sensor but verify it with the newping's example first and see if it works. 




# More detail information: 
```
https://realpython.com/arduino-python/
https://www.arduino.cc/reference/en/libraries/newping/
https://riptutorial.com/arduino/example/22868/first-serial-communication-between-arduino-and-python
https://www.makeuseof.com/tag/program-control-arduino-python/
https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0
https://www.tutorialspoint.com/python/file_readline.htm
https://github.com/pyserial/pyserial/tree/master/serial
```
