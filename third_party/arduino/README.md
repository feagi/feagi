# Set up with py2arduino
This method allows you to customize heavily using Python only and able to read ardiuno, obtain the value and output the value as well. So, Python can run on Foxy. The picture below displays how the py2ardiuno works:
![image](https://user-images.githubusercontent.com/65916520/119179001-a3d5f400-ba2b-11eb-8266-7dc10cf026d0.png)

modules being used in this method:
```
rclpy //for ros2
serial //to read arduino's output
time //timeout 
std_msgs //to use float/int
```

# Publishing sonar sensor data through ROS2

This document is for sonar sensor (HC-0SR04)


# Set up with hardware:
## 8bit board such as Arduino UNO.
Wiring on the board
```
VCC connects to 5V on the board.
Trig connects to pin #9
Echo connects to pin #11
GND connnects to GND
```

## 32bit board such as Teensy, Arduino ZERO, or Arduino DUE
Wiring on the board:
```
VCC connects to 5V on the board
Trig connects to pin #9, 1k resistor, and 2k resistor (series)
Echo connects to pin #11
GND connects to GND, and 2k resistor (series)
```
Schematic for 32bit:

![schematic](https://user-images.githubusercontent.com/65916520/119182601-5ad46e80-ba30-11eb-9e26-878c47a0a311.png)


**IMPORTANT INFORMATION:** be sure that your board can take the 5v output. For example: DUE, ZERO, and other 32bit ardiuno boards _REQUIRE_ resistors in the output values. 8bits ardiuno boards such as UNO doesn't need a resistor. 

More information is in the section, "More detail information" in the bottom of the file.


## To verify ardiuno/python3 only by doing this steps:
1. Navigate to /src/ros/ardiunotopython/ in terminal
2. chmod a+x test.py in terminal
3. Click ardiunotopython.ino to open arduino IDE
4. Set board to your type of board and set the port up as well too on the official ardiuno IDE.
5. Upload it to your board.
6. On your terminal and type this; python3 test.py
7. If Python recieves value from ardiuno, it means you are ready to move to the ardiuno/python to ROS2 section.

# Ardiuno/python to ROS2

First time only:
1. navigate to ~/feagi/src/ros/
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



## Py2arduino on FEAGI
More information: [FEAGI](py2ardiuno_FEAGI.md)




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

### Problem #7:
No executable found response when running ros2 run py_topic sonar_sensor

### Solution #7
is setup.py configured properly in ~/ros_ws/src/py_topic/ ?

Alternate option: 
cp ~/feagi/src/ros/setup.py ~/ros2_ws/src/py_topic/

### Problem #8:
Traceback (most recent call last):
  File "/home/ubuntu/ros2_ws/install/py_topic/lib/py_topic/sonar_sensor", line 11, in <module>
    load_entry_point('py-topic==0.0.0', 'console_scripts', 'sonar_sensor')()
  File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 490, in load_entry_point
    return get_distribution(dist).load_entry_point(group, name)
  File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2854, in load_entry_point
    return ep.load()
  File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2445, in load
    return self.resolve()
  File "/usr/lib/python3/dist-packages/pkg_resources/__init__.py", line 2451, in resolve
    module = __import__(self.module_name, fromlist=['__name__'], level=0)
  File "/home/ubuntu/ros2_ws/install/py_topic/lib/python3.8/site-packages/py_topic/HC_SR04_Foxy.py", line 2, in <module>
    import serial
ModuleNotFoundError: No module named 'serial'

### Solution #8:
pip3 install pyserial



# More detail information: 
```
https://realpython.com/arduino-python/
https://www.arduino.cc/reference/en/libraries/newping/
https://riptutorial.com/arduino/example/22868/first-serial-communication-between-arduino-and-python
https://www.makeuseof.com/tag/program-control-arduino-python/
https://create.arduino.cc/projecthub/ansh2919/serial-communication-between-python-and-arduino-e7cce0
https://www.tutorialspoint.com/python/file_readline.htm
https://github.com/pyserial/pyserial/tree/master/serial
https://learn.sparkfun.com/tutorials/voltage-dividers/all
```

32bit boards has a 3.3v maximum with 1% tolerance (3.5v). We can use the equation to determine the specific series voltage output:
![image](https://user-images.githubusercontent.com/65916520/119181148-6c1c7b80-ba2e-11eb-950b-37d242122bd1.png)

So, we have 1k and 2k resistors.
The output will be 3.3v which is under 3.5v. This will be safe to use on 32bit.

