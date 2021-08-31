# 4WD car's functions in Python
## Classes in controller.py
There are eight classes:
1. LED class
2. Ultrasonic class
3. Buzzer class
4. Motor class
5. Servo class
6. IR class
7. Photoresistor class
8. Battery class

Each class has their own functions. There is a multiple functions. Few of them are very similar as Robot Dog functions yet there's a huge difference on hardware side. 

## Controller.py
Controller.py allows you to control the buzzer, led, servo, and motors through PCA9685. You will be able to obtain the data from battery reader, IR and HC-SR04.

## Modulues
ADC.py, Led.py and PCA9685.py are modules. ADC is designed to work with photoresistor and battery to obtain the data. LED.py is defined system on the board and able to translate into the correct state. 
PCA9685 handles Servo and Motors classes by write the output into them.

## Motors
There is either invidiual motor, one side  or a full motors.
Functions for invidiual:
```
M3F() M3B()
M1F() M1B()
M2F() M2B()
M4F() M4B()
```
Those name are on the board as well.

There's extra functions for motors such as:
```
Backward()
Forward()
Right_Backward()
Left_Backward()
Left_Forward()
Right_Forward()
stop()
```

You can test all motors by using `motor_test_all()`
