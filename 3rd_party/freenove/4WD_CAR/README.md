# 4WD car's functions in Python
## Classes in controller.py
There are six classes:
1. LED class
2. Ultrasonic class
3. Buzzer class
4. PCA9685 (PWM controller) class
5. Motor class
6. Servo class

Each class has their own functions. There is a multiple functions. Few of them are very similar as Robot Dog functions yet there's a huge difference on hardware side. 

## Controller.py
Controller.py allows you to control the buzzer, led, servo, and motors through PCA9685. You will be able to obtain the data from battery reader, and HC-SR04.

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
