# Robot dog's functions in Python
## Classes in controller.py
There are six classes:
1. LED class
2. Ultrasonic class
3. Buzzer class
4. ADS7830(Battery reader) class
5. IMU class
6. Servo class

Each class has their own functions. There is a multiple functions. Few of them are very similar as 4WD functions yet there's a huge difference on hardware side. 

## Controller.py
Controller.py allows you to control the buzzer, led, servo. You will be able to obtain the data from IMU, battery reader, and HC-SR04.

## Servos on the robot dog

The full map of servos:
```
head(degree) (HEAD)
FL1(degree), FL2(degree), FL3(degree) #Front left
RL1(degree), RL2(degree), RL3(degree) #Rear left
FR1(degree), FR2(degree), FR3(degree) #Front right
RR1(degree), RR2(degree), RR3(degree) #Rear right
```
The degree on the servo can move from 0 to 180 degree. It's mechanically limited to 180 degree only. The code is implemented to keep it between 0 to 180 degree. If it's more than 180 degree or less than 0 degree, it will be prevented by the code. 



![image](https://user-images.githubusercontent.com/65916520/127222119-cf8c231f-684c-4797-9098-c85750a9f6fb.png)

