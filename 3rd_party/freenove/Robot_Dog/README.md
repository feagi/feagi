# Robot dog's functions in Python
## Classes
There is six classes:
1. LED class
2. Ultrasonic class
3. Buzzer class
4. ADS7830(Battery reader) class
5. IMU class
6. Servo class

Each class has their own functions. There is a multiple functions. Few of them are very similar as 4WD functions yet there's a huge difference on hardware side. 

## Functions on Python
###### LED_on(self, led_ID, Red_Intensity, Blue_Intensity, Green_intensity):
```
        """
	Intensity in the parameter from dimmest to brightest. This is using the RGB color combination: https://www.w3schools.com/colors/colors_rgb.asp
	led ID is available from 1 to 8.
        """
```

###### leds_off()
```
	Turn all leds off.
```

###### test_Led()
```
        Test all leds.
```

###### getDistance()
```
        """
	Measure the distance in Centimeter.
        """
```
###### buzz(self, pitch, duration):
```
        """
	Beep sound.
        """
```
###### play(self, pitch_level, seconds):
```
        """
	Play a note/piano.
        """
```

###### battery(channel)
```
        """
        Channel: 1 to 8 to read the battery
        """
```
###### imuUpdate()
```
        """
	Update with x,y,z in realtime.
        """
```

##### Functions in the Servo class:
Functions: 
```
head (HEAD)
FL1, FL2, FL3 (Front left)
RL1, RL2, RL3 (Rear left)
FR1, FR2, FR3 (Front right)
RR1, RR2, RR3 (Rear right)
```
```
        """
	Can move between 0 to 180 degree. Any number go past 180 or 0 will be ignored by the safety detector. The map of robot is on the picture below.
        """
```
![image](https://user-images.githubusercontent.com/65916520/127222119-cf8c231f-684c-4797-9098-c85750a9f6fb.png)

