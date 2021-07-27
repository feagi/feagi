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
        Parameters
        ----------
        led_ID: This is the ID of leds. It can be from 1 to 8
        Red_Intensity: 1 to 255, from dimmest to brightest
        Blue_Intensity: 1 to 255, from dimmest to brightest
        Green_intensity: 1 to 255, from dimmest to brightest
        -------
        """
```

###### leds_off()
```
        """
        No parameters
        ----------
	Turn all leds off
```

###### test_Led()
```
        """
        No parameters
        ----------
        Test all leds
```

###### getDistance()
```
        """

        Returns distance_cm in INT value
        -------

        """
```
###### buzz(self, pitch, duration):
```
        """

        Parameters
        ----------
        pitch: pitch level
        duration: Seconds
        -------

        """
```
###### play(self, pitch_level, seconds):
```
        """

        Parameters
        ----------
        pitch_level: pitch level
        seconds: duration
        -------

        """
```

###### battery(channel)
```
        """

        Parameters
        ----------
        channel: 1 to 8
        -------
        """
```
###### imuUpdate()
```
        """

        Returns z,x, and y
        -------

        """
```

##### Class of servo:
Function: 
```
FL1, FL2, FL3 (Front left)
RL1, RL2, RL3 (Rear left)
FR1, FR2, FR3 (Front right)
RR1, RR2, RR3 (Rear right)
head
```
```
        """
        Parameters
        ----------
        degree: 0 to 180 degree
        -------
        """
```
