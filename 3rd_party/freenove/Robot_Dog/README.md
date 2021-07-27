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
###Led class:
```
LED_on(led_ID, Red_Intensity, Blue_Intensity, Green_intensity)
leds_off()
test_leds()
```

LED_on is using the RGB combination. 
If you put like this:
`new.LED_on(1,1,0,0)`
The red in #1 led will dim.
If you put 100 in the red intensity like this:
`new.LED_on(1,255,0,0)`
The red in the #1 led will be very bright. 

So, if you put `LED_on(1,204,204,255)`, it will be lavender blue color ish. It's using the RGB rule as well.

`leds_off()` will turn all leds off so be sure to add this in the code.

`test_leds()` is designed by Freenove. It's good to test and see if all leds are working properly. 


