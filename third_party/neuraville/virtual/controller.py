
"""
This module contains a virtual set of hardware controllers used for simulation and testing

todo: Need to have all of the controller.py modules follow the same convention and be consistent especially input data
"""
from time import time

class LED:
    def __init__(self):
        self.led = Led()

    def LED_on(self, led_ID, Red_Intensity, Blue_Intensity, Green_intensity):
        """
        Parameters
        ----------
        led_ID: This is the ID of leds. It can be from 1 to 8
        Red_Intensity: 1 to 255, from dimmest to brightest
        Blue_Intensity: 1 to 255, from dimmest to brightest
        Green_intensity: 1 to 255, from dimmest to brightest
        -------
        """
        try:
            self.led.ledIndex(led_ID, Red_Intensity, Blue_Intensity, Green_intensity)
        except KeyboardInterrupt:
            self.led.colorWipe(led.strip, Color(0, 0, 0))  ##This is to turn all leds off/

    def test_Led(self):
        """
        This is to test all leds and do several different leds.
        """
        try:
            self.led.ledIndex(0x01, 255, 0, 0)  # Red
            self.led.ledIndex(0x02, 255, 125, 0)  # orange
            self.led.ledIndex(0x04, 255, 255, 0)  # yellow
            self.led.ledIndex(0x08, 0, 255, 0)  # green
            self.led.ledIndex(0x10, 0, 255, 255)  # cyan-blue
            self.led.ledIndex(0x20, 0, 0, 255)  # blue
            self.led.ledIndex(0x40, 128, 0, 128)  # purple
            self.led.ledIndex(0x80, 255, 255, 255)  # white'''
            print("The LED has been lit, the color is red orange yellow green cyan-blue blue white")
            time.sleep(3)  # wait 3s
            self.led.colorWipe("", Color(0, 0, 0))  # turn off the light
            print("\nEnd of program")
        except KeyboardInterrupt:
            self.led.colorWipe("", Color(0, 0, 0))  # turn off the light
            print("\nEnd of program")

    def leds_off(self):
        self.led.colorWipe("", Color(0, 0, 0))  ##This is to turn all leds off/


class IR:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

    def read(self):
        gpio_state = []
        ir_sensors = [self.IR01, self.IR02, self.IR03]
        for idx, sensor in enumerate(ir_sensors):
            if GPIO.input(sensor):
                gpio_state.append(idx)
        return gpio_state


class Buzzer(object):
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.buzzer_pin = 17  # set to GPIO pin 17. That's where the buzzer included.
        GPIO.setup(self.buzzer_pin, GPIO.IN)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        print("buzzer ready")

    def __del__(self):
        class_name = self.__class__.__name__
        print(class_name, "finished")

    def buzz(self, pitch, duration):  # create the function “buzz” and feed it the pitch and duration)
        """
        Parameters
        ----------
        pitch: pitch level
        duration: Seconds
        -------
        """

        if pitch == 0:
            time.sleep(duration)
            return
        period = 1.0 / pitch  # in physics, the period (sec/cyc) is the inverse of the frequency (cyc/sec)
        delay = period / 2  # calcuate the time for half of the wave
        cycles = int(duration * pitch)  # the number of waves to produce is the duration times the frequency

        for i in range(cycles):  # start a loop from 0 to the variable “cycles” calculated above
            GPIO.output(self.buzzer_pin, True)  # set pin 18 to high
            time.sleep(delay)  # wait with pin 18 high
            GPIO.output(self.buzzer_pin, False)  # set pin 18 to low
            time.sleep(delay)  # wait with pin 18 low

    def play(self, pitch_level, seconds): # The higher number, the higher pitch. The lower number, the lower pitch.
        """
        Parameters
        ----------
        pitch_level: pitch level
        seconds: duration
        -------
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        x = 0
        pitches = [pitch_level]
        duration = seconds
        for p in pitches:
            self.buzz(p, duration)  # feed the pitch and duration to the function, “buzz”
            time.sleep(duration * 0.5)

        GPIO.setup(self.buzzer_pin, GPIO.IN)


class Servo:
    """
    Functions: head_UP_DOWN and head_RIGHT_LEFT only. Other functions are just a support and defined system for Servo
    class to work with functions.
    """
    def __init__(self):
        print("Neuraville virtual servo has been initialized...")

    def setServoPwm(self,channel,angle,error=10):
        angle = int(angle)
        if channel == '0':
            self.PwmServo.setServoPulse(8,2500-int((angle+error)/0.09))
        elif channel == '1':
            self.PwmServo.setServoPulse(9,500+int((angle+error)/0.09))
        elif channel == '2':
            self.PwmServo.setServoPulse(10,500+int((angle+error)/0.09))
        elif channel == '3':
            self.PwmServo.setServoPulse(11,500+int((angle+error)/0.09))
        elif channel == '4':
            self.PwmServo.setServoPulse(12,500+int((angle+error)/0.09))
        elif channel == '5':
            self.PwmServo.setServoPulse(13,500+int((angle+error)/0.09))
        elif channel == '6':
            self.PwmServo.setServoPulse(14,500+int((angle+error)/0.09))
        elif channel == '7':
            self.PwmServo.setServoPulse(15,500+int((angle+error)/0.09))

    def head_UP_DOWN(self, num):
        """
        Parameters
        ----------
        num: degree from 0 to 180.
        -------
        """
        self.setServoPwm('1', num) #90 to 0 degree is turn the head down. 90 to 180 is to turn the head up

    def head_RIGHT_LEFT(self, num):
        """
        Parameters
        ----------
        num: degree from 0 to 180.
        -------
        """
        self.setServoPwm('0', num) #90 to 0 degree is turn the head left. 90 to 180 is to turn the head right


class Motor:
    def __init__(self):
        print("Neuraville virtual motor has been initialized...")

    def move(self, motor_index, speed):
        print("<< < ...Speed of motor %s is set to %s... > >>" % (motor_index, speed))


class Photoresistor:
    """
    Photoresistor has two photoresistors. 0 is the left photoresistor and 1 is the right photoresistor on the board.
    """
    def photoresistor(self, number):  # 0 is the left photoressitor and 1 is the right photoresistor
        adc = Adc()
        if number > 2 or number < 0:
            print("Please put 1 or 0 as an input only")
        elif number < 2 or number <= 0:
            output = adc.recvADC(number)
            # print(output)
            return output


class Battery:
    def battery_total(self):  ##It gives a full volt remain of battery
        """
        It reads the battery total. It will return the battery value.
        -------

        """
        adc = Adc()
        Power = adc.recvADC(2) * 3
        # print(Power)
        return Power


class Ultrasonic:
    def __init__(self):
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin,GPIO.OUT)
        GPIO.setup(self.echo_pin,GPIO.IN)

    def send_trigger_pulse(self):
        GPIO.output(self.trigger_pin,True)
        time.sleep(0.00015)
        GPIO.output(self.trigger_pin,False)

    def wait_for_echo(self,value,timeout):
        count = timeout
        while GPIO.input(self.echo_pin) != value and count>0:
            count = count-1

    def getDistance(self):
        """

        Returns distance_cm in INT value
        -------

        """
        distance_cm=[0,0,0]
        for i in range(3):
            self.send_trigger_pulse()
            self.wait_for_echo(True,10000)
            start = time.time()
            self.wait_for_echo(False,10000)
            finish = time.time()
            pulse_len = finish-start
            distance_cm[i] = pulse_len/0.000058
        distance_cm=sorted(distance_cm)
        return int(distance_cm[1])
