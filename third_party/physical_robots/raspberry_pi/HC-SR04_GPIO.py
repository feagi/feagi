#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
import zmq

socket_address = "tcp://0.0.0.0:2000"
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind(socket_address)

GPIO.setmode(GPIO.BCM)

TRIG = 27
ECHO = 22

print ("Distance Measurement In Progress")

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
try:
    while True:

        GPIO.output(TRIG, False)
        print ("Waiting For Sensor To Settle")
        time.sleep(2)

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO)==0:
          pulse_start = time.time()

        while GPIO.input(ECHO)==1:
          pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)

        print ("Distance:",distance,"cm")
        socket.send_pyobj(distance)

except KeyboardInterrupt: # If there is a KeyboardInterrupt (when you press ctrl+c), exit the program and cleanup
    print ("Closing")
    GPIO.cleanup()
