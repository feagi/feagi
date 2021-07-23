#!/usr/bin/python3

import motors
import time

def Backward():
	PWM.setMotorModel(2000,2000,2000,2000)

PWM=Motor()

def stop():
	PWM.setMotorModel(0,0,0,0)

def Forward():
	PWM.setMotorModel(-2000,-2000,-2000,-2000)

def Right_Backward():
	PWM.setMotorModel(-500,-500,2000,2000)

def Left_Backward():
	PWM.setMotorModel(2000,2000,-500,-500)       #Right

def Left_Forward():
	PWM.setMotorModel(-2000,-2000,-500,-500)

def Right_Forward():
        PWM.setMotorModel(-500,-500,-2000,-2000)

def M3F():
        PWM.setMotorModel(0,0,0,-2000) #M3 forward
def M3B():
        PWM.setMotorModel(0,0,0,2000) #M3 backward

def M1F():
        PWM.setMotorModel(-2000,0,0,0) #M1 forward

def M1B():
        PWM.setMotorModel(2000,0,0,0) #M1 backward

def M2F():
        PWM.setMotorModel(0,-2000,0,0) #M2 forward

def M2B():
        PWM.setMotorModel(0,2000,0,0) #M2 backward

def M4F():
        PWM.setMotorModel(0,0,-2000,0) #M4 forward
def M4B():
        PWM.setMotorModel(0,0,2000,0) #M4 backward

if __name__ == '__main__':
        motors.M1F()
        time.sleep(3)
        motors.M1B()
        time.sleep(3)
        motors.M2F()
        time.sleep(3)
        motors.M2B()
        time.sleep(3)
        motors.M3F()
        time.sleep(3)
        motors.M3B()
        time.sleep(3)
        motors.M4F()
        time.sleep(3)
        motors.M4B()
        time.sleep(3)
        motors.stop()
