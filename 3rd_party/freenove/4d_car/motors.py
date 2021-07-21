#This code is designed by Freenove: https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/blob/master/Code/Server/Motor.py
#This is a tutorial where the freenove board is being used with motors using PCA9685 (PWN controller)
# PCA9685.py must be in the same directory as this file. 

#!/usr/bin/python3

from PCA9685 import PCA9685
class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
    def duty_range(self,duty1,duty2,duty3,duty4):
        if duty1>4095:
            duty1=4095
        elif duty1<-4095:
            duty1=-4095        
        
        if duty2>4095:
            duty2=4095
        elif duty2<-4095:
            duty2=-4095
            
        if duty3>4095:
            duty3=4095
        elif duty3<-4095:
            duty3=-4095
            
        if duty4>4095:
            duty4=4095
        elif duty4<-4095:
            duty4=-4095
        return duty1,duty2,duty3,duty4
        
    def left_Upper_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(0,0)
            self.pwm.setMotorPwm(1,duty)
        elif duty<0:
            self.pwm.setMotorPwm(1,0)
            self.pwm.setMotorPwm(0,abs(duty))
        else:
            self.pwm.setMotorPwm(0,4095)
            self.pwm.setMotorPwm(1,4095)
    def left_Lower_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(3,0)
            self.pwm.setMotorPwm(2,duty)
        elif duty<0:
            self.pwm.setMotorPwm(2,0)
            self.pwm.setMotorPwm(3,abs(duty))
        else:
            self.pwm.setMotorPwm(2,4095)
            self.pwm.setMotorPwm(3,4095)
    def right_Upper_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(6,0)
            self.pwm.setMotorPwm(7,duty)
        elif duty<0:
            self.pwm.setMotorPwm(7,0)
            self.pwm.setMotorPwm(6,abs(duty))
        else:
            self.pwm.setMotorPwm(6,4095)
            self.pwm.setMotorPwm(7,4095)
    def right_Lower_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(4,0)
            self.pwm.setMotorPwm(5,duty)
        elif duty<0:
            self.pwm.setMotorPwm(5,0)
            self.pwm.setMotorPwm(4,abs(duty))
        else:
            self.pwm.setMotorPwm(4,4095)
            self.pwm.setMotorPwm(5,4095)
            
 
    def setMotorModel(self,duty1,duty2,duty3,duty4):
        duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)
    def stop():
    	PWM.setMotorModel(0,0,0,0)


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
