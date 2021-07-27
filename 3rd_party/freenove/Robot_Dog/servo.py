#coding:utf-8
import Adafruit_PCA9685
import time
class Servo:
    def __init__(self):
        self.angleMin=18
        self.angleMax=162
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)               # Set the cycle frequency of PWM
    #Convert the input angle to the value of pca9685
    def map(self,value,fromLow,fromHigh,toLow,toHigh):
        return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow
    def setServoAngle(self,channel, angle):
        if angle < self.angleMin:
            angle = self.angleMin
        elif angle >self.angleMax:
            angle=self.angleMax
        date=self.map(angle,0,180,102,512)
        #print(date,date/4096*0.02)
        self.pwm.set_pwm(channel, 0, int(date))
    def FL1(self,degree):
        degree = int(degree)
        servo=Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(4,degree) #front left of the top leg
        else:
            print ("Error, 0 to 180 degree only") #A protection to not damage the servos.
    def FL2(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(3, degree)  #front left of the middle leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def FL3(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(2, degree)  # front left of the bottom leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def RL1(self, degree):
        servo = Servo()
        degree = int(degree)
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(7, degree)  #rear left of the top leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def RL2(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(6, degree)  #rear left of the middle leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def RL3(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(5, degree)  #rear left of the top leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def FR1(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(11, degree)  #front right of the top leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def FR2(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(12, degree)  #front right of the middle leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def FR3(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(13, degree)  #rear left of the bottom leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def RR1(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(10, degree)  #rear right of the top leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def RR2(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(9, degree)  #rear right of the top leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def RR3(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(8, degree)  #rear right of the top leg
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.
    def head(self, degree):
        degree = int(degree)
        servo = Servo()
        if degree >= 0 or degree <= 180:
            servo.setServoAngle(15, degree)  # Head
        else:
            print("Error, 0 to 180 degree only")  # A protection to not damage the servos.


# Main program logic follows:
if __name__ == '__main__':
    S=Servo()
    while True: ##This is example to test the servo.
        try:
            i = input()
            S.head(i) #You can change the function as well
        except KeyboardInterrupt:
            print ("\nEnd of program")
            break
