#The class is designed by Freenove using PCA9685: https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/blob/master/Code/Server/servo.py
# The code is modified to meet the requirement on FEAGI using Freenove 4WD robot

from PCA9685 import PCA9685
class Servo:
    def __init__(self):
        self.PwmServo = PCA9685(0x40, debug=True)
        self.PwmServo.setPWMFreq(50)
        self.PwmServo.setServoPulse(8,1500)
        self.PwmServo.setServoPulse(9,1500)
    def setServoPwm(self,channel,angle,error=10):
        angle=int(angle)
        if channel=='0':
            self.PwmServo.setServoPulse(8,2500-int((angle+error)/0.09))
        elif channel=='1':
            self.PwmServo.setServoPulse(9,500+int((angle+error)/0.09))
        elif channel=='2':
            self.PwmServo.setServoPulse(10,500+int((angle+error)/0.09))
        elif channel=='3':
            self.PwmServo.setServoPulse(11,500+int((angle+error)/0.09))
        elif channel=='4':
            self.PwmServo.setServoPulse(12,500+int((angle+error)/0.09))
        elif channel=='5':
            self.PwmServo.setServoPulse(13,500+int((angle+error)/0.09))
        elif channel=='6':
            self.PwmServo.setServoPulse(14,500+int((angle+error)/0.09))
        elif channel=='7':
            self.PwmServo.setServoPulse(15,500+int((angle+error)/0.09))

# Main program logic follows:
if __name__ == '__main__':
    print("Now servos will rotate to 90° by default.")
    print("If they have already been at 90°, nothing will be observed.")
    print("Please keep the program running when installing the servos.")
    print("Exit the program by press Ctrl + C")
    pwm=Servo()
    while True:
        angle = input()
        pwm.setServoPwm('1', angle=angle) #Change the channel to '0' from '1' if you want to tilt up and down. '1' is for right and left




#       pwm.setServoPwm('0',90)
#       pwm.setServoPwm('1',90)
