#!/usr/bin/python3

import motors
import time

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
