

from opu.destination.output_matrix import motor_list
from inf import runtime_data

if runtime_data.hardware == 'raspberry_pi':

    print("*** ****  Running on a Raspberry PI  **** ***")
    import time
    from third_party.freenove.smart_car import controller as freenove_controller


    def motor_operator(motor_brand, motor_model, motor_id, speed, power):
        if motor_brand == "Freenove":
            print("operating a motor on Freenove device")
            motor = freenove_controller.Motor()
            motor.M3F()
            time.sleep(3)
            motor.stop()
