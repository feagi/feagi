import os
import sys
from inf import runtime_data

if runtime_data.hardware == 'raspberry_pi':
    sys.path.insert(1, '../third_party/freenove/smart_car/')
    import controller

    # from opu.destination.output_matrix import motor_list

    #
    # hw_brand = runtime_data.genome['species']['brand']
    # hw_model = runtime_data.genome['species']['model']

    hw_brand = 'freenove'
    hw_model = 'smart_car'

    motor = controller.Motor()


def motor_operator(motor_brand, motor_model, motor_id, speed, power):
    try:
        import time
        print("Operating a motor on %s %s" % (hw_model, hw_brand))
        try:
            # todo: Generalize the following section. using specifics for test only

            motor.M3F()
            time.sleep(3)
            motor.stop()
        except:
            print("ERROR: Requested controller not available for %s %s" % (hw_model, hw_brand))
    except Exception as e:
        print(e)
