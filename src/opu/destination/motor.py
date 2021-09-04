import sys
import traceback
from inf import runtime_data

if runtime_data.hardware == 'raspberry_pi':
    sys.path.insert(1, '../third_party/freenove/smart_car/')
    import controller

# hw_brand = runtime_data.genome['species']['brand']
# hw_model = runtime_data.genome['species']['model']

# hw_brand = 'freenove'
# hw_model = 'smart_car'

# motor = controller.Motor()


def motor_operator(motor_brand, motor_model, motor_id, speed, power):
    import time
    print("Operating Motor %s as a %s %s motor with %s speed" % (motor_id, motor_brand, motor_model, speed))
    try:
        # todo: Generalize the following section. using specifics for test only
        # motor = controller.Motor()

        # motor.M3F()
        # time.sleep(3)
        # motor.stop()
        print(f">>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")

        if motor_id == '0':
            # motor.left_Upper_Wheel(duty=speed)
            # motor.stop()
            print(f">>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")
        elif motor_id == '1':
            # motor.left_Lower_Wheel(duty=speed)
            # motor.stop()
            print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")
        elif motor_id == '2':
            # motor.right_Upper_Wheel(duty=speed)
            # motor.stop()
            print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")
        elif motor_id == '3':
            # motor.right_Lower_Wheel(duty=speed)
            # motor.stop()
            print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>s {motor_id} ACTIVATED AT SPEED {speed}")

    except Exception as e:
        print("ERROR: Requested controller not available for %s %s" % (hw_model, hw_brand))
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
