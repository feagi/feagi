import sys
import traceback
# from inf.initialize import init_hw_controller
from inf import runtime_data
from importlib.machinery import SourceFileLoader


def motor_operator(motor_id, speed, power):
    controller = SourceFileLoader("controller.py", runtime_data.hw_controller_path).load_module()

    import time
    # print("Operating a motor on %s %s" % (hw_model, hw_brand))
    try:
        # todo: Generalize the following section. using specifics for test only
        motor = controller.Motor()

        motor.M3F()
        time.sleep(3)
        motor.stop()
        print(f">>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")

        if motor_id == 'M1':
            # motor.left_Upper_Wheel(duty=speed)
            # motor.stop()
            print(f">>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")
        elif motor_id == 'M2':
            # motor.left_Lower_Wheel(duty=speed)
            # motor.stop()
            print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")
        elif motor_id == 'M3':
            # motor.right_Upper_Wheel(duty=speed)
            # motor.stop()
            print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")
        elif motor_id == 'M4':
            # motor.right_Lower_Wheel(duty=speed)
            # motor.stop()
            print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>s {motor_id} ACTIVATED AT SPEED {speed}")

    except Exception as e:
        # print("ERROR: Requested controller not available for %s %s" % (hw_model, hw_brand))
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
