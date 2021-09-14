import sys
import time
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
        motor.move(motor_index=motor_id, speed=speed)
        print(f">>>>>>>>>>>>>>>>>>>>>>>> {motor_id} ACTIVATED AT SPEED {speed}")

    except Exception as e:
        # print("ERROR: Requested controller not available for %s %s" % (hw_model, hw_brand))
        exc_info = sys.exc_info()
        traceback.print_exception(*exc_info)
